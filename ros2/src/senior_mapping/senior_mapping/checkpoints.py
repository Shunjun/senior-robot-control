"""Complete immutable map cuts, published through the shared catalog only after fsync.

This owner performs no localization or navigation admission. Its caller must
quiesce an old graph owner before replacing this instance after an ambiguous
capture failure; cancellation of a ROS request is not proof of remote completion.
"""

import asyncio
import hashlib
import json
import os
import shutil
import stat
import uuid
from collections.abc import AsyncIterator
from contextlib import asynccontextmanager
from dataclasses import dataclass
from pathlib import Path
from typing import Any, TypeVar

from senior_mapping.archive import MapArchive, _file_bytes
from senior_mapping.catalog import CatalogEntry, MapCatalog
from senior_mapping.references import SlamSnapshot, durable_move_tree, load_snapshot
from senior_mapping.store import MapReferenceStore
from vehicle_core.errors import DomainError, ErrorCode
from vehicle_core.values import MapRef


@dataclass(frozen=True)
class LoadedCheckpoint:
    snapshot: SlamSnapshot
    graph_directory: Path
    home_id: str
    original_digest: str


class MapCheckpoints:
    def __init__(
        self,
        root: Path,
        archive: MapArchive,
        catalog: MapCatalog,
        *,
        max_snapshot_bytes: int = 67_108_864,
    ) -> None:
        if not root.is_absolute():
            raise ValueError("checkpoint root must be absolute")
        if max_snapshot_bytes <= 0:
            raise ValueError("checkpoint byte limit must be positive")
        self._byte_limit = max_snapshot_bytes
        self.cleanup_error: Exception | None = None
        self.root, self._archive, self._catalog = root, archive, catalog
        self._lock = asyncio.Lock()
        self._io: asyncio.Task[Any] | None = None
        self._closing = False
        self._leases_finished = asyncio.Event()
        self._leases_finished.set()
        self._recovery_required = False
        self._pinned: dict[str, int] = {}
        self.saved_change_sequence = 0

    def _prepare(self) -> None:
        for path in (self.root, *self.root.parents):
            if path.is_symlink():
                raise DomainError(ErrorCode.VALIDATION, "checkpoint root contains a symlink")
        self.root.mkdir(mode=0o700, parents=True, exist_ok=True)
        metadata = self.root.stat()
        if metadata.st_uid != os.getuid() or metadata.st_mode & 0o077:
            raise DomainError(ErrorCode.VALIDATION, "checkpoint root must be private and owned")

    def _directory(self, map_ref: MapRef, identity: str) -> Path:
        if len(identity) != 32 or any(
            character not in "0123456789abcdef" for character in identity
        ):
            raise DomainError(ErrorCode.VALIDATION, "checkpoint identity is invalid")
        name = hashlib.sha256(map_ref.map_id.encode()).hexdigest()
        result = self.root / name / identity
        if result.is_symlink() or result.parent.is_symlink():
            raise DomainError(ErrorCode.VALIDATION, "checkpoint path contains a symlink")
        return result

    async def save(
        self,
        store: MapReferenceStore,
        *,
        home_id: str,
        expected: str | None,
        session_token: str | None = None,
    ) -> CatalogEntry:
        async with self._lock:
            if (
                self._closing
                or self._pinned
                or self.cleanup_error is not None
                or self._recovery_required
                or self._io is not None
                and not self._io.done()
            ):
                raise DomainError(ErrorCode.CONFLICT, "prior checkpoint has not reconciled")
            await asyncio.to_thread(self._prepare)
            identity = uuid.uuid4().hex
            stage = self.root / (".capture-" + identity)
            destination = self._directory(store.map_ref, identity)
            try:
                captured = await self._archive.capture_snapshot(store, stage / "pose_graph")
                if captured.review_required or not any(
                    binding.waypoint.waypoint_id == home_id and not binding.waypoint.review_required
                    for binding in captured.waypoint_bindings
                ):
                    raise DomainError(
                        ErrorCode.CONFLICT, "checkpoint Home is not in the captured cut"
                    )
                self._io = asyncio.create_task(
                    asyncio.to_thread(
                        _publish_files,
                        captured,
                        home_id,
                        identity,
                        stage,
                        destination,
                        self._byte_limit,
                    )
                )
                # If the awaiting owner is cancelled, the single writer stays
                # tracked. It can only publish an orphan immutable directory;
                # the catalog pointer remains unchanged until this await returns.
                self._io.add_done_callback(_consume_io)
                await asyncio.shield(self._io)
                await self._catalog.checkpoint(
                    captured.map_ref,
                    identity,
                    home_id=home_id,
                    expected=expected,
                    session_token=session_token,
                )
            except BaseException:
                # Keep the last complete catalog pointer. No unbounded retries
                # create staging trees while a remote capture outcome is unknown.
                self._recovery_required = True
                raise
            self.saved_change_sequence = (
                captured.fixed.watermark.change_sequence
                if captured.fixed is not None
                else store.change_sequence
            )
            entries = await self._catalog.list()
            entry = next(row for row in entries if row.map_ref == captured.map_ref)
            self._io = asyncio.create_task(
                asyncio.to_thread(
                    self._prune, destination.parent, {identity, expected}, set(self._pinned)
                )
            )
            self._io.add_done_callback(_consume_io)
            try:
                await asyncio.shield(self._io)
            except Exception as exc:
                # Publication already committed. Retention cleanup failure is
                # observable separately; it cannot turn a durable cut into a
                # false save failure or discard the last catalog pointer.
                self.cleanup_error = exc
            else:
                self.cleanup_error = None
            return entry

    @asynccontextmanager
    async def open(self, entry: CatalogEntry) -> AsyncIterator[LoadedCheckpoint]:
        """Pin graph files for the entire load, not only the metadata read."""
        if entry.checkpoint_id is None or entry.home_id is None:
            raise DomainError(ErrorCode.UNAVAILABLE, "map has no complete checkpoint")
        async with self._lock:
            if self._closing or self._pinned:
                raise DomainError(ErrorCode.CONFLICT, "checkpoint loader is occupied or closing")
            if self._io is not None and not self._io.done():
                try:
                    await asyncio.shield(self._io)
                except Exception:
                    # Failed later I/O does not invalidate an earlier complete
                    # cut; its own files and graph are verified below.
                    pass
            await asyncio.to_thread(self._prepare)
            directory = self._directory(entry.map_ref, entry.checkpoint_id)
            read_task = asyncio.create_task(
                asyncio.to_thread(_read_cut, entry, directory, self._byte_limit)
            )
            self._io = read_task
            read_task.add_done_callback(_consume_io)
            loaded = await asyncio.shield(read_task)
            await self._archive.verify_snapshot(directory / "pose_graph", loaded.snapshot)
            identity = entry.checkpoint_id
            self._pinned[identity] = self._pinned.get(identity, 0) + 1
            self._leases_finished.clear()
        try:
            yield loaded
        finally:
            async with self._lock:
                self._pinned[identity] -= 1
                if self._pinned[identity] == 0:
                    del self._pinned[identity]
                if not self._pinned:
                    self._leases_finished.set()

    async def quiesce(self) -> None:
        """Wait for local disk work at a lifecycle boundary, never on control callbacks."""
        async with self._lock:
            self._closing = True
            pending = self._io
        try:
            if pending is not None:
                await asyncio.shield(pending)
        finally:
            await self._leases_finished.wait()

    def _prune(self, directory: Path, keep: set[str | None], pinned: set[str]) -> None:
        for path in directory.iterdir():
            if path.name not in keep and path.name not in pinned:
                if (
                    len(path.name) != 32
                    or any(c not in "0123456789abcdef" for c in path.name)
                    or path.is_symlink()
                    or not path.is_dir()
                ):
                    raise DomainError(
                        ErrorCode.VALIDATION, "checkpoint tree contains an alien entry"
                    )
                shutil.rmtree(path)


def _publish_files(
    snapshot: SlamSnapshot,
    home_id: str,
    identity: str,
    stage: Path,
    destination: Path,
    byte_limit: int,
) -> None:
    snapshot.write(stage)
    if (stage / "occupancy.json").stat().st_size > byte_limit:
        raise DomainError(ErrorCode.VALIDATION, "checkpoint exceeds stable-layer byte limit")
    metadata = dict(version=1, checkpoint_id=identity, home_id=home_id)
    if snapshot.fixed is not None:
        graph = _file_bytes(stage / "pose_graph", byte_limit)
        members = json.dumps(
            {name: hashlib.sha256(data).hexdigest() for name, data in sorted(graph.items())},
            sort_keys=True,
            separators=(",", ":"),
        ).encode()
        (stage / "members.json").write_bytes(members)
        metadata.update(
            version=2,
            occupancy_sha256=hashlib.sha256((stage / "occupancy.json").read_bytes()).hexdigest(),
            members_sha256=hashlib.sha256(members).hexdigest(),
        )
    payload = json.dumps(metadata, sort_keys=True, separators=(",", ":")).encode()
    if len(payload) > 4096:
        raise DomainError(ErrorCode.VALIDATION, "checkpoint metadata exceeds 4096 bytes")
    (stage / "checkpoint.json").write_bytes(payload)
    durable_move_tree(stage, destination)


_Result = TypeVar("_Result")


def _consume_io(task: asyncio.Task[_Result]) -> None:
    if not task.cancelled():
        task.exception()


def _read_file(path: Path, limit: int) -> bytes:
    fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
    with os.fdopen(fd, "rb") as stream:
        metadata = os.fstat(stream.fileno())
        if not stat.S_ISREG(metadata.st_mode) or metadata.st_size > limit:
            raise ValueError("checkpoint member is not a bounded regular file")
        result = stream.read(limit + 1)
    if len(result) > limit:
        raise ValueError("checkpoint member exceeds its byte limit")
    return result


def _read_cut(entry: CatalogEntry, directory: Path, limit: int) -> LoadedCheckpoint:
    try:
        layer = _read_file(directory / "occupancy.json", limit)
        metadata = json.loads(_read_file(directory / "checkpoint.json", 4096))
        snapshot = load_snapshot(layer)
        graph = _file_bytes(directory / "pose_graph", limit)
        original_members = {
            "occupancy.json": hashlib.sha256(layer).hexdigest(),
            "checkpoint.json": hashlib.sha256(
                _read_file(directory / "checkpoint.json", 4096)
            ).hexdigest(),
            **{
                "pose_graph/" + name: hashlib.sha256(data).hexdigest()
                for name, data in graph.items()
            },
        }
        original_digest = hashlib.sha256(
            json.dumps(original_members, sort_keys=True, separators=(",", ":")).encode()
        ).hexdigest()
        if snapshot.fixed is not None:
            members = _read_file(directory / "members.json", 262144)
            graph = _file_bytes(directory / "pose_graph", limit)
            if (
                metadata.get("version") != 2
                or metadata.get("occupancy_sha256") != hashlib.sha256(layer).hexdigest()
                or metadata.get("members_sha256") != hashlib.sha256(members).hexdigest()
                or json.loads(members)
                != {name: hashlib.sha256(data).hexdigest() for name, data in graph.items()}
            ):
                raise ValueError("checkpoint immutable members changed")
            metadata = {
                k: v for k, v in metadata.items() if k not in ("occupancy_sha256", "members_sha256")
            }
        elif metadata.get("version") != 1:
            raise ValueError("checkpoint mixes layer versions")
    except (OSError, ValueError) as exc:
        raise DomainError(ErrorCode.UNAVAILABLE, "complete checkpoint is unreadable") from exc
    if (
        snapshot.map_ref != entry.map_ref
        or metadata
        != {
            "version": 2 if snapshot.fixed is not None else 1,
            "checkpoint_id": entry.checkpoint_id,
            "home_id": entry.home_id,
        }
        or not any(
            binding.waypoint.waypoint_id == entry.home_id and not binding.waypoint.review_required
            for binding in snapshot.waypoint_bindings
        )
    ):
        raise DomainError(ErrorCode.CONFLICT, "checkpoint metadata belongs to another cut")
    assert entry.home_id is not None
    return LoadedCheckpoint(snapshot, directory / "pose_graph", entry.home_id, original_digest)
