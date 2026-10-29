"""Transactional map import, export, and clear."""

import asyncio
import json
import shutil
import tempfile
import uuid
from collections.abc import AsyncIterator
from contextlib import asynccontextmanager
from dataclasses import dataclass
from pathlib import Path
from typing import Protocol

import aiosqlite

from senior_control.errors import DomainError, ErrorCode
from senior_control.mapping.archive import BundleManifest, MapArchive
from senior_control.mapping.references import SlamSnapshot, durable_move_tree
from senior_control.mapping.store import MapReferenceStore
from senior_control.models import MapRef, OperationResult, OperationStatus
from senior_control.storage.database import Database

OPERATION_JOURNAL_STATEMENTS: tuple[str, ...] = (
    """
    CREATE TABLE map_operation (
        operation_id TEXT PRIMARY KEY,
        action TEXT NOT NULL,
        payload TEXT NOT NULL,
        status TEXT NOT NULL,
        rollback_confirmed INTEGER NOT NULL,
        relocalized INTEGER NOT NULL
    )
    """,
    """
    CREATE TABLE map_operation_phase (
        operation_id TEXT NOT NULL,
        seq INTEGER NOT NULL,
        phase TEXT NOT NULL,
        PRIMARY KEY (operation_id, seq),
        FOREIGN KEY (operation_id) REFERENCES map_operation (operation_id)
    )
    """,
    """
    CREATE TABLE map_task_link (
        task_id TEXT PRIMARY KEY,
        definition TEXT NOT NULL,
        map_id TEXT NOT NULL,
        reference_revision INTEGER NOT NULL,
        invalidated INTEGER NOT NULL
    )
    """,
    """
    CREATE TABLE map_home_link (
        robot_id TEXT PRIMARY KEY,
        source_robot_id TEXT NOT NULL,
        map_id TEXT NOT NULL,
        reference_revision INTEGER NOT NULL,
        waypoint_id TEXT NOT NULL,
        status TEXT NOT NULL,
        adopted INTEGER NOT NULL
    )
    """,
)


class ProcessCrash(BaseException):
    """Injected process death. It is not an operation result."""


class MotionGate(Protocol):
    async def stop_and_block_navigation(self) -> None: ...


class PoseGraphLoader(Protocol):
    async def load(self, directory: Path, hint: object) -> None: ...


class Checkpoint(Protocol):
    async def checkpoint(self) -> None: ...


@dataclass(frozen=True)
class HomeLink:
    robot_id: str
    source_robot_id: str
    map_ref: MapRef
    waypoint_id: str
    status: str
    adopted: bool


@dataclass(frozen=True)
class TaskLink:
    task_id: str
    definition: str
    map_ref: MapRef
    invalidated: bool


@dataclass(frozen=True)
class OperationRecord:
    result: OperationResult
    rollback_confirmed: bool
    relocalized: bool


@dataclass(frozen=True)
class _StoredOperation:
    payload: str
    status: OperationStatus
    rollback_confirmed: bool
    relocalized: bool


class MapOperations:
    def __init__(
        self,
        database: Database,
        store: MapReferenceStore,
        archive: MapArchive,
        motion: MotionGate,
        *,
        robot_id: str,
        map_root: Path,
        config_dir: Path,
        loader: PoseGraphLoader | None = None,
        checkpoint: Checkpoint | None = None,
        fault_after: str | None = None,
    ) -> None:
        self._database = database
        self.store = store
        self._archive = archive
        self._motion = motion
        self.robot_id = robot_id
        self.map_root = map_root
        self.config_dir = config_dir
        self._loader = loader
        self._checkpoint = checkpoint
        self._fault_after = fault_after
        self._map_lock = asyncio.Lock()

    async def prepare(self) -> None:
        from senior_control.storage.migrations import migrate
        from senior_control.storage.schema import DOMAIN_MIGRATIONS

        await migrate(self._database, list(DOMAIN_MIGRATIONS))
        async with self._database.write_lock:
            async with self._transaction() as connection:
                await connection.execute(
                    """
                    UPDATE map_operation
                    SET status = ?
                    WHERE status IN (?, ?, ?)
                    """,
                    (
                        OperationStatus.NEEDS_REVIEW.value,
                        OperationStatus.ACCEPTED.value,
                        OperationStatus.DISPATCHING.value,
                        OperationStatus.RUNNING.value,
                    ),
                )

    async def export_bundle(
        self, map_ref: MapRef, destination: Path, *, operation_id: str | None = None
    ) -> BundleManifest:
        operation_id = _operation_id(operation_id)
        payload = _payload(
            "export",
            {
                "map_id": map_ref.map_id,
                "reference_revision": map_ref.reference_revision,
                "destination": str(destination),
            },
        )
        if not await self._accept(operation_id, "export", payload):
            return await self._archived_manifest(destination)
        await self._phase(operation_id, "dispatching", OperationStatus.DISPATCHING)
        baseline = self.store.map_ref
        try:
            async with self._map_lock:
                baseline = self.store.map_ref
                if self.store.map_ref != map_ref:
                    await self._finish(
                        operation_id,
                        OperationStatus.FAILED,
                        rollback_confirmed=False,
                        relocalized=False,
                    )
                    raise DomainError(ErrorCode.CONFLICT, "active map reference does not match")
                await self._phase(operation_id, "running", OperationStatus.RUNNING)
                manifest = await self._archive.export_bundle(self.store, map_ref, destination)
            await self._finish(
                operation_id,
                OperationStatus.SUCCEEDED,
                rollback_confirmed=False,
                relocalized=False,
            )
            return manifest
        except asyncio.CancelledError:
            await self._needs_review(operation_id)
            raise
        except Exception:
            await self._close_open(operation_id, baseline)
            raise

    async def import_bundle(
        self, path: Path, robot_id: str, *, operation_id: str | None = None
    ) -> OperationResult:
        operation_id = _operation_id(operation_id)
        with tempfile.TemporaryDirectory() as raw:
            staging = Path(raw)
            manifest = await self._archive.validate_and_extract(path, staging)
            try:
                snapshot = SlamSnapshot.read(staging / "occupancy")
            except ValueError as exc:
                raise DomainError(ErrorCode.VALIDATION, "stable layer is unreadable") from exc
            if snapshot.map_ref != manifest.map_ref:
                raise DomainError(
                    ErrorCode.VALIDATION, "stable layer does not match the bundle map"
                )
            payload = _payload(
                "import",
                {
                    "robot_id": robot_id,
                    "map_id": manifest.map_ref.map_id,
                    "reference_revision": manifest.map_ref.reference_revision,
                },
            )
            if not await self._accept(operation_id, "import", payload):
                return await self.query(operation_id)
            self._crash_if("accepted")
            baseline = self.store.map_ref
            try:
                async with self._map_lock:
                    baseline = self.store.map_ref
                    await self._phase(operation_id, "dispatching", OperationStatus.DISPATCHING)
                    self._crash_if("dispatching")
                    await self._motion.stop_and_block_navigation()
                    await self._pause_or_review(operation_id)
                    previous = self.store.snapshot(Path("pose_graph"), ())
                    history = _history_dir(self.map_root, previous.map_ref)
                    await self._backup(previous, history)
                    await self._phase(operation_id, "history_durable", None)
                    await self._phase(operation_id, "running", OperationStatus.RUNNING)
                    try:
                        await self._archive.read_pose_graph(staging / "pose_graph")
                        if self._loader is not None:
                            await self._loader.load(staging / "pose_graph", manifest.last_pose_hint)
                    except Exception:
                        await self._restore_saved(previous, history)
                        await self._finish(
                            operation_id,
                            OperationStatus.FAILED,
                            rollback_confirmed=True,
                            relocalized=False,
                        )
                        raise
                    self.store.replace(snapshot)
                    await self._publish_active(snapshot, staging / "pose_graph")
                    await self._phase(operation_id, "active_durable", None)
                    await self._finish_import(operation_id, robot_id, manifest)
                return OperationResult(operation_id, OperationStatus.SUCCEEDED)
            except asyncio.CancelledError:
                await self._needs_review(operation_id)
                raise
            except Exception:
                await self._close_open(operation_id, baseline)
                raise

    async def clear_map(
        self, expected: MapRef, *, operation_id: str | None = None
    ) -> OperationResult:
        operation_id = _operation_id(operation_id)
        payload = _payload(
            "clear",
            {"map_id": expected.map_id, "reference_revision": expected.reference_revision},
        )
        if not await self._accept(operation_id, "clear", payload):
            return await self.query(operation_id)
        self._crash_if("accepted")
        baseline = self.store.map_ref
        try:
            async with self._map_lock:
                baseline = self.store.map_ref
                if self.store.map_ref != expected:
                    await self._finish(
                        operation_id,
                        OperationStatus.FAILED,
                        rollback_confirmed=False,
                        relocalized=False,
                    )
                    raise DomainError(ErrorCode.CONFLICT, "active map reference does not match")
                await self._phase(operation_id, "dispatching", OperationStatus.DISPATCHING)
                self._crash_if("dispatching")
                await self._motion.stop_and_block_navigation()
                await self._pause_or_review(operation_id)
                previous = self.store.snapshot(Path("pose_graph"), ())
                history = _history_dir(self.map_root, previous.map_ref)
                await self._backup(previous, history)
                await self._phase(operation_id, "history_durable", None)
                await self._phase(operation_id, "running", OperationStatus.RUNNING)
                await self._archive.reset_pose_graph()
                owner = previous.odom_owner
                cell_size = previous.cell_size_m
                cleared = MapRef(f"unmapped-{uuid.uuid4().hex}", 0)
                self.store.clear(cleared, odom_owner=owner, cell_size_m=cell_size)
                await self._publish_active(self.store.snapshot(Path("pose_graph"), ()), None)
                await self._phase(operation_id, "active_durable", None)
                await self._invalidate(expected)
                await self._unadopt_home(expected)
                await self._finish(
                    operation_id,
                    OperationStatus.SUCCEEDED,
                    rollback_confirmed=False,
                    relocalized=False,
                )
            return OperationResult(operation_id, OperationStatus.SUCCEEDED)
        except asyncio.CancelledError:
            await self._needs_review(operation_id)
            raise
        except Exception:
            await self._close_open(operation_id, baseline)
            raise

    async def query(self, operation_id: str) -> OperationResult:
        stored = await self._stored(operation_id)
        if stored is None:
            raise DomainError(ErrorCode.OPERATION_UNKNOWN, "operation was never issued")
        return OperationResult(operation_id, stored.status)

    async def operation_record(self, operation_id: str) -> OperationRecord:
        stored = await self._stored(operation_id)
        if stored is None:
            raise DomainError(ErrorCode.OPERATION_UNKNOWN, "operation was never issued")
        return OperationRecord(
            OperationResult(operation_id, stored.status),
            stored.rollback_confirmed,
            stored.relocalized,
        )

    async def phases(self, operation_id: str) -> tuple[str, ...]:
        async with self._database.write_lock:
            connection = self._database.connection
            async with connection.execute(
                """
                SELECT phase
                FROM map_operation_phase
                WHERE operation_id = ?
                ORDER BY seq
                """,
                (operation_id,),
            ) as cursor:
                rows = await cursor.fetchall()
            await connection.rollback()
        found: list[str] = []
        for row in rows:
            found.append(_text(row[0]))
        return tuple(found)

    async def home_for(self, robot_id: str) -> HomeLink | None:
        async with self._database.write_lock:
            connection = self._database.connection
            async with connection.execute(
                """
                SELECT robot_id, source_robot_id, map_id, reference_revision,
                       waypoint_id, status, adopted
                FROM map_home_link
                WHERE robot_id = ?
                """,
                (robot_id,),
            ) as cursor:
                row = await cursor.fetchone()
            await connection.rollback()
        if row is None:
            return None
        return HomeLink(
            _text(row[0]),
            _text(row[1]),
            MapRef(_text(row[2]), _whole(row[3])),
            _text(row[4]),
            _text(row[5]),
            _flag(row[6]),
        )

    async def bind_task(self, task_id: str, definition: str, map_ref: MapRef) -> None:
        async with self._database.write_lock:
            async with self._transaction() as connection:
                await connection.execute(
                    """
                    INSERT INTO map_task_link (
                        task_id, definition, map_id, reference_revision, invalidated
                    )
                    VALUES (?, ?, ?, ?, 0)
                    ON CONFLICT(task_id) DO UPDATE SET
                        definition = excluded.definition,
                        map_id = excluded.map_id,
                        reference_revision = excluded.reference_revision,
                        invalidated = 0
                    """,
                    (task_id, definition, map_ref.map_id, map_ref.reference_revision),
                )

    async def task_links(self) -> tuple[TaskLink, ...]:
        async with self._database.write_lock:
            connection = self._database.connection
            async with connection.execute(
                """
                SELECT task_id, definition, map_id, reference_revision, invalidated
                FROM map_task_link
                ORDER BY rowid
                """
            ) as cursor:
                rows = await cursor.fetchall()
            await connection.rollback()
        return tuple(
            TaskLink(
                _text(row[0]),
                _text(row[1]),
                MapRef(_text(row[2]), _whole(row[3])),
                _flag(row[4]),
            )
            for row in rows
        )

    async def _accept(self, operation_id: str, action: str, payload: str) -> bool:
        async with self._database.write_lock:
            existing = await self._locked_stored(operation_id)
            if existing is not None:
                if existing.payload != payload:
                    raise DomainError(ErrorCode.CONFLICT, "operation payload does not match")
                return False
            async with self._transaction() as connection:
                await connection.execute(
                    """
                    INSERT INTO map_operation (
                        operation_id, action, payload, status, rollback_confirmed, relocalized
                    )
                    VALUES (?, ?, ?, ?, 0, 0)
                    """,
                    (operation_id, action, payload, OperationStatus.ACCEPTED.value),
                )
                await connection.execute(
                    """
                    INSERT INTO map_operation_phase (operation_id, seq, phase)
                    VALUES (?, 1, 'accepted')
                    """,
                    (operation_id,),
                )
        return True

    async def _phase(self, operation_id: str, phase: str, status: OperationStatus | None) -> None:
        async with self._database.write_lock:
            connection = self._database.connection
            async with connection.execute(
                """
                SELECT COALESCE(MAX(seq), 0)
                FROM map_operation_phase
                WHERE operation_id = ?
                """,
                (operation_id,),
            ) as cursor:
                row = await cursor.fetchone()
            await connection.rollback()
            seq = 1 if row is None else _whole(row[0]) + 1
            async with self._transaction() as writing:
                if status is not None:
                    await writing.execute(
                        "UPDATE map_operation SET status = ? WHERE operation_id = ?",
                        (status.value, operation_id),
                    )
                await writing.execute(
                    """
                    INSERT INTO map_operation_phase (operation_id, seq, phase)
                    VALUES (?, ?, ?)
                    """,
                    (operation_id, seq, phase),
                )

    async def _finish(
        self,
        operation_id: str,
        status: OperationStatus,
        *,
        rollback_confirmed: bool,
        relocalized: bool,
    ) -> None:
        async with self._database.write_lock:
            async with self._transaction() as connection:
                await connection.execute(
                    """
                    UPDATE map_operation
                    SET status = ?, rollback_confirmed = ?, relocalized = ?
                    WHERE operation_id = ?
                    """,
                    (
                        status.value,
                        int(rollback_confirmed),
                        int(relocalized),
                        operation_id,
                    ),
                )

    async def _finish_import(
        self, operation_id: str, robot_id: str, manifest: BundleManifest
    ) -> None:
        from senior_control.waypoints.store import (
            adopted_home_blocks_foreign,
            replace_imported_home,
        )

        home = next((item for item in manifest.waypoints if item.kind == "home"), None)
        # Another robot's home stays unconfirmed. The last pose is only a load hint.
        pending = manifest.source_robot_id != robot_id
        anchor_pose = home.pose if home is not None else None
        if home is not None:
            for binding in self.store.snapshot(Path("pose_graph"), ()).waypoint_bindings:
                if binding.waypoint.waypoint_id == home.waypoint_id:
                    anchor_pose = binding.anchor_pose
                    break
        async with self._database.write_lock:
            async with self._transaction() as connection:
                if home is not None and not (
                    pending and await adopted_home_blocks_foreign(connection, robot_id)
                ):
                    assert anchor_pose is not None
                    await replace_imported_home(
                        connection,
                        robot_id=robot_id,
                        waypoint_id=home.waypoint_id,
                        pose=home.pose,
                        map_ref=home.map_ref,
                        anchor_id=home.anchor_id,
                        anchor_pose=anchor_pose,
                        source_robot_id=manifest.source_robot_id,
                        status="pending_confirmation" if pending else "confirmed",
                    )
                    await connection.execute(
                        """
                        INSERT INTO map_home_link (
                            robot_id, source_robot_id, map_id, reference_revision,
                            waypoint_id, status, adopted
                        )
                        VALUES (?, ?, ?, ?, ?, ?, ?)
                        ON CONFLICT(robot_id) DO UPDATE SET
                            source_robot_id = excluded.source_robot_id,
                            map_id = excluded.map_id,
                            reference_revision = excluded.reference_revision,
                            waypoint_id = excluded.waypoint_id,
                            status = excluded.status,
                            adopted = excluded.adopted
                        """,
                        (
                            robot_id,
                            manifest.source_robot_id,
                            home.map_ref.map_id,
                            home.map_ref.reference_revision,
                            home.waypoint_id,
                            "pending_confirmation" if pending else "confirmed",
                            0 if pending else 1,
                        ),
                    )
                await connection.execute(
                    """
                    UPDATE map_operation
                    SET status = ?, rollback_confirmed = 0, relocalized = 0
                    WHERE operation_id = ?
                    """,
                    (OperationStatus.SUCCEEDED.value, operation_id),
                )

    async def _invalidate(self, map_ref: MapRef) -> None:
        async with self._database.write_lock:
            async with self._transaction() as connection:
                await connection.execute(
                    """
                    UPDATE map_task_link
                    SET invalidated = 1
                    WHERE map_id = ? AND reference_revision = ?
                    """,
                    (map_ref.map_id, map_ref.reference_revision),
                )

    async def _unadopt_home(self, map_ref: MapRef) -> None:
        async with self._database.write_lock:
            async with self._transaction() as connection:
                await connection.execute(
                    """
                    UPDATE map_home_link
                    SET adopted = 0, status = 'unadopted'
                    WHERE map_id = ? AND reference_revision = ?
                    """,
                    (map_ref.map_id, map_ref.reference_revision),
                )

    async def _stored(self, operation_id: str) -> _StoredOperation | None:
        async with self._database.write_lock:
            return await self._locked_stored(operation_id)

    async def _locked_stored(self, operation_id: str) -> _StoredOperation | None:
        connection = self._database.connection
        async with connection.execute(
            """
            SELECT payload, status, rollback_confirmed, relocalized
            FROM map_operation
            WHERE operation_id = ?
            """,
            (operation_id,),
        ) as cursor:
            row = await cursor.fetchone()
        await connection.rollback()
        if row is None:
            return None
        return _StoredOperation(
            _text(row[0]),
            OperationStatus(_text(row[1])),
            _flag(row[2]),
            _flag(row[3]),
        )

    @asynccontextmanager
    async def _transaction(self) -> AsyncIterator[aiosqlite.Connection]:
        connection = self._database.connection
        await connection.rollback()
        async with connection.execute("BEGIN IMMEDIATE"):
            pass
        try:
            yield connection
        except BaseException:
            await connection.rollback()
            raise
        else:
            await connection.commit()

    async def _pause_or_review(self, operation_id: str) -> None:
        try:
            if self._checkpoint is not None:
                await self._checkpoint.checkpoint()
        except asyncio.CancelledError:
            await self._needs_review(operation_id)
            raise

    async def _needs_review(self, operation_id: str) -> None:
        task = asyncio.current_task()
        if task is not None:
            uncancel = getattr(task, "uncancel", None)
            if callable(uncancel):
                uncancel()
        stored = await self._stored(operation_id)
        if stored is None or _closed(stored.status):
            return
        await self._phase(operation_id, "needs_review", OperationStatus.NEEDS_REVIEW)
        await self._finish(
            operation_id,
            OperationStatus.NEEDS_REVIEW,
            rollback_confirmed=False,
            relocalized=False,
        )

    async def _close_open(self, operation_id: str, baseline: MapRef) -> None:
        stored = await self._stored(operation_id)
        if stored is None or _closed(stored.status):
            return
        if self.store.map_ref != baseline:
            await self._needs_review(operation_id)
            return
        await self._finish(
            operation_id,
            OperationStatus.FAILED,
            rollback_confirmed=False,
            relocalized=False,
        )

    async def _backup(self, snapshot: SlamSnapshot, destination: Path) -> None:
        stage = self._stage_dir()
        await self._archive.write_pose_graph(stage / "pose_graph")
        snapshot.write(stage)
        durable_move_tree(stage, destination)

    async def _publish_active(self, snapshot: SlamSnapshot, pose_graph: Path | None) -> None:
        stage = self._stage_dir()
        if pose_graph is None:
            await self._archive.write_pose_graph(stage / "pose_graph")
        else:
            _copy_tree(pose_graph, stage / "pose_graph")
        snapshot.write(stage)
        durable_move_tree(stage, self.map_root / "active")

    async def _restore_saved(self, snapshot: SlamSnapshot, history: Path) -> None:
        await self._archive.read_pose_graph(history / "pose_graph")
        stage = self._stage_dir()
        _copy_tree(history / "pose_graph", stage / "pose_graph")
        snapshot.write(stage)
        durable_move_tree(stage, self.map_root / "active")

    def _stage_dir(self) -> Path:
        stage = self.map_root / ".staging" / uuid.uuid4().hex
        stage.mkdir(parents=True, exist_ok=True)
        return stage

    async def _archived_manifest(self, destination: Path) -> BundleManifest:
        with tempfile.TemporaryDirectory() as raw:
            return await self._archive.validate_and_extract(destination, Path(raw))

    def _crash_if(self, phase: str) -> None:
        if self._fault_after == phase:
            raise ProcessCrash(phase)


def _closed(status: OperationStatus) -> bool:
    return status in {
        OperationStatus.SUCCEEDED,
        OperationStatus.FAILED,
        OperationStatus.NEEDS_REVIEW,
    }


def _history_dir(root: Path, map_ref: MapRef) -> Path:
    return root / "history" / map_ref.map_id / str(map_ref.reference_revision)


def _copy_tree(source: Path, destination: Path) -> None:
    destination.mkdir(parents=True, exist_ok=True)
    for path in source.iterdir():
        if path.is_file():
            shutil.copyfile(path, destination / path.name)


def _operation_id(operation_id: str | None) -> str:
    if operation_id is None or not operation_id.strip():
        return uuid.uuid4().hex
    return operation_id


def _payload(action: str, fields: dict[str, object]) -> str:
    body = {"action": action, **fields}
    return json.dumps(body, sort_keys=True, separators=(",", ":"))


def _text(value: object) -> str:
    if not isinstance(value, str):
        raise DomainError(ErrorCode.VALIDATION, "expected text")
    return value


def _whole(value: object) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise DomainError(ErrorCode.VALIDATION, "expected integer")
    return value


def _flag(value: object) -> bool:
    if isinstance(value, bool):
        return value
    if value == 0:
        return False
    if value == 1:
        return True
    raise DomainError(ErrorCode.VALIDATION, "expected flag")
