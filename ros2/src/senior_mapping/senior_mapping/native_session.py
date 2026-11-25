"""Continuous native graph commits, stable occupancy and coalesced durable cuts.

The ROS adapter owns the sole SLAM process; this owner shares the manager's
catalog/database and never runs ROS spin or graph work on a motion callback.
"""

import asyncio
import hashlib
import json
import math
import os
import shutil
import stat
import tempfile
import time
import uuid
from collections.abc import Callable
from dataclasses import replace
from pathlib import Path
from typing import Any, cast

from senior_perception.vision.raw_capture import RawCaptureHeader

from senior_mapping.active import ActiveMap
from senior_mapping.archive import MapArchive
from senior_mapping.catalog import CatalogEntry, MapCatalog
from senior_mapping.checkpoint_worker import CheckpointWorker
from senior_mapping.checkpoints import MapCheckpoints
from senior_mapping.evidence import OccupancyThresholds, SourceIdentity, VerifiedBasis
from senior_mapping.fixed_evidence import FixedEvidenceConsumer
from senior_mapping.references import (
    AnchoredWaypoint,
    AnchorPose,
    AnchorSnapshot,
    GridExtent,
    SlamSnapshot,
    WaypointBinding,
    _watermark_payload,
    encode_fixed_snapshot,
    migrate_legacy,
    snapshot_from_fixed,
)
from senior_mapping.shared_scan import ScanKey, read_latest_scan
from senior_mapping.workspace import WorkspaceGrid, render_stable_grid
from vehicle_core.errors import DomainError, ErrorCode
from vehicle_core.sample_time import SampleTime
from vehicle_core.values import MapRef, Pose2D


async def transferred(reply: dict[str, Any]) -> dict[str, Any]:
    def read() -> dict[str, Any]:
        path = Path(reply["path"])
        # Only a private, owned regular file is accepted from the already verified IPC peer.
        fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
        with os.fdopen(fd, "rb") as stream:
            metadata = os.fstat(stream.fileno())
            if (
                not stat.S_ISREG(metadata.st_mode)
                or metadata.st_uid != os.getuid()
                or metadata.st_mode & 0o077
                or metadata.st_size > 8 * 1024 * 1024
            ):
                raise DomainError(
                    ErrorCode.VALIDATION, "mapping transfer is not private and bounded"
                )
            payload = stream.read(8 * 1024 * 1024 + 1)
        if len(payload) > 8 * 1024 * 1024:
            raise DomainError(ErrorCode.VALIDATION, "mapping transfer grew beyond budget")
        if hashlib.sha256(payload).hexdigest() != reply["sha256"]:
            raise DomainError(ErrorCode.CONFLICT, "mapping transfer changed")
        result = json.loads(payload)
        if not isinstance(result, dict):
            raise DomainError(ErrorCode.VALIDATION, "mapping transfer is not an object")
        return result

    return await asyncio.to_thread(read)


class NativeGraph:
    def __init__(self, client: Any, map_ref: MapRef) -> None:
        self.client, self._ref = client, map_ref
        self.boot = ""
        self.sequence = 0
        self._io: asyncio.Task[None] | None = None

    async def request(self, kind: str, **fields: Any) -> dict[str, Any]:
        result = await self.client.request("map.backend." + kind, **fields)
        if result.get("ok") is not True:
            try:
                code = ErrorCode(result.get("error", "unavailable"))
            except ValueError:
                code = ErrorCode.UNAVAILABLE
            raise DomainError(code, "mapping backend rejected request")
        return cast(dict[str, Any], result)

    def map_ref(self) -> MapRef:
        return self._ref

    def painted_cells(self) -> tuple[tuple[int, int], ...]:
        return ()

    def last_pose_hint(self) -> Pose2D | None:
        return None

    async def capture_consistent(
        self, directory: Path, snapshot: Callable[[], SlamSnapshot]
    ) -> SlamSnapshot:
        receipt = await transferred(await self.request("capture", boot_id=self.boot))
        captured = snapshot()
        if (
            receipt["boot_id"] != self.boot
            or receipt["sequence"] != self.sequence
            or not _same_anchors(receipt, captured)
            or (
                captured.fixed is not None
                and (
                    captured.fixed.watermark.backend_boot != receipt["boot_id"]
                    or captured.fixed.watermark.graph_sequence != receipt["sequence"]
                )
            )
        ):
            raise DomainError(
                ErrorCode.CONFLICT, "native graph and stable layer are different cuts"
            )

        def materialize() -> None:
            directory.mkdir(parents=True, exist_ok=True)
            prefix = Path(receipt["prefix"])
            files = []
            for suffix in (".posegraph", ".data"):
                source = Path(str(prefix) + suffix)
                if (
                    source.is_symlink()
                    or not source.is_file()
                    or source.stat().st_size > 64 * 1024 * 1024
                ):
                    raise DomainError(ErrorCode.VALIDATION, "native graph file is not bounded")
                destination = directory / ("graph" + suffix)
                shutil.copyfile(source, destination)
                files.append(hashlib.sha256(destination.read_bytes()).digest())
            if hashlib.sha256(b"".join(files)).hexdigest() != receipt["pair_digest"]:
                raise DomainError(ErrorCode.CONFLICT, "native graph digest changed")
            (directory / "native-proof.json").write_text(json.dumps(receipt, allow_nan=False))
            # A native capture slot is consumed only after copying and verifying both files.
            for suffix in (".posegraph", ".data"):
                Path(str(prefix) + suffix).unlink()

        self._io = asyncio.create_task(asyncio.to_thread(materialize))
        await asyncio.shield(self._io)
        return captured

    async def verify_consistent(self, directory: Path, snapshot: SlamSnapshot) -> bool:
        def verify() -> bool:
            proof = json.loads((directory / "native-proof.json").read_text())
            digests = [
                hashlib.sha256((directory / ("graph" + suffix)).read_bytes()).digest()
                for suffix in (".posegraph", ".data")
            ]
            return (
                _same_anchors(proof, snapshot)
                and (
                    snapshot.fixed is None
                    or (
                        snapshot.fixed.watermark.backend_boot == proof["boot_id"]
                        and snapshot.fixed.watermark.graph_sequence == proof["sequence"]
                    )
                )
                and hashlib.sha256(b"".join(digests)).hexdigest() == proof["pair_digest"]
            )

        return await asyncio.to_thread(verify)

    async def quiesce(self) -> None:
        if self._io is not None:
            await asyncio.shield(self._io)

    async def write(self, directory: Path) -> tuple[str, ...]:
        raise DomainError(ErrorCode.UNAVAILABLE, "graph writes require a complete checkpoint")

    async def read(self, directory: Path) -> None:
        raise DomainError(ErrorCode.UNAVAILABLE, "graph loads require a serial map switch")

    async def reset(self) -> None:
        await self.request("stop")


def _same_anchors(receipt: dict[str, Any], snapshot: SlamSnapshot) -> bool:
    actual = {str(row["id"]): Pose2D(row["x"], row["y"], row["yaw"]) for row in receipt["anchors"]}
    return actual == {anchor.anchor_id: anchor.pose for anchor in snapshot.anchors}


class NativeMappingBackend:
    def __init__(
        self,
        client: Any,
        catalog: MapCatalog,
        root: Path,
        *,
        robot_id: str,
        home: Any,
        home_id: str,
        max_delay_s: float = 10,
    ) -> None:
        self.client, self.catalog, self.root = client, catalog, root
        self.robot_id, self.home = robot_id, home
        self.home_id, self.max_delay_s = home_id, max_delay_s
        self.idle = False
        # Physical admission remains unresolved until an evidenced profile is supplied.
        self.geometry_profile = None
        self.clock_ns = time.monotonic_ns

    async def stopped(self) -> bool:
        result = await self.client.request("map.backend.status")
        self.idle = result.get("ok") is True and result.get("stopped") is True
        return self.idle

    async def delete(self, entry: CatalogEntry) -> None:
        def remove() -> None:
            root = self.root / "checkpoints"
            directory = root / hashlib.sha256(entry.map_ref.map_id.encode()).hexdigest()
            for path in (directory, *directory.parents):
                if path.is_symlink():
                    raise DomainError(ErrorCode.VALIDATION, "map cleanup path contains a symlink")
            if directory.exists():
                shutil.rmtree(directory)

        # The lifecycle has already retired the target and joined its reader/writer.
        await asyncio.to_thread(remove)

    async def open(
        self, entry: CatalogEntry, *, restore: bool, seed: Pose2D | None = None
    ) -> "NativeSession":
        session = NativeSession(self, entry)
        try:
            await session.open(restore=restore, seed=seed)
            return session
        except BaseException:
            await session.abort()
            raise


class NativeSession:
    def __init__(self, backend: NativeMappingBackend, entry: CatalogEntry) -> None:
        self.backend, self.entry = backend, entry
        self.localized = False
        self.graph = NativeGraph(backend.client, entry.map_ref)
        self.repository = MapCheckpoints(
            backend.root / "checkpoints",
            MapArchive(
                self.graph,
                source_robot_id=backend.robot_id,
                capture_timeout_ms=10000,
                verification_timeout_ms=10000,
            ),
            backend.catalog,
        )
        self.source = object()
        self.active: ActiveMap | None = None
        self.home_id = entry.home_id or backend.home_id
        self._lock = asyncio.Lock()
        self._paused = False
        self._pending_grid: WorkspaceGrid | None = None
        self._restore_snapshot: SlamSnapshot | None = None
        self._capture_root: Path | None = None
        self._task: asyncio.Task[None] | None = None
        self._worker = CheckpointWorker(
            self._save, idle=lambda: backend.idle, max_delay_s=backend.max_delay_s
        )
        self._durable_sequence = 0
        self._poses: dict[str, Pose2D] = {}
        self._activated = False
        self._navigation_grid: WorkspaceGrid | None = None
        self._navigation_generation: str | None = None
        self._navigation_sequence = 0
        thresholds = OccupancyThresholds(
            0.05, 3, 1.0, 3, 1.0, 0.1, 1.0, 1.0, 2.0, -1.0, -5.0, 5.0, 2.0, 0.5
        )
        self._evidence = FixedEvidenceConsumer(
            thresholds, profile=backend.geometry_profile, clock=backend.clock_ns
        )
        self._fixed_retired = False
        self._scan_revision = 0
        self._last_scan_capture = None
        self._scan_path = None
        self._scan_nonce = self._host_boot_id = ""
        self._graph_cut = None
        self._fixed_bound = False
        self._installed_watermark = None
        self._capturing = False
        self._notified_sequence = 0

    async def open(self, *, restore: bool, seed: Pose2D | None = None) -> None:
        fields: dict[str, Any] = {
            "capture_root": str(self.backend.root / "live" / uuid.uuid4().hex)
        }
        self._capture_root = Path(fields["capture_root"])
        if restore:
            async with self.repository.open(self.entry) as loaded:
                proof = await asyncio.to_thread(
                    lambda: json.loads((loaded.graph_directory / "native-proof.json").read_text())
                )
                waypoint = next(
                    item.waypoint
                    for item in loaded.snapshot.waypoint_bindings
                    if item.waypoint.waypoint_id == loaded.home_id
                )
                fields.update(
                    prefix=str(loaded.graph_directory / "graph"),
                    pair_digest=proof["pair_digest"],
                    seed=[waypoint.pose.x, waypoint.pose.y, waypoint.pose.yaw]
                    if seed is None
                    else [seed.x, seed.y, seed.yaw],
                )
                reply = await self.graph.request("start", **fields)
                self.graph.boot = reply["boot_id"]
                self._configure_scan_channel(reply)
                initial = await transferred(reply["loaded"])
                if not _same_anchors(initial, loaded.snapshot):
                    raise DomainError(
                        ErrorCode.CONFLICT, "loaded native graph differs from checkpoint"
                    )
                # A load/seed is not localization. Matching is completed by qualified input below.
                self._restore_snapshot = loaded.snapshot
                self._legacy_archive_digest = loaded.original_digest
                self._pending_grid = (
                    loaded.snapshot.fixed.grid
                    if loaded.snapshot.fixed is not None
                    else render_stable_grid(
                        loaded.snapshot.map_ref,
                        uuid.uuid4().hex,
                        loaded.snapshot.grid,
                        loaded.snapshot.cell_size_m,
                        loaded.snapshot.occupancy,
                    )
                )
                self._poses = {
                    a.anchor_id: a.pose for a in loaded.snapshot.anchors if a.pose is not None
                }
                await self.backend.home.restore(loaded.snapshot, loaded.home_id)
        else:
            reply = await self.graph.request("start", **fields)
            self.graph.boot = reply["boot_id"]
            self._configure_scan_channel(reply)
            self._restore_snapshot = None
        await self.graph.request("gate", boot_id=self.graph.boot, accepting=True)
        # A start acknowledgement is insufficient; require a bounded first commit.
        deadline = asyncio.get_running_loop().time() + 15
        while self.graph.sequence == 0:
            try:
                await self.ingest(None)
            except DomainError as exc:
                if restore and exc.code is ErrorCode.LOCALIZATION_UNAVAILABLE:
                    await self.graph.request("gate", boot_id=self.graph.boot, accepting=False)
                    self._paused = True
                    return
                raise
            if asyncio.get_running_loop().time() >= deadline:
                raise DomainError(
                    ErrorCode.LOCALIZATION_UNAVAILABLE, "SLAM produced no initial observation"
                )
            await asyncio.sleep(0.05)
        if not restore:
            await self._save(0)
        else:
            async with self.backend.home.transaction():
                await self.backend.home(
                    self.entry.map_ref,
                    self.home_id,
                    self.active,
                    self.source,
                    self._poses,
                    before=False,
                )

    async def activate(self) -> None:
        if self.active is None:
            return
        self._activated = True
        await self._publish_reference()
        self._task = asyncio.create_task(self._worker.run())
        if self.active.change_sequence > self._durable_sequence:
            self._worker.changed(self.active.change_sequence)
            self._notified_sequence = self.active.change_sequence

    def grid(self) -> WorkspaceGrid | None:
        if self.active is None:
            return self._pending_grid
        if self._navigation_generation == self.active.map_generation:
            return self._navigation_grid
        return self.active.grid()

    def _configure_scan_channel(self, reply: dict[str, Any]) -> None:
        if reply.get("qualified_scan_schema") != 1:
            raise DomainError(
                ErrorCode.UNAVAILABLE, "qualified scan owner requires a coordinated rebuild"
            )
        self._scan_path = Path(reply["qualified_scan_path"])
        self._scan_nonce, self._host_boot_id = reply["qualified_scan_nonce"], reply["host_boot_id"]

    def _retire_fixed(self) -> None:
        self._fixed_retired = True
        self.localized = False
        self._navigation_grid = None
        self._navigation_generation = None
        self._evidence.reset()
        if self.active is not None and self._fixed_bound:
            try:
                self.active.retire_fixed(self.source)
            except DomainError as exc:
                if exc.code is not ErrorCode.CONFLICT:
                    raise

    async def ingest(self, observation: Any) -> None:
        async with self._lock:
            if self._capturing or self._fixed_retired:
                return
            try:
                reply = await self.graph.request(
                    "next", boot_id=self.graph.boot, sequence=self.graph.sequence + 1
                )
                if not reply.get("pending"):
                    await self._apply_cut(await transferred(reply))
                await self._consume_latest_scan()
                await self._refresh_navigation()
                if (
                    self._task is not None
                    and self.active is not None
                    and self.active.change_sequence > self._notified_sequence
                ):
                    self._worker.changed(self.active.change_sequence)
                    self._notified_sequence = self.active.change_sequence
            except (ValueError, KeyError, TypeError, OverflowError) as exc:
                self._retire_fixed()
                raise DomainError(ErrorCode.VALIDATION, "mapping batch is invalid") from exc
            except BaseException:
                self._retire_fixed()
                raise

    async def _consume_latest_scan(self) -> None:
        if self.active is None or not self._fixed_bound or self._scan_path is None:
            return
        result = await asyncio.to_thread(
            read_latest_scan,
            self._scan_path,
            self._scan_revision,
            nonce=self._scan_nonce,
            backend_boot=self.graph.boot,
            host_boot_id=self._host_boot_id,
        )
        if not result.changed:
            return
        row = result.envelope
        self._scan_revision = row.revision
        if row.status != "valid":
            if row.reason in ("starting", "reference_changed") and row.status != "retired":
                self._evidence.reset()
                return
            self._retire_fixed()
            return
        cut = self.active.fixed_snapshot(self.source)
        basis = row.basis
        if basis is None:
            return
        if basis.token != (
            self.entry.map_ref.map_id,
            self.entry.map_ref.reference_revision,
            self.active.map_generation,
        ):
            if self._installed_watermark is not None and basis.token == (
                self._installed_watermark.map_ref.map_id,
                self._installed_watermark.map_ref.reference_revision,
                self._installed_watermark.reference_generation,
            ):
                self._evidence.reset()  # observed old R cannot authorize the new graph
                return
            self._retire_fixed()
            return
        verified = VerifiedBasis(
            basis.anchor_id,
            basis.map_basis,
            basis.odom_basis,
            basis.graph_sequence,
            basis.anchor_digest,
            self.entry.map_ref,
            basis.reference_generation,
        )
        if (basis.graph_sequence, basis.anchor_digest) != (
            cut.watermark.graph_sequence,
            cut.watermark.anchor_digest,
        ):
            self._evidence.reset()
            return
        graph = self._graph_cut
        anchor = next((a for a in graph["anchors"] if str(a["id"]) == basis.anchor_id), None)
        if (
            anchor is None
            or basis.anchor_id != str(graph["keyframe_id"])
            or basis.map_basis != Pose2D(anchor["x"], anchor["y"], anchor["yaw"])
            or basis.odom_basis != Pose2D(*graph["odom"])
            or row.acceptance_floor_steady_ns < self._reference_floor_ns
        ):
            raise DomainError(
                ErrorCode.CONFLICT, "scan TF/basis/floor differs from verified graph cut"
            )
        if row.acceptance_floor is not None and (
            row.key.source != row.acceptance_floor.source
            or row.key.sequence <= row.acceptance_floor.sequence
        ):
            self._retire_fixed()
            return
        previous = self._last_scan_capture
        current = row.scan.capture.time
        if previous is not None and (
            current.stamp_ns <= previous.stamp_ns
            or current.sample_steady_ns <= previous.sample_steady_ns
            or current.received_steady_ns <= previous.received_steady_ns
        ):
            self._evidence.reset()
            return
        self.active.accept_scan(self.source, row.scan, verified)
        self._last_scan_capture = current
        try:
            self._evidence.consume(self.active, self.source, row.scan, verified)
        except DomainError as exc:
            if exc.code is ErrorCode.VALIDATION and "fixed_capacity" in str(exc):
                self._evidence.reason = "coverage_limit"
                await self.graph.request(
                    "reference",
                    boot_id=self.graph.boot,
                    active=True,
                    sequence=self.graph.sequence,
                    map_id=self.entry.map_ref.map_id,
                    reference_revision=self.entry.map_ref.reference_revision,
                    generation=self.active.map_generation,
                    acceptance_floor_steady_ns=self._reference_floor_ns,
                    coverage_reason="coverage_limit",
                )
                return
            raise

    @property
    def map_generation(self) -> str | None:
        return self.active.map_generation if self.active is not None and self.localized else None

    async def _refresh_navigation(self) -> None:
        if (
            self.active is None
            or not self.localized
            or not self._fixed_bound
            or self._fixed_retired
            or self._paused
        ):
            return
        cut = self.active.fixed_snapshot(self.source)
        if cut.watermark == self._installed_watermark:
            return
        if self._navigation_generation != cut.watermark.reference_generation:
            await self.graph.request(
                "reference",
                boot_id=self.graph.boot,
                active=True,
                sequence=self.graph.sequence,
                map_id=self.entry.map_ref.map_id,
                reference_revision=self.entry.map_ref.reference_revision,
                generation=cut.watermark.reference_generation,
                acceptance_floor_steady_ns=self._reference_floor_ns,
            )
        path = (
            self._capture_root / "fixed.json"
            if self._capture_root
            else self._scan_path.parent / "fixed.json"
        )
        payload = encode_fixed_snapshot(cut, transfer=True)

        def write():
            path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
            fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_TRUNC | os.O_NOFOLLOW, 0o600)
            with os.fdopen(fd, "wb") as stream:
                os.fchmod(stream.fileno(), 0o600)
                stream.write(payload)

        await asyncio.to_thread(write)
        try:
            ack = await self.graph.request(
                "install_fixed",
                boot_id=self.graph.boot,
                path=str(path),
                sha256=hashlib.sha256(payload).hexdigest(),
                acceptance_floor_steady_ns=self._reference_floor_ns,
                map_id=self.entry.map_ref.map_id,
                reference_revision=self.entry.map_ref.reference_revision,
                generation=self.active.map_generation,
                coverage_reason="coverage_limit"
                if self._evidence.reason == "coverage_limit"
                else None,
            )
            if (
                ack.get("watermark") != _watermark_payload(cut.watermark)
                or ack.get("grid_id") != cut.grid.grid_id
                or self._fixed_retired
            ):
                raise DomainError(ErrorCode.CONFLICT, "bridge fixed ACK changed cut")
            self._installed_watermark = cut.watermark
            self._navigation_grid = replace(cut.grid, display_only=False, waypoint_writes=True)
            self._navigation_generation = cut.watermark.reference_generation
            self._navigation_sequence = cut.watermark.graph_sequence
        finally:
            path.unlink(missing_ok=True)

    async def _publish_reference(self) -> None:
        if self._fixed_retired:
            await self.graph.request("reference", boot_id=self.graph.boot, active=False)
            return
        await self._refresh_navigation()

    async def edit_home(self, pose: Pose2D, expected_revision: int) -> None:
        async with self.backend.home.transaction(), self._lock:
            if self.active is None or not self.localized:
                raise DomainError(ErrorCode.LOCALIZATION_UNAVAILABLE, "Home needs a localized map")
            await self.backend.home.edit(
                self.entry.map_ref,
                self.home_id,
                self.active,
                self.source,
                self._poses,
                pose,
                expected_revision,
            )
        await self._save(0)

    async def _apply_cut(self, cut: dict[str, Any]) -> None:
        if (
            self.active is None
            and self._restore_snapshot is not None
            and self._restore_snapshot.fixed is None
        ):
            # Exactly one legacy cut may consume the diagnostic native raster.
            prior = self._restore_snapshot
            reply = await self.graph.request(
                "grid", boot_id=self.graph.boot, sequence=cut["sequence"], purpose="migrate_legacy"
            )
            native = None
            if not reply.get("pending"):
                import base64

                raster = await transferred(reply)
                if raster["boot_id"] != self.graph.boot or raster["sequence"] != cut["sequence"]:
                    raise DomainError(ErrorCode.CONFLICT, "legacy raster belongs to another cut")
                native = WorkspaceGrid(
                    prior.map_ref,
                    raster["grid_id"],
                    raster["width"],
                    raster["height"],
                    raster["resolution"],
                    Pose2D(*raster["origin"]),
                    base64.b64decode(raster["cells_b64"], validate=True),
                )
            fixed = migrate_legacy(
                prior,
                legacy_grid=self._pending_grid,
                native_legacy_grid=native,
                original_archive_digest=self._legacy_archive_digest,
            )
            with tempfile.TemporaryDirectory() as raw:
                prior.write(Path(raw))
                audit = prior.legacy_json or (Path(raw) / "occupancy.json").read_bytes()
            self._restore_snapshot = snapshot_from_fixed(
                fixed,
                odom_owner=prior.odom_owner,
                review_required=prior.review_required,
                legacy_json=audit,
            )
        self._apply(cut)

    def _apply(self, cut: dict[str, Any]) -> None:
        if (
            cut["boot_id"] != self.graph.boot
            or cut["sequence"] != self.graph.sequence + 1
            or cut["odom_frame"] != "odom"
            or cut["map_frame"] != "map"
        ):
            raise DomainError(ErrorCode.CONFLICT, "SLAM observation identity changed")
        frames = {str(row["id"]): Pose2D(row["x"], row["y"], row["yaw"]) for row in cut["anchors"]}
        anchor_id = str(cut["keyframe_id"])
        anchor_pose = frames[anchor_id]
        sensor = Pose2D(*cut["sensor"])
        raw_capture = cut["capture"]
        capture = RawCaptureHeader(**{**raw_capture, "time": SampleTime(**raw_capture["time"])})
        identity = SourceIdentity.from_key(ScanKey.of(self.graph.boot, capture))
        if (
            self._fixed_bound
            and identity != self.active.fixed_snapshot(self.source).watermark.source
        ):
            self._retire_fixed()
            raise DomainError(ErrorCode.CONFLICT, "graph source changed within fixed session")
        if self.active is None:
            if self._restore_snapshot is None:
                self.active = ActiveMap(self.entry.map_ref, "odom", 0.05)
                self.active.bind_verified_source(
                    self.source, backend_boot=self.graph.boot, source_epoch=identity.source_epoch
                )
            else:
                # Check saved geometry before creating a restored writer.
                if not self._matches(cut, sensor):
                    raise DomainError(
                        ErrorCode.LOCALIZATION_UNAVAILABLE,
                        "saved map requires position calibration",
                    )
                self.active = ActiveMap.resume_verified(
                    self._restore_snapshot,
                    self.source,
                    backend_boot=self.graph.boot,
                    source_epoch=identity.source_epoch,
                )
        self._poses = frames
        # Raw graph hits are localization facts, never fixed classification.
        points = [(p.x, p.y) for p in frames.values()]
        x0 = math.floor(min(p[0] for p in points) / 0.05) - 1
        y0 = math.floor(min(p[1] for p in points) / 0.05) - 1
        width = math.ceil(max(p[0] for p in points) / 0.05) - x0 + 2
        height = math.ceil(max(p[1] for p in points) / 0.05) - y0 + 2
        prior = self.active.grid()
        if prior is not None:
            end_x, end_y = x0 + width, y0 + height
            x0 = min(x0, round(prior.origin.x / 0.05))
            y0 = min(y0, round(prior.origin.y / 0.05))
            width = max(end_x, round(prior.origin.x / 0.05) + prior.width) - x0
            height = max(end_y, round(prior.origin.y / 0.05) + prior.height) - y0
        anchors = AnchorSnapshot(
            self.entry.map_ref,
            "odom",
            tuple(AnchorPose(k, v, True) for k, v in frames.items()),
            GridExtent(width, height, x0 * 0.05, y0 * 0.05),
        )
        self.active.commit(self.source, cut["sequence"], anchor_id, anchors, ())
        if self.graph.sequence == 0 and self._restore_snapshot is None:
            self.active.update_waypoints(
                self.source,
                expected_sequence=1,
                changes=(
                    WaypointBinding(
                        AnchoredWaypoint(self.home_id, anchor_pose, anchor_id, False), anchor_pose
                    ),
                ),
            )
        if not self._fixed_bound:
            self._reference_floor_ns = self.backend.clock_ns()
            self.active.bind_fixed_context(
                self.source,
                source_identity=identity,
                host_boot_id=self._host_boot_id,
                acceptance_floor=None,
                acceptance_floor_steady_ns=self._reference_floor_ns,
                accepting=lambda: not self._fixed_retired,
                clock=self.backend.clock_ns,
                profiles=self._evidence.profiles(),
            )
            self._fixed_bound = True
        elif self._graph_cut is not None and self.active.map_generation != self._graph_cut.get(
            "generation"
        ):
            self._reference_floor_ns = self.backend.clock_ns()
            self._evidence.reset()
        self._graph_cut = {**cut, "generation": self.active.map_generation}
        self.graph.sequence = cut["sequence"]
        self.localized = True

    def _matches(self, cut: dict[str, Any], sensor: Pose2D) -> bool:
        # Confirmation uses saved stable geometry, not the new scan inserted into SLAM itself.
        assert self._restore_snapshot is not None
        saved = self._restore_snapshot
        if saved.fixed is not None:
            grid = saved.fixed.grid
            cells = {
                (
                    math.floor((grid.origin.x + (i % grid.width) * grid.resolution) / 0.05),
                    math.floor((grid.origin.y + (i // grid.width) * grid.resolution) / 0.05),
                )
                for i, value in enumerate(grid.cells)
                if value == 100
            }
            # Empty authoritative occupancy cannot provide a geometric veto;
            # the verified native resume/matcher remains localization authority.
            if not cells:
                return True
        else:
            cells = {item.cell for item in saved.occupancy.cells}
        total = matched = 0
        for i, distance in enumerate(cut["ranges"]):
            if distance is None or not cut["range_min"] <= distance <= cut["range_max"]:
                continue
            heading = sensor.yaw + cut["angle_min"] + i * cut["angle_increment"]
            x = math.floor((sensor.x + distance * math.cos(heading)) / 0.05)
            y = math.floor((sensor.y + distance * math.sin(heading)) / 0.05)
            total += 1
            matched += any((x + dx, y + dy) in cells for dx in (-1, 0, 1) for dy in (-1, 0, 1))
        return total >= 20 and matched / total >= 0.7

    async def _save(self, _: int) -> None:
        async with self.backend.home.transaction():
            async with self._lock:
                assert self.active is not None
                self._capturing = True
                try:
                    barrier = await self.graph.request(
                        "gate", boot_id=self.graph.boot, accepting=False
                    )
                    while self.graph.sequence < barrier["sequence"]:
                        reply = await self.graph.request(
                            "next", boot_id=self.graph.boot, sequence=self.graph.sequence + 1
                        )
                        if reply.get("pending"):
                            raise DomainError(
                                ErrorCode.CONFLICT, "capture drain lost a graph commit"
                            )
                        await self._apply_cut(await transferred(reply))
                    await self._refresh_navigation()
                    await self.backend.home(
                        self.entry.map_ref,
                        self.home_id,
                        self.active,
                        self.source,
                        self._poses,
                        before=True,
                    )
                except BaseException:
                    self._retire_fixed()
                    self._paused = True
                    self._capturing = False
                    raise
            # Immutable graph/fixed cut is frozen by the existing archive barrier.
            # Neither manager intake lock nor Q callback lock is held for disk I/O.
            succeeded = False
            try:
                selection = await self.backend.catalog.selection()
                self.entry = await self.active.save_checkpoint(
                    self.source,
                    self.repository,
                    home_id=self.home_id,
                    expected=self.entry.checkpoint_id,
                    session_token=selection.token
                    if selection.map_ref == self.entry.map_ref
                    else None,
                )
                self._durable_sequence = self.repository.saved_change_sequence
                await self.backend.home(
                    self.entry.map_ref,
                    self.home_id,
                    self.active,
                    self.source,
                    self._poses,
                    before=False,
                )
                succeeded = True
            finally:
                async with self._lock:
                    # C2 and free confidence retain only consecutive original
                    # frames. A skipped mailbox sequence resets their windows on
                    # the first post-capture scan; no replay fills the gap.
                    self._capturing = False
                    if not succeeded:
                        self._paused = True
                        self._retire_fixed()
                    elif not self._paused:
                        try:
                            await self.graph.request(
                                "gate", boot_id=self.graph.boot, accepting=True
                            )
                        except BaseException:
                            self._paused = True
                            self._retire_fixed()
                            raise

    async def export_bundle(self, destination: Path) -> None:
        if self.active is not None and self.localized and not self._fixed_retired:
            await self._save(0)
        # The existing Home/save serialization also protects the checkpoint lease
        # from a concurrent writer; graph/fixed intake and Q remain independent.
        async with self.backend.home.transaction():
            async with self.repository.open(self.entry) as loaded:
                archive = self.repository._archive
                archive._home_waypoint_id = self.home_id
                await archive.export_checkpoint(
                    loaded.snapshot, loaded.graph_directory, destination
                )

    @property
    def error(self) -> Exception | None:
        return self._worker.last_error

    async def flush(self) -> None:
        if self.active is None:
            return
        self._paused = True
        self._activated = False
        await self.graph.request("reference", boot_id=self.graph.boot, active=False)
        await self.graph.request("gate", boot_id=self.graph.boot, accepting=False)
        if self._task is not None:
            await self._worker.flush()
        # A graph cut accepted since the last checkpoint must also be drained and saved.
        await self._save(0)

    async def abort(self) -> None:
        self._retire_fixed()
        await self.graph.request("stop")
        if self._capture_root is not None:
            await asyncio.to_thread(shutil.rmtree, self._capture_root, True)

    async def close(self) -> None:
        try:
            if self._task is not None:
                await self._worker.close()
                await self._task
        finally:
            try:
                await self.graph.quiesce()
                await self.repository.quiesce()
            finally:
                try:
                    if self.active is not None:
                        try:
                            self.active.revoke(self.source)
                        except DomainError as exc:
                            if exc.code is not ErrorCode.CONFLICT:
                                raise
                finally:
                    await self.abort()
