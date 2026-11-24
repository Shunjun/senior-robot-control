"""Continuous native graph commits, stable occupancy and coalesced durable cuts.

The ROS adapter owns the sole SLAM process; this owner shares the manager's
catalog/database and never runs ROS spin or graph work on a motion callback.
"""

import asyncio
import base64
import hashlib
import json
import math
import os
import shutil
import uuid
from collections.abc import Callable
from pathlib import Path
from typing import Any, cast

from senior_mapping.active import ActiveMap
from senior_mapping.archive import MapArchive
from senior_mapping.catalog import CatalogEntry, MapCatalog
from senior_mapping.checkpoint_worker import CheckpointWorker
from senior_mapping.checkpoints import MapCheckpoints
from senior_mapping.evidence import MappingObservation, OccupancyThresholds, RangeRay, SceneHit
from senior_mapping.occupancy import OccupancyMaintainer
from senior_mapping.references import (
    AnchoredWaypoint,
    AnchorPose,
    AnchorSnapshot,
    GridExtent,
    SlamSnapshot,
    WaypointBinding,
)
from senior_mapping.workspace import WorkspaceGrid, render_stable_grid
from vehicle_core.errors import DomainError, ErrorCode
from vehicle_core.values import MapRef, Pose2D


async def transferred(reply: dict[str, Any]) -> dict[str, Any]:
    def read() -> dict[str, Any]:
        path = Path(reply["path"])
        # Only a private, owned regular file is accepted from the already verified IPC peer.
        metadata = path.lstat()
        if (
            path.is_symlink()
            or not path.is_file()
            or metadata.st_uid != os.getuid()
            or metadata.st_mode & 0o077
            or metadata.st_size > 8 * 1024 * 1024
        ):
            raise DomainError(ErrorCode.VALIDATION, "mapping transfer is not private and bounded")
        payload = path.read_bytes()
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
        captured = await asyncio.to_thread(snapshot)
        if (
            receipt["boot_id"] != self.boot
            or receipt["sequence"] != self.sequence
            or not _same_anchors(receipt, captured)
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
        self._navigation_at = float("-inf")
        self.occupancy = OccupancyMaintainer(
            OccupancyThresholds(0.05, 3, 1.0, 3, 1.0, 0.1, 1.0, 1.0, 2.0, -1.0, -5.0, 5.0, 2.0, 0.5)
        )

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
                initial = await transferred(reply["loaded"])
                if not _same_anchors(initial, loaded.snapshot):
                    raise DomainError(
                        ErrorCode.CONFLICT, "loaded native graph differs from checkpoint"
                    )
                # A load/seed is not localization. Matching is completed by qualified input below.
                self._restore_snapshot = loaded.snapshot
                self._pending_grid = render_stable_grid(
                    loaded.snapshot.map_ref,
                    uuid.uuid4().hex,
                    loaded.snapshot.grid,
                    loaded.snapshot.cell_size_m,
                    loaded.snapshot.occupancy,
                )
                self._poses = {
                    a.anchor_id: a.pose for a in loaded.snapshot.anchors if a.pose is not None
                }
                self.occupancy.restore(loaded.snapshot.occupancy, bindings=loaded.snapshot.bindings)
                await self.backend.home.restore(loaded.snapshot, loaded.home_id)
        else:
            reply = await self.graph.request("start", **fields)
            self.graph.boot = reply["boot_id"]
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

    def grid(self) -> WorkspaceGrid | None:
        if self.active is None:
            return self._pending_grid
        if self._navigation_generation == self.active.map_generation:
            return self._navigation_grid
        return self.active.grid()

    async def ingest(self, observation: Any) -> None:
        async with self._lock:
            reply = await self.graph.request(
                "next", boot_id=self.graph.boot, sequence=self.graph.sequence + 1
            )
            if reply.get("pending"):
                return
            cut = await transferred(reply)
            await self._apply_cut(cut)
            await self._refresh_navigation()
            if self._task is not None:
                assert self.active is not None
                self._worker.changed(self.active.change_sequence)

    @property
    def map_generation(self) -> str | None:
        return self.active.map_generation if self.active is not None and self.localized else None

    async def _refresh_navigation(self) -> None:
        if self.active is None or not self.localized:
            return
        generation = self.active.map_generation
        if self._navigation_generation != generation:
            if self._activated:
                await self.graph.request("reference", boot_id=self.graph.boot, active=False)
            self._navigation_grid = None
            self._navigation_generation = None
        now = asyncio.get_running_loop().time()
        interval = 1.0 if self.backend.idle else 5.0
        if self._navigation_grid is None or now - self._navigation_at >= interval:
            reply = await self.graph.request(
                "grid", boot_id=self.graph.boot, sequence=self.graph.sequence
            )
            self._navigation_at = now
            if not reply.get("pending"):
                cut = await transferred(reply)
                if cut["boot_id"] != self.graph.boot or cut["sequence"] != self.graph.sequence:
                    raise DomainError(ErrorCode.CONFLICT, "native raster identity changed")
                cells = base64.b64decode(cut["cells_b64"], validate=True)
                self._navigation_grid = WorkspaceGrid(
                    self.entry.map_ref,
                    cut["grid_id"],
                    cut["width"],
                    cut["height"],
                    cut["resolution"],
                    Pose2D(*cut["origin"]),
                    cells,
                    waypoint_writes=True,
                )
                self._navigation_generation = generation
                self._navigation_sequence = self.graph.sequence
        await self._publish_reference()

    async def _publish_reference(self) -> None:
        if not self._activated or self.active is None:
            return
        if (
            self._navigation_generation != self.active.map_generation
            or self._navigation_grid is None
        ):
            await self.graph.request("reference", boot_id=self.graph.boot, active=False)
            return
        await self.graph.request(
            "reference",
            boot_id=self.graph.boot,
            active=True,
            sequence=self.graph.sequence,
            grid_sequence=self._navigation_sequence,
            map_id=self.entry.map_ref.map_id,
            reference_revision=self.entry.map_ref.reference_revision,
            generation=self.active.map_generation,
        )

    async def edit_home(self, pose: Pose2D, expected_revision: int) -> None:
        async with self._lock, self.backend.home.transaction():
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
        # Cancellation must not release the session lock while its writer still runs.
        work = asyncio.create_task(asyncio.to_thread(self._apply, cut))
        try:
            await asyncio.shield(work)
        except asyncio.CancelledError:
            await work
            raise

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
        if self.active is None:
            if self._restore_snapshot is None:
                self.active = ActiveMap(self.entry.map_ref, "odom", 0.05)
                self.active.bind_verified_source(
                    self.source, backend_boot=self.graph.boot, source_epoch=self.graph.boot
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
                    source_epoch=self.graph.boot,
                )
        self.occupancy.reproject(self._poses, frames)
        self._poses = frames
        rays, hits = [], []
        for i, distance in enumerate(cut["ranges"]):
            angle = cut["angle_min"] + i * cut["angle_increment"]
            valid = distance is not None and cut["range_min"] <= distance <= cut["range_max"]
            rays.append(RangeRay(angle, distance if valid else 0.0, valid, False, None))
            if valid:
                heading = sensor.yaw + angle
                hits.append(
                    SceneHit(
                        sensor.x + distance * math.cos(heading),
                        sensor.y + distance * math.sin(heading),
                        "unknown_static",
                    )
                )
        observation = MappingObservation(
            self.entry.map_ref,
            anchor_id,
            sensor,
            cut["stamp"],
            tuple(rays),
            tuple(hits),
            (False,) * len(rays),
            (False,) * len(rays),
            1.0,
            True,
            False,
            0.0,
        )
        delta = self.occupancy.update(observation)
        if len(self.occupancy.snapshot().cells) > 100000:
            raise DomainError(ErrorCode.UNAVAILABLE, "stable map cell budget exhausted")
        points = [(p.x, p.y) for p in frames.values()] + [(h.x, h.y) for h in hits]
        points.extend(
            ((item.cell[0] + 0.5) * 0.05, (item.cell[1] + 0.5) * 0.05)
            for item in self.occupancy.snapshot().cells
        )
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
        changes = self.occupancy.bindings(delta, frames)
        self.active.commit(self.source, cut["sequence"], anchor_id, anchors, changes)
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
        self.graph.sequence = cut["sequence"]
        self.localized = True

    def _matches(self, cut: dict[str, Any], sensor: Pose2D) -> bool:
        # Confirmation uses saved stable geometry, not the new scan inserted into SLAM itself.
        assert self._restore_snapshot is not None
        cells = {item.cell for item in self._restore_snapshot.occupancy.cells}
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
        async with self._lock, self.backend.home.transaction():
            assert self.active is not None
            succeeded = False
            barrier = await self.graph.request("gate", boot_id=self.graph.boot, accepting=False)
            while self.graph.sequence < barrier["sequence"]:
                cut = await transferred(
                    await self.graph.request(
                        "next", boot_id=self.graph.boot, sequence=self.graph.sequence + 1
                    )
                )
                await self._apply_cut(cut)
            try:
                await self._refresh_navigation()
                await self.backend.home(
                    self.entry.map_ref,
                    self.home_id,
                    self.active,
                    self.source,
                    self._poses,
                    before=True,
                )
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
                await self.backend.home(
                    self.entry.map_ref,
                    self.home_id,
                    self.active,
                    self.source,
                    self._poses,
                    before=False,
                )
                self._durable_sequence = self.active.change_sequence
                succeeded = True
            finally:
                if not succeeded:
                    self._paused = True
                if succeeded and not self._paused:
                    await self.graph.request("gate", boot_id=self.graph.boot, accepting=True)

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
