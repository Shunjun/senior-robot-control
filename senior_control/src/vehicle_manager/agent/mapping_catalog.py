"""Management catalog reads and lifecycle RPCs, over the existing local connections.

Metadata is read from its functional owner's tables in the manager database;
physical graph/localization effects stay with the registered mapping runtime.
No directory-only edit is acknowledged as starting a SLAM session.
"""

import asyncio
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from senior_mapping.active import ActiveMap
    from senior_mapping.runtime import MappingRuntime

    from vehicle_core.values import MapRef, Pose2D

from pydantic import ValidationError
from senior_contracts.wire import MapCreateBody, MapPhase, MapRuntimeStatus, MapTargetBody

from vehicle_core.ipc import IpcError
from vehicle_manager.agent.local_adapters import IpcMapPort
from vehicle_manager.agent.request_ports import Requester
from vehicle_manager.storage.home_database import HomeDatabase


class RemoteMapRuntime:
    def __init__(self, reads: Requester, writes: Requester) -> None:
        self._reads, self._writes = reads, IpcMapPort(writes)
        self.phase: MapPhase = "unavailable"

    async def refresh(self) -> None:
        # A missing producer never turns old persisted metadata into live capability.
        self.phase = "unavailable"
        try:
            reply = MapRuntimeStatus.model_validate(await self._reads.request("map.status"))
        except (IpcError, OSError, ValidationError):
            return
        self.phase = reply.phase

    async def execute(
        self, kind: str, operation_id: str, body: MapCreateBody | MapTargetBody
    ) -> None:
        await self._writes.manage(kind, operation_id, body.model_dump())


class MappingRequests:
    def __init__(
        self, endpoint: Requester, runtime: RemoteMapRuntime, reads: Requester, writes: Requester
    ) -> None:
        self._endpoint, self._runtime = endpoint, runtime
        self._reads, self._writes = reads, writes

    async def request(self, kind: str, **fields: Any) -> dict[str, Any]:
        if kind in {
            "map.catalog",
            "map.create",
            "map.activate",
            "map.delete",
            "map.calibrate",
            "map.home",
        }:
            await self._runtime.refresh()
            return await self._endpoint.request(kind, **fields)
        owner = self._writes if kind in {"map.import", "map.export", "map.clear"} else self._reads
        return await owner.request(kind, **fields)


async def mapping_requests(
    database: HomeDatabase[Any],
    reads: Requester,
    writes: Requester,
    *,
    robot_id: str,
    root: Any = None,
    waypoints: Any = None,
    checkpoint_delay_s: float = 10.0,
) -> Requester | None:
    try:
        from senior_mapping.catalog import MapCatalog
        from senior_mapping.catalog_api import CatalogEndpoint
    except ModuleNotFoundError as exc:
        if exc.name not in {
            "senior_mapping",
            "senior_mapping.catalog",
            "senior_mapping.catalog_api",
        }:
            raise
        # //Compatible: installing management without optional mapping keeps original ports.
        return None
    catalog = MapCatalog(database, robot_id=robot_id)
    if root is not None and waypoints is not None:
        import uuid

        from senior_mapping.native_session import NativeMappingBackend
        from senior_mapping.runtime import MappingRuntime

        await catalog.prepare()
        current = await waypoints.home()
        home_id = (
            current.waypoint.id
            if current is not None
            else uuid.uuid5(uuid.NAMESPACE_URL, "senior-home:" + robot_id).hex
        )
        native = NativeMappingBackend(
            writes,
            catalog,
            root,
            robot_id=robot_id,
            home=MappingHome(waypoints, robot_id, lock=database.home_target_lock(robot_id)),
            home_id=home_id,
            max_delay_s=checkpoint_delay_s,
        )
        native_runtime = MappingRuntime(
            catalog, native, restore_required=(await catalog.selection()).generation > 0
        )
        endpoint = CatalogEndpoint(catalog, robot_id=robot_id, runtime=native_runtime)
        return ProductionMappingRequests(endpoint, native_runtime, reads, writes, native.home)
    # //Compatible: read-only injected ports retain the original remote lifecycle adapter.
    await catalog.prepare_reader()
    runtime = RemoteMapRuntime(reads, writes)
    endpoint = CatalogEndpoint(catalog, robot_id=robot_id, runtime=runtime)
    return MappingRequests(endpoint, runtime, reads, writes)


class ProductionMappingRequests:
    """Management persistence and ingestion, with ROS over local IPC."""

    def __init__(
        self,
        endpoint: Requester,
        runtime: "MappingRuntime",
        reads: Requester,
        writes: Requester,
        home: "MappingHome",
    ) -> None:
        self._endpoint, self._runtime = endpoint, runtime
        self._reads, self._writes = reads, writes
        self._home = home
        self._runtime.phase = "unavailable"

    def coordinate_home(self, port: Any) -> Any:
        coordinator = self._home

        class Home:
            async def home_revision(self) -> int:
                return int(await port.home_revision())

            async def set_current(self, expected_revision: int) -> int:
                async with coordinator.transaction():
                    return int(await port.set_current(expected_revision))

        return Home()

    async def run(self) -> None:
        import asyncio
        import logging

        from vehicle_core.errors import DomainError

        # Do not advertise the new lifecycle before the actual ROS adapter is available.
        self._runtime.phase = "unavailable"
        while True:
            try:
                if await self._runtime.backend.stopped():
                    reply = await self._writes.request("map.backend.stop")
                    if reply.get("ok") is True:
                        break
            except (IpcError, OSError, DomainError):
                logging.getLogger(__name__).debug("Mapping adapter not ready", exc_info=True)
            await asyncio.sleep(1)
        self._runtime.phase = "idle"
        try:
            await self._runtime.restore()
        except (IpcError, OSError, DomainError) as exc:
            self._runtime.last_error = exc
            self._runtime.phase = "failed"
        checked_at = 0.0
        while True:
            try:
                now = asyncio.get_running_loop().time()
                if now - checked_at >= 1:
                    await self._runtime.backend.stopped()
                    checked_at = now
                await self._runtime.ingest(None)
            except (IpcError, OSError, DomainError) as exc:
                self._runtime.last_error = exc
                self._runtime.phase = "failed"
            await asyncio.sleep(0.05)

    async def close(self) -> None:
        await self._runtime.close()

    async def request(self, kind: str, **fields: Any) -> dict[str, Any]:
        if kind in {
            "map.catalog",
            "map.status",
            "map.create",
            "map.activate",
            "map.delete",
            "map.calibrate",
            "map.home",
        }:
            return await self._endpoint.request(kind, **fields)
        if kind in {"map.active", "map.manifest", "map.tile"}:
            from senior_mapping.workspace import WorkspaceMap

            if self._runtime.owns_reference:
                return WorkspaceMap(self._runtime.grid).read(kind, fields)
        owner = self._writes if kind in {"map.import", "map.export", "map.clear"} else self._reads
        return await owner.request(kind, **fields)

    def state_requests(self, client: Requester) -> Requester:
        owner = self

        class State:
            async def request(self, kind: str, **fields: Any) -> dict[str, Any]:
                raw = await client.request(kind, **fields)
                if kind == "state.get" and owner._runtime.owns_reference:
                    grid = owner._runtime.grid()
                    ref, generation = (
                        (None if grid is None else grid.map_ref),
                        owner._runtime.map_generation,
                    )
                    if (
                        grid is not None
                        and ref is not None
                        and generation is not None
                        and raw.get("ok") is True
                        and raw.get("map_id") == ref.map_id
                        and raw.get("reference_revision") == ref.reference_revision
                        and raw.get("map_generation") == generation
                    ):
                        raw["map_grid_id"] = grid.grid_id
                        return raw
                    raw.update(
                        map_id=grid.map_ref.map_id if grid else None,
                        reference_revision=grid.map_ref.reference_revision if grid else None,
                        map_grid_id=grid.grid_id if grid else None,
                        map_generation=None,
                        localization_generation=None,
                        localized=False,
                        navigation_ready=False,
                        robot_pose=None,
                        route_progress=None,
                        navigation_block_reason="active mapping has no qualified navigation raster",
                    )
                if kind == "pose.get" and owner._runtime.owns_reference:
                    grid = owner._runtime.grid()
                    if (
                        grid is None
                        or owner._runtime.map_generation is None
                        or raw.get("map_id") != grid.map_ref.map_id
                        or raw.get("reference_revision") != grid.map_ref.reference_revision
                        or raw.get("map_generation") != owner._runtime.map_generation
                    ):
                        return {"ok": False, "error": "unavailable"}
                return raw

        return State()


class MappingHome:
    """Keep one canonical live Home; each checkpoint retains its frozen map binding."""

    def __init__(self, waypoints: Any, robot_id: str, *, lock: asyncio.Lock | None = None) -> None:
        self.points, self.robot_id = waypoints, robot_id
        self._revision: int | None = None
        self._capture_revision: int | None = None
        self._lock = lock if lock is not None else asyncio.Lock()

    def transaction(self) -> asyncio.Lock:
        return self._lock

    async def restore(self, snapshot: Any, home_id: str) -> None:
        async with self.transaction():
            await self._restore(snapshot, home_id)

    async def _restore(self, snapshot: Any, home_id: str) -> None:
        binding = next(
            row for row in snapshot.waypoint_bindings if row.waypoint.waypoint_id == home_id
        )
        point = binding.waypoint
        await self.points.save_home(
            waypoint_id=home_id,
            pose=point.pose,
            map_ref=snapshot.map_ref,
            anchor_id=point.anchor_id,
            anchor_pose=binding.anchor_pose,
            source_robot_id=self.robot_id,
            status="needs_review",
            adopted=True,
            latitude=None,
            longitude=None,
            expected_revision=await self.points.home_revision(),
        )
        self._revision = await self.points.home_revision()
        self._capture_revision = self._revision

    async def edit(
        self,
        map_ref: "MapRef",
        home_id: str,
        active: "ActiveMap",
        source: object,
        poses: dict[str, "Pose2D"],
        pose: "Pose2D",
        expected_revision: int,
    ) -> None:
        from senior_mapping.references import AnchoredWaypoint, WaypointBinding

        from vehicle_core.errors import DomainError, ErrorCode

        if await self.points.home_revision() != expected_revision:
            raise DomainError(ErrorCode.CONFLICT, "Home revision changed")
        current = await self.points.home()
        if current is None or current.waypoint.map_ref != map_ref:
            raise DomainError(ErrorCode.CONFLICT, "Home map changed")
        anchor = min(
            poses, key=lambda key: (poses[key].x - pose.x) ** 2 + (poses[key].y - pose.y) ** 2
        )
        await self.points.save_home(
            waypoint_id=home_id,
            pose=pose,
            map_ref=map_ref,
            anchor_id=anchor,
            anchor_pose=poses[anchor],
            source_robot_id=self.robot_id,
            status="confirmed",
            adopted=True,
            latitude=None,
            longitude=None,
            expected_revision=expected_revision,
        )
        active.update_waypoints(
            source,
            expected_sequence=active.observation_sequence,
            changes=(
                WaypointBinding(AnchoredWaypoint(home_id, pose, anchor, False), poses[anchor]),
            ),
        )
        self._revision = await self.points.home_revision()
        self._capture_revision = self._revision

    async def __call__(
        self,
        map_ref: "MapRef",
        home_id: str,
        active: "ActiveMap",
        source: object,
        poses: dict[str, "Pose2D"],
        *,
        before: bool,
    ) -> None:
        revision = await self.points.home_revision()
        current = await self.points.home()
        if before:
            self._capture_revision = revision
            if self._revision is not None and revision != self._revision and current is not None:
                point = current.waypoint
                if (
                    point.map_ref == map_ref
                    and point.anchor_id in poses
                    and point.status == "confirmed"
                ):
                    from senior_mapping.references import (
                        AnchoredWaypoint,
                        WaypointBinding,
                        rebase_pose,
                    )

                    active.update_waypoints(
                        source,
                        expected_sequence=active.observation_sequence,
                        changes=(
                            WaypointBinding(
                                AnchoredWaypoint(
                                    home_id,
                                    rebase_pose(
                                        point.pose, current.anchor_pose, poses[point.anchor_id]
                                    ),
                                    point.anchor_id,
                                    False,
                                ),
                                poses[point.anchor_id],
                            ),
                        ),
                    )
            return
        binding = next(
            row for row in active.waypoint_bindings(source) if row.waypoint.waypoint_id == home_id
        )
        point = binding.waypoint
        if (
            current is not None
            and current.waypoint.id == home_id
            and current.waypoint.map_ref == map_ref
            and current.waypoint.pose == point.pose
            and current.waypoint.anchor_id == point.anchor_id
            and current.anchor_pose == binding.anchor_pose
            and current.waypoint.status == "confirmed"
        ):
            self._revision = revision
            self._capture_revision = revision
            return
        await self.points.save_home(
            waypoint_id=home_id,
            pose=point.pose,
            map_ref=map_ref,
            anchor_id=point.anchor_id,
            anchor_pose=binding.anchor_pose,
            source_robot_id=self.robot_id,
            status="confirmed",
            adopted=True,
            latitude=None,
            longitude=None,
            expected_revision=self._capture_revision,
        )
        self._revision = await self.points.home_revision()
        self._capture_revision = self._revision
