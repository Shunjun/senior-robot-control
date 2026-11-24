"""Original management workspace reads and target checks; maps retain raster ownership."""

import math
from collections.abc import Mapping
from typing import Any

from pydantic import ValidationError
from senior_contracts.errors import ErrorCode
from senior_contracts.wire import (
    MapCatalogEnvelope,
    MapCatalogReply,
    MapRefPayload,
    WorkspaceEnvelope,
)

from vehicle_manager.agent.request_ports import Requester, WorkspacePorts
from vehicle_manager.tasks.models import TaskDefinition
from vehicle_manager.tasks.ports import MapReference, PoseReading
from vehicle_manager.tasks.store import TaskStore
from vehicle_manager.waypoints.store import WaypointStore

_PAGE_SIZE = 32


class WorkspaceReader:
    def __init__(
        self,
        robot_id: str,
        tasks: TaskStore,
        waypoints: WaypointStore[Any, Any, Any],
        maps: Requester,
        *,
        ports: WorkspacePorts,
        exported_grid: Any | None = None,
    ) -> None:
        self._ports = ports
        self._robot_id = robot_id
        self._tasks = tasks
        self._waypoints = waypoints
        self._maps = maps
        if exported_grid is not None and not exported_grid.display_only:
            raise ValueError("workspace export must be read-only")
        self._exported_grid = exported_grid

    async def active_map(self) -> MapReference | None:
        try:
            reply = self._ports.map_reply.model_validate(await self._maps.request("map.active"))
            return self._ports.map_reference(reply.map_ref.map_id, reply.map_ref.reference_revision)
        except (self._ports.ipc_error, OSError, ValidationError):
            return self._exported_grid.map_ref if self._exported_grid is not None else None

    async def create_waypoint(self, body: Mapping[str, Any]) -> None:
        try:
            point = self._ports.waypoint_create.model_validate(body)
            if not point.waypoint_id.strip():
                raise ValueError("empty waypoint id")
            pose = self._ports.pose(point.x, point.y, point.yaw)
            ref = MapRefPayload(map_id=point.map_id, reference_revision=point.reference_revision)
        except (ValidationError, ValueError) as exc:
            raise self._ports.error(ErrorCode.VALIDATION, "invalid waypoint") from exc

        manifest = await self._manifest(ref)
        if not manifest.waypoint_writes:
            raise self._ports.error(ErrorCode.UNAVAILABLE, "this map cannot create waypoints")
        await self._verify_free_target(ref, point.grid_id, pose, manifest)
        await self._waypoints.save_waypoint(
            waypoint_id=point.waypoint_id,
            pose=pose,
            map_ref=self._ports.map_reference(ref.map_id, ref.reference_revision),
            anchor_id="sim-world-origin",
            anchor_pose=self._ports.pose(0.0, 0.0, 0.0),
            source_robot_id=self._robot_id,
            status="confirmed",
            latitude=None,
            longitude=None,
            create_only=True,
        )

    async def verify_free_target(
        self, map_ref: MapReference, grid_id: str, pose: PoseReading
    ) -> None:
        ref = MapRefPayload(map_id=map_ref.map_id, reference_revision=map_ref.reference_revision)
        await self._verify_free_target(ref, grid_id, pose, await self._manifest(ref))

    async def _verify_free_target(
        self, ref: MapRefPayload, grid_id: str, pose: PoseReading, manifest: Any
    ) -> None:
        if manifest.grid_id != grid_id:
            raise self._ports.error(ErrorCode.CONFLICT, "map changed before accepting the target")
        dx, dy = pose.x - manifest.origin.x, pose.y - manifest.origin.y
        c, s = math.cos(manifest.origin.yaw), math.sin(manifest.origin.yaw)
        gx = (c * dx + s * dy) / manifest.resolution
        gy = (-s * dx + c * dy) / manifest.resolution
        if not (0 <= gx < manifest.width and 0 <= gy < manifest.height):
            raise self._ports.error(ErrorCode.VALIDATION, "target is outside the map")
        column, row = math.floor(gx), math.floor(gy)
        tx, ix = divmod(column, self._ports.tile_size)
        ty, iy = divmod(row, self._ports.tile_size)
        tile = await self._map(
            self._ports.tile, "map.tile", ref, grid_id=grid_id, tile_x=tx, tile_y=ty
        )
        assert isinstance(tile, self._ports.tile)
        if (
            (tile.grid_id, tile.tile_x, tile.tile_y) != (grid_id, tx, ty)
            or len(tile.cells) != tile.width * tile.height
            or not (ix < tile.width and iy < tile.height)
            or tile.cells[iy * tile.width + ix] != 0
        ):
            raise self._ports.error(ErrorCode.CONFLICT, "target must be on current free space")
        await self._require_grid(ref, grid_id)

    async def verify_task_edit(self, task: TaskDefinition, grid_id: str) -> None:
        try:
            ref = MapRefPayload(
                map_id=task.map_ref.map_id, reference_revision=task.map_ref.reference_revision
            )
        except ValidationError as exc:
            raise self._ports.error(
                ErrorCode.VALIDATION, "invalid workspace map reference"
            ) from exc
        await self._require_grid(ref, grid_id)
        for waypoint_id in dict.fromkeys(task.waypoint_ids):
            point = await self._waypoints.waypoint(waypoint_id)
            if point is not None and point.waypoint.kind == "home":
                point = await self._waypoints.home()
                if point is not None and (not point.adopted or point.link_status != "confirmed"):
                    point = None
            if (
                point is None
                or point.waypoint.id != waypoint_id
                or point.waypoint.map_ref != task.map_ref
                or point.waypoint.status != "confirmed"
            ):
                raise self._ports.error(ErrorCode.CONFLICT, "task contains an unavailable waypoint")
        await self._require_grid(ref, grid_id)

    async def _require_grid(self, ref: MapRefPayload, grid_id: str) -> None:
        if (await self._manifest(ref)).grid_id != grid_id:
            raise self._ports.error(ErrorCode.CONFLICT, "map changed during task editing")

    async def read(self, envelope: WorkspaceEnvelope | MapCatalogEnvelope) -> dict[str, Any]:
        if envelope.robot_id != self._robot_id:
            raise self._ports.error(ErrorCode.VALIDATION, "workspace belongs to another robot")
        if isinstance(envelope, MapCatalogEnvelope):
            try:
                reply = MapCatalogReply.model_validate(await self._maps.request("map.catalog"))
            except (self._ports.ipc_error, OSError, ValidationError) as exc:
                raise self._ports.error(ErrorCode.UNAVAILABLE, "map catalog unavailable") from exc
            if reply.robot_id != self._robot_id:
                raise self._ports.error(ErrorCode.CONFLICT, "catalog belongs to another robot")
            return reply.model_dump(exclude={"v", "ok"})
        payload = envelope.payload
        ref = payload.map_ref
        base = {"robot_id": self._robot_id, "map_ref": ref.model_dump()}

        if envelope.type == "map.tile":
            tile = await self._map(
                self._ports.tile,
                envelope.type,
                ref,
                allow_export=True,
                grid_id=payload.grid_id,
                tile_x=payload.tile_x,
                tile_y=payload.tile_y,
            )
            assert isinstance(tile, self._ports.tile)
            if (tile.grid_id, tile.tile_x, tile.tile_y) != (
                payload.grid_id,
                payload.tile_x,
                payload.tile_y,
            ) or len(tile.cells) != tile.width * tile.height:
                raise self._ports.error(ErrorCode.CONFLICT, "map tile identity or size changed")
            return {**base, **tile.model_dump(exclude={"v", "ok"})}

        if envelope.type == "map.manifest":
            manifest = await self._manifest(ref, allow_export=True)
            return {**base, **manifest.model_dump(exclude={"v", "ok"}, exclude_defaults=True)}
        active = await self._manifest(ref)
        data: dict[str, Any]
        if envelope.type == "task.get":
            assert payload.task_id is not None
            task = await self._tasks.get(payload.task_id)
            if task is None or task.map_ref != self._ports.map_reference(
                ref.map_id, ref.reference_revision
            ):
                raise self._ports.error(
                    ErrorCode.CONFLICT, "task is missing or belongs to another map"
                )
            schedule = (
                {"kind": "interval", "every_seconds": task.schedule.every_seconds}
                if task.schedule.kind == "interval"
                else {"kind": task.schedule.kind, "expression": task.schedule.expression}
            )
            data = {
                "task_id": task.task_id,
                "waypoint_ids": list(task.waypoint_ids),
                "schedule": schedule,
                "timezone": task.timezone,
                "enabled": task.enabled,
                "revision": task.revision,
                "created_at": task.created_at.isoformat(),
            }
        else:
            records = await self._waypoints.page(
                self._ports.map_reference(ref.map_id, ref.reference_revision),
                payload.cursor or "",
                _PAGE_SIZE + 1,
            )
            points = [record.waypoint for record in records[:_PAGE_SIZE]]
            data = {
                "waypoints": [
                    {
                        "waypoint_id": point.id,
                        "x": point.pose.x,
                        "y": point.pose.y,
                        "yaw": point.pose.yaw,
                        "kind": point.kind,
                        "status": point.status,
                    }
                    for point in points
                ],
                "next_cursor": points[-1].id if len(records) > _PAGE_SIZE else None,
            }
        if active.grid_id != (await self._manifest(ref)).grid_id:
            raise self._ports.error(ErrorCode.CONFLICT, "map changed during workspace read")
        return {**base, "grid_id": active.grid_id, **data}

    async def _manifest(self, ref: MapRefPayload, *, allow_export: bool = False) -> Any:
        result = await self._map(
            self._ports.manifest, "map.manifest", ref, allow_export=allow_export
        )
        assert isinstance(result, self._ports.manifest)
        if result.width * result.height > self._ports.max_grid_cells:
            raise self._ports.error(ErrorCode.UNAVAILABLE, "map raster is too large")
        if result.display_only and not allow_export:
            raise self._ports.error(ErrorCode.UNAVAILABLE, "exported maps are read-only")
        return result

    async def _map(
        self,
        model: type[Any],
        kind: str,
        ref: MapRefPayload,
        *,
        allow_export: bool = False,
        **fields: Any,
    ) -> Any:
        request = {**ref.model_dump(), **fields}
        try:
            raw = await self._maps.request(kind, **request)
        except (self._ports.ipc_error, OSError):
            raw = {"ok": False, "error": "unavailable"}
        if (
            allow_export
            and self._exported_grid is not None
            and raw.get("ok") is False
            and raw.get("error") in ("unknown_type", "unavailable")
        ):
            raw = {"v": 1, **self._ports.exported_read(self._exported_grid, kind, request)}
        if raw.get("ok") is False and raw.get("error") in ("conflict", "validation"):
            raise self._ports.error(ErrorCode(raw["error"]), "map reference or tile changed")
        try:
            result = model.model_validate(raw)
        except ValidationError as exc:
            raise self._ports.error(ErrorCode.UNAVAILABLE, "qualified map is unavailable") from exc
        if result.map_ref != ref:
            raise self._ports.error(ErrorCode.CONFLICT, "map reference changed")
        return result
