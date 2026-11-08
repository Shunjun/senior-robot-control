"""Read the vehicle's workspace without putting reads in its durable edit journal."""

from typing import Annotated, Any, Literal

from pydantic import Field, ValidationError

from senior_control.agent.ports import Requester
from senior_control.errors import DomainError, ErrorCode
from senior_control.mapping.workspace import (
    MAX_GRID_CELLS,
    MAX_GRID_SIDE,
    TILE_SIZE,
    WorkspaceGrid,
    WorkspaceMap,
)
from senior_control.models import MapRef
from senior_control.protocol import MapRefPayload, ShortText, StrictModel, WorkspaceEnvelope
from senior_control.runtime.ipc import IpcError
from senior_control.tasks import TaskDefinition, TaskStore
from senior_control.waypoints.store import WaypointStore

_PAGE_SIZE = 32


class _MapReply(StrictModel):
    v: Literal[1]
    ok: Literal[True]
    map_ref: MapRefPayload
    grid_id: ShortText


class _Origin(StrictModel):
    x: float
    y: float
    yaw: float


class _Manifest(_MapReply):
    width: int = Field(gt=0, le=MAX_GRID_SIDE)
    height: int = Field(gt=0, le=MAX_GRID_SIDE)
    resolution: float = Field(gt=0)
    origin: _Origin
    tile_size: Literal[32]
    display_only: bool = False


class _Tile(_MapReply):
    tile_x: int = Field(ge=0)
    tile_y: int = Field(ge=0)
    width: int = Field(gt=0, le=TILE_SIZE)
    height: int = Field(gt=0, le=TILE_SIZE)
    cells: list[Annotated[int, Field(ge=-1, le=100)]] = Field(max_length=TILE_SIZE**2)


class WorkspaceReader:
    def __init__(
        self,
        robot_id: str,
        tasks: TaskStore,
        waypoints: WaypointStore,
        maps: Requester,
        *,
        exported_grid: WorkspaceGrid | None = None,
    ) -> None:
        self._robot_id = robot_id
        self._tasks = tasks
        self._waypoints = waypoints
        self._maps = maps
        if exported_grid is not None and not exported_grid.display_only:
            raise ValueError("workspace export must be read-only")
        self._exported_grid = exported_grid

    async def active_map(self) -> MapRef | None:
        try:
            reply = _MapReply.model_validate(await self._maps.request("map.active"))
            return MapRef(reply.map_ref.map_id, reply.map_ref.reference_revision)
        except (IpcError, OSError, ValidationError):
            return self._exported_grid.map_ref if self._exported_grid is not None else None

    async def verify_task_edit(self, task: TaskDefinition, grid_id: str) -> None:
        try:
            ref = MapRefPayload(
                map_id=task.map_ref.map_id, reference_revision=task.map_ref.reference_revision
            )
        except ValidationError as exc:
            raise DomainError(ErrorCode.VALIDATION, "invalid workspace map reference") from exc
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
                raise DomainError(ErrorCode.CONFLICT, "task contains an unavailable waypoint")
        await self._require_grid(ref, grid_id)

    async def _require_grid(self, ref: MapRefPayload, grid_id: str) -> None:
        if (await self._manifest(ref)).grid_id != grid_id:
            raise DomainError(ErrorCode.CONFLICT, "map changed during task editing")

    async def read(self, envelope: WorkspaceEnvelope) -> dict[str, Any]:
        if envelope.robot_id != self._robot_id:
            raise DomainError(ErrorCode.VALIDATION, "workspace belongs to another robot")
        payload = envelope.payload
        ref = payload.map_ref
        base = {"robot_id": self._robot_id, "map_ref": ref.model_dump()}

        if envelope.type == "map.tile":
            tile = await self._map(
                _Tile,
                envelope.type,
                ref,
                allow_export=True,
                grid_id=payload.grid_id,
                tile_x=payload.tile_x,
                tile_y=payload.tile_y,
            )
            assert isinstance(tile, _Tile)
            if (tile.grid_id, tile.tile_x, tile.tile_y) != (
                payload.grid_id,
                payload.tile_x,
                payload.tile_y,
            ) or len(tile.cells) != tile.width * tile.height:
                raise DomainError(ErrorCode.CONFLICT, "map tile identity or size changed")
            return {**base, **tile.model_dump(exclude={"v", "ok"})}

        if envelope.type == "map.manifest":
            manifest = await self._manifest(ref, allow_export=True)
            return {**base, **manifest.model_dump(exclude={"v", "ok"}, exclude_defaults=True)}
        active = await self._manifest(ref)
        data: dict[str, Any]
        if envelope.type == "task.get":
            assert payload.task_id is not None
            task = await self._tasks.get(payload.task_id)
            if task is None or task.map_ref != MapRef(ref.map_id, ref.reference_revision):
                raise DomainError(ErrorCode.CONFLICT, "task is missing or belongs to another map")
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
                MapRef(ref.map_id, ref.reference_revision), payload.cursor or "", _PAGE_SIZE + 1
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
            raise DomainError(ErrorCode.CONFLICT, "map changed during workspace read")
        return {**base, "grid_id": active.grid_id, **data}

    async def _manifest(self, ref: MapRefPayload, *, allow_export: bool = False) -> _Manifest:
        result = await self._map(_Manifest, "map.manifest", ref, allow_export=allow_export)
        assert isinstance(result, _Manifest)
        if result.width * result.height > MAX_GRID_CELLS:
            raise DomainError(ErrorCode.UNAVAILABLE, "map raster is too large")
        if result.display_only and not allow_export:
            raise DomainError(ErrorCode.UNAVAILABLE, "exported maps are read-only")
        return result

    async def _map(
        self,
        model: type[_MapReply],
        kind: str,
        ref: MapRefPayload,
        *,
        allow_export: bool = False,
        **fields: Any,
    ) -> _MapReply:
        request = {**ref.model_dump(), **fields}
        try:
            raw = await self._maps.request(kind, **request)
        except (IpcError, OSError):
            raw = {"ok": False, "error": "unavailable"}
        if (
            allow_export
            and self._exported_grid is not None
            and raw.get("ok") is False
            and raw.get("error") in ("unknown_type", "unavailable")
        ):
            exported = WorkspaceMap(lambda: self._exported_grid)
            raw = {"v": 1, **exported.read(kind, request)}
        if raw.get("ok") is False and raw.get("error") in ("conflict", "validation"):
            raise DomainError(ErrorCode(raw["error"]), "map reference or tile changed")
        try:
            result = model.model_validate(raw)
        except ValidationError as exc:
            raise DomainError(ErrorCode.UNAVAILABLE, "qualified map is unavailable") from exc
        if result.map_ref != ref:
            raise DomainError(ErrorCode.CONFLICT, "map reference changed")
        return result
