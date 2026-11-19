"""Read the vehicle's workspace without putting reads in its durable edit journal."""

from typing import Annotated, Literal

from pydantic import Field
from senior_mapping.workspace import (
    MAX_GRID_CELLS,
    MAX_GRID_SIDE,
    TILE_SIZE,
    WorkspaceGrid,
    WorkspaceMap,
)
from vehicle_manager.agent.request_ports import WorkspacePorts
from vehicle_manager.agent.workspace import WorkspaceReader as _WorkspaceReader

from senior_control.agent.ports import Requester
from senior_control.errors import DomainError
from senior_control.models import MapRef, Pose2D
from senior_control.protocol import MapRefPayload, ShortText, StrictModel
from senior_control.runtime.ipc import IpcError
from senior_control.tasks import TaskStore
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
    waypoint_writes: bool = False
    navigation_ready: bool = False


class _Tile(_MapReply):
    tile_x: int = Field(ge=0)
    tile_y: int = Field(ge=0)
    width: int = Field(gt=0, le=TILE_SIZE)
    height: int = Field(gt=0, le=TILE_SIZE)
    cells: list[Annotated[int, Field(ge=-1, le=100)]] = Field(max_length=TILE_SIZE**2)


class _WaypointCreate(MapRefPayload):
    waypoint_id: ShortText
    grid_id: ShortText
    x: float
    y: float
    yaw: float


# //Compatible: local validators/types and original mapping fallback, sole manager orchestration.


class WorkspaceReader(_WorkspaceReader):
    def __init__(
        self,
        robot_id: str,
        tasks: TaskStore,
        waypoints: WaypointStore,
        maps: Requester,
        *,
        exported_grid: WorkspaceGrid | None = None,
    ) -> None:
        super().__init__(
            robot_id,
            tasks,
            waypoints,
            maps,
            exported_grid=exported_grid,
            ports=WorkspacePorts(
                MapRef,
                Pose2D,
                DomainError,
                IpcError,
                _MapReply,
                _Manifest,
                _Tile,
                _WaypointCreate,
                MAX_GRID_CELLS,
                TILE_SIZE,
                lambda grid, kind, fields: WorkspaceMap(lambda: grid).read(kind, fields),
            ),
        )
