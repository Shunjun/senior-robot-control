"""Concrete IPC workspace validation with an explicit optional exported-map read effect."""

from collections.abc import Callable, Mapping
from typing import Annotated, Any, Literal, Protocol

from pydantic import Field
from senior_contracts.wire import MapRefPayload, ShortText, StrictModel

from vehicle_core.errors import DomainError
from vehicle_core.ipc import IpcError
from vehicle_core.values import MapRef, Pose2D
from vehicle_core.workspace_limits import MAX_GRID_CELLS, MAX_GRID_SIDE, TILE_SIZE
from vehicle_manager.agent.request_ports import Requester, WorkspacePorts
from vehicle_manager.agent.workspace import WorkspaceReader as _WorkspaceReader
from vehicle_manager.storage.domain_bindings import TaskStore, WaypointStore

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


class _ExportedGrid(Protocol):
    @property
    def display_only(self) -> bool: ...

    @property
    def map_ref(self) -> MapRef: ...


ExportedRead = Callable[[_ExportedGrid, str, Mapping[str, Any]], dict[str, Any]]


def _missing_exported_read(
    grid: _ExportedGrid, kind: str, fields: Mapping[str, Any]
) -> dict[str, Any]:
    raise ValueError("workspace export requires an explicit read effect")


class WorkspaceReader(_WorkspaceReader):
    def __init__(
        self,
        robot_id: str,
        tasks: TaskStore,
        waypoints: WaypointStore,
        maps: Requester,
        *,
        exported_grid: _ExportedGrid | None = None,
        exported_read: ExportedRead | None = None,
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
                exported_read if exported_read is not None else _missing_exported_read,
            ),
        )
        # Preserve the generic read-only validation before checking the new effect seam.
        if exported_grid is not None and not callable(exported_read):
            raise ValueError("workspace export requires an explicit read effect")
