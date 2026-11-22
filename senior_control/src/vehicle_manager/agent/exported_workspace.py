"""//Compatible: original workspace constructor injects the same mapping read effect."""

from typing import cast

from senior_mapping.workspace import MAX_GRID_CELLS as MAX_GRID_CELLS
from senior_mapping.workspace import MAX_GRID_SIDE as MAX_GRID_SIDE
from senior_mapping.workspace import TILE_SIZE as TILE_SIZE
from senior_mapping.workspace import WorkspaceGrid, WorkspaceMap

from vehicle_manager.agent.request_ports import Requester
from vehicle_manager.agent.workspace_bindings import _PAGE_SIZE as _PAGE_SIZE
from vehicle_manager.agent.workspace_bindings import WorkspaceReader as _WorkspaceReader
from vehicle_manager.agent.workspace_bindings import _Manifest as _Manifest
from vehicle_manager.agent.workspace_bindings import _MapReply as _MapReply
from vehicle_manager.agent.workspace_bindings import _Origin as _Origin
from vehicle_manager.agent.workspace_bindings import _Tile as _Tile
from vehicle_manager.agent.workspace_bindings import _WaypointCreate as _WaypointCreate
from vehicle_manager.storage.domain_bindings import TaskStore, WaypointStore


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
            exported_read=lambda grid, kind, fields: WorkspaceMap(
                lambda: cast(WorkspaceGrid, grid)
            ).read(kind, fields),
        )
