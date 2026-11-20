"""//Compatible: original target-validator construction with the same local values."""

from typing import Protocol

from vehicle_manager.agent.local_adapters import IpcState
from vehicle_manager.waypoints.direct_navigation import DirectNavigation as _DirectNavigation
from vehicle_manager.waypoints.direct_navigation import _Goal as _Goal
from vehicle_manager.waypoints.direct_navigation import _Reference as _Reference
from vehicle_manager.waypoints.ports import WaypointValuePorts

from senior_control.agent.workspace import WorkspaceReader
from senior_control.waypoints.store import WaypointStore
from vehicle_core.errors import DomainError
from vehicle_core.values import MapRef, Pose2D


class _Navigation(Protocol):
    async def start_poses(
        self, execution_id: str, map_ref: MapRef, map_generation: str, poses: tuple[Pose2D, ...]
    ) -> None: ...

    async def cancel_current(self) -> None: ...


class DirectNavigation(_DirectNavigation[MapRef, Pose2D]):
    """//Compatible: original signature; inherited request/qualification implementation."""

    def __init__(
        self,
        state: IpcState,
        workspace: WorkspaceReader,
        waypoints: WaypointStore,
        navigation: _Navigation,
    ) -> None:
        super().__init__(
            state,
            workspace,
            waypoints,
            navigation,
            values=WaypointValuePorts(map_reference=MapRef, pose=Pose2D, error=DomainError),
        )
