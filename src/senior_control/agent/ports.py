"""//Compatible: canonical local bindings and the original navigation-only assembly."""

from vehicle_manager.agent.ipc_navigation import IpcNavigation as _IpcNavigation
from vehicle_manager.agent.local_adapters import IpcHomePort as IpcHomePort
from vehicle_manager.agent.local_adapters import IpcMapPort as IpcMapPort
from vehicle_manager.agent.local_adapters import IpcMotion as IpcMotion
from vehicle_manager.agent.local_adapters import IpcState as IpcState
from vehicle_manager.agent.local_adapters import TaskAwareMotion as TaskAwareMotion
from vehicle_manager.agent.local_ports import NavigationValues
from vehicle_manager.agent.local_state import RuntimeState as RuntimeState
from vehicle_manager.agent.local_state import StateReply as StateReply
from vehicle_manager.agent.pose_reply import PoseReply as PoseReply
from vehicle_manager.agent.request_ports import Requester as Requester

from senior_control.navigation import RoutePlan, RouteRejected
from senior_control.tasks.store import TaskStore
from senior_control.waypoints.store import WaypointStore
from vehicle_core.errors import DomainError
from vehicle_core.ipc import IpcError
from vehicle_core.values import Pose2D


class IpcNavigation(_IpcNavigation):
    """//Compatible: original values and Store instances; one request implementation."""

    def __init__(
        self,
        client: Requester,
        waypoints: WaypointStore | None = None,
        *,
        tasks: TaskStore | None = None,
    ) -> None:
        super().__init__(
            client,
            waypoints,
            tasks=tasks,
            values=NavigationValues(DomainError, IpcError, Pose2D, RoutePlan, RouteRejected),
        )
