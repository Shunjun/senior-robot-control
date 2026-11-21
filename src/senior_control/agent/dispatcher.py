"""//Compatible: canonical production request bindings and the two historical special adapters."""

from collections.abc import Callable

from senior_mapping.operations import MapOperations
from vehicle_manager.agent.dispatcher import Action as Action
from vehicle_manager.agent.dispatcher import Deadlines as Deadlines
from vehicle_manager.agent.dispatcher import HomePort as HomePort
from vehicle_manager.agent.dispatcher import NavigationAction as NavigationAction
from vehicle_manager.agent.dispatcher import StoreHomePort as _StoreHomePort
from vehicle_manager.agent.dispatcher import StoreMapPort as _StoreMapPort
from vehicle_manager.agent.dispatcher import TaskPort as TaskPort
from vehicle_manager.agent.dispatcher import _fingerprint as _fingerprint
from vehicle_manager.agent.dispatcher import _required as _required
from vehicle_manager.agent.dispatcher import _text as _text
from vehicle_manager.agent.request_bindings import DirectNavigation as DirectNavigation
from vehicle_manager.agent.request_bindings import Dispatcher as Dispatcher
from vehicle_manager.agent.request_bindings import (
    LifecycleNavigationOperations as LifecycleNavigationOperations,
)
from vehicle_manager.agent.request_bindings import MapPort as MapPort
from vehicle_manager.agent.request_bindings import OperationNotCommitted as OperationNotCommitted
from vehicle_manager.agent.request_bindings import StoreTaskPort as StoreTaskPort
from vehicle_manager.agent.request_bindings import TaskOperationResult as TaskOperationResult
from vehicle_manager.agent.request_bindings import _map_ref as _map_ref
from vehicle_manager.agent.request_bindings import _TaskRouteReceiver as _TaskRouteReceiver
from vehicle_manager.agent.request_bindings import _values as _values

from senior_control.waypoints import HomeService, LocalizedPose
from vehicle_core.values import OperationResult


class StoreHomePort(_StoreHomePort[OperationResult]):
    """//Compatible: original construction; inherited unique management implementation."""

    def __init__(self, service: HomeService, localized: Callable[[], LocalizedPose | None]) -> None:
        super().__init__(service, localized, values=_values())


class StoreMapPort(_StoreMapPort):
    """//Compatible: original construction; inherited unique management implementation."""

    def __init__(self, operations: MapOperations, *, robot_id: str) -> None:
        super().__init__(operations, robot_id=robot_id)
