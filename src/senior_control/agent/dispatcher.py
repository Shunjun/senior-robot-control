"""//Compatible: original management constructors and domain result/exception identities."""

from collections.abc import Awaitable, Callable, Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, Any, Protocol

from senior_mapping.operations import MapOperations
from vehicle_manager.agent.dispatcher import (
    Action as Action,
)
from vehicle_manager.agent.dispatcher import (
    Deadlines as Deadlines,
)
from vehicle_manager.agent.dispatcher import (
    Dispatcher as _Dispatcher,
)
from vehicle_manager.agent.dispatcher import (
    HomePort as HomePort,
)
from vehicle_manager.agent.dispatcher import (
    LifecycleNavigationOperations as _LifecycleNavigationOperations,
)
from vehicle_manager.agent.dispatcher import (
    NavigationAction as NavigationAction,
)
from vehicle_manager.agent.dispatcher import (
    StoreHomePort as _StoreHomePort,
)
from vehicle_manager.agent.dispatcher import (
    StoreMapPort as _StoreMapPort,
)
from vehicle_manager.agent.dispatcher import (
    StoreTaskPort as _StoreTaskPort,
)
from vehicle_manager.agent.dispatcher import (
    TaskPort as TaskPort,
)
from vehicle_manager.agent.dispatcher import (
    _fingerprint as _fingerprint,
)
from vehicle_manager.agent.dispatcher import (
    _map_ref as _canonical_map_ref,
)
from vehicle_manager.agent.dispatcher import (
    _required as _required,
)
from vehicle_manager.agent.dispatcher import _TaskRouteReceiver as _ManagerTaskRouteReceiver
from vehicle_manager.agent.dispatcher import (
    _text as _text,
)
from vehicle_manager.agent.request_ports import RequestValues
from vehicle_manager.observability import Metrics

from senior_control.agent.direct_navigation import DirectNavigation
from senior_control.agent.journal import AgentJournal
from senior_control.agent.ports import IpcState
from senior_control.agent.workspace import WorkspaceReader
from senior_control.errors import DomainError, ErrorCode
from senior_control.models import MapRef, OperationResult, OperationStatus
from senior_control.protocol import OperationEnvelope
from senior_control.runtime.diagnostics import PendingCounter
from senior_control.tasks import TaskDefinition, TaskStore
from senior_control.waypoints import HomeService, LocalizedPose
from senior_control.waypoints.store import WaypointStore

if TYPE_CHECKING:
    from senior_control.tasks.executor import TaskExecutor


# //Compatible: original concrete local map port signature.
class MapPort(Protocol):
    async def export(self, operation_id: str, map_ref: MapRef, destination: Path) -> None: ...

    async def import_(self, operation_id: str, path: Path) -> None: ...

    async def clear(self, operation_id: str, expected: MapRef) -> None: ...


class OperationNotCommitted(DomainError):
    """A retry found an operation that is not ``succeeded``. ``status`` is what is stored."""

    def __init__(self, code: ErrorCode, message: str, status: OperationStatus) -> None:
        super().__init__(code, message)
        self.status = status


@dataclass(frozen=True)
class TaskOperationResult(OperationResult):
    task_state: dict[str, Any]


def _values() -> RequestValues[OperationResult]:
    # //Compatible: original concrete values and diagnostics per constructed owner.
    return RequestValues(
        MapRef,
        DomainError,
        OperationStatus,
        OperationResult,
        OperationNotCommitted,
        TaskOperationResult,
        DirectNavigation,
        PendingCounter,
    )


class Dispatcher(_Dispatcher[OperationResult]):
    """//Compatible: original construction; inherited unique management implementation."""

    def __init__(
        self,
        robot_id: str,
        *,
        journal: AgentJournal,
        tasks: TaskPort,
        home: HomePort,
        maps: MapPort,
        deadlines: Deadlines,
        metrics: Metrics,
        map_root: Path,
        admit: Callable[[OperationEnvelope], None],
        navigation: NavigationAction | None = None,
        waypoint_create: Callable[[Mapping[str, Any]], Awaitable[None]] | None = None,
    ) -> None:
        super().__init__(
            robot_id,
            journal=journal,
            tasks=tasks,
            home=home,
            maps=maps,
            deadlines=deadlines,
            metrics=metrics,
            map_root=map_root,
            admit=admit,
            navigation=navigation,
            waypoint_create=waypoint_create,
            values=_values(),
        )


class StoreTaskPort(_StoreTaskPort[OperationResult]):
    """//Compatible: original construction; inherited unique management implementation."""

    def __init__(
        self,
        store: TaskStore,
        *,
        robot_id: str,
        verify_workspace: Callable[[TaskDefinition, str], Awaitable[None]] | None = None,
    ) -> None:
        super().__init__(
            store, robot_id=robot_id, verify_workspace=verify_workspace, values=_values()
        )


class StoreHomePort(_StoreHomePort[OperationResult]):
    """//Compatible: original construction; inherited unique management implementation."""

    def __init__(self, service: HomeService, localized: Callable[[], LocalizedPose | None]) -> None:
        super().__init__(service, localized, values=_values())


class StoreMapPort(_StoreMapPort):
    """//Compatible: original construction; inherited unique management implementation."""

    def __init__(self, operations: MapOperations, *, robot_id: str) -> None:
        super().__init__(operations, robot_id=robot_id)


class LifecycleNavigationOperations(_LifecycleNavigationOperations[OperationResult]):
    """//Compatible: original construction; inherited unique management implementation."""

    def __init__(
        self,
        executor: "TaskExecutor",
        state: IpcState,
        workspace: WorkspaceReader,
        waypoints: WaypointStore,
    ) -> None:
        super().__init__(executor, state, workspace, waypoints, values=_values())


def _map_ref(body: Mapping[str, Any]) -> MapRef:
    # //Compatible: helper retains the actual local constructor.
    return _canonical_map_ref(body, MapRef)


class _TaskRouteReceiver(_ManagerTaskRouteReceiver[OperationResult]):
    """//Compatible: original local construction; real task receiver stays canonical."""

    def __init__(self, executor: "TaskExecutor", fingerprint: str, state: IpcState) -> None:
        super().__init__(executor, fingerprint, state, values=_values())
