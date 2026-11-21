"""Concrete request/domain bindings over the sole management dispatcher and target owners."""

from collections.abc import Awaitable, Callable, Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Protocol

from senior_contracts.wire import OperationEnvelope

from vehicle_core.diagnostics import PendingCounter
from vehicle_core.errors import DomainError, ErrorCode
from vehicle_core.values import MapRef, OperationResult, OperationStatus, Pose2D
from vehicle_manager.agent.dispatcher import Deadlines, HomePort, NavigationAction, TaskPort
from vehicle_manager.agent.dispatcher import Dispatcher as _Dispatcher
from vehicle_manager.agent.dispatcher import (
    LifecycleNavigationOperations as _LifecycleNavigationOperations,
)
from vehicle_manager.agent.dispatcher import StoreTaskPort as _StoreTaskPort
from vehicle_manager.agent.dispatcher import _map_ref as _canonical_map_ref
from vehicle_manager.agent.dispatcher import _TaskRouteReceiver as _ManagerTaskRouteReceiver
from vehicle_manager.agent.local_adapters import IpcState
from vehicle_manager.agent.production_bindings import AgentJournal
from vehicle_manager.agent.request_ports import RequestValues
from vehicle_manager.agent.workspace import WorkspaceReader
from vehicle_manager.observability import Metrics
from vehicle_manager.tasks.execution_bindings import TaskExecutor
from vehicle_manager.tasks.models import TaskDefinition
from vehicle_manager.tasks.store import TaskStore
from vehicle_manager.waypoints.direct_navigation import DirectNavigation as _DirectNavigation
from vehicle_manager.waypoints.ports import HomeReader, WaypointValuePorts
from vehicle_manager.waypoints.store import WaypointStore


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
        waypoints: HomeReader[Pose2D],
        navigation: _Navigation,
    ) -> None:
        super().__init__(
            state,
            workspace,
            waypoints,
            navigation,
            values=WaypointValuePorts(map_reference=MapRef, pose=Pose2D, error=DomainError),
        )


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


class LifecycleNavigationOperations(_LifecycleNavigationOperations[OperationResult]):
    """//Compatible: original construction; inherited unique management implementation."""

    def __init__(
        self,
        executor: "TaskExecutor",
        state: IpcState,
        workspace: WorkspaceReader,
        waypoints: WaypointStore[Any, Any, Pose2D],
    ) -> None:
        super().__init__(executor, state, workspace, waypoints, values=_values())


def _map_ref(body: Mapping[str, Any]) -> MapRef:
    # //Compatible: helper retains the actual local constructor.
    return _canonical_map_ref(body, MapRef)


class _TaskRouteReceiver(_ManagerTaskRouteReceiver[OperationResult]):
    """//Compatible: original local construction; real task receiver stays canonical."""

    def __init__(self, executor: "TaskExecutor", fingerprint: str, state: IpcState) -> None:
        super().__init__(executor, fingerprint, state, values=_values())
