"""Concrete task execution and navigation-effect bindings over the original manager owners."""

from typing import Any, cast

from vehicle_core.errors import DomainError, ErrorCode
from vehicle_core.route_plan import RouteRejected
from vehicle_manager.storage.bindings import Database
from vehicle_manager.tasks.executor import NavigationPort
from vehicle_manager.tasks.executor import TaskExecutor as _TaskExecutor
from vehicle_manager.tasks.lifecycle_service import LifecycleService
from vehicle_manager.tasks.lifecycle_store import LifecycleStore
from vehicle_manager.tasks.navigation_port import NavigationEffects as _NavigationEffects
from vehicle_manager.tasks.navigation_port import TaskNavigation
from vehicle_manager.tasks.ports import TaskExecutionErrors
from vehicle_manager.tasks.store import TaskStore


class QualificationUnavailable(DomainError):
    """The occurrence was rejected before claiming or invoking navigation."""

    def __init__(self) -> None:
        super().__init__(ErrorCode.UNAVAILABLE, "qualified task navigation is unavailable")


def _error_code(error: Exception) -> ErrorCode:
    # //Compatible: called only after catching the injected original DomainError type.
    return cast(DomainError, error).code


class TaskExecutor(_TaskExecutor):
    """//Compatible: original signature; sole business implementation is inherited."""

    def __init__(
        self,
        database: Database,
        store: TaskStore,
        navigation: NavigationPort | None,
        *,
        lifecycle: LifecycleService | None = None,
        **legacy: Any,
    ) -> None:
        super().__init__(
            database,
            store,
            navigation,
            lifecycle=lifecycle,
            errors=TaskExecutionErrors(
                domain=DomainError,
                qualification_unavailable=QualificationUnavailable,
                route_rejected=RouteRejected,
                code=_error_code,
            ),
            **legacy,
        )


class NavigationEffects(_NavigationEffects):
    """//Compatible: original signature and rejection identity; inherited effect methods."""

    def __init__(
        self,
        store: LifecycleStore,
        navigation: TaskNavigation,
        *,
        timeout_ms: float = 300.0,
    ):
        super().__init__(store, navigation, rejection_type=RouteRejected, timeout_ms=timeout_ms)
