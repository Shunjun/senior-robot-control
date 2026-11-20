"""//Compatible: original task-driver construction and concrete exception identities."""

from typing import Any, cast

from vehicle_manager.tasks.executor import NavigationFailure as NavigationFailure
from vehicle_manager.tasks.executor import NavigationPort as NavigationPort
from vehicle_manager.tasks.executor import ProcessCrash as ProcessCrash
from vehicle_manager.tasks.executor import QualifiedNavigationPort as QualifiedNavigationPort
from vehicle_manager.tasks.executor import TaskExecutor as _TaskExecutor
from vehicle_manager.tasks.lifecycle_service import LifecycleService
from vehicle_manager.tasks.ports import TaskExecutionErrors

from senior_control.errors import DomainError, ErrorCode
from senior_control.navigation import RouteRejected
from senior_control.storage.database import Database
from senior_control.tasks.store import TaskStore


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
