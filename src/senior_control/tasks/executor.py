"""//Compatible: original execution bindings and helpers are the same manager objects."""

from vehicle_manager.storage.bindings import Database as Database
from vehicle_manager.tasks.execution_bindings import (
    QualificationUnavailable as QualificationUnavailable,
)
from vehicle_manager.tasks.execution_bindings import TaskExecutor as TaskExecutor
from vehicle_manager.tasks.execution_bindings import _error_code as _error_code
from vehicle_manager.tasks.executor import NavigationFailure as NavigationFailure
from vehicle_manager.tasks.executor import NavigationPort as NavigationPort
from vehicle_manager.tasks.executor import ProcessCrash as ProcessCrash
from vehicle_manager.tasks.executor import QualifiedNavigationPort as QualifiedNavigationPort
from vehicle_manager.tasks.lifecycle_service import LifecycleService as LifecycleService
from vehicle_manager.tasks.store import TaskStore as TaskStore

from vehicle_core.errors import DomainError as DomainError
from vehicle_core.errors import ErrorCode as ErrorCode
from vehicle_core.route_plan import RouteRejected as RouteRejected
