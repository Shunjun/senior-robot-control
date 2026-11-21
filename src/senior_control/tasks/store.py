"""//Compatible: original TaskStore construction over the sole manager business implementation."""

from vehicle_manager.storage.bindings import Database as Database
from vehicle_manager.storage.domain_bindings import TaskStore as TaskStore
from vehicle_manager.tasks.models import ExecutionRecord as ExecutionRecord
from vehicle_manager.tasks.models import ExecutionStatus as ExecutionStatus
from vehicle_manager.tasks.models import SkipRecord as SkipRecord
from vehicle_manager.tasks.models import TaskDefinition as TaskDefinition
from vehicle_manager.tasks.schedule import Schedule as Schedule
from vehicle_manager.tasks.store import _LATEST_EXECUTION_SQL as _LATEST_EXECUTION_SQL
from vehicle_manager.tasks.store import _SCAN_COLUMNS as _SCAN_COLUMNS
from vehicle_manager.tasks.store import _SCAN_LIMIT as _SCAN_LIMIT
from vehicle_manager.tasks.store import ACTIVE_SCAN_SQL as ACTIVE_SCAN_SQL
from vehicle_manager.tasks.store import TASK_STATEMENTS as TASK_STATEMENTS
from vehicle_manager.tasks.store import WINDOW_SCAN_SQL as WINDOW_SCAN_SQL

from vehicle_core.errors import DomainError as DomainError
from vehicle_core.errors import ErrorCode as ErrorCode
from vehicle_core.values import MapRef as MapRef
