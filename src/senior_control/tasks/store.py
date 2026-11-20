"""//Compatible: original TaskStore construction over the sole manager business implementation."""

from vehicle_manager.tasks.models import ExecutionRecord as ExecutionRecord
from vehicle_manager.tasks.models import ExecutionStatus as ExecutionStatus
from vehicle_manager.tasks.models import SkipRecord as SkipRecord
from vehicle_manager.tasks.models import TaskDefinition as TaskDefinition
from vehicle_manager.tasks.ports import TaskStorePorts
from vehicle_manager.tasks.schedule import Schedule as Schedule
from vehicle_manager.tasks.store import _LATEST_EXECUTION_SQL as _LATEST_EXECUTION_SQL
from vehicle_manager.tasks.store import _SCAN_COLUMNS as _SCAN_COLUMNS
from vehicle_manager.tasks.store import _SCAN_LIMIT as _SCAN_LIMIT
from vehicle_manager.tasks.store import ACTIVE_SCAN_SQL as ACTIVE_SCAN_SQL
from vehicle_manager.tasks.store import TASK_STATEMENTS as TASK_STATEMENTS
from vehicle_manager.tasks.store import WINDOW_SCAN_SQL as WINDOW_SCAN_SQL
from vehicle_manager.tasks.store import TaskStore as _TaskStore

from senior_control.storage.database import Database
from vehicle_core.errors import DomainError
from vehicle_core.errors import ErrorCode as ErrorCode
from vehicle_core.values import MapRef


class TaskStore(_TaskStore):
    """//Compatible: original signature and constructors; inherited SQL/transaction methods."""

    def __init__(self, database: Database, *, robot_id: str) -> None:
        async def prepare_schema() -> None:
            from senior_control.storage.core_schema import migrate_core
            from senior_control.storage.migrations import migrate
            from senior_control.storage.schema import DOMAIN_MIGRATIONS

            await migrate(database, list(DOMAIN_MIGRATIONS))
            await migrate_core(database)

        super().__init__(
            database,
            robot_id=robot_id,
            ports=TaskStorePorts(
                prepare_schema=prepare_schema,
                map_reference=MapRef,
                error=DomainError,
            ),
        )
