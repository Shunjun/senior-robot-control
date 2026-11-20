"""//Compatible: ordered domain migrations. Task 9 owns version 1.

Later tasks append their own Migration after this one. They must not renumber
these statements or add waypoint and schedule tables to version 1.
"""

from senior_mapping.schema import OPERATION_JOURNAL_STATEMENTS
from vehicle_manager.storage.domain_schema import domain_migrations

from senior_control.storage.migrations import Migration
from senior_control.tasks.store import TASK_STATEMENTS as TASK_STATEMENTS
from senior_control.waypoints.store import WAYPOINT_STATEMENTS as WAYPOINT_STATEMENTS

DOMAIN_MIGRATIONS: tuple[Migration, ...] = domain_migrations(OPERATION_JOURNAL_STATEMENTS)
