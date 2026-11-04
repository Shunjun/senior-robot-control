"""Ordered domain migrations. Task 9 owns version 1.

Later tasks append their own Migration after this one. They must not renumber
these statements or add waypoint and schedule tables to version 1.
"""

from senior_control.mapping.operations import OPERATION_JOURNAL_STATEMENTS
from senior_control.storage.migrations import Migration
from senior_control.tasks.store import TASK_STATEMENTS
from senior_control.waypoints.store import WAYPOINT_STATEMENTS

DOMAIN_MIGRATIONS: tuple[Migration, ...] = (
    Migration(1, OPERATION_JOURNAL_STATEMENTS),
    Migration(2, WAYPOINT_STATEMENTS),
    Migration(3, TASK_STATEMENTS),
)
