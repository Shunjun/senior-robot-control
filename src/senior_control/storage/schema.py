"""//Compatible: historical domain migration and SQL names retain canonical identity."""

from vehicle_manager.storage.domain_bindings import DOMAIN_MIGRATIONS as DOMAIN_MIGRATIONS
from vehicle_manager.storage.migrations import Migration as Migration
from vehicle_manager.tasks.store import TASK_STATEMENTS as TASK_STATEMENTS
from vehicle_manager.waypoints.store import WAYPOINT_STATEMENTS as WAYPOINT_STATEMENTS

from vehicle_core.map_journal_schema import (
    OPERATION_JOURNAL_STATEMENTS as OPERATION_JOURNAL_STATEMENTS,
)
