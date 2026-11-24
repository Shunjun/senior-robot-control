"""Ordered domain migrations with the map owner's original SQL supplied explicitly."""

from vehicle_manager.storage.migrations import Migration
from vehicle_manager.tasks.store import TASK_STATEMENTS
from vehicle_manager.waypoints.store import WAYPOINT_STATEMENTS


def domain_migrations(operation_journal_statements: tuple[str, ...]) -> tuple[Migration, ...]:
    return (
        Migration(1, operation_journal_statements),
        Migration(2, WAYPOINT_STATEMENTS),
        Migration(3, TASK_STATEMENTS),
        # //Compatible: nullable only for internal definitions/empty-catalog upgrades;
        # no historical user task backfill. All new inserts explicitly persist name.
        Migration(4, ("ALTER TABLE task_definition ADD COLUMN name TEXT",)),
    )
