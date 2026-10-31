"""Agent-side runtime migrations, separate from cloud and agent journal versions."""

from senior_control.storage.database import Database
from senior_control.storage.migrations import Migration

# //Compatible: version 1 adopts the tables created by earlier unversioned TaskStore builds.
CORE_MIGRATIONS = (
    Migration(
        1,
        (
            "CREATE TABLE IF NOT EXISTS task_scan_cursor "
            "(robot_id TEXT PRIMARY KEY, last_tick_utc TEXT NOT NULL)",
            "CREATE TABLE IF NOT EXISTS task_execution_generation "
            "(execution_id TEXT PRIMARY KEY, generation INTEGER NOT NULL, "
            "FOREIGN KEY(execution_id) REFERENCES task_execution(execution_id))",
        ),
    ),
    Migration(
        2,
        ("CREATE TABLE home_execution_guard (robot_id TEXT PRIMARY KEY, execution_id TEXT)",),
    ),
    Migration(
        3,
        (
            "CREATE TABLE map_graph_guard "
            "(robot_id TEXT PRIMARY KEY, operation_id TEXT NOT NULL, history TEXT NOT NULL)",
        ),
    ),
)


async def migrate_core(
    database: Database, migrations: tuple[Migration, ...] = CORE_MIGRATIONS
) -> None:
    versions = [migration.version for migration in migrations]
    if versions != list(range(1, len(versions) + 1)):
        raise ValueError("core migrations must be consecutive")
    async with database.write_lock:
        connection = database.connection
        await connection.rollback()
        await connection.execute("BEGIN IMMEDIATE")
        try:
            await connection.execute(
                "CREATE TABLE IF NOT EXISTS core_schema_version "
                "(id INTEGER PRIMARY KEY CHECK(id = 1), version INTEGER NOT NULL)"
            )
            await connection.execute(
                "INSERT OR IGNORE INTO core_schema_version (id, version) VALUES (1, 0)"
            )
            async with connection.execute(
                "SELECT version FROM core_schema_version WHERE id = 1"
            ) as cursor:
                row = await cursor.fetchone()
            if row is None or not isinstance(row[0], int) or not 0 <= row[0] <= len(migrations):
                raise ValueError("unknown core schema")
            for migration in migrations[row[0] :]:
                for statement in migration.statements:
                    await connection.execute(statement)
                await connection.execute(
                    "UPDATE core_schema_version SET version = ? WHERE id = 1",
                    (migration.version,),
                )
            await connection.commit()
        except BaseException:
            await connection.rollback()
            raise
