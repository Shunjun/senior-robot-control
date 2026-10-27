"""Transactional, ordered schema upgrades with SQLite backup."""

from dataclasses import dataclass

import aiosqlite

from senior_control.storage.database import Database


@dataclass(frozen=True)
class Migration:
    version: int
    statements: tuple[str, ...]


async def migrate(db: Database, migrations: list[Migration]) -> None:
    versions = [m.version for m in migrations]
    if versions != list(range(1, len(versions) + 1)):
        raise ValueError("migrations must be consecutive from version 1")
    async with db.write_lock:
        cursor = await db.connection.execute("PRAGMA user_version")
        row = await cursor.fetchone()
        assert row is not None
        current = int(row[0])
        if current > len(migrations):
            raise ValueError("unknown database schema")
        if current == len(migrations):
            return
        async with aiosqlite.connect(str(db.path) + ".backup") as backup:
            await db.connection.backup(backup)
        try:
            await db.connection.execute("BEGIN IMMEDIATE")
            for migration in migrations[current:]:
                for statement in migration.statements:
                    await db.connection.execute(statement)
                await db.connection.execute(f"PRAGMA user_version = {migration.version}")
            await db.connection.commit()
        except BaseException:
            await db.connection.rollback()
            raise
