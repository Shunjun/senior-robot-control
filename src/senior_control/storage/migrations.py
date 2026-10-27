"""Transactional, ordered schema upgrades with SQLite backup."""

import asyncio
import time
from dataclasses import dataclass

import aiosqlite

from senior_control.storage.database import Database


@dataclass(frozen=True)
class Migration:
    version: int
    statements: tuple[str, ...]


async def _backup(db: Database) -> None:
    """Let the SQLite worker finish or hit its deadline even if the caller cancels."""
    deadline = time.monotonic() + db.busy_timeout_ms / 1000

    def progress(status: int, remaining: int, total: int) -> None:
        if time.monotonic() > deadline:
            raise TimeoutError("SQLite backup deadline exceeded")

    async def run() -> None:
        async with aiosqlite.connect(
            str(db.path) + ".backup", timeout=db.busy_timeout_ms / 1000
        ) as backup:
            await db.connection.backup(backup, pages=128, progress=progress, sleep=0.01)

    task = asyncio.create_task(run())
    try:
        await asyncio.shield(task)
    except asyncio.CancelledError:
        try:
            await task
        except Exception:
            pass
        raise


async def migrate(db: Database, migrations: list[Migration]) -> None:
    versions = [m.version for m in migrations]
    if versions != list(range(1, len(versions) + 1)):
        raise ValueError("migrations must be consecutive from version 1")
    async with db.write_lock:
        async with db.connection.execute("PRAGMA user_version") as cursor:
            row = await cursor.fetchone()
        assert row is not None
        current = int(row[0])
        if current > len(migrations):
            raise ValueError("unknown database schema")
        if current == len(migrations):
            return
        await _backup(db)
        try:
            async with db.connection.execute("BEGIN IMMEDIATE"):
                pass
            for migration in migrations[current:]:
                for statement in migration.statements:
                    async with db.connection.execute(statement):
                        pass
                async with db.connection.execute(f"PRAGMA user_version = {migration.version}"):
                    pass
            await db.connection.commit()
        except BaseException:
            await db.connection.rollback()
            raise
