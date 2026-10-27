import sqlite3

import pytest

from senior_control.storage.database import Database
from senior_control.storage.migrations import Migration, migrate


@pytest.mark.asyncio
async def test_schema_migration_rollback(tmp_path):
    path = tmp_path / "test.db"
    async with Database(path) as db:
        await migrate(db, [Migration(1, ("CREATE TABLE item (value TEXT)",))])
        with pytest.raises(sqlite3.OperationalError):
            await migrate(
                db,
                [
                    Migration(1, ("CREATE TABLE item (value TEXT)",)),
                    Migration(2, ("ALTER TABLE item ADD COLUMN other TEXT", "INVALID SQL")),
                ],
            )
        cursor = await db.connection.execute("PRAGMA user_version")
        assert (await cursor.fetchone())[0] == 1
        cursor = await db.connection.execute("PRAGMA table_info(item)")
        assert [row[1] for row in await cursor.fetchall()] == ["value"]


@pytest.mark.asyncio
async def test_sqlite_busy_is_bounded(tmp_path):
    import asyncio

    path = tmp_path / "busy.db"
    async with (
        Database(path, busy_timeout_ms=50) as first,
        Database(path, busy_timeout_ms=50) as second,
    ):
        await first.connection.execute("BEGIN IMMEDIATE")
        with pytest.raises(sqlite3.OperationalError):
            await asyncio.wait_for(second.connection.execute("BEGIN IMMEDIATE"), timeout=1)
        await first.connection.rollback()


@pytest.mark.asyncio
async def test_unknown_schema_refuses_migration(tmp_path):
    async with Database(tmp_path / "future.db") as db:
        await db.connection.execute("PRAGMA user_version = 10")
        with pytest.raises(ValueError):
            await migrate(db, [])


@pytest.mark.asyncio
async def test_disk_full_has_no_success_ack(tmp_path):
    async with Database(tmp_path / "full.db") as db:
        await db.connection.execute("CREATE TABLE item (value BLOB)")
        await db.connection.commit()
        cursor = await db.connection.execute("PRAGMA page_count")
        count = (await cursor.fetchone())[0]
        await db.connection.execute(f"PRAGMA max_page_count = {count}")
        with pytest.raises(sqlite3.OperationalError, match="full"):
            await db.connection.execute("INSERT INTO item VALUES (zeroblob(1048576))")
            await db.connection.commit()
        await db.connection.rollback()
        cursor = await db.connection.execute("SELECT count(*) FROM item")
        assert (await cursor.fetchone())[0] == 0
