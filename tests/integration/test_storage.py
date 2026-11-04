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


@pytest.mark.parametrize("timeout", [-1, 0, 60001])
def test_busy_timeout_must_be_bounded(tmp_path, timeout):
    with pytest.raises(ValueError):
        Database(tmp_path / "x.db", busy_timeout_ms=timeout)


@pytest.mark.asyncio
async def test_database_pragmas_and_backup_snapshot(tmp_path):
    path = tmp_path / "snapshot.db"
    async with Database(path) as db:
        for name, expected in (("foreign_keys", 1), ("synchronous", 2), ("journal_mode", "wal")):
            async with db.connection.execute(f"PRAGMA {name}") as cursor:
                assert (await cursor.fetchone())[0] == expected
        await migrate(
            db,
            [
                Migration(
                    1, ("CREATE TABLE item (value TEXT)", "INSERT INTO item VALUES ('before')")
                )
            ],
        )
        await migrate(db, [Migration(1, ()), Migration(2, ("INSERT INTO item VALUES ('after')",))])
    with sqlite3.connect(str(path) + ".backup") as snapshot:
        assert snapshot.execute("SELECT value FROM item").fetchall() == [("before",)]
        assert snapshot.execute("PRAGMA user_version").fetchone()[0] == 1


@pytest.mark.asyncio
async def test_locked_wal_initialization_uses_requested_busy_timeout(tmp_path):
    import asyncio

    path = tmp_path / "locked.db"
    with sqlite3.connect(path) as blocker:
        blocker.execute("CREATE TABLE x (v INTEGER)")
        blocker.execute("BEGIN EXCLUSIVE")
        with pytest.raises(sqlite3.OperationalError):
            await asyncio.wait_for(Database(path, busy_timeout_ms=50).__aenter__(), 1)
        blocker.rollback()


@pytest.mark.asyncio
async def test_cancelled_initialization_releases_connection(tmp_path):
    import asyncio

    path = tmp_path / "cancel.db"
    with sqlite3.connect(path) as blocker:
        blocker.execute("CREATE TABLE x (v INTEGER)")
        blocker.execute("BEGIN EXCLUSIVE")
        task = asyncio.create_task(Database(path, busy_timeout_ms=100).__aenter__())
        await asyncio.sleep(0.02)
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task
        blocker.rollback()
    async with Database(path, busy_timeout_ms=100) as db:
        async with db.connection.execute("SELECT count(*) FROM x") as cursor:
            assert (await cursor.fetchone())[0] == 0
