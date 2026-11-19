from pathlib import Path

import pytest

from senior_control.storage.database import Database


async def test_database_rejects_reentry_without_losing_original_connection(tmp_path: Path) -> None:
    database = Database(tmp_path / "robot.db")
    async with database:
        original = database.connection
        with pytest.raises(RuntimeError, match="already open"):
            await database.__aenter__()
        assert database.connection is original
        async with original.execute("SELECT 1") as cursor:
            assert await cursor.fetchone() == (1,)
    with pytest.raises(ValueError):
        await original.execute("SELECT 1")


async def test_database_open_state_tracks_reentry(tmp_path: Path) -> None:
    database = Database(tmp_path / "robot.db")
    assert not database.is_open
    async with database:
        assert database.is_open
    assert not database.is_open
    async with database:
        assert database.is_open
