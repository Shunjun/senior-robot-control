"""Small helpers over the shared cloud database connection."""

from collections.abc import AsyncIterator, Sequence
from contextlib import asynccontextmanager
from typing import Any

import aiosqlite

from senior_control.storage.database import Database


@asynccontextmanager
async def write_transaction(database: Database) -> AsyncIterator[aiosqlite.Connection]:
    """Serialize writers, commit on success, roll back on any exit with an error."""
    async with database.write_lock:
        connection = database.connection
        async with connection.execute("BEGIN IMMEDIATE"):
            pass
        try:
            yield connection
            await connection.commit()
        except BaseException:
            await connection.rollback()
            raise


async def fetch_all(
    database: Database, sql: str, params: Sequence[Any] = ()
) -> list[tuple[Any, ...]]:
    async with database.connection.execute(sql, params) as cursor:
        rows = await cursor.fetchall()
    return [tuple(row) for row in rows]
