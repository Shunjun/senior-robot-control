"""Owned asynchronous SQLite connection with serialized writes."""

import asyncio
from pathlib import Path
from types import TracebackType

import aiosqlite


class Database:
    def __init__(self, path: Path, busy_timeout_ms: int = 1000) -> None:
        self.path = path
        self.busy_timeout_ms = busy_timeout_ms
        self.write_lock = asyncio.Lock()
        self.connection: aiosqlite.Connection

    async def __aenter__(self) -> "Database":
        self.connection = await aiosqlite.connect(self.path)
        try:
            await self.connection.execute("PRAGMA foreign_keys = ON")
            await self.connection.execute("PRAGMA journal_mode = WAL")
            await self.connection.execute("PRAGMA synchronous = FULL")
            await self.connection.execute(f"PRAGMA busy_timeout = {int(self.busy_timeout_ms)}")
        except BaseException:
            await self.connection.close()
            raise
        return self

    async def __aexit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        tb: TracebackType | None,
    ) -> None:
        await self.connection.close()
