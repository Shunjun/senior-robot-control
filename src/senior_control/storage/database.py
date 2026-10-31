"""Owned asynchronous SQLite connection with serialized writes."""

import asyncio
from pathlib import Path
from types import TracebackType

import aiosqlite


class Database:
    def __init__(self, path: Path, busy_timeout_ms: int = 1000) -> None:
        if isinstance(busy_timeout_ms, bool) or not 1 <= busy_timeout_ms <= 60000:
            raise ValueError("busy timeout must be between 1 and 60000 milliseconds")
        self.path = path
        self.busy_timeout_ms = busy_timeout_ms
        self.write_lock = asyncio.Lock()
        self.connection: aiosqlite.Connection
        self._entered = False

    @property
    def is_open(self) -> bool:
        return self._entered

    async def __aenter__(self) -> "Database":
        if self._entered:
            raise RuntimeError("database already open")
        self._entered = True
        try:
            return await self._open()
        except BaseException:
            self._entered = False
            raise

    async def _open(self) -> "Database":
        connect = asyncio.ensure_future(
            aiosqlite.connect(self.path, timeout=self.busy_timeout_ms / 1000)
        )
        try:
            self.connection = await asyncio.shield(connect)
        except asyncio.CancelledError:
            connection = await connect
            await connection.close()
            raise
        try:
            for statement in (
                f"PRAGMA busy_timeout = {self.busy_timeout_ms}",
                "PRAGMA foreign_keys = ON",
                "PRAGMA journal_mode = WAL",
                "PRAGMA synchronous = FULL",
            ):
                async with self.connection.execute(statement):
                    pass
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
        try:
            await self.connection.close()
        finally:
            self._entered = False
