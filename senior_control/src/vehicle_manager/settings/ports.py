"""Settings use the existing connection and lock; their owner is not imported here."""

import asyncio
from typing import Protocol

import aiosqlite


class SettingsDatabase(Protocol):
    @property
    def connection(self) -> aiosqlite.Connection: ...

    @property
    def write_lock(self) -> asyncio.Lock: ...
