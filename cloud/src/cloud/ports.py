"""Technical inputs supplied by the cloud or explicit legacy bootstrap.

These interfaces carry no implementation, process state or vehicle policy.
"""

import asyncio
from collections.abc import Awaitable, Callable, Iterator
from types import TracebackType
from typing import Any, Protocol

import aiosqlite
from pydantic import SecretStr


class Settings(Protocol):
    @property
    def secret(self) -> SecretStr: ...


class Database(Protocol):
    @property
    def connection(self) -> aiosqlite.Connection: ...

    @property
    def write_lock(self) -> asyncio.Lock: ...

    @property
    def is_open(self) -> bool: ...

    async def __aenter__(self) -> "Database": ...

    async def __aexit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        tb: TracebackType | None,
    ) -> None: ...


class FrameQueue(Protocol):
    """Original diagnostic deque operations used by the bounded cloud outbox."""

    def __len__(self) -> int: ...
    def __iter__(self) -> Iterator[tuple[str, bytes]]: ...
    def __getitem__(self, index: int) -> tuple[str, bytes]: ...
    def append(self, value: tuple[str, bytes]) -> None: ...
    def insert(self, index: int, value: tuple[str, bytes]) -> None: ...
    def popleft(self) -> tuple[str, bytes]: ...
    def discard_index(self, index: int, *, overwritten: bool = False) -> None: ...
    def reject(self, count: int = 1) -> None: ...


DequeFactory = Callable[[str, int], FrameQueue]
LogQueueFactory = Callable[[str, int], asyncio.Queue[dict[str, Any]]]
PrepareDatabase = Callable[[], Awaitable[None]]
ConfigureDiagnostics = Callable[[], object]
