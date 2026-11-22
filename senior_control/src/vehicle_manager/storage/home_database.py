"""Per-database Home coordination over the existing SQLite connection owner."""

import asyncio
from collections.abc import Awaitable, Callable
from pathlib import Path
from typing import Any, Generic, TypeVar

from vehicle_manager.storage.database import Database
from vehicle_manager.waypoints.home_ports import OperationT


class HomeExecutionCoordination(Generic[OperationT]):
    """Shared same-robot connection continuity and execution serialization."""

    def __init__(self, *, lock_factory: Callable[[str], asyncio.Lock]) -> None:
        self.lock = lock_factory("storage.transaction_wait")
        self.connected = True
        self.disconnect_generation = 0
        self.cancel_required = False
        self.cancel_attempt_generation = 0
        self.cancel_owner_token: str | None = None
        self.cancel_owner: Callable[[str], Awaitable[OperationT]] | None = None


CoordinationT = TypeVar("CoordinationT", bound=HomeExecutionCoordination[Any])


class HomeDatabase(Database, Generic[CoordinationT]):
    """Keep one robot's locks and coordination on this connection owner."""

    def __init__(
        self,
        path: Path,
        busy_timeout_ms: int = 1000,
        *,
        lock_factory: Callable[[str], asyncio.Lock],
        coordination_factory: Callable[[], CoordinationT],
    ) -> None:
        super().__init__(path, busy_timeout_ms, lock_factory=lock_factory)
        self._home_lock_factory = lock_factory
        self._home_coordination_factory = coordination_factory
        self._home_target_locks: dict[str, asyncio.Lock] = {}
        self._home_execution: dict[str, CoordinationT] = {}

    def home_target_lock(self, robot_id: str) -> asyncio.Lock:
        """Serialize one robot's Home target edits and acceptance across services."""
        return self._home_target_locks.setdefault(
            robot_id, self._home_lock_factory("storage.home_target_wait")
        )

    def home_execution(self, robot_id: str) -> CoordinationT:
        return self._home_execution.setdefault(robot_id, self._home_coordination_factory())
