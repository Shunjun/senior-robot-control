"""//Compatible: original diagnostic/Home composition over the sole SQLite engine."""

import asyncio
from collections.abc import Awaitable, Callable
from pathlib import Path

from vehicle_manager.storage.database import Database as _Database

from senior_control.models import OperationResult
from senior_control.runtime.diagnostics import DiagnosticLock


class HomeExecutionCoordination:
    """Shared same-robot connection continuity and execution serialization."""

    def __init__(self) -> None:
        self.lock = DiagnosticLock("storage.transaction_wait")
        self.connected = True
        self.disconnect_generation = 0
        self.cancel_required = False
        self.cancel_attempt_generation = 0
        self.cancel_owner_token: str | None = None
        self.cancel_owner: Callable[[str], Awaitable[OperationResult]] | None = None


class Database(_Database):
    """//Compatible: original signature and Home coordination, inherited SQLite behavior."""

    def __init__(self, path: Path, busy_timeout_ms: int = 1000) -> None:
        super().__init__(path, busy_timeout_ms, lock_factory=DiagnosticLock)
        self._home_target_locks: dict[str, asyncio.Lock] = {}
        self._home_execution: dict[str, HomeExecutionCoordination] = {}

    def home_target_lock(self, robot_id: str) -> asyncio.Lock:
        """Serialize one robot's Home target edits and acceptance across services."""
        return self._home_target_locks.setdefault(
            robot_id, DiagnosticLock("storage.home_target_wait")
        )

    def home_execution(self, robot_id: str) -> HomeExecutionCoordination:
        return self._home_execution.setdefault(robot_id, HomeExecutionCoordination())
