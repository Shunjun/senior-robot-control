"""//Compatible: original diagnostic/Home composition over the sole SQLite engine."""

from pathlib import Path

from vehicle_manager.storage.home_database import HomeDatabase as _HomeDatabase
from vehicle_manager.storage.home_database import (
    HomeExecutionCoordination as _HomeExecutionCoordination,
)

from vehicle_core.diagnostics import DiagnosticLock
from vehicle_core.values import OperationResult


class HomeExecutionCoordination(_HomeExecutionCoordination[OperationResult]):
    """//Compatible: original no-argument coordination and concrete result type."""

    def __init__(self) -> None:
        super().__init__(lock_factory=DiagnosticLock)


class Database(_HomeDatabase[HomeExecutionCoordination]):
    """//Compatible: original signature, diagnostic locks and coordination constructor."""

    def __init__(self, path: Path, busy_timeout_ms: int = 1000) -> None:
        super().__init__(
            path,
            busy_timeout_ms,
            lock_factory=DiagnosticLock,
            coordination_factory=HomeExecutionCoordination,
        )
