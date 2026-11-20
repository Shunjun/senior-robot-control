"""//Compatible: original journal values and construction over the sole manager SQL owner."""

from dataclasses import dataclass

from vehicle_manager.agent.journal import AgentJournal as _AgentJournal
from vehicle_manager.agent.journal import _transaction as _transaction
from vehicle_manager.agent.ports import JournalPorts
from vehicle_manager.storage.agent_schema import (
    AGENT_JOURNAL_STATEMENTS as AGENT_JOURNAL_STATEMENTS,
)

from senior_control.storage.database import Database
from vehicle_core.errors import ErrorCode
from vehicle_core.values import OperationStatus

_OPEN = (OperationStatus.ACCEPTED, OperationStatus.DISPATCHING, OperationStatus.RUNNING)


@dataclass(frozen=True)
class JournalEntry:
    operation_id: str
    fingerprint: str
    status: OperationStatus
    error: ErrorCode | None
    revision: int | None

    @property
    def closed(self) -> bool:
        return self.status not in _OPEN


class AgentJournal(_AgentJournal[JournalEntry, OperationStatus]):
    """//Compatible: original constructor/values; inherited SQL/transaction implementation."""

    def __init__(self, database: Database, *, robot_id: str) -> None:
        super().__init__(
            database,
            robot_id=robot_id,
            ports=JournalPorts(
                entry=JournalEntry,
                status=OperationStatus,
                active=_OPEN,
                running=OperationStatus.RUNNING,
                needs_review=OperationStatus.NEEDS_REVIEW,
            ),
        )
