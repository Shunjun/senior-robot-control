"""Concrete Agent bindings over the sole management implementations and core diagnostics."""

import time
from collections.abc import Callable
from dataclasses import dataclass
from typing import TYPE_CHECKING

from vehicle_core.diagnostics import (
    DiagnosticAsyncQueue,
    DiagnosticLock,
    PendingCounter,
    QueueSnapshot,
    process_diagnostics,
)
from vehicle_core.errors import DomainError, ErrorCode
from vehicle_core.values import OperationStatus
from vehicle_manager.agent.connection import HeartbeatFrame
from vehicle_manager.agent.journal import AgentJournal as _AgentJournal
from vehicle_manager.agent.local_adapters import IpcState
from vehicle_manager.agent.logs_wire import AgentLogConnection as _AgentLogConnection
from vehicle_manager.agent.logs_wire import CloudLogGrants
from vehicle_manager.agent.ports import JournalPorts, WebSocketPorts
from vehicle_manager.agent.settings_wire import SettingsRuntime as _SettingsRuntime
from vehicle_manager.agent.snapshots import RobotSnapshot, snapshot_envelope
from vehicle_manager.agent.websocket import Socket
from vehicle_manager.agent.websocket import WebSocketConnector as _WebSocketConnector
from vehicle_manager.agent.websocket import WebSocketLink as _WebSocketLink
from vehicle_manager.settings.models import SettingsSnapshot
from vehicle_manager.storage.bindings import Database
from vehicle_manager.vehicle_logs.diagnostics import LogDiagnostics
from vehicle_manager.vehicle_logs.service import LogAuthorizer
from vehicle_manager.vehicle_logs.service import VehicleLogService as _VehicleLogService
from vehicle_manager.vehicle_logs.store import VehicleLogStore

if TYPE_CHECKING:
    from vehicle_manager.tasks.lifecycle_store import LifecycleStore

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


class VehicleLogService(_VehicleLogService):
    """//Compatible: original constructor; all log business methods are canonical."""

    def __init__(
        self,
        store: VehicleLogStore,
        vehicle_id: str,
        authorizer: LogAuthorizer,
        settings: Callable[[], SettingsSnapshot],
        *,
        utc_now: Callable[[], float] = time.time,
    ) -> None:
        super().__init__(
            store,
            vehicle_id,
            authorizer,
            settings,
            utc_now=utc_now,
            diagnostics=LogDiagnostics(
                counter=PendingCounter,
                lock=DiagnosticLock,
                outbox=DiagnosticAsyncQueue,
                bind_async=lambda: process_diagnostics().bind_async(),
            ),
        )


class AgentLogConnection(_AgentLogConnection):
    """//Compatible: original constructor; no duplicate diagnostic state."""

    def __init__(self, service: VehicleLogService, grants: CloudLogGrants) -> None:
        super().__init__(
            service, grants, lock_factory=DiagnosticLock, counter_factory=PendingCounter
        )


class SettingsRuntime(_SettingsRuntime):
    """//Compatible: original signature; runtime evidence remains a single instance."""

    def __init__(self, state: IpcState, tasks: "LifecycleStore") -> None:
        super().__init__(state, tasks, error=DomainError)


def _receive_diagnostic(owner: object) -> None:
    process_diagnostics().queue(
        QueueSnapshot("websocket.receive", None, 16, depth_basis="library_depth_unavailable"),
        time.monotonic(),
        owner=owner,
    )


class WebSocketLink(_WebSocketLink[RobotSnapshot, HeartbeatFrame]):
    """//Compatible: original signature; sole heartbeat/transport state is inherited."""

    def __init__(
        self,
        socket: Socket,
        robot_id: str,
        epoch: int,
        *,
        wall_clock: Callable[[], float] = time.time,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        super().__init__(
            socket,
            robot_id,
            epoch,
            wall_clock=wall_clock,
            clock=clock,
            ports=WebSocketPorts(
                heartbeat=HeartbeatFrame,
                snapshot=snapshot_envelope,
                send_lock=lambda: DiagnosticLock("websocket.send_wait"),
                receive_diagnostic=_receive_diagnostic,
            ),
        )


class WebSocketConnector(_WebSocketConnector[WebSocketLink]):
    """//Compatible: original signature and original reporting-aware link constructor."""

    def __init__(self, url: str, token: str, robot_id: str) -> None:
        super().__init__(url, token, robot_id, link_factory=WebSocketLink)
