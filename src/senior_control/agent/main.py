"""//Compatible: original Agent names and unchanged CLI entry over canonical runtime bindings."""

from vehicle_manager.agent.main import HALT_TYPES as HALT_TYPES
from vehicle_manager.agent.main import MAX_INFLIGHT as MAX_INFLIGHT
from vehicle_manager.agent.main import MAX_SAFETY_PENDING as MAX_SAFETY_PENDING
from vehicle_manager.agent.main import _snapshot_change_key as _snapshot_change_key
from vehicle_manager.agent.runtime_bindings import HEARTBEAT_INTERVAL_MS as HEARTBEAT_INTERVAL_MS
from vehicle_manager.agent.runtime_bindings import Agent as Agent
from vehicle_manager.agent.runtime_bindings import AgentJournal as AgentJournal
from vehicle_manager.agent.runtime_bindings import AgentRuntimePorts as AgentRuntimePorts
from vehicle_manager.agent.runtime_bindings import Clock as Clock
from vehicle_manager.agent.runtime_bindings import Deadlines as Deadlines
from vehicle_manager.agent.runtime_bindings import DiagnosticPending as DiagnosticPending
from vehicle_manager.agent.runtime_bindings import Dispatcher as Dispatcher
from vehicle_manager.agent.runtime_bindings import DomainError as DomainError
from vehicle_manager.agent.runtime_bindings import HomePort as HomePort
from vehicle_manager.agent.runtime_bindings import IpcError as IpcError
from vehicle_manager.agent.runtime_bindings import (
    LifecycleNavigationOperations as LifecycleNavigationOperations,
)
from vehicle_manager.agent.runtime_bindings import Link as Link
from vehicle_manager.agent.runtime_bindings import ManualOperationResult as ManualOperationResult
from vehicle_manager.agent.runtime_bindings import MapPort as MapPort
from vehicle_manager.agent.runtime_bindings import Metrics as Metrics
from vehicle_manager.agent.runtime_bindings import MotionPort as MotionPort
from vehicle_manager.agent.runtime_bindings import MotionRejected as MotionRejected
from vehicle_manager.agent.runtime_bindings import NavigationAction as NavigationAction
from vehicle_manager.agent.runtime_bindings import OperationNotCommitted as OperationNotCommitted
from vehicle_manager.agent.runtime_bindings import OperationResult as OperationResult
from vehicle_manager.agent.runtime_bindings import OperationStatus as OperationStatus
from vehicle_manager.agent.runtime_bindings import PendingCounter as PendingCounter
from vehicle_manager.agent.runtime_bindings import QueueSnapshot as QueueSnapshot
from vehicle_manager.agent.runtime_bindings import RuntimeGate as RuntimeGate
from vehicle_manager.agent.runtime_bindings import SettingsService as SettingsService
from vehicle_manager.agent.runtime_bindings import StatePort as StatePort
from vehicle_manager.agent.runtime_bindings import SystemClock as SystemClock
from vehicle_manager.agent.runtime_bindings import TaskOperationResult as TaskOperationResult
from vehicle_manager.agent.runtime_bindings import TaskPort as TaskPort
from vehicle_manager.agent.runtime_bindings import VideoSession as VideoSession
from vehicle_manager.agent.runtime_bindings import VideoSignaling as VideoSignaling
from vehicle_manager.agent.runtime_bindings import WorkspaceReader as WorkspaceReader
from vehicle_manager.agent.runtime_bindings import _report_queues as _report_queues
from vehicle_manager.agent.runtime_bindings import logger as logger
from vehicle_manager.agent.runtime_bindings import process_diagnostics as process_diagnostics


def main() -> int:
    """//Compatible: original entry, configured with the same production owners per call."""
    from vehicle_manager.agent.__main__ import main as canonical_main

    return canonical_main()


if __name__ == "__main__":
    raise SystemExit(main())
