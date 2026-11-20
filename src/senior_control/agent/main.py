"""The on-robot agent: owns the robot database and the cloud link, never an rclpy executor."""

import logging
import time
from collections.abc import Awaitable, Callable, Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from vehicle_manager.agent.connection import Link, MotionPort
from vehicle_manager.agent.local_adapters import MotionRejected
from vehicle_manager.agent.main import HALT_TYPES as HALT_TYPES
from vehicle_manager.agent.main import MAX_INFLIGHT as MAX_INFLIGHT
from vehicle_manager.agent.main import MAX_SAFETY_PENDING as MAX_SAFETY_PENDING
from vehicle_manager.agent.main import Agent as _Agent
from vehicle_manager.agent.main import _snapshot_change_key as _snapshot_change_key
from vehicle_manager.agent.runtime_ports import AgentRuntimePorts
from vehicle_manager.observability import Metrics

from senior_control.agent.dispatcher import (
    Deadlines,
    Dispatcher,
    HomePort,
    LifecycleNavigationOperations,
    MapPort,
    NavigationAction,
    OperationNotCommitted,
    TaskOperationResult,
    TaskPort,
)
from senior_control.agent.journal import AgentJournal
from senior_control.agent.snapshots import StatePort
from senior_control.agent.workspace import WorkspaceReader
from senior_control.protocol import (
    HEARTBEAT_INTERVAL_MS,
)
from senior_control.settings.service import RuntimeGate, SettingsService
from senior_control.vision.signaling import VideoSignaling
from senior_control.vision.stream import VideoSession
from vehicle_core.clock import Clock, SystemClock
from vehicle_core.diagnostics import (
    DiagnosticPending,
    PendingCounter,
    QueueSnapshot,
    process_diagnostics,
)
from vehicle_core.errors import DomainError
from vehicle_core.ipc import IpcError
from vehicle_core.values import OperationResult, OperationStatus

logger = logging.getLogger("senior_control.agent")


@dataclass(frozen=True)
class ManualOperationResult(OperationResult):
    authority: dict[str, Any]


def _report_queues(owner: object, inflight: int, stragglers: int) -> None:
    """//Compatible: original queue observations through the original diagnostic owner."""
    now = time.monotonic()
    d = process_diagnostics()
    d.queue(
        QueueSnapshot("agent.inflight", None, None, running=inflight, kind="inflight"),
        now,
        owner=owner,
    )
    d.queue(
        QueueSnapshot("agent.stragglers", None, None, running=stragglers, kind="inflight"),
        now,
        owner=owner,
    )
    d.pump(now)


class Agent(_Agent[OperationResult]):
    """//Compatible: original signature and value identities; sole loop is inherited."""

    def __init__(
        self,
        robot_id: str,
        *,
        connector: Callable[[], Awaitable[Link]],
        journal: AgentJournal,
        tasks: TaskPort,
        home: HomePort,
        maps: MapPort,
        state: StatePort,
        motion: MotionPort,
        settings: SettingsService | None = None,
        runtime_gate: RuntimeGate | None = None,
        workspace: WorkspaceReader | None = None,
        navigation: NavigationAction | None = None,
        video_session: Callable[[], VideoSession | None] | None = None,
        deadlines: Deadlines | None = None,
        map_root: Path = Path("map-transfer"),
        background: Mapping[str, Callable[[], Awaitable[None]]] | None = None,
        cleanup: Callable[[], Awaitable[None]] | None = None,
        background_fault: Callable[[str], Awaitable[None]] | None = None,
        heartbeat_ms: float = HEARTBEAT_INTERVAL_MS,
        retry_delay_ms: float = 1000,
        command_ttl_ms: float = 300,
        snapshot_seconds: float = 2.0,
        state_poll_ms: float = 250,
        metrics: Metrics | None = None,
        clock: Clock | None = None,
        wall_clock: Callable[[], float] = time.time,
    ) -> None:
        super().__init__(
            robot_id,
            connector=connector,
            journal=journal,
            tasks=tasks,
            home=home,
            maps=maps,
            state=state,
            motion=motion,
            settings=settings,
            runtime_gate=runtime_gate,
            workspace=workspace,
            navigation=navigation,
            video_session=video_session,
            deadlines=deadlines,
            map_root=map_root,
            background=background,
            cleanup=cleanup,
            background_fault=background_fault,
            heartbeat_ms=heartbeat_ms,
            retry_delay_ms=retry_delay_ms,
            command_ttl_ms=command_ttl_ms,
            snapshot_seconds=snapshot_seconds,
            state_poll_ms=state_poll_ms,
            metrics=metrics,
            clock=clock,
            wall_clock=wall_clock,
            ports=AgentRuntimePorts(
                error=DomainError,
                status=OperationStatus,
                result=OperationResult,
                manual_result=ManualOperationResult,
                manual_result_type=ManualOperationResult,
                task_result=TaskOperationResult,
                not_committed=OperationNotCommitted,
                motion_rejected=MotionRejected,
                ipc_error=IpcError,
                dispatcher=Dispatcher,
                lifecycle_navigation=LifecycleNavigationOperations,
                clock=SystemClock,
                metrics=Metrics,
                counter=PendingCounter,
                safety_pending=DiagnosticPending,
                report_queues=_report_queues,
                video_signaling=VideoSignaling,
            ),
        )


def main() -> int:
    """//Compatible: original entry, configured with the same production owners per call."""
    from vehicle_manager.agent.application import main as application_main

    from senior_control.agent.composition import AgentConfig, run_agent

    return application_main(configuration=AgentConfig.from_environment, run_agent=run_agent)


if __name__ == "__main__":
    raise SystemExit(main())
