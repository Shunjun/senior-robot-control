"""Concrete Agent construction and its original diagnostic/media bindings; no execution entry."""

import logging
import time
from collections.abc import Awaitable, Callable, Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from senior_contracts.wire import HEARTBEAT_INTERVAL_MS, VideoEnvelope

from vehicle_core.clock import Clock, SystemClock
from vehicle_core.diagnostics import (
    DiagnosticAsyncQueue,
    DiagnosticPending,
    PendingCounter,
    QueueSnapshot,
    process_diagnostics,
)
from vehicle_core.errors import DomainError
from vehicle_core.ipc import IpcError
from vehicle_core.values import OperationResult, OperationStatus
from vehicle_manager.agent.connection import Link, MotionPort
from vehicle_manager.agent.local_adapters import MotionRejected
from vehicle_manager.agent.main import Agent as _Agent
from vehicle_manager.agent.production_bindings import AgentJournal
from vehicle_manager.agent.request_bindings import (
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
from vehicle_manager.agent.runtime_ports import AgentRuntimePorts
from vehicle_manager.agent.snapshots import StatePort
from vehicle_manager.agent.workspace import WorkspaceReader
from vehicle_manager.media.signaling import VideoSignaling as _VideoSignaling
from vehicle_manager.media.stream import VideoSession
from vehicle_manager.observability import Metrics
from vehicle_manager.settings.service import RuntimeGate, SettingsService

logger = logging.getLogger("senior_control.agent")


class VideoSignaling(_VideoSignaling):
    """//Compatible: sole inherited management loop, original per-instance queue binding."""

    def __init__(
        self,
        robot_id: str,
        epoch: int,
        *,
        send: Callable[[VideoEnvelope], Awaitable[None]],
        make_session: Callable[[], VideoSession | None],
    ) -> None:
        super().__init__(
            robot_id,
            epoch,
            send=send,
            make_session=make_session,
            queue_factory=DiagnosticAsyncQueue,
        )


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
