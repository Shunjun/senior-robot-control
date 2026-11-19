"""The on-robot agent: owns the robot database and the cloud link, never an rclpy executor."""

import argparse
import asyncio
import logging
import os
import sys
import time
from collections.abc import Awaitable, Callable, Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from vehicle_manager.agent.main import HALT_TYPES as HALT_TYPES
from vehicle_manager.agent.main import MAX_INFLIGHT as MAX_INFLIGHT
from vehicle_manager.agent.main import MAX_SAFETY_PENDING as MAX_SAFETY_PENDING
from vehicle_manager.agent.main import Agent as _Agent
from vehicle_manager.agent.main import _snapshot_change_key as _snapshot_change_key
from vehicle_manager.agent.runtime_ports import AgentRuntimePorts

from senior_control.agent.connection import (
    Link,
    MotionPort,
    MotionRejected,
)
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
from senior_control.clock import Clock, SystemClock
from senior_control.errors import DomainError
from senior_control.models import OperationResult, OperationStatus
from senior_control.observability import Metrics
from senior_control.protocol import (
    HEARTBEAT_INTERVAL_MS,
)
from senior_control.runtime.diagnostics import (
    DiagnosticPending,
    PendingCounter,
    QueueSnapshot,
    process_diagnostics,
)
from senior_control.runtime.ipc import IpcError
from senior_control.settings.service import RuntimeGate, SettingsService
from senior_control.vision.signaling import VideoSignaling
from senior_control.vision.stream import VideoSession

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
    """Load explicit configuration and run the owned agent composition."""
    from senior_control.agent.composition import AgentConfig, run_agent

    parser = argparse.ArgumentParser(description="Robot cloud agent")
    parser.add_argument("--robot-id")
    parser.add_argument("--cloud-url")
    parser.add_argument("--database-path")
    parser.add_argument("--socket-path")
    parser.add_argument("--token-file", type=Path)
    parser.add_argument("--credentials-file", type=Path)
    parser.add_argument("--pair", action="store_true")
    parser.add_argument("--device-name")
    args = parser.parse_args()
    values = dict(os.environ)
    for name, value in (
        ("SENIOR_ROBOT_ID", args.robot_id),
        ("SENIOR_CLOUD_URL", args.cloud_url),
        ("SENIOR_DATABASE_PATH", args.database_path),
        ("SENIOR_AGENT_SOCKET", args.socket_path),
    ):
        if value is not None:
            values[name] = value
    try:
        from senior_control.agent.pairing import pair_device, read_credentials, read_token

        credentials_file = args.credentials_file
        if credentials_file is None and values.get("SENIOR_CREDENTIALS_FILE"):
            credentials_file = Path(values["SENIOR_CREDENTIALS_FILE"])
        if args.pair:
            import getpass

            cloud_url = values.get("SENIOR_CLOUD_URL")
            if not cloud_url or credentials_file is None or not args.device_name:
                raise ValueError("pair requires cloud URL, device name and credentials file")
            code = getpass.getpass("Pairing code: ")
            robot_id = asyncio.run(pair_device(cloud_url, code, args.device_name, credentials_file))
            print(f"Paired robot {robot_id}")
            return 0
        if credentials_file is not None:
            credentials = read_credentials(credentials_file)
            values["SENIOR_ROBOT_ID"] = credentials.robot_id
            values["SENIOR_DEVICE_TOKEN"] = credentials.device_token
        if args.token_file is not None:
            values["SENIOR_DEVICE_TOKEN"] = read_token(args.token_file)
        config = AgentConfig.from_environment(values)
    except (OSError, ValueError) as exc:
        print(f"invalid agent configuration: {exc}", file=sys.stderr)
        return 78
    from senior_control.observability import configure_logging

    configure_logging()
    return asyncio.run(run_agent(config))


if __name__ == "__main__":
    raise SystemExit(main())
