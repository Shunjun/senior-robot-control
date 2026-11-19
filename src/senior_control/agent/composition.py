"""Configuration and composition for the robot agent."""

import asyncio
import logging
import os
import re
import time
import uuid
from collections.abc import AsyncIterator, Callable, Mapping
from contextlib import asynccontextmanager
from dataclasses import dataclass, field, replace
from datetime import datetime, timezone
from importlib.util import find_spec
from pathlib import Path
from urllib.parse import urlsplit

from senior_mapping.workspace_export import load_exported_grid
from vehicle_manager.agent.snapshot_state import SnapshotState as _SnapshotState
from vehicle_manager.agent.state_ports import SnapshotPorts

from senior_control.agent.dispatcher import LifecycleNavigationOperations, StoreTaskPort
from senior_control.agent.journal import AgentJournal
from senior_control.agent.logs_wire import AgentLogConnection, AgentLogHandler, CloudLogGrants
from senior_control.agent.main import Agent
from senior_control.agent.manual_control import vehicle_frame
from senior_control.agent.ports import (
    IpcHomePort,
    IpcMapPort,
    IpcNavigation,
    IpcState,
    TaskAwareMotion,
)
from senior_control.agent.settings_wire import SettingsRuntime
from senior_control.agent.websocket import WebSocketConnector
from senior_control.agent.workspace import WorkspaceReader
from senior_control.errors import DomainError
from senior_control.protocol import RuntimeObservation
from senior_control.runtime.diagnostics import (
    DiagnosticsSettings,
    configure_diagnostics,
    diagnostics_from_environment,
)
from senior_control.runtime.ipc import IpcClient
from senior_control.runtime.lifecycle_events import READ_FD, observe
from senior_control.settings.models import vehicle_settings_profile
from senior_control.settings.service import RuntimeGate, SettingsService
from senior_control.settings.store import SettingsStore
from senior_control.storage.database import Database
from senior_control.tasks import ExecutionContext, TaskExecutor, TaskStore
from senior_control.tasks.lifecycle import Context
from senior_control.tasks.lifecycle_service import LifecycleService
from senior_control.tasks.lifecycle_store import Admission, ContextProvider, LifecycleStore
from senior_control.tasks.navigation_port import NavigationEffects, TaskNavigation
from senior_control.tasks.runtime import TaskRuntime
from senior_control.vehicle_logs.service import VehicleLogService
from senior_control.vehicle_logs.store import VehicleLogStore
from senior_control.vision.ros_video import RosCameraSource
from senior_control.vision.simulation_video import LatestFrameFile
from senior_control.vision.stream import AiortcPeerFactory, VideoSession
from senior_control.waypoints.store import WaypointStore

logger = logging.getLogger("senior_control.agent")


@dataclass(frozen=True)
class AgentConfig:
    robot_id: str
    cloud_url: str
    device_token: str = field(repr=False)
    database_path: Path
    socket_path: Path
    workspace_map_bundle: Path | None = None
    workspace_map_sha256: str | None = None
    simulation_video_frame: Path | None = None
    ros_video_topic: str | None = None
    # //Compatible: accepted from old simulation environments; no longer gates settings.
    simulation_settings: bool = False
    diagnostics: DiagnosticsSettings = field(default_factory=DiagnosticsSettings)

    def __post_init__(self) -> None:
        url = urlsplit(self.cloud_url)
        if not self.robot_id.strip() or not self.device_token.strip():
            raise ValueError("robot id and credential are required")
        if url.scheme != "wss" or not url.hostname or url.username or url.password:
            raise ValueError("cloud URL must use authenticated TLS")
        if not self.database_path.is_absolute() or not self.socket_path.is_absolute():
            raise ValueError("database and socket paths must be absolute")
        if (self.workspace_map_bundle is None) != (self.workspace_map_sha256 is None):
            raise ValueError("workspace map bundle and SHA-256 must be configured together")
        if self.workspace_map_bundle is not None and not self.workspace_map_bundle.is_absolute():
            raise ValueError("workspace map bundle path must be absolute")
        if self.workspace_map_sha256 is not None and not re.fullmatch(
            r"[0-9a-f]{64}", self.workspace_map_sha256
        ):
            raise ValueError("workspace map SHA-256 must be 64 lowercase hexadecimal digits")
        if (
            self.simulation_video_frame is not None
            and not self.simulation_video_frame.is_absolute()
        ):
            raise ValueError("simulation video slot must have an absolute path")

    @classmethod
    def from_environment(cls, environment: Mapping[str, str]) -> "AgentConfig":
        try:
            bundle = environment.get("SENIOR_WORKSPACE_MAP_BUNDLE")
            video = environment.get("SENIOR_SIMULATION_VIDEO_FRAME")
            return cls(
                robot_id=environment["SENIOR_ROBOT_ID"],
                cloud_url=environment["SENIOR_CLOUD_URL"],
                device_token=environment["SENIOR_DEVICE_TOKEN"],
                database_path=Path(environment["SENIOR_DATABASE_PATH"]),
                socket_path=Path(environment["SENIOR_AGENT_SOCKET"]),
                workspace_map_bundle=Path(bundle) if bundle is not None else None,
                workspace_map_sha256=environment.get("SENIOR_WORKSPACE_MAP_SHA256"),
                simulation_video_frame=Path(video) if video is not None else None,
                ros_video_topic=environment.get("SENIOR_ROS_VIDEO_TOPIC"),
                simulation_settings=environment.get("SENIOR_SIMULATION_SETTINGS") == "1",
                diagnostics=diagnostics_from_environment(environment),
            )
        except KeyError as exc:
            raise ValueError(f"missing environment variable {exc.args[0]}") from exc


@dataclass(frozen=True)
class _ControlViewer:
    robot_id: str
    role: str = "control"


# //Compatible: original construction, canonical safety-frontier/display owner.


class SnapshotState(_SnapshotState):
    def __init__(
        self,
        state: IpcState,
        tasks: TaskStore,
        waypoints: WaypointStore,
        workspace: WorkspaceReader | None = None,
        *,
        navigation: LifecycleNavigationOperations | None = None,
        connected: Callable[[], bool] = lambda: True,
        epoch: Callable[[], int] = lambda: 0,
        manual_enabled: bool = False,
        clock: Callable[[], float] = time.monotonic,
        settings_enabled: bool = False,
        logs_enabled: Callable[[], bool] = lambda: False,
    ) -> None:
        super().__init__(
            state,
            tasks,
            waypoints,
            workspace,
            ports=SnapshotPorts(vehicle_frame, RuntimeObservation, DomainError),
            navigation=navigation,
            connected=connected,
            epoch=epoch,
            manual_enabled=manual_enabled,
            clock=clock,
            settings_enabled=settings_enabled,
            logs_enabled=logs_enabled,
        )


def build_task_runtime(
    database: Database,
    tasks: TaskStore,
    navigation: TaskNavigation,
    *,
    boot_id: str,
    connected: Callable[[], bool] | None = None,
    admission: Admission | None = None,
) -> tuple[TaskRuntime, TaskExecutor]:
    """Assembly only; all transitions and effects stay in the task driver/ports."""
    durable = LifecycleStore(tasks, boot_id=boot_id, admission=admission)
    effects = NavigationEffects(durable, navigation)

    def current() -> Context:
        value = executor.current_context()
        return replace(value, connected=connected()) if connected is not None else value

    service = LifecycleService(durable, effects, context=current)
    executor = TaskExecutor(database, tasks, None, lifecycle=service)
    return TaskRuntime(tasks, executor), executor


async def pause_failed_tasks(executor: TaskExecutor) -> bool:
    """Use the existing durable owner; absence of a revoke proof remains pending."""
    retired = await executor.control_current("pause", "runtime-tasks-failed")
    if retired is None:
        return True
    assert executor.lifecycle is not None
    current = await executor.lifecycle.store.load(retired[0])
    return current.phase in {"paused", "succeeded", "failed", "canceled"}


async def run_agent(config: AgentConfig) -> int:
    configure_diagnostics(config.diagnostics)
    async with Database(config.database_path) as database:
        tasks = TaskStore(database, robot_id=config.robot_id)
        waypoints = WaypointStore(database, robot_id=config.robot_id)
        journal = AgentJournal(database, robot_id=config.robot_id)
        await tasks.prepare()
        await waypoints.prepare()
        try:
            await journal.prepare()
        except Exception:
            logger.exception("could not initialize agent operation journal")
            return 78
        domain_value = os.environ.get("SENIOR_EXPECTED_ROS_DOMAIN")
        domain = int(domain_value) if domain_value is not None else None

        def ipc(*, timeout_ms: float = 1000.0) -> IpcClient:
            return IpcClient(
                config.socket_path,
                timeout_ms=timeout_ms,
                robot_id=config.robot_id,
                ros_domain=domain,
                owner_record=os.environ.get("SENIOR_VERIFIED_ADAPTER_RUN"),
            )

        motion = TaskAwareMotion(
            ipc(),
            intent_client=ipc(),
            disable_client=ipc(),
            safety_client=ipc(),
        )
        state_client = ipc()
        map_client = ipc()
        navigation_client = ipc(timeout_ms=3000.0)
        workspace_client = ipc()
        display_client = ipc()
        state = IpcState(state_client)
        exported_grid = None
        if config.workspace_map_bundle is not None and config.workspace_map_sha256 is not None:
            try:
                exported_grid = await asyncio.to_thread(
                    load_exported_grid,
                    config.workspace_map_bundle,
                    config.workspace_map_sha256,
                    config.robot_id,
                )
            except (DomainError, OSError):
                logger.warning("configured workspace display map is unavailable", exc_info=True)
        workspace = WorkspaceReader(
            config.robot_id, tasks, waypoints, workspace_client, exported_grid=exported_grid
        )
        display = WorkspaceReader(
            config.robot_id, tasks, waypoints, display_client, exported_grid=exported_grid
        )
        home = IpcHomePort(state_client, waypoints, robot_id=config.robot_id)
        agent: Agent
        navigation = IpcNavigation(
            navigation_client,
            waypoints,
            tasks=tasks,
        )
        context = ExecutionContext(False, False, False, False, None)

        settings_runtime: SettingsRuntime
        gate = RuntimeGate(
            lambda: settings_runtime.check(), refresh=lambda: settings_runtime.refresh()
        )

        @asynccontextmanager
        async def task_commit() -> AsyncIterator[ContextProvider]:
            while True:
                generation = gate.generation
                # Sample outside the gate; failed reads never reuse an earlier context.
                try:
                    sampled = await state.read()
                except DomainError:
                    # Withdrawal/recovery still commits; no new motion may use an old sample.
                    local = ExecutionContext(False, False, False, False, None)
                else:
                    local = sampled.for_tasks(agent.connected)
                    if "tasks" in agent.failed_modules:
                        local = replace(local, admission_ready=False)
                    authority = sampled.manual_authority
                    local = replace(
                        local,
                        manual_driving=local.manual_driving
                        or bool(authority and authority.get("state") in {"prepared", "active"}),
                    )
                async with gate.lock:
                    if gate.generation != generation:
                        continue
                    gate.invalidate()
                    admitted_generation = gate.generation

                    # This provider rechecks the original sample's expiry inside
                    # the SQLite transaction, even if it waited for a DB writer.
                    def current(
                        local: ExecutionContext = local, generation: int = admitted_generation
                    ) -> Context:
                        if gate.generation != generation:
                            return Context(agent.connected, False)
                        return replace(executor.current_context(local), connected=agent.connected)

                    yield current
                    return

        runtime, executor = build_task_runtime(
            database,
            tasks,
            navigation,
            boot_id=uuid.uuid4().hex,
            connected=lambda: agent.connected,
            admission=task_commit,
        )
        motion.bind_tasks(executor)
        settings = None
        log_store = None
        logs = None
        log_connection = None
        log_handler = None
        previous_log_level = logger.level
        assert executor.lifecycle is not None
        settings_runtime = SettingsRuntime(state, executor.lifecycle.store)
        settings = SettingsService(SettingsStore(database), vehicle_settings_profile(), gate)
        await settings.prepare()
        settings.start_debug_restore()
        candidate = VehicleLogStore(config.database_path.parent / "logs")
        try:
            await candidate.__aenter__()
        except (OSError, ValueError):
            logger.warning("vehicle log storage unavailable; control remains independent")
        else:
            log_store = candidate
            grants = CloudLogGrants()
            logs = VehicleLogService(
                log_store, config.robot_id, grants, lambda: settings.published_snapshot
            )
            await logs.start()
            log_connection = AgentLogConnection(logs, grants)
            log_handler = AgentLogHandler(logs)
            logging.getLogger("senior_control").addHandler(log_handler)
            logger.setLevel(logging.DEBUG)
            logger.info("agent_log_source_started", extra={"event": "agent_log_source_started"})
        motion.settings = settings
        navigation_actions = LifecycleNavigationOperations(executor, state, workspace, waypoints)

        async def task_loop() -> None:
            nonlocal context
            while True:
                try:
                    live = await state.read()
                except DomainError:
                    context = ExecutionContext(False, False, False, False, None)
                else:
                    context = live.for_tasks(agent.connected)
                await runtime.tick(datetime.now(timezone.utc), time.monotonic(), context)
                await asyncio.sleep(0.2)

        video_factory: AiortcPeerFactory | None = None

        def video_session() -> VideoSession | None:
            nonlocal video_factory
            if config.ros_video_topic is None and config.simulation_video_frame is None:
                return None
            if find_spec("aiortc") is None:
                logger.warning("video disabled because the video extra is unavailable")
                return None
            if video_factory is None:
                video_factory = AiortcPeerFactory()
            if config.ros_video_topic is not None:
                source = RosCameraSource(config.ros_video_topic)

                class RosVideoSession(VideoSession):
                    async def close(self) -> None:
                        try:
                            await super().close()
                        finally:
                            await source.close()

                return RosVideoSession(
                    robot_id=config.robot_id,
                    viewer=_ControlViewer(config.robot_id),
                    source=source,
                    factory=video_factory,
                )
            assert config.simulation_video_frame is not None
            return VideoSession(
                robot_id=config.robot_id,
                viewer=_ControlViewer(config.robot_id),
                source=LatestFrameFile(config.simulation_video_frame),
                factory=video_factory,
            )

        async def cleanup() -> None:
            await asyncio.gather(
                state_client.close(),
                map_client.close(),
                navigation_client.close(),
                workspace_client.close(),
                display_client.close(),
            )
            if log_handler is not None:
                logging.getLogger("senior_control").removeHandler(log_handler)
                logger.setLevel(previous_log_level)
            try:
                if logs is not None:
                    await logs.close()
            finally:
                try:
                    if log_store is not None:
                        await log_store.__aexit__(None, None, None)
                finally:
                    if settings is not None:
                        await settings.close()
                    if video_factory is not None:
                        await asyncio.to_thread(video_factory.close)

        async def background_fault(name: str) -> None:
            # Stop the failed task driver, persist pause/revoke through its sole owner.
            gate.invalidate()
            executor.context = replace(context, admission_ready=False)
            if not await pause_failed_tasks(executor):
                agent.report_runtime_event(
                    {
                        "module": "tasks",
                        "phase": "unconfirmed",
                        "detail": "task pause/revocation is pending",
                    }
                )

        event_fd = os.environ.get(READ_FD)

        async def lifecycle_loop() -> None:
            assert event_fd is not None
            await observe(int(event_fd), agent.report_runtime_event)
            # EOF is already recorded by observe; no global shutdown or restart.

        async def logs_loop() -> None:
            if log_connection is not None:
                await log_connection.run(config.cloud_url, config.device_token)

        agent = Agent(
            config.robot_id,
            connector=WebSocketConnector(
                config.cloud_url,
                config.device_token,
                config.robot_id,
            ),
            journal=journal,
            tasks=StoreTaskPort(
                tasks, robot_id=config.robot_id, verify_workspace=workspace.verify_task_edit
            ),
            home=home,
            maps=IpcMapPort(map_client),
            state=SnapshotState(
                state,
                tasks,
                waypoints,
                display,
                navigation=navigation_actions,
                connected=lambda: agent.connected,
                epoch=lambda: agent.connection_epoch,
                manual_enabled=settings is not None,
                settings_enabled=settings is not None,
                logs_enabled=lambda: log_connection is not None and log_connection.connected,
            ),
            settings=settings,
            runtime_gate=gate,
            workspace=workspace,
            navigation=navigation_actions,
            motion=motion,
            video_session=video_session,
            map_root=config.database_path.parent / "map-transfer",
            background={
                "tasks": task_loop,
                **({"logs": logs_loop} if logs is not None else {}),
                **({"observer": lifecycle_loop} if event_fd is not None else {}),
            },
            background_fault=background_fault,
            cleanup=cleanup,
            # Accepted tasks are vehicle-owned; only manual authority follows eligibility.
        )
        agent.install_signal_handlers()
        await agent.run()
        if agent.auth_rejected or agent.runtime_failed:
            return 78
        return 1 if agent.runtime_failed else 0
