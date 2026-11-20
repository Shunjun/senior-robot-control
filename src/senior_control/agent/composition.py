"""Configuration and composition for the robot agent."""

import asyncio
import logging
import re
import time
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
from importlib.util import find_spec
from pathlib import Path
from urllib.parse import urlsplit

from senior_mapping.workspace_export import load_exported_grid
from senior_robot.ros_video import RosCameraSource
from vehicle_manager.agent.bootstrap_ports import AgentBootstrapPorts, MediaBinding
from vehicle_manager.agent.composition import build_task_runtime as _build_task_runtime
from vehicle_manager.agent.composition import pause_failed_tasks as pause_failed_tasks
from vehicle_manager.agent.composition import run_agent as _run_agent
from vehicle_manager.agent.snapshot_state import SnapshotState as _SnapshotState
from vehicle_manager.agent.state_ports import SnapshotPorts
from vehicle_manager.media.stream import AiortcPeerFactory, VideoSession

from senior_control.agent.dispatcher import LifecycleNavigationOperations, StoreTaskPort
from senior_control.agent.journal import AgentJournal
from senior_control.agent.logs_wire import AgentLogConnection
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
from senior_control.storage.database import Database
from senior_control.tasks import TaskExecutor, TaskStore
from senior_control.tasks.lifecycle_store import Admission
from senior_control.tasks.navigation_port import NavigationEffects, TaskNavigation
from senior_control.tasks.runtime import TaskRuntime
from senior_control.vehicle_logs.service import VehicleLogService
from senior_control.vision.simulation_video import LatestFrameFile
from senior_control.waypoints.store import WaypointStore
from vehicle_core.diagnostics import (
    DiagnosticsSettings,
    configure_diagnostics,
    diagnostics_from_environment,
)
from vehicle_core.errors import DomainError
from vehicle_core.image_frame import ImageFrame
from vehicle_core.ipc import IpcClient
from vehicle_core.lifecycle_events import READ_FD, observe
from vehicle_core.runtime_state import RuntimeObservation

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
    """//Compatible: same database/stores and original concrete execution/error constructors."""
    return _build_task_runtime(
        database,
        tasks,
        navigation,
        boot_id=boot_id,
        connected=connected,
        admission=admission,
        executor_factory=TaskExecutor,
        navigation_effects=NavigationEffects,
    )


# //Compatible: the task pause policy is the same canonical function.


def _media_binding(config: AgentConfig) -> MediaBinding:
    """Original media ownership; ROS camera and peer backends remain here."""
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
            source = RosCameraSource(config.ros_video_topic, frame_factory=ImageFrame)

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

    async def close() -> None:
        if video_factory is not None:
            await asyncio.to_thread(video_factory.close)

    return MediaBinding(video_session, close)


async def run_agent(config: AgentConfig) -> int:
    """//Compatible: construct explicit original owners for this invocation only."""
    return await _run_agent(
        config,
        ports=AgentBootstrapPorts(
            configure_diagnostics=lambda: configure_diagnostics(config.diagnostics),
            database=Database,
            tasks=TaskStore,
            waypoints=WaypointStore,
            journal=AgentJournal,
            ipc=IpcClient,
            motion=TaskAwareMotion,
            state=IpcState,
            workspace=WorkspaceReader,
            home=IpcHomePort,
            maps=IpcMapPort,
            navigation=IpcNavigation,
            build_task_runtime=build_task_runtime,
            settings_runtime=SettingsRuntime,
            log_service=VehicleLogService,
            log_connection=AgentLogConnection,
            navigation_actions=LifecycleNavigationOperations,
            snapshot_state=SnapshotState,
            connector=WebSocketConnector,
            store_task_port=StoreTaskPort,
            agent=Agent,
            load_exported_grid=load_exported_grid,
            media=lambda: _media_binding(config),
            observe=observe,
            read_fd=READ_FD,
            error=DomainError,
        ),
    )
