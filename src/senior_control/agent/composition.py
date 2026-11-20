"""Configuration and composition for the robot agent."""

import asyncio
import logging
import time
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
from importlib.util import find_spec
from typing import Any

from senior_mapping.workspace_export import load_exported_grid
from senior_robot.ros_video import RosCameraSource
from vehicle_manager.agent.bootstrap_ports import AgentBootstrapPorts, MediaBinding
from vehicle_manager.agent.composition import build_task_runtime as _build_task_runtime
from vehicle_manager.agent.composition import pause_failed_tasks as pause_failed_tasks
from vehicle_manager.agent.composition import run_agent as _run_agent
from vehicle_manager.agent.config import AgentConfig as _AgentConfig
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
class AgentConfig(_AgentConfig):
    """//Compatible: original constructor with its sole concrete diagnostic binding."""

    diagnostics: DiagnosticsSettings = field(default_factory=DiagnosticsSettings)

    @classmethod
    def _environment_fields(cls, environment: Mapping[str, str]) -> dict[str, Any]:
        return {"diagnostics": diagnostics_from_environment(environment)}


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
