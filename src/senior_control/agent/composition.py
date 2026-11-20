"""Configuration and composition for the robot agent."""

import asyncio
import logging
from collections.abc import Callable
from dataclasses import dataclass
from importlib.util import find_spec

from senior_mapping.workspace_export import load_exported_grid
from senior_robot.ros_video import RosCameraSource
from vehicle_manager.agent.bootstrap_ports import AgentBootstrapPorts, MediaBinding
from vehicle_manager.agent.composition import build_task_runtime as _build_task_runtime
from vehicle_manager.agent.composition import pause_failed_tasks as pause_failed_tasks
from vehicle_manager.agent.composition import run_agent as _run_agent

# //Compatible: the original configuration name is the same canonical class.
from vehicle_manager.agent.config import AgentConfig as AgentConfig
from vehicle_manager.agent.local_adapters import (
    IpcHomePort,
    IpcMapPort,
    IpcNavigation,
    IpcState,
    TaskAwareMotion,
)
from vehicle_manager.agent.production_bindings import (
    AgentJournal,
    AgentLogConnection,
    SettingsRuntime,
    VehicleLogService,
    WebSocketConnector,
)
from vehicle_manager.agent.request_bindings import LifecycleNavigationOperations, StoreTaskPort
from vehicle_manager.agent.runtime_bindings import Agent

# //Compatible: keep the historical module factory names patchable at assembly time.
from vehicle_manager.agent.snapshot_bindings import SnapshotState as SnapshotState
from vehicle_manager.agent.snapshot_bindings import vehicle_frame as vehicle_frame
from vehicle_manager.media.frame_bindings import LatestFrameFile as LatestFrameFile
from vehicle_manager.media.stream import AiortcPeerFactory, VideoSession
from vehicle_manager.storage.bindings import Database
from vehicle_manager.storage.domain_bindings import TaskStore, WaypointStore
from vehicle_manager.tasks.execution_bindings import NavigationEffects, TaskExecutor
from vehicle_manager.tasks.navigation_port import TaskNavigation

from senior_control.agent.workspace import WorkspaceReader
from senior_control.tasks.lifecycle_store import Admission
from senior_control.tasks.runtime import TaskRuntime
from vehicle_core.diagnostics import configure_diagnostics
from vehicle_core.errors import DomainError
from vehicle_core.image_frame import ImageFrame
from vehicle_core.ipc import IpcClient
from vehicle_core.lifecycle_events import READ_FD, observe

logger = logging.getLogger("senior_control.agent")


@dataclass(frozen=True)
class _ControlViewer:
    robot_id: str
    role: str = "control"


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
