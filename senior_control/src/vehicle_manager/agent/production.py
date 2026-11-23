"""Concrete production Agent assembly with optional feature effects loaded on demand."""

import asyncio
import logging
from collections.abc import Awaitable, Callable, Mapping
from dataclasses import dataclass
from importlib.util import find_spec
from pathlib import Path
from typing import Any

from vehicle_core.diagnostics import configure_diagnostics
from vehicle_core.errors import DomainError, ErrorCode
from vehicle_core.image_frame import ImageFrame
from vehicle_core.ipc import IpcClient
from vehicle_core.lifecycle_events import READ_FD, observe
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
from vehicle_manager.agent.mapping_catalog import mapping_requests
from vehicle_manager.agent.production_bindings import (
    AgentJournal,
    AgentLogConnection,
    SettingsRuntime,
    VehicleLogService,
    WebSocketConnector,
)
from vehicle_manager.agent.request_bindings import LifecycleNavigationOperations, StoreTaskPort
from vehicle_manager.agent.request_ports import Requester
from vehicle_manager.agent.runtime_bindings import Agent

# //Compatible: keep the historical module factory names patchable at assembly time.
from vehicle_manager.agent.snapshot_bindings import SnapshotState as SnapshotState
from vehicle_manager.agent.snapshot_bindings import vehicle_frame as vehicle_frame
from vehicle_manager.agent.workspace_bindings import WorkspaceReader
from vehicle_manager.media.camera_lease import CameraLease
from vehicle_manager.media.frame_bindings import LatestFrameFile as LatestFrameFile
from vehicle_manager.media.stream import (
    AiortcPeerFactory,
    CameraSource,
    SessionDescription,
    VideoSession,
)
from vehicle_manager.storage.bindings import Database
from vehicle_manager.storage.domain_bindings import TaskStore, WaypointStore
from vehicle_manager.tasks.execution_bindings import NavigationEffects, TaskExecutor
from vehicle_manager.tasks.lifecycle_store import Admission
from vehicle_manager.tasks.navigation_port import TaskNavigation
from vehicle_manager.tasks.runtime import TaskRuntime

logger = logging.getLogger("vehicle_manager.agent")


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
    """Each preview owns its subscriber, camera demand and encoder worker."""
    sessions: set[VideoSession] = set()

    def video_session() -> VideoSession | None:
        if config.ros_video_topic is None and config.simulation_video_frame is None:
            return None
        if find_spec("aiortc") is None:
            logger.warning("video disabled because the video extra is unavailable")
            return None
        source: CameraSource
        close_source: Callable[[], Awaitable[None]] | None = None
        lease: CameraLease | None = None
        if config.ros_video_topic is not None:
            try:
                from senior_sensor_ingress.ros_video import RosCameraSource
            except ModuleNotFoundError as exc:
                if exc.name != "senior_sensor_ingress":
                    raise
                logger.warning("video disabled because the ROS camera feature is unavailable")
                return None
            ros_source = RosCameraSource(config.ros_video_topic, frame_factory=ImageFrame)
            source = ros_source
            close_source = ros_source.close
            lease = CameraLease(config.socket_path.with_name("service.sock"))
        else:
            assert config.simulation_video_frame is not None
            source = LatestFrameFile(config.simulation_video_frame)
        factory = AiortcPeerFactory()
        closing = asyncio.Lock()
        released = False

        class ManagedVideoSession(VideoSession):
            async def offer(self, robot_id: str, offer: SessionDescription) -> SessionDescription:
                try:
                    if lease is not None:
                        await lease.start(self.close)
                    return await super().offer(robot_id, offer)
                except BaseException:
                    await self.close()
                    raise

            async def close(self) -> None:
                nonlocal released
                async with closing:
                    if released:
                        return
                    released = True
                    try:
                        await super().close()
                    finally:
                        try:
                            if close_source is not None:
                                await close_source()
                        finally:
                            try:
                                await asyncio.to_thread(factory.close)
                            finally:
                                if lease is not None:
                                    await lease.close()
                                sessions.discard(self)

        session = ManagedVideoSession(
            robot_id=config.robot_id,
            viewer=_ControlViewer(config.robot_id),
            source=source,
            factory=factory,
        )
        sessions.add(session)
        return session

    async def close() -> None:
        await asyncio.gather(*(session.close() for session in tuple(sessions)))

    return MediaBinding(video_session, close)


def load_exported_grid(path: Path, digest: str, robot_id: str) -> Any:
    """Load the configured display-only map through its optional public owner."""
    try:
        from senior_mapping.workspace_export import load_exported_grid as load
    except ModuleNotFoundError as exc:
        if exc.name != "senior_mapping":
            raise
        raise DomainError(
            ErrorCode.UNAVAILABLE, "workspace mapping feature is unavailable"
        ) from exc
    return load(path, digest, robot_id)


def _read_exported_map(grid: Any, kind: str, fields: Mapping[str, Any]) -> dict[str, Any]:
    from senior_mapping.workspace import WorkspaceMap

    return WorkspaceMap(lambda: grid).read(kind, fields)


def _workspace_reader(
    robot_id: str,
    tasks: TaskStore,
    waypoints: WaypointStore,
    maps: Requester,
    *,
    exported_grid: Any = None,
) -> WorkspaceReader:
    return WorkspaceReader(
        robot_id,
        tasks,
        waypoints,
        maps,
        exported_grid=exported_grid,
        exported_read=_read_exported_map if exported_grid is not None else None,
    )


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
            workspace=_workspace_reader,
            home=IpcHomePort,
            maps=IpcMapPort,
            mapping_requests=mapping_requests,
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
