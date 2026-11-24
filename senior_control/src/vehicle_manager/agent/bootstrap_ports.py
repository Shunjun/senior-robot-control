"""Explicit production assembly inputs, without importing legacy or ROS implementations."""

from collections.abc import Awaitable, Callable
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Protocol

from vehicle_manager.agent.connection import Link, MotionPort
from vehicle_manager.agent.dispatcher import HomePort, LifecycleNavigationOperations, MapPort
from vehicle_manager.agent.journal import AgentJournal
from vehicle_manager.agent.logs_wire import AgentLogConnection
from vehicle_manager.agent.main import Agent
from vehicle_manager.agent.request_ports import Requester
from vehicle_manager.agent.settings_wire import SettingsRuntime
from vehicle_manager.agent.snapshot_state import SnapshotState
from vehicle_manager.agent.state_ports import StatePort
from vehicle_manager.agent.workspace import WorkspaceReader
from vehicle_manager.settings.service import SettingsService
from vehicle_manager.storage.database import Database
from vehicle_manager.tasks.executor import TaskExecutor
from vehicle_manager.tasks.navigation_port import TaskNavigation
from vehicle_manager.tasks.runtime import TaskRuntime
from vehicle_manager.tasks.store import TaskStore
from vehicle_manager.vehicle_logs.service import VehicleLogService
from vehicle_manager.waypoints.store import WaypointStore


class AgentConfiguration(Protocol):
    @property
    def robot_id(self) -> str: ...
    @property
    def cloud_url(self) -> str: ...
    @property
    def device_token(self) -> str: ...
    @property
    def database_path(self) -> Path: ...
    @property
    def socket_path(self) -> Path: ...
    @property
    def workspace_map_bundle(self) -> Path | None: ...
    @property
    def workspace_map_sha256(self) -> str | None: ...


class LocalClient(Protocol):
    async def request(self, kind: str, **fields: Any) -> dict[str, Any]: ...
    async def close(self) -> None: ...


class ManagedMotion(MotionPort, Protocol):
    settings: SettingsService | None

    def bind_tasks(self, tasks: Any) -> None: ...


@dataclass(frozen=True)
class MediaBinding:
    session: Callable[[], object | None]
    close: Callable[[], Awaitable[None]]


@dataclass(frozen=True)
class AgentBootstrapPorts:
    configure_diagnostics: Callable[[], object]
    database: Callable[[Path], Database]
    tasks: Callable[..., TaskStore]
    waypoints: Callable[..., WaypointStore[Any, Any, Any]]
    journal: Callable[..., AgentJournal[Any, Any]]
    ipc: Callable[..., LocalClient]
    motion: Callable[..., ManagedMotion]
    state: Callable[..., StatePort]
    workspace: Callable[..., WorkspaceReader]
    home: Callable[..., HomePort]
    maps: Callable[..., MapPort[Any]]
    navigation: Callable[..., TaskNavigation]
    build_task_runtime: Callable[..., tuple[TaskRuntime, TaskExecutor]]
    settings_runtime: Callable[..., SettingsRuntime]
    log_service: Callable[..., VehicleLogService]
    log_connection: Callable[..., AgentLogConnection]
    navigation_actions: Callable[..., LifecycleNavigationOperations[Any]]
    snapshot_state: Callable[..., SnapshotState]
    connector: Callable[..., Callable[[], Awaitable[Link]]]
    store_task_port: Callable[..., Any]
    agent: Callable[..., Agent[Any]]
    load_exported_grid: Callable[..., Any]
    media: Callable[[], MediaBinding]
    observe: Callable[[int, Callable[[dict[str, Any]], None]], Awaitable[None]]
    read_fd: str
    error: type[Exception]
    mapping_requests: Callable[..., Awaitable[Requester | None]] | None = None
