"""//Compatible: historical production assembly names share the canonical implementation."""

# //Compatible: original optional camera symbol remains available to SDK callers.
from senior_robot.ros_video import RosCameraSource as RosCameraSource
from vehicle_manager.agent.production import READ_FD as READ_FD
from vehicle_manager.agent.production import Admission as Admission
from vehicle_manager.agent.production import Agent as Agent
from vehicle_manager.agent.production import AgentBootstrapPorts as AgentBootstrapPorts
from vehicle_manager.agent.production import AgentConfig as AgentConfig
from vehicle_manager.agent.production import AgentJournal as AgentJournal
from vehicle_manager.agent.production import AgentLogConnection as AgentLogConnection
from vehicle_manager.agent.production import AiortcPeerFactory as AiortcPeerFactory
from vehicle_manager.agent.production import Callable as Callable
from vehicle_manager.agent.production import Database as Database
from vehicle_manager.agent.production import DomainError as DomainError
from vehicle_manager.agent.production import ImageFrame as ImageFrame
from vehicle_manager.agent.production import IpcClient as IpcClient
from vehicle_manager.agent.production import IpcHomePort as IpcHomePort
from vehicle_manager.agent.production import IpcMapPort as IpcMapPort
from vehicle_manager.agent.production import IpcNavigation as IpcNavigation
from vehicle_manager.agent.production import IpcState as IpcState
from vehicle_manager.agent.production import LatestFrameFile as LatestFrameFile
from vehicle_manager.agent.production import (
    LifecycleNavigationOperations as LifecycleNavigationOperations,
)
from vehicle_manager.agent.production import MediaBinding as MediaBinding
from vehicle_manager.agent.production import NavigationEffects as NavigationEffects
from vehicle_manager.agent.production import SettingsRuntime as SettingsRuntime
from vehicle_manager.agent.production import SnapshotState as SnapshotState
from vehicle_manager.agent.production import StoreTaskPort as StoreTaskPort
from vehicle_manager.agent.production import TaskAwareMotion as TaskAwareMotion
from vehicle_manager.agent.production import TaskExecutor as TaskExecutor
from vehicle_manager.agent.production import TaskNavigation as TaskNavigation
from vehicle_manager.agent.production import TaskRuntime as TaskRuntime
from vehicle_manager.agent.production import TaskStore as TaskStore
from vehicle_manager.agent.production import VehicleLogService as VehicleLogService
from vehicle_manager.agent.production import VideoSession as VideoSession
from vehicle_manager.agent.production import WaypointStore as WaypointStore
from vehicle_manager.agent.production import WebSocketConnector as WebSocketConnector
from vehicle_manager.agent.production import _build_task_runtime as _build_task_runtime
from vehicle_manager.agent.production import _ControlViewer as _ControlViewer
from vehicle_manager.agent.production import _media_binding as _media_binding
from vehicle_manager.agent.production import _run_agent as _run_agent
from vehicle_manager.agent.production import build_task_runtime as build_task_runtime
from vehicle_manager.agent.production import configure_diagnostics as configure_diagnostics
from vehicle_manager.agent.production import dataclass as dataclass
from vehicle_manager.agent.production import find_spec as find_spec
from vehicle_manager.agent.production import load_exported_grid as load_exported_grid
from vehicle_manager.agent.production import logger as logger
from vehicle_manager.agent.production import observe as observe
from vehicle_manager.agent.production import pause_failed_tasks as pause_failed_tasks
from vehicle_manager.agent.production import run_agent as run_agent
from vehicle_manager.agent.production import vehicle_frame as vehicle_frame

# //Compatible: original exported-map constructor retains its explicit mapping effect.
from senior_control.agent.workspace import WorkspaceReader as WorkspaceReader
