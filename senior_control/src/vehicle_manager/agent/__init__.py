"""Management transport and persistence foundations, loaded only when requested."""

from importlib import import_module
from typing import Any

__all__ = [
    "AgentJournal",
    "Agent",
    "HeartbeatFrame",
    "Link",
    "MotionPort",
    "run_agent",
    "RobotSnapshot",
    "LiveState",
    "SnapshotState",
    "WorkspaceReader",
    "Dispatcher",
    "SettingsRuntime",
    "AgentLogConnection",
    "AuthRejected",
    "Credentials",
    "Supervisor",
    "WebSocketLink",
    "WebSocketConnector",
    "pair_device",
    "read_credentials",
    "read_token",
]


def __getattr__(name: str) -> Any:
    if name in {"AuthRejected", "Supervisor"}:
        return getattr(import_module("vehicle_manager.agent.supervisor"), name)
    if name in {"WebSocketLink", "WebSocketConnector"}:
        return getattr(import_module("vehicle_manager.agent.websocket"), name)
    if name in {"Credentials", "pair_device", "read_credentials", "read_token"}:
        return getattr(import_module("vehicle_manager.agent.pairing"), name)
    if name == "AgentJournal":
        return getattr(import_module("vehicle_manager.agent.journal"), name)
    modules = {
        "Agent": "main",
        "HeartbeatFrame": "connection",
        "Link": "connection",
        "MotionPort": "connection",
        "run_agent": "composition",
        "RobotSnapshot": "snapshots",
        "LiveState": "snapshots",
        "SnapshotState": "snapshot_state",
        "WorkspaceReader": "workspace",
        "Dispatcher": "dispatcher",
        "SettingsRuntime": "settings_wire",
        "AgentLogConnection": "logs_wire",
    }
    if name in modules:
        return getattr(import_module("vehicle_manager.agent." + modules[name]), name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
