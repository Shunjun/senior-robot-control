"""Management transport and persistence foundations, loaded only when requested."""

from importlib import import_module
from typing import Any

__all__ = [
    "AgentJournal",
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
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
