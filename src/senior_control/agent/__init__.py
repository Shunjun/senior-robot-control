"""//Compatible: retain Agent and RobotSnapshot without loading their owners eagerly."""

from importlib import import_module
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from senior_control.agent.main import Agent as Agent
    from senior_control.agent.snapshots import RobotSnapshot as RobotSnapshot

__all__ = ["Agent", "RobotSnapshot"]
_EXPORTS = {"Agent": "senior_control.agent.main", "RobotSnapshot": "senior_control.agent.snapshots"}


def __getattr__(name: str) -> Any:
    if name not in _EXPORTS:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    value = getattr(import_module(_EXPORTS[name]), name)
    globals()[name] = value
    return value


def __dir__() -> list[str]:
    return sorted(set(globals()) | set(__all__))
