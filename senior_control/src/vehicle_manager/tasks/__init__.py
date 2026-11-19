"""Pure task scheduling and lifecycle interfaces; owners load only when requested."""

from importlib import import_module
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from vehicle_manager.tasks import lifecycle
    from vehicle_manager.tasks.schedule import Schedule

__all__ = ["Schedule", "lifecycle"]


def __getattr__(name: str) -> Any:
    if name == "Schedule":
        from vehicle_manager.tasks.schedule import Schedule

        return Schedule
    if name == "lifecycle":
        return import_module("vehicle_manager.tasks.lifecycle")
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
