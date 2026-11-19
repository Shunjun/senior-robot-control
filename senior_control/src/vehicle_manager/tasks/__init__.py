"""Pure task scheduling and lifecycle interfaces; owners load only when requested."""

from importlib import import_module
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from vehicle_manager.tasks import lifecycle
    from vehicle_manager.tasks.models import (
        ExecutionContext,
        ExecutionRecord,
        ExecutionStatus,
        IntentAction,
        SkipRecord,
        TaskDefinition,
        TaskIntent,
    )
    from vehicle_manager.tasks.runner import TaskRunner
    from vehicle_manager.tasks.schedule import Schedule
    from vehicle_manager.tasks.store import TaskStore

__all__ = [
    "ExecutionContext",
    "ExecutionRecord",
    "ExecutionStatus",
    "IntentAction",
    "Schedule",
    "SkipRecord",
    "TaskDefinition",
    "TaskIntent",
    "TaskRunner",
    "TaskStore",
    "lifecycle",
]


def __getattr__(name: str) -> Any:
    if name == "Schedule":
        from vehicle_manager.tasks.schedule import Schedule

        return Schedule
    if name == "lifecycle":
        return import_module("vehicle_manager.tasks.lifecycle")
    if name in {
        "ExecutionContext",
        "ExecutionRecord",
        "ExecutionStatus",
        "IntentAction",
        "SkipRecord",
        "TaskDefinition",
        "TaskIntent",
    }:
        return getattr(import_module("vehicle_manager.tasks.models"), name)
    if name == "TaskRunner":
        from vehicle_manager.tasks.runner import TaskRunner

        return TaskRunner
    if name == "TaskStore":
        from vehicle_manager.tasks.store import TaskStore

        return TaskStore
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
