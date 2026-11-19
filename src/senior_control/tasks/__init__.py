"""//Compatible: original task names, loaded only when requested."""

from importlib import import_module
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from vehicle_manager.tasks.schedule import Schedule

    from senior_control.tasks.executor import NavigationPort, ProcessCrash, TaskExecutor
    from senior_control.tasks.models import (
        ExecutionContext,
        ExecutionRecord,
        ExecutionStatus,
        IntentAction,
        SkipRecord,
        TaskDefinition,
        TaskIntent,
    )
    from senior_control.tasks.runner import TaskRunner
    from senior_control.tasks.store import TaskStore

__all__ = [
    "ExecutionContext",
    "ExecutionRecord",
    "ExecutionStatus",
    "IntentAction",
    "NavigationPort",
    "ProcessCrash",
    "Schedule",
    "SkipRecord",
    "TaskDefinition",
    "TaskExecutor",
    "TaskIntent",
    "TaskRunner",
    "TaskStore",
]


def __getattr__(name: str) -> Any:
    if name == "Schedule":
        from vehicle_manager.tasks.schedule import Schedule

        return Schedule
    if name in {"NavigationPort", "ProcessCrash", "TaskExecutor"}:
        return getattr(import_module("senior_control.tasks.executor"), name)
    if name in {
        "ExecutionContext",
        "ExecutionRecord",
        "ExecutionStatus",
        "IntentAction",
        "SkipRecord",
        "TaskDefinition",
        "TaskIntent",
    }:
        return getattr(import_module("senior_control.tasks.models"), name)
    if name == "TaskRunner":
        from senior_control.tasks.runner import TaskRunner

        return TaskRunner
    if name == "TaskStore":
        from senior_control.tasks.store import TaskStore

        return TaskStore
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
