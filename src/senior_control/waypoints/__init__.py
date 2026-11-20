"""//Compatible: original waypoint names, loaded only when requested."""

from importlib import import_module
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from senior_control.waypoints.home import (
        HomeResult,
        HomeService,
        LocalizedPose,
        StartupReference,
    )
    from senior_control.waypoints.store import Waypoint

__all__ = ["HomeResult", "HomeService", "LocalizedPose", "StartupReference", "Waypoint"]


def __getattr__(name: str) -> Any:
    if name == "Waypoint":
        return getattr(import_module("senior_control.waypoints.store"), name)
    if name in {"HomeResult", "HomeService", "LocalizedPose", "StartupReference"}:
        return getattr(import_module("senior_control.waypoints.home"), name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
