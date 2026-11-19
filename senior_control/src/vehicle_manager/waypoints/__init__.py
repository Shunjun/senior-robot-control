"""Waypoint storage interfaces; Home execution and value owners remain outside."""

from importlib import import_module
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from vehicle_manager.waypoints.direct_navigation import DirectNavigation
    from vehicle_manager.waypoints.home_capture import HomeCapture
    from vehicle_manager.waypoints.ports import WaypointStorePorts
    from vehicle_manager.waypoints.store import WaypointStore

__all__ = ["HomeCapture", "DirectNavigation", "WaypointStore", "WaypointStorePorts"]


def __getattr__(name: str) -> Any:
    if name == "HomeCapture":
        return getattr(import_module("vehicle_manager.waypoints.home_capture"), name)
    if name == "DirectNavigation":
        return getattr(import_module("vehicle_manager.waypoints.direct_navigation"), name)
    if name == "WaypointStore":
        return getattr(import_module("vehicle_manager.waypoints.store"), name)
    if name == "WaypointStorePorts":
        return getattr(import_module("vehicle_manager.waypoints.ports"), name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
