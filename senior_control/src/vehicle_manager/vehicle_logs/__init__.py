"""Vehicle log interfaces; public values do not eagerly load service/store owners."""

from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from vehicle_manager.vehicle_logs.service import LogAuthorizer, VehicleLogService
    from vehicle_manager.vehicle_logs.store import VehicleLogStore

__all__ = ["LogAuthorizer", "VehicleLogService", "VehicleLogStore"]


def __getattr__(name: str) -> Any:
    if name in {"LogAuthorizer", "VehicleLogService"}:
        from vehicle_manager.vehicle_logs.service import LogAuthorizer, VehicleLogService

        return LogAuthorizer if name == "LogAuthorizer" else VehicleLogService
    if name == "VehicleLogStore":
        from vehicle_manager.vehicle_logs.store import VehicleLogStore

        return VehicleLogStore
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
