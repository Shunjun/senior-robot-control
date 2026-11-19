"""Public setting values can be imported without loading service or store owners."""

from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from vehicle_manager.settings.service import RuntimeGate, SettingsService

__all__ = ["RuntimeGate", "SettingsService"]


def __getattr__(name: str) -> Any:
    if name == "RuntimeGate":
        from vehicle_manager.settings.service import RuntimeGate

        return RuntimeGate
    if name == "SettingsService":
        from vehicle_manager.settings.service import SettingsService

        return SettingsService
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
