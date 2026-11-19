"""//Compatible: preserve public names without eagerly loading their business owners."""

from typing import TYPE_CHECKING

from vehicle_manager.settings import __all__ as __all__
from vehicle_manager.settings import __getattr__ as __getattr__

if TYPE_CHECKING:
    from vehicle_manager.settings.service import RuntimeGate as RuntimeGate
    from vehicle_manager.settings.service import SettingsService as SettingsService
