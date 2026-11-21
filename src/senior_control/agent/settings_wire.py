"""//Compatible: original names forward to the same concrete manager bindings and helpers."""

from vehicle_manager.agent.local_adapters import IpcState as IpcState
from vehicle_manager.agent.production_bindings import SettingsRuntime as SettingsRuntime
from vehicle_manager.agent.settings_wire import settings_command as settings_command

from vehicle_core.errors import DomainError as DomainError
