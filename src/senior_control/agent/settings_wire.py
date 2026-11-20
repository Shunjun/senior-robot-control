"""//Compatible: settings wire implementation with the original local error type."""

from typing import TYPE_CHECKING

from vehicle_manager.agent.settings_wire import SettingsRuntime as _SettingsRuntime
from vehicle_manager.agent.settings_wire import settings_command as settings_command

from senior_control.agent.ports import IpcState
from vehicle_core.errors import DomainError

if TYPE_CHECKING:
    from senior_control.tasks.lifecycle_store import LifecycleStore


class SettingsRuntime(_SettingsRuntime):
    """//Compatible: original signature; runtime evidence remains a single instance."""

    def __init__(self, state: IpcState, tasks: "LifecycleStore") -> None:
        super().__init__(state, tasks, error=DomainError)
