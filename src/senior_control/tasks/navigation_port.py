"""//Compatible: original navigation construction over the sole manager adapter."""

from vehicle_manager.tasks.navigation_port import NavigationEffects as _NavigationEffects
from vehicle_manager.tasks.navigation_port import TaskNavigation as TaskNavigation

from senior_control.navigation import RouteRejected
from senior_control.tasks.lifecycle_store import LifecycleStore


class NavigationEffects(_NavigationEffects):
    """//Compatible: original signature and rejection identity; inherited effect methods."""

    def __init__(
        self,
        store: LifecycleStore,
        navigation: TaskNavigation,
        *,
        timeout_ms: float = 300.0,
    ):
        super().__init__(store, navigation, rejection_type=RouteRejected, timeout_ms=timeout_ms)
