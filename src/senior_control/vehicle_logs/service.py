"""//Compatible: explicit aliases of the sole manager log implementation."""

import time
from collections.abc import Callable

from vehicle_manager.settings.models import SettingsSnapshot
from vehicle_manager.vehicle_logs.diagnostics import LogDiagnostics
from vehicle_manager.vehicle_logs.service import Action as Action
from vehicle_manager.vehicle_logs.service import LogAuthorizer as LogAuthorizer
from vehicle_manager.vehicle_logs.service import LogSubscription as LogSubscription
from vehicle_manager.vehicle_logs.service import VehicleLogService as _VehicleLogService
from vehicle_manager.vehicle_logs.store import VehicleLogStore

from vehicle_core.diagnostics import (
    DiagnosticAsyncQueue,
    DiagnosticLock,
    PendingCounter,
    process_diagnostics,
)


class VehicleLogService(_VehicleLogService):
    """//Compatible: original constructor; all log business methods are canonical."""

    def __init__(
        self,
        store: VehicleLogStore,
        vehicle_id: str,
        authorizer: LogAuthorizer,
        settings: Callable[[], SettingsSnapshot],
        *,
        utc_now: Callable[[], float] = time.time,
    ) -> None:
        super().__init__(
            store,
            vehicle_id,
            authorizer,
            settings,
            utc_now=utc_now,
            diagnostics=LogDiagnostics(
                counter=PendingCounter,
                lock=DiagnosticLock,
                outbox=DiagnosticAsyncQueue,
                bind_async=lambda: process_diagnostics().bind_async(),
            ),
        )
