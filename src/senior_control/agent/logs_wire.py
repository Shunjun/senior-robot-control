"""//Compatible: log wire adapter with the original diagnostic factories."""

from vehicle_manager.agent.logs_wire import (
    AgentLogConnection as _AgentLogConnection,
)
from vehicle_manager.agent.logs_wire import (
    AgentLogHandler as AgentLogHandler,
)
from vehicle_manager.agent.logs_wire import (
    CloudLogGrants as CloudLogGrants,
)
from vehicle_manager.agent.logs_wire import (
    log_url as log_url,
)

from senior_control.runtime.diagnostics import DiagnosticLock, PendingCounter
from senior_control.vehicle_logs.service import VehicleLogService


class AgentLogConnection(_AgentLogConnection):
    """//Compatible: original constructor; no duplicate diagnostic state."""

    def __init__(self, service: VehicleLogService, grants: CloudLogGrants) -> None:
        super().__init__(
            service, grants, lock_factory=DiagnosticLock, counter_factory=PendingCounter
        )
