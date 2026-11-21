"""//Compatible: original names forward to the same concrete manager bindings and helpers."""

from vehicle_manager.agent.logs_wire import AgentLogHandler as AgentLogHandler
from vehicle_manager.agent.logs_wire import CloudLogGrants as CloudLogGrants
from vehicle_manager.agent.logs_wire import log_url as log_url
from vehicle_manager.agent.production_bindings import AgentLogConnection as AgentLogConnection
from vehicle_manager.agent.production_bindings import VehicleLogService as VehicleLogService

from vehicle_core.diagnostics import DiagnosticLock as DiagnosticLock
from vehicle_core.diagnostics import PendingCounter as PendingCounter
