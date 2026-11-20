"""//Compatible: original names forward to the same concrete manager bindings and helpers."""

from vehicle_manager.agent.production_bindings import VehicleLogService as VehicleLogService
from vehicle_manager.settings.models import SettingsSnapshot as SettingsSnapshot
from vehicle_manager.vehicle_logs.diagnostics import LogDiagnostics as LogDiagnostics
from vehicle_manager.vehicle_logs.service import Action as Action
from vehicle_manager.vehicle_logs.service import LogAuthorizer as LogAuthorizer
from vehicle_manager.vehicle_logs.service import LogSubscription as LogSubscription
from vehicle_manager.vehicle_logs.store import VehicleLogStore as VehicleLogStore

from vehicle_core.diagnostics import DiagnosticAsyncQueue as DiagnosticAsyncQueue
from vehicle_core.diagnostics import DiagnosticLock as DiagnosticLock
from vehicle_core.diagnostics import PendingCounter as PendingCounter
from vehicle_core.diagnostics import process_diagnostics as process_diagnostics
