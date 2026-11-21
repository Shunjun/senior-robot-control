"""//Compatible: original names forward to the same concrete manager bindings and helpers."""

from vehicle_manager.agent.connection import HeartbeatFrame as HeartbeatFrame
from vehicle_manager.agent.ports import WebSocketPorts as WebSocketPorts
from vehicle_manager.agent.production_bindings import WebSocketConnector as WebSocketConnector
from vehicle_manager.agent.production_bindings import WebSocketLink as WebSocketLink
from vehicle_manager.agent.production_bindings import _receive_diagnostic as _receive_diagnostic
from vehicle_manager.agent.snapshots import RobotSnapshot as RobotSnapshot
from vehicle_manager.agent.snapshots import snapshot_envelope as snapshot_envelope
from vehicle_manager.agent.websocket import (
    SOCKET_SEND_TIMEOUT_SECONDS as SOCKET_SEND_TIMEOUT_SECONDS,
)
from vehicle_manager.agent.websocket import ProtocolRejected as ProtocolRejected
from vehicle_manager.agent.websocket import Socket as Socket

from vehicle_core.diagnostics import DiagnosticLock as DiagnosticLock
from vehicle_core.diagnostics import QueueSnapshot as QueueSnapshot
from vehicle_core.diagnostics import process_diagnostics as process_diagnostics
