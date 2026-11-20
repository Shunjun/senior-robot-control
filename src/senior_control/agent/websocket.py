"""//Compatible: original network constructors with the existing reporting/diagnostic owners."""

import time
from collections.abc import Callable

from vehicle_manager.agent.connection import HeartbeatFrame
from vehicle_manager.agent.ports import WebSocketPorts
from vehicle_manager.agent.websocket import (
    SOCKET_SEND_TIMEOUT_SECONDS as SOCKET_SEND_TIMEOUT_SECONDS,
)
from vehicle_manager.agent.websocket import ProtocolRejected as ProtocolRejected
from vehicle_manager.agent.websocket import Socket as Socket
from vehicle_manager.agent.websocket import WebSocketConnector as _WebSocketConnector
from vehicle_manager.agent.websocket import WebSocketLink as _WebSocketLink

from senior_control.agent.snapshots import RobotSnapshot, snapshot_envelope
from vehicle_core.diagnostics import DiagnosticLock, QueueSnapshot, process_diagnostics


def _receive_diagnostic(owner: object) -> None:
    process_diagnostics().queue(
        QueueSnapshot("websocket.receive", None, 16, depth_basis="library_depth_unavailable"),
        time.monotonic(),
        owner=owner,
    )


class WebSocketLink(_WebSocketLink[RobotSnapshot, HeartbeatFrame]):
    """//Compatible: original signature; sole heartbeat/transport state is inherited."""

    def __init__(
        self,
        socket: Socket,
        robot_id: str,
        epoch: int,
        *,
        wall_clock: Callable[[], float] = time.time,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        super().__init__(
            socket,
            robot_id,
            epoch,
            wall_clock=wall_clock,
            clock=clock,
            ports=WebSocketPorts(
                heartbeat=HeartbeatFrame,
                snapshot=snapshot_envelope,
                send_lock=lambda: DiagnosticLock("websocket.send_wait"),
                receive_diagnostic=_receive_diagnostic,
            ),
        )


class WebSocketConnector(_WebSocketConnector[WebSocketLink]):
    """//Compatible: original signature and original reporting-aware link constructor."""

    def __init__(self, url: str, token: str, robot_id: str) -> None:
        super().__init__(url, token, robot_id, link_factory=WebSocketLink)
