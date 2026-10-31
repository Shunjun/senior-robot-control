"""ROS-side end of the agent IPC.

The agent process owns the cloud link and the robot database. It owns no rclpy executor and
publishes no ``/cmd_vel``. This process takes its requests over a local Unix socket and
hands every motion request to the arbiter through ``CommandNode``: drive, stop, and estop
arrive as validated envelopes and go through ``submit``; disable and exit go through
``disconnect``. Nothing here writes a velocity of its own, so the arbiter's timer is the
only publisher and cannot replay an old lease behind a stop. ``rclpy`` is imported inside
``main`` so the request handling below can be tested without ROS.
"""

import asyncio
import os
from collections.abc import Callable
from pathlib import Path
from typing import Any

from senior_control.protocol import (
    SimpleEnvelope,
    VelocityEnvelope,
    validate_envelope,
)
from senior_control.safety.arbiter import CommandResult

SOCKET_ENV = "SENIOR_AGENT_SOCKET"

_ENVELOPE_KINDS = {
    "drive": "drive.velocity",
    "stop": "drive.stop",
    "estop": "safety.estop",
}


class AdapterCore:
    """Maps IPC requests to the arbiter. After ``disable_motion`` nothing drives."""

    def __init__(
        self,
        *,
        robot_id: str,
        submit: Callable[..., CommandResult],
        disconnect: Callable[[int], None],
        request_exit: Callable[[], None],
        server_heartbeat: Callable[[int, float], bool] | None = None,
    ) -> None:
        self._robot_id = robot_id
        self._submit = submit
        self._disconnect = disconnect
        self._request_exit = request_exit
        self._server_heartbeat = server_heartbeat
        self._active_epoch: int | None = None
        self._highest_epoch = -1
        self._disabled_through_epoch = -1
        # //Compatible: legacy isolated adapter tests do not have a heartbeat callback.
        self.motion_disabled = server_heartbeat is not None
        self._exiting = False

    def release(self) -> None:
        """End the active epoch only. Safe to repeat."""
        if self._active_epoch is not None:
            self._disconnect(self._active_epoch)
            self._active_epoch = None

    async def handle(self, message: dict[str, Any]) -> dict[str, Any]:
        kind = message["type"]
        if kind == "server_heartbeat":
            epoch, deadline = message.get("connection_epoch"), message.get("deadline")
            if (
                self._exiting
                or self._server_heartbeat is None
                or not isinstance(epoch, int)
                or isinstance(epoch, bool)
                or isinstance(deadline, bool)
                or not isinstance(deadline, (int, float))
                or epoch <= self._disabled_through_epoch
            ):
                return {"ok": False, "error": "bad_request"}
            accepted = self._server_heartbeat(epoch, float(deadline))
            return {"ok": accepted}
        if kind == "ping":
            return {"ok": True}
        if kind == "disable_motion":
            epoch = message.get("connection_epoch")
            if not isinstance(epoch, int) or isinstance(epoch, bool):
                return {"ok": False, "error": "bad_request"}
            if epoch < max(self._highest_epoch, self._disabled_through_epoch):
                return {"ok": True}
            self._disabled_through_epoch = epoch
            self.motion_disabled = True
            self.release()
            return {"ok": True}
        if kind == "exit":
            self._exiting = True
            self.motion_disabled = True
            self.release()
            self._request_exit()
            return {"ok": True}
        if kind in {"stop", "estop"}:
            return self._send(message, kind, safety=True)
        if self.motion_disabled:
            if kind == "command" and not self._exiting:
                return self._reenable(message)
            return {"ok": False, "error": "motion_disabled"}
        if kind == "drive":
            return self._send(message, kind, safety=False)
        if kind == "command":
            return self._send(message, None, safety=False)
        if kind == "ros_call":
            return {"ok": False, "error": "unsupported"}
        return {"ok": False, "error": "unknown_type"}

    def _reenable(self, message: dict[str, Any]) -> dict[str, Any]:
        raw = message.get("envelope")
        if not isinstance(raw, str):
            return {"ok": False, "error": "motion_disabled"}
        try:
            envelope = validate_envelope(raw.encode(), self._robot_id)
        except ValueError:
            return {"ok": False, "error": "motion_disabled"}
        if envelope.type != "control.acquire" or envelope.connection_epoch <= max(
            self._highest_epoch, self._disabled_through_epoch
        ):
            return {"ok": False, "error": "motion_disabled"}
        result = self._send(message, None, safety=False)
        if result.get("ok"):
            self.motion_disabled = False
        return result

    def _send(self, message: dict[str, Any], kind: str | None, *, safety: bool) -> dict[str, Any]:
        del safety
        raw = message.get("envelope")
        if not isinstance(raw, str):
            return {"ok": False, "error": "bad_request"}
        try:
            envelope = validate_envelope(raw.encode(), self._robot_id)
        except ValueError:
            return {"ok": False, "error": "bad_request"}
        if kind is not None and envelope.type != _ENVELOPE_KINDS[kind]:
            return {"ok": False, "error": "bad_request"}
        if not isinstance(envelope, (VelocityEnvelope, SimpleEnvelope)):
            return {"ok": False, "error": "bad_request"}
        if self._server_heartbeat is not None and (
            envelope.type.startswith("navigation.") or envelope.type == "safety.reset"
        ):
            return {"ok": False, "error": "unavailable"}
        received_at = message.get("received_at")
        if isinstance(received_at, bool) or not isinstance(received_at, (int, float)):
            received_at = None
        result = self._submit(envelope, received_at)
        if not result.accepted:
            # The agent distinguishes stale frames from an unconfirmed safety request.
            return {"ok": False, "accepted": False, "error": result.error.value}
        if envelope.type not in {"safety.estop", "safety.reset"}:
            self._highest_epoch = max(self._highest_epoch, envelope.connection_epoch)
            self._active_epoch = (
                None if envelope.type == "control.release" else envelope.connection_epoch
            )
        return {"ok": True, "accepted": True}


def main() -> None:
    import rclpy

    from senior_control.runtime.ipc import IpcServer
    from senior_robot.command_node import CommandNode

    rclpy.init()
    node = CommandNode(require_server_heartbeat=True)
    robot_id = str(node.get_parameter("robot_id").value)
    path = Path(os.environ[SOCKET_ENV])
    exiting = False

    def request_exit() -> None:
        nonlocal exiting
        exiting = True

    core = AdapterCore(
        robot_id=robot_id,
        submit=lambda envelope, received_at=None: node.submit(envelope, received_at),
        disconnect=node.disconnect,
        request_exit=request_exit,
        server_heartbeat=node.server_heartbeat,
    )

    async def serve() -> None:
        server = IpcServer(path, core.handle)
        await server.start()
        try:
            while rclpy.ok() and not exiting:
                rclpy.spin_once(node, timeout_sec=0.0)
                await asyncio.sleep(0.01)
        finally:
            await server.close()

    try:
        asyncio.run(serve())
    finally:
        # The agent has normally ended every lease already; end them again through the arbiter.
        core.release()
        node.destroy_node()
        rclpy.shutdown()
