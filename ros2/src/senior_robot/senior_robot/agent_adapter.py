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
    Envelope,
    SimpleEnvelope,
    VelocityEnvelope,
    validate_envelope,
)

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
        submit: Callable[[Envelope], bool],
        disconnect: Callable[[int], None],
        request_exit: Callable[[], None],
    ) -> None:
        self._robot_id = robot_id
        self._submit = submit
        self._disconnect = disconnect
        self._request_exit = request_exit
        self._epochs: set[int] = set()
        self.motion_disabled = False

    def release(self) -> None:
        """End every lease epoch this adapter has seen. Safe to repeat."""
        for epoch in sorted(self._epochs):
            self._disconnect(epoch)

    async def handle(self, message: dict[str, Any]) -> dict[str, Any]:
        kind = message["type"]
        if kind == "ping":
            return {"ok": True}
        if kind == "disable_motion":
            self.motion_disabled = True
            epoch = message.get("connection_epoch")
            if isinstance(epoch, int) and not isinstance(epoch, bool):
                self._epochs.add(epoch)
            self.release()
            return {"ok": True}
        if kind == "exit":
            self.motion_disabled = True
            self.release()
            self._request_exit()
            return {"ok": True}
        if kind in {"stop", "estop"}:
            return self._send(message, kind, safety=True)
        if self.motion_disabled:
            return {"ok": False, "error": "motion_disabled"}
        if kind == "drive":
            return self._send(message, kind, safety=False)
        if kind == "command":
            return self._send(message, None, safety=False)
        if kind == "ros_call":
            return {"ok": False, "error": "unsupported"}
        return {"ok": False, "error": "unknown_type"}

    def _send(self, message: dict[str, Any], kind: str | None, *, safety: bool) -> dict[str, Any]:
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
        self._epochs.add(envelope.connection_epoch)
        accepted = self._submit(envelope)
        if safety and not accepted:
            # The arbiter refused it (stale seq, no lease). Motion must still end.
            self.release()
        return {"ok": True, "accepted": accepted}


def main() -> None:
    import rclpy

    from senior_control.runtime.ipc import IpcServer
    from senior_robot.command_node import CommandNode

    rclpy.init()
    node = CommandNode()
    robot_id = str(node.get_parameter("robot_id").value)
    path = Path(os.environ[SOCKET_ENV])
    exiting = False

    def request_exit() -> None:
        nonlocal exiting
        exiting = True

    core = AdapterCore(
        robot_id=robot_id,
        submit=lambda envelope: node.submit(envelope).accepted,
        disconnect=node.disconnect,
        request_exit=request_exit,
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
