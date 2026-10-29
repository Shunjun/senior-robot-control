"""ROS-side end of the agent IPC.

The agent process owns the cloud link and the robot database. It owns no rclpy executor and
publishes no ``/cmd_vel``. This process takes its requests over a local Unix socket and
writes velocity only to the safety input through ``CommandNode``. ``rclpy`` is imported inside
``main`` so the request handling below can be tested without ROS.
"""

import asyncio
import os
from collections.abc import Callable
from pathlib import Path
from typing import Any

from senior_control.models import Velocity
from senior_control.protocol import Envelope, validate_envelope

SOCKET_ENV = "SENIOR_AGENT_SOCKET"


class AdapterCore:
    """Maps IPC requests to injected ROS actions. After ``disable_motion`` nothing drives."""

    def __init__(
        self,
        *,
        robot_id: str,
        publish: Callable[[Velocity], None],
        submit: Callable[[Envelope], None],
        request_exit: Callable[[], None],
    ) -> None:
        self._robot_id = robot_id
        self._publish = publish
        self._submit = submit
        self._request_exit = request_exit
        self.motion_disabled = False

    async def handle(self, message: dict[str, Any]) -> dict[str, Any]:
        kind = message["type"]
        if kind == "ping":
            return {"ok": True}
        if kind in {"stop", "estop", "disable_motion"}:
            # Zero first, then refuse further motion; both are safe to repeat.
            self._publish(Velocity(0.0, 0.0))
            if kind == "disable_motion":
                self.motion_disabled = True
            return {"ok": True}
        if kind == "exit":
            self.motion_disabled = True
            self._request_exit()
            return {"ok": True}
        if self.motion_disabled:
            return {"ok": False, "error": "motion_disabled"}
        if kind == "drive":
            self._publish(Velocity(float(message["vx"]), float(message["wz"])))
            return {"ok": True}
        if kind == "command":
            raw = message["envelope"]
            if not isinstance(raw, str):
                return {"ok": False, "error": "bad_request"}
            self._submit(validate_envelope(raw.encode(), self._robot_id))
            return {"ok": True}
        if kind == "ros_call":
            return {"ok": False, "error": "unsupported"}
        return {"ok": False, "error": "unknown_type"}


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
        publish=node.publish_velocity,
        submit=lambda envelope: node.submit(envelope),
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
        # Leave the safety input at zero even if the agent died without saying goodbye.
        node.publish_velocity(Velocity(0.0, 0.0))
        node.destroy_node()
        rclpy.shutdown()
