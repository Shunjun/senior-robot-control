"""Own one Nav2 controller publisher for one navigation execution."""

import asyncio
import os
import secrets
import signal
from collections.abc import AsyncIterator
from contextlib import asynccontextmanager, suppress
from pathlib import Path

from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from rclpy.node import Node


class DedicatedController:
    def __init__(self, node: Node, params: Path) -> None:
        self._node = node
        self._params = params
        self._lock = asyncio.Lock()

    async def _transition(self, transition: int, process: asyncio.subprocess.Process) -> None:
        client = self._node.create_client(ChangeState, "/controller_server/change_state")
        deadline = asyncio.get_running_loop().time() + 10
        try:
            while not client.service_is_ready():
                if process.returncode is not None or asyncio.get_running_loop().time() >= deadline:
                    raise RuntimeError("scoped Nav2 controller service unavailable")
                await asyncio.sleep(0.025)
            request = ChangeState.Request()
            request.transition.id = transition
            pending = client.call_async(request)
            while not pending.done():
                if process.returncode is not None or asyncio.get_running_loop().time() >= deadline:
                    pending.cancel()
                    raise RuntimeError("scoped Nav2 controller transition timed out")
                await asyncio.sleep(0.025)
            reply = pending.result()
            if reply is None or not reply.success:
                raise RuntimeError("scoped Nav2 controller transition rejected")
        finally:
            self._node.destroy_client(client)

    @asynccontextmanager
    async def open(self, execution_id: str) -> AsyncIterator[str]:
        async with self._lock:
            if not execution_id or not self._params.is_file():
                raise RuntimeError("qualified Nav2 controller unavailable")
            if any(
                name == "controller_server"
                for name, _ in self._node.get_node_names_and_namespaces()
            ):
                raise RuntimeError("a Nav2 controller is already present")
            topic = f"/control/navigation/run_{secrets.token_hex(16)}/cmd_vel"
            sim = str(bool(self._node.get_parameter("use_sim_time").value)).lower()
            controller = await asyncio.create_subprocess_exec(
                "ros2",
                "run",
                "nav2_controller",
                "controller_server",
                "--ros-args",
                "--params-file",
                str(self._params),
                "-p",
                f"use_sim_time:={sim}",
                "-r",
                f"cmd_vel:={topic}",
                "-r",
                f"/cmd_vel:={topic}",
                start_new_session=True,
            )
            try:
                await self._transition(Transition.TRANSITION_CONFIGURE, controller)
                await self._transition(Transition.TRANSITION_ACTIVATE, controller)
                yield topic
            finally:
                # Stop the exact process group created for this run.
                with suppress(ProcessLookupError):
                    os.killpg(controller.pid, signal.SIGINT)
                try:
                    await asyncio.wait_for(controller.wait(), 4)
                except asyncio.TimeoutError:
                    with suppress(ProcessLookupError):
                        os.killpg(controller.pid, signal.SIGKILL)
                    await controller.wait()
