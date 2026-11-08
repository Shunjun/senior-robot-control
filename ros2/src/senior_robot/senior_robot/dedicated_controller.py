"""Own one Nav2 controller publisher for one navigation execution."""

import asyncio
import os
import secrets
import signal
from collections.abc import AsyncIterator
from contextlib import AsyncExitStack, asynccontextmanager, suppress
from pathlib import Path

from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from rclpy.node import Node


class DedicatedController:
    def __init__(self, node: Node, params: Path) -> None:
        self._node = node
        self._params = params
        self._lock = asyncio.Lock()

    async def _transition(
        self, name: str, transition: int, process: asyncio.subprocess.Process
    ) -> None:
        client = self._node.create_client(ChangeState, f"/{name}/change_state")
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

    async def _stop(self, process: asyncio.subprocess.Process) -> None:
        # Stop the exact process group created for this run.
        with suppress(ProcessLookupError):
            os.killpg(process.pid, signal.SIGINT)
        try:
            await asyncio.wait_for(process.wait(), 4)
        except asyncio.TimeoutError:
            with suppress(ProcessLookupError):
                os.killpg(process.pid, signal.SIGKILL)
            await process.wait()

    async def _start(
        self, stack: AsyncExitStack, package: str, name: str, *remappings: str
    ) -> None:
        process = await asyncio.create_subprocess_exec(
            "ros2",
            "run",
            package,
            name,
            "--ros-args",
            "--params-file",
            str(self._params),
            *remappings,
            start_new_session=True,
        )
        stack.push_async_callback(self._stop, process)
        await self._transition(name, Transition.TRANSITION_CONFIGURE, process)
        await self._transition(name, Transition.TRANSITION_ACTIVATE, process)

    @asynccontextmanager
    async def open(self, execution_id: str) -> AsyncIterator[str]:
        async with self._lock, AsyncExitStack() as stack:
            if not execution_id or not self._params.is_file():
                raise RuntimeError("qualified Nav2 controller unavailable")
            if any(
                name in {"controller_server", "bt_navigator"}
                for name, _ in self._node.get_node_names_and_namespaces()
            ):
                raise RuntimeError("a Nav2 controller or navigator is already present")
            topic = f"/control/navigation/run_{secrets.token_hex(16)}/cmd_vel"
            await self._start(
                stack,
                "nav2_controller",
                "controller_server",
                "-r",
                f"cmd_vel:={topic}",
                "-r",
                f"/cmd_vel:={topic}",
            )
            await self._start(stack, "nav2_bt_navigator", "bt_navigator")
            yield topic
