"""Own one Nav2 controller publisher for one navigation execution."""

import asyncio
import os
import secrets
import signal
from collections.abc import AsyncIterator
from contextlib import AsyncExitStack, asynccontextmanager, suppress
from pathlib import Path

from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
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
        observer = self._node.create_client(GetState, f"/{name}/get_state")
        expected = (
            State.PRIMARY_STATE_INACTIVE
            if transition == Transition.TRANSITION_CONFIGURE
            else State.PRIMARY_STATE_ACTIVE
        )
        clock = asyncio.get_running_loop()
        deadline = clock.time() + 10
        request_future = state_future = None
        try:
            while not (client.service_is_ready() and observer.service_is_ready()):
                if process.returncode is not None or clock.time() >= deadline:
                    raise RuntimeError(f"{name} lifecycle services unavailable")
                await asyncio.sleep(0.025)
            request = ChangeState.Request()
            request.transition.id = transition
            request_future = client.call_async(request)
            deadline = clock.time() + 10
            last_query = float("-inf")
            while process.returncode is None and clock.time() < deadline:
                if request_future.done():
                    response = request_future.result()
                    if response is not None and not response.success:
                        raise RuntimeError(f"{name} lifecycle transition {transition} rejected")
                if state_future is not None and state_future.done():
                    state = state_future.result()
                    if state is not None and state.current_state.id == expected:
                        return
                    state_future = None
                if clock.time() - last_query >= 0.25:
                    if state_future is not None:
                        state_future.cancel()
                    state_future = observer.call_async(GetState.Request())
                    last_query = clock.time()
                await asyncio.sleep(0.025)
            raise RuntimeError(f"{name} lifecycle transition {transition} was unconfirmed")
        finally:
            for pending in (request_future, state_future):
                if pending is not None and not pending.done():
                    pending.cancel()
            self._node.destroy_client(observer)
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
                "-r",
                "odom:=/wheel/odom",
            )
            await self._start(stack, "nav2_bt_navigator", "bt_navigator")
            yield topic
