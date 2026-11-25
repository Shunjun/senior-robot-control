"""Own one Nav2 controller publisher for one navigation execution."""

import asyncio
import os
import secrets
import signal
from collections.abc import AsyncIterator
from contextlib import AsyncExitStack, asynccontextmanager, suppress
from pathlib import Path

from ament_index_python.packages import get_package_prefix
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.node import Node

from vehicle_core.launch_recovery import owner_command


class DedicatedController:
    def __init__(
        self, node: Node, params: Path, *, odometry_topic: str = "/wheel/odom", permit_changed=None
    ) -> None:
        self._permit_changed = permit_changed
        self._node = node
        self._params = params
        self._odometry_topic = odometry_topic
        self._lock = asyncio.Lock()
        self._standby: tuple[AsyncExitStack, str] | None = None
        self._in_use = False
        self._closing = False
        self._owned: dict[AsyncExitStack, asyncio.Task | None] = {}

    def available(self) -> bool:
        return self._standby is not None or self._in_use

    async def _settle(self, stack: AsyncExitStack) -> None:
        # One owner task per stack, shared by route exit and controller shutdown.
        # Cancelling a waiter must not interrupt process-group settlement.
        if stack not in self._owned:
            return
        task = self._owned[stack]
        if task is None:
            task = self._owned[stack] = asyncio.create_task(stack.aclose())
        await asyncio.shield(task)
        self._owned.pop(stack, None)

    async def close(self) -> None:
        async with self._lock:
            self._closing = True
            standby, self._standby = self._standby, None
            if standby is not None:
                self._owned.setdefault(standby[0], None)
            stacks = tuple(self._owned)
        # Never hold the state lock while waiting for process exit. Observe every
        # stack, including failed/interrupted route cleanup, before claiming exit.
        results = await asyncio.gather(
            *(self._settle(stack) for stack in stacks), return_exceptions=True
        )
        for result in results:
            if isinstance(result, BaseException):
                raise RuntimeError("dedicated controller cleanup incomplete") from result
        # Compatible: close also retires the static stack before a live handoff;
        # successful settlement must allow the existing warmer to run again.
        self._closing = False

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
            # Compatible: retain the original SIGINT -> exact-group SIGKILL
            # escalation; an unconfirmed kill must fail shutdown, never hang it.
            await asyncio.wait_for(process.wait(), 4)
        if process.returncode is None:
            raise RuntimeError("dedicated controller process exit unconfirmed")

    async def _start(
        self, stack: AsyncExitStack, package: str, name: str, *remappings: str
    ) -> None:
        process = await asyncio.create_subprocess_exec(
            *owner_command(
                [
                    str(Path(get_package_prefix(package)) / "lib" / package / name),
                    "--ros-args",
                    "--params-file",
                    str(self._params),
                    "-p",
                    f"use_sim_time:={str(bool(self._node.get_parameter('use_sim_time').value)).lower()}",
                    "-p",
                    f"odom_topic:={self._odometry_topic}",
                    *remappings,
                ]
            ),
            start_new_session=True,
        )
        stack.push_async_callback(self._stop, process)
        await self._transition(name, Transition.TRANSITION_CONFIGURE, process)
        await self._transition(name, Transition.TRANSITION_ACTIVATE, process)

    async def warm(self) -> None:
        stack = None
        ready = False
        try:
            async with self._lock:
                if self._closing:
                    raise RuntimeError("dedicated controller is closing")
                if self.available():
                    return
                if self._owned:
                    raise RuntimeError("dedicated controller cleanup pending or failed")
                if not self._params.is_file():
                    raise RuntimeError("qualified Nav2 controller unavailable")
                if any(
                    name in {"controller_server", "bt_navigator"}
                    for name, _ in self._node.get_node_names_and_namespaces()
                ):
                    raise RuntimeError("a Nav2 controller or navigator is already present")
                stack = AsyncExitStack()
                self._owned[stack] = None
                from senior_navigation.parameters import rotate_role_permit

                permit = await asyncio.to_thread(rotate_role_permit, self._params, "local")
                if self._permit_changed is not None:
                    self._permit_changed(permit)
                    stack.callback(self._permit_changed, "")
                run_id = secrets.token_hex(16)
                topic = f"/control/navigation/run_{run_id}/cmd_vel"
                await self._start(
                    stack,
                    "nav2_controller",
                    "controller_server",
                    "-r",
                    f"cmd_vel:={topic}",
                    "-r",
                    f"/cmd_vel:={topic}",
                    "-r",
                    f"odom:={self._odometry_topic}",
                )
                await self._start(stack, "nav2_bt_navigator", "bt_navigator")
                self._standby = stack, topic
                ready = True
        finally:
            if stack is not None and not ready:
                await self._settle(stack)

    @asynccontextmanager
    async def open(self, execution_id: str) -> AsyncIterator[str]:
        async with self._lock:
            if self._closing or not execution_id or self._standby is None or self._in_use:
                raise RuntimeError("scoped Nav2 controller is not ready")
            (stack, topic), self._standby = self._standby, None
            self._owned.setdefault(stack, None)
            self._in_use = True
        try:
            yield topic
        finally:
            try:
                await self._settle(stack)
            finally:
                self._in_use = False
