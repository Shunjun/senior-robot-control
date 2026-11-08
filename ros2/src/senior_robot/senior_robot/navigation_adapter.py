"""Nav2 goal lifecycle. This process does not publish /cmd_vel.

Controller and recovery output go to separate arbiter inputs. A rejected
cancel leaves the current goal in place and does not start a replacement.
"""

import asyncio
import logging
import math
import time
from collections.abc import Callable, Sequence
from contextlib import AbstractAsyncContextManager, AsyncExitStack
from dataclasses import dataclass
from typing import Protocol

from senior_control.models import MapRef, OperationResult, OperationStatus, Pose2D
from senior_control.navigation import QualifiedRoute, RoutePlan, RouteProgress, RouteStatus

SAFETY_COMMAND_TOPIC = "/control/recovery_cmd"
ERROR_NONE = 0
ERROR_GOAL_TERMINATED = 3


def cancel_status(return_code: int | None, result_arrived: bool) -> OperationStatus:
    """Success means the goal is terminal. An accepted cancel is not enough.

    ``ERROR_GOAL_TERMINATED`` means the goal had already finished. That is not
    a rejected cancel.
    """

    if return_code == ERROR_GOAL_TERMINATED:
        return OperationStatus.SUCCEEDED
    if return_code != ERROR_NONE or not result_arrived:
        return OperationStatus.FAILED
    return OperationStatus.SUCCEEDED


@dataclass(frozen=True)
class MapGoal:
    map_ref: MapRef
    pose: Pose2D


class NavigationRejected(Exception):
    def __init__(self, execution_id: str) -> None:
        super().__init__(execution_id)
        self.execution_id = execution_id


class GoalExecutor(Protocol):
    async def start(self, goal: MapGoal) -> str:
        """Start one goal and return its execution id."""

    async def cancel(self, execution_id: str) -> OperationResult:
        """Cancel that execution. A failed status rejects the cancel."""


class NavigationAdapter:
    def __init__(self, executor: GoalExecutor) -> None:
        self._executor = executor
        self._goal_lock = asyncio.Lock()
        self._active: str | None = None
        self._cancel_rejected = False

    async def navigate(self, goal: MapGoal) -> str:
        async with self._goal_lock:
            if self._cancel_rejected or self._active is not None:
                raise NavigationRejected(self._active or "")
            execution_id = await self._executor.start(goal)
            self._active = execution_id
            return execution_id

    async def run_route(
        self,
        ref: MapRef,
        generation: str,
        poses: Sequence[Pose2D],
        *,
        qualified: Callable[[MapRef, str], bool],
        goal_changed: Callable[[int | None], None] | None = None,
    ) -> bool:
        get_result = getattr(self._executor, "result", None)
        if not callable(get_result):
            return False

        index = 0

        async def go(pose: Pose2D) -> bool:
            nonlocal index
            async with self._goal_lock:
                if self._active is not None or self._cancel_rejected:
                    return False
                if not qualified(ref, generation):
                    return False
                self._active = execution_id = await self._executor.start(MapGoal(ref, pose))
            result = asyncio.create_task(get_result(execution_id))
            try:
                if goal_changed is not None and qualified(ref, generation):
                    goal_changed(index)
                index += 1
                while qualified(ref, generation):
                    done, _ = await asyncio.wait({result}, timeout=0.05)
                    if done:
                        succeeded = result.result()
                        async with self._goal_lock:
                            if self._active != execution_id or self._cancel_rejected:
                                return False
                            self._active = None
                        return succeeded
                if goal_changed is not None:
                    goal_changed(None)
                await self.cancel(execution_id)
                return False
            except asyncio.CancelledError:
                if goal_changed is not None:
                    goal_changed(None)
                await self.cancel(execution_id)
                raise
            except Exception:
                if goal_changed is not None:
                    goal_changed(None)
                await self.cancel(execution_id)
                raise
            finally:
                if goal_changed is not None:
                    goal_changed(None)
                if not result.done():
                    result.cancel()
                    await asyncio.gather(result, return_exceptions=True)

        return await QualifiedRoute(go, qualified).run(ref, generation, poses)

    @property
    def active_goal(self) -> str | None:
        return self._active

    async def cancel(self, execution_id: str) -> OperationResult:
        async with self._goal_lock:
            if not execution_id.strip():
                return OperationResult("unknown", OperationStatus.FAILED)
            if self._active is None and not self._cancel_rejected:
                return OperationResult(execution_id, OperationStatus.SUCCEEDED)
            if self._active != execution_id:
                return OperationResult(execution_id, OperationStatus.FAILED)
            result = await self._executor.cancel(execution_id)
            if result.status == OperationStatus.SUCCEEDED:
                self._active = None
                self._cancel_rejected = False
                return result
            if result.status == OperationStatus.FAILED:
                self._cancel_rejected = True
            return result


@dataclass
class _Route:
    plan: RoutePlan
    started: asyncio.Future[bool]
    status: RouteStatus = "starting"
    active_waypoint_index: int | None = None
    cancel_requested: bool = False
    task: asyncio.Task[None] | None = None

    def progress(self) -> RouteProgress:
        return RouteProgress(
            execution_id=self.plan.execution_id,
            task_id=self.plan.task_id,
            map_generation=self.plan.map_generation,
            status=self.status,
            active_waypoint_index=self.active_waypoint_index,
        )


class RouteService:
    """One local route; cancellation revokes motion before awaiting the Nav2 result."""

    def __init__(
        self,
        adapter: NavigationAdapter,
        *,
        qualified: Callable[[MapRef, str], bool],
        goal_enabled: Callable[[bool], None] | None = None,
        prepare: Callable[[str], AbstractAsyncContextManager[str]] | None = None,
        bind_source: Callable[[str, str, Callable[[], bool]], bool] | None = None,
        clear_source: Callable[[str], None] | None = None,
        source_active: Callable[[str], bool] | None = None,
    ) -> None:
        self._adapter = adapter
        self._qualified = qualified
        self._goal_enabled = goal_enabled
        self._prepare = prepare
        self._bind_source = bind_source
        self._clear_source = clear_source
        self._source_active = source_active
        self._current: _Route | None = None
        self._terminal: dict[str, RouteProgress] = {}

    async def start(self, plan: RoutePlan, *, wait_for_goal: bool = True) -> bool:
        if self._current is not None:
            if (
                self._current.plan != plan
                or self._current.cancel_requested
                or self._current.status == "cancel_pending"
            ):
                return False
            return await asyncio.shield(self._current.started) if wait_for_goal else True
        if (
            plan.execution_id in self._terminal
            or self._adapter.active_goal is not None
            or not self._qualified(plan.map_ref, plan.map_generation)
        ):
            return False
        route = _Route(plan, asyncio.get_running_loop().create_future())
        self._current = route
        route.task = asyncio.create_task(self._run(route))
        return await asyncio.shield(route.started) if wait_for_goal else True

    async def _run(self, route: _Route) -> None:
        plan = route.plan

        topic = None
        binding_failed = False

        def qualified(ref: MapRef, generation: str) -> bool:
            return (
                not route.cancel_requested
                and not binding_failed
                and self._qualified(ref, generation)
                and (
                    route.active_waypoint_index is None
                    or self._source_active is None
                    or self._source_active(plan.execution_id)
                )
            )

        def changed(index: int | None) -> None:
            nonlocal binding_failed
            if index is None:
                self._revoke(route)
            elif topic is not None and self._bind_source is not None and not self._bind_source(
                plan.execution_id, topic, lambda: qualified(plan.map_ref, plan.map_generation)
            ):
                binding_failed = True
                return
            route.active_waypoint_index = index
            if index is not None and self._goal_enabled is not None:
                self._goal_enabled(True)
            if index is not None and not route.started.done():
                route.status = "running"
                route.started.set_result(True)

        succeeded = False
        try:
            async with AsyncExitStack() as stack:
                if self._prepare is not None:
                    topic = await stack.enter_async_context(self._prepare(plan.execution_id))
                succeeded = await self._adapter.run_route(
                    plan.map_ref, plan.map_generation,
                    tuple(Pose2D(*pose) for pose in plan.poses),
                    qualified=qualified, goal_changed=changed,
                )
        except Exception:
            # Unknown terminality is retained when the adapter still owns a goal.
            logging.getLogger(__name__).exception("qualified Nav2 route failed")
        finally:
            changed(None)
            if not route.started.done():
                route.started.set_result(False)
            if self._adapter.active_goal is not None:
                route.status = "cancel_pending"
            else:
                self._finish(route, "canceled" if route.cancel_requested else (
                    "succeeded" if succeeded else "failed"
                ))

    def _revoke(self, route: _Route) -> None:
        if self._clear_source is not None:
            self._clear_source(route.plan.execution_id)
        if self._goal_enabled is not None:
            self._goal_enabled(False)

    def _finish(self, route: _Route, status: RouteStatus) -> None:
        route.status = status
        self._terminal[route.plan.execution_id] = route.progress()
        if len(self._terminal) > 64:
            self._terminal.pop(next(iter(self._terminal)))
        if self._current is route:
            self._current = None

    def status(self, execution_id: str) -> RouteProgress | None:
        current = self._current
        if current is not None and current.plan.execution_id == execution_id:
            return current.progress()
        return self._terminal.get(execution_id)

    @property
    def current(self) -> RouteProgress | None:
        return self._current.progress() if self._current is not None else None

    async def cancel(self, execution_id: str) -> bool:
        route = self._current
        if route is None or route.plan.execution_id != execution_id:
            # Unknown requests cannot affect the current route.
            return execution_id in self._terminal
        route.cancel_requested = True
        self._revoke(route)
        if route.task is not None:
            await asyncio.shield(route.task)
        if self._current is not route:
            return True
        goal = self._adapter.active_goal
        if goal is not None:
            result = await self._adapter.cancel(goal)
            if result.status is not OperationStatus.SUCCEEDED:
                return False
        self._finish(route, "canceled")
        return True


def create_navigation_node() -> type:
    """ROS node type. Imported only when rclpy is installed."""
    from geometry_msgs.msg import Twist
    from rclpy.node import Node

    class NavigationNode(Node):
        def __init__(self) -> None:
            super().__init__("navigation_adapter")
            self.declare_parameter("command_topic", SAFETY_COMMAND_TOPIC)
            topic = str(self.get_parameter("command_topic").value).strip()
            if not topic or self.resolve_topic_name(topic) == "/cmd_vel":
                raise ValueError("navigation adapter must not publish /cmd_vel")
            self._publisher = self.create_publisher(Twist, topic, 10)

        def publish_recovery(self, vx: float, wz: float) -> None:
            message = Twist()
            message.linear.x = vx
            message.angular.z = wz
            self._publisher.publish(message)

    return NavigationNode


def _yaw_quaternion(yaw: float) -> tuple[float, float]:
    return math.sin(yaw * 0.5), math.cos(yaw * 0.5)


def create_planner_readiness(node) -> Callable[[], bool]:
    """Check a fresh ACTIVE lifecycle response, without waiting inside ROS callbacks."""
    from lifecycle_msgs.msg import State
    from lifecycle_msgs.srv import GetState

    client = node.create_client(GetState, "/planner_server/get_state")
    request = None
    requested_at = float("-inf")
    active_until = float("-inf")

    def ready() -> bool:
        nonlocal request, requested_at, active_until
        now = time.monotonic()
        if not client.service_is_ready():
            active_until = float("-inf")
            return False
        if request is not None:
            if request.done():
                response = request.result()
                active_until = (
                    now + 0.75
                    if response is not None
                    and response.current_state.id == State.PRIMARY_STATE_ACTIVE
                    else float("-inf")
                )
                request = None
            elif now - requested_at >= 0.75:
                request.cancel()
                request = None
                active_until = float("-inf")
        if request is None and now - requested_at >= 0.25:
            request = client.call_async(GetState.Request())
            requested_at = now
        return now < active_until

    return ready


def create_nav2_adapter(node) -> NavigationAdapter:
    import rclpy
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient

    async def wait_for_result(future, timeout: float) -> bool:
        deadline = asyncio.get_running_loop().time() + timeout
        while not future.done() and asyncio.get_running_loop().time() < deadline:
            await asyncio.sleep(0.01)
        return bool(future.done())

    class Nav2Executor:
        def __init__(self, node: rclpy.node.Node) -> None:
            self._node = node
            self._client: ActionClient = ActionClient(node, NavigateToPose, "navigate_to_pose")
            self._handles: dict[str, object] = {}
            self._counter = 0
            self._last_finished: str | None = None
            self._start_unconfirmed = False

        async def start(self, goal: MapGoal) -> str:
            deadline = asyncio.get_running_loop().time() + 2.0
            while not self._client.server_is_ready():
                if self._start_unconfirmed or asyncio.get_running_loop().time() >= deadline:
                    raise NavigationRejected("")
                await asyncio.sleep(0.02)
            if self._start_unconfirmed:
                raise NavigationRejected("")
            message = NavigateToPose.Goal()
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self._node.get_clock().now().to_msg()
            pose.pose.position.x = goal.pose.x
            pose.pose.position.y = goal.pose.y
            pose.pose.orientation.z, pose.pose.orientation.w = _yaw_quaternion(goal.pose.yaw)
            message.pose = pose
            send_future = self._client.send_goal_async(message)
            if not await wait_for_result(send_future, 0.8):
                # Acceptance may arrive after the deadline. Hold the adapter closed
                # and cancel that specific handle if Nav2 eventually accepted it.
                self._start_unconfirmed = True

                def cancel_late_acceptance(future) -> None:
                    try:
                        late = future.result()
                        if late is not None and late.accepted:
                            late.cancel_goal_async()
                    except Exception:
                        self._node.get_logger().error("Late Nav2 acceptance could not be canceled")

                send_future.add_done_callback(cancel_late_acceptance)
                raise NavigationRejected("")
            handle = send_future.result()
            if handle is None or not handle.accepted:
                raise NavigationRejected("")
            self._counter += 1
            execution_id = f"nav-{self._counter}"
            self._handles[execution_id] = handle
            return execution_id

        async def result(self, execution_id: str) -> bool:
            handle = self._handles[execution_id]
            result_future = handle.get_result_async()
            while not result_future.done():
                await asyncio.sleep(0.01)
            response = result_future.result()
            succeeded = response is not None and response.status == GoalStatus.STATUS_SUCCEEDED
            self._handles.pop(execution_id, None)
            self._last_finished = execution_id
            return succeeded

        async def cancel(self, execution_id: str) -> OperationResult:
            handle = self._handles.get(execution_id)
            if handle is None:
                status = (
                    OperationStatus.SUCCEEDED
                    if execution_id == self._last_finished
                    else OperationStatus.FAILED
                )
                return OperationResult(execution_id, status)
            cancel_goal = getattr(handle, "cancel_goal_async", None)
            if cancel_goal is None:
                return OperationResult(execution_id, OperationStatus.FAILED)
            cancel_future = cancel_goal()
            if not await wait_for_result(cancel_future, 0.8):
                return OperationResult(execution_id, OperationStatus.FAILED)
            response = cancel_future.result()
            return_code = None if response is None else int(response.return_code)
            result_arrived = return_code == ERROR_GOAL_TERMINATED
            if return_code == ERROR_NONE:
                get_result = getattr(handle, "get_result_async", None)
                if get_result is None:
                    result_arrived = False
                else:
                    result_future = get_result()
                    result_arrived = await wait_for_result(result_future, 0.8)
            status = cancel_status(return_code, result_arrived)
            if status == OperationStatus.SUCCEEDED:
                self._handles.pop(execution_id, None)
            return OperationResult(execution_id, status)

    return NavigationAdapter(Nav2Executor(node))


def main() -> None:
    import rclpy
    from rclpy.executors import ExternalShutdownException

    rclpy.init()
    node = None
    try:
        node = create_navigation_node()()
        node.goal_adapter = create_nav2_adapter(node)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if node is not None:
                node.destroy_node()
        finally:
            # //Compatible: Humble's signal handler can shut down the context before spin exits.
            rclpy.try_shutdown()
