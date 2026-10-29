"""Nav2 goal lifecycle. This process does not publish /cmd_vel.

Controller and behavior output is remapped to /safety/command_in. A rejected
cancel leaves the current goal in place and does not start a replacement.
"""

import math
from dataclasses import dataclass
from typing import Protocol

from senior_control.models import MapRef, OperationResult, OperationStatus, Pose2D

SAFETY_COMMAND_TOPIC = "/safety/command_in"
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
        self._active: str | None = None
        self._cancel_rejected = False

    async def navigate(self, goal: MapGoal) -> str:
        if self._cancel_rejected or self._active is not None:
            raise NavigationRejected(self._active or "")
        execution_id = await self._executor.start(goal)
        self._active = execution_id
        return execution_id

    async def cancel(self, execution_id: str) -> OperationResult:
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


def main() -> None:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient

    class Nav2Executor:
        def __init__(self, node: rclpy.node.Node) -> None:
            self._node = node
            self._client: ActionClient = ActionClient(node, NavigateToPose, "navigate_to_pose")
            self._handles: dict[str, object] = {}
            self._counter = 0

        async def start(self, goal: MapGoal) -> str:
            if not self._client.wait_for_server(timeout_sec=1.0):
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
            rclpy.spin_until_future_complete(self._node, send_future, timeout_sec=2.0)
            if not send_future.done():
                raise NavigationRejected("")
            handle = send_future.result()
            if handle is None or not handle.accepted:
                raise NavigationRejected("")
            self._counter += 1
            execution_id = f"nav-{self._counter}"
            self._handles[execution_id] = handle
            return execution_id

        async def cancel(self, execution_id: str) -> OperationResult:
            handle = self._handles.get(execution_id)
            if handle is None:
                return OperationResult(execution_id, OperationStatus.FAILED)
            cancel_goal = getattr(handle, "cancel_goal_async", None)
            if cancel_goal is None:
                return OperationResult(execution_id, OperationStatus.FAILED)
            cancel_future = cancel_goal()
            rclpy.spin_until_future_complete(self._node, cancel_future, timeout_sec=2.0)
            if not cancel_future.done():
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
                    rclpy.spin_until_future_complete(self._node, result_future, timeout_sec=2.0)
                    result_arrived = bool(result_future.done())
            status = cancel_status(return_code, result_arrived)
            if status == OperationStatus.SUCCEEDED:
                self._handles.pop(execution_id, None)
            return OperationResult(execution_id, status)

    rclpy.init()
    node_type = create_navigation_node()
    node = node_type()
    node.goal_adapter = NavigationAdapter(Nav2Executor(node))
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
