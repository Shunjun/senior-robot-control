"""Publish arbitration results to the safety gate.

This process does not publish /cmd_vel. Lease expiry, estop, disconnect, and a
zero tick are written to /safety/command_in immediately so the gate does not
keep the previous command until its own TTL. The watchdog reads the injected
monotonic clock and keeps running when ROS time is frozen. The timer period is
simulation configuration, not a physical braking guarantee.
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.clock import Clock as RosClock
from rclpy.clock import ClockType
from rclpy.node import Node
from rclpy.parameter import Parameter

from senior_control.clock import Clock, SystemClock
from senior_control.models import Velocity
from senior_control.protocol import Envelope
from senior_control.safety.arbiter import Arbiter, CommandResult


class CommandNode(Node):
    def __init__(
        self,
        arbiter: Arbiter | None = None,
        clock: Clock | None = None,
        *,
        parameter_overrides: list[Parameter] | None = None,
    ) -> None:
        super().__init__("command_node", parameter_overrides=parameter_overrides or [])
        self.declare_parameter("robot_id", "robot")
        self.declare_parameter("heartbeat_seconds", 0.2)
        self.declare_parameter("disconnect_seconds", 1.0)
        self.declare_parameter("command_ttl_seconds", 0.3)
        self.declare_parameter("command_topic", "/safety/command_in")
        topic = str(self.get_parameter("command_topic").value).strip()
        if not topic or self.resolve_topic_name(topic) == "/cmd_vel":
            raise ValueError("command node must publish only to the safety input")
        self._clock = clock if clock is not None else SystemClock()
        if arbiter is None:
            self._arbiter = Arbiter(
                robot_id=str(self.get_parameter("robot_id").value),
                heartbeat_seconds=float(self.get_parameter("heartbeat_seconds").value),
                disconnect_seconds=float(self.get_parameter("disconnect_seconds").value),
                command_ttl_seconds=float(self.get_parameter("command_ttl_seconds").value),
            )
        else:
            self._arbiter = arbiter
        self._publisher = self.create_publisher(Twist, topic, 10)
        heartbeat = float(self.get_parameter("heartbeat_seconds").value)
        ttl = float(self.get_parameter("command_ttl_seconds").value)
        period = min(heartbeat, ttl) / 2.0
        if period <= 0.0:
            period = 0.05
        steady = RosClock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(period, self.publish_command, clock=steady)

    def submit(self, envelope: Envelope) -> CommandResult:
        result = self._arbiter.accept(envelope, self._clock.monotonic())
        self.publish_command()
        return result

    def disconnect(self, connection_epoch: int) -> None:
        self._arbiter.disconnect(connection_epoch)
        self.publish_command()

    def publish_command(self) -> None:
        self._publish(self._arbiter.tick(self._clock.monotonic()))

    def publish_velocity(self, velocity: Velocity) -> None:
        """Write to the safety input only. Used by the agent adapter to hold or zero motion."""
        self._publish(velocity)

    def _publish(self, velocity: Velocity) -> None:
        message = Twist()
        message.linear.x = velocity.vx
        message.angular.z = velocity.wz
        self._publisher.publish(message)


def main() -> None:
    rclpy.init()
    node = CommandNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
