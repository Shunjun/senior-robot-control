"""Publish arbitration results to the safety gate.

This process does not publish /cmd_vel. Lease expiry, estop, disconnect, and a
zero tick are written to /safety/command_in immediately so the gate does not
keep the previous command until its own TTL. The watchdog reads the injected
monotonic clock and keeps running when ROS time is frozen. The timer period is
simulation configuration, not a physical braking guarantee.
"""

import secrets
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.clock import Clock as RosClock
from rclpy.clock import ClockType
from rclpy.node import Node
from rclpy.parameter import Parameter
from senior_motion_msgs.msg import FinalCommand

from senior_control.clock import Clock, SystemClock
from senior_control.models import Velocity
from senior_control.protocol import Envelope
from senior_control.safety.arbiter import Arbiter, CommandResult, PersistentEstop


class CommandNode(Node):
    def __init__(
        self,
        arbiter: Arbiter | None = None,
        clock: Clock | None = None,
        *,
        parameter_overrides: list[Parameter] | None = None,
        require_server_heartbeat: bool = False,
    ) -> None:
        super().__init__("command_node", parameter_overrides=parameter_overrides or [])
        self.declare_parameter("robot_id", "robot")
        self.declare_parameter("safety_state_path", "")
        self.declare_parameter("heartbeat_seconds", 0.2)
        self.declare_parameter("disconnect_seconds", 2.0 if require_server_heartbeat else 1.0)
        self.declare_parameter("command_ttl_seconds", 0.3)
        self.declare_parameter("command_topic", "/safety/command_in")
        self.declare_parameter("final_command_topic", "/safety/final_command")
        topic = str(self.get_parameter("command_topic").value).strip()
        if not topic or self.resolve_topic_name(topic) == "/cmd_vel":
            raise ValueError("command node must publish only to the safety input")
        self._monotonic_clock = clock if clock is not None else SystemClock()
        self._estop_store = None
        if require_server_heartbeat:
            path = str(self.get_parameter("safety_state_path").value)
            if not path:
                raise ValueError("production command node requires safety_state_path")
            self._estop_store = PersistentEstop(
                Path(path), str(self.get_parameter("robot_id").value)
            )
        if arbiter is None:
            self._arbiter = Arbiter(
                robot_id=str(self.get_parameter("robot_id").value),
                heartbeat_seconds=float(self.get_parameter("heartbeat_seconds").value),
                disconnect_seconds=float(self.get_parameter("disconnect_seconds").value),
                command_ttl_seconds=float(self.get_parameter("command_ttl_seconds").value),
                require_server_heartbeat=require_server_heartbeat,
                initial_estop=self._estop_store.latched if self._estop_store else False,
            )
        else:
            self._arbiter = arbiter
        self._publisher = self.create_publisher(Twist, topic, 10)
        self._final_publisher = self.create_publisher(
            FinalCommand, str(self.get_parameter("final_command_topic").value), 10
        )
        self._owner_boot = secrets.token_hex(16)
        self._generation = 1
        self._last_server_epoch = 0
        self._active = False
        heartbeat = float(self.get_parameter("heartbeat_seconds").value)
        ttl = float(self.get_parameter("command_ttl_seconds").value)
        period = min(heartbeat, ttl) / 2.0
        if period <= 0.0:
            period = 0.05
        steady = RosClock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(period, self.publish_command, clock=steady)

    def submit(
        self,
        envelope: Envelope,
        received_at: float | None = None,
        *,
        local_deadline: float | None = None,
    ) -> CommandResult:
        result = self._arbiter.accept(
            envelope,
            self._monotonic_clock.monotonic(),
            received_at=received_at,
            local_deadline=local_deadline,
        )
        if result.accepted and envelope.type != "drive.velocity":
            self._generation += 1
            self._active = False
            self._publish_barrier()
        self.publish_command()
        if envelope.type == "safety.estop" and self._estop_store is not None:
            return self._estop_store.commit(envelope, result)
        return result

    def server_heartbeat(self, connection_epoch: int, deadline: float) -> bool:
        accepted = self._arbiter.server_heartbeat(
            connection_epoch, self._monotonic_clock.monotonic(), deadline
        )
        if accepted and connection_epoch != self._last_server_epoch:
            self._last_server_epoch = connection_epoch
            self._generation += 1
            self._active = False
            self._publish_barrier()
        return accepted

    def disconnect(self, connection_epoch: int) -> None:
        self._arbiter.disconnect(connection_epoch)
        self._generation += 1
        self._active = False
        self._publish_barrier()
        self.publish_command()

    def publish_command(self) -> None:
        now = self._monotonic_clock.monotonic()
        final = self._arbiter.final_command(now)
        if final is None:
            if self._active:
                self._generation += 1
                self._active = False
            self._publish_barrier()
            self._publish(Velocity(0.0, 0.0))
            return
        velocity, session, epoch, sequence, deadline = final
        msg = FinalCommand()
        msg.robot_id = str(self.get_parameter("robot_id").value)
        msg.owner_boot_id = self._owner_boot
        msg.session_id = session
        msg.generation = self._generation
        msg.connection_epoch = epoch
        msg.sequence = sequence
        msg.steady_deadline = deadline
        msg.vx, msg.wz = velocity.vx, velocity.wz
        self._final_publisher.publish(msg)
        self._active = True
        self._publish(velocity)

    def _publish_barrier(self) -> None:
        msg = FinalCommand()
        msg.robot_id = str(self.get_parameter("robot_id").value)
        msg.owner_boot_id = self._owner_boot
        msg.generation = self._generation
        self._final_publisher.publish(msg)

    def _publish(self, velocity: Velocity) -> None:
        message = Twist()
        message.linear.x = velocity.vx
        message.angular.z = velocity.wz
        self._publisher.publish(message)

    def destroy_node(self):
        if self._estop_store is not None:
            self._estop_store.close()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = CommandNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
