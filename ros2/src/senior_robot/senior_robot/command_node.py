"""Publish arbitration results to the safety gate.

This process does not publish /cmd_vel. Lease expiry, estop, disconnect, and a
zero tick are written to /safety/command_in immediately so the gate does not
keep the previous command until its own TTL. The watchdog reads the injected
monotonic clock and keeps running when ROS time is frozen. The timer period is
simulation configuration, not a physical braking guarantee.
"""

import re
import secrets
import time
from collections import deque
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import rclpy
from geometry_msgs.msg import Twist
from rclpy.clock import Clock as RosClock
from rclpy.clock import ClockType
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.subscription import Subscription
from senior_motion_msgs.msg import DriveExecutionFeedback, FinalCommand

from senior_control.clock import Clock, SystemClock
from senior_control.models import Velocity
from senior_control.protocol import (
    DriveExecutionFeedback as ExecutionFeedback,
)
from senior_control.protocol import (
    Envelope,
    VelocityEnvelope,
)
from senior_control.safety.arbiter import Arbiter, CommandResult, PersistentEstop


@dataclass
class _NavigationSource:
    execution_id: str
    topic: str
    offset: float
    subscription: Subscription
    bound_at_ns: int
    qualified: Callable[[], bool]
    sequence: int = 0
    last_stamp: int = 0


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
        self._drive_sample: VelocityEnvelope | None = None
        self._feedback: deque[ExecutionFeedback] = deque(maxlen=24)
        self._awaiting_feedback: dict[str, VelocityEnvelope] = {}
        if require_server_heartbeat:
            self.create_subscription(
                DriveExecutionFeedback, "/safety/drive_feedback",
                self._on_drive_feedback, qos_profile_sensor_data
            )
        self._navigation_source: _NavigationSource | None = None
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
        if result.accepted and isinstance(envelope, VelocityEnvelope):
            self._drive_sample = envelope if envelope.payload.gesture is not None else None
            if self._drive_sample is not None:
                self._awaiting_feedback[envelope.request_id] = envelope
                if len(self._awaiting_feedback) > 32:
                    self._awaiting_feedback.pop(next(iter(self._awaiting_feedback)))
        if result.accepted and envelope.type != "drive.velocity":
            self._drive_sample = None
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

    def navigation_qualified(self) -> bool:
        return self._arbiter.navigation_qualified(self._monotonic_clock.monotonic())

    def bind_navigation_source(
        self, execution_id: str, topic: str, *, qualified: Callable[[], bool]
    ) -> bool:
        # An execution-specific topic must be configured at the Nav2 publisher.
        if (
            self._navigation_source is not None
            or not qualified()
            or not re.fullmatch(r"/control/navigation/run_[0-9a-f]{32}/cmd_vel", topic)
            or not self._arbiter.begin_navigation(execution_id, self._monotonic_clock.monotonic())
        ):
            return False
        try:
            def on_sample(message: Twist, info: dict[str, Any]) -> None:
                self._on_navigation_sample(topic, message, info)

            subscription = self.create_subscription(
                Twist, topic, on_sample, qos_profile_sensor_data
            )
        except Exception:
            self._arbiter.end_navigation(execution_id)
            raise
        bound_at = time.time_ns()
        self._navigation_source = _NavigationSource(
            execution_id,
            topic,
            bound_at / 1e9 - self._monotonic_clock.monotonic(),
            subscription,
            bound_at,
            qualified,
        )
        return True

    def clear_navigation_source(self, execution_id: str) -> None:
        source = self._navigation_source
        if source is None or source.execution_id != execution_id:
            return
        self._navigation_source = None
        self._arbiter.end_navigation(execution_id)
        self.destroy_subscription(source.subscription)
        self.publish_command()

    def _on_navigation_sample(self, topic: str, msg: Twist, info: dict[str, Any]) -> None:
        source = self._navigation_source
        if source is None or source.topic != topic:
            return
        now = self._monotonic_clock.monotonic()
        stamp = info.get("source_timestamp")
        if not source.qualified() or abs((time.time() - now) - source.offset) > 0.01:
            self.clear_navigation_source(source.execution_id)
            return
        if not isinstance(stamp, int) or stamp <= max(source.bound_at_ns, source.last_stamp):
            return
        endpoints = self.get_publishers_info_by_topic(source.topic)
        if len(endpoints) != 1 or endpoints[0].node_name != "controller_server":
            return
        # //Compatible: Humble's Python callback omits the writer GID. Topic isolation
        # still comes from the controller's run-specific remapping and a private ROS domain.
        gid = info.get("publisher_gid")
        if gid is not None and tuple(gid) != tuple(endpoints[0].endpoint_gid):
            return
        try:
            velocity = Velocity(msg.linear.x, msg.angular.z)
        except ValueError:
            return
        sampled = stamp / 1e9 - source.offset - 0.01
        if self._arbiter.accept_navigation_sample(
            source.execution_id, source.sequence + 1, velocity, sampled, now
        ):
            source.last_stamp = stamp
            source.sequence += 1
            self.publish_command()

    def publish_command(self) -> None:
        source = self._navigation_source
        if source is not None and (
            not source.qualified()
            or not self._arbiter.navigation_source_active(
                source.execution_id, self._monotonic_clock.monotonic()
            )
        ):
            self.clear_navigation_source(source.execution_id)
            return
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
        sample = self._drive_sample
        if sample is not None and (session, epoch, sequence) == (
            sample.session_id, sample.connection_epoch, sample.seq
        ):
            msg.request_id = sample.request_id
            msg.gesture = sample.payload.gesture or 0
        self._final_publisher.publish(msg)
        self._active = True
        self._publish(velocity)

    def _on_drive_feedback(self, msg: DriveExecutionFeedback, info: dict[str, Any]) -> None:
        endpoints = self.get_publishers_info_by_topic("/safety/drive_feedback")
        if len(endpoints) != 1:
            return
        endpoint = endpoints[0]
        sample = self._awaiting_feedback.get(msg.request_id)
        # //Compatible: Humble's Python subscription metadata omits publisher_gid.
        # Require sole expected topology, the command owner boot, and the exact admitted sample.
        gid = info.get("publisher_gid")
        if (
            endpoint.node_name != "safety_gate"
            or endpoint.node_namespace != "/"
            or (gid is not None and bytes(endpoint.endpoint_gid) != bytes(gid))
            or sample is None
            or (sample.session_id, sample.connection_epoch, sample.seq, sample.payload.gesture) != (
                msg.session_id, msg.connection_epoch, msg.sequence, msg.gesture
            )
            or msg.owner_boot_id != self._owner_boot
            or msg.robot_id != str(self.get_parameter("robot_id").value)
            or msg.connection_epoch != self._last_server_epoch
        ):
            return
        output = (
            {"vx": msg.vx, "wz": msg.wz, "limited": msg.limited} if msg.status == "applied" else {}
        )
        try:
            feedback = ExecutionFeedback(
                robot_id=msg.robot_id, session_id=msg.session_id,
                connection_epoch=msg.connection_epoch, request_id=msg.request_id,
                seq=msg.sequence, gesture=msg.gesture, status=msg.status,
                reason=msg.reason or None, **output,
            )
        except ValueError:
            return
        self._awaiting_feedback.pop(msg.request_id)
        self._feedback.append(feedback)

    def take_drive_feedback(self) -> list[dict[str, Any]]:
        return [self._feedback.popleft().model_dump(exclude_none=True)
                for _ in range(min(len(self._feedback), 8))]

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
