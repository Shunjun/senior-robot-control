"""Independent safety process.

The default 20 Hz timer is simulation configuration, not a physical braking or
timing guarantee. Braking, reaction, hard margin, and staleness stay unset
until parameters provide them, and an unset limit publishes zero velocity.
An unset or expired command TTL also publishes zero. The age is measured with
a monotonic clock and does not wait for a stop message.
This process does not arbitrate control, publish heartbeats, or remap Nav2.
It does not read wheel odometry or /ground_truth/odom, and it does not use a
wheel transform to relabel a scan into the body frame.
"""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan

from senior_control.config import DeviceProfile
from senior_control.models import Pose2D, Velocity
from senior_control.safety.guard import command_expired, evaluate_motion
from senior_control.safety.self_filter import FilteredObservation, ScanObservation, filter_self


def _positive(value: float) -> float | None:
    if not math.isfinite(value) or value <= 0.0:
        return None
    return value


def _nonnegative(value: float) -> float | None:
    if not math.isfinite(value) or value < 0.0:
        return None
    return value


class SafetyGate(Node):
    def __init__(self) -> None:
        super().__init__("safety_gate")
        self.declare_parameter("simulation_rate_hz", 20.0)
        self.declare_parameter("body_frame", "base_link")
        self.declare_parameter("braking_deceleration", -1.0)
        self.declare_parameter("yaw_braking_deceleration", -1.0)
        self.declare_parameter("reaction_time", -1.0)
        self.declare_parameter("hard_margin", -1.0)
        self.declare_parameter("observation_max_age", -1.0)
        self.declare_parameter("footprint_xy", rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("self_mask_xy", rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("self_mask_calibrated", False)
        self.declare_parameter("transform_static", False)
        self.declare_parameter("interpolate", False)
        self.declare_parameter("max_angular_gap", 0.0)
        self.declare_parameter("no_return_clear", False)
        self.declare_parameter("command_topic", "/safety/command_in")
        self.declare_parameter("measured_topic", "")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("output_topic", "/cmd_vel")
        self.declare_parameter("command_ttl_seconds", -1.0)
        self._body = str(self.get_parameter("body_frame").value)
        rate = float(self.get_parameter("simulation_rate_hz").value)
        if not self._body or not math.isfinite(rate) or rate <= 0.0:
            raise ValueError("simulation_rate_hz must be positive and body_frame nonempty")
        self._command = Velocity(0.0, 0.0)
        self._command_monotonic: float | None = None
        self._measured = Velocity(0.0, 0.0)
        self._filtered: FilteredObservation | None = None
        output = str(self.get_parameter("output_topic").value)
        command_topic = str(self.get_parameter("command_topic").value)
        scan_topic = str(self.get_parameter("scan_topic").value)
        self._publisher = self.create_publisher(Twist, output, 10)
        self.create_subscription(Twist, command_topic, self._on_command, 10)
        measured_topic = str(self.get_parameter("measured_topic").value)
        if measured_topic:
            self.create_subscription(Twist, measured_topic, self._on_measured, 10)
        self.create_subscription(LaserScan, scan_topic, self._on_scan, qos_profile_sensor_data)
        self.create_timer(1.0 / rate, self._on_timer)

    def _profile(self) -> DeviceProfile:
        footprint_xy = self.get_parameter("footprint_xy").value or []
        mask_xy = self.get_parameter("self_mask_xy").value or []
        footprint = (
            list(zip(footprint_xy[::2], footprint_xy[1::2], strict=False))
            if len(footprint_xy) >= 6 and len(footprint_xy) % 2 == 0
            else []
        )
        mask = (
            [list(zip(mask_xy[::2], mask_xy[1::2], strict=False))]
            if len(mask_xy) >= 6 and len(mask_xy) % 2 == 0
            else []
        )
        return DeviceProfile(
            footprint=footprint,
            self_mask=mask,
            self_mask_calibrated=bool(self.get_parameter("self_mask_calibrated").value),
            braking_deceleration=_positive(float(self.get_parameter("braking_deceleration").value)),
            yaw_braking_deceleration=_positive(
                float(self.get_parameter("yaw_braking_deceleration").value)
            ),
            reaction_time=_nonnegative(float(self.get_parameter("reaction_time").value)),
            hard_margin=_nonnegative(float(self.get_parameter("hard_margin").value)),
            observation_max_age=_nonnegative(
                float(self.get_parameter("observation_max_age").value)
            ),
        )

    def _velocity(self, msg: Twist) -> Velocity | None:
        vx = float(msg.linear.x)
        wz = float(msg.angular.z)
        if math.isfinite(vx) and math.isfinite(wz):
            return Velocity(vx, wz)
        return None

    def _on_command(self, msg: Twist) -> None:
        velocity = self._velocity(msg)
        if velocity is not None:
            self._command = velocity
            self._command_monotonic = time.monotonic()

    def _on_measured(self, msg: Twist) -> None:
        # Caller must supply independent body velocity. This node never subscribes
        # to wheel odometry or /ground_truth/odom on its own.
        velocity = self._velocity(msg)
        if velocity is not None:
            self._measured = velocity

    def _on_scan(self, msg: LaserScan) -> None:
        if msg.header.frame_id != self._body:
            self._filtered = None
            return
        try:
            stamp = Time.from_msg(msg.header.stamp).nanoseconds / 1e9
            static = bool(self.get_parameter("transform_static").value)
            gap = float(self.get_parameter("max_angular_gap").value)
            observation = ScanObservation(
                tuple(msg.ranges),
                float(msg.angle_min),
                float(msg.angle_increment),
                float(msg.range_min),
                float(msg.range_max),
                stamp,
                "ros",
                Pose2D(0.0, 0.0, 0.0),
                0.0 if static else stamp,
                self._body,
                stamp,
                bool(self.get_parameter("interpolate").value),
                gap,
                bool(self.get_parameter("no_return_clear").value),
                static,
            )
            self._filtered = filter_self(observation, self._profile())
        except (ValueError, OverflowError):
            self._filtered = None

    def _publish(self, velocity: Velocity) -> None:
        twist = Twist()
        twist.linear.x = velocity.vx
        twist.angular.z = velocity.wz
        self._publisher.publish(twist)

    def _on_timer(self) -> None:
        ttl = _positive(float(self.get_parameter("command_ttl_seconds").value))
        if command_expired(self._command_monotonic, time.monotonic(), ttl):
            self._publish(Velocity(0.0, 0.0))
            return
        now = self.get_clock().now().nanoseconds / 1e9
        observations = [] if self._filtered is None else [self._filtered]
        try:
            decision = evaluate_motion(
                self._command, self._measured, observations, self._profile(), now
            )
        except ValueError:
            self._publish(Velocity(0.0, 0.0))
            return
        self._publish(decision.velocity)


def main() -> None:
    rclpy.init()
    node = SafetyGate()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
