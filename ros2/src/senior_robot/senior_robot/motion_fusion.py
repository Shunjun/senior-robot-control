"""Align independent scan intervals with wheel feedback, IMU and arbiter output."""

import math
import time
from uuid import uuid4

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.clock import Clock as RosClock
from rclpy.clock import ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from senior_motion_msgs.msg import MotionEvidence, ScanMotion
from sensor_msgs.msg import Imu
from tf2_ros import Buffer, TransformException, TransformListener

from senior_control.models import Pose2D, Velocity
from senior_control.safety.motion_estimator import MotionEstimator
from senior_control.safety.motion_runtime import (
    ImuSample,
    ProducerTopology,
    ScanInterval,
    SourceEpochFence,
    SourceStampFrontier,
    StopAcknowledgement,
    TimedCommandBuffer,
    TimedImuBuffer,
    TimedPoseBuffer,
    TimedPoseSample,
    qualify_interval,
)
from senior_control.safety.slip import SlipMonitor
from senior_robot.motion_lifecycle import run


def seconds(stamp) -> float:
    return Time.from_msg(stamp).nanoseconds / 1e9


def planar_yaw(q) -> float:
    values = (q.x, q.y, q.z, q.w)
    if (
        not all(math.isfinite(v) for v in values)
        or abs(sum(v * v for v in values) - 1) > 0.001
        or abs(q.x) > 0.001
        or abs(q.y) > 0.001
    ):
        raise ValueError("invalid planar orientation")
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class MotionFusion(Node):
    def __init__(self, *, parameter_overrides=None) -> None:
        super().__init__("motion_fusion", parameter_overrides=parameter_overrides or [])
        self.declare_parameter("body_frame", "base_link")
        self.declare_parameter("wheel_frame", "odom")
        self.declare_parameter("imu_frame", "imu_link")
        self.declare_parameter("sensor_max_age", -1.0)
        self.declare_parameter("alignment_max_gap", -1.0)
        self.declare_parameter("pending_wait", -1.0)
        self.declare_parameter("max_variance", -1.0)
        self.declare_parameter("imu_zero_variance", -1.0)
        self.declare_parameter("min_quality", -1.0)
        self.declare_parameter("max_yaw_error", -1.0)
        self.declare_parameter("startup_window", -1.0)
        self.declare_parameter("wheel_motion_limit", -1.0)
        self.declare_parameter("stall_distance", -1.0)
        self.declare_parameter("lateral_limit", -1.0)
        self.declare_parameter("yaw_limit", -1.0)
        self._body = str(self.get_parameter("body_frame").value)
        self._wheel_frame = str(self.get_parameter("wheel_frame").value)
        self._imu_frame = str(self.get_parameter("imu_frame").value)
        self._max_age = float(self.get_parameter("sensor_max_age").value)
        gap = float(self.get_parameter("alignment_max_gap").value)
        self._wait = float(self.get_parameter("pending_wait").value)
        variance = float(self.get_parameter("max_variance").value)
        self._imu_zero_variance = float(self.get_parameter("imu_zero_variance").value)
        if (
            not self._body
            or not self._wheel_frame
            or not self._imu_frame
            or not all(
                math.isfinite(v) and v > 0 for v in (self._max_age, gap, self._wait, variance)
            )
        ):
            raise ValueError("unqualified motion alignment profile")
        self._wheel = TimedPoseBuffer(max_gap=gap, max_samples=256, max_variance=variance)
        self._imu = TimedImuBuffer(max_gap=gap, max_samples=512, max_variance=variance)
        self._commands = TimedCommandBuffer(max_gap=gap, max_samples=256)
        self._estimator = MotionEstimator(
            max_skew=0.001,
            min_quality=float(self.get_parameter("min_quality").value),
            max_yaw_error=float(self.get_parameter("max_yaw_error").value),
            max_covariance=variance,
        )
        self._slip = SlipMonitor(
            startup_window=float(self.get_parameter("startup_window").value),
            wheel_motion_limit=float(self.get_parameter("wheel_motion_limit").value),
            # //Compatible: older explicit profiles retain their conservative wheel limit.
            stall_distance=(
                None
                if float(self.get_parameter("stall_distance").value) == -1.0
                else float(self.get_parameter("stall_distance").value)
            ),
            lateral_limit=float(self.get_parameter("lateral_limit").value),
            yaw_limit=float(self.get_parameter("yaw_limit").value),
        )
        self._stop_acknowledgement = StopAcknowledgement()
        self._scan_topology = ProducerTopology("scan_motion_node", "/")
        self._scan_epochs = SourceEpochFence(max_retired=256)
        self._scan_frontier = SourceStampFrontier()
        self._boot_epoch = uuid4().hex
        self._last_ros_ns: int | None = None
        self._wheel_received: float | None = None
        self._imu_received: float | None = None
        self._pending = None
        self._pending_at = 0.0
        self._tf = Buffer()
        self._listener = TransformListener(self._tf, self)
        self._publisher = self.create_publisher(MotionEvidence, "/motion/evidence", 10)
        self.create_subscription(Odometry, "/wheel/odom", self._on_wheel, qos_profile_sensor_data)
        self.create_subscription(Imu, "/imu/data", self._on_imu, qos_profile_sensor_data)
        self.create_subscription(Twist, "/cmd_vel", self._on_command, 10)
        self.create_subscription(Twist, "/safety/command_in", self._on_requested, 10)
        self.create_subscription(ScanMotion, "/motion/scan", self._on_scan, 10)
        self.create_timer(0.02, self._try_pending, clock=RosClock(clock_type=ClockType.STEADY_TIME))

    def _recent_stamp(self, stamp: float) -> bool:
        now = self.get_clock().now().nanoseconds / 1e9
        return math.isfinite(stamp) and stamp > 0 and 0 <= now - stamp <= self._max_age

    def _clock_ok(self) -> bool:
        now_ns = self.get_clock().now().nanoseconds
        if self._last_ros_ns is not None and now_ns < self._last_ros_ns:
            pending = self._pending
            self._scan_epochs.retire_active()
            self._scan_frontier.reset()
            self._pending = None
            self._wheel.clear()
            self._imu = TimedImuBuffer(
                max_gap=self._imu.max_gap, max_samples=512, max_variance=self._imu.max_variance
            )
            self._commands = TimedCommandBuffer(max_gap=self._commands.max_gap, max_samples=256)
            self._wheel_received = self._imu_received = None
            self._last_ros_ns = now_ns
            if pending is not None:
                self._invalid(pending, "clock_rollback")
            return False
        self._last_ros_ns = now_ns
        return True

    def _on_wheel(self, msg: Odometry) -> None:
        self._wheel_received = None
        try:
            stamp = seconds(msg.header.stamp)
            if (
                msg.header.frame_id != self._wheel_frame
                or msg.child_frame_id != self._body
                or not self._recent_stamp(stamp)
            ):
                raise ValueError("wheel time or frame")
            position = msg.pose.pose.position
            yaw = planar_yaw(msg.pose.pose.orientation)
            covariance = msg.pose.covariance
            sample = TimedPoseSample(
                stamp,
                Pose2D(position.x, position.y, yaw),
                (covariance[0], covariance[7], covariance[35]),
                self._wheel_frame,
            )
            if not self._wheel.add(sample):
                raise ValueError("wheel covariance or order")
            self._wheel_received = time.monotonic()
        except (ValueError, OverflowError):
            self._wheel = TimedPoseBuffer(
                max_gap=self._wheel.max_gap, max_samples=256, max_variance=self._wheel.max_variance
            )
        self._try_pending()

    def _on_imu(self, msg: Imu) -> None:
        self._imu_received = None
        try:
            stamp = seconds(msg.header.stamp)
            if msg.header.frame_id != self._imu_frame or not self._recent_stamp(stamp):
                raise ValueError("IMU time or frame")
            transform = self._tf.lookup_transform(
                self._body, self._imu_frame, Time.from_msg(msg.header.stamp)
            )
            planar_yaw(transform.transform.rotation)
            if msg.angular_velocity_covariance[0] < 0:
                raise ValueError("IMU angular velocity unavailable")
            variance = msg.angular_velocity_covariance[8]
            if variance == 0.0:
                # Simulation-only conservative assumption; real IMU noise needs calibration.
                variance = self._imu_zero_variance
            if not self._imu.add(ImuSample(stamp, msg.angular_velocity.z, variance)):
                raise ValueError("IMU covariance or order")
            self._imu_received = time.monotonic()
        except (ValueError, OverflowError, TransformException):
            self._imu = TimedImuBuffer(
                max_gap=self._imu.max_gap, max_samples=512, max_variance=self._imu.max_variance
            )
        self._try_pending()

    def _on_command(self, msg: Twist) -> None:
        try:
            velocity = Velocity(msg.linear.x, msg.angular.z)
            self._commands.add(self.get_clock().now().nanoseconds / 1e9, velocity)
            self._stop_acknowledgement.applied(velocity)
            self._release_if_acknowledged()
        except ValueError:
            self._commands = TimedCommandBuffer(max_gap=self._commands.max_gap, max_samples=256)
        self._try_pending()

    def _on_requested(self, msg: Twist) -> None:
        try:
            self._stop_acknowledgement.requested(Velocity(msg.linear.x, msg.angular.z))
            self._release_if_acknowledged()
        except ValueError:
            pass

    def _release_if_acknowledged(self) -> None:
        if self._stop_acknowledgement.ready:
            self._slip.reset()
            self._stop_acknowledgement.clear()

    def _invalid(self, message: ScanMotion, reason: str) -> None:
        output = MotionEvidence()
        output.header = message.header
        output.interval_start = message.interval_start
        output.source_epoch = self._output_epoch(message.source_epoch)
        output.reason = reason
        self._publisher.publish(output)

    def _output_epoch(self, scan_epoch: str) -> str:
        return f"{self._boot_epoch}:{scan_epoch}" if scan_epoch else ""

    def _on_scan(self, message: ScanMotion) -> None:
        if not self._clock_ok():
            self._invalid(message, "clock_rollback")
            return
        if not self._scan_topology_ready():
            self._pending = None
            self._invalid(message, "scan_producer_untrusted_or_retired")
            return
        previous_epoch = self._scan_epochs.active
        if not self._scan_epochs.accept(message.source_epoch):
            if message.source_epoch and self._scan_epochs.active is not None:
                # A retired source cannot cancel a pending interval from its successor.
                return
            self._pending = None
            self._invalid(message, "scan_producer_untrusted_or_retired")
            return
        if previous_epoch != message.source_epoch:
            self._scan_frontier.reset()
        stamp_ns = Time.from_msg(message.header.stamp).nanoseconds
        start_ns = Time.from_msg(message.interval_start).nanoseconds
        eligible = (
            message.valid
            and bool(message.source_epoch)
            and message.header.frame_id == self._body
            and 0 < start_ns < stamp_ns
            and self._recent_stamp(stamp_ns / 1e9)
        )
        if not self._scan_frontier.accept(
            stamp_ns, eligible=eligible, receipt_ros_ns=self.get_clock().now().nanoseconds
        ):
            return
        if eligible and not self._scan_frontier.interval_after_failure(start_ns):
            return
        if self._pending is not None:
            self._invalid(self._pending, "alignment_superseded")
            self._pending = None
        if not eligible:
            self._invalid(message, message.reason or "scan_unavailable")
            return
        self._pending = message
        self._pending_at = time.monotonic()
        self._try_pending()

    def _try_pending(self) -> None:
        if not self._clock_ok() or not self._scan_topology_ready():
            self._pending = None
            return
        message = self._pending
        if message is None:
            return
        steady = time.monotonic()
        start, end = seconds(message.interval_start), seconds(message.header.stamp)
        if not self._recent_stamp(end):
            self._invalid(message, "imu_wheel_command_unavailable_or_conflicting")
            self._pending = None
            return
        fresh = all(
            received is not None and 0 <= steady - received <= self._max_age
            for received in (self._wheel_received, self._imu_received)
        )
        command = self._commands.integrated(start, end)
        result = None
        if fresh and command is not None:
            try:
                delta = message.body_delta
                scan = ScanInterval(
                    start,
                    end,
                    Pose2D(delta.x, delta.y, delta.theta),
                    (message.covariance[0], message.covariance[4], message.covariance[8]),
                    message.response,
                )
                result = qualify_interval(
                    scan, self._wheel, self._imu, command.velocity, self._estimator, self._slip
                )
            except (ValueError, OverflowError):
                result = None
        if result is None:
            if steady - self._pending_at >= self._wait or not self._recent_stamp(end):
                self._invalid(message, "imu_wheel_command_unavailable_or_conflicting")
                self._pending = None
            return
        output = MotionEvidence()
        output.header = message.header
        output.interval_start = message.interval_start
        output.source_epoch = self._output_epoch(message.source_epoch)
        output.valid = True
        output.reason = "motion_anomaly" if result.anomalous else "qualified"
        output.body_delta = message.body_delta
        output.local_pose = message.local_pose
        output.body_velocity.linear.x = result.estimate.velocity.vx
        output.body_velocity.angular.z = result.estimate.velocity.wz
        output.covariance = message.covariance
        output.quality = result.estimate.quality
        output.imu_valid = output.wheel_valid = True
        output.motion_anomaly = result.anomalous
        if result.anomalous:
            self._stop_acknowledgement.trip()
        # A continuously tracked scan origin alone does not prove the power-on pose.
        output.startup_reference_valid = False
        self._publisher.publish(output)
        self._pending = None

    def _scan_topology_ready(self) -> bool:
        endpoints = self.get_publishers_info_by_topic("/motion/scan")
        ready, changed = self._scan_topology.observe(
            [
                (endpoint.node_name, endpoint.node_namespace, bytes(endpoint.endpoint_gid))
                for endpoint in endpoints
            ]
        )
        if changed or not ready:
            self._scan_epochs.retire_active()
            self._scan_frontier.reset()
            self._pending = None
        return ready


def main() -> None:
    run(MotionFusion)
