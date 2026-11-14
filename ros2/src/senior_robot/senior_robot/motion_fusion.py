"""Align independent scan intervals with wheel feedback, IMU and arbiter output."""

import json
import math
import time
from collections import deque
from dataclasses import asdict
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
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from senior_control.models import Pose2D, Velocity
from senior_control.safety.motion_estimator import MotionEstimator
from senior_control.safety.motion_runtime import (
    DeferredSample,
    DeferredSourceQueue,
    ImuSample,
    ProducerTopology,
    ProgressInterval,
    ProgressQueue,
    ProgressRequalification,
    ScanInterval,
    SourceEpochFence,
    SourceStampFrontier,
    StopAcknowledgement,
    TimedCommandBuffer,
    TimedImuBuffer,
    TimedPoseBuffer,
    TimedPoseSample,
    align_interval,
    progress_interval_fresh,
    qualify_interval,
    stationary_interval,
)
from senior_control.safety.slip import SlipMonitor
from senior_robot.motion_lifecycle import MotionExecutor, run


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
        self.declare_parameter("progress_stationary_max_linear_speed", -1.0)
        self.declare_parameter("progress_stationary_max_yaw_rate", -1.0)
        self._body = str(self.get_parameter("body_frame").value)
        self._wheel_frame = str(self.get_parameter("wheel_frame").value)
        self._imu_frame = str(self.get_parameter("imu_frame").value)
        self._max_age = float(self.get_parameter("sensor_max_age").value)
        gap = float(self.get_parameter("alignment_max_gap").value)
        self._wait = float(self.get_parameter("pending_wait").value)
        variance = float(self.get_parameter("max_variance").value)
        self._imu_zero_variance = float(self.get_parameter("imu_zero_variance").value)
        self._stationary_linear = float(
            self.get_parameter("progress_stationary_max_linear_speed").value
        )
        self._stationary_yaw = float(self.get_parameter("progress_stationary_max_yaw_rate").value)
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
        self._last_applied_command: tuple[int, Velocity] | None = None
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
        # Bare zero Twist samples may be watchdog/protection outputs. They are
        # not a fenced authorization to clear a real slip latch (deferred 4a).
        self._stop_acknowledgement = StopAcknowledgement()
        self._progress = ProgressQueue(max_pending=64)
        self._progress_requalification = ProgressRequalification(max_gap=gap, required_intervals=2)
        self._progress_started = False
        self._deferred_wheel = DeferredSourceQueue(max_pending=16)
        self._deferred_imu = DeferredSourceQueue(max_pending=16)
        self._deferred_scan = DeferredSourceQueue(max_pending=16)
        self._deferred_cursor = 0
        self._scan_topology = ProducerTopology("scan_motion_node", "/")
        self._scan_epochs = SourceEpochFence(max_retired=256)
        self._scan_frontier = SourceStampFrontier()
        self._boot_epoch = uuid4().hex
        self._last_ros_ns: int | None = None
        self._wheel_received: float | None = None
        self._imu_received: float | None = None
        self._pending = None
        self._pending_at = 0.0
        self._pending_received_ros_ns = 0
        self._scan_events = deque(maxlen=256)
        self._last_accepted_scan = None
        self._last_valid_source = None
        self._first_progress_break = None
        self._progress_break_logged = False
        self._tf = Buffer()
        self._listener = TransformListener(self._tf, self)
        self._publisher = self.create_publisher(MotionEvidence, "/motion/evidence", 10)
        self.create_subscription(Odometry, "/wheel/odom", self._on_wheel, qos_profile_sensor_data)
        self.create_subscription(Imu, "/imu/data", self._on_imu, qos_profile_sensor_data)
        self.create_subscription(Twist, "/cmd_vel", self._on_command, 10)
        self.create_subscription(Twist, "/safety/command_in", self._on_requested, 10)
        self.create_subscription(ScanMotion, "/motion/scan", self._on_scan, 10)
        self.create_service(Trigger, "~/requalify_progress", self._on_progress_requalification)
        self.create_timer(0.02, self._try_pending, clock=RosClock(clock_type=ClockType.STEADY_TIME))
        # //Compatible: legacy profiles may omit both shared-core selections.
        self.declare_parameter("motion_core_profile", "")
        self._native_motion_bridge = None
        self.declare_parameter("hardware_motion_profile", "")
        hardware_path = str(self.get_parameter("hardware_motion_profile").value)
        candidate = str(self.get_parameter("motion_core_profile").value)
        if candidate or hardware_path:
            import os

            from senior_robot.motion_observation_adapter import NativeObservationRosBridge
            from senior_robot.motion_profile import select_motion_profile

            profile = select_motion_profile(
                candidate,
                hardware_path=hardware_path,
                simulated_clock=bool(self.get_parameter("use_sim_time").value),
                localhost_only=os.environ.get("ROS_LOCALHOST_ONLY", ""),
                domain_id=os.environ.get("ROS_DOMAIN_ID", ""),
            )
            self._native_motion_bridge = NativeObservationRosBridge(self, profile)

    def _recent_stamp(self, stamp: float) -> bool:
        now = self.get_clock().now().nanoseconds / 1e9
        return math.isfinite(stamp) and stamp > 0 and 0 <= now - stamp <= self._max_age

    def _clock_ok(self) -> bool:
        now_ns = self.get_clock().now().nanoseconds
        if self._last_ros_ns is not None and now_ns < self._last_ros_ns:
            was_active = self._progress_started
            deferred = self._deferred_scan.peek()
            reserved_epoch = deferred.value.source_epoch if deferred is not None else None
            self._break_progress(
                "clock_rollback",
                details={"previous_ros_ns": self._last_ros_ns, "new_ros_ns": now_ns},
            )
            self._progress_requalification.disarm()
            self._clear_deferred()
            if reserved_epoch is not None:
                self._scan_epochs.retire_unadmitted(reserved_epoch)
            pending = self._pending
            self._scan_epochs.retire_active()
            self._scan_frontier.reset()
            self._pending = None
            self._wheel.clear()
            self._imu = TimedImuBuffer(
                max_gap=self._imu.max_gap, max_samples=512, max_variance=self._imu.max_variance
            )
            self._commands = TimedCommandBuffer(max_gap=self._commands.max_gap, max_samples=256)
            self._last_applied_command = None
            self._wheel_received = self._imu_received = None
            self._last_ros_ns = now_ns
            invalidated = pending if pending is not None else self._last_valid_source
            if invalidated is not None and (pending is not None or was_active):
                self._invalid(invalidated, "clock_rollback")
            return False
        self._last_ros_ns = now_ns
        return True

    def _on_wheel(self, msg: Odometry) -> None:
        received_steady = time.monotonic()
        received_ros_ns = self.get_clock().now().nanoseconds
        if not self._clock_ok():
            return
        prior_bounds = self._wheel.diagnostic_bounds()
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        try:
            if (
                stamp_ns <= 0
                or msg.header.stamp.nanosec >= 1_000_000_000
                or msg.header.frame_id != self._wheel_frame
                or msg.child_frame_id != self._body
            ):
                raise ValueError("wheel time or frame")
            position = msg.pose.pose.position
            yaw = planar_yaw(msg.pose.pose.orientation)
            covariance = msg.pose.covariance
            sample = TimedPoseSample(
                stamp_ns / 1e9,
                Pose2D(position.x, position.y, yaw),
                (covariance[0], covariance[7], covariance[35]),
                self._wheel_frame,
            )
            if any(
                not math.isfinite(value) or not 0 < value <= self._wheel.max_variance
                for value in sample.covariance
            ):
                raise ValueError("wheel covariance")
            if self._wheel.latest is not None and sample.stamp <= self._wheel.latest.stamp:
                raise ValueError("wheel order")
        except (ValueError, OverflowError) as exc:
            self._deferred_failure(
                "wheel",
                "wheel_input_invalid",
                details={
                    "stamp_ns": stamp_ns,
                    "error": str(exc)[:120],
                    "prior_bounds": prior_bounds,
                },
            )
            self._try_pending()
            return
        item = DeferredSample(stamp_ns, received_ros_ns, received_steady, sample)
        if self._deferred_wheel.peek() is not None or stamp_ns > received_ros_ns:
            if not self._deferred_wheel.offer(item):
                self._deferred_failure(
                    "wheel", "wheel_deferred_rejected", details={"stamp_ns": stamp_ns}
                )
        else:
            self._admit_wheel(item)
        self._try_pending()

    def _admit_wheel(self, item) -> bool:
        sample = item.value
        prior_bounds = self._wheel.diagnostic_bounds()
        if not self._recent_stamp(sample.stamp) or not self._wheel.add(sample):
            self._deferred_failure(
                "wheel",
                "wheel_input_invalid",
                details={"stamp_ns": item.stamp_ns, "prior_bounds": prior_bounds},
            )
            return False
        self._wheel_received = item.received_steady
        return True

    def _on_imu(self, msg: Imu) -> None:
        received_steady = time.monotonic()
        received_ros_ns = self.get_clock().now().nanoseconds
        if not self._clock_ok():
            return
        prior_bounds = self._imu.diagnostic_bounds()
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        try:
            if (
                stamp_ns <= 0
                or msg.header.stamp.nanosec >= 1_000_000_000
                or msg.header.frame_id != self._imu_frame
            ):
                raise ValueError("IMU time or frame")
            if msg.angular_velocity_covariance[0] < 0:
                raise ValueError("IMU angular velocity unavailable")
            variance = msg.angular_velocity_covariance[8]
            if variance == 0.0:
                # Simulation-only conservative assumption; real IMU noise needs calibration.
                variance = self._imu_zero_variance
            sample = ImuSample(stamp_ns / 1e9, msg.angular_velocity.z, variance)
            if (
                not math.isfinite(sample.yaw_rate)
                or not math.isfinite(sample.variance)
                or not 0 < sample.variance <= self._imu.max_variance
            ):
                raise ValueError("IMU covariance or value")
            last_stamp = self._imu.diagnostic_bounds()[1]
            if last_stamp is not None and sample.stamp <= last_stamp:
                raise ValueError("IMU order")
        except (ValueError, OverflowError) as exc:
            self._deferred_failure(
                "imu",
                "imu_input_invalid",
                details={
                    "stamp_ns": stamp_ns,
                    "error": str(exc)[:120],
                    "prior_bounds": prior_bounds,
                },
            )
            self._try_pending()
            return
        item = DeferredSample(stamp_ns, received_ros_ns, received_steady, sample)
        if self._deferred_imu.peek() is not None or stamp_ns > received_ros_ns:
            if not self._deferred_imu.offer(item):
                self._deferred_failure(
                    "imu", "imu_deferred_rejected", details={"stamp_ns": stamp_ns}
                )
        else:
            self._admit_imu(item)
        self._try_pending()

    def _admit_imu(self, item) -> bool:
        sample = item.value
        prior_bounds = self._imu.diagnostic_bounds()
        try:
            if not self._recent_stamp(sample.stamp):
                raise ValueError("IMU time")
            transform = self._tf.lookup_transform(
                self._body,
                self._imu_frame,
                Time(nanoseconds=item.stamp_ns, clock_type=ClockType.ROS_TIME),
            )
            planar_yaw(transform.transform.rotation)
            if not self._imu.add(sample):
                raise ValueError("IMU covariance or order")
        except (ValueError, OverflowError, TransformException) as exc:
            self._deferred_failure(
                "imu",
                "imu_input_invalid",
                details={
                    "stamp_ns": item.stamp_ns,
                    "error": str(exc)[:120],
                    "prior_bounds": prior_bounds,
                },
            )
            return False
        self._imu_received = item.received_steady
        return True

    def _on_command(self, msg: Twist) -> None:
        try:
            velocity = Velocity(msg.linear.x, msg.angular.z)
            ros_ns = self.get_clock().now().nanoseconds
            if self._last_applied_command == (ros_ns, velocity):
                return
            steady = time.monotonic()
            self._progress_requalification.applied(velocity, steady=steady, ros_ns=ros_ns)
            prior_bounds = self._commands.diagnostic_bounds()
            # Only simulation's discrete /clock may legitimately receive ordered
            # output changes at one instant. Keep wall-clock conflicts fail closed.
            # The zero-duration boundary governs only subsequent ROS time.
            same_instant = (
                bool(self.get_parameter("use_sim_time").value)
                and self._last_applied_command is not None
                and self._last_applied_command[0] == ros_ns
            )
            accepted = (
                self._commands.replace_latest(ros_ns / 1e9, velocity)
                if same_instant
                else self._commands.add(ros_ns / 1e9, velocity)
            )
            if not accepted:
                self._last_applied_command = None
                self._break_progress(
                    "applied_command_timestamp_rejected",
                    details={
                        "ros_ns": ros_ns,
                        "prior_bounds": prior_bounds,
                        "velocity": asdict(velocity),
                    },
                )
            else:
                self._last_applied_command = (ros_ns, velocity)
            self._stop_acknowledgement.applied(velocity)
        except ValueError as exc:
            self._break_progress("applied_command_invalid", details={"error": str(exc)[:120]})
            self._progress_requalification.interrupt()
            self._commands = TimedCommandBuffer(max_gap=self._commands.max_gap, max_samples=256)
            self._last_applied_command = None
        self._try_pending()

    def _on_requested(self, msg: Twist) -> None:
        try:
            velocity = Velocity(msg.linear.x, msg.angular.z)
            self._progress_requalification.requested(
                velocity, steady=time.monotonic(), ros_ns=self.get_clock().now().nanoseconds
            )
            self._stop_acknowledgement.requested(velocity)
        except ValueError:
            self._progress_requalification.interrupt()

    def _break_progress(self, reason: str, *, details=None, prior_queue=None) -> None:
        if self._progress_started or self._progress.unqualified:
            if self._first_progress_break is None:
                queue = dict(prior_queue or self._progress.diagnostic_state())
                if isinstance(queue["head"], ProgressInterval):
                    queue["head"] = asdict(queue["head"])
                pending = self._pending
                self._first_progress_break = {
                    "reason": reason,
                    "details": details or {},
                    "steady": time.monotonic(),
                    "ros_ns": self.get_clock().now().nanoseconds,
                    "queue": queue,
                    "pending": (
                        {
                            "source_epoch": pending.source_epoch[:95],
                            "start_ns": Time.from_msg(pending.interval_start).nanoseconds,
                            "end_ns": Time.from_msg(pending.header.stamp).nanoseconds,
                            "received_steady": self._pending_at,
                        }
                        if pending is not None
                        else None
                    ),
                    "wheel_bounds": self._wheel.diagnostic_bounds(),
                    "imu_bounds": self._imu.diagnostic_bounds(),
                    "command_bounds": self._commands.diagnostic_bounds(),
                    "wheel_received_steady": self._wheel_received,
                    "imu_received_steady": self._imu_received,
                    "last_accepted": self._last_accepted_scan,
                    "recent_scan_events": list(self._scan_events)[-32:],
                    "max_age": self._max_age,
                    "pending_wait": self._wait,
                }
            self._progress.invalidate()
            self._progress_started = False
            self._progress_requalification.disarm()
            self._clear_deferred()

    def _clear_deferred(self) -> None:
        pending_scan = self._deferred_scan.peek()
        if pending_scan is not None and not pending_scan.value.valid:
            # A failure that never reached ROS time cannot be forgotten and
            # later bridged by that same source epoch.
            self._scan_epochs.retire_unadmitted(pending_scan.value.source_epoch)
        self._deferred_wheel.clear()
        self._deferred_imu.clear()
        self._deferred_scan.clear()

    def _deferred_failure(self, stream: str, reason: str, *, details=None) -> None:
        was_active = self._progress_started
        self._break_progress(reason, details=details)
        self._clear_deferred()
        self._progress_requalification.interrupt()
        if stream == "wheel":
            self._wheel_received = None
            self._wheel = TimedPoseBuffer(
                max_gap=self._wheel.max_gap, max_samples=256, max_variance=self._wheel.max_variance
            )
        elif stream == "imu":
            self._imu_received = None
            self._imu = TimedImuBuffer(
                max_gap=self._imu.max_gap, max_samples=512, max_variance=self._imu.max_variance
            )
        if was_active and self._last_valid_source is not None:
            self._invalid(self._last_valid_source, "progress_history_unqualified")

    def _on_progress_requalification(self, _request, response):
        if not self._progress.unqualified:
            response.success = False
            response.message = "progress history is not broken"
            return response
        if self._slip.latched:
            response.success = False
            response.message = "motion anomaly remains latched"
            return response
        if not all(
            math.isfinite(value) and value > 0
            for value in (self._stationary_linear, self._stationary_yaw)
        ):
            response.success = False
            response.message = "stationary profile unqualified"
            return response
        try:
            self._progress_requalification.arm(
                steady=time.monotonic(), ros_ns=self.get_clock().now().nanoseconds
            )
        except ValueError:
            response.success = False
            response.message = "clock unqualified"
            return response
        response.success = True
        response.message = (
            "armed; fresh requested zero, applied zero and stationary evidence required"
        )
        return response

    @staticmethod
    def _wire_fits(value: str, limit: int) -> bool:
        if len(value) > limit:
            return False
        try:
            return len(value.encode("utf-8")) <= limit
        except UnicodeEncodeError:
            return False

    def _new_evidence(self, message: ScanMotion) -> MotionEvidence:
        output = MotionEvidence()
        output.header.stamp = message.header.stamp
        if self._wire_fits(message.header.frame_id, 128):
            output.header.frame_id = message.header.frame_id
        if self._wire_fits(message.source_epoch, 95):
            output.source_epoch = self._output_epoch(message.source_epoch)
        output.interval_start = message.interval_start
        return output

    def _invalid(self, message: ScanMotion, reason: str) -> None:
        output = self._new_evidence(message)
        output.reason = reason if self._wire_fits(reason, 64) else "evidence_field_overflow"
        self._record_scan_event(message, "invalid", output.reason)
        self._publisher.publish(output)
        if self._first_progress_break is not None and not self._progress_break_logged:
            self._progress_break_logged = True
            self.get_logger().warning(
                "motion_progress_break "
                + json.dumps(self._first_progress_break, sort_keys=True, separators=(",", ":"))
            )

    def _record_scan_event(self, message: ScanMotion, status: str, reason: str = "") -> None:
        self._scan_events.append(
            {
                "source_epoch": (
                    message.source_epoch
                    if self._wire_fits(message.source_epoch, 95)
                    else "<overflow>"
                ),
                "start_ns": message.interval_start.sec * 1_000_000_000
                + message.interval_start.nanosec,
                "end_ns": message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec,
                "status": status,
                "reason": reason,
            }
        )

    def _output_epoch(self, scan_epoch: str) -> str:
        return f"{self._boot_epoch}:{scan_epoch}" if scan_epoch else ""

    def _scan_shape_qualified(self, message: ScanMotion) -> bool:
        stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        start_ns = message.interval_start.sec * 1_000_000_000 + message.interval_start.nanosec
        return (
            message.valid
            and message.header.frame_id == self._body
            and self._wire_fits(message.header.frame_id, 128)
            and self._wire_fits(message.source_epoch, 95)
            and bool(message.source_epoch)
            and self._wire_fits(message.reason, 64)
            and 0 < start_ns < stamp_ns
            and message.header.stamp.nanosec < 1_000_000_000
            and message.interval_start.nanosec < 1_000_000_000
            and math.isfinite(message.response)
            and self._estimator.min_quality <= message.response <= 1.0
            and math.isfinite(message.residual)
            and message.residual >= 0.0
            and message.valid_rays > 0
            and all(math.isfinite(value) for value in message.covariance)
            and all(
                0 < message.covariance[index] <= self._wheel.max_variance for index in (0, 4, 8)
            )
            and all(
                math.isfinite(value)
                for value in (
                    message.body_delta.x,
                    message.body_delta.y,
                    message.body_delta.theta,
                    message.local_pose.x,
                    message.local_pose.y,
                    message.local_pose.theta,
                )
            )
        )

    def _on_scan(self, message: ScanMotion) -> None:
        received_steady = time.monotonic()
        received_ros_ns = self.get_clock().now().nanoseconds
        stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        start_ns = message.interval_start.sec * 1_000_000_000 + message.interval_start.nanosec
        shape_valid = self._scan_shape_qualified(message)
        future_invalid_epoch = (
            not shape_valid
            and stamp_ns > received_ros_ns
            and message.source_epoch != self._scan_epochs.active
            and bool(message.source_epoch)
            and self._wire_fits(message.source_epoch, 95)
        )
        if future_invalid_epoch:
            if (
                message.header.stamp.nanosec < 1_000_000_000
                and message.interval_start.nanosec < 1_000_000_000
            ):
                self._hold_future_invalid_epoch(message, stamp_ns, received_steady, received_ros_ns)
            else:
                self._reject_unclockable_epoch(message)
            return
        if not shape_valid or (stamp_ns <= received_ros_ns and self._deferred_scan.peek() is None):
            self._admit_scan(message, received_steady, received_ros_ns)
            self._try_pending()
            return
        if not self._clock_ok():
            self._invalid(message, "clock_rollback")
            return
        if not self._scan_topology_ready():
            self._invalid(message, "scan_producer_untrusted_or_retired")
            return
        if self._scan_epochs.is_retired(message.source_epoch):
            self._record_scan_event(message, "discarded", "retired_epoch")
            return
        active = self._scan_epochs.active
        if active is not None and active != message.source_epoch:
            was_active = self._progress_started
            self._break_progress(
                "future_source_epoch_changed",
                details={"previous_epoch": active, "new_epoch": message.source_epoch},
            )
            self._scan_epochs.retire_active()
            self._scan_frontier.reset()
            if was_active and self._last_valid_source is not None:
                self._invalid(self._last_valid_source, "progress_history_unqualified")
        tail = self._deferred_scan.latest()
        if tail is not None and tail.value.source_epoch != message.source_epoch:
            old_epoch = tail.value.source_epoch
            self._scan_epochs.retire_unadmitted(old_epoch)
            self._deferred_failure(
                "scan", "deferred_source_epoch_changed", details={"previous_epoch": old_epoch}
            )
            tail = None
        expected_start = (
            tail.stamp_ns
            if tail is not None
            else self._progress.diagnostic_state()["last_end_ns"]
            if self._progress_started
            else None
        )
        if expected_start is not None and start_ns != expected_start:
            self._deferred_failure(
                "scan",
                "deferred_source_gap_or_overlap",
                details={"expected_start_ns": expected_start, "start_ns": start_ns},
            )
            return
        copied = self._copy_scan(message)
        item = DeferredSample(stamp_ns, received_ros_ns, received_steady, copied)
        if not self._deferred_scan.offer(item):
            self._deferred_failure("scan", "scan_deferred_rejected", details={"stamp_ns": stamp_ns})
        self._try_pending()

    def _reject_unclockable_epoch(self, message: ScanMotion) -> None:
        """Retire a new source whose future timestamp cannot identify a time."""
        if not self._clock_ok() or not self._scan_topology_ready():
            return
        if self._scan_epochs.is_retired(message.source_epoch):
            self._record_scan_event(message, "discarded", "retired_epoch")
            return
        pending = self._deferred_scan.peek()
        reserved_epoch = pending.value.source_epoch if pending is not None else None
        was_active = self._progress_started
        self._break_progress("source_time_noncanonical")
        self._clear_deferred()
        if reserved_epoch is not None:
            self._scan_epochs.retire_unadmitted(reserved_epoch)
        self._scan_epochs.retire_active()
        self._scan_epochs.retire_unadmitted(message.source_epoch)
        self._scan_frontier.reset()
        self._progress_requalification.disarm()
        if was_active and self._last_valid_source is not None:
            self._invalid(self._last_valid_source, "progress_history_unqualified")

    def _discard_deferred_scan_after_failure(self, accepted_epoch: str) -> None:
        """Do not forget a replaced source or a newer deferred failure frontier."""
        pending = self._deferred_scan.peek()
        if pending is not None:
            if pending.value.source_epoch != accepted_epoch:
                self._scan_epochs.retire_unadmitted(pending.value.source_epoch)
            elif not pending.value.valid:
                self._scan_frontier.accept(
                    pending.stamp_ns, eligible=False, receipt_ros_ns=pending.received_ros_ns
                )
        self._deferred_scan.clear()

    def _hold_future_invalid_epoch(
        self, message: ScanMotion, stamp_ns: int, received_steady: float, received_ros_ns: int
    ) -> None:
        """Revoke now, but do not admit a future epoch or advance its frontier."""
        if not self._clock_ok() or not self._scan_topology_ready():
            return
        if self._scan_epochs.is_retired(message.source_epoch):
            self._record_scan_event(message, "discarded", "retired_epoch")
            return
        tail = self._deferred_scan.latest()
        if (
            tail is not None
            and tail.value.source_epoch == message.source_epoch
            and stamp_ns < tail.stamp_ns
        ):
            self._record_scan_event(message, "discarded", "deferred_stamp_frontier")
            return
        active = self._scan_epochs.active
        was_active = self._progress_started
        if active is not None and active != message.source_epoch:
            self._break_progress(
                "future_invalid_source_epoch_changed",
                details={"previous_epoch": active, "new_epoch": message.source_epoch},
            )
            self._scan_epochs.retire_active()
            self._scan_frontier.reset()
            if was_active and self._last_valid_source is not None:
                self._invalid(self._last_valid_source, "progress_history_unqualified")
        elif tail is not None:
            if tail.value.source_epoch != message.source_epoch:
                self._scan_epochs.retire_unadmitted(tail.value.source_epoch)
            # A failure cannot leave earlier deferred intervals to be released later.
            self._deferred_scan.clear()
        self._progress_requalification.disarm()
        copied = self._copy_scan_failure(message)
        item = DeferredSample(stamp_ns, received_ros_ns, received_steady, copied)
        if not self._deferred_scan.offer(item):
            self._deferred_failure("scan", "scan_deferred_rejected", details={"stamp_ns": stamp_ns})
        self._try_pending()

    @staticmethod
    def _copy_scan(message: ScanMotion) -> ScanMotion:
        copied = ScanMotion()
        copied.header.stamp.sec = message.header.stamp.sec
        copied.header.stamp.nanosec = message.header.stamp.nanosec
        copied.header.frame_id = message.header.frame_id
        copied.interval_start.sec = message.interval_start.sec
        copied.interval_start.nanosec = message.interval_start.nanosec
        copied.source_epoch = message.source_epoch
        copied.valid, copied.reason = message.valid, message.reason
        copied.body_delta.x = message.body_delta.x
        copied.body_delta.y = message.body_delta.y
        copied.body_delta.theta = message.body_delta.theta
        copied.local_pose.x = message.local_pose.x
        copied.local_pose.y = message.local_pose.y
        copied.local_pose.theta = message.local_pose.theta
        copied.covariance = list(message.covariance)
        copied.response, copied.residual = message.response, message.residual
        copied.valid_rays = message.valid_rays
        copied.startup_reference_valid = message.startup_reference_valid
        copied.startup_reference_stamp.sec = message.startup_reference_stamp.sec
        copied.startup_reference_stamp.nanosec = message.startup_reference_stamp.nanosec
        return copied

    def _copy_scan_failure(self, message: ScanMotion) -> ScanMotion:
        """Keep exact bounded source identity/times without retaining invalid payloads."""
        copied = ScanMotion()
        copied.header.stamp.sec = message.header.stamp.sec
        copied.header.stamp.nanosec = message.header.stamp.nanosec
        copied.interval_start.sec = message.interval_start.sec
        copied.interval_start.nanosec = message.interval_start.nanosec
        copied.source_epoch = message.source_epoch
        if self._wire_fits(message.header.frame_id, 128):
            copied.header.frame_id = message.header.frame_id
        if not self._wire_fits(message.header.frame_id, 128) or not self._wire_fits(
            message.reason, 64
        ):
            copied.reason = "evidence_field_overflow"
        elif message.valid:
            copied.reason = "scan_shape_unqualified"
        else:
            copied.reason = message.reason
        return copied

    def _admit_scan(
        self, message: ScanMotion, received_steady: float, received_ros_ns: int
    ) -> None:
        if (
            message.source_epoch
            and self._wire_fits(message.source_epoch, 95)
            and self._scan_epochs.is_retired(message.source_epoch)
        ):
            if self._clock_ok() and self._scan_topology_ready():
                self._record_scan_event(message, "discarded", "retired_epoch")
            return
        if not self._wire_fits(message.source_epoch, 95):
            self._break_progress("scan_field_overflow")
            if self._pending is not None:
                self._record_scan_event(self._pending, "discarded", "input_field_overflow")
            self._pending = None
            self._invalid(message, "evidence_field_overflow")
            return
        if not self._clock_ok():
            self._invalid(message, "clock_rollback")
            return
        if not self._scan_topology_ready():
            self._pending = None
            self._invalid(
                message,
                "evidence_field_overflow"
                if not self._wire_fits(message.header.frame_id, 128)
                else "scan_producer_untrusted_or_retired",
            )
            return
        previous_epoch = self._scan_epochs.active
        if not self._scan_epochs.accept(message.source_epoch):
            if message.source_epoch and self._scan_epochs.active is not None:
                # A retired source cannot cancel a pending interval from its successor.
                self._record_scan_event(message, "discarded", "retired_epoch")
                return
            self._break_progress("source_epoch_invalid")
            if self._pending is not None:
                self._record_scan_event(self._pending, "discarded", "source_untrusted")
            self._pending = None
            self._invalid(message, "scan_producer_untrusted_or_retired")
            return
        if previous_epoch != message.source_epoch:
            if previous_epoch is not None:
                self._break_progress(
                    "source_epoch_changed",
                    details={
                        "previous_epoch": previous_epoch,
                        "new_epoch": message.source_epoch,
                        "valid": message.valid,
                        "reason": message.reason[:120],
                        "start_ns": message.interval_start.sec * 1_000_000_000
                        + message.interval_start.nanosec,
                        "end_ns": message.header.stamp.sec * 1_000_000_000
                        + message.header.stamp.nanosec,
                    },
                )
            self._scan_frontier.reset()
        stamp_ns = Time.from_msg(message.header.stamp).nanoseconds
        start_ns = Time.from_msg(message.interval_start).nanoseconds
        shape_valid = self._scan_shape_qualified(message)
        eligible = shape_valid and self._recent_stamp(stamp_ns / 1e9)
        if not self._scan_frontier.accept(
            stamp_ns, eligible=eligible, receipt_ros_ns=received_ros_ns
        ):
            self._record_scan_event(message, "discarded", "stamp_frontier")
            return
        if eligible and not self._scan_frontier.interval_after_failure(start_ns):
            self._record_scan_event(message, "discarded", "crosses_failed_interval")
            return
        if self._pending is not None:
            self._invalid(self._pending, "alignment_superseded")
            self._pending = None
        if not eligible:
            self._discard_deferred_scan_after_failure(message.source_epoch)
            self._break_progress(
                "source_observation_invalid",
                details={"reason": message.reason[:120], "start_ns": start_ns, "end_ns": stamp_ns},
            )
            self._progress_requalification.interrupt()
            if not self._wire_fits(message.header.frame_id, 128) or not self._wire_fits(
                message.reason, 64
            ):
                reason = "evidence_field_overflow"
            elif message.valid and not shape_valid:
                reason = "scan_shape_unqualified"
            elif message.valid:
                reason = "scan_stamp_unqualified"
            else:
                reason = message.reason or "scan_unavailable"
            self._invalid(message, reason)
            return
        if self._progress_started:
            try:
                item = self._progress_interval(message, received_steady, received_ros_ns)
            except (ValueError, OverflowError) as exc:
                self._break_progress("source_interval_invalid", details={"error": str(exc)[:120]})
                self._invalid(message, "progress_history_unqualified")
                return
            prior_queue = self._progress.diagnostic_state()
            if not self._progress.offer(item):
                if prior_queue["pending_count"] >= self._progress.max_pending:
                    cause = "queue_overflow"
                elif prior_queue["source_epoch"] not in (None, item.source_epoch):
                    cause = "queue_epoch_changed"
                elif prior_queue["last_end_ns"] not in (None, item.start_ns):
                    cause = "source_gap_or_overlap"
                else:
                    cause = "source_interval_rejected"
                self._break_progress(
                    cause, details={"incoming": asdict(item)}, prior_queue=prior_queue
                )
        elif not self._progress.unqualified and not self._progress_requalification.armed:
            try:
                self._progress_requalification.arm(
                    steady=time.monotonic(), ros_ns=self.get_clock().now().nanoseconds
                )
            except ValueError:
                self._invalid(message, "progress_history_unqualified")
                return
        self._pending = message
        self._pending_at = received_steady
        self._pending_received_ros_ns = received_ros_ns
        self._record_scan_event(message, "pending")

    @staticmethod
    def _progress_interval(
        message: ScanMotion, received_steady: float, received_ros_ns: int
    ) -> ProgressInterval:
        start_ns = Time.from_msg(message.interval_start).nanoseconds
        end_ns = Time.from_msg(message.header.stamp).nanoseconds
        delta = message.body_delta
        scan = ScanInterval(
            start_ns / 1e9,
            end_ns / 1e9,
            Pose2D(delta.x, delta.y, delta.theta),
            (message.covariance[0], message.covariance[4], message.covariance[8]),
            message.response,
        )
        return ProgressInterval(
            message.source_epoch, start_ns, end_ns, scan, received_steady, received_ros_ns
        )

    def _drain_deferred(self) -> bool:
        """Do bounded work; never age a queued observation by its dequeue time."""
        queues = (
            ("wheel", self._deferred_wheel),
            ("imu", self._deferred_imu),
            ("scan", self._deferred_scan),
        )
        for _ in range(8):
            selected = None
            for offset in range(len(queues)):
                index = (self._deferred_cursor + offset) % len(queues)
                name, queue = queues[index]
                head = queue.peek()
                if head is None:
                    continue
                now_ros_ns = self.get_clock().now().nanoseconds
                now_steady = time.monotonic()
                status = queue.status(
                    now_ros_ns=now_ros_ns,
                    now_steady=now_steady,
                    max_age=self._max_age,
                    pending_wait=self._wait,
                )
                if status in ("expired", "clock_rollback"):
                    self._deferred_failure(
                        name,
                        f"{name}_deferred_{status}",
                        details={
                            "stamp_ns": head.stamp_ns,
                            "received_ros_ns": head.received_ros_ns,
                            "received_steady": head.received_steady,
                            "now_ros_ns": now_ros_ns,
                            "now_steady": now_steady,
                        },
                    )
                    return False
                if status == "ready":
                    selected = (index, name, queue)
                    break
            if selected is None:
                break
            index, name, queue = selected
            self._deferred_cursor = (index + 1) % len(queues)
            item = queue.consume()
            assert item is not None
            if name == "wheel":
                if not self._admit_wheel(item):
                    return False
            elif name == "imu":
                if not self._admit_imu(item):
                    return False
            else:
                self._admit_scan(item.value, item.received_steady, item.received_ros_ns)
        return True

    def _try_pending(self) -> None:
        if not self._clock_ok() or not self._scan_topology_ready():
            self._pending = None
            return
        if not self._drain_deferred():
            self._pending = None
            return
        message = self._pending
        if message is None:
            return
        steady = time.monotonic()
        try:
            current = self._progress_interval(
                message, self._pending_at, self._pending_received_ros_ns
            )
        except (ValueError, OverflowError) as exc:
            self._break_progress("pending_interval_invalid", details={"error": str(exc)[:120]})
            self._invalid(message, "progress_history_unqualified")
            self._pending = None
            return
        now_ros_ns = self.get_clock().now().nanoseconds
        if not progress_interval_fresh(
            current,
            now_steady=steady,
            now_ros_ns=now_ros_ns,
            max_age=self._max_age,
            pending_wait=self._wait,
        ):
            self._break_progress(
                "pending_deadline_expired",
                details={"current": asdict(current), "now_ros_ns": now_ros_ns},
            )
            self._invalid(message, "progress_history_unqualified")
            self._pending = None
            return
        fresh = all(
            received is not None and 0 <= steady - received <= self._max_age
            for received in (self._wheel_received, self._imu_received)
        )
        if not self._progress_started:
            if not self._progress_requalification.armed:
                self._invalid(message, "progress_history_unqualified")
                self._pending = None
                return
            command = self._commands.integrated(current.scan.start, current.scan.end)
            aligned = None
            if fresh and command is not None:
                try:
                    aligned = align_interval(current.scan, self._wheel, self._imu, self._estimator)
                except (ValueError, OverflowError):
                    aligned = None
            if aligned is None or command is None:
                if steady - self._pending_at >= self._wait:
                    self._invalid(message, "progress_history_unqualified")
                    self._pending = None
                return
            try:
                stationary = stationary_interval(
                    aligned,
                    command,
                    max_linear_speed=self._stationary_linear,
                    max_yaw_rate=self._stationary_yaw,
                )
            except (ValueError, OverflowError):
                stationary = False
            ready = self._progress_requalification.observe(
                current.source_epoch,
                current.start_ns,
                current.end_ns,
                stationary=stationary,
                steady=steady,
                ros_ns=now_ros_ns,
            )
            if not ready or self._slip.latched:
                self._invalid(message, "progress_history_unqualified")
                self._pending = None
                return
            self._progress.reset_after_requalification()
            self._slip.reset()
            self._progress_started = True
            if not self._progress.offer(current):
                self._break_progress(
                    "initial_progress_offer_rejected", details={"current": asdict(current)}
                )
                self._invalid(message, "progress_history_unqualified")
                self._pending = None
                return

        result = None
        latch_item = None
        previous_accepted = self._last_accepted_scan
        # Process a fixed maximum per callback, always in exact source order.
        for _ in range(8):
            head = self._progress.peek()
            if head is None:
                break
            if not self._clock_ok():
                self._pending = None
                if latch_item is not None:
                    self._log_first_latch(latch_item, previous_accepted)
                return
            step_steady = time.monotonic()
            step_ros_ns = self.get_clock().now().nanoseconds
            if not progress_interval_fresh(
                head,
                now_steady=step_steady,
                now_ros_ns=step_ros_ns,
                max_age=self._max_age,
                pending_wait=self._wait,
            ):
                self._break_progress(
                    "queued_interval_deadline_expired",
                    details={
                        "head": asdict(head),
                        "now_ros_ns": step_ros_ns,
                        "now_steady": step_steady,
                    },
                )
                self._invalid(message, "progress_history_unqualified")
                self._pending = None
                if latch_item is not None:
                    self._log_first_latch(latch_item, previous_accepted)
                return
            if not all(
                received is not None and 0 <= step_steady - received <= self._max_age
                for received in (self._wheel_received, self._imu_received)
            ):
                break
            command = self._commands.integrated(head.scan.start, head.scan.end)
            if command is None:
                break
            previous_latch = self._slip.first_latch
            try:
                qualified = qualify_interval(
                    head.scan, self._wheel, self._imu, command.velocity, self._estimator, self._slip
                )
            except (ValueError, OverflowError):
                qualified = None
            if qualified is None:
                break
            self._progress.consume()
            self._scan_events.append(
                {
                    "source_epoch": head.source_epoch,
                    "start_ns": head.start_ns,
                    "end_ns": head.end_ns,
                    "status": "progress_processed",
                    "reason": "",
                }
            )
            if previous_latch is None and self._slip.first_latch is not None:
                latch_item = head
                self._stop_acknowledgement.trip()
            if head.source_epoch == current.source_epoch and head.end_ns == current.end_ns:
                result = qualified
                break
        if result is None:
            if latch_item is not None:
                self._invalid(message, "motion_anomaly")
                self._pending = None
                self._log_first_latch(latch_item, previous_accepted)
            elif steady - self._pending_at >= self._wait:
                self._break_progress("alignment_wait_expired", details={"current": asdict(current)})
                self._invalid(message, "progress_history_unqualified")
                self._pending = None
            return
        if not self._clock_ok():
            self._pending = None
            if latch_item is not None:
                self._log_first_latch(latch_item, previous_accepted)
            return
        publish_steady = time.monotonic()
        if not progress_interval_fresh(
            current,
            now_steady=publish_steady,
            now_ros_ns=self.get_clock().now().nanoseconds,
            max_age=self._max_age,
            pending_wait=self._wait,
        ) or not all(
            received is not None and 0 <= publish_steady - received <= self._max_age
            for received in (self._wheel_received, self._imu_received)
        ):
            self._break_progress(
                "publish_deadline_or_sensor_stale",
                details={"current": asdict(current), "publish_steady": publish_steady},
            )
            self._invalid(message, "progress_history_unqualified")
            self._pending = None
            if latch_item is not None:
                self._log_first_latch(latch_item, previous_accepted)
            return
        body_twist = result.estimate.body_twist
        if body_twist is None:
            self._break_progress("independent_twist_unavailable")
            self._invalid(message, "independent_body_twist_unavailable")
            self._pending = None
            if latch_item is not None:
                self._log_first_latch(latch_item, previous_accepted)
            return
        output = self._new_evidence(message)
        output.valid = True
        output.scan_received_steady = current.received_steady
        output.scan_received_ros = Time(nanoseconds=current.received_ros_ns).to_msg()
        output.reason = "motion_anomaly" if result.anomalous else "qualified"
        output.body_delta = message.body_delta
        output.local_pose = message.local_pose
        output.body_velocity.linear.x = body_twist.vx
        output.body_velocity.linear.y = body_twist.vy
        output.body_velocity.angular.z = body_twist.wz
        output.covariance = message.covariance
        output.quality = result.estimate.quality
        output.imu_valid = output.wheel_valid = True
        output.motion_anomaly = result.anomalous
        if result.anomalous:
            self._stop_acknowledgement.trip()
        # A continuously tracked scan origin alone does not prove the power-on pose.
        output.startup_reference_valid = False
        self._publisher.publish(output)
        self._last_valid_source = self._copy_scan(message)
        self._pending = None
        self._record_scan_event(message, "accepted")
        self._last_accepted_scan = {
            "source_epoch": message.source_epoch,
            "end_ns": current.end_ns,
        }
        if latch_item is not None:
            self._log_first_latch(latch_item, previous_accepted)

    def _log_first_latch(self, item, previous_accepted) -> None:
        latch = self._slip.first_latch
        if latch is None:
            return
        # Safety evidence is already published before the bounded diagnostic.
        recent_intervals = [
            {
                "command": asdict(entry.command),
                "wheel": asdict(entry.wheel),
                "body_delta": asdict(entry.actual.pose_delta),
                "body_dt": entry.actual.dt,
                "body_stamp": entry.actual.stamp,
                "body_quality": entry.actual.quality,
                "body_covariance": entry.actual.covariance,
            }
            for entry in self._slip.recent_intervals
        ]
        snapshot = {
            "source_epoch": item.source_epoch,
            "start_ns": item.start_ns,
            "end_ns": item.end_ns,
            "previous_accepted": previous_accepted,
            "gap_since_previous_ns": (
                item.start_ns - previous_accepted["end_ns"]
                if previous_accepted is not None
                and previous_accepted["source_epoch"] == item.source_epoch
                else None
            ),
            "scan_events": list(self._scan_events),
            "latch": asdict(latch),
            "recent_intervals": recent_intervals,
        }
        self.get_logger().warning(
            "motion_first_latch " + json.dumps(snapshot, sort_keys=True, separators=(",", ":"))
        )

    def _scan_topology_ready(self) -> bool:
        endpoints = self.get_publishers_info_by_topic("/motion/scan")
        ready, changed = self._scan_topology.observe(
            [
                (endpoint.node_name, endpoint.node_namespace, bytes(endpoint.endpoint_gid))
                for endpoint in endpoints
            ]
        )
        if changed or not ready:
            was_active = self._progress_started
            deferred = self._deferred_scan.peek()
            reserved_epoch = deferred.value.source_epoch if deferred is not None else None
            self._break_progress(
                "source_topology_changed",
                details={"endpoint_count": len(endpoints), "ready": ready, "changed": changed},
            )
            self._progress_requalification.disarm()
            self._clear_deferred()
            if reserved_epoch is not None:
                self._scan_epochs.retire_unadmitted(reserved_epoch)
            self._scan_epochs.retire_active()
            self._scan_frontier.reset()
            if self._pending is not None:
                self._record_scan_event(self._pending, "discarded", "topology_changed")
            self._pending = None
            if was_active and self._last_valid_source is not None:
                self._invalid(self._last_valid_source, "scan_producer_untrusted_or_retired")
        return ready


def main() -> None:
    run(MotionFusion, cli=True, executor_factory=MotionExecutor)
