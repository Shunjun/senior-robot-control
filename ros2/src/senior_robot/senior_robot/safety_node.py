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
from collections import deque

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from rclpy.clock import Clock as RosClock
from rclpy.clock import ClockType
from rclpy.exceptions import ParameterUninitializedException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from senior_motion_msgs.msg import MotionEvidence
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from senior_control.config import DeviceProfile
from senior_control.models import Pose2D, Velocity
from senior_control.safety.async_guard import (
    BoundedSafetyWorker,
    SafetyTicket,
    SafetyWork,
    SafetyWorkResult,
)
from senior_control.safety.freshness import SafetyFreshness
from senior_control.safety.guard import command_expired
from senior_control.safety.motion_estimator import body_twist_delta
from senior_control.safety.motion_runtime import (
    ProducerTopology,
    SourceEpochFence,
    SourceStampFrontier,
    StampedObservation,
    TimedPoseBuffer,
    TimedPoseSample,
    angle,
    compose_pose,
    reference_steady_deadline,
    relative_pose,
)
from senior_control.safety.self_filter import FilteredObservation, ScanObservation, filter_self
from senior_robot.motion_lifecycle import run


def _positive(value: float) -> float | None:
    if not math.isfinite(value) or value <= 0.0:
        return None
    return value


def _nonnegative(value: float) -> float | None:
    if not math.isfinite(value) or value < 0.0:
        return None
    return value


class SafetyGate(Node):
    def __init__(self, *, parameter_overrides=None) -> None:
        super().__init__("safety_gate", parameter_overrides=parameter_overrides or [])
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
        # //Compatible: unset profiles retain exact-mask filtering, uncalibrated by default.
        self.declare_parameter("self_mask_range_uncertainty", 0.0)
        self.declare_parameter("transform_static", False)
        self.declare_parameter("interpolate", False)
        self.declare_parameter("max_angular_gap", 0.0)
        self.declare_parameter("no_return_clear", False)
        self.declare_parameter("command_topic", "/safety/command_in")
        self.declare_parameter("evidence_topic", "/motion/evidence")
        self.declare_parameter("measurement_max_age", -1.0)
        self.declare_parameter("motion_min_quality", -1.0)
        self.declare_parameter("motion_max_variance", -1.0)
        self.declare_parameter("motion_max_interval", -1.0)
        self.declare_parameter("max_reference_lag", -1.0)
        self.declare_parameter("max_linear_speed", -1.0)
        self.declare_parameter("max_yaw_rate", -1.0)
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
        self._scans: deque[StampedObservation] = deque(maxlen=8)
        self._last_ros: float | None = None
        self._last_ros_advance_steady: float | None = None
        self._epoch_fence = SourceEpochFence(max_retired=256)
        self._motion_frontier = SourceStampFrontier()
        self._evidence_topology = ProducerTopology("motion_fusion", "/")
        self._command_topology = ProducerTopology("command_node", "/")
        self._command_generation = 0
        self._scan_generation = 0
        self._motion_generation = 0
        self._worker = BoundedSafetyWorker()
        self._approved: tuple[SafetyTicket, SafetyWorkResult] | None = None
        self._worker_finished = 0
        self._worker_discarded = 0
        self._worker_last_result = ""
        self._history = self._new_history()
        self._reference_admission: tuple[float, float, float] | None = None
        scan_age = _positive(float(self.get_parameter("observation_max_age").value))
        measured_age = _positive(float(self.get_parameter("measurement_max_age").value))
        self._freshness = (
            SafetyFreshness(min(scan_age, measured_age)) if scan_age and measured_age else None
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        output = str(self.get_parameter("output_topic").value)
        command_topic = str(self.get_parameter("command_topic").value)
        self._command_topic = command_topic
        scan_topic = str(self.get_parameter("scan_topic").value)
        self._publisher = self.create_publisher(Twist, output, 10)
        self._status_publisher = self.create_publisher(DiagnosticArray, "/safety/status", 10)
        self._status_counts: dict[str, int] = {}
        self._last_status = float("-inf")
        self._status_geometry: dict[str, object] = {}
        self.create_subscription(Twist, command_topic, self._on_command, 10)
        evidence_topic = str(self.get_parameter("evidence_topic").value)
        self._evidence_topic = evidence_topic
        if evidence_topic:
            self.create_subscription(MotionEvidence, evidence_topic, self._on_evidence, 10)
        self.create_subscription(LaserScan, scan_topic, self._on_scan, qos_profile_sensor_data)
        self.create_timer(
            1.0 / rate, self._on_timer, clock=RosClock(clock_type=ClockType.STEADY_TIME)
        )

    def _array_parameter(self, name: str) -> list[float]:
        try:
            return self.get_parameter(name).value or []
        except ParameterUninitializedException:
            return []

    def _profile(self) -> DeviceProfile:
        footprint_xy = self._array_parameter("footprint_xy")
        mask_xy = self._array_parameter("self_mask_xy")
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

    def _new_history(self) -> TimedPoseBuffer | None:
        gap = _positive(float(self.get_parameter("motion_max_interval").value))
        variance = _positive(float(self.get_parameter("motion_max_variance").value))
        if gap is None or variance is None:
            return None
        return TimedPoseBuffer(max_gap=gap, max_samples=64, max_variance=variance)

    def _clock_ok(self, now: float) -> bool:
        if not math.isfinite(now) or (self._last_ros is not None and now < self._last_ros):
            self._scan_generation += 1
            self._motion_generation += 1
            self._epoch_fence.retire_active()
            self._motion_frontier.reset()
            self._history = self._new_history()
            self._reference_admission = None
            self._filtered = None
            self._scans.clear()
            if self._freshness is not None:
                self._freshness.scan_received = self._freshness.measured_received = None
            self._last_ros = now if math.isfinite(now) else None
            self._last_ros_advance_steady = time.monotonic() if math.isfinite(now) else None
            return False
        if self._last_ros is None or now > self._last_ros:
            self._last_ros_advance_steady = time.monotonic()
        self._last_ros = now
        return True

    def _on_command(self, msg: Twist) -> None:
        if not self._command_topology_ready():
            return
        velocity = self._velocity(msg)
        if velocity is not None:
            if velocity != self._command:
                self._command_generation += 1
            self._command = velocity
            self._command_monotonic = time.monotonic()

    def _command_topology_ready(self) -> bool:
        endpoints = (
            self.get_publishers_info_by_topic(self._command_topic) if self._command_topic else []
        )
        ready, changed = self._command_topology.observe(
            [
                (endpoint.node_name, endpoint.node_namespace, bytes(endpoint.endpoint_gid))
                for endpoint in endpoints
            ]
        )
        if changed or not ready:
            if changed:
                self._command_generation += 1
            self._command = Velocity(0.0, 0.0)
            self._command_monotonic = None
        return ready

    def _topology_ready(self) -> bool:
        if not self._evidence_topic:
            return False
        endpoints = self.get_publishers_info_by_topic(self._evidence_topic)
        ready, changed = self._evidence_topology.observe(
            [
                (endpoint.node_name, endpoint.node_namespace, bytes(endpoint.endpoint_gid))
                for endpoint in endpoints
            ]
        )
        if changed or not ready:
            if changed:
                self._motion_generation += 1
            self._epoch_fence.retire_active()
            self._motion_frontier.reset()
            self._history = self._new_history()
            self._reference_admission = None
            if self._freshness is not None:
                self._freshness.measured_received = None
        return ready

    def _on_evidence(self, msg: MotionEvidence) -> None:
        if self._topology_ready():
            self._accept_evidence(msg)

    def _accept_evidence(self, msg: MotionEvidence) -> None:
        previous_epoch = self._epoch_fence.active
        if msg.source_epoch:
            if not self._epoch_fence.accept(msg.source_epoch):
                # Retired traffic cannot revoke or resurrect the current epoch.
                # Exhaustion still fails closed because it has no active epoch.
                if self._epoch_fence.active is None:
                    self._history = self._new_history()
                    self._reference_admission = None
                    if self._freshness is not None:
                        self._freshness.measured_received = None
                return
            if previous_epoch != msg.source_epoch:
                self._motion_frontier.reset()
            if previous_epoch is not None and previous_epoch != msg.source_epoch:
                self._history = self._new_history()
                self._reference_admission = None
                if self._freshness is not None:
                    self._freshness.measured_received = None
        received = time.monotonic()
        now_ns = self.get_clock().now().nanoseconds
        stamp_ns = Time.from_msg(msg.header.stamp).nanoseconds
        now = now_ns / 1e9
        stamp = stamp_ns / 1e9
        start_ns = Time.from_msg(msg.interval_start).nanoseconds
        start = start_ns / 1e9
        quality_min = _positive(float(self.get_parameter("motion_min_quality").value))
        maximum = _positive(float(self.get_parameter("motion_max_variance").value))
        interval_max = _positive(float(self.get_parameter("motion_max_interval").value))
        covariance = tuple(float(msg.covariance[i]) for i in (0, 4, 8))
        try:
            dt = stamp - start
            valid = (
                self._clock_ok(now)
                and self._freshness is not None
                and self._history is not None
                and msg.valid
                and msg.imu_valid
                and msg.wheel_valid
                and not msg.motion_anomaly
                and msg.header.frame_id == self._body
                and bool(msg.source_epoch)
                and quality_min is not None
                and maximum is not None
                and interval_max is not None
                and math.isfinite(msg.quality)
                and quality_min <= msg.quality <= 1.0
                and 0 < dt <= interval_max
                and start > 0
                and all(math.isfinite(v) and 0 < v <= maximum for v in covariance)
                and all(math.isfinite(v) for v in msg.covariance)
                and all(
                    math.isfinite(v)
                    for v in (
                        msg.body_delta.x,
                        msg.body_delta.y,
                        msg.body_delta.theta,
                        msg.local_pose.x,
                        msg.local_pose.y,
                        msg.local_pose.theta,
                    )
                )
            )
            if not self._motion_frontier.accept(stamp_ns, eligible=valid, receipt_ros_ns=now_ns):
                return
            if valid and not self._motion_frontier.interval_after_failure(start_ns):
                self._motion_generation += 1
                raise ValueError("motion interval crosses a prior failure")
            self._motion_generation += 1
            if not valid:
                raise ValueError("motion evidence unqualified")
            delta = Pose2D(msg.body_delta.x, msg.body_delta.y, msg.body_delta.theta)
            pose = Pose2D(msg.local_pose.x, msg.local_pose.y, msg.local_pose.theta)
            assert self._history is not None and self._freshness is not None
            latest = self._history.latest
            if latest is not None:
                expected = compose_pose(latest.pose, delta)
                if (
                    latest.epoch != msg.source_epoch
                    or abs(latest.stamp - start) > 1e-6
                    or math.hypot(expected.x - pose.x, expected.y - pose.y) > 1e-5
                    or abs(angle(expected.yaw - pose.yaw)) > 1e-5
                ):
                    raise ValueError("motion continuity lost")
            else:
                initial = compose_pose(pose, relative_pose(delta, Pose2D(0, 0, 0)))
                if not self._history.add(
                    TimedPoseSample(start, initial, covariance, msg.source_epoch)
                ):
                    raise ValueError("initial pose unavailable")
            if not self._history.add(
                TimedPoseSample(stamp, pose, covariance, msg.source_epoch)
            ) or not self._freshness.measured(stamp, received, now):
                raise ValueError("stale motion evidence")
            if self._last_ros_advance_steady is None:
                raise ValueError("clock progress unavailable")
            self._reference_admission = (stamp, now, min(received, self._last_ros_advance_steady))
            twist = body_twist_delta(delta)
            self._measured = Velocity(twist.x / dt, twist.yaw / dt)
        except (ValueError, OverflowError):
            self._history = self._new_history()
            self._reference_admission = None
            if self._freshness is not None:
                self._freshness.measured_received = None

    def _on_scan(self, msg: LaserScan) -> None:
        self._scan_generation += 1
        if self._freshness is None:
            return
        try:
            stamp_time = Time.from_msg(msg.header.stamp)
            stamp = stamp_time.nanoseconds / 1e9
            received = time.monotonic()
            now = self.get_clock().now().nanoseconds / 1e9
            if (
                not self._clock_ok(now)
                or not msg.header.frame_id
                or not math.isfinite(msg.time_increment)
                or msg.time_increment != 0.0
                or not self._freshness.scan(stamp, received, now)
            ):
                raise ValueError("stale or out-of-order scan")
            tf = self._tf_buffer.lookup_transform(self._body, msg.header.frame_id, stamp_time)
            q = tf.transform.rotation
            values = (q.x, q.y, q.z, q.w, tf.transform.translation.x, tf.transform.translation.y)
            if (
                not all(math.isfinite(v) for v in values)
                or abs(sum(v * v for v in (q.x, q.y, q.z, q.w)) - 1) > 0.001
                or abs(q.x) > 0.001
                or abs(q.y) > 0.001
            ):
                raise ValueError("invalid planar transform")
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            tf_stamp = Time.from_msg(tf.header.stamp).nanoseconds / 1e9
            static = bool(self.get_parameter("transform_static").value)
            if not static and not 0 <= stamp - tf_stamp <= self._freshness.max_age:
                raise ValueError("stale transform")
            observation = ScanObservation(
                tuple(msg.ranges),
                float(msg.angle_min),
                float(msg.angle_increment),
                float(msg.range_min),
                float(msg.range_max),
                stamp,
                "ros",
                Pose2D(tf.transform.translation.x, tf.transform.translation.y, yaw),
                tf_stamp,
                self._body,
                stamp,
                bool(self.get_parameter("interpolate").value),
                float(self.get_parameter("max_angular_gap").value),
                bool(self.get_parameter("no_return_clear").value),
                static,
            )
            self._filtered = filter_self(
                observation,
                self._profile(),
                self_mask_range_uncertainty=float(
                    self.get_parameter("self_mask_range_uncertainty").value
                ),
            )
            if not self._filtered.free_rays:
                raise ValueError("scan has no usable returns")
            self._scans.append(
                StampedObservation(self._filtered, received, now, self._last_ros_advance_steady)
            )
        except (ValueError, OverflowError, TransformException):
            self._filtered = None
            self._scans.clear()
            self._freshness.scan_received = None

    def _publish(self, velocity: Velocity, reason: str) -> None:
        twist = Twist()
        twist.linear.x = velocity.vx
        twist.angular.z = velocity.wz
        self._publisher.publish(twist)
        self._report(reason)

    def _report(self, reason: str) -> None:
        """Publish bounded rate/counters; diagnostics never grant motion."""
        if reason not in self._status_counts and len(self._status_counts) >= 32:
            reason = "other"
        self._status_counts[reason] = min(self._status_counts.get(reason, 0) + 1, 2**31 - 1)
        steady = time.monotonic()
        if steady - self._last_status < 0.5:
            return
        self._last_status = steady
        now = self.get_clock().now().nanoseconds / 1e9
        freshness = self._freshness
        latest = self._history.latest if self._history is not None else None

        def age(value: float | None, reference: float) -> str:
            return "unavailable" if value is None else f"{reference - value:.6f}"

        values = {
            "request_vx": str(self._command.vx),
            "request_wz": str(self._command.wz),
            "measured_vx": str(self._measured.vx),
            "measured_wz": str(self._measured.wz),
            "command_steady_age": age(self._command_monotonic, steady),
            "scan_ros_age": age(freshness.scan_stamp if freshness else None, now),
            "scan_steady_age": age(freshness.scan_received if freshness else None, steady),
            "motion_ros_age": age(freshness.measured_stamp if freshness else None, now),
            "motion_steady_age": age(freshness.measured_received if freshness else None, steady),
            "reference_ros_age": age(latest.stamp if latest else None, now),
            "buffered_scans": str(len(self._scans)),
            "self_masked_rays": str(len(self._filtered.occluded_regions) if self._filtered else 0),
            "ambiguous_self_overlap_rays": str(
                sum(ray.kind == "ambiguous_self_overlap" for ray in self._filtered.rays)
                if self._filtered
                else 0
            ),
            "worker_finished": str(self._worker_finished),
            "worker_discarded": str(self._worker_discarded),
            "worker_last_result": self._worker_last_result,
            **{key: str(value) for key, value in self._status_geometry.items()},
            **{f"count.{key}": str(value) for key, value in self._status_counts.items()},
        }
        status = DiagnosticStatus()
        status.name = "safety_gate"
        status.hardware_id = self._body
        status.level = (
            DiagnosticStatus.OK
            if reason in ("hard_sweep_clear", "warning_overlap")
            else DiagnosticStatus.WARN
        )
        status.message = reason
        status.values = [KeyValue(key=key, value=value) for key, value in values.items()]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self._status_publisher.publish(array)

    def _on_timer(self) -> None:
        self._status_geometry = {}
        command_ready = self._command_topology_ready()
        if not self._topology_ready():
            self._publish(Velocity(0.0, 0.0), "evidence_topology_unqualified")
            return
        if not command_ready:
            self._publish(Velocity(0.0, 0.0), "command_topology_unqualified")
            return
        ttl = _positive(float(self.get_parameter("command_ttl_seconds").value))
        if command_expired(self._command_monotonic, time.monotonic(), ttl):
            self._approved = None
            self._publish(Velocity(0.0, 0.0), "command_expired")
            return
        if self._command == Velocity(0.0, 0.0):
            self._approved = None
            self._publish(Velocity(0.0, 0.0), "zero_requested")
            return
        time_reference = time.monotonic()
        now = self.get_clock().now().nanoseconds / 1e9
        if (
            not self._clock_ok(now)
            or self._freshness is None
            or not self._freshness.usable(now, time.monotonic())
        ):
            self._approved = None
            self._publish(Velocity(0.0, 0.0), "sensor_freshness_unqualified")
            return
        try:
            profile = self._profile()
            speed = _positive(float(self.get_parameter("max_linear_speed").value))
            yaw_rate = _positive(float(self.get_parameter("max_yaw_rate").value))
            max_lag = _positive(float(self.get_parameter("max_reference_lag").value))
            if (
                self._filtered is None
                or self._history is None
                or speed is None
                or yaw_rate is None
                or max_lag is None
                or profile.observation_max_age is None
                or profile.hard_margin is None
                or profile.braking_deceleration is None
                or profile.reaction_time is None
                or not profile.footprint
                or not self._scans
                or self._history.latest is None
                or not self._epoch_fence.active
                or self._history.latest.epoch != self._epoch_fence.active
                or self._reference_admission is None
                or self._reference_admission[0] != self._history.latest.stamp
                or self._freshness.scan_received is None
                or self._freshness.measured_received is None
                or abs(self._command.vx) > speed
                or abs(self._measured.vx) > speed
                or abs(self._command.wz) > yaw_rate
                or abs(self._measured.wz) > yaw_rate
            ):
                raise ValueError("unqualified temporal profile")
            radius = (
                max(math.hypot(x, y) for x, y in profile.footprint)
                + speed * profile.reaction_time
                + speed * speed / (2 * profile.braking_deceleration)
                + profile.hard_margin
            )
            configuration = (
                profile.model_dump_json(),
                speed,
                yaw_rate,
                max_lag,
                ttl,
                self.get_parameter("interpolate").value,
                self.get_parameter("max_angular_gap").value,
                self.get_parameter("transform_static").value,
                self.get_parameter("self_mask_range_uncertainty").value,
            )
            steady = time.monotonic()
            reply = self._worker.poll()
            if reply is not None:
                self._worker_finished += 1
                self._worker_last_result = reply.error or "finished"
                self._approved = (
                    (reply.ticket, reply.value)
                    if reply.error is None and isinstance(reply.value, SafetyWorkResult)
                    else None
                )
            if self._approved is not None:
                ticket, result = self._approved
                current_ros = self.get_clock().now().nanoseconds / 1e9
                current_steady = time.monotonic()
                valid = (
                    self._command_topology_ready()
                    and self._topology_ready()
                    and ticket.permits(
                        command_generation=self._command_generation,
                        scan_generation=self._scan_generation,
                        motion_generation=self._motion_generation,
                        epoch=self._epoch_fence.active,
                        command=self._command,
                        steady=current_steady,
                        ros=current_ros,
                        configuration=configuration,
                    )
                    and self._clock_ok(current_ros)
                    and self._freshness.usable(current_ros, current_steady)
                    and (not result.decision.allowed or result.decision.velocity == ticket.command)
                    and (
                        not result.decision.allowed
                        or (
                            result.selected_received is not None
                            and 0
                            <= current_steady - result.selected_received
                            <= profile.observation_max_age
                        )
                    )
                )
                if valid:
                    self._status_geometry = {
                        "compute_seconds": result.compute_seconds,
                        "evaluation_total_seconds": current_steady - ticket.steady_start,
                        "geometry_steady_deadline": ticket.steady_deadline,
                        "geometry_ros_deadline": ticket.ros_deadline,
                        "selected_scan_stamp": result.selected_stamp,
                        "reference_stamp": result.reference_stamp,
                        "uncertainty_margin": result.uncertainty_margin,
                        **result.diagnostics,
                    }
                    if result.sensor_origin is not None:
                        self._status_geometry.update(
                            zip(
                                ("sensor_origin_x", "sensor_origin_y", "sensor_yaw"),
                                result.sensor_origin,
                                strict=True,
                            )
                        )
                    final_ros = self.get_clock().now().nanoseconds / 1e9
                    final_steady = time.monotonic()
                    if (
                        ticket.permits(
                            command_generation=self._command_generation,
                            scan_generation=self._scan_generation,
                            motion_generation=self._motion_generation,
                            epoch=self._epoch_fence.active,
                            command=self._command,
                            steady=final_steady,
                            ros=final_ros,
                            configuration=configuration,
                        )
                        and self._clock_ok(final_ros)
                        and self._freshness.usable(final_ros, final_steady)
                        and (
                            not result.decision.allowed
                            or (
                                result.selected_received is not None
                                and 0
                                <= final_steady - result.selected_received
                                <= profile.observation_max_age
                            )
                        )
                    ):
                        self._publish(result.decision.velocity, result.decision.reason)
                        return
                self._worker_discarded += 1
                self._worker_last_result = (
                    f"discarded cmd={ticket.command_generation}/{self._command_generation} "
                    f"scan={ticket.scan_generation}/{self._scan_generation} "
                    f"motion={ticket.motion_generation}/{self._motion_generation} "
                    f"steady_left={ticket.steady_deadline - current_steady:.6f} "
                    f"ros_left={ticket.ros_deadline - current_ros:.6f} "
                    f"compute={result.compute_seconds:.6f}"
                )
                self._approved = None
            if self._worker.pending:
                self._publish(Velocity(0.0, 0.0), "evaluation_pending")
                return
            latest = self._history.latest
            assert latest is not None and self._command_monotonic is not None and ttl is not None
            assert self._reference_admission is not None
            deadline_ros = math.nextafter(latest.stamp + max_lag, -math.inf)
            remaining = deadline_ros - now
            if remaining <= 0:
                raise ValueError("reference window expired")
            fixed_deadline = reference_steady_deadline(*self._reference_admission, max_lag)
            if fixed_deadline is None:
                raise ValueError("reference admission expired")
            for sample in self._scans:
                if (
                    sample.observation.scan.stamp == latest.stamp
                    and sample.received_ros is not None
                    and sample.clock_advance_steady is not None
                ):
                    scan_deadline = reference_steady_deadline(
                        latest.stamp,
                        sample.received_ros,
                        min(sample.received, sample.clock_advance_steady),
                        max_lag,
                    )
                    if scan_deadline is None:
                        raise ValueError("scan reference admission expired")
                    fixed_deadline = min(fixed_deadline, scan_deadline)
            deadline_steady = min(
                self._command_monotonic + ttl,
                self._freshness.scan_received + self._freshness.max_age,
                self._freshness.measured_received + self._freshness.max_age,
                time_reference + remaining,
                fixed_deadline,
            )
            if deadline_steady <= steady:
                raise ValueError("steady window expired")
            ticket = SafetyTicket(
                self._command_generation,
                self._scan_generation,
                self._motion_generation,
                self._epoch_fence.active,
                self._command,
                steady,
                deadline_steady,
                now,
                deadline_ros,
                configuration,
            )
            work = SafetyWork(
                self._command,
                self._measured,
                profile,
                tuple(self._scans),
                self._history.snapshot(),
                deadline_ros,
                steady,
                max_lag,
                speed,
                yaw_rate,
                radius,
                time.monotonic() - self._last_status >= 0.5,
            )
            self._worker.submit(ticket, work)
        except ValueError as exc:
            self._publish(Velocity(0.0, 0.0), str(exc))
            return
        self._publish(Velocity(0.0, 0.0), "evaluation_pending")

    def destroy_node(self):
        self._worker.close()
        return super().destroy_node()


def main() -> None:
    run(SafetyGate)
