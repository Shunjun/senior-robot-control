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
import socket
import struct
import subprocess
import time
from collections import deque
from pathlib import Path
from types import SimpleNamespace

import rclpy
from ament_index_python.packages import get_package_prefix
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from rclpy.clock import Clock as RosClock
from rclpy.clock import ClockType
from rclpy.exceptions import ParameterUninitializedException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from senior_motion_msgs.msg import (
    BoundedDepthEvidence,
    FencedDepthEvidence,
    FencedSafetyScan,
    MotionEvidence,
)
from senior_motion_msgs.msg import (
    DriveExecutionFeedback as DriveFeedbackMessage,
)

from senior_control.config import DeviceProfile
from senior_control.models import BodyTwist, Pose2D, Velocity
from senior_control.protocol import DriveExecutionFeedback, DriveFeedbackKey
from senior_control.safety.arbiter import FinalCommandFence
from senior_control.safety.async_guard import (
    BoundedSafetyWorker,
    RawSafetyObservation,
    SafetyTicket,
    SafetyWork,
    SafetyWorkResult,
    WorkerReply,
)
from senior_control.safety.drive_feedback import OutputFeedback
from senior_control.safety.freshness import SafetyFreshness
from senior_control.safety.guard import command_expired
from senior_control.safety.motion_estimator import body_twist_delta
from senior_control.safety.motion_runtime import (
    ProducerTopology,
    SourceEpochFence,
    SourceStampFrontier,
    TimedPoseBuffer,
    TimedPoseSample,
    angle,
    compose_pose,
    reference_steady_deadline,
    relative_pose,
)
from senior_control.safety.velocity_uncertainty import (
    PHYSICAL_CAP_CONTEXT,
    IntervalVelocityBounds,
    interval_velocity_bounds,
)
from senior_control.vision.egress_receipt import (
    EgressAdmission,
    depth_capture_headers,
    egress_from_message,
    safety_scan_capture,
)
from senior_control.vision.raw_capture import RawCaptureAdmission
from senior_control.vision.runtime_evidence import DepthAdmission, DepthHeader
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
        super().__init__(
            "safety_gate",
            parameter_overrides=parameter_overrides or [],
            start_parameter_services=False,
        )
        self.declare_parameter("simulation_rate_hz", 20.0)
        self.declare_parameter("body_frame", "base_link")
        self.declare_parameter("braking_deceleration", -1.0)
        self.declare_parameter("lateral_braking_deceleration", -1.0)
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
        self.declare_parameter("final_command_topic", "/safety/final_command")
        self.declare_parameter("robot_id", "robot")
        self.declare_parameter("evidence_topic", "/motion/evidence")
        self.declare_parameter("measurement_max_age", -1.0)
        self.declare_parameter("motion_min_quality", -1.0)
        self.declare_parameter("motion_max_variance", -1.0)
        self.declare_parameter("motion_max_interval", -1.0)
        self.declare_parameter("max_reference_lag", -1.0)
        self.declare_parameter("max_linear_speed", -1.0)
        self.declare_parameter("max_yaw_rate", -1.0)
        self.declare_parameter("physical_cap_context", "")
        self.declare_parameter("bounded_scan_topic", "/safety/bounded_scan")
        self.declare_parameter("raw_scan_topic", "/safety/raw/scan")
        self.declare_parameter("bounded_depth_topic", "/safety/bounded_depth")
        self.declare_parameter("raw_depth_topic", "/safety/raw/depth")
        self.declare_parameter("raw_depth_info_topic", "/safety/raw/depth_info")
        self.declare_parameter("depth_policy", "required")
        self.declare_parameter("depth_collision_min_z", 1e10)
        self.declare_parameter("depth_collision_max_z", -1e10)
        self.declare_parameter("output_topic", "/cmd_vel")
        self.declare_parameter("command_ttl_seconds", -1.0)
        self._body = str(self.get_parameter("body_frame").value)
        rate = float(self.get_parameter("simulation_rate_hz").value)
        if not self._body or not math.isfinite(rate) or rate <= 0.0:
            raise ValueError("simulation_rate_hz must be positive and body_frame nonempty")
        self._command = Velocity(0.0, 0.0)
        self._command_monotonic: float | None = None
        self._command_deadline = float("-inf")
        self._final_fence = FinalCommandFence()
        self._final_identity = None
        self._feedback = OutputFeedback()
        self._feedback_boot = ""
        self._timed_command = False
        self._guard_socket = None
        self._guard_process = None
        self._guard_seen = float("-inf")
        self._guard_ready = False
        self._measured: BodyTwist | None = None
        self._velocity_interval: IntervalVelocityBounds | None = None
        self._scans: deque[RawSafetyObservation] = deque(maxlen=8)
        self._scan_fence = SourceEpochFence(max_retired=256)
        self._scan_sequence = 0
        self._scan_source = ""
        self._scan_masked_count = 0
        self._scan_ambiguous_count = 0
        self._last_ros: float | None = None
        self._last_ros_advance_steady: float | None = None
        self._epoch_fence = SourceEpochFence(max_retired=256)
        self._motion_frontier = SourceStampFrontier()
        self._evidence_topology = ProducerTopology("motion_fusion", "/")
        self._command_topology = ProducerTopology("command_node", "/")
        self._scan_topology = ProducerTopology("scan_receipt", "/")
        self._depth_topology = ProducerTopology("raw_receipt", "/")
        self._command_generation = 0
        self._scan_generation = 0
        self._motion_generation = 0
        # Configuration is startup-only; cap geometry before any slicing or geometry work.
        self._profile()
        self.declare_parameter("simulation_eager_safety", False)
        self._safety_wakeup = (
            self.create_guard_condition(self._on_timer)
            if self.get_parameter("use_sim_time").value
            and self.get_parameter("simulation_eager_safety").value
            else None
        )
        self._worker = BoundedSafetyWorker(on_result=self._on_worker_result)
        if self._safety_wakeup is not None:
            self._worker.warmup()
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
        self._depth_policy = str(self.get_parameter("depth_policy").value)
        if self._depth_policy not in {"required", "laser_only"}:
            raise ValueError("depth policy must be required or laser_only")
        self._depth_admission = (
            DepthAdmission(max_age_s=scan_age, future_wait_s=min(0.02, scan_age))
            if scan_age
            else None
        )
        self._depth_messages = {}
        self._scan_egress = (
            EgressAdmission(max_age_s=scan_age, hard_limit_s=5.0)
            if scan_age and scan_age <= 5.0
            else None
        )
        self._scan_raw_admission = (
            RawCaptureAdmission(max_age_s=scan_age, hard_limit_s=5.0)
            if scan_age and scan_age <= 5.0
            else None
        )
        self._scan_relay_gid = ""
        self._depth_egress = EgressAdmission(max_age_s=min(scan_age, 1.0)) if scan_age else None
        self._depth_relay_gid = ""
        output = str(self.get_parameter("output_topic").value)
        command_topic = str(self.get_parameter("command_topic").value)
        self._command_topic = command_topic
        self._bounded_scan_topic = str(self.get_parameter("bounded_scan_topic").value)
        self._raw_scan_topic = str(self.get_parameter("raw_scan_topic").value)
        self._bounded_depth_topic = str(self.get_parameter("bounded_depth_topic").value)
        self._raw_depth_topic = str(self.get_parameter("raw_depth_topic").value)
        self._raw_depth_info_topic = str(self.get_parameter("raw_depth_info_topic").value)
        self._publisher = self.create_publisher(Twist, output, 10)
        self._drive_feedback_publisher = self.create_publisher(
            DriveFeedbackMessage, "/safety/drive_feedback", qos_profile_sensor_data
        )
        self._status_publisher = self.create_publisher(DiagnosticArray, "/safety/status", 10)
        self._status_counts: dict[str, int] = {}
        self._last_status = float("-inf")
        self._status_geometry: dict[str, object] = {}
        self._command_topic = str(self.get_parameter("final_command_topic").value)
        self._start_command_guard()
        evidence_topic = str(self.get_parameter("evidence_topic").value)
        self._evidence_topic = evidence_topic
        if evidence_topic:
            self.create_subscription(MotionEvidence, evidence_topic, self._on_evidence, 10)
        self.create_subscription(
            FencedSafetyScan, self._bounded_scan_topic, self._on_scan, qos_profile_sensor_data
        )
        if self._depth_policy == "required":
            self.create_subscription(
                FencedDepthEvidence,
                self._bounded_depth_topic,
                self._on_depth,
                qos_profile_sensor_data,
            )
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
        if len(footprint_xy) > 128 or len(mask_xy) > 128:
            raise ValueError("footprint or self-mask exceeds safety geometry complexity limit")
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
            lateral_braking_deceleration=_positive(
                float(self.get_parameter("lateral_braking_deceleration").value)
            ),
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
            self._velocity_interval = None
            self._scan_fence.retire_active()
            self._scan_sequence = 0
            self._scan_source = ""
            self._scans.clear()
            if self._depth_admission is not None:
                self._depth_admission.retire()
            self._depth_messages.clear()
            self._approved = None
            if self._freshness is not None:
                self._freshness.scan_received = self._freshness.measured_received = None
            self._last_ros = now if math.isfinite(now) else None
            self._last_ros_advance_steady = time.monotonic() if math.isfinite(now) else None
            return False
        if self._last_ros is None or now > self._last_ros:
            self._last_ros_advance_steady = time.monotonic()
        self._last_ros = now
        return True

    def _on_command(self, msg, gid: bytes) -> None:
        now = time.monotonic()
        velocity = Velocity(float(msg.vx), float(msg.wz))
        if (
            msg.robot_id != str(self.get_parameter("robot_id").value)
            or (
                velocity != Velocity(0.0, 0.0) and (not msg.session_id or msg.connection_epoch == 0)
            )
            or not self._final_fence.accept(
                gid,
                msg.owner_boot_id,
                msg.generation,
                msg.sequence,
                msg.steady_deadline,
                velocity,
                now,
                session=msg.session_id,
                epoch=msg.connection_epoch,
            )
        ):
            return
        identity = (
            msg.owner_boot_id,
            msg.generation,
            msg.session_id,
            msg.connection_epoch,
            velocity,
        )
        # A fresh sample of the same continuous request keeps existing geometry qualified.
        if identity != self._final_identity or self._command_deadline <= now < msg.steady_deadline:
            self._command_generation += 1
            self._final_identity = identity
        self._command = velocity
        # //Compatible: isolated older callers lack timing correlation fields.
        self._timed_command = bool(getattr(msg, "gesture", 0))
        key = None
        if self._timed_command:
            try:
                key = DriveFeedbackKey(
                    robot_id=msg.robot_id,
                    session_id=msg.session_id,
                    connection_epoch=msg.connection_epoch,
                    request_id=msg.request_id,
                    seq=msg.sequence,
                    gesture=msg.gesture,
                )
            except ValueError:
                pass
        self._replace_drive_feedback(key, msg.owner_boot_id, velocity, msg.steady_deadline)
        self._command_deadline = msg.steady_deadline
        self._command_monotonic = now

    def _command_topology_ready(self) -> bool:
        return (
            self._guard_ready
            and self._guard_process is not None
            and self._guard_process.poll() is None
            and time.monotonic() - self._guard_seen < 0.2
        )

    def _start_command_guard(self) -> None:
        try:
            executable = (
                Path(get_package_prefix("senior_motion_msgs"))
                / "lib/senior_motion_msgs/final_command_guard"
            )
            parent, child = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
            try:
                process = subprocess.Popen(
                    [
                        str(executable),
                        str(child.fileno()),
                        str(self.get_parameter("robot_id").value),
                        self._command_topic,
                    ],
                    pass_fds=(child.fileno(),),
                    close_fds=True,
                )
            except BaseException:
                parent.close()
                raise
            finally:
                child.close()
            parent.setblocking(False)
            self._guard_socket, self._guard_process = parent, process
        except (OSError, LookupError):
            self.get_logger().error("final command guard unavailable; motion disabled")

    def _poll_command_guard(self) -> None:
        channel = self._guard_socket
        if channel is None:
            return
        layout = struct.Struct("!B24s129s129s129s129sQQQQddd")
        try:
            for _ in range(64):
                try:
                    packet = channel.recv(layout.size + 1)
                except BlockingIOError:
                    return
                if len(packet) != layout.size:
                    raise ValueError("invalid guard frame")
                (
                    kind,
                    gid,
                    robot,
                    boot,
                    session,
                    request,
                    generation,
                    epoch,
                    sequence,
                    gesture,
                    deadline,
                    vx,
                    wz,
                ) = layout.unpack(packet)
                if kind not in (0, 1, 2):
                    raise ValueError("invalid guard frame type")
                publisher = gid if any(gid) else None
                self._final_fence.set_publisher(publisher)
                self._guard_seen = time.monotonic()
                if kind == 0:
                    self._guard_ready = False
                    self._command_generation += 1
                    self._command = Velocity(0.0, 0.0)
                    self._command_monotonic = None
                    self._command_deadline = float("-inf")
                    self._final_identity = None
                    self._timed_command = False
                    self._replace_drive_feedback(None, "", self._command, self._command_deadline)
                elif kind == 2:
                    self._guard_ready = publisher is not None
                else:

                    def decode(value):
                        return value.split(b"\0", 1)[0].decode("utf-8", errors="strict")

                    msg = SimpleNamespace(
                        robot_id=decode(robot),
                        owner_boot_id=decode(boot),
                        session_id=decode(session),
                        request_id=decode(request),
                        gesture=gesture,
                        generation=generation,
                        connection_epoch=epoch,
                        sequence=sequence,
                        steady_deadline=deadline,
                        vx=vx,
                        wz=wz,
                    )
                    self._on_command(msg, gid)
            raise ValueError("guard queue overflow")
        except (OSError, ValueError, UnicodeError):
            self._guard_ready = False
            self._final_fence.set_publisher(None)
            self._command = Velocity(0.0, 0.0)
            self._command_monotonic = None
            self._command_deadline = float("-inf")
            channel.close()
            self._guard_socket = None
            if self._guard_process is not None:
                self._guard_process.terminate()

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
            self._velocity_interval = None
            if self._freshness is not None:
                self._freshness.measured_received = None
        return ready

    def _on_evidence(self, msg: MotionEvidence) -> None:
        if self._topology_ready():
            self._accept_evidence(msg)
            if (
                self._safety_wakeup is not None
                and not self._worker.pending
                and self._command != Velocity(0.0, 0.0)
            ):
                self._safety_wakeup.trigger()

    def _on_worker_result(self, reply: WorkerReply) -> None:
        if (
            self._safety_wakeup is not None
            and isinstance(reply.value, SafetyWorkResult)
            and reply.value.decision.allowed
        ):
            self._safety_wakeup.trigger()

    def _accept_evidence(self, msg: MotionEvidence) -> None:
        previous_epoch = self._epoch_fence.active
        if msg.source_epoch:
            if not self._epoch_fence.accept(msg.source_epoch):
                # Retired traffic cannot revoke or resurrect the current epoch.
                # Exhaustion still fails closed because it has no active epoch.
                if self._epoch_fence.active is None:
                    self._history = self._new_history()
                    self._reference_admission = None
                    self._velocity_interval = None
                    if self._freshness is not None:
                        self._freshness.measured_received = None
                return
            if previous_epoch != msg.source_epoch:
                self._motion_frontier.reset()
            if previous_epoch is not None and previous_epoch != msg.source_epoch:
                self._history = self._new_history()
                self._reference_admission = None
                self._velocity_interval = None
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
            source_received = float(msg.scan_received_steady)
            source_received_ros_ns = Time.from_msg(msg.scan_received_ros).nanoseconds
            dt = stamp - start
            valid = (
                self._clock_ok(now)
                and self._freshness is not None
                and self._history is not None
                and math.isfinite(source_received)
                and 0 < source_received <= received
                and 0 < source_received_ros_ns <= now_ns
                and msg.scan_received_ros.nanosec < 1_000_000_000
                and 0 <= received - source_received <= self._freshness.max_age
                and 0 <= (now_ns - source_received_ros_ns) / 1e9 <= self._freshness.max_age
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
            measured = None
            interval = None
            if valid:
                delta_twist = body_twist_delta(
                    Pose2D(msg.body_delta.x, msg.body_delta.y, msg.body_delta.theta)
                )
                measured = BodyTwist(delta_twist.x / dt, delta_twist.y / dt, delta_twist.yaw / dt)
                actual = (measured.vx, measured.vy, measured.wz)
                advertised = (
                    msg.body_velocity.linear.x,
                    msg.body_velocity.linear.y,
                    msg.body_velocity.angular.z,
                )
                valid = all(
                    math.isfinite(given)
                    and math.isclose(given, recomputed, rel_tol=1e-5, abs_tol=1e-6)
                    for given, recomputed in zip(advertised, actual, strict=True)
                )
                if valid:
                    try:
                        interval = interval_velocity_bounds(
                            Pose2D(msg.body_delta.x, msg.body_delta.y, msg.body_delta.theta),
                            tuple(float(value) for value in msg.covariance),
                            dt,
                        )
                    except ValueError:
                        valid = False
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
            ) or not self._freshness.measured(stamp, source_received, now):
                raise ValueError("stale motion evidence")
            if self._last_ros_advance_steady is None:
                raise ValueError("clock progress unavailable")
            self._reference_admission = (
                stamp,
                now,
                min(source_received, self._last_ros_advance_steady),
            )
            assert measured is not None and interval is not None
            self._measured = measured
            self._velocity_interval = interval
        except (ValueError, OverflowError):
            self._history = self._new_history()
            self._reference_admission = None
            self._velocity_interval = None
            if self._freshness is not None:
                self._freshness.measured_received = None

    def _clear_scan(self) -> None:
        self._scans.clear()
        self._scan_masked_count = self._scan_ambiguous_count = 0
        if self._freshness is not None:
            self._freshness.scan_received = None

    def _scan_topology_ready(self) -> bool:
        endpoints = self.get_publishers_info_by_topic(self._bounded_scan_topic)
        ready, changed = self._scan_topology.observe(
            [(item.node_name, item.node_namespace, bytes(item.endpoint_gid)) for item in endpoints]
        )
        self._scan_relay_gid = bytes(endpoints[0].endpoint_gid).hex() if len(endpoints) == 1 else ""
        if self._scan_egress is None:
            ready = False
        else:
            ready = self._scan_egress.observe_relay(self._scan_relay_gid) and ready
        if changed or not ready:
            if changed:
                self._scan_generation += 1
            self._scan_fence.retire_active()
            self._scan_sequence = 0
            self._scan_source = ""
            self._clear_scan()
        return ready

    def _on_scan(self, envelope: FencedSafetyScan) -> None:
        if (
            not self._scan_topology_ready()
            or self._freshness is None
            or self._scan_egress is None
            or self._scan_raw_admission is None
        ):
            return
        try:
            owner = egress_from_message(envelope.owner)
            previous = self._scan_egress.generation
            if not self._scan_egress.offer(owner, relay_gid=self._scan_relay_gid):
                if self._scan_egress.generation != previous:
                    if owner.owner_lost:
                        self._scan_fence.retire_active()
                    self._scan_generation += 1
                    self._clear_scan()
                    self._approved = None
                return
            msg = envelope.body
            ros_ns, steady = self.get_clock().now().nanoseconds, time.monotonic()
            endpoints = self.get_publishers_info_by_topic(self._raw_scan_topic)
            bridge_gid = bytes(endpoints[0].endpoint_gid).hex() if len(endpoints) == 1 else ""
            capture = safety_scan_capture(
                msg,
                bridge_gid=bridge_gid,
                now_ros_ns=ros_ns,
                now_steady=steady,
                max_age_s=self._freshness.max_age,
            )
            if (
                owner.owner_epoch != msg.source_epoch
                or owner.owner_sequence != msg.sequence
                or capture is None
            ):
                self._scan_generation += 1
                self._clear_scan()
                self._approved = None
                return
            before = self._scan_raw_admission.generation
            if not self._scan_raw_admission.offer(
                capture, bridge_gid=bridge_gid, now_ros_ns=ros_ns, now_steady=steady
            ):
                if self._scan_raw_admission.generation != before:
                    self._scan_generation += 1
                    self._clear_scan()
                    self._approved = None
                return
        except (ValueError, OverflowError, TypeError, AttributeError):
            self._scan_generation += 1
            self._clear_scan()
            self._approved = None
            return
        if not msg.source_epoch:
            self._scan_generation += 1
            self._clear_scan()
            return
        previous_epoch = self._scan_fence.active
        if not self._scan_fence.accept(msg.source_epoch):
            # A retired source cannot revoke its successor. Exhaustion has no
            # active source, so it must revoke every outstanding scan ticket.
            if self._scan_fence.active is None:
                self._scan_generation += 1
                self._clear_scan()
            return
        if previous_epoch != msg.source_epoch:
            self._scan_sequence = 0
            self._scan_source = msg.raw_source
            self._clear_scan()
        if msg.sequence <= self._scan_sequence:
            return
        self._scan_sequence = msg.sequence
        self._scan_generation += 1
        try:
            now = self.get_clock().now().nanoseconds / 1e9
            steady = time.monotonic()
            stamp = Time.from_msg(msg.stamp).nanoseconds / 1e9
            if (
                not self._clock_ok(now)
                or not msg.valid
                or not msg.raw_source
                or msg.raw_source != self._scan_source
                or not msg.frame_id
                or not 1 <= len(msg.ranges) <= 2048
                or not all(math.isfinite(v) for v in (msg.ingress_steady, msg.ingress_ros))
                or not 0 <= steady - msg.ingress_steady <= self._freshness.max_age
                or not 0 <= now - msg.ingress_ros <= self._freshness.max_age
                or not 0 <= msg.ingress_ros - stamp <= self._freshness.max_age
                or self._last_ros_advance_steady is None
                or not self._freshness.scan(stamp, msg.ingress_steady, now)
            ):
                raise ValueError("bounded scan unqualified")
            self._scans.append(
                RawSafetyObservation(
                    msg,
                    msg.ingress_steady,
                    msg.ingress_ros,
                    min(msg.ingress_steady, self._last_ros_advance_steady),
                    stamp,
                )
            )
        except (ValueError, OverflowError):
            self._clear_scan()

    def _depth_topology_ready(self) -> bool:
        endpoints = self.get_publishers_info_by_topic(self._bounded_depth_topic)
        ready, changed = self._depth_topology.observe(
            [(item.node_name, item.node_namespace, bytes(item.endpoint_gid)) for item in endpoints]
        )
        self._depth_relay_gid = (
            bytes(endpoints[0].endpoint_gid).hex() if len(endpoints) == 1 else ""
        )
        if self._depth_egress is not None:
            ready = self._depth_egress.observe_relay(self._depth_relay_gid) and ready
        if changed or not ready:
            if self._depth_admission is not None:
                self._depth_admission.retire()
            self._depth_messages.clear()
            self._approved = None
        return ready

    def _depth_shape_valid(self, msg: BoundedDepthEvidence, ros_ns: int, steady: float) -> bool:
        if self._depth_admission is None:
            return False
        try:
            stamp = Time.from_msg(msg.stamp).nanoseconds
            info_stamp = Time.from_msg(msg.camera_info_stamp).nanoseconds
            info_receipt = Time.from_msg(msg.camera_info_received_ros).nanoseconds
            transform_stamp = Time.from_msg(msg.transform_stamp).nanoseconds
            age = self._depth_admission.max_age_s
            scalars = (
                msg.fx,
                msg.fy,
                msg.cx,
                msg.cy,
                msg.min_range_m,
                msg.max_range_m,
                msg.range_error_m,
                msg.camera_info_received_steady,
                *msg.sensor_origin,
                *msg.optical_to_body_quaternion,
            )
            return (
                msg.valid
                and msg.body_frame == self._body
                and bool(msg.frame_id)
                and bool(msg.raw_source)
                and bool(msg.camera_info_source)
                and len(msg.calibration_id) == 64
                and all(ch in "0123456789abcdef" for ch in msg.calibration_id)
                and 1 <= msg.width <= 4096
                and 1 <= msg.height <= 4096
                and msg.width * msg.height <= 2_073_600
                and msg.row_stride > 0
                and msg.column_stride > 0
                and 1 <= len(msg.samples) <= 8192
                and all(math.isfinite(value) for value in scalars)
                and msg.fx > 0
                and msg.fy > 0
                and 0 <= msg.cx <= msg.width
                and 0 <= msg.cy <= msg.height
                and 0 < msg.min_range_m < msg.max_range_m <= 100
                and msg.range_error_m > 0
                and abs(sum(v * v for v in msg.optical_to_body_quaternion) - 1.0) <= 0.001
                and stamp > 0
                and info_stamp == stamp
                and 0 <= ros_ns - info_receipt <= age * 1e9
                and 0 <= steady - msg.camera_info_received_steady <= age
                and (msg.transform_static or 0 <= stamp - transform_stamp <= age * 1e9)
            )
        except (ValueError, OverflowError, TypeError):
            return False

    def _prune_depth_messages(self) -> None:
        admission = self._depth_admission
        if admission is None:
            self._depth_messages.clear()
            return
        headers = list(admission.pending_headers)
        if admission.latest is not None:
            headers.append(admission.latest)
        keep = {(header.source_epoch, header.sequence) for header in headers}
        self._depth_messages = {
            key: value for key, value in self._depth_messages.items() if key in keep
        }

    def _on_depth(self, envelope: FencedDepthEvidence) -> None:
        admission = self._depth_admission
        egress = self._depth_egress
        if admission is None or egress is None or not self._depth_topology_ready():
            return
        ros_ns = self.get_clock().now().nanoseconds
        steady = time.monotonic()
        try:
            owner = egress_from_message(envelope.owner)
            prior = egress.generation
            if not egress.offer(owner, relay_gid=self._depth_relay_gid):
                if egress.generation != prior:
                    if owner.owner_lost:
                        admission.retire()
                    else:
                        body = envelope.body
                        admission.accept(
                            DepthHeader(
                                owner.owner_epoch,
                                body.raw_source,
                                owner.owner_sequence,
                                Time.from_msg(body.stamp).nanoseconds,
                                Time.from_msg(body.received_ros).nanoseconds,
                                body.received_steady,
                                body.frame_id,
                                body.calibration_id,
                                False,
                            ),
                            now_ros_ns=ros_ns,
                            now_steady=steady,
                        )
                    self._depth_messages.clear()
                    self._approved = None
                    self._publish(Velocity(0.0, 0.0), "depth_observation_invalid")
                return
            msg = envelope.body

            def bridge_gid(topic: str) -> str:
                endpoints = self.get_publishers_info_by_topic(topic)
                return bytes(endpoints[0].endpoint_gid).hex() if len(endpoints) == 1 else ""

            captures = depth_capture_headers(
                msg,
                depth_gid=bridge_gid(self._raw_depth_topic),
                info_gid=bridge_gid(self._raw_depth_info_topic),
                now_ros_ns=ros_ns,
                now_steady=steady,
                max_age_s=admission.max_age_s,
            )
            valid = (
                owner.owner_epoch == msg.source_epoch
                and owner.owner_sequence == msg.sequence
                and captures is not None
                and self._depth_shape_valid(msg, ros_ns, steady)
            )
            header = DepthHeader(
                msg.source_epoch,
                msg.raw_source,
                int(msg.sequence),
                Time.from_msg(msg.stamp).nanoseconds,
                Time.from_msg(msg.received_ros).nanoseconds,
                msg.received_steady,
                msg.frame_id,
                msg.calibration_id,
                valid,
            )
            generation = admission.generation
            admission.accept(header, now_ros_ns=ros_ns, now_steady=steady)
            if admission.generation != generation:
                if valid:
                    self._depth_messages[(header.source_epoch, header.sequence)] = msg
                self._prune_depth_messages()
                self._approved = None
                self._publish(Velocity(0.0, 0.0), "depth_observation_changed")
        except (ValueError, OverflowError, TypeError):
            admission.retire()
            self._depth_messages.clear()
            self._approved = None
            self._publish(Velocity(0.0, 0.0), "depth_observation_invalid")

    def _depth_usable(self, ros_ns: int, steady: float) -> BoundedDepthEvidence | None:
        admission = self._depth_admission
        if admission is None or not self._depth_topology_ready():
            return None
        admission.release(now_ros_ns=ros_ns, now_steady=steady)
        self._prune_depth_messages()
        if not admission.usable(now_ros_ns=ros_ns, now_steady=steady):
            return None
        header = admission.latest
        assert header is not None
        msg = self._depth_messages.get((header.source_epoch, header.sequence))
        return msg if msg is not None and self._depth_shape_valid(msg, ros_ns, steady) else None

    def _depth_still_usable(self, generation: int, ros_ns: int, steady: float) -> bool:
        if self._depth_policy == "laser_only":
            return True
        return (
            self._depth_usable(ros_ns, steady) is not None
            and self._depth_admission is not None
            and self._depth_admission.generation == generation
        )

    def _publish(
        self, velocity: Velocity, reason: str, *, ticket: SafetyTicket | None = None
    ) -> None:
        feedback_key = self._feedback.current
        self._poll_command_guard()
        if velocity != Velocity(0.0, 0.0) and (
            ticket is None
            or ticket.command_generation != self._command_generation
            or ticket.command != self._command
            or ticket.command != velocity
            or not self._command_topology_ready()
            or not self._final_fence.usable(time.monotonic())
            or self.get_clock().now().nanoseconds / 1e9 > ticket.ros_deadline
            or time.monotonic() >= ticket.steady_deadline
        ):
            velocity = Velocity(0.0, 0.0)
            reason = "command_fence_expired"
        twist = Twist()
        twist.linear.x = velocity.vx
        twist.angular.z = velocity.wz
        output_at = time.monotonic()
        self._publisher.publish(twist)
        if self._feedback.current == feedback_key:
            self._publish_drive_feedback(self._feedback.published(velocity, output_at, reason))
        self._report(reason)

    def _replace_drive_feedback(
        self, key: DriveFeedbackKey | None, boot: str, velocity: Velocity, deadline: float
    ) -> None:
        self._publish_drive_feedback(self._feedback.note(key, velocity, deadline))
        self._feedback_boot = boot

    def _publish_drive_feedback(self, feedback: DriveExecutionFeedback | None) -> None:
        if feedback is None:
            return
        msg = DriveFeedbackMessage()
        msg.owner_boot_id = self._feedback_boot
        msg.robot_id, msg.session_id = feedback.robot_id, feedback.session_id
        msg.connection_epoch, msg.sequence = feedback.connection_epoch, feedback.seq
        msg.request_id, msg.gesture = feedback.request_id, feedback.gesture
        msg.status, msg.reason = feedback.status, feedback.reason or ""
        msg.vx, msg.wz = feedback.vx or 0.0, feedback.wz or 0.0
        msg.limited = feedback.limited is True
        self._drive_feedback_publisher.publish(msg)

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

        evidence_ready = freshness is not None and all(
            value is not None and 0 <= reference - value <= freshness.max_age
            for value, reference in (
                (freshness.scan_stamp, now),
                (freshness.scan_received, steady),
                (freshness.measured_stamp, now),
                (freshness.measured_received, steady),
            )
        )
        values = {
            "evidence_ready": "true" if evidence_ready else "false",
            "request_vx": str(self._command.vx),
            "request_wz": str(self._command.wz),
            "measured_vx": str(self._measured.vx if self._measured else "unavailable"),
            "measured_vy": str(self._measured.vy if self._measured else "unavailable"),
            "measured_wz": str(self._measured.wz if self._measured else "unavailable"),
            "command_steady_age": age(self._command_monotonic, steady),
            "scan_ros_age": age(freshness.scan_stamp if freshness else None, now),
            "scan_steady_age": age(freshness.scan_received if freshness else None, steady),
            "motion_ros_age": age(freshness.measured_stamp if freshness else None, now),
            "motion_steady_age": age(freshness.measured_received if freshness else None, steady),
            "reference_ros_age": age(latest.stamp if latest else None, now),
            "buffered_scans": str(len(self._scans)),
            "self_masked_rays": str(self._scan_masked_count),
            "ambiguous_self_overlap_rays": str(self._scan_ambiguous_count),
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
        self._poll_command_guard()
        command_ready = self._command_topology_ready()
        if not self._topology_ready():
            self._publish(Velocity(0.0, 0.0), "evidence_topology_unqualified")
            return
        if not command_ready:
            self._publish(Velocity(0.0, 0.0), "command_topology_unqualified")
            return
        if not self._scan_topology_ready():
            self._publish(Velocity(0.0, 0.0), "scan_topology_unqualified")
            return
        ttl = _positive(float(self.get_parameter("command_ttl_seconds").value))
        if (
            # //Compatible: untimed commands retain the receipt-based secondary watchdog.
            ttl is None
            or (
                not self._timed_command
                and command_expired(self._command_monotonic, time.monotonic(), ttl)
            )
            or time.monotonic() >= self._command_deadline
        ):
            self._approved = None
            self._publish(Velocity(0.0, 0.0), "command_expired")
            return
        if self._command == Velocity(0.0, 0.0):
            self._approved = None
            self._publish(Velocity(0.0, 0.0), "zero_requested")
            return
        time_reference = time.monotonic()
        now_ros_ns = self.get_clock().now().nanoseconds
        now = now_ros_ns / 1e9
        if (
            not self._clock_ok(now)
            or self._freshness is None
            or not self._freshness.usable(now, time.monotonic())
        ):
            self._approved = None
            self._publish(Velocity(0.0, 0.0), "sensor_freshness_unqualified")
            return
        depth_message = None
        depth_generation = 0
        if self._depth_policy == "required":
            depth_message = self._depth_usable(now_ros_ns, time_reference)
            if depth_message is None or self._depth_admission is None:
                self._approved = None
                self._publish(Velocity(0.0, 0.0), "depth_freshness_unqualified")
                return
            depth_generation = self._depth_admission.generation
        try:
            profile = self._profile()
            speed = _positive(float(self.get_parameter("max_linear_speed").value))
            yaw_rate = _positive(float(self.get_parameter("max_yaw_rate").value))
            max_lag = _positive(float(self.get_parameter("max_reference_lag").value))
            cap_context = str(self.get_parameter("physical_cap_context").value)
            # The test stand is physically fixed, so the same conservative cap
            # family is valid only while its fixture contract is maintained.
            if cap_context not in {PHYSICAL_CAP_CONTEXT, "fortress-fixed-stand-v1"}:
                raise ValueError("physical_motion_cap_unqualified")
            if profile.lateral_braking_deceleration is None:
                raise ValueError("lateral_braking_unqualified")
            if (
                self._measured is None
                or self._velocity_interval is None
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
                or not self._scan_fence.active
                or not self._scan_source
                or self._history.latest is None
                or not self._epoch_fence.active
                or self._history.latest.epoch != self._epoch_fence.active
                or self._reference_admission is None
                or self._reference_admission[0] != self._history.latest.stamp
                or self._freshness.scan_received is None
                or self._freshness.measured_received is None
                or abs(self._command.vx) > speed
                or abs(self._command.wz) > yaw_rate
                or abs(self._measured.wz) > yaw_rate
            ):
                raise ValueError("unqualified temporal profile")
            if math.hypot(self._measured.vx, self._measured.vy) > speed:
                raise ValueError("measured_translational_speed_exceeded")
            # This named context has only the documented flat-scene limits.
            # A different cap cannot inherit its qualification by name alone.
            if (
                speed != 0.3
                or yaw_rate != 0.5
                or profile.braking_deceleration > 0.3
                or profile.lateral_braking_deceleration > 0.3
                or profile.yaw_braking_deceleration is None
                or profile.yaw_braking_deceleration > 0.5
                or profile.reaction_time < 0.2
                or profile.hard_margin < 0.02
                or profile.footprint
                != [(-0.337, -0.276), (0.337, -0.276), (0.337, 0.276), (-0.337, 0.276)]
            ):
                raise ValueError("physical_motion_profile_unqualified")
            radius = (
                max(math.hypot(x, y) for x, y in profile.footprint)
                + speed * profile.reaction_time
                + speed * speed / (2 * profile.braking_deceleration)
                + profile.hard_margin
            )
            if profile.lateral_braking_deceleration is not None:
                radius += speed * profile.reaction_time + speed * speed / (
                    2 * profile.lateral_braking_deceleration
                )
            configuration = (
                profile.model_dump_json(),
                cap_context,
                speed,
                yaw_rate,
                max_lag,
                ttl,
                self.get_parameter("interpolate").value,
                self.get_parameter("max_angular_gap").value,
                self.get_parameter("transform_static").value,
                self.get_parameter("self_mask_range_uncertainty").value,
                self._depth_policy,
                depth_generation,
                self.get_parameter("depth_collision_min_z").value,
                self.get_parameter("depth_collision_max_z").value,
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
                current_ros_ns = self.get_clock().now().nanoseconds
                current_ros = current_ros_ns / 1e9
                current_steady = time.monotonic()
                valid = (
                    self._command_topology_ready()
                    and self._topology_ready()
                    and self._scan_topology_ready()
                    and self._depth_still_usable(depth_generation, current_ros_ns, current_steady)
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
                    self._scan_masked_count = result.self_masked_rays
                    self._scan_ambiguous_count = result.ambiguous_rays
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
                    final_ros_ns = self.get_clock().now().nanoseconds
                    final_ros = final_ros_ns / 1e9
                    final_steady = time.monotonic()
                    if (
                        self._command_topology_ready()
                        and self._topology_ready()
                        and self._scan_topology_ready()
                        and self._depth_still_usable(depth_generation, final_ros_ns, final_steady)
                        and ticket.permits(
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
                        self._publish(
                            result.decision.velocity, result.decision.reason, ticket=ticket
                        )
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
            if depth_message is not None and self._depth_admission is not None:
                depth_age = self._depth_admission.max_age_s
                deadline_ros = min(
                    deadline_ros,
                    Time.from_msg(depth_message.stamp).nanoseconds / 1e9 + depth_age,
                    Time.from_msg(depth_message.received_ros).nanoseconds / 1e9 + depth_age,
                    Time.from_msg(depth_message.camera_info_received_ros).nanoseconds / 1e9
                    + depth_age,
                )
            remaining = deadline_ros - now
            if remaining <= 0:
                raise ValueError("reference window expired")
            fixed_deadline = reference_steady_deadline(*self._reference_admission, max_lag)
            if fixed_deadline is None:
                raise ValueError("reference admission expired")
            for sample in self._scans:
                if (
                    sample.stamp == latest.stamp
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
                self._command_deadline,
                self._freshness.scan_received + self._freshness.max_age,
                self._freshness.measured_received + self._freshness.max_age,
                time_reference + remaining,
                fixed_deadline,
            )
            if depth_message is not None and self._depth_admission is not None:
                depth_age = self._depth_admission.max_age_s
                deadline_steady = min(
                    deadline_steady,
                    depth_message.received_steady + depth_age,
                    depth_message.camera_info_received_steady + depth_age,
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
                bool(self.get_parameter("interpolate").value),
                float(self.get_parameter("max_angular_gap").value),
                bool(self.get_parameter("no_return_clear").value),
                bool(self.get_parameter("transform_static").value),
                float(self.get_parameter("self_mask_range_uncertainty").value),
                self._body,
                self._velocity_interval,
                True,
                depth_required=self._depth_policy == "required",
                depth_message=depth_message,
                depth_collision_z=(
                    float(self.get_parameter("depth_collision_min_z").value),
                    float(self.get_parameter("depth_collision_max_z").value),
                ),
            )
            self._worker.submit(ticket, work)
        except ValueError as exc:
            self._publish(Velocity(0.0, 0.0), str(exc))
            return
        self._publish(Velocity(0.0, 0.0), "evaluation_pending")

    def destroy_node(self):
        self._worker.close()
        if self._guard_socket is not None:
            self._guard_socket.close()
            self._guard_socket = None
        if self._guard_process is not None:
            self._guard_process.terminate()
            try:
                self._guard_process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                self._guard_process.kill()
                self._guard_process.wait(timeout=1.0)
        return super().destroy_node()


def main() -> None:
    run(SafetyGate, cli=True)
