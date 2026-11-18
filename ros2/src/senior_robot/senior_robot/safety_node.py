"""Independent safety process.

Startup requires an explicit generic motion profile file.
The default 20 Hz control frequency is a software setting, not a physical braking
or timing guarantee. The shared motion core owns constraints and continuous braking;
original command deadlines use the monotonic clock and need no stop message.
This process does not arbitrate control, publish heartbeats, or remap Nav2.
It does not read wheel odometry or /ground_truth/odom, and it does not use a
wheel transform to relabel a scan into the body frame.
"""

import math
import socket
import subprocess
import time
from collections import deque
from contextlib import ExitStack
from dataclasses import replace
from pathlib import Path

from ament_index_python.packages import get_package_prefix
from rclpy.clock import Clock as RosClock
from rclpy.clock import ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from senior_motion_msgs.msg import (
    BoundedDepthEvidence,
    FencedDepthEvidence,
    FencedSafetyScan,
)
from senior_motion_msgs.msg import (
    DriveExecutionFeedback as DriveFeedbackMessage,
)

from senior_control.models import Velocity
from senior_control.motion.source_fences import (
    ProducerTopology,
    SourceEpochFence,
)
from senior_control.safety.arbiter import FinalCommandFence
from senior_control.safety.freshness import SafetyFreshness
from senior_control.sensors.policy import default_policy
from senior_control.vision.egress_receipt import (
    EgressAdmission,
    depth_capture_headers,
    egress_from_message,
    safety_scan_capture,
)
from senior_control.vision.raw_capture import RawCaptureAdmission, capture_from_message
from senior_control.vision.runtime_evidence import DepthAdmission, DepthHeader
from senior_robot.motion_lifecycle import MotionExecutor, run
from senior_robot.raw_safety_observation import RawSafetyObservation


class SafetyGate(Node):
    def __init__(self, *, parameter_overrides=None, input_channel=None) -> None:
        self._resources = ExitStack()
        try:
            self._initialize(parameter_overrides=parameter_overrides, input_channel=input_channel)
        except BaseException:
            self._resources.close()
            raise

    def _own(self, resource):
        self._resources.callback(resource.close)
        return resource

    def _initialize(self, *, parameter_overrides, input_channel):
        super().__init__(
            "safety_input" if input_channel is not None else "safety_gate",
            parameter_overrides=parameter_overrides or [],
            start_parameter_services=False,
        )
        self._input_channel = input_channel
        self._input_process = None
        self._input_scan_ready = False
        from senior_control.motion.cadence import ControlCadence

        self.declare_parameter("simulation_rate_hz", 20.0)
        self.declare_parameter("body_frame", "base_link")
        self.declare_parameter("wheel_frame", "odom")
        self.declare_parameter("final_command_topic", "/safety/final_command")
        self.declare_parameter("robot_id", "robot")
        self.declare_parameter("bounded_scan_topic", "/safety/bounded_scan")
        self.declare_parameter("raw_scan_topic", "/safety/raw/scan")
        self.declare_parameter("bounded_depth_topic", "/safety/bounded_depth")
        self.declare_parameter("raw_depth_topic", "/safety/raw/depth")
        self.declare_parameter("raw_depth_info_topic", "/safety/raw/depth_info")
        self.declare_parameter("depth_policy", "required")
        self.declare_parameter("depth_collision_min_z", 1e10)
        self.declare_parameter("depth_collision_max_z", -1e10)
        self.declare_parameter("output_topic", "/cmd_vel")
        self._body = str(self.get_parameter("body_frame").value)
        rate = float(self.get_parameter("simulation_rate_hz").value)
        if not self._body or not math.isfinite(rate) or rate <= 0:
            raise ValueError("control frequency must be finite/positive and body_frame nonempty")
        self._cadence = ControlCadence((1.0 / rate) * 1000)
        self.declare_parameter("hardware_motion_profile", "")
        hardware_path = str(self.get_parameter("hardware_motion_profile").value)
        from senior_robot.motion_profile import select_motion_profile

        candidate_profile = select_motion_profile(hardware_path=hardware_path)
        from senior_robot.motion_profile import configure_startup_learning

        for name in (
            "startup_learning_front_occlusion_stop",
            "startup_learning_rear_occlusion_stop",
        ):
            self.declare_parameter(name, getattr(candidate_profile, name))
        candidate_profile = configure_startup_learning(
            candidate_profile,
            front_stop=self.get_parameter("startup_learning_front_occlusion_stop").value,
            rear_stop=self.get_parameter("startup_learning_rear_occlusion_stop").value,
        )
        self._final_fence = FinalCommandFence()
        self._guard_socket = None
        self._guard_process = None
        self._guard_seen = float("-inf")
        self._guard_ready = False
        self._scans: deque[RawSafetyObservation] = deque(maxlen=1)
        self._scan_compensator = None
        self._scan_fence = SourceEpochFence(max_retired=256)
        self._scan_sequence = 0
        self._scan_source = ""
        self._scan_masked_count = 0
        self._scan_ambiguous_count = 0
        self._scan_topology = ProducerTopology("scan_receipt", "/")
        self._depth_topology = ProducerTopology("raw_receipt", "/")
        self._scan_generation = 0
        hard_age_ms = default_policy().hard_age_ms
        scan_age = hard_age_ms / 1000
        self._freshness = SafetyFreshness(scan_age) if scan_age else None
        self._depth_policy = str(self.get_parameter("depth_policy").value)
        if self._depth_policy not in {"required", "laser_only"}:
            raise ValueError("depth policy must be required or laser_only")
        self._depth_admission = DepthAdmission(max_age_ms=hard_age_ms) if scan_age else None
        self._depth_messages = {}
        self._scan_egress = (
            EgressAdmission(max_age_ms=hard_age_ms, hard_limit_ms=5000)
            if scan_age and scan_age <= 5.0
            else None
        )
        self._scan_raw_admission = (
            RawCaptureAdmission(max_age_ms=hard_age_ms, hard_limit_ms=5000)
            if scan_age and scan_age <= 5.0
            else None
        )
        self._scan_relay_gid = ""
        self._depth_egress = (
            EgressAdmission(max_age_ms=min(hard_age_ms, 1000.0)) if scan_age else None
        )
        self._depth_relay_gid = ""
        output = str(self.get_parameter("output_topic").value)
        self._bounded_scan_topic = str(self.get_parameter("bounded_scan_topic").value)
        self._raw_scan_topic = str(self.get_parameter("raw_scan_topic").value)
        self._bounded_depth_topic = str(self.get_parameter("bounded_depth_topic").value)
        self._raw_depth_topic = str(self.get_parameter("raw_depth_topic").value)
        self._raw_depth_info_topic = str(self.get_parameter("raw_depth_info_topic").value)
        from senior_robot.motion_output import MotionOutput

        self._output_transport = (
            self._own(MotionOutput(self.resolve_topic_name(output)))
            if input_channel is None
            else None
        )
        self._drive_feedback_publisher = (
            self.create_publisher(
                DriveFeedbackMessage, "/safety/drive_feedback", qos_profile_sensor_data
            )
            if input_channel is None
            else None
        )
        self._status_counts: dict[str, int] = {}
        self._last_status = float("-inf")
        self._control_timing: dict[str, float] = {}
        self._last_control_tick: float | None = None
        self._command_topic = str(self.get_parameter("final_command_topic").value)
        if input_channel is not None:
            from tf2_ros import Buffer, TransformListener

            from senior_control.runtime.diagnostics import configure_diagnostics
            from senior_robot.motion_observation_adapter import NativeObservationRosBridge
            from senior_robot.motion_sensor_runtime import MotionSensorRuntime
            from senior_robot.scan_gap_compensation import ScanGapCompensator

            self._scan_compensator = ScanGapCompensator(candidate_profile)
            self._input_sensors = MotionSensorRuntime(
                candidate_profile, configure_diagnostics(candidate_profile.diagnostics)
            )
            from senior_control.motion.diagnostic_pump import DiagnosticPump

            self._input_diagnostics = self._own(DiagnosticPump(self._input_sensors.diagnostics))
            if not self.has_parameter("wheel_frame"):
                self.declare_parameter("wheel_frame", "odom")
            self._tf = Buffer()
            self._tf_listener = TransformListener(self._tf, self)
            self._input_bridge = NativeObservationRosBridge(
                self,
                candidate_profile,
                sample_sink=self._input_sensors.ingest,
                source_invalidated=self._input_sensors.invalidate,
                publish=False,
            )
            self._optional_inputs = None
            if candidate_profile.gps_input is not None or candidate_profile.rgbd_input is not None:
                from senior_robot.optional_motion_inputs import OptionalMotionInputs

                self._optional_inputs = OptionalMotionInputs(
                    self, candidate_profile, self._input_sensors
                )
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
                0.05, self._publish_inputs, clock=RosClock(clock_type=ClockType.STEADY_TIME)
            )
        else:
            from senior_robot.control_inputs import ControlInputs
            from senior_robot.motion_core_output import MotionCoreOutput

            self._start_command_guard()
            self._resources.callback(self._stop_guard)
            self._motion_core = self._own(MotionCoreOutput(self, candidate_profile))
            self._input_process = self._own(ControlInputs(self))
            # Wake frequently, but only absolute 50ms slots can start a tick.
            self.create_timer(
                min(0.005, 1.0 / rate),
                self._on_timer,
                clock=RosClock(clock_type=ClockType.STEADY_TIME),
            )

    def _publish_inputs(self):
        sensors = self._input_sensors
        now = time.monotonic()
        ros_ns = self.get_clock().now().nanoseconds
        ros = ros_ns / 1e9
        sensors.poses = list(self._input_bridge.core.poses)
        try:
            sensors.ingest_poses(sensors.poses)
            reference = sensors.estimate(ros, now, now_ros_ns=ros_ns)
        except ValueError:
            # Optional motion estimation cannot suppress the independent scan snapshot.
            reference = None
            sensors.independent = sensors.tracking = None
        ready = self._scan_topology_ready()
        if self._scans:
            # Odometry may arrive after scan admission; retry only this current raw identity.
            self._scans[-1] = self._scan_compensator.update(
                self._scans[-1], self._input_bridge.core.poses, now=now
            )
        # The decision owner reads current values, not estimator/pose histories.
        # raw_feedback/wheel_linear only need these roots and the local age policy.
        state = {
            "latest": None if sensors.latest is None else replace(sensors.latest, path=None),
            "independent": sensors.independent,
            "raw_wheel": sensors.raw_wheel,
            "raw_imu": sensors.raw_imu,
        }
        if reference is not None:
            # Only the reporting average/mode is consumed; tracking stays with its owner.
            reference = ({key: reference[0][key] for key in ("average", "mode")}, None, None)
        self._input_channel.put(
            (state, tuple(self._scans)[-1:], ready, self._scan_generation, reference)
        )

    def _clock_ok(self, now: float) -> bool:
        # Control clock anomalies are separate from the ROS/TF timestamp label.
        return math.isfinite(now)

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
        accepted = self._motion_core.command(msg)
        if not accepted:
            self._motion_core.revoke("target_handoff_failed")
            return
        if (
            msg.source_kind == "navigation"
            and msg.close_reason
            in {"task_revoke", "task_done", "qualification_lost", "shutdown", "expiry", "fault"}
            and msg.request_id
            and msg.sequence == 0
            and msg.steady_deadline == 0
            and velocity == Velocity(0, 0)
        ):
            # Old source fenced + unique local braking owner installed.
            # This is neither a zero-Twist ACK nor a body-stationary claim.
            receipt = DriveFeedbackMessage()
            receipt.robot_id, receipt.owner_boot_id = msg.robot_id, msg.owner_boot_id
            receipt.session_id, receipt.request_id = msg.session_id, msg.request_id
            epoch = self._motion_core.retirement_epoch(msg)
            if epoch is None:
                return
            receipt.connection_epoch = epoch
            receipt.sequence, receipt.status = msg.generation, "retired"
            receipt.reason = self._motion_core.boot
            self._drive_feedback_publisher.publish(receipt)
        return

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

    def _on_task_terminal(self, msg) -> None:
        if msg.robot_id != str(self.get_parameter("robot_id").value):
            return
        collected = msg.close_reason == "task_settled"
        operation = (
            self._motion_core.collect_terminal if collected else self._motion_core.release_terminal
        )
        if not operation(msg):
            return
        receipt = DriveFeedbackMessage()
        receipt.robot_id, receipt.owner_boot_id = msg.robot_id, msg.owner_boot_id
        receipt.session_id, receipt.request_id = msg.source_id, msg.request_id
        receipt.connection_epoch, receipt.sequence = msg.source_epoch, msg.generation
        receipt.status = "task_collected" if collected else "task_retired"
        receipt.reason = self._motion_core.boot
        self._drive_feedback_publisher.publish(receipt)

    def _poll_command_guard(self) -> None:
        channel = self._guard_socket
        if channel is None:
            return
        from senior_robot.final_command_frame import LAYOUT, decode_frame

        layout = LAYOUT
        try:
            for _ in range(64):
                try:
                    packet = channel.recv(layout.size + 1)
                except BlockingIOError:
                    return
                if len(packet) != layout.size:
                    raise ValueError("invalid guard frame")
                kind, gid, msg = decode_frame(packet)
                publisher = gid if any(gid) else None
                self._final_fence.set_publisher(publisher)
                self._guard_seen = time.monotonic()
                if kind == 0:
                    self._guard_ready = False
                elif kind == 2:
                    self._guard_ready = publisher is not None
                elif kind in (3, 4):
                    self._on_task_terminal(msg)
                else:
                    self._on_command(msg, gid)
            raise ValueError("guard queue overflow")
        except (OSError, ValueError, UnicodeError):
            self._guard_ready = False
            self._final_fence.set_publisher(None)
            channel.close()
            self._guard_socket = None
            if self._guard_process is not None:
                self._guard_process.terminate()

    def _clear_scan(self) -> None:
        self._scans.clear()
        if self._scan_compensator is not None:
            self._scan_compensator.clear()
        self._scan_masked_count = self._scan_ambiguous_count = 0
        if self._freshness is not None:
            self._freshness.scan_received = None

    def _scan_topology_ready(self) -> bool:
        if self._input_channel is None:
            return self._input_scan_ready
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
                max_age_ms=(self._freshness.max_age) * 1000,
            )
            if (
                owner.owner_epoch != msg.source_epoch
                or owner.owner_sequence != msg.sequence
                or capture is None
            ):
                self._scan_generation += 1
                self._clear_scan()
                return
            before = self._scan_raw_admission.generation
            if not self._scan_raw_admission.offer(
                capture, bridge_gid=bridge_gid, now_ros_ns=ros_ns, now_steady=steady
            ):
                if self._scan_raw_admission.generation != before:
                    self._scan_generation += 1
                    self._clear_scan()
                return
        except (ValueError, OverflowError, TypeError, AttributeError):
            self._scan_generation += 1
            self._clear_scan()
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
            steady = time.monotonic()
            stamp = Time.from_msg(msg.stamp).nanoseconds / 1e9
            capture = capture_from_message(msg.raw_capture)
            if (
                not self._clock_ok(steady)
                or not msg.valid
                or not msg.raw_source
                or msg.raw_source != self._scan_source
                or not msg.frame_id
                or not 1 <= len(msg.ranges) <= 2048
                or not all(math.isfinite(v) for v in (msg.ingress_steady, msg.ingress_ros))
                or not 0 <= steady - msg.ingress_steady <= self._freshness.max_age
                or not self._freshness.scan(capture.measurement_steady, msg.ingress_steady, steady)
            ):
                raise ValueError("bounded scan unqualified")
            raw = RawSafetyObservation(
                msg,
                msg.ingress_steady,
                msg.ingress_ros,
                stamp,
                capture.measurement_steady,
            )
            self._scans.append(
                self._scan_compensator.update(raw, self._input_bridge.core.poses, now=steady)
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
        return ready

    def _depth_shape_valid(self, msg: BoundedDepthEvidence, ros_ns: int, steady: float) -> bool:
        if self._depth_admission is None:
            return False
        try:
            stamp = Time.from_msg(msg.stamp).nanoseconds
            info_stamp = Time.from_msg(msg.camera_info_stamp).nanoseconds
            info_receipt = Time.from_msg(msg.camera_info_received_ros).nanoseconds
            transform_stamp = Time.from_msg(msg.transform_stamp).nanoseconds
            age = self._depth_admission.max_age_ms / 1000
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
                and stamp >= 0
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
        headers = []
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
                                measurement_steady=capture_from_message(
                                    body.depth_capture
                                ).measurement_steady,
                            ),
                            now_ros_ns=ros_ns,
                            now_steady=steady,
                        )
                    self._depth_messages.clear()
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
                max_age_ms=admission.max_age_ms,
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
                measurement_steady=capture_from_message(msg.depth_capture).measurement_steady,
            )
            generation = admission.generation
            admission.accept(header, now_ros_ns=ros_ns, now_steady=steady)
            if admission.generation != generation:
                if valid:
                    self._depth_messages[(header.source_epoch, header.sequence)] = msg
                self._prune_depth_messages()
        except (ValueError, OverflowError, TypeError):
            admission.retire()
            self._depth_messages.clear()

    def _depth_usable(self, ros_ns: int, steady: float) -> BoundedDepthEvidence | None:
        admission = self._depth_admission
        if admission is None or not self._depth_topology_ready():
            return None
        self._prune_depth_messages()
        if not admission.usable(now_ros_ns=ros_ns, now_steady=steady):
            return None
        header = admission.latest
        assert header is not None
        msg = self._depth_messages.get((header.source_epoch, header.sequence))
        return msg if msg is not None and self._depth_shape_valid(msg, ros_ns, steady) else None

    def _emit_motion_core(
        self, velocity: Velocity, reason: str, deadline: float = 0.0, *, computed_steady: float
    ) -> float:
        self._output_transport.publish(velocity, deadline, computed_steady=computed_steady)
        submitted = time.monotonic()
        self._report(reason)
        return submitted

    def _report(self, reason: str) -> None:
        """Publish bounded rate/counters; diagnostics never grant motion."""
        if reason not in self._status_counts and len(self._status_counts) >= 32:
            reason = "other"
        self._status_counts[reason] = min(self._status_counts.get(reason, 0) + 1, 2**31 - 1)
        steady = time.monotonic()
        if steady - self._last_status < 0.5:
            return
        self._last_status = steady
        values = {
            "buffered_scans": str(len(self._scans)),
            **{f"count.{key}": str(value) for key, value in self._status_counts.items()},
            # Temporary stage timings describe the last completed control callback.
            **{f"timing.{key}": str(value) for key, value in self._control_timing.items()},
        }
        self._motion_core.report_status(dict(body=self._body, reason=reason, values=values))

    def _on_timer(self) -> None:
        started = False
        timing = {}
        try:
            entered = time.monotonic()
            if not self._cadence.due(entered):
                return
            started = True
            timing["sampled_steady"] = entered
            if self._last_control_tick is not None:
                timing["tick_gap_ms"] = (entered - self._last_control_tick) * 1000
            self._last_control_tick = entered
            self._input_process.update(self, self._motion_core.sensors)
            updated = time.monotonic()
            timing["input_update_ms"] = (updated - entered) * 1000
            self._poll_command_guard()
            if not self._command_topology_ready():
                self._motion_core.revoke("command_topology_unqualified")
            guarded = time.monotonic()
            timing["command_guard_ms"] = (guarded - updated) * 1000
            # Sample control time after command ingress has been drained.
            self._motion_core.tick()
            timing["tick_ms"] = (time.monotonic() - guarded) * 1000
        except ValueError as exc:
            self._motion_core.revoke(str(exc))
            self._emit_motion_core(Velocity(0, 0), str(exc), computed_steady=time.monotonic())
        finally:
            if started:
                self._control_timing = timing
                self._cadence.finished(time.monotonic())

    def _stop_guard(self):
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

    def destroy_node(self):
        try:
            if self._input_channel is None:
                self._motion_core.revoke("shutdown")
                try:
                    self._emit_motion_core(
                        Velocity(0, 0), "shutdown", computed_steady=time.monotonic()
                    )
                except ValueError:
                    pass  # A lost output channel is already withdrawn by the native owner.
        finally:
            try:
                self._resources.close()
            finally:
                super().destroy_node()


def main() -> None:
    run(SafetyGate, cli=True, executor_factory=MotionExecutor)
