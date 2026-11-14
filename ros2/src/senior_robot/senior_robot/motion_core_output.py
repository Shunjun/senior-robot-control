"""Shared motion output composition; safety_gate remains the sole actuator writer.

Deployment profiles supply response evidence; no ground truth enters this module.
"""

import json
import math
import time
from concurrent.futures import ThreadPoolExecutor
from typing import Any
from uuid import uuid4

from senior_control.models import BodyTwist, Pose2D, Velocity
from senior_control.motion.constraint_worker import ConstraintScheduler, ExecutorWorker
from senior_control.motion.constraints import (
    ConstraintConfig,
    ConstraintEngine,
    ConstraintSnapshot,
    Footprint,
    Perception,
    RiskEnvelope,
    prepare_perception,
)
from senior_control.motion.contracts import (
    BodyMotion,
    MotionEvent,
    MotionMode,
    MotionTarget,
    ResponseBounds,
)
from senior_control.motion.contracts import (
    MotionEventKind as K,
)
from senior_control.motion.controller import Feedback, MotionController
from senior_control.motion.source_modes import SourceMode
from senior_control.motion.state_machine import MotionStateMachine, RecoveryObservation
from senior_control.protocol import MotionRuntimeState
from senior_control.safety.self_filter import ScanObservation, filter_self
from senior_robot.final_command_frame import PAYLOAD_FIELDS
from senior_robot.motion_observation_adapter import observation_origin


class MotionCoreOutput:
    def __init__(self, node: Any, profile: Any):
        from std_msgs.msg import String

        self.node, self.profile = node, profile
        self.boot = uuid4().hex
        self.sequence = 0
        self.event_sequence = 0
        self.model_deadline = time.monotonic() + profile.model_seconds
        self.engine = ConstraintEngine(
            ConstraintConfig((1.0, 0.75, 0.5, 0.25), 0.15, 0.05, 256, 0.005)
        )
        self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="directional-refine")
        self.scheduler = ConstraintScheduler(
            self.engine, ExecutorWorker(self.executor), worker_budget_s=0.25
        )
        self.controller = MotionController(
            self.engine, profile.controller(), Velocity(0, 0), Velocity(0, 0)
        )
        self.machine = MotionStateMachine()
        self.target = MotionTarget(Velocity(0, 0), "idle", self.boot, self.model_deadline)
        self.output_generation = 0
        self._input_boot = ""
        self._input_floor = -1
        self._manual_motion_source: str | None = None
        self._reference: dict[str, Any] | None = None
        self._reference_boot = ""
        self._reference_sequence = 0
        self._reference_warning_at = float("-inf")
        self._perception_key: tuple[Any, ...] | None = None
        self._perception: Perception | None = None
        self._barrier_key: tuple[str, int, int] | None = None
        self._barrier_payload: tuple[Any, ...] | None = None
        self._barrier_complete = False
        self._last_fault_condition: tuple[str, str] | None = None
        self._closed = False
        self._hardware = False
        self._hardware_seen = float("-inf")
        self._hardware_gid = None
        self._last_healthy = False
        self._body_epoch_used: str | None = None
        self._wheel_epoch_used: str | None = None
        self._runtime = node.create_publisher(String, "/motion/runtime", 10)
        self._runtime_type = String
        from senior_robot.motion_lifecycle import with_dds_info

        with_dds_info(type(self)._on_reference)
        if profile.simulation_only:
            # //Compatible: the isolated candidate keeps its existing transport.
            self._subscription = node.create_subscription(
                String, "/motion/core_reference", self._on_reference, 10
            )
        else:
            from senior_robot.native_receipt import subscribe

            self._subscription = subscribe(
                node,
                String,
                "/motion/core_reference",
                self._on_reference,
                writer="motion_fusion",
                qos="reliable",
                invalidate=self._invalidate_reference,
            )
            if profile.hardware_estop.mode == "readable":
                from std_msgs.msg import Bool

                self._hardware_subscription = subscribe(
                    node,
                    Bool,
                    profile.hardware_estop.topic,
                    self._on_hardware_estop,
                    writer=profile.hardware_estop.publisher,
                    namespace=profile.hardware_estop.namespace,
                    qos="reliable",
                    invalidate=self._invalidate_hardware_status,
                )

    def _invalidate_reference(self) -> None:
        self._reference = None
        self.controller.invalidate_limited_rearm()
        self._warn_reference("native receipt revoked or unavailable")

    def _warn_reference(self, reason: str) -> None:
        now = time.monotonic()
        if now - self._reference_warning_at >= 2.0:
            self._reference_warning_at = now
            self.node.get_logger().warning(f"motion core reference rejected: {reason}")

    def _invalidate_hardware_status(self) -> None:
        self._hardware_seen = float("-inf")
        self._hardware_gid = None

    def _hardware_status_ready(self, now: float) -> bool:
        if self.profile.simulation_only:
            return True
        capability = self.profile.hardware_estop
        if capability.mode != "readable":
            # No physical state channel selected: keep observation unknown,
            # without turning missing operator confirmation into a motion gate.
            return True
        endpoints = self.node.get_publishers_info_by_topic(capability.topic)
        return (
            now - self._hardware_seen <= capability.max_age_s
            and len(endpoints) == 1
            and endpoints[0].node_name == capability.publisher
            and endpoints[0].node_namespace == capability.namespace
            and bytes(endpoints[0].endpoint_gid) == self._hardware_gid
        )

    def _on_hardware_estop(self, msg: Any, info: Any) -> None:
        from senior_robot.native_receipt import received_steady

        capability = self.profile.hardware_estop
        origin = observation_origin(
            info,
            self.node.get_publishers_info_by_topic(capability.topic),
            capability.publisher,
            allow_topology_only=False,
            expected_namespace=capability.namespace,
        )
        if origin is None:
            return
        now = time.monotonic()
        self._hardware_seen = received_steady(info)
        self._hardware_gid = origin.topology_gid
        active = bool(msg.data)
        if active and not self._hardware:
            self._hardware = True
            self._event(K.HARDWARE_ESTOP, now)
            # Physical release never automatically resumes an old authority.
            self._event(K.SOFTWARE_ESTOP, now)
            self.revoke("hardware_estop", now=now)
        elif not active and self._hardware:
            self._hardware = False
            self._event(K.HARDWARE_RELEASE, now)

    def _event(self, kind: K, now: float, source: str = "", revision: int = 0) -> None:
        self.event_sequence += 1
        self.machine.reduce(
            MotionEvent(kind, source, revision, self.event_sequence), now_steady=now
        )
        if kind in {K.FEEDBACK_LOST, K.EXECUTION_FAULT}:
            self.revoke(kind.value, now=now)

    def _on_reference(self, msg: Any, info: Any) -> None:
        endpoints = self.node.get_publishers_info_by_topic("/motion/core_reference")
        origin = observation_origin(
            info,
            endpoints,
            "motion_fusion",
            allow_topology_only=bool(self.profile.simulation_only and self.profile.unvalidated),
        )
        if origin is None:
            self._warn_reference(
                "writer identity unavailable; discovered="
                + repr([(x.node_namespace, x.node_name) for x in endpoints])
            )
            return
        try:
            if len(msg.data) > 131072:
                raise ValueError("oversize core reference")
            value = json.loads(msg.data)
            if (
                value["schema"] != 1
                or not 0 <= time.monotonic() - value["sampled_steady"] <= self.profile.max_age
            ):
                raise ValueError(
                    f"reference schema/time: schema={value['schema']}, "
                    f"age_s={time.monotonic() - value['sampled_steady']:.6f}, "
                    f"max_age_s={self.profile.max_age}"
                )
            if (
                value["boot_id"] == self._reference_boot
                and value["sequence"] <= self._reference_sequence
            ):
                return
            self._reference_boot = value["boot_id"]
            self._reference_sequence = value["sequence"]
            self._reference = value
        except (ValueError, KeyError, TypeError) as exc:
            self._warn_reference(f"payload validation: {str(exc)[:256]}")
            return

    def _stationary(self, body: Feedback | None, wheel: Feedback | None, now: float) -> bool:
        c = self.controller.config
        if not self.profile.simulation_only and body is not None:
            packet = self._reference
            if packet is None:
                return False
            ref = packet["reference"]
            age = max(0.0, self._reference_age_ros(ref)) + (ref["end_ros"] - ref["start_ros"]) / 2
            if (
                abs(body.velocity.vx)
                + self.profile.linear_error
                + self.profile.reaction_linear_accel * age
                > self.profile.stationary_linear_mps
                or abs(body.velocity.wz)
                + self.profile.yaw_error
                + self.profile.reaction_yaw_accel * age
                > self.profile.stationary_yaw_rps
            ):
                return False
        return bool(
            body is not None
            and wheel is not None
            and body.fresh(now)
            and wheel.fresh(now)
            and body.epoch == self._body_epoch_used
            and wheel.epoch == self._wheel_epoch_used
            and self.controller.measured_still
            and max(abs(body.velocity.vx), abs(wheel.velocity.vx)) <= c.still_speed_mps
            and max(abs(body.velocity.wz), abs(wheel.velocity.wz)) <= c.still_yaw_rps
        )

    def command(self, msg: Any) -> bool:
        """Called only AFTER the existing authenticated FinalCommand generation fence."""
        now = time.monotonic()
        if self._closed:
            return False
        reason = str(msg.close_reason)
        if reason not in {
            "prepared",
            "intent",
            "release",
            "expiry",
            "connection_retired",
            "estop",
            "fault",
            "task_revoke",
            "shutdown",
            "qualification_lost",
            "estop_reset",
            "fault_recover",
        }:
            return False
        if msg.source_kind not in {"manual", "navigation", "idle"}:
            return False
        if (
            not all(
                math.isfinite(x) for x in (msg.received_steady, msg.steady_deadline, msg.vx, msg.wz)
            )
            or not 0 <= msg.received_steady <= now
        ):
            return False
        if msg.owner_boot_id == self._input_boot and msg.generation < self.output_generation:
            return False
        if msg.owner_boot_id != self._input_boot:
            self._input_boot = msg.owner_boot_id
            self._input_floor = -1
        self.output_generation = int(msg.generation)
        if reason != "intent":
            if msg.sequence != 0 or msg.steady_deadline != 0 or msg.vx != 0 or msg.wz != 0:
                return False
            key = (msg.owner_boot_id, int(msg.generation), int(msg.sequence))
            payload = tuple(getattr(msg, name) for name in PAYLOAD_FIELDS)
            if key == self._barrier_key:
                if payload != self._barrier_payload:
                    return False
                if self._barrier_complete:
                    return True
            else:
                self._barrier_key = key
                self._barrier_payload = payload
                self._barrier_complete = False
            accepted = self._apply_close(msg, reason, now)
            if accepted:
                self._barrier_complete = True
            return accepted
        if (
            msg.generation <= self._input_floor
            or msg.source_kind == "idle"
            or not msg.source_id
            or not math.isfinite(msg.received_steady)
            or not msg.received_steady <= now < msg.steady_deadline
        ):
            return False
        velocity = Velocity(float(msg.vx), float(msg.wz))
        if not all(math.isfinite(x) for x in (velocity.vx, velocity.wz)):
            return False
        source_id = f"{msg.source_id}:{msg.source_epoch}"
        if msg.source_kind == "manual" and source_id != self._manual_motion_source:
            try:
                _, body, wheel = self._inputs(now)
                imu = self._imu_feedback(now)
                if body is None and not self._limited_ready(now, wheel, imu):
                    return False
            except (ValueError, KeyError, TypeError):
                return False
        if self.target.source_id != source_id:
            if msg.source_kind == "navigation" and self.machine.task_paused:
                self._event(K.TASK_RESUME, now, msg.source_id)
            self._event(
                K.MANUAL_START if msg.source_kind == "manual" else K.TASK_START, now, msg.source_id
            )
        expected = MotionMode.MANUAL if msg.source_kind == "manual" else MotionMode.TASK
        if self.machine.state.mode not in {expected, MotionMode.OBSTACLE_HOLD}:
            self.revoke("state_rejected_target")
            return False
        self.target = MotionTarget(velocity, msg.source_kind, source_id, float(msg.steady_deadline))
        return True

    def _apply_close(self, msg: Any, reason: str, now: float) -> bool:
        self.revoke(reason, now=now)
        if reason in {"estop_reset", "fault_recover"}:
            if not self._hardware_status_ready(now):
                return False
            try:
                _, body, wheel = self._inputs(now)
                if not self._stationary(body, wheel, now):
                    return False
                assert body is not None and wheel is not None
                self.machine.observe(
                    RecoveryObservation(
                        body.epoch,
                        body.sequence,
                        body.stamp_steady,
                        body.deadline_steady,
                        True,
                        True,
                    ),
                    now,
                )
                self._event(
                    K.ESTOP_RESET if reason == "estop_reset" else K.FAULT_RECOVER,
                    now,
                    msg.source_id,
                    int(msg.gesture),
                )
                if self.machine.state.mode in {MotionMode.ESTOP, MotionMode.EXECUTION_FAULT}:
                    return False
                return self.controller.reset_at_stillness(body, wheel, now)
            except (ValueError, KeyError, TypeError):
                return False
        elif reason == "estop":
            explicit = bool(msg.request_id) and msg.source_kind != "navigation"
            if self.machine.state.mode is not MotionMode.ESTOP or explicit:
                self._event(K.SOFTWARE_ESTOP, now, msg.source_id)
        elif reason == "fault":
            # Producer fault close is a latch reflection, not a new detector event.
            state = self.machine.state
            already = state.mode is MotionMode.EXECUTION_FAULT or (
                state.mode is MotionMode.ESTOP and state.fault_revision > 0
            )
            if not already:
                self._event(K.EXECUTION_FAULT, now, msg.source_id)
        elif msg.source_kind == "manual":
            self._event(
                K.MANUAL_EXPIRED if reason == "expiry" else K.MANUAL_RELEASE, now, msg.source_id
            )
        elif reason in {"task_revoke", "qualification_lost", "shutdown"}:
            self._event(K.TASK_PAUSE, now, msg.source_id)
        return True

    def revoke(self, reason: str, *, now: float | None = None) -> None:
        # One logical transition uses one steady instant. Resampling inside a
        # close followed by its event would make that event appear to go back.
        now = time.monotonic() if now is None else now
        if reason not in {
            "prepared",
            "release",
            "expiry",
            "connection_retired",
            "task_revoke",
            "qualification_lost",
            "shutdown",
        }:
            self._input_floor = max(self._input_floor, self.output_generation)
        self.event_sequence += 1
        kind = K.MANUAL_EXPIRED if self.target.source == "manual" else K.TASK_PAUSE
        # The state machine already records detector-caused suspension. Do not
        # recast it as an explicit user pause (which cancels feedback replan).
        if self.target.source != "idle" and reason not in {
            K.FEEDBACK_LOST.value,
            K.EXECUTION_FAULT.value,
        }:
            self.machine.reduce(
                MotionEvent(kind, self.target.source_id.rsplit(":", 1)[0], 0, self.event_sequence),
                now_steady=now,
            )
        self.scheduler.stop()
        self.target = MotionTarget(Velocity(0, 0), "idle", self.boot, self.model_deadline)

    def _observation(self, now: float) -> tuple[Any, Pose2D]:
        if self.node._depth_policy != "laser_only":
            raise ValueError("candidate depth adapter not configured")
        if not self.node._scan_topology_ready():
            raise ValueError("scan_topology_unqualified")
        if not self.node._scans:
            raise ValueError("scan_missing")
        raw = self.node._scans[-1]
        m = raw.message
        if not 0 <= now - raw.received <= self.profile.max_age:
            raise ValueError("scan_expired")
        packet = self._reference
        if packet is None:
            raise ValueError("reference_missing")
        poses = packet.get("poses", [])
        prior = [p for p in poses if p["stamp_ros"] <= raw.stamp]
        if not prior or not poses:
            raise ValueError("scan_pose_alignment_missing")
        old, current = prior[-1], poses[-1]
        if old["epoch"] != current["epoch"] or raw.stamp - old["stamp_ros"] > 0.15:
            raise ValueError("scan_pose_alignment_gap")
        dx = current["x"] - old["x"]
        dy = current["y"] - old["y"]
        yaw = old["yaw"]
        pose = Pose2D(
            math.cos(yaw) * dx + math.sin(yaw) * dy,
            -math.sin(yaw) * dx + math.cos(yaw) * dy,
            current["yaw"] - yaw,
        )
        key = (m.source_epoch, m.sequence)
        if key != self._perception_key:
            scan = ScanObservation(
                tuple(m.ranges),
                m.angle_min,
                m.angle_increment,
                m.range_min,
                m.range_max,
                raw.stamp,
                "ros",
                Pose2D(m.sensor_x, m.sensor_y, m.sensor_yaw),
                m.transform_stamp,
                str(self.node.get_parameter("body_frame").value),
                raw.stamp,
                True,
                0.01,
                False,
                True,
            )
            filtered = filter_self(
                scan, self.node._profile(), self_mask_range_uncertainty=self.profile.sensor_error
            )
            from senior_robot.motion_core_perception import scan_free_patches

            policy = self.profile.perception_no_return_policy(m.frame_id, m.range_max)
            patches = scan_free_patches(filtered, no_return_policy=policy)
            hits = tuple(r.endpoint for r in filtered.external_hits if r.endpoint is not None)
            self._perception = prepare_perception(
                patches,
                hits,
                error_m=self.profile.sensor_error,
                stamp_steady=raw.received,
                deadline_steady=raw.received + self.profile.max_age,
                source_epoch=m.source_epoch,
                frame_id="scan_reference",
                revision=m.sequence,
                forecast_horizon_s=self.profile.horizon_s,
                no_return_policy=policy,
                basis=(self.profile.perception_basis + "; " + policy.value),
                max_patches=2048,
                max_vertices=8192,
                max_hits=2048,
            )
            self._perception_key = key
        assert self._perception is not None
        return self._perception, pose

    def _reference_age_ros(self, ref: dict[str, Any]) -> float:
        return float(self.node.get_clock().now().nanoseconds / 1e9) - float(ref["end_ros"])

    def _inputs(self, now: float) -> tuple[Any, Any, Any]:
        data = self._reference
        if data is None or not 0 <= now - data["sampled_steady"] <= self.profile.max_age:
            raise ValueError("reference_expired")
        ref = data["reference"]
        live = {
            x["source"]: x for x in data["sources"] if x["ready"] and now < x["valid_until_steady"]
        }
        wheel = None
        body = None
        if "wheel" in live:
            row = live["wheel"]
            s = row["sample"]
            v = s["values"]
            wheel = Feedback(
                Velocity(v[0], v[1]),
                s["received_steady"],
                row["valid_until_steady"],
                s["epoch"],
                s["sequence"],
                True,
                "native wheel point; not body truth",
            )
        average = ref["average"]
        if average is not None and "laser" in live:
            row = live["laser"]
            s = row["sample"]
            age = self._reference_age_ros(ref)
            # Explicit bounded constant-rate predictor, not a relabelled measured
            # interval. The horizon is in ROS time, not an extra steady TTL;
            # steady expiry remains the original admission owner's deadline.
            # Its physical error is charged to the candidate risk model.
            if 0 <= age <= self.profile.temporal().prediction_horizon_s:
                body = Feedback(
                    Velocity(average["vx"], average["wz"]),
                    s["received_steady"],
                    row["valid_until_steady"],
                    s["epoch"],
                    s["sequence"],
                    True,
                    "candidate bounded constant-rate projection; original interval retained",
                )
        if body is not None and not body.fresh(now):
            body = None
        return ref, body, wheel

    def _imu_feedback(self, now: float) -> Feedback | None:
        """Native yaw point only; the unused linear slot is not body evidence."""
        packet = self._reference
        if packet is None:
            return None
        ros_now = self.node.get_clock().now().nanoseconds / 1e9
        for row in packet["sources"]:
            if row["source"] != "imu" or not row["ready"]:
                continue
            sample = row["sample"]
            if (
                not 0
                <= ros_now - sample["stamp_ros"]
                <= self.profile.temporal().prediction_horizon_s
            ):
                return None
            point = Feedback(
                Velocity(0, sample["values"][0]),
                sample["received_steady"],
                row["valid_until_steady"],
                sample["epoch"],
                sample["sequence"],
                True,
                "native IMU yaw only; linear component unsupported",
            )
            return point if point.fresh(now) else None
        return None

    def _limited_ready(
        self,
        now: float,
        wheel: Feedback | None,
        imu: Feedback | None,
    ) -> bool:
        return bool(
            wheel is not None
            and wheel.fresh(now)
            and imu is not None
            and imu.fresh(now)
            and self.controller.limited_rearm_ready(now, epochs=(wheel.epoch, imu.epoch))
        )

    def tick(self) -> None:
        now = time.monotonic()
        self.sequence += 1
        body = None
        wheel = None
        healthy = False
        reason = "not_ready"
        body_state = None
        deadline = None
        stamp = None
        output = Velocity(0, 0)
        try:
            if not self._hardware_status_ready(now):
                raise ValueError("hardware_estop_status_missing_or_expired")
            if now >= self.target.deadline_steady and self.target.source != "idle":
                self.scheduler.stop()
                self._event(
                    K.MANUAL_EXPIRED if self.target.source == "manual" else K.TASK_PAUSE,
                    now,
                    self.target.source_id.rsplit(":", 1)[0],
                )
                self.target = MotionTarget(Velocity(0, 0), "idle", self.boot, self.model_deadline)
            ref, body, wheel = self._inputs(now)
            imu_point = self._imu_feedback(now)
            packet = self._reference
            if packet is None:
                raise ValueError("reference_missing")
            live = {
                x["source"]: x
                for x in packet["sources"]
                if x["ready"] and now < x["valid_until_steady"]
            }
            imu = live.get("imu")
            imu_current = (
                imu is not None
                and 0
                <= self.node.get_clock().now().nanoseconds / 1e9 - imu["sample"]["stamp_ros"]
                <= self.profile.temporal().prediction_horizon_s
            )
            healthy = (
                wheel is not None
                and (body is not None or imu_current)
                and now < self.model_deadline
            )
            if body is not None:
                avg = ref["average"]
                body_state = {
                    "average": avg,
                    "source": ref["mode"],
                    "computed_steady": body.stamp_steady,
                    "valid_until_steady": body.deadline_steady,
                }
                stamp, deadline = body.stamp_steady, body.deadline_steady
                self.machine.observe(
                    RecoveryObservation(
                        body.epoch,
                        body.sequence,
                        body.stamp_steady,
                        body.deadline_steady,
                        healthy,
                        self._stationary(body, wheel, now),
                    ),
                    now,
                )
            if healthy:
                if body is None and wheel is not None:
                    stamp, deadline = wheel.stamp_steady, wheel.deadline_steady
                if body is not None:
                    self._event(K.FEEDBACK_HEALTHY, now)
            elif self._last_healthy:
                self._event(K.FEEDBACK_LOST, now)
            self._last_healthy = healthy
            perception, pose = self._observation(now)
            mode = SourceMode(ref["mode"])
            if body is None:
                mode = (
                    SourceMode.WHEEL_IMU_LIMITED
                    if wheel is not None and imu_current
                    else SourceMode.UNAVAILABLE
                )
            if wheel is None:
                raise ValueError("wheel_feedback_missing")
            chosen = {"wheel", "imu"} if body is None else {"laser", "wheel", "imu"}
            consumed = [
                x
                for x in ref["source_health"]
                if x["source"] in chosen and x["ready"] and now < x["valid_until_steady"]
            ]
            if not consumed:
                raise ValueError("motion_sources_not_ready")
            epoch = "|".join(x["epoch"] for x in consumed)
            average = (
                BodyTwist(**ref["average"])
                if body is not None and ref["average"] is not None
                else None
            )
            motion = BodyMotion(
                average,
                None,
                ref["start_ros"],
                ref["end_ros"],
                data_time(self._reference),
                min(x["valid_until_steady"] for x in consumed),
                epoch,
                mode not in {SourceMode.UNAVAILABLE, SourceMode.RECOVERING},
                "native T3 reference; upper unsupported",
            )
            p = self.profile
            response = ResponseBounds(
                p.accel,
                p.decel,
                p.jerk,
                p.yaw_accel,
                p.yaw_decel,
                p.yaw_jerk,
                p.lag,
                self.model_deadline,
                p.profile_id,
            )
            cap = p.degraded_cap if mode is SourceMode.WHEEL_IMU_LIMITED else p.cap
            a = self.controller.trajectory.acceleration
            v = self.controller.trajectory.velocity
            # Independent deployment response envelope, not wheel truth.
            # Hardware requires explicit evidence and operating conditions.
            if body is None:
                # Body speed is unknown: use the independent whole-body envelope.
                linear = (-p.assumed_body_cap, p.assumed_body_cap)
                gyro = float(imu["sample"]["values"][0]) if imu_current and imu is not None else 0.0
                angular = (gyro - p.yaw_error, gyro + p.yaw_error)
            else:
                age = (
                    max(0.0, self._reference_age_ros(ref)) + (ref["end_ros"] - ref["start_ros"]) / 2
                )
                predictor_accel = (
                    Velocity(p.accel, p.yaw_accel)
                    if p.simulation_only
                    else p.reaction_acceleration(a)
                )
                dv = p.linear_error + predictor_accel.vx * age
                dw = p.yaw_error + predictor_accel.wz * age
                linear = (min(body.velocity.vx - dv, v.vx), max(body.velocity.vx + dv, v.vx))
                angular = (min(body.velocity.wz - dw, v.wz), max(body.velocity.wz + dw, v.wz))
            risk = RiskEnvelope(
                p.profile_id,
                p.risk_basis or p.profile_id,
                epoch,
                (
                    SourceMode.LASER_REFERENCE,
                    SourceMode.FUSED_REFERENCE,
                    SourceMode.GPS_IMU_REFERENCE,
                    SourceMode.WHEEL_IMU_LIMITED,
                ),
                p.profile_id,
                self.model_deadline,
                p.horizon_s,
                linear,
                angular,
                p.reaction_acceleration(a),
                p.position_error,
                p.orientation_error_rad,
                p.lateral_drift_mps,
                cap,
                p.yaw_cap,
                bool(p.simulation_only and p.unvalidated),
                lateral_stop_s=p.lateral_stop_s,
                brake_drive_overshoot=Velocity(p.drive_linear_overshoot, p.drive_yaw_overshoot),
                brake_tracking_basis=p.brake_tracking_basis or p.profile_id,
            )
            ctx = ConstraintSnapshot(
                self.target,
                motion,
                mode,
                response,
                Footprint(p.length, p.width),
                perception,
                pose,
                "scan_reference",
                p.profile_id,
                p.profile_id,
                risk,
            )
            permission = self.scheduler.update(ctx, now)
            estop = self.machine.state.mode is MotionMode.ESTOP
            decision = self.controller.tick(
                self.target.velocity,
                ctx,
                permission,
                body,
                wheel,
                now,
                estop=estop,
                imu=imu_point,
                require_limited_rearm=(
                    body is None
                    and self.target.source == "manual"
                    and self.target.source_id != self._manual_motion_source
                    and self.target.velocity != Velocity(0, 0)
                ),
            )
            reason = decision.reason
            if self.target.velocity == Velocity(0, 0) and decision.verified:
                self._event(K.OBSTACLE_CLEAR, now)
            elif self.target.velocity != Velocity(0, 0):
                if permission.permit is not None:
                    self._event(K.OBSTACLE_CLEAR, now)
                elif permission.brake is not None and permission.brake.reason in {
                    "sweep_obstructed",
                    "sweep_unknown",
                }:
                    self._event(K.OBSTACLE, now)
            if decision.reason not in {"clock_initialized", "invalid_tick"}:
                if body is not None:
                    self._body_epoch_used = body.epoch
                if wheel is not None:
                    self._wheel_epoch_used = wheel.epoch
            if decision.command is not None:
                output = decision.command
            # No candidate never holds the previous nonzero Twist. The fallback
            # is explicitly unverified stop/fault output, not a safe-stop claim.
            elif decision.emergency and reason in {
                "stop_envelope_blocked",
                "stop_response_failed",
                "infeasible_no_reverse",
            }:
                condition = (reason, epoch)
                if condition != self._last_fault_condition:
                    self._event(K.EXECUTION_FAULT, now)
                    self._last_fault_condition = condition
            elif not decision.emergency:
                self._last_fault_condition = None
            reason = f"{mode.value}:{reason}"
            self.node._emit_motion_core(output, reason)
            if self.target.source == "manual" and output != Velocity(0, 0):
                self._manual_motion_source = self.target.source_id
            if body is None and wheel is not None and imu_point is not None:
                ready = self._limited_ready(now, wheel, imu_point)
                readiness_deadline = min(wheel.deadline_steady, imu_point.deadline_steady)
                if ready:
                    readiness_deadline = min(readiness_deadline, decision.evidence_deadline)
                    deadline = (
                        min(deadline, readiness_deadline)
                        if deadline is not None
                        else readiness_deadline
                    )
                self.machine.observe(
                    RecoveryObservation(
                        epoch,
                        self.sequence,
                        min(wheel.stamp_steady, imu_point.stamp_steady),
                        readiness_deadline,
                        False,
                        False,
                        manual_rearm_ready=ready,
                    ),
                    now,
                )
                if healthy:
                    self._event(K.FEEDBACK_HEALTHY, now)
        except (ValueError, TypeError, KeyError, AttributeError, OverflowError) as exc:
            self.controller.invalidate_limited_rearm()
            reason = str(exc)[:256] or type(exc).__name__
            self.node._emit_motion_core(Velocity(0, 0), reason)
            if self._last_healthy:
                self._event(K.FEEDBACK_LOST, now)
                self._last_healthy = False
            healthy = False
        state = self.machine.state
        actions = list(state.allowed_actions)
        actual_stationary = healthy and self._stationary(body, wheel, now)
        if not actual_stationary:
            actions = [x for x in actions if x not in {"estop_reset", "fault_recover"}]
        if not healthy:
            actions = [x for x in actions if x not in {"acquire", "replan_task"}]
        # Schema is backend-owned; strict validation precedes every publication.
        runtime = MotionRuntimeState.model_validate(
            {
                "schema": 1,
                "robot_id": str(self.node.get_parameter("robot_id").value),
                "boot_id": self.boot,
                "sequence": self.sequence,
                "sampled_steady": now,
                "valid_until": now + 0.15,
                "mode": state.mode.value,
                "reason": reason,
                "stationary": bool(actual_stationary),
                "healthy": healthy,
                "evidence_stamp_steady": stamp,
                "evidence_deadline_steady": deadline,
                "estop_revision": state.estop_revision,
                "fault_revision": state.fault_revision,
                "confirmed_estop_status": "hardware_latched"
                if self._hardware
                else "software_latched"
                if state.mode is MotionMode.ESTOP
                else "clear",
                "hardware_estop_observation": (
                    "not_configured"
                    if self.profile.simulation_only
                    else "unknown"
                    if self.profile.hardware_estop.mode == "unknown"
                    else "independent_unreadable"
                    if self.profile.hardware_estop.mode == "independent_unreadable"
                    else "unavailable"
                    if not self._hardware_status_ready(now)
                    else "latched"
                    if self._hardware
                    else "clear"
                ),
                "allowed_actions": actions,
                "task_id": state.task_id,
                "output_generation": self.output_generation,
                "body_reference": body_state,
                "wheel_linear_mps": wheel.velocity.vx if wheel is not None else None,
            }
        )
        msg = self._runtime_type()
        msg.data = runtime.model_dump_json()
        self._runtime.publish(msg)

    def close(self) -> None:
        self._closed = True
        self.scheduler.stop()
        self.executor.shutdown(wait=False, cancel_futures=True)


def data_time(value: dict[str, Any] | None) -> float:
    return float(value["sampled_steady"]) if value is not None else 0.0
