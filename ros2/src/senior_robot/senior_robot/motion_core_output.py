"""Shared motion output composition; safety_gate remains the sole actuator writer.

Deployment profiles supply response evidence; no ground truth enters this module.
"""

import json
import math
import os
import time
from concurrent.futures import ThreadPoolExecutor
from dataclasses import replace
from pathlib import Path
from typing import Any
from uuid import uuid4

from senior_control.models import Pose2D, Velocity
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
from senior_control.motion.estimator import Origin
from senior_control.motion.kinematics import compose, inverse
from senior_control.motion.manual_target import ManualLinearTarget
from senior_control.motion.source_fences import SourceEpochFence
from senior_control.motion.source_modes import SourceMode
from senior_control.motion.startup_learning import StartupLearning
from senior_control.motion.state_machine import MotionStateMachine, RecoveryObservation
from senior_control.motion.stopping_input import StoppingRisk
from senior_control.protocol import MotionRuntimeState
from senior_control.runtime.diagnostics import QueueSnapshot, configure_diagnostics
from senior_control.safety.self_filter import ScanObservation, filter_self
from senior_control.safety.self_learning import SelfReturnModel
from senior_control.safety.space_estimator import SpaceEstimation, SpaceObservation
from senior_control.sensors.policy import default_policy
from senior_robot.final_command_frame import PAYLOAD_FIELDS
from senior_robot.motion_observation_adapter import NativeObservationRosBridge, observation_origin
from senior_robot.motion_sensor_runtime import MotionSensorRuntime

_REFERENCE_DEPTH = 10


class MotionCoreOutput:
    def __init__(self, node: Any, profile: Any, *, ingress_factory=NativeObservationRosBridge):
        from std_msgs.msg import String

        self.node, self.profile = node, profile
        self.diagnostics = configure_diagnostics(profile.diagnostics)
        self.sensors = MotionSensorRuntime(profile, self.diagnostics)
        self.diagnostics.queue(
            QueueSnapshot(
                "ros.control.callbacks", None, None, depth_basis="executor_depth_unavailable"
            ),
            time.monotonic(),
            owner=self,
        )
        self._ingress = None
        self._last_tick_diagnostic = None
        self.acceleration_model = profile.acceleration_model()
        self.stopping = StoppingRisk()
        self.boot = uuid4().hex
        claim = os.environ.get("SENIOR_STARTUP_LEARNING_CLAIM")
        self._startup_claim_path = Path(claim) if claim else None
        self.startup = StartupLearning(
            Path(claim) if claim else None,
            linear_error_mps=profile.linear_error,
            yaw_error_rps=profile.yaw_error,
        )
        self.self_returns = None
        self._learning_scan_key = None
        self._filter_signature = None
        self._learning_geometry = None
        self._learning_motion_emitted = False
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
        self._task_identity: tuple[str, int] | None = None
        # Proofs are in-flight only; explicit durable collection never recreates identity.
        self._task_retirements: dict[tuple[str, str, int, str], tuple[int, float, int]] = {}
        self._terminal_released: set[tuple[str, str, int, str]] = set()
        self._input_boot = ""
        self._input_floor = -1
        self._manual_motion_source: str | None = None
        # One-way startup phase. Zero commands never establish body stillness;
        # only independent stillness or an accepted nonzero source ends it.
        self._startup_motion_unconfirmed = True
        self._reference: dict[str, Any] | None = None
        self._reference_boot = ""
        self._reference_sequence = 0
        self._reference_boot_fence = SourceEpochFence(
            max_retired=profile.admission().epoch_history_limit
        )
        self._reference_warning_at = float("-inf")
        self._perception_key: tuple[Any, ...] | None = None
        self._perception_pose = None
        self._space_estimate = None
        self._perception: Perception | None = None
        self.space_estimator = SpaceEstimation(
            frame=self.sensors.pose_estimator.frame,
            frame_epoch="1:0",
            software_pose_error_m=profile.position_error,
        )
        self._barrier_key: tuple[str, int, int] | None = None
        self._barrier_payload: tuple[Any, ...] | None = None
        self._barrier_complete = False
        self._last_fault_condition: tuple[str, str] | None = None
        self._closed = False
        self._last_healthy = False
        self._body_epoch_used: str | None = None
        self._wheel_epoch_used: str | None = None
        self._runtime = node.create_publisher(String, "/motion/runtime", 10)
        self._runtime_type = String
        from senior_robot.motion_lifecycle import with_dds_info

        with_dds_info(type(self)._on_reference)
        # The estimator is local to the output cycle. No subscription to a second
        # node's already-selected /motion/core_reference drives this owner.
        self._subscription = None
        if ingress_factory is not None:
            if not node.has_parameter("wheel_frame"):
                node.declare_parameter("wheel_frame", "odom")
            if not hasattr(node, "_tf"):
                from tf2_ros import Buffer, TransformListener

                node._tf = Buffer()
                self._tf_listener = TransformListener(node._tf, node)
            self._ingress = ingress_factory(
                node,
                profile,
                sample_sink=self.sensors.ingest,
                source_invalidated=self.sensors.invalidate,
                publish=False,
            )
        self._optional_inputs = None
        if ingress_factory is not None and (
            profile.gps_input is not None or profile.rgbd_input is not None
        ):
            from senior_robot.optional_motion_inputs import OptionalMotionInputs

            self._optional_inputs = OptionalMotionInputs(node, profile, self.sensors)
    def _warn_reference(self, reason: str) -> None:
        now = time.monotonic()
        if now - self._reference_warning_at >= 2.0:
            self._reference_warning_at = now
            self.node.get_logger().warning(f"motion core reference rejected: {reason}")

    def _event(self, kind: K, now: float, source: str = "", revision: int = 0) -> None:
        self.event_sequence += 1
        self.machine.reduce(
            MotionEvent(kind, source, revision, self.event_sequence), now_steady=now
        )
        if kind in {K.FEEDBACK_LOST, K.EXECUTION_FAULT}:
            if (
                kind is K.FEEDBACK_LOST
                and self.startup.state == "initializing"
                and self.target.source == "idle"
            ):
                # Initialization has not proposed any motion. Sensor/pose
                # readiness may arrive later; safety state still records loss.
                return
            self.revoke(kind.value, now=now)

    def _on_reference(self, msg: Any, info: Any) -> None:
        # //Compatible: retained offline replay transport carries raw sources only.
        origin = observation_origin(
            info,
            self.node.get_publishers_info_by_topic("/motion/core_reference"),
            "motion_fusion",
            allow_topology_only=bool(self.profile.simulation_only and self.profile.unvalidated),
        )
        if origin is None:
            return
        try:
            if len(msg.data) > 131072:
                raise ValueError("oversize raw reference")
            packet = json.loads(msg.data)
            if (
                type(packet.get("schema")) is not int
                or packet["schema"] != 1
                or type(packet.get("sequence")) is not int
                or packet["sequence"] < 1
                or not isinstance(packet.get("boot_id"), str)
                or not packet["boot_id"]
                or len(packet["boot_id"]) > 128
            ):
                raise ValueError("invalid offline raw snapshot")
            ros = self.node.get_clock().now().nanoseconds / 1e9
            default_policy().age(
                stamp_ros=packet["sampled_ros"],
                received_ros=packet["sampled_ros"],
                received_steady=packet["sampled_steady"],
                now_ros=ros,
                now_steady=time.monotonic(),
            )
            if (
                max(ros - packet["sampled_ros"], time.monotonic() - packet["sampled_steady"])
                >= default_policy().hard_age_s
            ):
                return
            if (
                packet["boot_id"] == self._reference_boot
                and packet["sequence"] <= self._reference_sequence
            ):
                return
            if not self._reference_boot_fence.accept(packet["boot_id"]):
                return
            self.sensors.ingest_packet(packet)
            self._reference_boot, self._reference_sequence = packet["boot_id"], packet["sequence"]
        except (ValueError, TypeError, KeyError) as exc:
            self._warn_reference(str(exc))

    def _stationary(self, body: Feedback | None, wheel: Feedback | None, now: float) -> bool:
        c = self.controller.config
        # Physical reset/stillness uses independent observed evidence. A wheel
        # estimate remains useful for control without becoming physical truth.
        body = self.sensors.independent
        return bool(
            body is not None
            and body.fresh(now)
            and all(body.observed_axes)
            and self.sensors.stationary_supported
            and abs(body.velocity.vx) <= c.still_speed_mps
            and abs(body.velocity.wz) <= c.still_yaw_rps
        )

    def _consume_ready_reference(self) -> None:
        # Native ingress callbacks feed the single owner. No second snapshot queue.
        return

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
            "task_done",
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
        manual_linear = None
        try:
            metadata = ManualLinearTarget(
                msg.manual_linear, msg.manual_linear_limit, msg.degraded_linear_limit
            )
        except (ValueError, AttributeError):
            return False
        if msg.source_kind == "manual" and reason == "intent":
            if msg.vx != metadata.fraction * metadata.full_stick_mps:
                return False
            manual_linear = metadata
        elif metadata != ManualLinearTarget(0, 0, 0):
            return False
        if msg.owner_boot_id == self._input_boot and msg.generation < self.output_generation:
            return False
        if msg.owner_boot_id != self._input_boot:
            self._input_boot = msg.owner_boot_id
            self._input_floor = -1
            self._task_retirements.clear()
            self._terminal_released.clear()
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
                self._remember_task_retirement(msg)
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
            self._interrupt_learning("control_takeover")
            self._retire_startup(now)
            if msg.source_kind == "navigation" and self.machine.task_paused:
                self._event(K.TASK_RESUME, now, msg.source_id)
            self._event(
                K.MANUAL_START if msg.source_kind == "manual" else K.TASK_START, now, msg.source_id
            )
        expected = MotionMode.MANUAL if msg.source_kind == "manual" else MotionMode.TASK
        if self.machine.state.mode not in {expected, MotionMode.OBSTACLE_HOLD}:
            self.revoke("state_rejected_target")
            return False
        if velocity != Velocity(0, 0):
            self._startup_motion_unconfirmed = False
        self._interrupt_learning("control_takeover")
        self.target = MotionTarget(
            velocity, msg.source_kind, source_id, float(msg.steady_deadline), manual_linear
        )
        if msg.source_kind == "navigation":
            self._task_identity = (str(msg.source_id), int(msg.source_epoch))
            # An accepted higher epoch irreversibly supersedes old pause receipts.
            obsolete = [
                key
                for key, proof in self._task_retirements.items()
                if key[1] == msg.source_id
                and proof[0] < msg.source_epoch
                and key not in self._terminal_released
            ]
            for key in obsolete:
                self._task_retirements.pop(key)
                self._terminal_released.discard(key)
        return True

    def _startup_stop_uncertain(self, body: Feedback | None, context: ConstraintSnapshot) -> bool:
        """Classify initial unknown motion; never relax the T4 stopping result."""
        return (
            self._startup_motion_unconfirmed
            and self.sensors.independent is None
            and context.mode is SourceMode.WHEEL_IMU_LIMITED
            and context.motion.average is None
            and context.motion.upper is None
            and self.target.source == "idle"
            and self.target.velocity == Velocity(0, 0)
            and self.controller.trajectory.velocity == Velocity(0, 0)
            and self.controller.trajectory.acceleration == Velocity(0, 0)
            and self.machine.state.mode not in {MotionMode.ESTOP, MotionMode.EXECUTION_FAULT}
        )

    def _apply_close(self, msg: Any, reason: str, now: float) -> bool:
        if reason == "task_done" and (
            msg.source_kind != "navigation"
            or self.machine.state.task_id not in {None, msg.source_id}
        ):
            return False
        self.revoke(reason, now=now)
        if reason in {"estop_reset", "fault_recover"}:
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
        elif reason == "task_done":
            # Successful Nav2 completion retires output, not the durable task identity.
            self._event(K.TASK_PAUSE, now, msg.source_id)
        elif reason in {"task_revoke", "qualification_lost", "shutdown"}:
            self._event(K.TASK_PAUSE, now, msg.source_id)
        return True

    @staticmethod
    def _task_retirement_key(msg: Any) -> tuple[str, str, int, str]:
        return (msg.owner_boot_id, msg.source_id, int(msg.generation), msg.request_id)

    def _remember_task_retirement(self, msg: Any) -> None:
        if (
            msg.source_kind != "navigation"
            or not msg.request_id
            or msg.source_epoch < 1
            or msg.close_reason
            not in {"task_revoke", "task_done", "qualification_lost", "shutdown", "expiry", "fault"}
        ):
            return
        key = self._task_retirement_key(msg)
        if key in self._task_retirements:
            return
        if len(self._task_retirements) >= 256:
            return  # Output remains revoked; no new identity-release proof is issued.
        epoch = int(msg.source_epoch)
        if self.machine.state.task_id == msg.source_id and self._task_identity is None:
            return  # A retained task with unknown epoch is not an identity-release proof.
        if self._task_identity is not None and self._task_identity[0] == msg.source_id:
            if epoch < self._task_identity[1]:
                return
            epoch = self._task_identity[1]
        self._task_retirements[key] = (epoch, float(msg.received_steady), int(msg.connection_epoch))

    def retirement_epoch(self, msg: Any) -> int | None:
        proof = self._task_retirements.get(self._task_retirement_key(msg))
        return proof[0] if proof is not None else None

    def collect_terminal(self, msg: Any) -> bool:
        """Durable settlement only reclaims metadata; it never releases identity.

        Old barriers cannot recreate forgotten proofs: command generation is
        irreversible, and an identical current barrier returns without minting.
        Unknown collections are idempotent no-ops, never terminal-release ACKs.
        """
        if (
            self._closed
            or msg.owner_boot_id != self._input_boot
            or msg.close_reason != "task_settled"
            or msg.source_kind != "navigation"
            or msg.session_id != msg.source_id
            or not msg.source_id
            or not msg.request_id
            or msg.source_epoch < 1
            or msg.generation != 0
            or msg.connection_epoch != msg.source_epoch
            or msg.sequence != 0
            or msg.steady_deadline != 0
            or msg.vx != 0
            or msg.wz != 0
            or msg.gesture != 0
        ):
            return False
        if self.machine.state.task_id == msg.source_id and (
            self._task_identity is None or self._task_identity[1] <= msg.source_epoch
        ):
            return False
        keys = [
            key
            for key, proof in self._task_retirements.items()
            if key[0] == msg.owner_boot_id
            and key[1] == msg.source_id
            and proof[0] <= msg.source_epoch
        ]
        if keys and not any(key in self._terminal_released for key in keys):
            return False
        for key in keys:
            self._task_retirements.pop(key)
            self._terminal_released.discard(key)
        return True

    def release_terminal(self, msg: Any) -> bool:
        """Consume only a proven identity event; never revoke a newer motion owner."""
        if (
            self._closed
            or msg.owner_boot_id != self._input_boot
            or msg.close_reason != "task_terminal"
            or msg.source_kind != "navigation"
            or msg.session_id != msg.source_id
            or not msg.source_id
            or not msg.request_id
            or msg.sequence != 0
            or msg.steady_deadline != 0
            or msg.gesture != 0
            or msg.vx != 0
            or msg.wz != 0
        ):
            return False
        key = self._task_retirement_key(msg)
        proof = self._task_retirements.get(key)
        if proof != (msg.source_epoch, msg.received_steady, msg.connection_epoch):
            return False
        if key in self._terminal_released:
            return True
        if self.target.source == "navigation":
            # Even the same epoch must not be released while its navigation source is live.
            return False
        identity = (str(msg.source_id), int(msg.source_epoch))
        if self._task_identity is not None and self._task_identity != identity:
            return False
        if self.machine.state.task_id not in {None, msg.source_id} or (
            self.machine.state.task_id is not None and self._task_identity != identity
        ):
            return False
        revision = self.machine.state.fault_revision
        self._event(K.TASK_RETIRED, time.monotonic(), msg.source_id)
        if self.machine.state.task_id is not None or self.machine.state.fault_revision != revision:
            return False
        self._task_identity = None
        self._terminal_released.add(key)
        return True

    def _body_footprint(self):
        # //Compatible: offline profile-only nodes retain their rectangular footprint.
        boundary = self.node._profile().footprint if hasattr(self.node, "_profile") else ()
        return Footprint(
            self.profile.length, self.profile.width, tuple(boundary) if boundary else None
        )

    def _startup_permission_lost(self, permission, now=None):
        if self.startup.state == "validating":
            return (
                False  # Stationary post-sampling proposes zero; controller still checks stopping.
            )
        if (
            self.startup.state == "braking"
            and permission.brake is not None
            and permission.brake.stop_verified
        ):
            return False  # Planned, verified stopping does not need a driving permit.
        if self.startup.active and self._learning_motion_emitted and permission.permit is None:
            self._interrupt_learning("learning_permission_lost")
            self._startup_target(time.monotonic() if now is None else now)
            self.scheduler.stop()
            return True
        return False

    def _retire_startup(self, now):
        if self.machine.state.task_id == self.boot:
            self._event(K.TASK_RETIRED, now, self.boot)

    def _startup_target(self, now):
        if self.startup.active:
            if self.target.source not in {"idle", "startup_learning"}:
                self._interrupt_learning("other_control_source")
                return
            if self.machine.state.mode in {MotionMode.ESTOP, MotionMode.EXECUTION_FAULT}:
                self._interrupt_learning("latched_stop")
            elif self.startup.state != "initializing":
                if self.machine.state.task_id != self.boot:
                    self._event(K.TASK_START, now, self.boot)
                if self.machine.state.task_id != self.boot:
                    return  # Shared arbitration has not admitted this startup task.
                self.target = MotionTarget(
                    Velocity(self.startup.velocity, 0),
                    "startup_learning",
                    self.boot,
                    self.model_deadline,
                )
                return
        if self.target.source == "startup_learning":
            self._retire_startup(now)
            self.target = MotionTarget(Velocity(0, 0), "idle", self.boot, self.model_deadline)

    def _interrupt_learning(self, reason):
        was_active = self.startup.active
        self.startup.interrupt(reason)
        if was_active and self.self_returns is not None:
            self.self_returns.clear()
            self.space_estimator.invalidate("scan")
            self._perception_key = None

    def revoke(self, reason: str, *, now: float | None = None) -> None:
        self._interrupt_learning(reason)
        # One logical transition uses one steady instant. Resampling inside a
        # close followed by its event would make that event appear to go back.
        now = time.monotonic() if now is None else now
        self._retire_startup(now)
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

    def _sync_learning_model(self, message):
        if self.startup.state == "disabled":
            return
        boundary = tuple(self.node._profile().footprint)
        geometry = (
            message.source_epoch,
            message.sensor_x,
            message.sensor_y,
            message.sensor_yaw,
            message.angle_min,
            message.angle_increment,
            len(message.ranges),
            boundary,
            self.profile.sensor_error,
            self.profile.orientation_error_rad,
            self.profile.linear_error,
            self.profile.yaw_error,
        )
        if geometry == self._learning_geometry:
            return
        if self._learning_geometry is not None:
            self._interrupt_learning("scan_or_body_geometry_changed")
            self.startup.state, self.startup.reason = "interrupted", "scan_or_body_geometry_changed"
            if self.self_returns is not None:
                self.self_returns.clear()
            self._perception_key = None
            self._filter_signature = None
            self.space_estimator.invalidate("scan")
        self._learning_geometry = geometry
        self.self_returns = None
        if len(boundary) < 3:
            self.startup.state, self.startup.reason = "failed", "physical_polygon_missing"
            raise ValueError("physical_polygon_missing")
        radius = max(math.hypot(x, y) for x, y in boundary)
        error = self.profile.sensor_error + radius * abs(
            math.sin(self.profile.orientation_error_rad)
        )
        model = SelfReturnModel(boundary, error_m=error)
        if self._startup_claim_path is not None:
            from senior_control.safety.external_motion_features import write_feature_boundary

            write_feature_boundary(self._startup_claim_path, boundary, error)
        self.self_returns = model
        self.startup.motion_radius = radius
        self.startup.linear_error_mps = self.profile.linear_error
        self.startup.yaw_error_rps = self.profile.yaw_error

    def _observation(self, now: float) -> tuple[Any, Pose2D]:
        if self.node._depth_policy != "laser_only":
            raise ValueError("candidate depth adapter not configured")
        if not self.node._scan_topology_ready():
            raise ValueError("scan_topology_unqualified")
        if not self.node._scans:
            raise ValueError("scan_missing")
        raw = self.node._scans[-1]
        m = raw.message
        ros = self.node.get_clock().now().nanoseconds / 1e9
        root = Origin("scan", m.source_epoch, m.sequence, raw.stamp, raw.received_ros, raw.received)
        raw_key = (root.epoch, root.sequence)
        try:
            age = default_policy().age(
                stamp_ros=root.stamp_ros,
                received_ros=root.received_ros,
                received_steady=root.received_steady,
                now_ros=ros,
                now_steady=now,
            )
        except ValueError:
            self.diagnostics.sensor_attempt("scan", "raw", None, ("invalid_time",), key=raw_key)
            self.diagnostics.sensor_attempt("scan", "use", None, ("invalid_time",))
            raise
        self.diagnostics.sensor_attempt("scan", "raw", age, ("observed",), key=raw_key)
        self.diagnostics.sensor_attempt(
            "scan", "use", age, ("expired" if age.band == "invalid" else "evaluated",)
        )
        if age.band == "invalid":
            raise ValueError("scan_expired")
        self._sync_learning_model(m)
        motion = self.sensors.latest
        current = self.sensors.pose_estimator.at(ros, now_ros=ros, now_steady=now, motion=motion)
        alignment_epoch = f"{motion.generation}:{self.sensors.pose_estimator.frame_generation}"
        self.space_estimator.reset_frame(alignment_epoch)
        signature = (
            self.startup.provisional,
            None if self.self_returns is None else self.self_returns.revision,
        )
        if self._filter_signature is not None and signature != self._filter_signature:
            self.space_estimator.invalidate("scan")
        self._filter_signature = signature
        key = (m.source_epoch, m.sequence, alignment_epoch, signature)
        old = self._perception_pose if key == self._perception_key else None
        if old is None:
            old = self.sensors.pose_estimator.at(
                raw.stamp, now_ros=ros, now_steady=now, motion=motion
            )
        pose = compose(inverse(old.pose), current.pose)
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
            if self.startup.state != "disabled":
                if self.self_returns is not None:
                    if self._learning_scan_key != raw_key:
                        self._learning_scan_key = raw_key
                        self.startup.frame(self.self_returns, filtered)
                    filtered = self.self_returns.filter(
                        filtered, provisional=self.startup.provisional
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
                deadline_steady=age.deadline_steady,
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
            self.space_estimator.ingest(
                SpaceObservation(
                    "scan",
                    self._perception.occupied,
                    self._perception.free_space,
                    (root,),
                    error_m=old.error_m,
                    software_error_m=self.profile.position_error
                    + old.prediction_error.software_position_m,
                    frame=self.sensors.pose_estimator.frame,
                    pose_in_frame=old.pose,
                    frame_epoch=alignment_epoch,
                )
            )
            # Keep the geometry's original pose association stable. A later
            # re-evaluation of historical pose must not silently move its frame.
            self._perception_pose = old
            self._perception_key = key
        assert self._perception is not None
        relative_error = self.sensors.pose_estimator.relative_error(old, current)
        # Convert current-body orientation uncertainty in the scan frame using
        # the footprint radius. Apply this displacement once to geometry;
        # prepare_perception already charged the separate sensor error.
        radius = math.hypot(self.profile.length, self.profile.width) / 2
        angular = relative_error.software_yaw_rad
        software_error = relative_error.software_position_m + 2 * radius * math.sin(
            min(math.pi, angular) / 2
        )
        known_error = (
            None
            if relative_error.position_m is None or relative_error.yaw_rad is None
            else relative_error.position_m
            + 2 * radius * math.sin(min(math.pi, relative_error.yaw_rad) / 2)
        )
        space = self._space_estimate = self.space_estimator.at(
            ros,
            now,
            reference_pose=old.pose,
            alignment_error_m=known_error,
            software_alignment_error_m=software_error,
        )
        return replace(
            self._perception,
            occupied=space.occupied,
            free_space=space.free,
            deadline_steady=space.deadline_steady,
        ), pose

    def _reference_age_ns(self, ref: dict[str, Any], ros: int | None = None) -> int:
        # The wire carries float seconds: round the serialized endpoint to its
        # nearest ROS nanosecond. Precision already lost on the wire is not recovered.
        now = self.node.get_clock().now().nanoseconds if ros is None else ros
        return now - round(ref["end_ros"] * 1_000_000_000)

    def _reference_time_valid(
        self, ref: dict[str, Any], ros: int | None = None, *, future: bool = False
    ) -> bool:
        age = self._reference_age_ns(ref, ros)
        horizon = round(self.profile.temporal().prediction_horizon_s * 1_000_000_000)
        return age < 0 if future else 0 <= age <= horizon

    def _reference_age_ros(self, ref: dict[str, Any]) -> float:
        return self._reference_age_ns(ref) / 1_000_000_000

    def _inputs(self, now: float) -> tuple[Any, Any, Any]:
        ros = self.node.get_clock().now().nanoseconds / 1e9
        if self._ingress is not None:
            self.sensors.poses = list(self._ingress.core.poses)
            self.sensors.ingest_poses(self.sensors.poses)
        ref, tracking, secondary = self.sensors.estimate(ros, now)
        # Remember motion even if perception later fails before _snapshot.
        self.stopping.domains(
            self.sensors.latest,
            self.controller.trajectory.velocity,
            self._response(),
            linear_error=self.profile.linear_error,
            yaw_error=self.profile.yaw_error,
        )
        self._reference = dict(
            sampled_steady=now, sampled_ros=ros, reference=ref, poses=self.sensors.poses
        )
        return ref, tracking, secondary

    def _imu_feedback(self, now: float) -> Feedback | None:
        r = self.sensors.latest
        if r is None or r.wz.value is None:
            return None
        return Feedback(
            Velocity(0.0, r.wz.value),
            now,
            r.wz.deadline_steady,
            str(r.generation),
            self.sensors._evidence_sequence,
            True,
            "unified yaw estimate",
            observed_axes=(False, r.wz.observed),
            unified=True,
        )

    def _limited_ready(self, now: float, wheel: Feedback | None, imu: Feedback | None) -> bool:
        return bool(
            self.sensors.latest is not None
            and self.sensors.latest.healthy
            and self.sensors.tracking is not None
            and self.sensors.tracking.fresh(now)
        )

    def _snapshot(self, ref, body, wheel, imu, imu_current, now, perception, pose):
        mode = SourceMode(ref["mode"])
        if wheel is None:
            raise ValueError("all_relevant_motion_feedback_unavailable")
        epoch = wheel.epoch
        # Keep the historical interval field separate from a current control estimate.
        motion = BodyMotion(
            None,
            None,
            ref["start_ros"],
            ref["end_ros"],
            now,
            wheel.deadline_steady,
            epoch,
            True,
            "unified current control estimate; no truth bound",
        )
        p = self.profile
        response = self._response()
        degraded = mode is SourceMode.WHEEL_IMU_LIMITED
        cap = p.degraded_cap if degraded else p.cap
        target = self.target
        if target.source == "manual" and target.manual_linear is not None:
            # Manual degraded speed has one policy owner: the saved driving setting.
            # p.degraded_cap is the legacy navigation policy, not measured capability.
            if degraded:
                cap = min(p.cap, target.manual_linear.degraded_mps)
            target = replace(
                target,
                velocity=Velocity(
                    target.manual_linear.velocity(degraded=degraded, capability_mps=cap),
                    target.velocity.wz,
                ),
            )
        estimate = self.sensors.latest
        if estimate is not None and (estimate.vx.value is None or estimate.wz.value is None):
            target = replace(
                target,
                velocity=Velocity(
                    target.velocity.vx if estimate.vx.value is not None else 0.0,
                    target.velocity.wz if estimate.wz.value is not None else 0.0,
                ),
            )
        a = self.controller.trajectory.acceleration
        v = self.controller.trajectory.velocity
        if estimate is None or not estimate.healthy:
            raise ValueError("all_relevant_motion_feedback_unavailable")
        linear, angular = self.stopping.domains(
            estimate, v, response, linear_error=p.linear_error, yaw_error=p.yaw_error
        )
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
            self.stopping.reaction(self.acceleration_model.reaction(a), response),
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
            target,
            motion,
            mode,
            response,
            self._body_footprint(),
            perception,
            pose,
            "scan_reference",
            p.profile_id,
            p.profile_id,
            risk,
        )
        return ctx

    def _response(self):
        p = self.profile
        a = self.acceleration_model.upper
        return ResponseBounds(
            a.vx,
            p.decel,
            p.jerk,
            a.wz,
            p.yaw_decel,
            p.yaw_jerk,
            p.lag,
            self.model_deadline,
            p.profile_id,
        )

    def _record_stopping_output(self, output, now):
        p = self.profile
        self.stopping.commanded(
            output, now, self._response(), Velocity(p.drive_linear_overshoot, p.drive_yaw_overshoot)
        )

    def _diagnose_stopping(self, ctx, reason, evaluation):
        estimate = self.sensors.latest
        values = dict(
            basis="software risk assumptions; no measured guarantee",
            linear_interval_mps=ctx.envelope.linear_interval,
            yaw_interval_rps=ctx.envelope.yaw_interval,
            model_allowed_s=ctx.envelope.max_horizon_s,
            feedback_deadline_steady=ctx.motion.valid_until_steady,
            axis_age_s=[estimate.vx.observation_age_s, estimate.wz.observation_age_s],
            axis_reason=[estimate.vx.reason, estimate.wz.reason],
            constraint_reason=reason,
            residual_reason=[self.stopping.linear.reason, self.stopping.yaw.reason],
            residual_support_deadline=[
                value if math.isfinite(value) else None
                for value in (
                    self.stopping.linear.support_deadline,
                    self.stopping.yaw.support_deadline,
                )
            ],
        )
        self.diagnostics.sample("stopping.reason." + (reason or "permit"), 1.0)
        prediction = evaluation.stop_prediction
        if prediction is not None:
            self.diagnostics.sample("stopping.predicted_s", prediction.horizon_s)
            self.diagnostics.sample("stopping.linear_distance_m", prediction.linear.distance)
            self.diagnostics.sample("stopping.yaw_angle_rad", prediction.yaw.distance)
            values.update(
                predicted_stop_s=prediction.horizon_s,
                linear_stop_distance_m=prediction.linear.distance,
                yaw_stop_angle_rad=prediction.yaw.distance,
                model_horizon_exceeded=prediction.horizon_s > ctx.envelope.max_horizon_s,
            )
        self.diagnostics.summary("stopping", values)

    def _imu_state(self, packet, now):
        # //Compatible: retain the old raw-shaped independent yaw view for
        # existing controller diagnostics. Stopping consumes the fused estimate.
        estimate = self.sensors.latest
        axis = (
            None
            if estimate is None or estimate.independent_axes is None
            else estimate.independent_axes[1]
        )
        usable = axis is not None and axis.observed and now < axis.deadline_steady
        return (None if not usable else {"sample": {"values": [axis.value]}}, usable)

    def _stop_fallback(self, prepared, packet, now):
        # Recompute from original raw roots once at output time. A tick projection
        # may change its object identity; that is not a new sensor observation.
        ref, body, wheel = self._inputs(now)
        imu, imu_current = self._imu_state(packet, now)
        perception, pose = self._observation(now)
        ctx = self._snapshot(ref, body, wheel, imu, imu_current, now, perception, pose)
        if ctx.mode is not prepared.context.mode:
            # A prediction leaving its original ROS window cannot become a
            # different feedback mode just to salvage this same output.
            raise ValueError("reference_expired")
        error = self.engine._context_error(ctx, now)
        if error is not None:
            raise ValueError(error)
        return self.controller.prepare_stop(prepared, ctx, body, wheel, now)

    def _decision_output(self, decision, body, ctx, epoch, now):
        reason = decision.reason
        output = Velocity(0, 0)
        if decision.command is not None:
            output = decision.command
        # No candidate never holds the previous nonzero Twist. The fallback
        # is explicitly unverified stop/fault output, not a safe-stop claim.
        elif (
            self.startup.active
            and not self._learning_motion_emitted
            and self.target.source in {"idle", "startup_learning"}
        ):
            # Pre-movement sampling holds zero on blocked/unknown space. No
            # certificate is issued and obstacle checks still run on every tick.
            return output
        elif decision.emergency and reason in {
            "stop_envelope_blocked",
            "stop_response_failed",
            "infeasible_no_reverse",
        }:
            if reason == "stop_envelope_blocked" and self._startup_stop_uncertain(body, ctx):
                # Keep emergency/unverified STOP and zero output. Missing
                # initial body evidence is not an observed execution failure.
                if self.machine.state.mode is not MotionMode.FEEDBACK_STOP:
                    self._event(K.FEEDBACK_LOST, now)
            else:
                condition = (reason, epoch)
                if condition != self._last_fault_condition:
                    self._event(K.EXECUTION_FAULT, now)
                    self._last_fault_condition = condition
        elif not decision.emergency:
            self._last_fault_condition = None
        return output

    def tick(self) -> None:
        self._consume_ready_reference()
        now = time.monotonic()
        if self._last_tick_diagnostic is not None:
            self.diagnostics.duration("control_period", now - self._last_tick_diagnostic)
        self._last_tick_diagnostic = now
        event_now = now  # Runtime sampling remains t0; later events follow exit sampling.
        self.sequence += 1
        body = None
        wheel = None
        healthy = False
        reason = "not_ready"
        body_state = None
        deadline = None
        stamp = None
        output = Velocity(0, 0)
        prepared = None
        try:
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
            imu, imu_current = self._imu_state(packet, now)
            healthy = (
                self.sensors.latest is not None
                and self.sensors.latest.healthy
                and now < self.model_deadline
            )
            if body is not None:
                independent = self.sensors.independent
                body_state = (
                    None
                    if independent is None
                    else {
                        "average": {
                            "vx": independent.velocity.vx,
                            "vy": ref["average"]["vy"],
                            "wz": independent.velocity.wz,
                        },
                        "source": ref["mode"],
                        "computed_steady": independent.stamp_steady,
                        "valid_until_steady": independent.deadline_steady,
                    }
                )
                stamp, deadline = body.stamp_steady, body.deadline_steady
                self.machine.observe(
                    RecoveryObservation(
                        body.epoch,
                        body.sequence,
                        body.stamp_steady,
                        body.deadline_steady,
                        healthy,
                        self._stationary(body, wheel, event_now),
                        manual_rearm_ready=healthy,
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
            estimate = self.sensors.latest
            stationary = (
                estimate is not None
                and estimate.vx.observed
                and estimate.wz.observed
                and abs(estimate.vx.value or 0) < 0.01
                and abs(estimate.wz.value or 0) < 0.02
            )
            self.startup.advance(
                estimate, self.sensors.wheel_linear(now), now=now, stationary=stationary
            )
            self._startup_target(now)
            perception, pose = self._observation(now)
            self._startup_target(now)
            ctx = self._snapshot(ref, body, wheel, imu, imu_current, now, perception, pose)
            mode, epoch = ctx.mode, ctx.motion.source_epoch
            evaluation = self.engine.evaluate(ctx, now)
            permission = self.scheduler.update(ctx, now, evaluation=evaluation)
            if self._startup_permission_lost(permission, now):
                # Rebuild with provisional filtering revoked before proposing
                # any output. The current raw sequence is never re-stamped.
                perception, pose = self._observation(now)
                ctx = self._snapshot(ref, body, wheel, imu, imu_current, now, perception, pose)
                evaluation = self.engine.evaluate(ctx, now)
                permission = self.scheduler.update(ctx, now, evaluation=evaluation)
            self._diagnose_stopping(
                ctx, None if permission.brake is None else permission.brake.reason, evaluation
            )
            estop = self.machine.state.mode is MotionMode.ESTOP
            prepared = self.controller.propose_tick(
                ctx.target.velocity,
                ctx,
                permission,
                body,
                wheel,
                now,
                estop=estop,
                evaluation=evaluation,
                imu=imu_point,
                require_limited_rearm=False,
            )
            decision = prepared.decision
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
            output = self._decision_output(decision, body, ctx, epoch, event_now)
            reason = f"{mode.value}:{reason}"
            if output != Velocity(0, 0):
                # The decision already selects its applicable drive/brake deadlines.
                # Check actual output time without renewing that sampled evidence.
                emit_now = event_now = time.monotonic()
                if event_now < now:
                    raise ValueError("motion_clock_regressed")
                if (
                    decision.verified
                    and not decision.emergency
                    and not prepared.braking
                    and emit_now >= decision.evidence_deadline
                    and self.machine.state.mode
                    not in {MotionMode.ESTOP, MotionMode.EXECUTION_FAULT}
                ):
                    prepared = self._stop_fallback(prepared, packet, emit_now)
                    decision = prepared.decision
                    output = self._decision_output(
                        decision,
                        prepared.body,
                        prepared.context,
                        prepared.context.motion.source_epoch,
                        event_now,
                    )
                    reason = f"{prepared.context.mode.value}:{decision.reason}"
                    # One fallback only: expiry during its computation is a
                    # rejection, never another proposal or renewed evidence.
                    event_now = time.monotonic()
                    if event_now < emit_now:
                        raise ValueError("motion_clock_regressed")
                    emit_now = event_now
                    if not emit_now < decision.evidence_deadline:
                        raise ValueError("motion_expired_or_unqualified")
                if output != Velocity(0, 0) and (
                    not decision.verified or not emit_now < decision.evidence_deadline
                ):
                    raise ValueError("motion_expired_or_unqualified")
            if self.startup.active and output != Velocity(0, 0):
                self._learning_motion_emitted = True
            self.node._emit_motion_core(output, reason)
            self._record_stopping_output(output, time.monotonic())
            self.controller.commit(prepared)
            if self.target.source == "manual" and output != Velocity(0, 0):
                self._manual_motion_source = self.target.source_id
            if body is None and wheel is not None and imu_point is not None:
                ready = self._limited_ready(event_now, wheel, imu_point)
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
                    event_now,
                )
                if healthy:
                    self._event(K.FEEDBACK_HEALTHY, event_now)
        except (ValueError, TypeError, KeyError, AttributeError, OverflowError) as exc:
            if prepared is not None:
                self.controller.discard(prepared)
            self.controller.invalidate_limited_rearm()
            reason = str(exc)[:256] or type(exc).__name__
            if self.startup.state != "initializing":
                self._interrupt_learning(reason)
            self.node._emit_motion_core(Velocity(0, 0), reason)
            self._record_stopping_output(Velocity(0, 0), time.monotonic())
            if self._last_healthy:
                self._event(K.FEEDBACK_LOST, event_now)
                self._last_healthy = False
            healthy = False
        state = self.machine.state
        actions = list(state.allowed_actions)
        actual_stationary = healthy and self._stationary(body, wheel, event_now)
        if actual_stationary:
            self._startup_motion_unconfirmed = False
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
                "confirmed_estop_status": "software_latched"
                if state.mode is MotionMode.ESTOP
                else "clear",
                "allowed_actions": actions,
                "task_id": state.task_id,
                "output_generation": self.output_generation,
                "body_reference": body_state,
                "wheel_linear_mps": self.sensors.wheel_linear(now),
            }
        )
        msg = self._runtime_type()
        msg.data = runtime.model_dump_json()
        self._runtime.publish(msg)
        self.diagnostics.summary(
            "startup_learning",
            self.startup.summary(
                learned_count=0 if self.self_returns is None else len(self.self_returns.learned)
            ),
        )
        self.diagnostics.duration("control_compute", time.monotonic() - now)
        self.diagnostics.pump(time.monotonic())

    def close(self) -> None:
        self._closed = True
        self._interrupt_learning("shutdown")
        self.scheduler.stop()
        self.executor.shutdown(wait=False, cancel_futures=True)


def data_time(value: dict[str, Any] | None) -> float:
    return float(value["sampled_steady"]) if value is not None else 0.0
