"""Single control decision owner; private native transport publishes its output.

Deployment profiles supply response evidence; no ground truth enters this module.
"""

import math
import os
import time
from contextlib import ExitStack
from dataclasses import replace
from functools import partial
from pathlib import Path
from typing import Any
from uuid import uuid4

from senior_control.models import Pose2D, Velocity
from senior_control.motion.constraint_worker import ConstraintScheduler
from senior_control.motion.constraints import (
    ConstraintConfig,
    ConstraintEngine,
    ConstraintSnapshot,
    Footprint,
    Perception,
    RiskEnvelope,
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
from senior_control.motion.learning_record import LearningGeometry
from senior_control.motion.manual_target import ManualLinearTarget
from senior_control.motion.process_resources import ProcessWorker
from senior_control.motion.source_modes import SourceMode
from senior_control.motion.startup_learning import StartupLearning
from senior_control.motion.state_machine import MotionStateMachine
from senior_control.motion.stopping_input import StoppingRisk
from senior_control.protocol import MotionRuntimeState
from senior_control.runtime.diagnostics import QueueSnapshot, configure_diagnostics
from senior_control.runtime.learning_client import LearningClient
from senior_control.safety.self_filter import ScanObservation, filter_self
from senior_control.safety.self_learning import SelfReturnModel
from senior_control.safety.space_estimator import SpaceEstimation, SpaceObservation
from senior_control.sensors.policy import default_policy
from senior_control.vision.raw_capture import capture_from_message
from senior_robot.final_command_frame import PAYLOAD_FIELDS
from senior_robot.motion_sensor_runtime import MotionSensorRuntime


class MotionCoreOutput:
    def __init__(self, node: Any, profile: Any):
        self._resources = ExitStack()
        try:
            self._initialize(node, profile)
        except BaseException:
            self._resources.close()
            raise

    def _own(self, resource):
        self._resources.callback(resource.close)
        return resource

    def _initialize(self, node, profile):
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
        self._last_tick_diagnostic = None
        self.acceleration_model = profile.acceleration_model()
        self.stopping = StoppingRisk()
        self.boot = uuid4().hex
        claim = os.environ.get("SENIOR_STARTUP_LEARNING_CLAIM")
        self._learning_client = self._own(LearningClient(Path(claim))) if claim else None
        self.startup = StartupLearning(
            self._learning_client,
        )
        self.self_returns = None
        self._learning_scan_key = None
        self._filter_signature = None
        self._learning_geometry = None
        self.sequence = 0
        self.event_sequence = 0
        self.engine = ConstraintEngine(
            ConstraintConfig((1.0, 0.75, 0.5, 0.25), 0.15, 0.05, 256, 0.005)
        )
        self.worker = self._own(ProcessWorker())
        self.scheduler = ConstraintScheduler(self.engine, self.worker, worker_budget_ms=250)
        self.controller = MotionController(
            self.engine, profile.controller(), Velocity(0, 0), Velocity(0, 0)
        )
        self.machine = MotionStateMachine()
        self.target = MotionTarget(Velocity(0, 0), "idle", self.boot, None)
        self.output_generation = 0
        self._task_identity: tuple[str, int] | None = None
        # Proofs are in-flight only; explicit durable collection never recreates identity.
        self._task_retirements: dict[tuple[str, str, int, str], tuple[int, float, int]] = {}
        self._terminal_released: set[tuple[str, str, int, str]] = set()
        self._input_boot = ""
        self._input_floor = -1
        # One-way startup phase. Zero commands never establish body stillness;
        # only wheel/IMU stillness or an accepted nonzero source ends it.
        self._startup_motion_unconfirmed = True
        self._perception_key: tuple[Any, ...] | None = None
        self._perception_pose = None
        from senior_robot.motion_core_perception import ScanGeometry

        self.scan_geometry = self._own(ScanGeometry())
        self._scan_flight_started = None
        self._scan_job_status = {}
        self._first_fault_status = {}
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
        from senior_control.motion.diagnostic_pump import DiagnosticPump
        from senior_robot.runtime_reports import RuntimeReports

        self._runtime = self._own(RuntimeReports(node))
        self._diagnostic_pump = self._own(DiagnosticPump(self.diagnostics))
        self._runtime_type = String

    def _record_first_fault(self, now, detector, source=""):
        state = self.machine.state
        if state.fault_revision > 0 and not self._first_fault_status:
            self._first_fault_status = dict(
                sampled_steady=now,
                revision=state.fault_revision,
                detector=detector,
                state_reason=state.reason,
                source=source,
                target_source=self.target.source,
            )

    def _event(
        self, kind: K, now: float, source: str = "", revision: int = 0, *, detail: str = ""
    ) -> None:
        self.event_sequence += 1
        self.machine.reduce(
            MotionEvent(kind, source, revision, self.event_sequence), now_steady=now
        )
        self._record_first_fault(now, detail or kind.value, source)
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
        if reason == "estop" and (not msg.request_id or msg.source_kind == "navigation"):
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
                if not self.sensors.stationary(now):
                    return False
                evidence = self.sensors.recovery(now)
                if evidence is None:
                    return False
                self.machine.observe(evidence, now)
                self._record_first_fault(now, "recovery_observation")
                self._event(
                    K.ESTOP_RESET if reason == "estop_reset" else K.FAULT_RECOVER,
                    now,
                    msg.source_id,
                    int(msg.gesture),
                )
                if self.machine.state.mode in {MotionMode.ESTOP, MotionMode.EXECUTION_FAULT}:
                    return False
                return self.controller.reset_at_stillness(
                    self.sensors.raw_feedback("wheel", now),
                    self.sensors.raw_feedback("imu", now),
                    now,
                )
            except (ValueError, KeyError, TypeError):
                return False
        elif reason == "estop":
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

    def _retire_startup(self, now):
        if self.machine.state.task_id == self.boot:
            self._event(K.TASK_RETIRED, now, self.boot)

    def _startup_target(self, now):
        if self.startup.active:
            if self.target.source not in {"idle", "startup_learning"}:
                self._interrupt_learning("other_control_source")
                return
            if self.machine.state.mode in {MotionMode.ESTOP, MotionMode.EXECUTION_FAULT}:
                self.startup.hold_motion()
                self._retire_startup(now)
                if self.target.source == "startup_learning":
                    self.target = MotionTarget(Velocity(0, 0), "idle", self.boot, None)
                return
            elif self.startup.state not in {"initializing", "unconfirmed"}:
                if self.machine.state.task_id != self.boot:
                    self._event(K.TASK_START, now, self.boot)
                if self.machine.state.task_id != self.boot:
                    return  # Shared arbitration has not admitted this startup task.
                self.target = MotionTarget(
                    Velocity(self.startup.velocity, 0),
                    "startup_learning",
                    self.boot,
                    None,
                )
                return
        if self.target.source == "startup_learning":
            self._retire_startup(now)
            self.target = MotionTarget(Velocity(0, 0), "idle", self.boot, None)

    def _interrupt_learning(self, reason):
        was_active = self.startup.active
        self.startup.interrupt(reason)
        if was_active and self.self_returns is not None:
            self.self_returns.clear()
            self.space_estimator.invalidate("scan")
            self._perception_key = None

    def revoke(self, reason: str, *, now: float | None = None) -> None:
        if reason in {"estop", "fault", "estop_reset", "fault_recover", K.EXECUTION_FAULT.value}:
            self.startup.hold_motion()
        elif reason == K.FEEDBACK_LOST.value:
            self.startup.evidence_gap("motion_feedback_temporarily_unavailable")
        else:
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
        self.target = MotionTarget(Velocity(0, 0), "idle", self.boot, None)

    def _sync_learning_model(self, message):
        if self.startup.state == "disabled":
            return
        geometry = LearningGeometry(
            body_frame=str(self.node.get_parameter("body_frame").value),
            sensor_frame=message.frame_id,
            source_kind="scan",
            physical_polygon=tuple(self.node._profile().footprint),
            sensor_to_body=(message.sensor_x, message.sensor_y, message.sensor_yaw),
            angle_min=message.angle_min,
            angle_increment=message.angle_increment,
            count=len(message.ranges),
            range_min=message.range_min,
            range_max=message.range_max,
        )
        if self._learning_client is not None:
            self._learning_client.poll()
            self._learning_client.geometry(geometry)
        if geometry == self._learning_geometry:
            return
        if self._learning_geometry is not None:
            self.startup.hold_motion()
            self.startup.evidence_gap("scan_or_body_geometry_changed")
            self._perception_key = None
            self._filter_signature = None
            self.space_estimator.invalidate("scan")
        self._learning_geometry = geometry
        self.self_returns = SelfReturnModel(geometry.physical_polygon)

    def _filter_key(self):
        view = None if self._learning_client is None else self._learning_client.view
        return (
            self._learning_geometry,
            self.startup.provisional,
            None if view is None else view.catalog.model_id,
        )

    def _filtered_scan(self, raw):
        m = raw.message
        self._sync_learning_model(m)
        key = (m.source_epoch, m.sequence, self._filter_key())
        if getattr(self, "_latest_filtered_key", None) == key:
            return self._latest_unmasked, self._latest_masked
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
        unmasked = filter_self(scan, self.node._profile())
        masked = unmasked
        view = None if self._learning_client is None else self._learning_client.view
        if view is not None:
            masked = view.filter(unmasked)
        elif self.self_returns is not None and self.startup.provisional:
            masked = self.self_returns.filter(unmasked, provisional=True)
        self._latest_filtered_key = (m.source_epoch, m.sequence, self._filter_key())
        self._latest_unmasked, self._latest_masked = unmasked, masked
        return unmasked, masked

    def _scan_geometry_result(self, raw, key, deadline):
        from senior_robot.motion_core_perception import prepare_scan_geometry

        m = raw.message
        _, masked = self._filtered_scan(raw)
        key = (m.source_epoch, m.sequence, key[2], self._filter_key())
        policy = self.profile.perception_no_return_policy(m.frame_id, m.range_max)
        previous = self.scan_geometry.flight
        previous_key = self.scan_geometry.flight_key
        result = self.scan_geometry.update(
            key,
            partial(
                prepare_scan_geometry,
                masked,
                self.profile.sensor_error,
                policy,
                dict(
                    stamp_steady=raw.received,
                    deadline_steady=deadline,
                    source_epoch=m.source_epoch,
                    frame_id="scan_reference",
                    revision=m.sequence,
                    forecast_horizon_s=self.profile.horizon_s,
                    basis=self.profile.perception_basis + "; " + policy.value,
                ),
            ),
            snapshot=raw,
        )
        self._record_scan_job(previous, previous_key, key, deadline)
        return result

    def _record_scan_job(self, previous, previous_key, key, deadline):
        now = time.monotonic()
        geometry = self.scan_geometry
        flight, ready = geometry.flight, geometry.ready
        status = self._scan_job_status
        if previous is not None and flight is not previous:
            # update() has already consumed this completed Future; no extra poll.
            error = None if previous.cancelled() else previous.exception(timeout=0)
            status["last_error"] = "cancelled" if previous.cancelled() else str(error or "")[:256]
            status["completed"] = status.get("completed", 0) + 1
            status["last_scope_matches"] = geometry.scope(previous_key) == geometry.scope(key)
            status["last_flight_ms"] = (
                None
                if self._scan_flight_started is None
                else (now - self._scan_flight_started) * 1000
            )
        if flight is not previous:
            self._scan_flight_started = now if flight is not None else None
        process = getattr(geometry.worker, "process", None)
        status.update(
            sampled_steady=now,
            worker_pid=None if process is None else process.pid,
            worker_alive=None if process is None else process.is_alive(),
            requested_revision=key[1],
            requested_alignment=key[2],
            flight_revision=None if flight is None else geometry.flight_key[1],
            flight_scope_matches=flight is not None
            and geometry.scope(geometry.flight_key) == geometry.scope(key),
            flight_waiting_mailbox=flight is not None
            and getattr(flight, "_pending_task", None) is not None,
            flight_age_ms=None
            if self._scan_flight_started is None
            else (now - self._scan_flight_started) * 1000,
            ready_revision=None if ready is None else ready.key[1],
            ready_deadline_steady=None if ready is None else ready.perception.deadline_steady,
            ready_remaining_ms=None
            if ready is None
            else (ready.perception.deadline_steady - now) * 1000,
            requested_remaining_ms=(deadline - now) * 1000,
        )

    def prefetch_scan(self):
        """Feed geometry between control slots; never wait for a Future.

        This avoids a mandatory one-control-period dispatch lag. The control
        slot fences source/frame/filter changes and tightens an older completed
        scan against the newest observation, without renewing its deadline.
        """
        # Reception/geometry prefetch is independent of optional startup learning.
        if not self.node._scans:
            return
        motion = self.sensors.latest
        if motion is None:
            return
        raw = self.node._scans[-1]
        m = raw.message
        signature = self._filter_key()
        alignment = f"{motion.generation}:{self.sensors.pose_estimator.frame_generation}"
        key = (m.source_epoch, m.sequence, alignment, signature)
        if key == self._perception_key:
            return
        now = time.monotonic()
        try:
            age = default_policy().age(
                measured_steady=raw.measurement_steady,
                received_steady=raw.received,
                now_steady=now,
            )
            if age.band != "invalid":
                self._scan_geometry_result(raw, key, age.deadline_steady)
        except (ValueError, TypeError, KeyError):
            return  # Tick owns evidence errors and output selection.

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
        root = Origin(
            "scan",
            m.source_epoch,
            m.sequence,
            capture_from_message(raw.message.raw_capture).time,
        )
        raw_key = (root.epoch, root.sequence)
        try:
            age = default_policy().age(
                measured_steady=root.measurement_steady,
                received_steady=root.received_steady,
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
        if motion is None or motion.measurement_host_steady is None or motion.stamp_ns is None:
            raise ValueError("all_relevant_motion_feedback_unavailable")
        current = self.sensors.pose_estimator.at(
            ros,
            measurement_steady=motion.measurement_host_steady,
            stamp_ns=motion.stamp_ns,
            now_ros=ros,
            now_steady=now,
            motion=motion,
        )
        alignment_epoch = f"{motion.generation}:{self.sensors.pose_estimator.frame_generation}"
        self.space_estimator.reset_frame(alignment_epoch)
        key = (m.source_epoch, m.sequence, alignment_epoch, self._filter_key())
        result = self._scan_geometry_result(raw, key, age.deadline_steady)
        signature = self._filter_key()
        if self._filter_signature is not None and signature != self._filter_signature:
            self.space_estimator.invalidate("scan")
        self._filter_signature = signature
        if result is None:
            raise ValueError("scan_geometry_pending")
        selected = result.snapshot
        selected_message = selected.message
        selected_age = default_policy().age(
            measured_steady=selected.measurement_steady,
            received_steady=selected.received,
            now_steady=now,
        )
        if selected_age.band == "invalid" or now >= result.perception.deadline_steady:
            raise ValueError("scan_geometry_pending")
        if self.startup.state != "disabled" and self.self_returns is not None:
            if self._learning_scan_key != raw_key:
                self._learning_scan_key = raw_key
                unmasked, _ = self._filtered_scan(raw)
                self.startup.frame(
                    self.self_returns, unmasked, capture=capture_from_message(m.raw_capture)
                )
                if signature != self._filter_key():
                    raise ValueError("scan_geometry_pending")
        old = self._perception_pose if result.key == self._perception_key else None
        if old is None:
            old = self.sensors.pose_estimator.at(
                selected.stamp,
                measurement_steady=selected.measurement_steady,
                stamp_ns=capture_from_message(selected.message.raw_capture).stamp_ns,
                now_ros=ros,
                now_steady=now,
                motion=motion,
            )
        pose = compose(inverse(old.pose), current.pose)
        if result.key != self._perception_key:
            self._perception = result.perception
            selected_root = Origin(
                "scan",
                selected_message.source_epoch,
                selected_message.sequence,
                capture_from_message(selected.message.raw_capture).time,
            )
            self.space_estimator.ingest(
                SpaceObservation(
                    "scan",
                    self._perception.occupied,
                    self._perception.free_space,
                    (selected_root,),
                    error_m=old.error_m,
                    software_error_m=self.profile.position_error
                    + old.prediction_error.software_position_m,
                    frame=self.sensors.pose_estimator.frame,
                    pose_in_frame=old.pose,
                    frame_epoch=alignment_epoch,
                )
            )
            self._perception_pose, self._perception_key = old, result.key
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
        from shapely.affinity import rotate, translate
        from shapely.geometry import GeometryCollection

        from senior_robot.motion_core_perception import fast_scan_guard

        latest_pose = self.sensors.pose_estimator.at(
            raw.stamp,
            measurement_steady=raw.measurement_steady,
            stamp_ns=capture_from_message(raw.message.raw_capture).stamp_ns,
            now_ros=ros,
            now_steady=now,
            motion=motion,
        )
        latest_error = self.sensors.pose_estimator.relative_error(latest_pose, current)
        guard_error = (
            latest_error.software_position_m
            + 2 * radius * math.sin(min(math.pi, latest_error.software_yaw_rad) / 2)
            if latest_error.position_m is None or latest_error.yaw_rad is None
            else latest_error.position_m
            + 2 * radius * math.sin(min(math.pi, latest_error.yaw_rad) / 2)
        )
        _, masked = self._filtered_scan(raw)
        policy = self.profile.perception_no_return_policy(m.frame_id, m.range_max)
        free, occupied = fast_scan_guard(
            masked, policy, error_m=self.profile.sensor_error + guard_error
        )
        transform = compose(inverse(old.pose), latest_pose.pose)

        def aligned(geometry):
            return translate(
                rotate(geometry, transform.yaw, origin=(0, 0), use_radians=True),
                transform.x,
                transform.y,
            )

        return replace(
            self._perception,
            occupied=GeometryCollection((space.occupied, aligned(occupied))),
            free_space=space.free.intersection(aligned(free)),
            deadline_steady=min(space.deadline_steady, age.deadline_steady),
            revision=m.sequence,
        ), pose

    def _inputs(self, now: float) -> tuple[Any, Any, Any]:
        if not hasattr(self.sensors, "input_reference"):
            raise ValueError("input_snapshot_missing")
        ref, tracking, secondary = self.sensors.input_reference
        # Remember motion even if perception later fails before _snapshot.
        self.stopping.domains(
            self.sensors.latest,
            self.controller.trajectory.velocity,
            self._response(),
            linear_error=self.profile.linear_error,
            yaw_error=self.profile.yaw_error,
        )
        return ref, tracking, secondary

    def _startup_visibility(self):
        visibility = None
        if self.startup.active and self.target.source in {"idle", "startup_learning"}:
            from senior_control.motion.constraints import NoReturnPolicy
            from senior_control.safety.startup_visibility import startup_visibility

            raw = self.node._scans[-1]
            _, masked = self._filtered_scan(raw)
            visibility = startup_visibility(
                masked,
                front_stop=self.profile.startup_learning_front_occlusion_stop,
                rear_stop=self.profile.startup_learning_rear_occlusion_stop,
                no_return_clear=self.profile.perception_no_return_policy(
                    raw.message.frame_id, raw.message.range_max
                )
                is NoReturnPolicy.VALIDATED_FREE_TO_RANGE,
            )
        return visibility

    def _snapshot(self, ref, wheel, now, perception, pose, *, visibility=None):
        mode = SourceMode(ref["mode"])
        if wheel is None:
            raise ValueError("all_relevant_motion_feedback_unavailable")
        epoch = wheel.epoch
        # Keep the historical interval field separate from a current control estimate.
        motion = BodyMotion(
            None,
            None,
            ref["start_steady"],
            ref["end_steady"],
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
            p.horizon_s,
            linear,
            angular,
            self.stopping.reaction(self.acceleration_model.reaction(a), response),
            p.position_error,
            p.orientation_error_rad,
            p.lateral_drift_mps,
            cap,
            p.yaw_cap,
            False,
            lateral_stop_s=None if p.lateral_stop_ms is None else p.lateral_stop_ms / 1000,
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
            startup_visibility=visibility,
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
            p.lag_ms / 1000,
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
        return {
            key: values.get(key)
            for key in (
                "linear_interval_mps",
                "yaw_interval_rps",
                "linear_stop_distance_m",
                "yaw_stop_angle_rad",
            )
        }

    def _stop_fallback(self, prepared, now):
        # Recompute from original raw roots once at output time. A tick projection
        # may change its object identity; that is not a new sensor observation.
        ref, body, wheel = self._inputs(now)
        perception, pose = self._observation(now)
        ctx = self._snapshot(
            ref, wheel, now, perception, pose, visibility=self._startup_visibility()
        )
        if ctx.mode is not prepared.context.mode:
            # A prediction leaving its original ROS window cannot become a
            # different feedback mode just to salvage this same output.
            raise ValueError("reference_expired")
        error = self.engine._context_error(ctx, now)
        if error is not None:
            raise ValueError(error)
        return self.controller.prepare_stop(prepared, ctx, body, wheel, now)

    def _first_fault_scan_evidence(self, ctx):
        """Report actual latest hit coordinates, not an inferred collision witness."""
        from shapely.affinity import rotate, translate
        from shapely.geometry import Point, Polygon
        from shapely.ops import nearest_points

        masked = self._latest_masked
        body = Polygon(ctx.footprint.physical_vertices)
        hits = [ray for ray in masked.external_hits if ray.endpoint is not None]
        points = [(ray, Point(ray.endpoint)) for ray in hits]
        closest = min(
            points,
            key=lambda row: (body.distance(row[1]), math.hypot(*row[0].endpoint)),
            default=None,
        )
        view = None if self._learning_client is None else self._learning_client.view
        evidence = dict(
            latest_scan_source=self._latest_filtered_key[:2],
            geometry_scan_source=None if self._perception_key is None else self._perception_key[:2],
            learning_state=self.startup.state,
            interior_prior_active=self.startup.provisional and view is None,
            learned_filter_active=view is not None,
            physical_body_xy=ctx.footprint.physical_vertices,
            sensor_to_body=(
                masked.scan.sensor_to_body.x,
                masked.scan.sensor_to_body.y,
                masked.scan.sensor_to_body.yaw,
            ),
            latest_external_hits=len(hits),
            latest_self_hits=len(masked.occluded_regions),
            latest_interior_hits=sum(body.contains(point) for _, point in points),
            closest_latest_external_hit=None,
        )
        # _observation keeps retained scan and latest guard as separate parts.
        # These points include their existing error expansion; they are not raw hits.
        world_body = translate(
            rotate(body, ctx.pose.yaw, origin=(0, 0), use_radians=True), ctx.pose.x, ctx.pose.y
        )
        cosine, sine = math.cos(ctx.pose.yaw), math.sin(ctx.pose.yaw)
        for source, occupied in zip(
            ("retained_scan", "latest_scan_guard"), ctx.perception.occupied.geoms, strict=True
        ):
            value = None
            if not occupied.is_empty:
                point = nearest_points(world_body, occupied)[1]
                dx, dy = point.x - ctx.pose.x, point.y - ctx.pose.y
                value = dict(
                    body_xy=(cosine * dx + sine * dy, -sine * dx + cosine * dy),
                    physical_body_clearance_m=world_body.distance(occupied),
                )
            evidence[source + "_expanded_occupied"] = value
        if closest is not None:
            ray, point = closest
            evidence["closest_latest_external_hit"] = dict(
                ray_index=ray.index,
                body_xy=ray.endpoint,
                body_bearing_rad=math.atan2(point.y, point.x),
                sensor_range_m=math.dist(ray.origin, ray.endpoint),
                physical_body_clearance_m=body.distance(point),
                inside_physical_body=body.contains(point),
            )
        return evidence

    def _decision_output(self, decision, body, ctx, epoch, now, *, stopping_context=None):
        reason = decision.reason
        output = Velocity(0, 0)
        if decision.command is not None:
            output = decision.command
        # No candidate never holds the previous nonzero Twist. The fallback
        # is explicitly unverified stop/fault output, not a safe-stop claim.
        elif (
            self.startup.active
            and not self.startup.motion_started
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
                    first_fault = not self._first_fault_status
                    scan_evidence = self._first_fault_scan_evidence(ctx) if first_fault else None
                    self._event(K.EXECUTION_FAULT, now, detail=reason)
                    if first_fault:
                        self._first_fault_status.update(scan_evidence)
                        if stopping_context is not None:
                            self._first_fault_status.update(stopping_context)
                    self._last_fault_condition = condition
        elif not decision.emergency:
            self._last_fault_condition = None
        return output

    def tick(self) -> None:
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
            if (
                self.target.deadline_steady is not None
                and now >= self.target.deadline_steady
                and self.target.source != "idle"
            ):
                self.scheduler.stop()
                self._event(
                    K.MANUAL_EXPIRED if self.target.source == "manual" else K.TASK_PAUSE,
                    now,
                    self.target.source_id.rsplit(":", 1)[0],
                )
                self.target = MotionTarget(Velocity(0, 0), "idle", self.boot, None)
            ref, body, wheel = self._inputs(now)
            healthy = self.sensors.latest is not None and self.sensors.latest.healthy
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
            recovery = self.sensors.recovery(now)
            if recovery is not None:
                self.machine.observe(recovery, now)
                self._record_first_fault(now, "recovery_observation")
                stamp, deadline = recovery.stamp_steady, recovery.deadline_steady
            if healthy:
                if body is None and wheel is not None:
                    stamp, deadline = wheel.stamp_steady, wheel.deadline_steady
                if body is not None:
                    self._event(K.FEEDBACK_HEALTHY, now)
            elif self._last_healthy:
                self._event(K.FEEDBACK_LOST, now)
            self._last_healthy = healthy
            stationary = self.sensors.stationary(now)
            self.startup.advance(self.sensors.wheel_linear(now), now=now, stationary=stationary)
            if self.startup.deadline is not None and now >= self.startup.deadline and stationary:
                self.controller.reset_at_stillness(
                    self.sensors.raw_feedback("wheel", now),
                    self.sensors.raw_feedback("imu", now),
                    now,
                )
            self._startup_target(now)
            perception, pose = self._observation(now)
            self._startup_target(now)
            ctx = self._snapshot(
                ref, wheel, now, perception, pose, visibility=self._startup_visibility()
            )
            mode, epoch = ctx.mode, ctx.motion.source_epoch
            evaluation = self.engine.evaluate(ctx, now)
            permission = self.scheduler.update(ctx, now, evaluation=evaluation)
            stopping_context = self._diagnose_stopping(
                ctx, None if permission.brake is None else permission.brake.reason, evaluation
            )
            estop = self.machine.state.mode in {MotionMode.ESTOP, MotionMode.EXECUTION_FAULT}
            prepared = self.controller.propose_tick(
                ctx.target.velocity,
                ctx,
                permission,
                body,
                wheel,
                now,
                estop=estop,
                evaluation=evaluation,
                imu=self.sensors.raw_feedback("imu", now),
                still_wheel=self.sensors.raw_feedback("wheel", now),
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
            output = self._decision_output(
                decision, body, ctx, epoch, event_now, stopping_context=stopping_context
            )
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
                    prepared = self._stop_fallback(prepared, emit_now)
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
            output_now = time.monotonic()
            output_deadline = decision.evidence_deadline
            learning_output = self.target.source == "startup_learning"
            if (
                self.target.source == "idle"
                and self.startup.action_tail_active
                and self.startup.deadline is not None
                and output_now >= self.startup.deadline
                and output != Velocity(0, 0)
            ):
                raise ValueError("learning_action_window_finished")
            if learning_output and output != Velocity(0, 0):
                if not self.startup.action_permitted and not self.startup.prepare_action():
                    raise ValueError("learning_action_not_reserved")
                end = self.startup.deadline
                if end is not None and output_now >= end:
                    raise ValueError("learning_action_window_finished")
                output_deadline = min(output_deadline, output_now + 3.0 if end is None else end)
            self.node._emit_motion_core(output, reason, output_deadline, computed_steady=now)
            if learning_output and output != Velocity(0, 0):
                self.startup.motion_emitted(output_now, deadline=output_deadline)
            else:
                self.startup.output_stopped(output_now)
            self._record_stopping_output(output, time.monotonic())
            self.controller.commit(prepared)
        except (ValueError, TypeError, KeyError, AttributeError, OverflowError) as exc:
            if prepared is not None:
                self.controller.discard(prepared)
            reason = str(exc)[:256] or type(exc).__name__
            if reason not in {
                "scan_geometry_pending",
                "learning_action_window_finished",
                "learning_action_not_reserved",
            }:
                self.startup.evidence_gap(reason)
            self.node._emit_motion_core(Velocity(0, 0), reason, computed_steady=now)
            self.startup.output_stopped(time.monotonic())
            self._record_stopping_output(Velocity(0, 0), time.monotonic())
            if reason not in {
                "scan_geometry_pending",
                "learning_action_window_finished",
                "learning_action_not_reserved",
            }:
                if self._last_healthy:
                    self._event(K.FEEDBACK_LOST, event_now)
                    self._last_healthy = False
                healthy = False
        state = self.machine.state
        actions = list(state.allowed_actions)
        actual_stationary = self.sensors.stationary(event_now)
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
                learned_count=(
                    0
                    if self._learning_client is None or self._learning_client.view is None
                    else len(self._learning_client.view.catalog.anchors)
                )
            ),
        )
        self.diagnostics.duration("control_compute", time.monotonic() - now)

    def report_status(self, value):
        # Existing bounded status transport only; never changes a control decision.
        values = dict(value["values"])
        for prefix, status in (
            ("scan_geometry", self._scan_job_status),
            ("first_fault", self._first_fault_status),
        ):
            values.update({prefix + "." + key: str(item) for key, item in status.items()})
        self._runtime.status({**value, "values": values})

    def close(self) -> None:
        self._closed = True
        self._interrupt_learning("shutdown")
        self.scheduler.stop()
        self._resources.close()
