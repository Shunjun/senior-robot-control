"""Single motion owner: authenticated targets, directional distance caps and output.

Wheel/independent observations remain reports, never admission certificates.
The native transport remains the sole standard /cmd_vel Twist publisher.
"""

import math
import os
import time
from contextlib import ExitStack
from pathlib import Path
from typing import Any
from uuid import uuid4

from senior_control.models import Velocity
from senior_control.motion.contracts import MotionEvent, MotionMode, MotionTarget
from senior_control.motion.contracts import MotionEventKind as K
from senior_control.motion.learning_record import LearningGeometry
from senior_control.motion.manual_target import ManualLinearTarget
from senior_control.motion.startup_learning import StartupLearning
from senior_control.motion.state_machine import MotionStateMachine
from senior_control.protocol import MotionRuntimeState
from senior_control.runtime.diagnostics import configure_diagnostics
from senior_control.runtime.learning_client import LearningClient
from senior_control.safety.self_learning import SelfReturnModel
from senior_control.safety.startup_visibility import startup_visibility
from senior_control.sensors.policy import default_policy
from senior_control.vision.raw_capture import capture_from_message
from senior_robot.final_command_frame import PAYLOAD_FIELDS
from senior_robot.motion_core_perception import (
    avoidance_observation,
    directional_clearance,
    limit_velocity,
    scan_observation,
)
from senior_robot.motion_sensor_runtime import MotionSensorRuntime


class MotionCoreOutput:
    def __init__(self, node: Any, profile: Any):
        from std_msgs.msg import String

        from senior_control.motion.diagnostic_pump import DiagnosticPump
        from senior_robot.runtime_reports import RuntimeReports

        self._resources = ExitStack()
        try:
            self.node, self.profile = node, profile
            self.diagnostics = configure_diagnostics(profile.diagnostics)
            self.sensors = MotionSensorRuntime(profile, self.diagnostics)
            self.boot = uuid4().hex
            self.sequence = self.event_sequence = self.output_generation = 0
            self.machine = MotionStateMachine()
            self.target = MotionTarget(Velocity(0, 0), "idle", self.boot, None)
            self._task_identity: tuple[str, int] | None = None
            self._task_retirements: dict[tuple[str, str, int, str], tuple[int, float, int]] = {}
            self._terminal_released: set[tuple[str, str, int, str]] = set()
            self._input_boot, self._input_floor = "", -1
            self._barrier_key = self._barrier_payload = None
            self._barrier_complete = False
            self._closed = False
            self._launch_recovered = os.environ.get("SENIOR_LAUNCH_RECOVERED") == "1"
            self._recovery_pending = self._launch_recovered
            self._last_tick = None
            self._output = Velocity(0, 0)
            self._runtime = RuntimeReports(node)
            self._resources.callback(self._runtime.close)
            self._diagnostic_pump = DiagnosticPump(self.diagnostics)
            self._resources.callback(self._diagnostic_pump.close)
            self._runtime_type = String
            claim = os.environ.get("SENIOR_STARTUP_LEARNING_CLAIM")
            self._learning_client = None
            if claim:
                if self._launch_recovered:
                    self._learning_client = LearningClient(Path(claim), read_only=True)
                else:
                    self._learning_client = LearningClient(Path(claim))
            if self._learning_client is not None:
                self._resources.callback(self._learning_client.close)
            self.startup = StartupLearning(
                None if self._launch_recovered else self._learning_client
            )
            self.self_returns = None
            self._learning_geometry = None
            self._scan_result_key = None
            self._scan_results = [None, None]
            self._retired_learning_owner = False
            self._old_learning_owner = None
        except BaseException:
            self._resources.close()
            raise

    def _event(self, kind, now, source="", revision=0):
        self.event_sequence += 1
        self.machine.reduce(
            MotionEvent(kind, source, revision, self.event_sequence), now_steady=now
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
        handoff = (
            msg.source_kind == "idle"
            and msg.source_id == self.boot
            and (
                (reason == "prepared" and msg.request_id == self.boot and msg.source_epoch == 0)
                or (reason == "estop" and bool(msg.request_id) and msg.source_epoch > 0)
            )
        )
        # A fresh output cannot identify old intents from timestamps alone.
        # Only this random boot's causal zero echo finishes recovery. An estop
        # may always lock first, but an unrelated estop does not finish handoff.
        if self._recovery_pending and not handoff and reason != "estop":
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
                if handoff:
                    self._recovery_pending = False
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
        self._interrupt_learning("control_takeover", now)
        source_id = f"{msg.source_id}:{msg.source_epoch}"
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

    def _apply_close(self, msg, reason, now):
        if reason == "task_done" and (
            msg.source_kind != "navigation"
            or self.machine.state.task_id not in {None, msg.source_id}
        ):
            return False
        if (
            reason == "prepared"
            and msg.source_kind == "idle"
            and self.target.source == "startup_learning"
        ):
            # Initial owner heartbeat is not an operator takeover of automatic learning.
            return True
        if reason == "estop" or msg.source_kind in {"manual", "navigation"}:
            self._interrupt_learning("estop" if reason == "estop" else "control_takeover", now)
        self.revoke(reason, now=now)
        if reason == "estop":
            if msg.source_kind == "idle" and msg.source_id == self.boot and msg.source_epoch > 0:
                # Private recovery meaning ONLY for this boot-matched idle
                # estop: source_epoch preserves the original latch revision.
                # request_id remains the operator's original request identity;
                # gesture stays zero, as required by the native v3 guard.
                return self.machine.restore_software_estop(int(msg.source_epoch), now_steady=now)
            self._event(K.SOFTWARE_ESTOP, now, msg.source_id)
        elif reason in {"estop_reset", "fault_recover"}:
            # Only an explicit revision-matched operator request clears a latch.
            self._event(
                K.ESTOP_RESET if reason == "estop_reset" else K.FAULT_RECOVER,
                now,
                msg.source_id,
                int(msg.gesture),
            )
            return self.machine.state.mode not in {MotionMode.ESTOP, MotionMode.EXECUTION_FAULT}
        elif msg.source_kind == "manual":
            self._event(
                K.MANUAL_EXPIRED if reason == "expiry" else K.MANUAL_RELEASE, now, msg.source_id
            )
        elif msg.source_kind == "navigation":
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

    def _interrupt_learning(self, reason, now):
        self.startup._last_now = now
        self.startup.interrupt(reason)
        if self.machine.state.task_id == self.boot:
            self._event(K.TASK_RETIRED, now, self.boot)
        if self.target.source == "startup_learning":
            self.target = MotionTarget(Velocity(0, 0), "idle", self.boot, None)
            self._output = Velocity(0, 0)

    def _learning_observation(self, now):
        client = self._learning_client
        if client is None:
            return None
        client.poll()
        record = client.record
        if self._launch_recovered:
            if (
                self._old_learning_owner is None
                and record is not None
                and record.expected_output_boot_id != self.boot
            ):
                # Pin the observed old assignment before reporting its loss. A
                # later State reply/new manual attempt cannot retarget this fact.
                self._old_learning_owner = record
            if (
                not self._recovery_pending
                and not self._retired_learning_owner
                and self._old_learning_owner is not None
            ):
                self._retired_learning_owner = client.retire_motion_owner(
                    self._old_learning_owner, self.boot
                )
        if (
            self._launch_recovered
            and record is not None
            and record.expected_output_boot_id != self.boot
        ):
            # Preserve launch history and initial read-only observation; never
            # adopt a consumed predecessor or reopen automatic startup.
            pass
        elif record is not None and getattr(self, "_learning_attempt", None) != record.attempt_id:
            needs_admission = (
                record.trigger == "manual"
                and not record.action.consumed
                and record.phase not in {"completed", "failed", "unconfirmed"}
                and record.capture_summary.state != "complete"
            )
            if needs_admission:
                # An observed candidate is not an executable/adopted session.
                self.startup = StartupLearning(None)
                proposal = client.proposal
                permitted = (
                    not self._recovery_pending
                    and proposal is not None
                    and proposal.output_boot_id == self.boot
                    and time.monotonic() < proposal.deadline_steady
                    and proposal.owner_boot_id == self._input_boot
                    and proposal.generation == self.output_generation
                    and self.target.source == "idle"
                    and self.sensors.stationary(now)
                    and self.machine.state.mode
                    not in {MotionMode.ESTOP, MotionMode.EXECUTION_FAULT}
                )
                if permitted:
                    if not client.bind_attempt(
                        record.attempt_id, self.boot, self.output_generation
                    ):
                        return None
                    self._manual_learning_request = proposal
                    client.accept_attempt(True, "accepted")
                else:
                    if not client.decline_attempt(record, self.boot, "control_source_changed"):
                        return None
                    self._learning_attempt = record.attempt_id
                    return None
            self.startup = StartupLearning(client)
            self._learning_attempt = record.attempt_id
        self.startup.advance(
            self.sensors.wheel_linear(now), now=now, stationary=self.sensors.stationary(now)
        )
        if not self.startup.active and client.view is None:
            return None
        if not self.node._scans:
            self.startup.hold_motion("learning_scan_unavailable")
            return None
        raw = self.node._scans[-1]
        age = default_policy().age(
            measured_steady=raw.measurement_steady, received_steady=raw.received, now_steady=now
        )
        if age.band == "invalid":
            self.startup.hold_motion("scan_temporarily_unavailable")
            return None
        return self._filtered_scan(raw)

    def _scan_key(self, raw):
        """Value identity survives snapshot unpickling; missing metadata only skips reuse."""
        try:
            m = raw.message
            ranges = tuple(m.ranges)
            return (
                capture_from_message(m.raw_capture),
                (m.source_epoch, m.sequence, m.raw_source),
                (raw.stamp, raw.received, raw.received_ros, raw.measurement_steady),
                (self.node._body, m.frame_id),
                (m.angle_min, m.angle_increment, m.range_min, m.range_max),
                (m.sensor_x, m.sensor_y, m.sensor_yaw, m.transform_stamp),
                (self.profile.length, self.profile.width, len(ranges)),
                ranges,
            )
        except (AttributeError, TypeError, ValueError, OverflowError):
            return None

    def _filtered_scan(self, raw, *, avoidance=False):
        key = self._scan_key(raw)
        if key is None or key != self._scan_result_key:
            self._scan_result_key = key
            self._scan_results = [None, None]
        slot = int(avoidance)
        composition = tuple(raw.predictions) if avoidance else ()
        entry = self._scan_results[slot]
        self._scan_results[slot] = None  # Never retain a partial result after an exception.
        if entry is not None and entry[0] == composition:
            _, observation, previous_filter, result = entry
        else:
            decode = avoidance_observation if avoidance else scan_observation
            observation = decode(raw, self.node._body)
            previous_filter = result = None
        client = self._learning_client
        view = None if client is None else client.view
        if not avoidance and client is not None and view is None:
            # Preserve decoding before geometry reports/holds, even on a cold cache.
            m = raw.message
            x, y = self.profile.length / 2, self.profile.width / 2
            geometry = LearningGeometry(
                body_frame=self.node._body,
                sensor_frame=m.frame_id,
                physical_polygon=((-x, -y), (x, -y), (x, y), (-x, y)),
                sensor_to_body=(m.sensor_x, m.sensor_y, m.sensor_yaw),
                angle_min=m.angle_min,
                angle_increment=m.angle_increment,
                count=len(m.ranges),
                range_min=m.range_min,
                range_max=m.range_max,
            )
            client.geometry(geometry)
            if geometry != self._learning_geometry:
                if self._learning_geometry is not None:
                    self.startup.hold_motion("scan_or_body_geometry_changed")
                self._learning_geometry = geometry
                self.self_returns = SelfReturnModel(geometry.physical_polygon)
        filter_key = (
            self.startup.provisional,
            self._learning_geometry,
            None
            if view is None
            else (client.active_revision, view.catalog.model_id, view.geometry),
        )
        if previous_filter != filter_key:
            if view is not None:
                result = view.filter(observation)
            elif self.self_returns is not None:
                result = self.self_returns.filter(observation, provisional=self.startup.provisional)
            else:
                result = observation
        if key is not None:
            self._scan_results[slot] = (composition, observation, filter_key, result)
        return result

    def _startup_target(self, now, observation):
        if self._launch_recovered and getattr(self, "_manual_learning_request", None) is None:
            return
        if self.target.source not in {"idle", "startup_learning"}:
            return
        if not self.startup.active or (
            self.startup.deadline is not None and now >= self.startup.deadline
        ):
            if self.target.source == "startup_learning":
                if self.machine.state.task_id == self.boot:
                    self._event(K.TASK_RETIRED, now, self.boot)
                self.target = MotionTarget(Velocity(0, 0), "idle", self.boot, None)
                self._output = Velocity(0, 0)
            return
        if self.machine.state.mode in {MotionMode.ESTOP, MotionMode.EXECUTION_FAULT}:
            return
        if not self.node._command_topology_ready():
            if self.startup.motion_started:
                self._interrupt_learning("command_topology_unqualified", now)
            return
        proposal = getattr(self, "_manual_learning_request", None)
        if (
            proposal is not None
            and not self.startup.motion_started
            and (
                now >= proposal.deadline_steady
                or proposal.output_boot_id != self.boot
                or proposal.owner_boot_id != self._input_boot
                or proposal.generation != self.output_generation
            )
        ):
            self._interrupt_learning("control_source_changed", now)
            return
        velocity = self.startup.velocity
        reason = self.startup.reason
        if observation is None:
            velocity, reason = 0.0, "learning_scan_unavailable"
        elif velocity:
            visibility = startup_visibility(
                observation,
                front_stop=self.profile.startup_learning_front_occlusion_stop,
                rear_stop=self.profile.startup_learning_rear_occlusion_stop,
            )
            if visibility.blocked_reason is not None:
                velocity, reason = 0.0, visibility.blocked_reason
                self.startup.hold_motion(reason)
                if not self.startup.active:
                    self._interrupt_learning(self.startup.reason, now)
                    return
        if self.machine.state.task_id != self.boot:
            self._event(K.TASK_START, now, self.boot)
        if self.machine.state.task_id == self.boot:
            self.target = MotionTarget(Velocity(velocity, 0), "startup_learning", self.boot, None)
            self.startup.reason = reason

    def revoke(self, reason: str, *, now: float | None = None) -> None:
        now = time.monotonic() if now is None else now
        if reason != "prepared" and not (
            reason == "command_topology_unqualified" and not self.startup.motion_started
        ):
            self._interrupt_learning(reason, now)
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
        if self.target.source != "idle":
            self._event(
                K.MANUAL_EXPIRED if self.target.source == "manual" else K.TASK_PAUSE,
                now,
                self.target.source_id.rsplit(":", 1)[0],
            )
        self.target = MotionTarget(Velocity(0, 0), "idle", self.boot, None)
        self._output = Velocity(0, 0)

    def tick(self) -> None:
        now = time.monotonic()
        self.sequence += 1
        dt = 0.0 if self._last_tick is None else max(0.0, min(now - self._last_tick, 0.1))
        self._last_tick = now
        deadline = self.target.deadline_steady
        if deadline is not None and now >= deadline:
            self.revoke("expiry", now=now)
        try:
            filtered = self._learning_observation(now)
        except (ValueError, TypeError, KeyError):
            # Learning cannot withdraw manual/navigation or suppress raw scan handling.
            self.startup.hold_motion("learning_scan_unavailable")
            filtered = None
        self._startup_target(now, filtered)
        state = self.machine.state
        output, reason, scan_deadline = Velocity(0, 0), state.reason, None
        if state.mode not in {MotionMode.ESTOP, MotionMode.EXECUTION_FAULT}:
            requested = self.target.velocity
            if requested != Velocity(0, 0):
                control_filtered = filtered
                if self.node._scans and self.node._scans[-1].predictions:
                    control_filtered = self._filtered_scan(self.node._scans[-1], avoidance=True)
                clearance = directional_clearance(
                    self.node._scans, self.profile, now, filtered=control_filtered
                )
                if clearance is None:
                    reason = "scan_temporarily_unavailable"
                else:
                    current = self.sensors.wheel_linear(now)
                    output = limit_velocity(
                        requested, self._output, current, clearance, self.profile, dt
                    )
                    scan_deadline = clearance.deadline
                    reason = "directional_limit" if output != requested else "target"
            else:
                reason = (
                    self.startup.reason
                    if self.target.source == "startup_learning"
                    else ("standby" if self.target.source == "idle" else "stick_stopped")
                )
        learning_output = self.target.source == "startup_learning"
        if learning_output and output != Velocity(0, 0) and not self.startup.prepare_action():
            output, reason = Velocity(0, 0), "learning_action_reservation_pending"
        if (
            learning_output
            and self.startup.deadline is not None
            and time.monotonic() >= self.startup.deadline
        ):
            output, reason = Velocity(0, 0), "learning_action_window_finished"
        self._output = output
        # A control-process stall stops the native owner even if a source TTL is longer.
        output_deadline = min(
            now + 0.15,
            self.target.deadline_steady or now + 0.15,
            scan_deadline or now + 0.15,
            self.startup.deadline
            if learning_output and self.startup.deadline is not None
            else now + 0.15,
        )
        if learning_output and output != Velocity(0, 0) and time.monotonic() >= output_deadline:
            output = self._output = Velocity(0, 0)
            reason = "learning_output_expired"
            # Discard this candidate only. The same live owner keeps its grant
            # and original window; the next tick must compute a fresh output.
        submitted = self.node._emit_motion_core(
            output, reason, output_deadline, computed_steady=now
        )
        if learning_output and output != Velocity(0, 0):
            self.startup.motion_emitted(submitted, deadline=output_deadline)
        else:
            self.startup.output_stopped(submitted)
        self.diagnostics.summary(
            "startup_learning",
            self.startup.summary(
                learned_count=0
                if self._learning_client is None or self._learning_client.view is None
                else len(self._learning_client.view.catalog.anchors)
            ),
        )
        self._publish_runtime(now, reason)
        self.diagnostics.duration("control_compute", time.monotonic() - now)

    def _publish_runtime(self, now, reason):
        state = self.machine.state
        independent = self.sensors.independent
        packet = getattr(self.sensors, "input_reference", None)
        reference = None if packet is None else packet[0]
        average = None if reference is None else reference.get("average")
        body = None
        if (
            independent is not None
            and average
            and all(value is not None and math.isfinite(value) for value in average.values())
        ):
            body = {
                "average": reference["average"],
                "source": reference["mode"],
                "computed_steady": independent.stamp_steady,
                "valid_until_steady": independent.deadline_steady,
            }
        actions = list(state.allowed_actions)
        if (
            state.task_id == self.boot
            and state.mode not in {MotionMode.ESTOP, MotionMode.EXECUTION_FAULT}
            and "acquire" not in actions
        ):
            actions.append("acquire")
        runtime = MotionRuntimeState.model_validate(
            {
                "schema": 1,
                "robot_id": str(self.node.get_parameter("robot_id").value),
                "boot_id": self.boot,
                "sequence": self.sequence,
                "sampled_steady": now,
                "mode": state.mode.value,
                "reason": reason,
                "stationary": bool(self.sensors.stationary(now)),
                "healthy": bool(self.sensors.latest is not None and self.sensors.latest.healthy),
                "evidence_stamp_steady": None,
                "evidence_deadline_steady": None,
                "estop_revision": state.estop_revision,
                "fault_revision": state.fault_revision,
                "confirmed_estop_status": "software_latched"
                if state.mode is MotionMode.ESTOP
                else "clear",
                "allowed_actions": actions,
                "task_id": state.task_id,
                "output_generation": self.output_generation,
                "body_reference": body,
                "wheel_linear_mps": self.sensors.wheel_linear(now),
            }
        )
        msg = self._runtime_type()
        msg.data = runtime.model_dump_json()
        self._runtime.publish(msg)

    def report_status(self, value):
        self._runtime.status(value)

    def close(self) -> None:
        self._closed = True
        self._interrupt_learning("shutdown", time.monotonic())
        self._resources.close()
