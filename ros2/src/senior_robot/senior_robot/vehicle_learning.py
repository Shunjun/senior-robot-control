
"""Learning client lifecycle, observation adoption and failure facts."""

import json
import time
from dataclasses import asdict

from senior_control.models import Velocity
from senior_control.motion.contracts import MotionEventKind as K
from senior_control.motion.contracts import MotionMode, MotionTarget
from senior_control.motion.learning_record import LearningGeometry
from senior_control.motion.startup_learning import StartupLearning
from senior_control.runtime.learning_client import LearningClient
from senior_control.safety.self_learning import SelfReturnModel
from senior_control.safety.startup_visibility import startup_visibility
from senior_control.sensors.policy import default_policy
from senior_control.vision.raw_capture import capture_from_message
from senior_robot.motion_core_perception import (
    avoidance_observation,
    scan_observation,
)


class LearningRuntime:
    """Owns learning state; context supplies runtime ports."""

    def __init__(self, context):
        self.context = context

    def _interrupt_learning(self, reason, now):
        self.startup._last_now = now
        self.startup.interrupt(reason)
        if self.context.machine.state.task_id == self.context.boot:
            self.context._event(K.TASK_RETIRED, now, self.context.boot)
        if self.context.target.source == "startup_learning":
            self.context.target = MotionTarget(Velocity(0, 0), "idle", self.context.boot, None)
            self.context._output = Velocity(0, 0)

    def _learning_observation(self, now):
        self._learning_stage, self._learning_age = "poll", None
        client = self._learning_client
        if client is None:
            return None
        self.context._trace_mark("poll_begin")
        client.poll()
        self.context._trace_mark("poll_end")
        self._learning_stage = "record-adoption"
        record = client.record
        self._learning_record_phase = None if record is None else record.phase
        if self.context._launch_recovered:
            if (
                self._old_learning_owner is None
                and record is not None
                and record.expected_output_boot_id != self.context.boot
            ):
                # Pin the observed old assignment before reporting its loss. A
                # later State reply/new manual attempt cannot retarget this fact.
                self._old_learning_owner = record
            if (
                not self.context._recovery_pending
                and not self._retired_learning_owner
                and self._old_learning_owner is not None
            ):
                self._retired_learning_owner = client.retire_motion_owner(
                    self._old_learning_owner, self.context.boot
                )
        if (
            self.context._launch_recovered
            and record is not None
            and record.expected_output_boot_id != self.context.boot
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
                    not self.context._recovery_pending
                    and proposal is not None
                    and proposal.output_boot_id == self.context.boot
                    and time.monotonic() < proposal.deadline_steady
                    and proposal.owner_boot_id == self.context._input_boot
                    and proposal.generation == self.context.output_generation
                    and self.context.target.source == "idle"
                    and self.context.sensors.stationary(now)
                    and self.context.machine.state.mode
                    not in {MotionMode.ESTOP, MotionMode.EXECUTION_FAULT}
                )
                if permitted:
                    if not client.bind_attempt(
                        record.attempt_id, self.context.boot, self.context.output_generation
                    ):
                        return None
                    self._manual_learning_request = proposal
                    client.accept_attempt(True, "accepted")
                else:
                    if not client.decline_attempt(
                        record, self.context.boot, "control_source_changed"
                    ):
                        return None
                    self._learning_attempt = record.attempt_id
                    return None
            self.startup = StartupLearning(client)
            self._learning_attempt = record.attempt_id
        self.context._trace_mark("adoption_end")
        self._learning_stage = "advance"
        self.startup.advance(
            self.context.sensors.wheel_linear(now),
            now=now,
            stationary=self.context.sensors.stationary(now),
        )
        self.context._trace_mark("advance_end")
        if not self.startup.active and (
            client.view is None or self.context.target.velocity == Velocity(0, 0)
        ):
            return None
        self._learning_stage = "selection"
        if not self.context.node._scans:
            self._remember_learning_failure(now)
            self.startup.hold_motion("learning_scan_unavailable")
            return None
        raw = self.context.node._scans[-1]
        self._learning_stage = "age"
        age = self._learning_age = default_policy().age(
            measured_steady=raw.measurement_steady, received_steady=raw.received, now_steady=now
        )
        if age.band == "invalid":
            self._remember_learning_failure(now)
            self.startup.hold_motion("scan_temporarily_unavailable")
            return None
        self._learning_stage = "filter"
        return self._filtered_scan(raw)

    def _remember_learning_failure(self, now, error=None):
        """Keep the first motion failure until close, independently of status cadence."""
        if not self.startup.motion_started or self._learning_failure is not None:
            return
        try:
            client = self._learning_client
            age = self._learning_age
            fact = dict(
                boot=self.context.boot,
                sequence=self.context.sequence,
                now=now,
                stage=self._learning_stage,
                selected_count=len(self.context.node._scans),
                scan_generation=getattr(self.context.node, "_scan_generation", None),
                age=None if age is None else dict(band=age.band, seconds=age.seconds),
                exception=None
                if error is None
                else dict(type=type(error).__name__, message=str(error)[:256]),
                client_error=None if client is None else client.error,
                record_phase=self._learning_record_phase,
                startup=dict(
                    state=self.startup.state,
                    reason=self.startup.reason,
                    start=self.startup.started,
                    deadline=self.startup.deadline,
                ),
            )
            if self.context.node._scans:
                raw = self.context.node._scans[-1]
                m = raw.message
                fact["selected"] = dict(
                    bounded_source=m.source_epoch,
                    bounded_sequence=m.sequence,
                    raw_source=m.raw_source,
                    measurement=raw.measurement_steady,
                    receipt=raw.received,
                    received_ros=raw.received_ros,
                    stamp=raw.stamp,
                    frame=m.frame_id,
                    body_frame=self.context.node._body,
                    sensor_to_body=(m.sensor_x, m.sensor_y, m.sensor_yaw),
                    transform_stamp=m.transform_stamp,
                    angle_min=m.angle_min,
                    angle_increment=m.angle_increment,
                    count=len(m.ranges),
                    range_min=m.range_min,
                    range_max=m.range_max,
                )
                try:
                    fact["selected"]["capture"] = asdict(capture_from_message(m.raw_capture))
                except Exception as exc:
                    fact["selected"]["capture_error"] = str(exc)[:256]
            self._learning_failure = json.dumps(fact, allow_nan=False, separators=(",", ":"))
        except Exception as exc:
            # Preserve the first boundary even if malformed evidence cannot be serialized.
            self._learning_failure = json.dumps(
                dict(
                    boot=self.context.boot,
                    sequence=self.context.sequence,
                    now=now,
                    stage=self._learning_stage,
                    observation_error=type(exc).__name__ + ": " + str(exc)[:256],
                )
            )

    def _filtered_scan(self, raw, *, avoidance=False):
        key = self.context._scan_key(raw)
        if key is None or key != self.context._scan_result_key:
            self.context._scan_result_key = key
            self.context._scan_results = [None, None]
        slot = int(avoidance)
        composition = tuple(raw.predictions) if avoidance else ()
        entry = self.context._scan_results[slot]
        self.context._scan_results[slot] = None  # Never retain a partial result after an exception.
        self.context._trace_mark(
            "avoid_cache_hit"
            if avoidance and entry is not None and entry[0] == composition
            else "raw_cache_hit"
            if entry is not None and entry[0] == composition
            else "avoid_decode"
            if avoidance
            else "raw_decode"
        )
        if entry is not None and entry[0] == composition:
            _, observation, previous_filter, result = entry
        else:
            decode = avoidance_observation if avoidance else scan_observation
            observation = decode(raw, self.context.node._body)
            previous_filter = result = None
        self.context._trace_mark("decode_end")
        client = self._learning_client
        view = None if client is None else client.view
        self.context._trace_mark("geometry_begin")
        if not avoidance and client is not None and view is None:
            # Preserve decoding before geometry reports/holds, even on a cold cache.
            m = raw.message
            x, y = self.context.profile.length / 2, self.context.profile.width / 2
            geometry = LearningGeometry(
                body_frame=self.context.node._body,
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
            previous_geometry = self._learning_geometry
            if previous_geometry is None or not geometry.same_structure(previous_geometry):
                if previous_geometry is not None:
                    self.startup.hold_motion("scan_or_body_geometry_changed")
                self.self_returns = SelfReturnModel(geometry.physical_polygon)
            self._learning_geometry = geometry
        self.context._trace_mark("geometry_end")
        filter_key = (
            self.startup.provisional,
            self._learning_geometry,
            None
            if view is None
            else (client.active_revision, view.catalog.model_id, view.geometry),
        )
        self.context._trace_mark(
            "filter_begin" if previous_filter != filter_key else "filter_cache_hit"
        )
        if previous_filter != filter_key:
            if view is not None:
                result = view.filter(observation)
            elif self.self_returns is not None:
                result = self.self_returns.filter(observation, provisional=self.startup.provisional)
            else:
                result = observation
        self.context._trace_mark("filter_end")
        if key is not None:
            self.context._scan_results[slot] = (composition, observation, filter_key, result)
        return result

    def _startup_target(self, now, observation):
        if (
            self.context._launch_recovered
            and getattr(self, "_manual_learning_request", None) is None
        ):
            return
        if self.context.target.source not in {"idle", "startup_learning"}:
            return
        if not self.startup.active or (
            self.startup.deadline is not None and now >= self.startup.deadline
        ):
            if self.context.target.source == "startup_learning":
                if self.context.machine.state.task_id == self.context.boot:
                    self.context._event(K.TASK_RETIRED, now, self.context.boot)
                self.context.target = MotionTarget(Velocity(0, 0), "idle", self.context.boot, None)
                self.context._output = Velocity(0, 0)
            return
        if self.context.machine.state.mode in {MotionMode.ESTOP, MotionMode.EXECUTION_FAULT}:
            return
        if not self.context.node._command_topology_ready():
            if self.startup.motion_started:
                self._interrupt_learning("command_topology_unqualified", now)
            return
        proposal = getattr(self, "_manual_learning_request", None)
        if (
            proposal is not None
            and not self.startup.motion_started
            and (
                now >= proposal.deadline_steady
                or proposal.output_boot_id != self.context.boot
                or proposal.owner_boot_id != self.context._input_boot
                or proposal.generation != self.context.output_generation
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
                front_stop=self.context.profile.startup_learning_front_occlusion_stop,
                rear_stop=self.context.profile.startup_learning_rear_occlusion_stop,
            )
            if visibility.blocked_reason is not None:
                velocity, reason = 0.0, visibility.blocked_reason
                self.startup.hold_motion(reason)
                if not self.startup.active:
                    self._interrupt_learning(self.startup.reason, now)
                    return
        if self.context.machine.state.task_id != self.context.boot:
            self.context._event(K.TASK_START, now, self.context.boot)
        if self.context.machine.state.task_id == self.context.boot:
            self.context.target = MotionTarget(
                Velocity(velocity, 0), "startup_learning", self.context.boot, None
            )
            self.startup.reason = reason

    def initialize(self):
        import os
        from pathlib import Path

        claim = os.environ.get("SENIOR_STARTUP_LEARNING_CLAIM")
        self._learning_client = None
        if claim:
            if self.context._launch_recovered:
                self._learning_client = LearningClient(Path(claim), read_only=True)
            else:
                self._learning_client = LearningClient(Path(claim))
        if self._learning_client is not None:
            self.context._resources.callback(self._learning_client.close)
        self.startup = StartupLearning(
            None if self.context._launch_recovered else self._learning_client
        )
        self.self_returns = None
        self._learning_geometry = None
        self._retired_learning_owner = False
        self._old_learning_owner = None
        self._learning_stage = "poll"
        self._learning_age = None
        self._learning_record_phase = None
        self._learning_failure = None

    def summary(self):
        client = self._learning_client
        return self.startup.summary(
            learned_count=0
            if client is None or client.view is None
            else len(client.view.catalog.anchors)
        )
