"""Learning client lifecycle, observation adoption and failure facts."""

import json
import time
from collections.abc import Callable, Sequence
from dataclasses import asdict, dataclass
from typing import Any, Literal

from senior_motion_control.hardware_profile import HardwareMotionProfile

from senior_perception.learning_client import LearningClient
from senior_perception.learning_record import LearningGeometry
from senior_perception.raw_safety_observation import RawSafetyObservation
from senior_perception.scan_access import SensorAccess
from senior_perception.scan_decode import (
    avoidance_observation,
    scan_observation,
)
from senior_perception.self_learning import SelfReturnModel
from senior_perception.startup_learning import StartupLearning
from senior_perception.startup_visibility import startup_visibility
from senior_perception.vision.raw_capture import capture_from_message
from vehicle_core.motion_state import MotionMode
from vehicle_core.sensor_policy import default_policy
from vehicle_core.values import Velocity


@dataclass(frozen=True)
class LearningControlInput:
    """Control-owned values sampled for one call; no mutable machine or target port."""

    output_boot_id: str
    input_boot_id: str
    generation: int
    mode: MotionMode
    task_id: str | None
    target_source: str
    target_velocity: Velocity
    launch_recovered: bool
    recovery_pending: bool
    sequence: int


@dataclass(frozen=True)
class LearningMotionRequest:
    """A proposal, never permission: control rechecks the producing identity."""

    kind: Literal["retire", "target"]
    output_boot_id: str
    input_boot_id: str
    generation: int
    velocity: float = 0.0
    reason: str = ""


@dataclass(frozen=True)
class LearningPorts:
    """Observation/cache resources and bounded lifecycle/trace hooks, not arbitration."""

    get_profile: Callable[[], HardwareMotionProfile]
    get_body_frame: Callable[[], str]
    scans: Callable[[], Sequence[RawSafetyObservation]]
    scan_generation: Callable[[], int | None]
    wheel_linear: Callable[[float], float | None]
    stationary: Callable[[float], bool]
    command_topology_ready: Callable[[], bool]
    trace: Callable[[str], None]
    register_close: Callable[[Callable[[], None]], Any]
    scan_access: SensorAccess


class LearningRuntime:
    """Own learning facts; return motion requests for control to apply."""

    def __init__(self, ports: LearningPorts):
        self.ports = ports

    @staticmethod
    def _request(control, kind, *, velocity=0.0, reason=""):
        return LearningMotionRequest(
            kind,
            control.output_boot_id,
            control.input_boot_id,
            control.generation,
            velocity,
            reason,
        )

    def _interrupt_learning(self, reason, now, control: LearningControlInput):
        self.startup._last_now = now
        self.startup.interrupt(reason)
        return self._request(control, "retire", reason=reason)

    def applied(self, request: LearningMotionRequest) -> None:
        """Only the learning owner applies its reason after control accepts a target."""
        if request.kind == "target":
            self.startup.reason = request.reason

    def _learning_observation(self, now, control: LearningControlInput):
        self._learning_stage, self._learning_age = "poll", None
        client = self._learning_client
        if client is None:
            return None
        self.ports.trace("poll_begin")
        client.poll()
        self.ports.trace("poll_end")
        self._learning_stage = "record-adoption"
        record = client.record
        self._learning_record_phase = None if record is None else record.phase
        if control.launch_recovered:
            if (
                self._old_learning_owner is None
                and record is not None
                and record.expected_output_boot_id != control.output_boot_id
            ):
                # Pin the observed old assignment before reporting its loss. A
                # later State reply/new manual attempt cannot retarget this fact.
                self._old_learning_owner = record
            if (
                not control.recovery_pending
                and not self._retired_learning_owner
                and self._old_learning_owner is not None
            ):
                self._retired_learning_owner = client.retire_motion_owner(
                    self._old_learning_owner, control.output_boot_id
                )
        if (
            control.launch_recovered
            and record is not None
            and record.expected_output_boot_id != control.output_boot_id
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
                    not control.recovery_pending
                    and proposal is not None
                    and proposal.output_boot_id == control.output_boot_id
                    and time.monotonic() < proposal.deadline_steady
                    and proposal.owner_boot_id == control.input_boot_id
                    and proposal.generation == control.generation
                    and control.target_source == "idle"
                    and self.ports.stationary(now)
                    and control.mode not in {MotionMode.ESTOP, MotionMode.EXECUTION_FAULT}
                )
                if permitted:
                    if not client.bind_attempt(
                        record.attempt_id, control.output_boot_id, control.generation
                    ):
                        return None
                    self._manual_learning_request = proposal
                    client.accept_attempt(True, "accepted")
                else:
                    if not client.decline_attempt(
                        record, control.output_boot_id, "control_source_changed"
                    ):
                        return None
                    self._learning_attempt = record.attempt_id
                    return None
            self.startup = StartupLearning(client)
            self._learning_attempt = record.attempt_id
        self.ports.trace("adoption_end")
        self._learning_stage = "advance"
        self.startup.advance(
            self.ports.wheel_linear(now),
            now=now,
            stationary=self.ports.stationary(now),
        )
        self.ports.trace("advance_end")
        if not self.startup.active and (
            client.view is None or control.target_velocity == Velocity(0, 0)
        ):
            return None
        self._learning_stage = "selection"
        if not self.ports.scans():
            self._remember_learning_failure(now, control)
            self.startup.hold_motion("learning_scan_unavailable")
            return None
        raw = self.ports.scans()[-1]
        self._learning_stage = "age"
        age = self._learning_age = default_policy().age(
            measured_steady=raw.measurement_steady, received_steady=raw.received, now_steady=now
        )
        if age.band == "invalid":
            self._remember_learning_failure(now, control)
            self.startup.hold_motion("scan_temporarily_unavailable")
            return None
        self._learning_stage = "filter"
        return self._filtered_scan(raw)

    def _remember_learning_failure(self, now, control: LearningControlInput, error=None):
        """Keep the first motion failure until close, independently of status cadence."""
        if not self.startup.motion_started or self._learning_failure is not None:
            return
        try:
            client = self._learning_client
            age = self._learning_age
            fact = dict(
                boot=control.output_boot_id,
                sequence=control.sequence,
                now=now,
                stage=self._learning_stage,
                selected_count=len(self.ports.scans()),
                scan_generation=self.ports.scan_generation(),
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
            if self.ports.scans():
                raw = self.ports.scans()[-1]
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
                    body_frame=self.ports.get_body_frame(),
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
                    boot=control.output_boot_id,
                    sequence=control.sequence,
                    now=now,
                    stage=self._learning_stage,
                    observation_error=type(exc).__name__ + ": " + str(exc)[:256],
                )
            )

    def _filtered_scan(self, raw, *, avoidance=False):
        key = self.ports.scan_access._scan_key(raw)
        if key is None or key != self.ports.scan_access._scan_result_key:
            self.ports.scan_access._scan_result_key = key
            self.ports.scan_access._scan_results = [None, None]
        slot = int(avoidance)
        composition = tuple(raw.predictions) if avoidance else ()
        entry = self.ports.scan_access._scan_results[slot]
        self.ports.scan_access._scan_results[slot] = (
            None  # Never retain a partial result after an exception.
        )
        self.ports.trace(
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
            observation = decode(raw, self.ports.get_body_frame())
            previous_filter = result = None
        self.ports.trace("decode_end")
        client = self._learning_client
        view = None if client is None else client.view
        self.ports.trace("geometry_begin")
        if not avoidance and client is not None and view is None:
            # Preserve decoding before geometry reports/holds, even on a cold cache.
            m = raw.message
            x, y = self.ports.get_profile().length / 2, self.ports.get_profile().width / 2
            geometry = LearningGeometry(
                body_frame=self.ports.get_body_frame(),
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
        self.ports.trace("geometry_end")
        filter_key = (
            self.startup.provisional,
            self._learning_geometry,
            None
            if view is None
            else (client.active_revision, view.catalog.model_id, view.geometry),
        )
        self.ports.trace("filter_begin" if previous_filter != filter_key else "filter_cache_hit")
        if previous_filter != filter_key:
            if view is not None:
                result = view.filter(observation)
            elif self.self_returns is not None:
                result = self.self_returns.filter(observation, provisional=self.startup.provisional)
            else:
                result = observation
        self.ports.trace("filter_end")
        if key is not None:
            self.ports.scan_access._scan_results[slot] = (
                composition,
                observation,
                filter_key,
                result,
            )
        return result

    def _startup_target(self, now, observation, control: LearningControlInput):
        if control.launch_recovered and getattr(self, "_manual_learning_request", None) is None:
            return
        if control.target_source not in {"idle", "startup_learning"}:
            return
        if not self.startup.active or (
            self.startup.deadline is not None and now >= self.startup.deadline
        ):
            if control.target_source == "startup_learning":
                return self._request(control, "retire")
            return None
        if control.mode in {MotionMode.ESTOP, MotionMode.EXECUTION_FAULT}:
            return
        if not self.ports.command_topology_ready():
            if self.startup.motion_started:
                return self._interrupt_learning("command_topology_unqualified", now, control)
            return
        proposal = getattr(self, "_manual_learning_request", None)
        if (
            proposal is not None
            and not self.startup.motion_started
            and (
                now >= proposal.deadline_steady
                or proposal.output_boot_id != control.output_boot_id
                or proposal.owner_boot_id != control.input_boot_id
                or proposal.generation != control.generation
            )
        ):
            return self._interrupt_learning("control_source_changed", now, control)
            return
        velocity = self.startup.velocity
        reason = self.startup.reason
        if observation is None:
            velocity, reason = 0.0, "learning_scan_unavailable"
        elif velocity:
            visibility = startup_visibility(
                observation,
                front_stop=self.ports.get_profile().startup_learning_front_occlusion_stop,
                rear_stop=self.ports.get_profile().startup_learning_rear_occlusion_stop,
            )
            if visibility.blocked_reason is not None:
                velocity, reason = 0.0, visibility.blocked_reason
                self.startup.hold_motion(reason)
                if not self.startup.active:
                    return self._interrupt_learning(self.startup.reason, now, control)
                    return
        return self._request(control, "target", velocity=velocity, reason=reason)

    def initialize(self, control: LearningControlInput):
        import os
        from pathlib import Path

        claim = os.environ.get("SENIOR_STARTUP_LEARNING_CLAIM")
        self._learning_client = None
        if claim:
            if control.launch_recovered:
                self._learning_client = LearningClient(Path(claim), read_only=True)
            else:
                self._learning_client = LearningClient(Path(claim))
        if self._learning_client is not None:
            self.ports.register_close(self._learning_client.close)
        self.startup = StartupLearning(None if control.launch_recovered else self._learning_client)
        self.self_returns = None
        self._learning_geometry = None
        self._retired_learning_owner = False
        self._old_learning_owner = None
        self._learning_stage = "poll"
        self._learning_age = None
        self._learning_record_phase = None
        self._learning_failure = None

    @property
    def failure_report(self) -> str | None:
        """Read-only failure fact for the control report owner."""
        return self._learning_failure

    def summary(self):
        client = self._learning_client
        return self.startup.summary(
            learned_count=0
            if client is None or client.view is None
            else len(client.view.catalog.anchors)
        )
