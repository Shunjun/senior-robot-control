"""Run-owned timed scan collection; no displacement integration or actuator ownership."""

from __future__ import annotations

import math
from pathlib import Path
from typing import TYPE_CHECKING, Protocol
from uuid import uuid4

from senior_control.motion.learning_record import LearningRecord, Phase
from senior_control.vision.raw_capture import RawCaptureHeader

if TYPE_CHECKING:
    from senior_control.safety.self_filter import FilteredObservation
    from senior_control.safety.self_learning import SelfReturnModel


class LearningSession(Protocol):
    error: str | None

    @property
    def record(self) -> LearningRecord | None: ...
    @property
    def ready(self) -> bool: ...
    def reserve_action(self) -> bool: ...
    def phase(self, phase: Phase, reason: str, segment_id: str | None = None) -> None: ...
    def action(self, start_ns: int, interval_ns: int, *, finished: bool) -> None: ...
    def complete(self, points: tuple[tuple[float, float], ...]) -> None: ...


class StartupLearning:
    active_states = {"initializing", "sampling", "moving", "braking", "validating", "unconfirmed"}
    duration_s = 3.0

    def __init__(
        self,
        session: LearningSession | None,
    ) -> None:
        self.session = session
        self.state = "disabled" if session is None else "initializing"
        self.reason = "no_program_start_context" if session is None else "learning_record_pending"
        self.validation_after_steady: float | None = None
        self.started: float | None = None
        self.deadline: float | None = None
        self.motion_withdrawn = False
        self.segment_id = uuid4().hex
        self.resample_required = False
        self._last_now: float | None = None
        self._first_now: float | None = None
        self._stationary = False
        self._frame_keys: set[tuple[str, str, int]] = set()
        self._capture_origin: tuple[str, str] | None = None
        self._capture_frontier = 0.0
        self._capture_measurement: int | None = None
        self._capture_generation: int | None = None
        self._segment_floor = 0.0
        self.validation_samples = 0
        self._motion_frames = 0
        self._record_seen = False
        self._action_granted = False
        self._reservation_requested = False
        self._completion_pending = False
        self._reported_finished = False
        self._last_output: float | None = None
        self._last_output_deadline = 0.0
        self._submitted_duration = 0.0
        self._stop_confirmed = False

    @property
    def active(self) -> bool:
        return self.state in self.active_states

    @property
    def provisional(self) -> bool:
        return self.state in {"sampling", "moving"} and not self.motion_withdrawn

    @property
    def velocity(self) -> float:
        return 0.05 if self.state == "moving" and not self.motion_withdrawn else 0.0

    def prepare_action(self) -> bool:
        # Only the final owner calls this after selecting a qualified nonzero
        # learning output. Waiting under estop/obstacles does not consume a run.
        if self._action_granted:
            return self.state == "moving" and not self.motion_withdrawn
        if (
            self.state == "moving"
            and not self.motion_withdrawn
            and self.session is not None
            and self.session.ready
        ):
            self._reservation_requested = True
            self._action_granted = self.session.reserve_action()
        return self._action_granted

    @property
    def action_permitted(self) -> bool:
        return self._action_granted and self.state == "moving" and not self.motion_withdrawn

    @property
    def motion_started(self) -> bool:
        return self.started is not None

    def _phase(self) -> None:
        if self.session is None:
            return
        phases: dict[str, Phase] = {
            "initializing": "not_started",
            "sampling": "sampling",
            "moving": "moving",
            "braking": "validating",
            "validating": "validating",
            "unconfirmed": "unconfirmed",
            "completed": "completed",
            "interrupted": "failed",
            "failed": "failed",
        }
        phase = phases.get(self.state)
        if phase is not None and phase != "completed":
            self.session.phase(phase, self.reason, self.segment_id)

    def _restore(self) -> None:
        if self.session is None:
            return
        record = self.session.record
        if record is None:
            if self.session.error is not None:
                self.state, self.reason = "unconfirmed", "learning_record_unavailable"
            return
        if record.phase == "completed" and record.model is not None:
            self.state, self.reason = "completed", "restored_completed_record"
            self.motion_withdrawn = True
            self._completion_pending = False
        elif not self._record_seen and record.action.consumed:
            self.motion_withdrawn = True
            self.state, self.reason = "validating", "consumed_action_no_replay"
        self._record_seen = True
        if self.session.error is not None and self.active:
            self.motion_withdrawn = True
            self.state, self.reason = "unconfirmed", "learning_record_unavailable"

    def _accrue_output(self, now: float) -> None:
        if self._last_output is not None:
            end = min(now, self._last_output_deadline)
            self._submitted_duration += max(0.0, end - self._last_output)
            self._last_output = now if now < self._last_output_deadline else None

    def motion_emitted(self, now: float, *, deadline: float | None = None) -> None:
        """Called only AFTER the private output frame was successfully submitted."""
        if not math.isfinite(now) or now < 0 or not self._action_granted:
            raise ValueError("learning_output_clock_or_reservation")
        self._accrue_output(now)
        if self.started is None:
            self._segment("learning_collection_started")
            self.started, self.deadline = now, now + self.duration_s
            if self.session is not None:
                self.session.action(round(now * 1e9), 0, finished=False)
        assert self.deadline is not None
        self._last_output = now
        self._last_output_deadline = min(
            self.deadline, self.deadline if deadline is None else deadline
        )

    @property
    def action_tail_active(self) -> bool:
        return self.started is not None and not self._stop_confirmed

    def output_stopped(self, now: float) -> None:
        self._accrue_output(now)
        self._last_output = None

    def _finish_action(self) -> None:
        if self.started is not None and not self._reported_finished and self.session is not None:
            self.session.action(
                round(self.started * 1e9),
                min(3_000_000_000, round(self._submitted_duration * 1e9)),
                finished=True,
            )
            self._reported_finished = True

    def hold_motion(self) -> None:
        if self.motion_started or self._reservation_requested:
            self.motion_withdrawn = True
            self.state, self.reason = "braking", "learning_output_withdrawn"

    def interrupt(self, reason: str) -> None:
        if self.active:
            if reason in {"control_takeover", "other_control_source"}:
                self._stop_confirmed = True
            self.motion_withdrawn = True
            self.state, self.reason = "interrupted", reason
            self._finish_action()
            self._phase()

    def evidence_gap(self, reason: str) -> None:
        if self.active:
            self._segment(reason)

    def _segment(self, reason: str) -> None:
        self.segment_id = uuid4().hex
        self._segment_floor = 0.0 if self._last_now is None else self._last_now
        self.resample_required = True
        self.validation_samples = 0
        self._motion_frames = 0
        self._frame_keys.clear()
        self._capture_origin = None
        self._capture_measurement = None
        self.reason = reason

    def advance(self, wheel: float | None, *, now: float, stationary: bool) -> None:
        self._restore()
        if not self.active:
            return
        if (
            not math.isfinite(now)
            or now < 0
            or (self._last_now is not None and now < self._last_now)
        ):
            self.interrupt("learning_monotonic_clock_regressed")
            return
        self._last_now = now
        if self._first_now is None:
            self._first_now = now
        self._stationary = stationary and wheel is not None and math.isfinite(wheel)
        self._accrue_output(now)
        if self.deadline is not None and now >= self.deadline:
            if self.state == "moving":
                self.state, self.reason = "braking", "action_window_finished"
            self._finish_action()
        if (
            self._stationary
            and self.started is not None
            and (self.motion_withdrawn or (self.deadline is not None and now >= self.deadline))
        ):
            self._stop_confirmed = True
        if self.state == "initializing" and self._stationary:
            self.state, self.reason = "sampling", "physical_interior_prior"
        if self.state == "braking" and self._stationary:
            self.state, self.reason = "validating", "post_motion_validation"
            self.validation_after_steady = now
        if self.state == "validating" and self.validation_after_steady is None and self._stationary:
            self.validation_after_steady = now
        self._phase()

    def frame(
        self, model: SelfReturnModel, observation: FilteredObservation, *, capture: RawCaptureHeader
    ) -> None:
        self._restore()
        if (
            not self.active
            or self._completion_pending
            or self.validation_samples >= 5
            and self.state == "unconfirmed"
        ):
            return
        if self._first_now is None or capture.end_steady < max(
            self._first_now, self._segment_floor
        ):
            return
        origin = capture.bridge_boot_id, capture.source_epoch
        generation = capture.time.clock_generation
        physical = capture.time.stamp_ns
        changed_source = self._capture_origin is not None and self._capture_origin != origin
        changed_axis = (
            self._capture_generation is not None and generation != self._capture_generation
        )
        if changed_source or changed_axis:
            if capture.end_steady < self._capture_frontier:
                return  # Late old segment cannot reset the current one.
            self._segment("scan_evidence_segment_changed")
            self._segment_floor = capture.start_steady
        self._capture_origin = origin
        self._capture_measurement = physical
        self._capture_generation = generation
        self._capture_frontier = max(self._capture_frontier, capture.end_steady)
        if self.resample_required or model.segment_id != self.segment_id:
            model.begin_segment(self.segment_id)
            self.resample_required = False
        key = capture.bridge_boot_id, capture.source_epoch, capture.sequence
        if key in self._frame_keys:
            return
        collecting = (
            self.started is not None
            and self.deadline is not None
            and self.started <= capture.start_steady <= capture.end_steady < self.deadline
        )
        if self.started is None or collecting:
            if not model.observe(observation, capture=capture, segment_id=self.segment_id):
                return
            self._frame_keys = set(model.frame_keys)
            if collecting:
                self._motion_frames += 1
            elif self.state == "moving":
                model.retain_recent(5)
                self._frame_keys = set(model.frame_keys)
            if self.state == "sampling" and len(model.frames) >= 5:
                self.state, self.reason = "moving", "learning_forward_window"
        elif (
            self.state in {"validating", "unconfirmed"}
            and self.validation_after_steady is not None
            and capture.start_steady >= self.validation_after_steady
            and self._stationary
        ):
            # These fresh post-stop frames confirm the collection lifecycle,
            # never reclassify or revoke the surface measured during the action.
            self._frame_keys.add(key)
            self.validation_samples += 1
            if self.validation_samples >= 5:
                count = model.validate() if self._motion_frames else 0
                no_interior = any(model.frames) and not any(
                    model.interior(point) for points in model.frames for point in points
                )
                if self._motion_frames and (count or no_interior):
                    if self.session is not None:
                        self.session.complete(model.learned)
                        self._completion_pending = True
                        self.reason = "learning_completion_pending_commit"
                else:
                    self.state, self.reason = "unconfirmed", "classification_unverified"
        self._phase()

    def summary(self, learned_count: int = 0) -> dict[str, object]:
        return dict(
            state=self.state,
            reason=self.reason,
            action_elapsed_s=0.0
            if self.started is None or self._last_now is None
            else min(3.0, max(0.0, self._last_now - self.started)),
            action_consumed=bool(
                self.session and self.session.record and self.session.record.action.consumed
            ),
            classification="provisional"
            if self.provisional
            else "learned"
            if learned_count
            else "unconfirmed"
            if self.state == "unconfirmed"
            else "prior"
            if self.state == "initializing"
            else "none",
            learned_local_returns=learned_count,
        )


def start_environment(environment: dict[str, str], root: Path) -> dict[str, str]:
    """One explicit program start allocates one shared-record identity."""
    result = dict(environment)
    result["SENIOR_STARTUP_LEARNING_CLAIM"] = str(root / (".startup-learning-" + uuid4().hex))
    return result
