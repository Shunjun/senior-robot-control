"""Run-owned timed scan collection; no displacement integration or actuator ownership."""

from __future__ import annotations

import math
import time
from pathlib import Path
from typing import Protocol
from uuid import uuid4

from senior_control.motion.learning_record import LearningRecord, Phase


class LearningSession(Protocol):
    error: str | None

    @property
    def record(self) -> LearningRecord | None: ...
    @property
    def ready(self) -> bool: ...
    def reserve_action(self) -> bool: ...
    def phase(self, phase: Phase, reason: str, segment_id: str | None = None) -> None: ...
    def action(self, start_ns: int, interval_ns: int, *, finished: bool) -> None: ...

    accepted_frames: int

    def observed(
        self,
        *,
        segment_id: str,
        now_ns: int,
        validation_after_ns: int | None,
        first_takeover_ns: int | None,
        stationary: bool,
        reason: str | None = None,
    ) -> None: ...


class StartupLearning:
    active_states = {"initializing", "sampling", "moving", "braking"}
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
        self._last_now: float | None = None
        self._stationary = False
        self.first_takeover_ns: int | None = None
        self._record_seen = False
        self._action_granted = False
        self._reservation_requested = False
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
        elif not self._record_seen and record.phase in {"failed", "unconfirmed"}:
            self.motion_withdrawn = True
            self.state = "interrupted"
            self.reason = "restored_terminal_attempt_no_replay"
        elif not self._record_seen and record.capture_summary.state == "complete":
            self.state, self.reason = "validating", "capture_complete_waiting_compute"
            self.motion_withdrawn = True
        elif not self._record_seen and record.action.consumed:
            # A new consumer has no local collection to validate or motion to replay.
            self.interrupt("consumed_action_no_replay")
        if self._record_seen and not self.active and record.phase in {"failed", "unconfirmed"}:
            self.state = "unconfirmed" if record.phase == "unconfirmed" else "interrupted"
            self.reason = record.reason
            self.motion_withdrawn = True
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

    def hold_motion(self, reason: str = "learning_output_withdrawn") -> None:
        if not self.active:
            return
        if self.motion_started:
            self.motion_withdrawn = True
            self.state, self.reason = "braking", reason
        elif self._reservation_requested:
            # Reservation is irreversible, but it is not a submitted motion window.
            self.interrupt(reason)

    def interrupt(self, reason: str) -> None:
        if self.state in {"completed", "failed", "interrupted", "disabled"}:
            return
        if (
            self.session is not None
            and self.session.record is not None
            and self.session.record.capture_summary.state == "complete"
        ):
            return  # Closed raw data belongs to the compute owner, irrespective of control.
        if self.first_takeover_ns is None:
            self.first_takeover_ns = (
                time.monotonic_ns() if self._last_now is None else round(self._last_now * 1e9)
            )
        if self.active or self.state == "validating":
            if reason in {"control_takeover", "other_control_source"}:
                self._stop_confirmed = True
            self.motion_withdrawn = True
            self.state, self.reason = "interrupted", reason
            self._finish_action()
            if self.session is not None:
                self.session.observed(
                    segment_id=self.segment_id,
                    now_ns=self.first_takeover_ns,
                    validation_after_ns=None
                    if self.validation_after_steady is None
                    else round(self.validation_after_steady * 1e9),
                    first_takeover_ns=self.first_takeover_ns,
                    reason=reason,
                    stationary=self._stationary,
                )
            self._phase()

    def _observe(self, now: float) -> None:
        if self.session is not None:
            self.session.observed(
                segment_id=self.segment_id,
                now_ns=round(now * 1e9),
                validation_after_ns=None
                if self.validation_after_steady is None
                else round(self.validation_after_steady * 1e9),
                first_takeover_ns=self.first_takeover_ns,
                reason=self.reason if self.first_takeover_ns is not None else None,
                stationary=self._stationary,
            )

    def advance(self, wheel: float | None, *, now: float, stationary: bool) -> None:
        if (
            not math.isfinite(now)
            or now < 0
            or (self._last_now is not None and now < self._last_now)
        ):
            self.interrupt("learning_monotonic_clock_regressed")
            return
        self._last_now = now
        self._stationary = stationary and wheel is not None and math.isfinite(wheel)
        self._restore()
        if self.state in {"braking", "validating"} and not self.motion_started:
            self.interrupt("learning_output_withdrawn")
        if not self.active:
            if self.state == "validating" and self.deadline is not None and now >= self.deadline:
                self._finish_action()
            self._observe(now)
            self._phase()
            return
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
        if (
            self.state == "sampling"
            and self.session is not None
            and self.session.ready
            and self.session.accepted_frames >= 5
        ):
            self.state, self.reason = "moving", "learning_forward_window"
        self._observe(now)
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
