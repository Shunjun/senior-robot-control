"""Single motion-state reducer; no task runner, authority manager or actuator."""

import math
from dataclasses import dataclass, replace

from senior_control.motion.contracts import (
    MotionEvent,
    MotionState,
)
from senior_control.motion.contracts import (
    MotionEventKind as K,
)
from senior_control.motion.contracts import (
    MotionMode as M,
)


@dataclass(frozen=True)
class RecoveryObservation:
    epoch: str
    sequence: int
    stamp_steady: float
    deadline_steady: float
    healthy: bool
    stationary: bool
    # //Compatible: only the scoped T5 stop owner can provide this permission.
    manual_rearm_ready: bool = False


class MotionStateMachine:
    """Events are sequenced by ONE trusted composition stream, not browser IDs.

    Source health admission remains T3-owned. This reducer only counts distinct
    admitted evidence for motion fault recovery; repeated ticks do not count.
    Task resume requests replan, never directly starts a paused task.
    """

    def __init__(self) -> None:
        self._software = False
        self._hardware = False
        self._fault = False
        self._feedback_lost = False
        self._feedback_task = False
        self._obstacle = False
        self._manual: str | None = None
        self._task: str | None = None
        self.task_paused = False
        self._replan = False
        self._estop_revision = 0
        self._fault_revision = 0
        self._last_sequence = -1
        self._last_now = -math.inf
        self._evidence: RecoveryObservation | None = None
        self._count = 0
        self._first: float | None = None
        self.state = MotionState(M.IDLE, "standby", None, 0, 0, False, ())

    def observe(self, evidence: RecoveryObservation, now: float) -> None:
        if not self._advance_clock(now):
            return
        old = self._evidence
        if not (
            evidence.epoch
            and type(evidence.sequence) is int
            and evidence.sequence >= 0
            and math.isfinite(now)
            and math.isfinite(evidence.stamp_steady)
            and math.isfinite(evidence.deadline_steady)
            and evidence.stamp_steady <= now < evidence.deadline_steady
        ):
            self._count = 0
            self._first = None
            self._evidence = None
            return
        if (
            old is not None
            and old.epoch == evidence.epoch
            and (evidence.sequence <= old.sequence or evidence.stamp_steady <= old.stamp_steady)
        ):
            if old.manual_rearm_ready and not evidence.manual_rearm_ready:
                # A newer proof veto may tighten permission while point sensors
                # are unchanged. Never recount or extend that original sample.
                self._evidence = replace(old, manual_rearm_ready=False)
                self._project(now, "manual_permission_revoked")
            return
        continuous = old is not None and old.epoch == evidence.epoch and now < old.deadline_steady
        if not continuous or not evidence.healthy:
            self._count = 0
            self._first = None
        self._evidence = evidence
        if evidence.healthy:
            self._count += 1
            if self._first is None:
                self._first = evidence.stamp_steady
        self._project(now, "observation")

    def _healthy(self, now: float, *, recovered: bool = False) -> bool:
        e = self._evidence
        fresh = e is not None and e.healthy and e.stamp_steady <= now < e.deadline_steady
        return bool(
            fresh
            and (
                not recovered
                or (
                    self._count >= 5
                    and self._first is not None
                    and e is not None
                    and e.stamp_steady >= self._first + 0.5
                )
            )
        )

    def _stationary(self, now: float) -> bool:
        return self._healthy(now) and self._evidence is not None and self._evidence.stationary

    def _manual_ready(self, now: float) -> bool:
        e = self._evidence
        return bool(e and e.manual_rearm_ready and e.stamp_steady <= now < e.deadline_steady)

    def _pause_sources(self) -> None:
        self._manual = None
        if self._task is not None:
            self.task_paused = True
        self._replan = False
        self._feedback_task = False

    def _project(self, now: float, reason: str) -> MotionState:
        if self._software or self._hardware:
            mode = M.ESTOP
        elif self._fault:
            mode = M.EXECUTION_FAULT
        elif self._feedback_lost:
            mode = M.FEEDBACK_STOP
        elif self._obstacle:
            mode = M.OBSTACLE_HOLD
        elif self._manual is not None:
            mode = M.MANUAL
        elif self._task is not None and not self.task_paused:
            mode = M.TASK
        elif self._stationary(now):
            mode = M.IDLE
        else:
            mode = M.BRAKING
        actions = []
        if mode is M.ESTOP and not self._hardware:
            actions.append("estop_reset")
        if self._fault and not self._software and not self._hardware:
            actions.append("fault_recover")
        runnable = not (self._software or self._hardware or self._fault or self._feedback_lost)
        if self._replan and runnable:
            actions.append("replan_task")
        if mode is M.IDLE or (
            runnable
            and self._manual is None
            and (self._task is None or self.task_paused)
            and self._manual_ready(now)
        ):
            actions.append("acquire")
        if self.task_paused and runnable and self._stationary(now):
            actions.append("resume_task")
        self.state = MotionState(
            mode,
            reason,
            self._task,
            self._estop_revision,
            self._fault_revision,
            self._stationary(now),
            tuple(actions),
        )
        return self.state

    def _recovery_event(self, event: MotionEvent, now: float) -> None:
        if event.kind is K.HARDWARE_RELEASE:
            self._hardware = False
        elif event.kind is K.ESTOP_RESET:
            if (
                event.revision == self._estop_revision
                and not self._hardware
                and self._stationary(now)
            ):
                self._software = False
                self._pause_sources()
        elif event.kind is K.FAULT_RECOVER:
            if (
                event.revision == self._fault_revision
                and not self._hardware
                and not self._software
                and self._healthy(now, recovered=True)
                and self._stationary(now)
            ):
                self._fault = False
                self._pause_sources()
        elif event.kind is K.FEEDBACK_HEALTHY:
            if self._healthy(now, recovered=True):
                self._feedback_lost = False
                if self._feedback_task and not (self._software or self._hardware or self._fault):
                    self._replan = True
                self._feedback_task = False
            elif self._manual_ready(now):
                # Manual-only recovery never consumes the pending task replan.
                self._feedback_lost = False

    def _source_event(self, event: MotionEvent, now: float) -> None:
        blocked = self._software or self._hardware or self._fault or self._feedback_lost
        kind = event.kind
        if (
            kind is K.MANUAL_START
            and not blocked
            and (self._healthy(now) or self._manual_ready(now))
        ):
            if self._task is not None:
                self.task_paused = True
            self._manual = event.source_id
            self._replan = False
            self._feedback_task = False
        elif kind in (K.MANUAL_RELEASE, K.MANUAL_EXPIRED) and event.source_id == self._manual:
            self._manual = None
        elif kind is K.TASK_START and not blocked and self._manual is None and self._healthy(now):
            if self._task is None or (self._task == event.source_id and self._replan):
                self._task = event.source_id
                self.task_paused = False
                self._replan = False
        elif (
            kind is K.TASK_RESUME
            and not blocked
            and self._manual is None
            and self._stationary(now)
            and self._task == event.source_id
        ):
            self._replan = True
        elif kind is K.TASK_PAUSE and self._task == event.source_id:
            self._feedback_task = False
            self.task_paused = True
            self._replan = False
        elif kind in (K.TASK_DONE,) and self._task == event.source_id:
            self._feedback_task = False
            self._task = None
            self.task_paused = False
            self._replan = False

    def _advance_clock(self, now: float) -> bool:
        if not math.isfinite(now) or now < self._last_now:
            self._fault = True
            self._fault_revision += 1
            self._pause_sources()
            self._evidence = None
            self._count = 0
            self._first = None
            self._project(self._last_now, "clock_fault")
            return False
        self._last_now = now
        return True

    def reduce(self, event: MotionEvent, *, now_steady: float) -> MotionState:
        if not self._advance_clock(now_steady):
            return self.state
        if event.sequence <= self._last_sequence:
            return self._project(now_steady, "duplicate_event")
        self._last_sequence = event.sequence
        kind = event.kind
        if kind in (K.SOFTWARE_ESTOP, K.HARDWARE_ESTOP):
            if kind is K.SOFTWARE_ESTOP:
                self._software = True
            else:
                self._hardware = True
            self._estop_revision += 1
            self._pause_sources()
        elif kind is K.EXECUTION_FAULT:
            self._fault = True
            self._fault_revision += 1
            self._pause_sources()
            self._count = 0
            self._first = None
        elif kind is K.FEEDBACK_LOST:
            if not self._feedback_lost:
                self._feedback_task = (
                    self._task is not None and not self.task_paused and self._manual is None
                )
            if self._feedback_task:
                self.task_paused = True
            self._feedback_lost = True
            self._manual = None
            self._count = 0
            self._first = None
        elif kind is K.OBSTACLE:
            self._obstacle = True
        elif kind is K.OBSTACLE_CLEAR:
            self._obstacle = False
        elif kind in (K.HARDWARE_RELEASE, K.ESTOP_RESET, K.FAULT_RECOVER, K.FEEDBACK_HEALTHY):
            self._recovery_event(event, now_steady)
        else:
            self._source_event(event, now_steady)
        return self._project(now_steady, kind.value)
