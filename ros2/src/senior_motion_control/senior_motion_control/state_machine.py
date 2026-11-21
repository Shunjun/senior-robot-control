"""Motion source lifecycle and explicit operator emergency latch.

Sensor observations report stillness; they never qualify driving. Obstacles and
missing observations do not revoke sources or create a persistent fault.
"""

import math
from dataclasses import dataclass

from senior_motion_control.contracts import MotionEvent
from senior_motion_control.contracts import MotionEventKind as K
from vehicle_core.motion_state import MotionMode as M
from vehicle_core.motion_state import MotionState


@dataclass(frozen=True)
class RecoveryObservation:
    epoch: str
    sequence: int
    stamp_steady: float
    deadline_steady: float
    healthy: bool
    stationary: bool
    # //Compatible: retained for existing observation producers, never admission.
    manual_rearm_ready: bool = False


class MotionStateMachine:
    def __init__(self) -> None:
        self._software = self._fault = self._obstacle = False
        self._manual: str | None = None
        self._task: str | None = None
        self.task_paused = False
        self._replan = False
        self._estop_revision = self._fault_revision = 0
        self._last_sequence = -1
        self._last_now = -math.inf
        self._evidence: RecoveryObservation | None = None
        self.state = MotionState(M.IDLE, "standby", None, 0, 0, False, ("acquire",))

    def restore_software_estop(self, revision: int, *, now_steady: float) -> bool:
        """Restore a boot-matched output handoff; this never clears a known latch.

        The output owner must verify its boot echo before calling. Ordinary
        emergency events still increment in reduce; explicit reset still has
        to match this exact restored revision.
        """
        if type(revision) is not int or revision < 1 or revision < self._estop_revision:
            return False
        if not self._advance_clock(now_steady):
            return False
        self._software = True
        self._estop_revision = revision
        self._pause_sources()
        self._project(now_steady, K.SOFTWARE_ESTOP.value)
        return True

    def observe(self, evidence: RecoveryObservation, now: float) -> None:
        if not self._advance_clock(now):
            return
        old = self._evidence
        if not (
            evidence.epoch
            and evidence.healthy
            and evidence.stamp_steady <= now < evidence.deadline_steady
        ):
            self._evidence = None
        elif old is None or old.epoch != evidence.epoch or evidence.sequence > old.sequence:
            self._evidence = evidence
        self._project(now, "observation")

    def _stationary(self, now: float) -> bool:
        e = self._evidence
        return bool(e and e.healthy and e.stationary and e.stamp_steady <= now < e.deadline_steady)

    def _pause_sources(self) -> None:
        self._manual = None
        if self._task is not None:
            self.task_paused = True
        self._replan = False

    def _project(self, now: float, reason: str) -> MotionState:
        if self._software:
            mode = M.ESTOP
        elif self._fault:
            mode = M.EXECUTION_FAULT
        elif self._obstacle:
            mode = M.OBSTACLE_HOLD
        elif self._manual is not None:
            mode = M.MANUAL
        elif self._task is not None and not self.task_paused:
            mode = M.TASK
        else:
            mode = M.IDLE
        runnable = not (self._software or self._fault)
        actions = []
        if self._software:
            actions.append("estop_reset")
        elif self._fault:
            actions.append("fault_recover")
        if runnable and self._manual is None and (self._task is None or self.task_paused):
            actions.append("acquire")
        if runnable and self.task_paused:
            actions.append("resume_task")
        if runnable and self._replan:
            actions.append("replan_task")
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

    def _source_event(self, event: MotionEvent) -> None:
        kind = event.kind
        runnable = not (self._software or self._fault)
        if kind is K.MANUAL_START and runnable:
            if self._task is not None:
                self.task_paused = True
            self._manual = event.source_id
            self._replan = False
        elif kind in (K.MANUAL_RELEASE, K.MANUAL_EXPIRED) and event.source_id == self._manual:
            self._manual = None
        elif kind is K.TASK_START and runnable and self._manual is None:
            if self._task is None or (self._task == event.source_id and self._replan):
                self._task = event.source_id
                self.task_paused = self._replan = False
        elif (
            kind is K.TASK_RESUME
            and runnable
            and self._manual is None
            and self._task == event.source_id
        ):
            self._replan = True
        elif kind is K.TASK_PAUSE and self._task == event.source_id:
            self.task_paused = True
            self._replan = False
        elif kind in (K.TASK_DONE, K.TASK_RETIRED) and self._task == event.source_id:
            self._task = None
            self.task_paused = self._replan = False

    def _advance_clock(self, now: float) -> bool:
        if not math.isfinite(now) or now < self._last_now:
            self._fault = True
            self._fault_revision += 1
            self._pause_sources()
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
        if kind is K.SOFTWARE_ESTOP:
            self._software = True
            self._estop_revision += 1
            self._pause_sources()
        elif kind is K.EXECUTION_FAULT:
            self._fault = True
            self._fault_revision += 1
            self._pause_sources()
        elif kind is K.ESTOP_RESET and event.revision == self._estop_revision:
            self._software = False
            self._pause_sources()
        elif (
            kind is K.FAULT_RECOVER
            and event.revision == self._fault_revision
            and not self._software
        ):
            self._fault = False
            self._pause_sources()
        elif kind is K.OBSTACLE:
            self._obstacle = True
        elif kind is K.OBSTACLE_CLEAR:
            self._obstacle = False
        else:
            self._source_event(event)
        return self._project(now_steady, kind.value)
