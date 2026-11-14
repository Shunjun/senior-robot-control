"""Single-owner bounded refinement scheduling. Workers only return immutable values."""

import math
from collections.abc import Callable
from concurrent.futures import Executor, Future
from dataclasses import dataclass, replace
from typing import Protocol

from senior_control.models import Velocity
from senior_control.motion.constraints import ConstraintEngine, ConstraintResult, ConstraintSnapshot
from senior_control.motion.contracts import BrakeKind, BrakeRequest


class Worker(Protocol):
    def submit(self, task: Callable[[], ConstraintResult]) -> Future[ConstraintResult]: ...


class ExecutorWorker:
    """Caller owns the executor; this adapter starts no thread/service implicitly."""

    def __init__(self, executor: Executor):
        self.executor = executor

    def submit(self, task: Callable[[], ConstraintResult]) -> Future[ConstraintResult]:
        return self.executor.submit(task)


@dataclass(frozen=True)
class _Pending:
    snapshot: ConstraintSnapshot
    submitted_at: float
    fence: int


@dataclass(frozen=True)
class _Flight:
    future: Future[ConstraintResult]
    pending: _Pending


def _merge_refinement(current: ConstraintResult, refined: ConstraintResult) -> ConstraintResult:
    """Latest physical braking outranks every computational relaxation."""
    if current.brake is not None and (
        current.brake.kind is BrakeKind.EMERGENCY or not current.brake.stop_verified
    ):
        return current
    return refined if refined.permit is not None else current


class ConstraintScheduler:
    def __init__(self, engine: ConstraintEngine, worker: Worker, *, worker_budget_s: float):
        if not math.isfinite(worker_budget_s) or worker_budget_s <= 0:
            raise ValueError("worker budget must be explicitly positive")
        self.engine = engine
        self.worker = worker
        self.budget = worker_budget_s
        self.current = ConstraintResult(
            None, BrakeRequest(BrakeKind.CONTROLLED, "not_started", False)
        )
        self.worker_diagnostic: str | None = None
        self._latest: ConstraintSnapshot | None = None
        self._pending: _Pending | None = None
        self._flight: _Flight | None = None
        self._fence = 0
        self._active = False
        self._last_now: float | None = None

    def stop(self) -> None:
        self._fence += 1
        self._active = False
        self._pending = None
        brake = self.current.brake
        self.current = ConstraintResult(
            None,
            replace(brake, stop_verified=False)
            if brake is not None and brake.kind is BrakeKind.EMERGENCY
            else BrakeRequest(BrakeKind.CONTROLLED, "cancelled", False),
        )
        # Fence only: cancelling a queued Future need not remove its executor
        # queue item. Retain the one slot until completion, including after stop.

    def _start(self) -> None:
        if self._active and self._flight is None and self._pending is not None:
            pending = self._pending
            self._pending = None
            engine = self.engine
            # No closure over mutable scheduler state; captured snapshot/time are immutable.
            try:
                future = self.worker.submit(
                    lambda: engine.refine(pending.snapshot, pending.submitted_at)
                )
            except Exception:
                self._worker_failure("worker_unavailable")
                return
            self._flight = _Flight(future, pending)

    def _worker_failure(self, reason: str) -> None:
        # Bounded diagnostic only: never rewrite the latest physical brake/permit.
        self.worker_diagnostic = reason

    def _clock_valid(self, now: float) -> bool:
        valid = math.isfinite(now) and (self._last_now is None or now >= self._last_now)
        if valid:
            self._last_now = now
        else:
            self.stop()
        return valid

    def update(self, snapshot: ConstraintSnapshot, now: float) -> ConstraintResult:
        if not self._clock_valid(now):
            return self.current
        if (
            self._latest is not None
            and snapshot.perception.source_epoch == self._latest.perception.source_epoch
            and (
                snapshot.perception.stamp_steady < self._latest.perception.stamp_steady
                or snapshot.perception.revision < self._latest.perception.revision
            )
        ):
            snapshot = replace(snapshot, perception=self._latest.perception)
        if self._latest is not None and self.engine.identity(snapshot) != self.engine.identity(
            self._latest
        ):
            self._fence += 1
            self.current = ConstraintResult(
                None, BrakeRequest(BrakeKind.CONTROLLED, "identity_changed", False)
            )
        self._latest = snapshot
        if snapshot.target.velocity == Velocity(0, 0):
            self.stop()
            self.current = self.engine.refine(snapshot, now)
            return self.current
        self._active = True
        self._pending = _Pending(snapshot, now, self._fence)  # replace, never queue every scan
        return self.poll(now)

    def poll(self, now: float) -> ConstraintResult:
        if not self._clock_valid(now):
            return self.current
        latest = self._latest
        if latest is None:
            return self.current
        if not self._active:
            # Cancel/zero ends authorization, not physical deceleration. Reuse the
            # engine's original evidence deadlines; never renew or dispatch work.
            stopping = replace(latest, target=replace(latest.target, velocity=Velocity(0, 0)))
            self.current = self.engine.refine(stopping, now)
            if self._flight is not None and self._flight.future.done():
                self._flight = None  # fenced result is discarded, never admitted
            return self.current
        self.current = self.engine.tighten(self.current, latest, now)
        flight = self._flight
        if flight is not None and flight.future.done():
            self._flight = None
            admissible = (
                flight.pending.fence == self._fence
                and now - flight.pending.submitted_at <= self.budget
            )
            if not admissible:
                self._worker_failure(
                    "worker_budget_exceeded"
                    if now - flight.pending.submitted_at > self.budget
                    else "cancelled_result"
                )
            elif flight.future.cancelled():
                self._worker_failure("cancelled_result")
            if admissible and not flight.future.cancelled():
                try:
                    result = flight.future.result()
                except Exception:
                    self._worker_failure("worker_failed")
                    result = None
                if result is not None:
                    admitted = self.engine.admit(result, latest, now)
                    self.current = _merge_refinement(self.current, admitted)
                    if admitted.permit is not None and self.current is admitted:
                        self.worker_diagnostic = None
        if self._flight is not None and now - self._flight.pending.submitted_at > self.budget:
            self._worker_failure("worker_budget_exceeded")
        self._start()
        return self.current
