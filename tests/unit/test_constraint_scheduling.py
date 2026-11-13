"""Controlled worker/clock; no thread or Linux scheduling claims."""

from concurrent.futures import Future
from dataclasses import replace

import pytest
from test_directional_constraints import engine, scene, snapshot

from senior_control.models import Velocity
from senior_control.motion.constraint_worker import ConstraintScheduler
from senior_control.motion.contracts import BrakeKind


class Worker:
    def __init__(self):
        self.jobs = []

    def submit(self, task):
        future = Future()
        self.jobs.append((future, task))
        return future

    def complete(self):
        future, task = self.jobs.pop(0)
        if not future.cancelled():
            future.set_result(task())


def test_ten_hz_updates_do_not_starve_slow_refinement_or_renew_deadline():
    w = Worker()
    q = ConstraintScheduler(engine(), w, worker_budget_s=0.6)
    q.update(snapshot(), 0)
    for i in range(1, 5):
        assert q.update(snapshot(i / 10), i / 10).permit is None
        assert len(w.jobs) == 1
    w.complete()
    result = q.update(snapshot(0.5), 0.5)
    assert result.permit is not None
    assert result.permit.deadline_steady == 1.0
    later = q.update(snapshot(0.6, target=Velocity(0.55, 0)), 0.6)
    assert later.permit is not None
    assert later.permit.deadline_steady == 1.0
    assert all(v.vx <= 0.55 for v in later.permit.velocities)
    assert not q.engine.validate_output(later, snapshot(0.6), Velocity(0.37, 0), 0.6)


def test_new_hazard_tightens_now_and_rejects_older_relaxation():
    w = Worker()
    q = ConstraintScheduler(engine(), w, worker_budget_s=0.6)
    q.update(snapshot(), 0)
    w.complete()
    assert q.update(snapshot(0.1), 0.1).permit is not None
    hazard = snapshot(0.2, hits=((0.34, 0),))
    assert q.update(hazard, 0.2).permit is None
    w.complete()
    assert q.update(hazard, 0.3).permit is None


def test_source_config_and_stop_fence_reject_late_result():
    for changed in ("epoch", "config", "stop"):
        w = Worker()
        q = ConstraintScheduler(engine(), w, worker_budget_s=0.6)
        s = snapshot()
        q.update(s, 0)
        if changed == "stop":
            q.stop()
        elif changed == "epoch":
            q.update(replace(s, perception=scene(0.1, epoch="new")), 0.1)
        else:
            q.update(replace(s, config_version="config-2"), 0.1)
        w.complete()
        result = q.poll(0.2)
        assert result.permit is None


def test_worker_computation_has_no_authority_or_scheduler_side_effect():
    w = Worker()
    q = ConstraintScheduler(engine(), w, worker_budget_s=0.6)
    q.update(snapshot(), 0)
    w.complete()
    assert q.current.permit is None
    assert q.poll(0.1).permit is not None


def test_identity_round_trip_cannot_resurrect_old_job():
    w = Worker()
    q = ConstraintScheduler(engine(), w, worker_budget_s=0.6)
    s = snapshot()
    q.update(s, 0)
    q.update(replace(s, config_version="config-2"), 0.1)
    q.update(s, 0.2)
    w.complete()
    assert q.poll(0.3).permit is None


def test_worker_budget_rejects_late_relaxation_without_new_qualification_gate():
    w = Worker()
    q = ConstraintScheduler(engine(), w, worker_budget_s=0.2)
    q.update(snapshot(), 0)
    w.complete()
    result = q.poll(0.3)
    assert result.permit is None
    assert result.brake.reason == "waiting_refinement"
    assert q.worker_diagnostic == "worker_budget_exceeded"


def test_busy_worker_reports_budget_immediately_and_keeps_only_one_slot():
    w = Worker()
    q = ConstraintScheduler(engine(), w, worker_budget_s=0.2)
    q.update(snapshot(), 0)
    assert q.poll(0.3).brake.reason == "waiting_refinement"
    assert q.worker_diagnostic == "worker_budget_exceeded"
    for i in range(4, 10):
        q.update(snapshot(i / 10), i / 10)
    assert len(w.jobs) == 1


def test_old_scan_cannot_erase_a_newer_hazard_or_refresh_its_deadline():
    w = Worker()
    q = ConstraintScheduler(engine(), w, worker_budget_s=0.6)
    q.update(snapshot(), 0)
    w.complete()
    q.update(snapshot(0.1), 0.1)
    q.update(snapshot(0.2, hits=((0.34, 0),)), 0.2)
    stale = replace(snapshot(0.3), perception=scene(0.1))
    w.complete()
    assert q.update(stale, 0.3).permit is None


def test_slow_worker_continues_serving_during_repeated_identical_obstacles():
    w = Worker()
    q = ConstraintScheduler(engine(), w, worker_budget_s=0.5)
    q.update(snapshot(hits=((3, 0),)), 0)
    for i in range(1, 17):
        now = i / 10
        if i % 4 == 0:
            w.complete()
        result = q.update(snapshot(now, hits=((3, 0),)), now)
        if i >= 4:
            assert result.permit is not None
        assert len(w.jobs) == 1


def test_worker_failure_isolated_from_brake_and_future_requests():
    w = Worker()
    q = ConstraintScheduler(engine(), w, worker_budget_s=0.5)
    q.update(snapshot(), 0)
    future, task = w.jobs.pop(0)
    future.set_exception(RuntimeError("synthetic worker failure"))
    result = q.poll(0.1)
    assert result.permit is None and result.brake.reason == "waiting_refinement"
    assert q.worker_diagnostic == "worker_failed"


def test_submit_failure_is_a_bounded_brake_result_not_a_tick_exception():
    class ClosedWorker:
        def submit(self, task):
            raise RuntimeError("synthetic executor closed")

    q = ConstraintScheduler(engine(), ClosedWorker(), worker_budget_s=0.5)
    assert q.update(snapshot(), 0).brake.reason == "waiting_refinement"
    assert q.worker_diagnostic == "worker_unavailable"


def test_stop_then_new_request_cannot_use_old_result_but_can_make_progress():
    w = Worker()
    q = ConstraintScheduler(engine(), w, worker_budget_s=0.5)
    q.update(snapshot(), 0)
    q.stop()
    q.update(snapshot(0.1), 0.1)
    assert len(w.jobs) == 1
    w.complete()
    assert q.poll(0.2).permit is None
    w.complete()
    assert q.poll(0.3).permit is not None


@pytest.mark.parametrize(
    "failure",
    ["submit", "future", "running_timeout", "completed_timeout", "old_fence", "cancelled"],
)
def test_computation_status_cannot_downgrade_physical_emergency(failure):
    class ClosedWorker:
        def submit(self, task):
            raise RuntimeError("synthetic submit error")

    w = Worker()
    q = ConstraintScheduler(
        engine(), ClosedWorker() if failure == "submit" else w, worker_budget_s=0.2
    )
    danger = snapshot(current=0.4, hits=((0.5, 0),))
    expected = engine().refine(danger, 0).brake
    assert expected.kind is BrakeKind.EMERGENCY
    q.update(danger, 0)
    now = 0.1
    if failure == "future":
        future, _ = w.jobs.pop(0)
        future.set_exception(RuntimeError("synthetic worker error"))
    elif failure == "cancelled":
        future, _ = w.jobs.pop(0)
        future.cancel()
    elif failure == "completed_timeout":
        w.complete()
        now = 0.3
    elif failure == "running_timeout":
        now = 0.3
    elif failure == "old_fence":
        q.update(replace(danger, config_version="new"), 0.05)
        w.complete()
    result = q.current if failure == "submit" else q.poll(now)
    assert result.brake == expected
    assert result.permit is None
    assert q.worker_diagnostic is not None


@pytest.mark.parametrize("failure", ["submit", "future", "running_timeout", "completed_timeout"])
def test_worker_failure_preserves_latest_valid_permit(failure):
    w = Worker()
    q = ConstraintScheduler(engine(), w, worker_budget_s=0.2)
    q.update(snapshot(), 0)
    w.complete()
    permitted = q.poll(0.05)
    assert permitted.permit is not None
    if failure == "submit":

        class ClosedWorker:
            def submit(self, task):
                raise RuntimeError("closed")

        q.worker = ClosedWorker()
    q.update(snapshot(0.1), 0.1)
    now = 0.15
    if failure == "future":
        future, _ = w.jobs.pop(0)
        future.set_exception(RuntimeError("failed"))
    elif failure == "completed_timeout":
        w.complete()
        now = 0.35
    elif failure == "running_timeout":
        now = 0.35
    result = q.poll(now)
    assert result.permit is not None
    assert result.permit.deadline_steady == permitted.permit.deadline_steady
    assert q.worker_diagnostic is not None


@pytest.mark.parametrize("deadline", ["motion", "perception", "response", "envelope"])
def test_zero_target_poll_rechecks_each_original_proof_deadline(deadline):
    s = snapshot(target=Velocity(0, 0))
    if deadline == "perception":
        s = replace(s, perception=replace(s.perception, deadline_steady=0.2))
    else:
        value = getattr(s, deadline)
        s = replace(s, **{deadline: replace(value, valid_until_steady=0.2)})
    w = Worker()
    q = ConstraintScheduler(engine(), w, worker_budget_s=0.5)
    assert q.update(s, 0).brake.stop_verified
    assert not q.poll(0.3).brake.stop_verified
    assert not q.poll(30).brake.stop_verified
    assert not w.jobs


def test_cancel_rechecks_braking_without_allowing_late_relaxation():
    w = Worker()
    q = ConstraintScheduler(engine(), w, worker_budget_s=0.5)
    q.update(snapshot(), 0)
    q.stop()
    w.complete()
    assert q.poll(0.1).permit is None
    assert q.poll(0.1).brake.stop_verified
    assert not q.poll(3).brake.stop_verified
    assert len(w.jobs) == 0
    q.update(snapshot(3.1), 3.1)
    assert len(w.jobs) == 1
    w.complete()
    assert q.poll(3.2).permit is not None


def test_zero_and_cancel_keep_checking_actual_forward_inertia_without_worker():
    danger = snapshot(target=Velocity(0, 0), current=0.4, hits=((0.5, 0),))
    w = Worker()
    q = ConstraintScheduler(engine(), w, worker_budget_s=0.5)
    assert q.update(danger, 0).brake.kind is BrakeKind.EMERGENCY
    q.stop()
    assert q.current.brake.kind is BrakeKind.EMERGENCY
    result = q.poll(0.1)
    assert result.brake.kind is BrakeKind.EMERGENCY
    assert result.brake.reason == "stop_envelope_blocked"
    assert not result.brake.stop_verified
    assert not w.jobs
    assert not q.poll(30).brake.stop_verified
