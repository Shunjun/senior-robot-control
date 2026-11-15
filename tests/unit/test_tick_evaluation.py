"""One sampled context may reuse computation, never another tick's evidence."""

import copy
import json
import threading
from dataclasses import replace
from types import SimpleNamespace as N

import pytest
from test_constraint_scheduling import Worker
from test_directional_constraints import engine, scene, snapshot
from test_motion_controller import config, feedback
from test_reference_nanosecond_boundary import replay as boundary_fixture

from senior_control.models import Pose2D, Velocity
from senior_control.motion.constraint_worker import ConstraintScheduler
from senior_control.motion.controller import MotionController


@pytest.fixture
def replay(monkeypatch):
    yield from boundary_fixture.__wrapped__(monkeypatch)


def test_real_core_shares_one_base_stop_but_keeps_independent_tracking(replay, monkeypatch):
    core = replay.core
    core.tick()  # Establish health from the captured body/scan through the real core.
    replay.clock.steady += 0.01
    assert core.command(
        N(
            close_reason="intent",
            manual_linear=0.0,
            manual_linear_limit=0.0,
            degraded_linear_limit=0.0,
            source_kind="navigation",
            source_id="captured-route",
            source_epoch=1,
            owner_boot_id="command-owner",
            generation=2,
            sequence=1,
            received_steady=replay.clock.steady,
            steady_deadline=replay.clock.steady + 0.1,
            vx=0.01,
            wz=0.01,
        )
    )
    calls = []
    tracking = N(active=False)
    stop = core.engine._stop_certificate
    proof = core.engine.braking_proof

    def counted(snapshot, now):
        if threading.current_thread().name == "MainThread":
            calls.append("tracking" if tracking.active else "base")
        return stop(snapshot, now)

    def independent(*args, **kwargs):
        tracking.active = True
        try:
            return proof(*args, **kwargs)
        finally:
            tracking.active = False

    monkeypatch.setattr(core.engine, "_stop_certificate", counted)
    monkeypatch.setattr(core.engine, "braking_proof", independent)
    core.tick()
    core.executor.shutdown(wait=True)
    assert calls.count("base") == 1
    assert calls.count("tracking") == 1


def test_checked_result_is_only_this_evaluations_actual_result(monkeypatch):
    e, ctx = engine(), snapshot(0.1)
    permit = e.refine(ctx, 0.1)
    ev = e.evaluate(ctx, 0.1)
    checked = ev.tighten(permit)
    assert checked == e.tighten(permit, ctx, 0.1)
    clear = e._clear
    calls = []

    def counted(*args):
        calls.append(1)
        return clear(*args)

    monkeypatch.setattr(e, "_clear", counted)
    assert ev.tighten(checked) is checked
    assert not calls
    # Equal contents do not confer checked status on a different object.
    ev.tighten(replace(checked))
    assert calls
    assert checked.permit is not None
    expired = replace(checked, permit=replace(checked.permit, deadline_steady=0.1))
    assert ev.tighten(expired).permit is None


@pytest.mark.parametrize("kind", ["pose", "risk", "hazard", "source", "config", "target", "copy"])
def test_changed_context_cannot_reuse_tick_result(kind):
    ctx = snapshot(0.1)
    changed = {
        "pose": replace(ctx, pose=Pose2D(0.01, 0, 0)),
        "risk": replace(ctx, envelope=replace(ctx.envelope, position_error_m=0.1)),
        "hazard": replace(ctx, perception=scene(0.1, hits=((0.3, 0),), revision=2)),
        "source": replace(ctx, motion=replace(ctx.motion, source_epoch="new")),
        "config": replace(ctx, config_version="new"),
        "target": replace(ctx, target=replace(ctx.target, deadline_steady=0.1)),
        "copy": replace(ctx),
    }[kind]
    e = engine()
    ev = e.evaluate(ctx, 0.1)
    ev.tighten(e.refine(ctx, 0.1))
    assert not ev.matches(e, changed, 0.1)
    assert not ev.matches(e, ctx, 0.100000001)
    assert not ev.matches(engine(), ctx, 0.1)


@pytest.mark.parametrize("expired", [False, True])
def test_zero_goal_and_actual_candidate_keep_public_engine_semantics(expired):
    e, ctx = engine(), snapshot(0.1)
    if expired:
        ctx = replace(ctx, target=replace(ctx.target, deadline_steady=0.1))
    ev = e.evaluate(ctx, 0.1)
    stopping = replace(ctx, target=replace(ctx.target, velocity=Velocity(0, 0)))
    assert ev.zero_stop() == e.refine(stopping, 0.1)
    result = e.refine(ctx, 0.1)
    for candidate in (Velocity(0.6, 0), Velocity(0.37, 0), Velocity(-0.1, 0), Velocity(0.6, 0.5)):
        assert ev.validate_output(result, candidate) == e.validate_output(
            result, ctx, candidate, 0.1
        )


@pytest.mark.parametrize("mismatch", ["context", "clock", "engine"])
def test_controller_mismatched_evaluation_uses_original_defensive_path(mismatch, monkeypatch):
    e, old = engine(), snapshot(0.1)
    ev = e.evaluate(old, 0.1)
    ctx, now = (replace(old), 0.1) if mismatch == "context" else (old, 0.1001)
    if mismatch == "engine":
        e, ctx, now = engine(), old, 0.1

    def forbidden(*_args):
        raise AssertionError("foreign evaluation must not be used")

    monkeypatch.setattr(ev, "zero_stop", forbidden)
    monkeypatch.setattr(ev, "tighten", forbidden)
    permission = e.refine(ctx, now)
    decisions = []
    for evaluation in (None, ev):
        c = MotionController(e, config(), Velocity(0, 0), Velocity(0, 0))
        c._last_now = now - 0.02
        decisions.append(
            c.tick(
                ctx.target.velocity,
                ctx,
                permission,
                feedback(now, 0, 1),
                feedback(now, 0, 1, "wheel"),
                now,
                evaluation=evaluation,
            )
        )
    assert decisions[0] == decisions[1]


def test_scheduler_perception_fallback_and_worker_never_retain_evaluation(monkeypatch):
    e, w = engine(), Worker()
    q = ConstraintScheduler(e, w, worker_budget_s=0.6)
    newest = snapshot(0.2)
    q.update(newest, 0.2)
    old = snapshot(0.1)
    ev = e.evaluate(old, 0.3)

    def forbidden(*_args):
        raise AssertionError("perception substitution must use defensive path")

    monkeypatch.setattr(ev, "tighten", forbidden)
    q.update(old, 0.3, evaluation=ev)
    assert q._latest.perception is newest.perception
    assert all(value is not ev for value in vars(q).values())
    assert q._pending is not None
    assert all(value is not ev for value in vars(q._pending).values())
    for _, task in w.jobs:
        assert all(cell.cell_contents is not ev for cell in task.__closure__)
    w.complete()
    assert q.poll(0.4).permit is not None


@pytest.mark.parametrize("waiting", [False, True])
@pytest.mark.parametrize("release", [False, True])
def test_controller_decision_trajectory_and_deadlines_match_defensive_path(waiting, release):
    e = engine()
    controllers = [
        MotionController(e, config(), Velocity(0.1, 0), Velocity(0, 0)) for _ in range(2)
    ]
    for c in controllers:
        c._last_now = 0.08
    for i in range(3):
        now = 0.1 + i * 0.02
        goal = Velocity(0, 0) if release else Velocity(0.4, 0)
        ctx = snapshot(now, target=goal, current=0.1)
        ctx = replace(
            ctx,
            envelope=replace(
                ctx.envelope,
                brake_drive_overshoot=Velocity(0, 0),
                brake_tracking_basis="synthetic independent whole-stop model",
            ),
        )
        permit = None if waiting else e.refine(ctx, now)
        decisions = [
            c.tick(
                goal,
                ctx,
                permit,
                feedback(now, 0.1, i),
                feedback(now, 0.1, i, "wheel"),
                now,
                evaluation=e.evaluate(ctx, now) if index else None,
            )
            for index, c in enumerate(controllers)
        ]
        assert decisions[0] == decisions[1]
        assert vars(controllers[0].trajectory) == vars(controllers[1].trajectory)
        assert controllers[0].integral == controllers[1].integral
        assert controllers[0]._brake_deadline == controllers[1]._brake_deadline


def test_worker_admission_shares_stop_without_renewing_lease(monkeypatch):
    e, w = engine(), Worker()
    q = ConstraintScheduler(e, w, worker_budget_s=0.6)
    first = snapshot(0)
    q.update(first, 0, evaluation=e.evaluate(first, 0))
    w.complete()
    current = snapshot(0.1)
    calls = []
    stop = e._stop_certificate

    def counted(*args):
        calls.append(1)
        return stop(*args)

    monkeypatch.setattr(e, "_stop_certificate", counted)
    result = q.update(current, 0.1, evaluation=e.evaluate(current, 0.1))
    assert result.permit is not None and result.permit.deadline_steady == 1.0
    assert len(calls) == 1


@pytest.mark.parametrize("cause", ["expired", "veto"])
def test_scheduler_no_permit_still_normalizes_in_controller_with_one_base_stop(cause, monkeypatch):
    e, w = engine(), Worker()
    q = ConstraintScheduler(e, w, worker_budget_s=0.6)

    def tracking_context(now, target=None):
        ctx = snapshot(now, target=Velocity(0.6, 0) if target is None else target)
        return replace(
            ctx,
            envelope=replace(
                ctx.envelope,
                brake_drive_overshoot=Velocity(0, 0),
                brake_tracking_basis="synthetic stop",
            ),
        )

    q.update(tracking_context(0), 0)
    w.complete()
    prior = q.poll(0.1)
    assert prior.permit is not None
    now = 1.1 if cause == "expired" else 0.2
    ctx = tracking_context(now, Velocity(0.6 if cause == "expired" else -0.6, 0))

    def control(permission, evaluation=None):
        c = MotionController(e, config(), Velocity(0, 0), Velocity(0, 0))
        c._last_now = now - 0.02
        return c.tick(
            ctx.target.velocity,
            ctx,
            permission,
            feedback(now, 0, 1),
            feedback(now, 0, 1, "wheel"),
            now,
            evaluation=evaluation,
        )

    expected = control(e.tighten(prior, ctx, now))
    assert expected.reason == "waiting_refinement"
    calls = []
    stop = e._stop_certificate

    def counted(*args):
        calls.append(1)
        return stop(*args)

    monkeypatch.setattr(e, "_stop_certificate", counted)
    ev = e.evaluate(ctx, now)
    result = q.update(ctx, now, evaluation=ev)
    assert result.permit is None
    assert result.brake.reason == (
        "permit_expired_or_changed" if cause == "expired" else "latest_observation_veto"
    )
    assert control(result, ev) == expected
    assert len(calls) == 2  # One base proof plus the independent braking proof.


def test_real_core_runtime_normalizes_expired_scheduler_permit(replay, monkeypatch):
    core = replay.core
    core.tick()
    replay.clock.steady += 0.01
    assert core.command(
        N(
            close_reason="intent",
            manual_linear=0.0,
            manual_linear_limit=0.0,
            degraded_linear_limit=0.0,
            source_kind="navigation",
            source_id="route",
            source_epoch=1,
            owner_boot_id="owner",
            generation=2,
            sequence=1,
            received_steady=replay.clock.steady,
            steady_deadline=replay.clock.steady + 0.5,
            vx=0.01,
            wz=0.01,
        )
    )
    core.tick()
    core.scheduler._flight.future.result(timeout=2)
    # Hold only the NEXT refinement, after a real worker produced the prior permit.
    core.scheduler.worker = Worker()
    replay.clock.steady += 0.01
    core.tick()
    assert core.scheduler.current.permit is not None
    replay.clock.steady = core.scheduler.current.permit.deadline_steady
    captured = []
    update = core.scheduler.update

    def observe(*args, **kwargs):
        result = update(*args, **kwargs)
        captured.append(result)
        return result

    monkeypatch.setattr(core.scheduler, "update", observe)
    defensive = copy.deepcopy(core.controller)
    tick = core.controller.propose_tick
    decisions = []

    def compare(*args, **kwargs):
        expected = defensive.tick(*args, **(kwargs | {"evaluation": None}))
        actual = tick(*args, **kwargs)
        assert actual.decision == expected  # Includes command and original evidence deadline.
        decisions.append(expected)
        return actual

    monkeypatch.setattr(core.controller, "propose_tick", compare)
    core.tick()
    core.executor.shutdown(wait=True)
    assert captured[0].brake.reason == "permit_expired_or_changed"
    state = json.loads(replay.node.messages[-1].data)
    assert state["reason"] == "fused_reference:" + decisions[0].reason
    assert state["reason"] == "fused_reference:waiting_refinement"
    assert replay.node.output[-1][0] == decisions[0].command
    assert state["sampled_steady"] == replay.clock.steady
    assert state["valid_until"] == replay.clock.steady + 0.15
    assert state["healthy"] and state["fault_revision"] == 0


@pytest.mark.parametrize("perception_failure", [False, True])
def test_actual_output_cycle_retains_expired_linear_motion_and_starts_zero_response(
    replay, perception_failure
):
    from test_unified_motion_runtime import point

    from senior_control.motion.observations import Source

    core, clock = replay.core, replay.clock

    def observation(now):
        if perception_failure:
            raise ValueError("synthetic_observation_unavailable")
        return replace(scene(now, hits=((1.0, 0),)), frame_id="scan_reference"), Pose2D(0, 0, 0)

    core._observation = observation
    t = clock.ns / 1e9
    core.sensors.ingest(point(Source.WHEEL, t, clock.steady, (0.8, 0.0)), t, clock.steady, t)
    core.sensors.ingest(point(Source.IMU, t, clock.steady, (0.0,)), t, clock.steady, t)
    core.tick()
    assert core.stopping.linear.interval[1] >= 0.8
    assert core.stopping.linear.brake is None
    deadline = core.sensors.latest.vx.deadline_steady
    perception_failure = False
    clock.steady = deadline
    clock.ns = round((t + 1.0) * 1e9)
    core.sensors.ingest(
        point(Source.IMU, t + 1, clock.steady, (0.0,), 2), t + 1, clock.steady, t + 1
    )
    core.tick()
    assert core.sensors.latest.vx.value is None
    assert core.sensors.latest.wz.observed
    assert core.stopping.linear.interval[1] >= 0.8
    assert core.stopping.linear.brake is not None
    assert core.stopping.linear.at == deadline
    assert core.stopping.linear.support_deadline == deadline
    assert replay.node.output[-1][0] == Velocity(0, 0)
