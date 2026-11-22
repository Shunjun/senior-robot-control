"""A proposal cannot commit unpublished drive state or be consumed twice."""

from dataclasses import replace

import pytest
from senior_motion_control.controller import MotionController
from test_directional_constraints import engine, snapshot
from test_motion_controller import config, feedback

from vehicle_core.values import Velocity


def setup():
    e = engine()
    ctx = snapshot(0.1, target=Velocity(0.4, 0), current=0.1)
    ctx = replace(
        ctx,
        envelope=replace(
            ctx.envelope,
            brake_drive_overshoot=Velocity(0, 0),
            brake_tracking_basis="synthetic stop",
        ),
    )
    c = MotionController(e, config(), Velocity(0.1, 0), Velocity(0, 0))
    c._last_now = 0.08
    return c, ctx, e.refine(ctx, 0.1)


def test_proposal_keeps_command_state_uncommitted_then_consumes_once():
    c, ctx, permit = setup()
    before = (vars(c.trajectory).copy(), c.integral, c._direction, c._brake_deadline)
    prepared = c.propose_tick(
        ctx.target.velocity, ctx, permit, feedback(0.1, 0.1, 1), feedback(0.1, 0.1, 1, "wheel"), 0.1
    )
    assert (vars(c.trajectory), c.integral, c._direction, c._brake_deadline) == before
    c.commit(prepared)
    assert c.trajectory.velocity == prepared.decision.command
    with pytest.raises(ValueError):
        c.commit(prepared)


def propose(c, ctx, permit):
    return c.propose_tick(
        ctx.target.velocity, ctx, permit, feedback(0.1, 0.1, 1), feedback(0.1, 0.1, 1, "wheel"), 0.1
    )


def state(c):
    return (vars(c.trajectory).copy(), c.integral, c._direction, c._brake_deadline)


def test_stop_uses_original_trajectory_dt_and_admits_feedback_only_once(monkeypatch):
    c, ctx, permit = setup()
    c._context_key = (*c.engine.identity(ctx), ctx.mode.value)
    c.integral = Velocity(0.02, 0.01)
    c._direction = Velocity(0, 0)
    c._brake_deadline = 0.8
    before = state(c)
    calls = []
    original = c._admit_feedback

    def admit(*args):
        calls.append(args[0])
        return original(*args)

    monkeypatch.setattr(c, "_admit_feedback", admit)
    drive = propose(c, ctx, permit)
    assert drive.decision.verified and not drive.braking
    assert state(c) == before
    stop = c.prepare_stop(drive, ctx, drive.body, drive.wheel, 0.11)
    assert calls == ["body", "wheel", "imu"]
    assert stop.dt == drive.dt and stop.sampled_now == drive.sampled_now
    assert state(c) == before
    expected = c.trajectory.propose(Velocity(0, 0), ctx.response, drive.dt)
    assert stop.step == expected
    assert stop.braking and stop.fallback and stop.decision.verified
    assert stop.brake_deadline == 0.8
    with pytest.raises(ValueError):
        c.commit(drive)
    with pytest.raises(ValueError):
        c.prepare_stop(stop, ctx, stop.body, stop.wheel, 0.12)
    # The second attempt invalidated the token, never committed a command.
    assert state(c) == before


def test_first_stop_deadline_is_staged_until_selected_commit():
    c, ctx, _ = setup()
    prepared = propose(c, ctx, None)
    assert prepared.braking and prepared.brake_deadline is not None
    assert c._brake_deadline is None
    c.commit(prepared)
    assert c._brake_deadline == prepared.brake_deadline


def test_token_is_immutable_owner_bound_and_invalidated_by_next_proposal():
    from dataclasses import FrozenInstanceError

    c, ctx, permission = setup()
    prepared = propose(c, ctx, permission)
    with pytest.raises(FrozenInstanceError):
        prepared.dt = 0.5
    other, _, _ = setup()
    with pytest.raises(ValueError):
        other.commit(prepared)
    next_proposal = propose(c, ctx, permission)
    with pytest.raises(ValueError):
        c.commit(prepared)
    c.discard(next_proposal)
    with pytest.raises(ValueError):
        c.commit(next_proposal)


def test_expired_first_braking_supervision_still_rejects_moving_feedback():
    c, ctx, permit = setup()
    c._brake_deadline = 0.105
    drive = propose(c, ctx, permit)
    stop = c.prepare_stop(drive, ctx, drive.body, drive.wheel, 0.11)
    assert stop.decision.emergency
    assert stop.decision.reason == "stop_response_failed"
    assert c._brake_deadline == 0.105


@pytest.mark.parametrize("changed", ["target", "body", "source_expired", "estop"])
def test_stop_does_not_reuse_changed_or_unqualified_candidate(changed):
    c, ctx, permit = setup()
    drive = c.propose_tick(
        ctx.target.velocity,
        ctx,
        permit,
        feedback(0.1, 0.1, 1),
        feedback(0.1, 0.1, 1, "wheel"),
        0.1,
        estop=changed == "estop",
    )
    body, now = drive.body, 0.11
    if changed == "target":
        ctx = replace(ctx, target=replace(ctx.target, velocity=Velocity(0, 0)))
    elif changed == "body":
        body = replace(body, sequence=2)
    elif changed == "source_expired":
        now = drive.body.deadline_steady
    with pytest.raises(ValueError):
        c.prepare_stop(drive, ctx, body, drive.wheel, now)
