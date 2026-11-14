"""Closed-loop synthetic plant; no physical bounds are frozen by these tests."""

from dataclasses import replace

import pytest
from shapely.geometry import Point
from test_directional_constraints import engine, snapshot

from senior_control.models import Velocity
from senior_control.motion.controller import ControllerConfig, Feedback, MotionController
from senior_control.motion.source_modes import SourceMode


def config():
    return ControllerConfig(
        Velocity(0.5, 0.3),
        Velocity(0.3, 0.1),
        Velocity(0.2, 0.1),
        0.4,
        1.0,
        0.1,
        0.02,
        0.03,
        3,
        0.1,
        0.3,
        "synthetic plant",
        True,
    )


def feedback(now, v, seq, epoch="body", supported=True):
    return Feedback(
        Velocity(v, 0), now, now + 0.08, epoch, seq, supported, "synthetic current plant"
    )


def loop(steps=500, release=200):
    c = MotionController(engine(), config(), Velocity(0, 0), Velocity(0, 0))
    body = 0.0
    x = 0.0
    rows = []
    for i in range(steps):
        now = i * 0.02
        goal = 0.4 if i < release else 0.0
        ctx = snapshot(now, target=Velocity(0.7, 0), current=body)
        ctx = replace(
            ctx,
            response=replace(ctx.response, valid_until_steady=20.0),
            envelope=replace(
                ctx.envelope,
                valid_until_steady=20.0,
                brake_drive_overshoot=Velocity(0, 0),
                brake_tracking_basis="synthetic whole-stop tracking and settling hypothesis",
            ),
        )
        permit = engine().refine(ctx, now)
        d = c.tick(
            Velocity(goal, 0),
            ctx,
            permit,
            feedback(now, body, i),
            feedback(now, body / 0.9, i, "wheel"),
            now,
        )
        if d.command is not None:
            body += (0.9 * d.command.vx - body) * 0.2
        x += body * 0.02
        rows.append((now, goal, body, x, d))
    return c, rows


def test_real_t4_plant_accelerates_and_release_keeps_braking_until_body_still():
    _, rows = loop()
    assert rows[180][2] == pytest.approx(0.4, abs=0.025)
    assert rows[201][2] > 0.1
    assert rows[-1][2] < 0.005
    assert rows[-1][3] > rows[200][3]
    assert all(r[4].command is None or r[4].command.vx >= 0 for r in rows)
    assert not [r for r in rows[1:] if r[4].command is None]


def test_final_pi_candidate_is_validated_and_integral_bounded():
    c, rows = loop(250, release=240)
    assert abs(c.integral.vx) <= config().integral_limit.vx
    assert all(r[4].command is None or r[4].verified for r in rows)
    assert max(r[4].command.vx for r in rows if r[4].command) <= 0.7


def test_stale_feedback_brakes_not_fake_body_or_instant_zero():
    c, rows = loop(100, release=1000)
    now = 2.0
    ctx = context_for(c, now, rows[-1][2])
    p = engine().refine(ctx, now)
    d = c.tick(Velocity(0.4, 0), ctx, p, feedback(0, 0.4, 1), feedback(0, 0.4, 1, "wheel"), now)
    assert d.reason == "feedback_braking"
    assert d.command is not None and d.command.vx > 0
    assert c.integral == Velocity(0, 0)


def test_long_tick_has_no_unbounded_catchup_and_estop_is_distinct():
    c = MotionController(engine(), config(), Velocity(0.3, 0), Velocity(0, 0))
    ctx = snapshot(current=0.3)
    c.tick(Velocity(0.4, 0), ctx, None, None, None, 0)
    d = c.tick(Velocity(0.4, 0), ctx, None, None, None, 1)
    assert d.command is None and d.reason == "invalid_tick"
    assert c.tick(Velocity(0.4, 0), ctx, None, None, None, 1.01, estop=True).command == Velocity(
        0, 0
    )


def test_degraded_wheel_reference_is_not_used_to_claim_measured_stillness():
    c = MotionController(engine(), config(), Velocity(0.3, 0), Velocity(0, 0))
    for i in range(10):
        now = i * 0.02
        ctx = snapshot(
            now, target=Velocity(-0.3, 0), current=0.1, mode=SourceMode.WHEEL_IMU_LIMITED
        )
        d = c.tick(
            Velocity(-0.3, 0),
            ctx,
            engine().refine(ctx, now),
            None,
            feedback(now, 0, i, "wheel"),
            now,
        )
        assert d.command is None or d.command.vx >= 0
    assert not c.measured_still


def context_for(c, now, body, budget=0.7, *, rear=True, mode=SourceMode.LASER_REFERENCE, cap=1.0):
    s = snapshot(now, target=Velocity(budget, 0), current=body, rear=rear, mode=mode)
    return replace(
        s,
        response=replace(s.response, valid_until_steady=20.0),
        envelope=replace(
            s.envelope,
            valid_until_steady=20.0,
            linear_interval=(body - 0.005, body + 0.005),
            brake_drive_overshoot=Velocity(0, 0),
            brake_tracking_basis="synthetic whole-stop tracking and settling hypothesis",
            reaction_accel_upper=Velocity(
                max(c.trajectory.acceleration.vx * (1 if body >= 0 else -1), 0), 0
            ),
            linear_cap_mps=cap,
        ),
    )


def test_front_block_reverse_waits_for_measured_stop_then_reverses_plant():
    c = MotionController(engine(), config(), Velocity(0.3, 0), Velocity(0, 0))
    body = 0.3
    x = 0.0
    negative = []
    for i in range(500):
        now = i * 0.02
        ctx = context_for(c, now, body, -0.3)
        ctx = replace(
            ctx,
            perception=replace(
                ctx.perception, occupied=ctx.perception.occupied.union(Point(1, 0).buffer(0.02))
            ),
        )
        d = c.tick(
            Velocity(-0.3 if i < 300 else 0, 0),
            ctx,
            engine().refine(ctx, now),
            feedback(now, body, i),
            feedback(now, body, i, "wheel"),
            now,
        )
        if d.command is not None:
            if d.command.vx < 0:
                negative.append((body, x))
            body += (d.command.vx - body) * 0.2
        x += body * 0.02
    assert negative and abs(negative[0][0]) < 0.02
    assert min(v[0] for v in negative) < -0.25 and x < 0
    assert abs(body) < 0.005


def test_rear_unknown_has_no_reverse_and_body_stops():
    c = MotionController(engine(), config(), Velocity(0.3, 0), Velocity(0, 0))
    body = 0.3
    for i in range(250):
        now = i * 0.02
        ctx = context_for(c, now, body, -0.3, rear=False)
        ctx = replace(ctx, envelope=replace(ctx.envelope, linear_interval=(body, body)))
        d = c.tick(
            Velocity(-0.3, 0),
            ctx,
            engine().refine(ctx, now),
            feedback(now, body, i),
            feedback(now, body, i, "wheel"),
            now,
        )
        if d.command is not None:
            assert d.command.vx >= 0
            body += (d.command.vx - body) * 0.2
    assert body < 0.005


def test_mode_cap_reduction_brakes_and_recovery_is_jerk_limited():
    c = MotionController(engine(), config(), Velocity(0.4, 0), Velocity(0, 0))
    body = 0.4
    commands = []
    for i in range(250):
        now = i * 0.02
        degraded = i < 140
        cap = 0.2 if degraded else 0.7
        ctx = context_for(
            c,
            now,
            body,
            mode=SourceMode.WHEEL_IMU_LIMITED if degraded else SourceMode.LASER_REFERENCE,
            cap=cap,
        )
        d = c.tick(
            Velocity(0.4, 0),
            ctx,
            engine().refine(ctx, now),
            None if degraded else feedback(now, body, i),
            feedback(now, body, i, "wheel"),
            now,
        )
        if i > 0:
            assert d.command is not None, d.reason
        if d.command is not None:
            commands.append(d.command.vx)
            body += (d.command.vx - body) * 0.2
    assert commands[130] < 0.21 and commands[-1] > 0.35
    assert (
        max(abs(b - a) for a, b in zip(commands, commands[1:], strict=False)) <= 0.5 * 0.02 + 1e-9
    )


def test_out_of_order_and_unsupported_body_do_not_enter_pi():
    c, _ = loop(100, release=1000)
    now = 2.0
    ctx = context_for(c, now, 0.3)
    d = c.tick(
        Velocity(0.4, 0),
        ctx,
        engine().refine(ctx, now),
        feedback(now, 100, 0),
        feedback(now, 0.3, 100, "wheel"),
        now,
    )
    assert d.reason == "feedback_braking" and c.integral == Velocity(0, 0)
    now += 0.02
    ctx = context_for(c, now, 0.3)
    d = c.tick(
        Velocity(0.4, 0),
        ctx,
        engine().refine(ctx, now),
        feedback(now, 0, 101, supported=False),
        feedback(now, 0.3, 101, "wheel"),
        now,
    )
    assert d.reason == "feedback_braking"


def test_final_rejection_does_not_commit_trajectory_or_integral(monkeypatch):
    e = engine()
    c = MotionController(e, config(), Velocity(0, 0), Velocity(0, 0))
    ctx = context_for(c, 0, 0)
    c.tick(
        Velocity(0.4, 0), ctx, e.refine(ctx, 0), feedback(0, 0, 1), feedback(0, 0, 1, "wheel"), 0
    )
    seen = []

    def reject(result, context, candidate, now):
        seen.append(candidate)
        return False

    monkeypatch.setattr(e, "validate_output", reject)
    ctx = context_for(c, 0.02, 0)
    d = c.tick(
        Velocity(0.4, 0),
        ctx,
        e.refine(ctx, 0.02),
        feedback(0.02, 0, 2),
        feedback(0.02, 0, 2, "wheel"),
        0.02,
    )
    assert d.command is None and d.reason == "final_candidate_rejected"
    assert 0 < seen[0].vx < 0.4
    assert c.trajectory.velocity == Velocity(0, 0) and c.integral == Velocity(0, 0)


def test_reset_needs_independent_fresh_measured_stillness():
    c = MotionController(engine(), config(), Velocity(0.3, 0), Velocity(0, 0))
    assert not c.reset_at_stillness(feedback(0, 0, 1), feedback(0, 0, 1, "wheel"), 1)
    for i in range(4):
        now = 1 + i * 0.05
        reset = c.reset_at_stillness(
            feedback(now, 0, 10 + i), feedback(now, 0, 10 + i, "wheel"), now
        )
    assert reset and c.trajectory.velocity == Velocity(0, 0)
    assert c.integral == Velocity(0, 0)


def test_consumed_permit_deadline_is_not_extended_in_output_receipt():
    c = MotionController(engine(), config(), Velocity(0, 0), Velocity(0, 0))
    ctx = context_for(c, 0, 0)
    p = engine().refine(ctx, 0)
    c.tick(Velocity(0.4, 0), ctx, p, feedback(0, 0, 1), feedback(0, 0, 1, "wheel"), 0)
    ctx = context_for(c, 0.02, 0)
    p = engine().refine(ctx, 0.02)
    p = replace(p, permit=replace(p.permit, deadline_steady=0.025))
    d = c.tick(Velocity(0.4, 0), ctx, p, feedback(0.02, 0, 2), feedback(0.02, 0, 2, "wheel"), 0.02)
    assert d.verified and d.evidence_deadline <= 0.025


def test_slip_does_not_accumulate_integral_to_chase_body_speed():
    c, rows = loop(180, release=1000)
    for i in range(30):
        now = 3.6 + i * 0.02
        ctx = context_for(c, now, 0)
        d = c.tick(
            Velocity(0.4, 0),
            ctx,
            engine().refine(ctx, now),
            feedback(now, 0, 200 + i),
            feedback(now, 0.8, 200 + i, "wheel"),
            now,
        )
        assert c.integral == Velocity(0, 0)
        assert d.reason == "inconsistent_feedback_limited"
    assert c.trajectory.velocity.vx <= 0.4 + 1e-9


def test_t3_partial_fusion_to_real_t4_to_controller_wheel_limited():
    from test_motion_temporal import point

    from senior_control.motion.contracts import BodyMotion
    from senior_control.motion.observations import Source
    from tools.motion_fusion_replay import ReplayChain, replay_policy

    policy = replay_policy()
    policy = replace(
        policy, admission=replace(policy.admission, recovery_samples=1, recovery_span_s=0)
    )
    chain = ReplayChain(policy)
    events = [
        replace(point(Source.IMU, i / 50, i + 1, (0,)), received_steady=i / 50) for i in range(6)
    ]
    events += [
        replace(point(Source.WHEEL, i / 30, i + 1, (0, 0)), received_steady=i / 30)
        for i in range(4)
    ]
    for p in sorted(events, key=lambda p: p.end_ros):
        assert chain.ingest(p, p.end_ros, p.received_steady).accepted
    fusion = chain.window(0, 0.1, 0.1, 0.1)
    assert fusion.average is None and fusion.upper is None
    epoch = "|".join(f"{h.source.value}:{h.epoch}" for h in fusion.source_health)
    partial = BodyMotion(
        None,
        None,
        0,
        0.1,
        0.1,
        min(h.valid_until_steady for h in fusion.source_health),
        epoch,
        True,
        "wheel_imu_limited",
    )
    c = MotionController(engine(), config(), Velocity(0, 0), Velocity(0, 0))
    for i in range(2):
        now = 0.1 + i * 0.02
        ctx = context_for(c, now, 0, mode=fusion.mode)
        ctx = replace(ctx, motion=partial, envelope=replace(ctx.envelope, motion_epoch=epoch))
        d = c.tick(
            Velocity(0.2, 0),
            ctx,
            engine().refine(ctx, now),
            None,
            feedback(now, 0, i, "wheel"),
            now,
        )
    assert d.command.vx > 0 and d.verified
    assert not c.measured_still and c.integral == Velocity(0, 0)


def test_fresh_but_nonresponding_body_exhausts_original_stop_model():
    c = MotionController(engine(), config(), Velocity(0.4, 0), Velocity(0, 0))
    failures = []
    for i in range(180):
        now = i * 0.02
        ctx = context_for(c, now, 0.4)
        d = c.tick(
            Velocity(0, 0), ctx, None, feedback(now, 0.4, i), feedback(now, 0.4, i, "wheel"), now
        )
        if d.reason == "stop_response_failed":
            failures.append(d)
    assert failures and all(d.command is None and d.emergency for d in failures)


def test_latest_collision_risk_outranks_exhausted_brake_supervision():
    c = MotionController(engine(), config(), Velocity(0.4, 0), Velocity(0, 0))
    for i in range(100):
        now = i * 0.02
        ctx = context_for(c, now, 0.4)
        c.tick(
            Velocity(0, 0), ctx, None, feedback(now, 0.4, i), feedback(now, 0.4, i, "wheel"), now
        )
    now = 2.0
    ctx = snapshot(now, target=Velocity(0, 0), current=0.4, hits=((0.5, 0),))
    d = c.tick(
        Velocity(0, 0), ctx, None, feedback(now, 0.4, 100), feedback(now, 0.4, 100, "wheel"), now
    )
    assert d.emergency and d.reason == "stop_envelope_blocked"


def test_controller_and_reducer_estop_reset_never_restart_old_manual_target():
    from senior_control.motion.contracts import MotionEvent, MotionMode
    from senior_control.motion.contracts import MotionEventKind as K
    from senior_control.motion.state_machine import MotionStateMachine, RecoveryObservation

    c = MotionController(engine(), config(), Velocity(0.3, 0), Velocity(0, 0))
    m = MotionStateMachine()
    m.reduce(MotionEvent(K.SOFTWARE_ESTOP, "user", 0, 1), now_steady=0)
    body = 0.3
    for i in range(40):
        now = i * 0.02
        ctx = context_for(c, now, body)
        m.observe(RecoveryObservation("body", i, now, now + 0.08, True, body < 0.02), now)
        d = c.tick(
            Velocity(0.4, 0),
            ctx,
            None,
            feedback(now, body, i),
            feedback(now, body, i, "wheel"),
            now,
            estop=True,
        )
        body += (d.command.vx - body) * 0.2
    now = 0.8
    m.observe(RecoveryObservation("body", 40, now, now + 0.08, True, body < 0.02), now)
    state = m.reduce(MotionEvent(K.ESTOP_RESET, "user", m.state.estop_revision, 2), now_steady=now)
    assert state.mode is MotionMode.IDLE
    assert c.reset_at_stillness(feedback(now, body, 40), feedback(now, body, 40, "wheel"), now)
    now = 0.82
    ctx = context_for(c, now, body)
    d = c.tick(
        Velocity(0, 0), ctx, None, feedback(now, body, 41), feedback(now, body, 41, "wheel"), now
    )
    assert d.command == Velocity(0, 0) and body < 0.005


def test_expired_intent_continues_jerk_braking_not_zero_assignment():
    c, rows = loop(160, release=1000)
    now = 3.2
    ctx = context_for(c, now, rows[-1][2])
    ctx = replace(ctx, target=replace(ctx.target, deadline_steady=3.1))
    d = c.tick(
        Velocity(0.4, 0),
        ctx,
        None,
        feedback(now, rows[-1][2], 160),
        feedback(now, rows[-1][2], 160, "wheel"),
        now,
    )
    assert d.command is not None and d.command.vx > 0.1 and d.verified
    assert d.reason == "target_expired"


def test_infeasible_jerk_step_reports_failure_without_inventing_safe_zero():
    c = MotionController(engine(), config(), Velocity(0.001, 0), Velocity(-0.5, 0))
    ctx = context_for(c, 0, 0.001)
    c.tick(Velocity(0, 0), ctx, None, feedback(0, 0.001, 1), feedback(0, 0.001, 1, "wheel"), 0)
    ctx = context_for(c, 0.02, 0.001)
    d = c.tick(
        Velocity(0, 0), ctx, None, feedback(0.02, 0.001, 2), feedback(0.02, 0.001, 2, "wheel"), 0.02
    )
    assert d.command is None and d.emergency and d.reason == "infeasible_no_reverse"
    assert c.trajectory.velocity.vx == 0.001


def test_rate_saturation_freezes_integral_instead_of_winding_up_at_start():
    c = MotionController(engine(), config(), Velocity(0, 0), Velocity(0, 0))
    for i in range(10):
        now = i * 0.02
        ctx = context_for(c, now, 0)
        d = c.tick(
            Velocity(0.4, 0),
            ctx,
            engine().refine(ctx, now),
            feedback(now, 0, i),
            feedback(now, 0, i, "wheel"),
            now,
        )
        assert c.integral == Velocity(0, 0)
    assert d.command.vx > 0


def test_combined_wheel_limit_reaches_curvature_preserving_target_without_tick_clipping():
    from senior_control.models import BodyTwist

    cfg = replace(config(), kp=Velocity(0, 0), ki=Velocity(0, 0), wheel_speed_mps=0.6)
    c = MotionController(engine(), cfg, Velocity(0, 0), Velocity(0, 0))
    body = Velocity(0, 0)
    commands = []
    for i in range(180):
        now = i * 0.02
        ctx = context_for(c, now, body.vx)
        ctx = replace(
            ctx,
            target=replace(ctx.target, velocity=Velocity(0.7, 1)),
            motion=replace(ctx.motion, average=BodyTwist(body.vx, 0, body.wz)),
            envelope=replace(ctx.envelope, yaw_interval=(body.wz, body.wz)),
        )
        b = replace(feedback(now, body.vx, i), velocity=body)
        w = replace(b, epoch="wheel")
        d = c.tick(Velocity(0.5, 1), ctx, engine().refine(ctx, now), b, w, now)
        if i:
            assert d.command is not None, d.reason
        if d.command:
            v = d.command
            commands.append(v)
            assert max(abs(v.vx - v.wz * 0.2), abs(v.vx + v.wz * 0.2)) <= 0.6 + 1e-12
            body = Velocity(body.vx + (v.vx - body.vx) * 0.2, body.wz + (v.wz - body.wz) * 0.2)
    assert commands[-1].wz / commands[-1].vx == pytest.approx(2, abs=1e-6)


def test_real_wheel_cap_change_is_not_treated_as_numerical_roundoff():
    cfg = replace(config(), wheel_speed_mps=0.2)
    c = MotionController(engine(), cfg, Velocity(0.6, 0), Velocity(0, 0))
    ctx = context_for(c, 0, 0.6)
    c.tick(Velocity(0, 0), ctx, None, feedback(0, 0.6, 1), feedback(0, 0.6, 1, "wheel"), 0)
    ctx = context_for(c, 0.02, 0.6)
    d = c.tick(
        Velocity(0, 0), ctx, None, feedback(0.02, 0.6, 2), feedback(0.02, 0.6, 2, "wheel"), 0.02
    )
    assert d.command is None and d.reason == "wheel_limit_infeasible_step"
    assert c.trajectory.velocity.vx == 0.6


def test_body_stop_proof_does_not_authorize_high_historical_drive():
    c = MotionController(engine(), config(), Velocity(0.8, 0), Velocity(0, 0))
    for i in range(2):
        now = i * 0.02
        ctx = context_for(c, now, 0)
        ctx = replace(
            ctx,
            envelope=replace(ctx.envelope, linear_interval=(0, 0)),
            perception=replace(ctx.perception, occupied=Point(0.5, 0).buffer(0.02)),
        )
        d = c.tick(
            Velocity(0, 0), ctx, None, feedback(now, 0, i), feedback(now, 0.8, i, "wheel"), now
        )
    assert d.command is None and not d.verified


@pytest.mark.parametrize(
    "command,acceleration",
    [(Velocity(0.4, 0.3), Velocity(0.2, 0.2)), (Velocity(-0.4, -0.3), Velocity(-0.2, -0.2))],
)
def test_braking_drive_model_binds_combined_path_and_original_expiry(command, acceleration):
    c = MotionController(engine(), config(), command, acceleration)
    ctx = context_for(c, 0, 0)
    proof = engine().validate_braking(ctx, command, acceleration, command, 0)
    assert proof.stop_verified
    assert not engine().validate_braking(ctx, command, acceleration, command, 3).stop_verified
    missing = replace(ctx, envelope=replace(ctx.envelope, brake_drive_overshoot=None))
    assert not engine().validate_braking(missing, command, acceleration, command, 0).stop_verified


def test_normal_body_lag_does_not_expire_before_drive_stop_family():
    c = MotionController(engine(), config(), Velocity(0.4, 0), Velocity(0, 0))
    body = 0.05
    wheel = 0.4
    for i in range(130):
        now = i * 0.02
        ctx = context_for(c, now, body)
        d = c.tick(
            Velocity(0, 0), ctx, None, feedback(now, body, i), feedback(now, wheel, i, "wheel"), now
        )
        if i:
            assert d.command is not None, d.reason
        if d.command is not None:
            wheel = d.command.vx
            body += (wheel - body) * 0.2
    assert body < 0.005
