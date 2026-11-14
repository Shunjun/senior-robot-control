"""Synthetic scoped stop-model contracts; never independent body measurements."""

from dataclasses import replace

from senior_robot.simulation_motion_profile import SimulationMotionProfile
from test_directional_constraints import scene

from senior_control.models import Pose2D, Velocity
from senior_control.motion.constraints import (
    ConstraintConfig,
    ConstraintEngine,
    ConstraintSnapshot,
    Footprint,
    RiskEnvelope,
)
from senior_control.motion.contracts import BodyMotion, MotionTarget, ResponseBounds
from senior_control.motion.controller import Feedback, MotionController
from senior_control.motion.source_modes import SourceMode


def context(now, target=None):
    target = target if target is not None else Velocity(0, 0)
    p = SimulationMotionProfile()
    return ConstraintSnapshot(
        MotionTarget(target, "manual", "new-authority", now + 0.2),
        BodyMotion(
            None, None, now - 0.1, now, now, now + 0.1, "wheel|imu", True, "synthetic unknown body"
        ),
        SourceMode.WHEEL_IMU_LIMITED,
        ResponseBounds(
            p.accel,
            p.decel,
            p.jerk,
            p.yaw_accel,
            p.yaw_decel,
            p.yaw_jerk,
            p.lag,
            200.0,
            p.profile_id,
        ),
        Footprint(p.length, p.width),
        scene(now),
        Pose2D(0, 0, 0),
        "local",
        p.profile_id,
        p.profile_id,
        RiskEnvelope(
            p.profile_id,
            "synthetic simulation-only passive floor",
            "wheel|imu",
            (SourceMode.WHEEL_IMU_LIMITED,),
            p.profile_id,
            200.0,
            12.0,
            (-p.assumed_body_cap, p.assumed_body_cap),
            (-0.1, 0.1),
            Velocity(0, 0),
            p.position_error,
            0.0,
            0.0,
            p.degraded_cap,
            p.yaw_cap,
            True,
            brake_drive_overshoot=Velocity(0.1, 0.1),
            brake_tracking_basis="simulation-only whole drive stop family; unvalidated",
        ),
    )


def point(now, sequence, value=None, epoch="wheel"):
    value = value if value is not None else Velocity(0, 0)
    return Feedback(
        value, now, now + 0.1, epoch, sequence, True, "synthetic wheel/IMU point, not body"
    )


def stopped_controller():
    e = ConstraintEngine(ConstraintConfig((1.0, 0.75, 0.5, 0.25), 0.15, 0.05, 256, 0.005))
    c = MotionController(
        e, SimulationMotionProfile().controller(), Velocity(0.277, 0), Velocity(0, 0)
    )
    deadlines = []
    for i in range(170):
        now = 10 + i * 0.05
        ctx = context(now)
        before = c.trajectory.velocity
        decision = c.tick(
            Velocity(0, 0),
            ctx,
            e.refine(ctx, now),
            None,
            point(now, i, before),
            now,
            imu=point(now, i, epoch="imu"),
        )
        if c._brake_deadline is not None:
            deadlines.append(c._brake_deadline)
        if deadlines and now < deadlines[0]:
            assert not c.limited_rearm_ready(now)
        assert decision.command is not None or i == 0
    return c, now, deadlines


def test_completed_stop_is_manual_permission_not_body_stillness():
    c, now, deadlines = stopped_controller()
    assert len(set(deadlines)) == 1
    assert c.limited_rearm_ready(now)
    assert not c.measured_still
    assert c.trajectory.velocity == Velocity(0, 0) and c.trajectory.acceleration == Velocity(0, 0)
    assert not c.limited_rearm_ready(now + 0.11)


def test_epoch_change_and_missing_model_remove_permission():
    c, now, _ = stopped_controller()
    now += 0.05
    ctx = context(now)
    c.tick(
        Velocity(0, 0),
        ctx,
        c.engine.refine(ctx, now),
        None,
        point(now, 200),
        now,
        imu=point(now, 200, epoch="replacement"),
    )
    assert not c.limited_rearm_ready(now)
    ctx = replace(ctx, envelope=replace(ctx.envelope, brake_drive_overshoot=None))
    c.tick(
        Velocity(0, 0),
        ctx,
        None,
        None,
        point(now, 201),
        now + 0.001,
        imu=point(now, 201, epoch="replacement"),
    )
    assert not c.limited_rearm_ready(now + 0.001)


def test_manual_only_readiness_never_clears_safety_or_grants_task_actions():
    from senior_control.motion.contracts import MotionEvent
    from senior_control.motion.contracts import MotionEventKind as K
    from senior_control.motion.state_machine import MotionStateMachine, RecoveryObservation

    m = MotionStateMachine()

    def observe(t, seq):
        m.observe(
            RecoveryObservation("limited", seq, t, t + 0.1, False, False, manual_rearm_ready=True),
            t,
        )

    observe(10.0, 1)
    assert m.state.mode.value == "braking" and m.state.allowed_actions == ("acquire",)
    assert not m.state.stationary
    m.reduce(MotionEvent(K.SOFTWARE_ESTOP, "user", 0, 1), now_steady=10.0)
    observe(10.05, 2)
    m.reduce(MotionEvent(K.ESTOP_RESET, "user", m.state.estop_revision, 2), now_steady=10.05)
    assert m.state.mode.value == "estop" and "acquire" not in m.state.allowed_actions
    f = MotionStateMachine()
    f.reduce(MotionEvent(K.EXECUTION_FAULT, "owner", 0, 1), now_steady=10.0)
    f.observe(
        RecoveryObservation("limited", 1, 10.0, 10.1, False, False, manual_rearm_ready=True), 10.0
    )
    f.reduce(MotionEvent(K.FAULT_RECOVER, "user", f.state.fault_revision, 2), now_steady=10.0)
    assert f.state.mode.value == "execution_fault" and "acquire" not in f.state.allowed_actions


def test_unvalidated_simulation_model_is_not_enabled_for_hardware():
    c, now, _ = stopped_controller()
    c.config = replace(c.config, synthetic=False)
    now += 0.05
    ctx = context(now)
    c.tick(
        Velocity(0, 0),
        ctx,
        c.engine.refine(ctx, now),
        None,
        point(now, 200),
        now,
        imu=point(now, 200, epoch="imu"),
    )
    assert not c.limited_rearm_ready(now)


def test_manual_rearm_consumes_pending_task_auto_replan():
    from senior_control.motion.contracts import MotionEvent
    from senior_control.motion.contracts import MotionEventKind as K
    from senior_control.motion.state_machine import MotionStateMachine, RecoveryObservation

    m = MotionStateMachine()
    m.observe(RecoveryObservation("body", 1, 1.0, 1.1, True, False), 1.0)
    m.reduce(MotionEvent(K.TASK_START, "task", 0, 1), now_steady=1.0)
    m.reduce(MotionEvent(K.FEEDBACK_LOST, "source", 0, 2), now_steady=1.01)
    m.observe(RecoveryObservation("limited", 1, 1.05, 1.15, False, False, True), 1.05)
    m.reduce(MotionEvent(K.FEEDBACK_HEALTHY, "source", 0, 3), now_steady=1.05)
    m.reduce(MotionEvent(K.MANUAL_START, "new-authority", 0, 4), now_steady=1.05)
    for seq in range(8):
        now = 1.1 + seq * 0.1
        m.observe(RecoveryObservation("body", seq, now, now + 0.11, True, False), now)
    m.reduce(MotionEvent(K.FEEDBACK_HEALTHY, "source", 0, 5), now_steady=now)
    assert m.state.mode.value == "manual"
    assert "replan_task" not in m.state.allowed_actions


def test_current_proof_veto_revokes_permission_without_recounting_sensor_sample():
    from senior_control.motion.state_machine import MotionStateMachine, RecoveryObservation

    m = MotionStateMachine()
    m.observe(RecoveryObservation("limited", 1, 10.0, 10.1, False, False, True), 10.0)
    assert "acquire" in m.state.allowed_actions
    m.observe(RecoveryObservation("limited", 2, 10.0, 10.1, False, False, False), 10.01)
    assert "acquire" not in m.state.allowed_actions
    assert not m.state.stationary
