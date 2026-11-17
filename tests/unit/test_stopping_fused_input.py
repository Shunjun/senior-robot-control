"""Actual output snapshot + constraint engine, with synthetic observed geometry only."""

from pathlib import Path
from types import SimpleNamespace

import pytest
from senior_robot.motion_core_output import MotionCoreOutput
from senior_robot.motion_sensor_runtime import MotionSensorRuntime
from sensor_time_test_support import sample_time
from shapely.geometry import GeometryCollection, Point, box
from test_unified_motion_runtime import point

from senior_control.config import DeviceProfile
from senior_control.models import Pose2D, Velocity
from senior_control.motion.constraints import ConstraintConfig, ConstraintEngine, Perception
from senior_control.motion.contracts import MotionTarget
from senior_control.motion.controller import MotionController
from senior_control.motion.hardware_profile import load_hardware_profile
from senior_control.motion.observations import Source
from senior_control.motion.stopping_input import StoppingRisk
from senior_control.runtime.diagnostics import RuntimeDiagnostics

ROOT = Path(__file__).resolve().parents[2]


def output(speed=0.0, yaw=0.0, imu=False):
    p = load_hardware_profile(ROOT / "config/hardware-motion.json")
    core = MotionCoreOutput.__new__(MotionCoreOutput)
    core.profile = p
    core.node = SimpleNamespace(_profile=lambda: DeviceProfile())
    core.diagnostics = RuntimeDiagnostics(lambda *_: None)
    core.sensors = MotionSensorRuntime(p, core.diagnostics)
    core.acceleration_model = p.acceleration_model()
    core.stopping = StoppingRisk()
    core.engine = ConstraintEngine(ConstraintConfig((1.0, 0.75, 0.5, 0.25), 0.15, 0.05, 256, 0.005))
    core.controller = MotionController(core.engine, p.controller(), Velocity(0, 0), Velocity(0, 0))
    core.target = MotionTarget(Velocity(-0.15, 0), "test", "boot", 21.0)
    core.sensors.ingest(point(Source.WHEEL, 10, 20, (speed, yaw)), 10, 20, 10)
    if imu:
        core.sensors.ingest(point(Source.IMU, 10, 20, (yaw,)), 10, 20, 10)
    return core


def snapshot(core, now=20.0, obstacle=None):
    ref, body, wheel = core.sensors.estimate(now - 10, now)
    occupied = GeometryCollection() if obstacle is None else obstacle
    scene = Perception(
        box(-20, -20, 20, 20),
        occupied,
        now,
        now + 1,
        "scan",
        "scan_reference",
        1,
        core.profile.horizon_s,
        "synthetic observed free space and obstacle",
    )
    return core._snapshot(ref, wheel, now, scene, Pose2D(0, 0, 0))


@pytest.mark.parametrize("speed,imu", [(0.0, False), (0.02, False), (0.0, True)])
def test_stationary_and_low_speed_do_not_invent_front_inertia(speed, imu):
    core = output(speed, imu=imu)
    ctx = snapshot(core, obstacle=Point(1.0, 0))
    assert ctx.envelope.linear_interval == pytest.approx((speed - 0.02, speed + 0.02))
    result = core.engine.refine(ctx, 20)
    assert result.brake.stop_verified
    assert result.brake.reason == "no_clear_candidate"


def test_repository_hardware_deceleration_is_operator_assumption():
    p = output().profile
    assert p.decel == 0.5
    assert "ASSUMED" in p.evidence_basis and "no physical measurements" in p.evidence_basis


@pytest.mark.parametrize("speed,imu", [(0.0, False), (0.02, False), (0.0, True)])
def test_reverse_with_front_obstacle_and_observed_rear_clearance(speed, imu):
    core = output(speed, imu=imu)
    ctx = snapshot(core, obstacle=Point(2.0, 0))
    assert core.engine.refine(ctx, 20).permit is not None


def test_real_forward_inertia_blocks_reverse_at_original_front_obstacle():
    core = output(0.8)
    ctx = snapshot(core, obstacle=Point(1.0, 0))
    result = core.engine.refine(ctx, 20)
    assert result.permit is None
    assert result.brake.reason == "stop_envelope_blocked"
    assert not result.brake.stop_verified


def test_forward_motion_requires_braking_before_reversing_even_in_clear_space():
    core = output(0.2)
    ctx = snapshot(core)
    result = core.engine.refine(ctx, 20)
    assert ctx.envelope.linear_interval[0] > 0
    assert result.permit is None and result.brake.reason == "reverse_requires_stop"


def test_turning_sweep_keeps_real_side_obstacle():
    core = output(0.0, 0.5, imu=True)
    core.target = MotionTarget(Velocity(0, 0.5), "test", "boot", 21)
    ctx = snapshot(core, obstacle=Point(0.6, 0.4))
    result = core.engine.refine(ctx, 20)
    assert result.permit is None and result.brake.reason == "stop_envelope_blocked"


def test_failed_laser_with_live_wheel_still_uses_unified_result():
    from senior_control.motion.estimator import MotionObservation, Origin

    core = output(0.02, imu=True)
    laser = MotionObservation(
        "laser",
        (0.02, None, 0.0),
        10,
        (
            Origin(
                "laser",
                "boot",
                1,
                sample_time(
                    stamp_ns=round((10) * 1e9),
                    received_ros_ns=round((10) * 1e9),
                    received_steady_ns=round((20) * 1e9),
                ),
            ),
        ),
        independent=(True, True, True),
        measured_steady=20,
        stamp_ns=round((10) * 1e9),
    )
    core.sensors.ingest_observation(laser, 10, 20)
    snapshot(core)
    core.sensors.invalidate(Source.LASER)
    ctx = snapshot(core, obstacle=Point(2, 0))
    assert core.sensors.independent is None
    assert core.sensors.latest.vx.sources == ("wheel",)
    assert ctx.envelope.linear_interval == pytest.approx((0, 0.04))
    assert core.engine.refine(ctx, 20).permit is not None


@pytest.mark.parametrize("newer", ["wheel", "laser"])
def test_conflict_stopping_uses_owner_choice_not_independent_view(newer):
    from senior_control.motion.estimator import MotionObservation, Origin

    core = output()
    core.sensors.invalidate(Source.WHEEL)
    for name, value in (("wheel", -0.02), ("laser", 0.8)):
        stamp = 10 if name == newer else 9.9
        o = MotionObservation(
            name,
            (value, None, 0.0),
            stamp,
            (
                Origin(
                    name,
                    "boot2",
                    1,
                    sample_time(
                        stamp_ns=round((stamp) * 1e9),
                        received_ros_ns=round((stamp) * 1e9),
                        received_steady_ns=round((stamp + 10) * 1e9),
                    ),
                ),
            ),
            independent=(name == "laser",) * 3,
            measured_steady=stamp + 10,
            stamp_ns=round((stamp) * 1e9),
        )
        core.sensors.ingest_observation(o, 10, 20)
    ctx = snapshot(core, obstacle=Point(1, 0))
    chosen = core.sensors.latest.vx
    assert chosen.sources == (newer,)
    assert ctx.envelope.linear_interval == pytest.approx((chosen.value - 0.02, chosen.value + 0.02))
    result = core.engine.refine(ctx, 20)
    if newer == "laser":
        assert result.brake.reason == "stop_envelope_blocked"
    else:
        assert result.permit is not None


@pytest.mark.parametrize("age", [0.3, 0.6, 0.999])
def test_soft_age_is_usable_and_prediction_error_charged_once(age):
    core = output()
    ctx = snapshot(core, 20 + age)
    axis = core.sensors.latest.vx
    assert axis.band == "low" and axis.observed
    error = 0.02 + 0.3 * age
    assert ctx.envelope.linear_interval == pytest.approx((-error, error))
    assert ctx.motion.valid_until_steady == 21
    assert core.engine._context_error(ctx, 20 + age) is None


def test_one_second_expiry_is_not_renewed_by_model():
    core = output()
    snapshot(core, 20.999)
    with pytest.raises(ValueError, match="all_relevant_motion_feedback_unavailable"):
        snapshot(core, 21)


def test_missing_axis_hold_keeps_own_cutoff_then_closes_only_that_target_axis():
    core = output(0.04)
    snapshot(core)
    core.sensors.invalidate(Source.WHEEL)
    core.sensors.ingest(point(Source.IMU, 10.4, 20.4, (0.0,)), 10.4, 20.4, 10.4)
    held = snapshot(core, 20.5)
    assert core.sensors.latest.vx.reason == "model_only_bounded_hold"
    assert held.motion.valid_until_steady == 21
    assert held.target.velocity.vx == -0.15
    expired = snapshot(core, 21)
    assert core.sensors.latest.vx.value is None
    assert expired.target.velocity.vx == 0
    assert expired.envelope.linear_interval[0] <= held.envelope.linear_interval[0]
    assert expired.envelope.linear_interval[1] >= held.envelope.linear_interval[1]
    assert expired.motion.valid_until_steady == pytest.approx(21.4)


def test_known_error_does_not_double_charge_projection_or_switch_to_independent_axis():
    from dataclasses import replace

    core = output(0.02)
    first = snapshot(core, 20.5)
    r = core.sensors.latest
    # Known total error already includes .15 age projection.
    ref, body, wheel = core.sensors.estimate(10.5, 20.5)
    core.sensors.latest = replace(r, vx=replace(r.vx, error=0.17))
    ctx = core._snapshot(ref, wheel, 20.5, first.perception, Pose2D(0, 0, 0))
    assert ctx.envelope.linear_interval == pytest.approx((-0.15, 0.19))


def test_decel_changes_model_duration_without_changing_age_or_other_parameters():
    from dataclasses import replace

    core = output(1.5)
    ctx = snapshot(core)
    prediction = core.engine.stop_prediction(ctx)
    old = replace(ctx, response=replace(ctx.response, decel=0.1))
    old_prediction = core.engine.stop_prediction(old)
    assert old_prediction.horizon_s > ctx.envelope.max_horizon_s
    assert prediction.horizon_s < ctx.envelope.max_horizon_s
    assert prediction.linear.distance < old_prediction.linear.distance
    assert old.motion.valid_until_steady == ctx.motion.valid_until_steady == 21
    assert core.engine.refine(old, 20).brake.reason == "model_horizon_exceeded"


def test_stopping_diagnostics_share_periodic_debug_and_distinguish_geometry_age_and_horizon():
    import logging

    events = []
    core = output(0.8)
    core.diagnostics = RuntimeDiagnostics(lambda level, values: events.append((level, values)))
    core.diagnostics.pump(20)
    ctx = snapshot(core, obstacle=Point(1, 0))
    evaluation = core.engine.evaluate(ctx, 20)
    result = evaluation.zero_stop()
    for _ in range(50):
        core._diagnose_stopping(ctx, result.brake.reason, evaluation)
    core.diagnostics.pump(24.99)
    assert not events
    core.diagnostics.pump(25)
    assert len(events) == 1 and events[0][0] == logging.DEBUG
    report = events[0][1]["summaries"]["stopping"]
    assert report["constraint_reason"] == "stop_envelope_blocked"
    assert report["axis_age_s"] == [0.0, 0.0]
    assert report["predicted_stop_s"] < report["model_allowed_s"]
    assert not report["model_horizon_exceeded"]
    assert report["linear_stop_distance_m"] > 0.5


@pytest.mark.parametrize(
    "speed,obstacle,expected",
    [(0.0, Point(2, 0), "tracking"), (0.8, Point(1, 0), "stop_envelope_blocked")],
)
def test_actual_snapshot_constraints_and_controller_output(speed, obstacle, expected):
    core = output(speed)
    ctx = snapshot(core, obstacle=obstacle)
    permission = core.engine.refine(ctx, 20)
    core.controller._last_now = 19.95
    prepared = core.controller.propose_tick(
        ctx.target.velocity, ctx, permission, core.sensors.tracking, core.sensors.tracking, 20
    )
    assert prepared.decision.reason == expected
    if expected == "tracking":
        assert prepared.decision.command.vx < 0 and prepared.decision.verified
    else:
        assert prepared.decision.emergency


def test_hardware_profile_schema_and_launch_resolver_preserve_other_parameters():
    import json

    from senior_robot.motion_profile import select_motion_profile

    from senior_control.motion.hardware_profile import HardwareMotionProfile

    schema = json.loads((ROOT / "config/hardware-motion.schema.json").read_text())
    assert schema == HardwareMotionProfile.model_json_schema()
    p = select_motion_profile(
        hardware_path=str(ROOT / "config/hardware-motion.json"),
    )
    assert p.decel == 0.5
    assert (p.accel, p.yaw_decel, p.jerk, p.horizon_s) == (0.2, 0.15, 0.4, 12)


@pytest.mark.parametrize("fraction", [-1.0, -0.5, 0.5, 1.0])
def test_hardware_manual_degraded_default_and_percentage_remain_owned_by_setting(fraction):
    from senior_control.motion.manual_target import ManualLinearTarget

    core = output()
    core.target = MotionTarget(
        Velocity(fraction * 0.55, 0),
        "manual",
        "boot",
        21,
        ManualLinearTarget(fraction, 0.55, 0.3),
    )
    ctx = snapshot(core, obstacle=Point(2, 0))
    assert ctx.target.velocity.vx == pytest.approx(fraction * 0.3)
    assert ctx.envelope.linear_cap_mps == 0.3
    assert core.engine.refine(ctx, 20).permit is not None


def test_nonzero_trajectory_residual_survives_opposite_fused_velocity():
    from senior_control.motion.trajectory import Trajectory

    core = output(-0.02)
    core.controller.trajectory = Trajectory(Velocity(0.8, 0), Velocity(0.1, 0))
    ctx = snapshot(core, obstacle=Point(1, 0))
    assert ctx.envelope.linear_interval == pytest.approx((-0.04, 0.8))
    assert ctx.envelope.reaction_accel_upper.vx == 0.1
    assert core.engine.refine(ctx, 20).brake.reason == "stop_envelope_blocked"


@pytest.mark.parametrize("trajectory", [0.0, 0.2])
def test_expired_linear_axis_does_not_erase_inertia_while_imu_remains_fresh(trajectory):
    from senior_control.motion.trajectory import Trajectory

    core = output(0.8, imu=True)
    core.target = MotionTarget(Velocity(0, 0.2), "test", "boot", 30)
    core.controller.trajectory = Trajectory(Velocity(trajectory, 0), Velocity(0, 0))
    snapshot(core, 20, Point(1, 0))
    core.sensors.ingest(point(Source.IMU, 10.9, 20.9, (0.0,), 2), 10.9, 20.9, 10.9)
    before = snapshot(core, 20.999, Point(1, 0))
    after = snapshot(core, 21, Point(1, 0))
    assert core.sensors.latest.vx.value is None
    assert core.sensors.latest.wz.observed
    assert core.sensors.latest.vx.deadline_steady == 21
    assert after.envelope.linear_interval[1] >= before.envelope.linear_interval[1]
    permission = core.engine.refine(after, 21)
    assert permission.permit is None and permission.brake.reason == "stop_envelope_blocked"
    core.controller._last_now = 20.95
    decision = core.controller.propose_tick(
        after.target.velocity, after, permission, core.sensors.tracking, core.sensors.tracking, 21
    ).decision
    assert decision.emergency and decision.command is None


def missing_linear(core, now, sequence=2):
    core.sensors.ingest(point(Source.IMU, now - 10, now, (0.0,), sequence), now - 10, now, now - 10)
    return snapshot(core, now, Point(1, 0))


@pytest.mark.parametrize("trajectory", [0.0, 0.2])
def test_only_actual_zero_output_can_start_missing_axis_response_and_settle(trajectory):
    from senior_control.motion.trajectory import Trajectory

    core = output(0.8, imu=True)
    core.target = MotionTarget(Velocity(0, 0.2), "test", "boot", 40)
    core.controller.trajectory = Trajectory(Velocity(trajectory, 0), Velocity(0, 0))
    snapshot(core)
    lost = missing_linear(core, 21)
    later = missing_linear(core, 22, 3)
    assert later.envelope.linear_interval[1] >= lost.envelope.linear_interval[1]
    assert core.stopping.linear.brake is None  # A zero target/trajectory did nothing.
    assert later.envelope.reaction_accel_upper.vx == core._response().accel
    assert core.engine.refine(later, 22).brake.reason == "stop_envelope_blocked"
    core._record_stopping_output(Velocity(0, 0), 22)
    started = core.stopping.linear.at
    for now, seq in ((22.1, 4), (23.0, 5), (30.0, 6)):
        ctx = missing_linear(core, now, seq)
        core._record_stopping_output(Velocity(0, 0), now)
        assert core.stopping.linear.at == started
    assert ctx.envelope.linear_interval == (0.0, 0.0)
    assert core.stopping.linear.reason == "assumed_settled"
    assert core.stopping.linear.support_deadline == 21
    assert core.sensors.latest.vx.value is None and not core.sensors.latest.vx.observed
    assert core.engine.refine(ctx, 30).permit is not None  # Other axis can continue.
    assert not core.sensors.stationary(now)  # Model settling cannot reset a fault.


def test_nonzero_output_cancels_settling_and_fresh_observation_replaces_residual():
    core = output(0.8, imu=True)
    snapshot(core)
    missing_linear(core, 21)
    core._record_stopping_output(Velocity(0, 0), 21)
    missing_linear(core, 21.5, 3)
    core._record_stopping_output(Velocity(0.2, 0), 21.5)
    assert core.stopping.linear.brake is None
    core.sensors.ingest(point(Source.WHEEL, 11.6, 21.6, (0.0, 0.0), 2), 11.6, 21.6, 11.6)
    restored = snapshot(core, 21.6, Point(1, 0))
    assert restored.envelope.linear_interval == pytest.approx((-0.02, 0.02))
    assert core.sensors.latest.vx.observed
    assert core.engine._stop(restored, 21.6).stop_verified


def test_diagnostics_reuse_tick_prediction_and_keep_transient_reason_counts_and_peaks(monkeypatch):
    events = []
    core = output(0.8)
    core.diagnostics = RuntimeDiagnostics(lambda _level, value: events.append(value))
    core.diagnostics.pump(20)
    blocked = snapshot(core, 20, Point(1, 0))
    evaluation = core.engine.evaluate(blocked, 20)
    result = evaluation.zero_stop()
    prediction = evaluation.stop_prediction

    def forbidden(*args, **kwargs):
        pytest.fail("diagnostic recomputed an existing stopping prediction")

    monkeypatch.setattr(core.engine, "stop_prediction", forbidden)
    core._diagnose_stopping(blocked, result.brake.reason, evaluation)
    monkeypatch.undo()
    core.sensors.ingest(point(Source.WHEEL, 10.1, 20.1, (0.0, 0.0), 2), 10.1, 20.1, 10.1)
    clear = snapshot(core, 20.1)
    clear_evaluation = core.engine.evaluate(clear, 20.1)
    clear_evaluation.zero_stop()
    core._diagnose_stopping(clear, "controlled_stop", clear_evaluation)
    core.diagnostics.pump(25)
    assert len(events) == 1
    assert events[0]["summaries"]["stopping"]["constraint_reason"] == "controlled_stop"
    assert events[0]["streams"]["stopping.reason.stop_envelope_blocked"]["count"] == 1
    assert events[0]["streams"]["stopping.reason.controlled_stop"]["count"] == 1
    assert events[0]["streams"]["stopping.predicted_s"]["max"] == prediction.horizon_s


def test_model_settling_never_turns_total_feedback_loss_into_live_motion():
    core = output(0.8)
    snapshot(core)
    with pytest.raises(ValueError, match="all_relevant_motion_feedback_unavailable"):
        snapshot(core, 21)
    core._record_stopping_output(Velocity(0, 0), 21)
    with pytest.raises(ValueError, match="all_relevant_motion_feedback_unavailable"):
        snapshot(core, 30)
    assert not core.sensors.latest.healthy
    assert core.sensors.tracking is None
