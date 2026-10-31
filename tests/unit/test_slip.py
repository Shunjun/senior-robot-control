import math

from senior_control.models import Pose2D, Velocity
from senior_control.safety.motion_estimator import MotionEstimate, WheelOdom
from senior_control.safety.slip import SlipMonitor


def monitor() -> SlipMonitor:
    return SlipMonitor(
        startup_window=0.5,
        wheel_motion_limit=0.05,
        lateral_limit=0.03125,
        yaw_limit=0.125,
    )


def body(
    x: float = 0.0,
    y: float = 0.0,
    yaw: float = 0.0,
    *,
    dt: float = 0.25,
    vx: float = 0.0,
    wz: float = 0.0,
) -> MotionEstimate:
    return MotionEstimate(Pose2D(x, y, yaw), Velocity(vx, wz), (0.25, 0.25, 0.25), 1.0, dt)


def still_body() -> MotionEstimate:
    return body()


def wheel(distance: float, dt: float = 0.25) -> WheelOdom:
    return WheelOdom(Pose2D(distance, 0.0, 0.0), 10.0, dt)


def test_wheels_spin_without_body_motion_stops():
    checker = monitor()
    command = Velocity(0.2, 0.0)
    early = checker.update(command, wheel(0.0625), still_body())
    mid = checker.update(command, wheel(0.0625), still_body())
    late = checker.update(command, wheel(0.0625), still_body())
    assert early.anomalous is False
    assert early.velocity == command
    assert early.increase_torque is False
    assert mid.anomalous is False
    assert late.anomalous is True
    assert late.velocity == Velocity(0.0, 0.0)
    assert late.increase_torque is False
    assert abs(late.velocity.vx) <= abs(command.vx)
    assert abs(late.velocity.wz) <= abs(command.wz)


def test_braking_coast_inside_window_is_not_slip():
    # 0.8 m/s for one frame, then the command drops to zero and the body
    # coasts 0.17 m in 0.25 s. That coast is inside the braking window.
    checker = monitor()
    driving = checker.update(Velocity(0.8, 0.0), wheel(0.2), body(0.2, vx=0.8))
    coast = checker.update(Velocity(0.0, 0.0), wheel(0.0), body(0.17, vx=0.68))
    resumed = checker.update(Velocity(0.8, 0.0), wheel(0.2), body(0.2, vx=0.8))
    assert driving.anomalous is False
    assert coast.anomalous is False
    assert coast.velocity == Velocity(0.0, 0.0)
    assert coast.increase_torque is False
    assert resumed.anomalous is False
    assert resumed.velocity == Velocity(0.8, 0.0)


def test_startup_latency_not_slip():
    checker = monitor()
    driving = checker.update(Velocity(0.2, 0.0), wheel(0.0625), still_body())
    braking = checker.update(Velocity(0.0, 0.0), wheel(0.0625, dt=0.5), still_body())
    assert driving.anomalous is False
    assert driving.velocity == Velocity(0.2, 0.0)
    assert driving.increase_torque is False
    assert braking.anomalous is False
    assert braking.velocity == Velocity(0.0, 0.0)
    assert braking.increase_torque is False


def test_lateral_slip_and_unexpected_yaw():
    lateral = monitor().update(
        Velocity(0.2, 0.0),
        wheel(0.0625),
        body(0.0625, 0.0625, vx=0.25),
    )
    yaw = monitor().update(
        Velocity(0.2, 0.0),
        wheel(0.0625),
        body(0.0625, yaw=0.5, vx=0.25),
    )
    assert lateral.anomalous is True
    assert lateral.velocity == Velocity(0.0, 0.0)
    assert lateral.increase_torque is False
    assert yaw.anomalous is True
    assert yaw.velocity == Velocity(0.0, 0.0)
    assert yaw.increase_torque is False


def test_anomaly_latches_until_explicit_reset_after_stop():
    checker = monitor()
    command = Velocity(0.2, 0.0)
    for _ in range(3):
        checker.update(command, wheel(0.0625), still_body())
    held = checker.update(command, wheel(0.0), still_body())
    assert held.anomalous is True
    assert held.velocity == Velocity(0.0, 0.0)
    assert held.increase_torque is False
    assert checker.update(Velocity(0.0, 0.0), wheel(0.0), still_body()).anomalous
    checker.reset()
    cleared = checker.update(Velocity(0.0, 0.0), wheel(0.0), still_body())
    assert cleared.anomalous is False
    assert cleared.velocity == Velocity(0.0, 0.0)
    resumed = checker.update(command, wheel(0.0625), still_body())
    assert resumed.anomalous is False
    assert resumed.velocity == command


def test_slip_reset_clears_latch():
    checker = monitor()
    command = Velocity(0.2, 0.0)
    for _ in range(3):
        checker.update(command, wheel(0.0625), still_body())
    checker.reset()
    released = checker.update(command, wheel(0.0), still_body())
    assert released.anomalous is False
    assert released.velocity == command
    assert released.increase_torque is False


def test_opposite_direction_stops_inside_startup_window():
    decision = monitor().update(Velocity(0.2, 0.0), wheel(0.0625), body(-0.0625, vx=-0.25))
    assert decision.anomalous is True
    assert decision.velocity == Velocity(0.0, 0.0)
    assert decision.increase_torque is False


def test_short_body_travel_stops_after_startup_window():
    checker = monitor()
    command = Velocity(0.2, 0.0)
    moved = body(0.03, vx=0.04)
    early = checker.update(command, wheel(1.0), moved)
    late = checker.update(command, wheel(1.0, dt=0.5), body(0.03, dt=0.5, vx=0.06))
    assert early.anomalous is False
    assert late.anomalous is True
    assert late.velocity == Velocity(0.0, 0.0)
    assert late.increase_torque is False


def test_uncommanded_body_motion_stops():
    decision = monitor().update(Velocity(0.0, 0.0), wheel(0.0), body(0.1, vx=0.4))
    assert decision.anomalous is True
    assert decision.velocity == Velocity(0.0, 0.0)
    assert decision.increase_torque is False


def test_rotation_startup_lag_is_not_slip():
    lag = monitor().update(Velocity(0.0, 0.8), wheel(0.0), still_body())
    opposite = monitor().update(Velocity(0.0, 0.8), wheel(0.0), body(yaw=-0.2, wz=-0.8))
    assert lag.anomalous is False
    assert lag.velocity == Velocity(0.0, 0.8)
    assert lag.increase_torque is False
    assert opposite.anomalous is True
    assert opposite.velocity == Velocity(0.0, 0.0)


def test_body_and_wheel_intervals_share_one_yaw_budget():
    # 0.8 rad/s over a 0.1 s body sample is 0.08 rad. The wheel window is 0.5 s.
    # Scaling both onto one interval leaves zero yaw error. The startup window
    # is already consumed, so a raw comparison would stop.
    checker = monitor()
    checker.update(Velocity(0.0, 0.8), wheel(0.0), body(yaw=0.2, wz=0.8))
    aligned = checker.update(
        Velocity(0.0, 0.8),
        wheel(0.0, dt=0.5),
        body(yaw=0.08, dt=0.1, wz=0.8),
    )
    assert aligned.anomalous is False
    assert aligned.velocity == Velocity(0.0, 0.8)


def test_curved_differential_drive_is_not_lateral_slip():
    # A one-second 0.5 m/s, 1 rad/s arc has y≈0.23 m despite zero side slip.
    dx, dy, dyaw = 0.5 * math.sin(1.0), 0.5 * (1 - math.cos(1.0)), 1.0
    command = Velocity(0.5, 1.0)
    actual = body(dx, dy, dyaw, dt=1.0, vx=0.5, wz=1.0)
    wheel_arc = WheelOdom(Pose2D(dx, dy, dyaw), 10.0, 1.0)
    assert not monitor().update(command, wheel_arc, actual).anomalous
    slipped = body(dx, dy + 0.1, dyaw, dt=1.0, vx=0.5, wz=1.0)
    assert monitor().update(command, wheel_arc, slipped).anomalous


def test_changing_curvature_is_not_lateral_slip_but_real_drift_is():
    from senior_control.safety.motion_runtime import TimedCommandBuffer

    commands = TimedCommandBuffer(max_gap=1.01, max_samples=10)
    assert commands.add(1.0, Velocity(1.0, 0.1))
    assert commands.add(2.0, Velocity(0.1, 1.0))
    integrated = commands.integrated(1.0, 3.0)
    assert integrated is not None and abs(integrated.lateral_velocity) > 0.03125
    delta = integrated.pose_delta
    wheel_path = WheelOdom(delta, 3.0, 2.0)
    actual = body(delta.x, delta.y, delta.yaw, dt=2.0)
    assert not monitor().update(integrated.velocity, wheel_path, actual).anomalous
    drift = body(delta.x, delta.y + 0.1, delta.yaw, dt=2.0)
    assert monitor().update(integrated.velocity, wheel_path, drift).anomalous


def test_sustained_turning_wheels_with_stationary_body_trips_after_startup():
    checker = monitor()
    command = Velocity(0.0, 0.4)
    for _ in range(5):
        assert not checker.update(
            command, WheelOdom(Pose2D(0, 0, 0.04), 10, 0.1), body(dt=0.1)
        ).anomalous
    decisions = [
        checker.update(command, WheelOdom(Pose2D(0, 0, 0.04), 10, 0.1), body(dt=0.1))
        for _ in range(5)
    ]
    assert any(decision.anomalous for decision in decisions)


def _low_speed_monitor():
    return SlipMonitor(
        startup_window=0.6,
        wheel_motion_limit=0.004,
        lateral_limit=0.04,
        yaw_limit=0.12,
        stall_distance=0.04,
    )


def test_sustained_low_speed_wheel_stall_is_detected():
    checker = _low_speed_monitor()
    decisions = [
        checker.update(Velocity(0.02, 0), wheel(0.002, 0.1), body(dt=0.1)) for _ in range(100)
    ]
    assert any(item.anomalous for item in decisions)


def test_protection_zero_pulses_do_not_reset_low_speed_stall_budget():
    checker = _low_speed_monitor()
    decisions = [
        checker.update(
            Velocity(0.02 if i % 2 == 0 else 0.0, 0),
            wheel(0.002 if i % 2 == 0 else 0.0, 0.1),
            body(dt=0.1),
        )
        for i in range(100)
    ]
    assert any(item.anomalous for item in decisions)


def test_long_safety_zero_intervals_do_not_reopen_startup_budget():
    checker = _low_speed_monitor()
    decisions = []
    for _ in range(30):
        decisions.append(checker.update(Velocity(0.02, 0), wheel(0.002, 0.1), body(dt=0.1)))
        for _ in range(8):
            decisions.append(checker.update(Velocity(0, 0), wheel(0, 0.1), body(dt=0.1)))
    assert any(item.anomalous for item in decisions)


def test_normal_low_speed_and_quantized_body_motion_are_not_stall():
    low = _low_speed_monitor()
    for _ in range(100):
        assert not low.update(Velocity(0.02, 0), wheel(0.002, 0.1), body(0.002, dt=0.1)).anomalous
    quantized = _low_speed_monitor()
    for i in range(100):
        body_step = 0.02 if i % 4 == 3 else 0.0
        assert not quantized.update(
            Velocity(0.05, 0), wheel(0.005, 0.1), body(body_step, dt=0.1)
        ).anomalous


def test_applied_motion_without_wheel_or_body_progress_eventually_stops():
    checker = _low_speed_monitor()
    decisions = [
        checker.update(Velocity(0.02, 0), wheel(0.0, 0.1), body(dt=0.1)) for _ in range(100)
    ]
    assert any(item.anomalous for item in decisions)


def test_wheels_turn_while_applied_zero_and_body_stationary_are_detected():
    checker = _low_speed_monitor()
    decisions = [
        checker.update(Velocity(0, 0), wheel(0.002, 0.1), body(dt=0.1)) for _ in range(100)
    ]
    assert any(item.anomalous for item in decisions)


def test_small_zero_mean_wheel_noise_does_not_accumulate_as_one_way_rotation():
    checker = _low_speed_monitor()
    for i in range(100):
        sample = 0.002 if i % 2 == 0 else -0.002
        assert not checker.update(Velocity(0, 0), wheel(sample, 0.1), body(dt=0.1)).anomalous


def test_wheel_noise_then_sustained_opposite_spin_is_detected():
    checker = _low_speed_monitor()
    checker.update(Velocity(0, 0), wheel(0.002, 0.1), body(dt=0.1))
    decisions = [
        checker.update(Velocity(0, 0), wheel(-0.002, 0.1), body(dt=0.1)) for _ in range(100)
    ]
    assert any(item.anomalous for item in decisions)


def test_persistent_subthreshold_lateral_and_yaw_drift_are_detected():
    lateral = _low_speed_monitor()
    sideways = [
        lateral.update(Velocity(0.02, 0), wheel(0.002, 0.1), body(0.002, 0.003, dt=0.1))
        for _ in range(100)
    ]
    assert any(item.anomalous for item in sideways)
    yaw = _low_speed_monitor()
    turning = [
        yaw.update(Velocity(0.02, 0), wheel(0.002, 0.1), body(0.002, 0, 0.005, dt=0.1))
        for _ in range(100)
    ]
    assert any(item.anomalous for item in turning)


def test_sustained_wheel_rotation_while_applied_zero_and_body_stationary_stops():
    checker = _low_speed_monitor()
    results = [
        checker.update(Velocity(0, 0), WheelOdom(Pose2D(0, 0, 0.003), 10.0, 0.1), body(dt=0.1))
        for _ in range(100)
    ]
    assert any(result.anomalous for result in results)


def test_sustained_aligned_low_yaw_is_not_an_anomaly():
    checker = _low_speed_monitor()
    for _ in range(100):
        result = checker.update(
            Velocity(0, 0.03),
            WheelOdom(Pose2D(0, 0, 0.003), 10.0, 0.1),
            body(yaw=0.003, dt=0.1),
        )
        assert not result.anomalous


def test_applied_turn_pulses_without_wheel_or_body_progress_eventually_stop():
    checker = _low_speed_monitor()
    decisions = [
        checker.update(Velocity(0, 0.02 if i % 2 == 0 else 0.0), wheel(0, 0.1), body(dt=0.1))
        for i in range(200)
    ]
    assert any(result.anomalous for result in decisions)


def test_gate_forced_zero_cannot_clear_anomaly_without_a_stop_acknowledgement():
    checker = SlipMonitor(
        startup_window=0,
        wheel_motion_limit=0.01,
        lateral_limit=0.03,
        yaw_limit=0.1,
    )
    assert checker.update(Velocity(0.2, 0), wheel(0.1), still_body()).anomalous
    assert checker.update(Velocity(0, 0), wheel(0), still_body()).anomalous
    checker.reset()
    assert not checker.update(Velocity(0, 0), wheel(0), still_body()).anomalous
