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


def test_anomaly_latches_until_command_is_zero():
    checker = monitor()
    command = Velocity(0.2, 0.0)
    for _ in range(3):
        checker.update(command, wheel(0.0625), still_body())
    held = checker.update(command, wheel(0.0), still_body())
    assert held.anomalous is True
    assert held.velocity == Velocity(0.0, 0.0)
    assert held.increase_torque is False
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
