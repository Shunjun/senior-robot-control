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


def still_body() -> MotionEstimate:
    return MotionEstimate(Pose2D(0.0, 0.0, 0.0), Velocity(0.0, 0.0), (0.25, 0.25, 0.25), 1.0)


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
        MotionEstimate(Pose2D(0.0625, 0.0625, 0.0), Velocity(0.25, 0.0), (0.25, 0.25, 0.25), 1.0),
    )
    yaw = monitor().update(
        Velocity(0.2, 0.0),
        wheel(0.0625),
        MotionEstimate(Pose2D(0.0625, 0.0, 0.5), Velocity(0.25, 0.0), (0.25, 0.25, 0.25), 1.0),
    )
    assert lateral.anomalous is True
    assert lateral.velocity == Velocity(0.0, 0.0)
    assert lateral.increase_torque is False
    assert yaw.anomalous is True
    assert yaw.velocity == Velocity(0.0, 0.0)
    assert yaw.increase_torque is False
