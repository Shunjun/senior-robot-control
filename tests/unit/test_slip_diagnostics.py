"""The first latched decision must retain bounded, replayable evidence."""

import pytest

from senior_control.models import Pose2D, Velocity
from senior_control.safety.motion_estimator import MotionEstimate, WheelOdom
from senior_control.safety.slip import SlipMonitor


def _monitor() -> SlipMonitor:
    return SlipMonitor(
        startup_window=0.2,
        wheel_motion_limit=0.004,
        lateral_limit=0.04,
        yaw_limit=0.12,
        stall_distance=0.04,
    )


def _body(delta: Pose2D, dt: float = 0.1) -> MotionEstimate:
    return MotionEstimate(delta, Velocity(0.0, 0.0), (0.001,) * 3, 1.0, dt)


def test_first_latch_records_applied_motion_and_stall_budget_without_changing_decision():
    monitor = _monitor()
    command = Velocity(0.2, 0.0)
    for index in range(3):
        wheel = WheelOdom(Pose2D(0.02, 0.0, 0.0), 10.1 + index * 0.1, 0.1)
        decision = monitor.update(command, wheel, _body(Pose2D(0.0, 0.0, 0.0)))
        assert decision.anomalous is (index == 2)

    latch = monitor.first_latch
    assert latch is not None
    assert set(latch.causes) == {"wheel_stall", "command_stall"}
    assert latch.command == command
    assert latch.wheel == wheel
    assert latch.actual.pose_delta == Pose2D(0.0, 0.0, 0.0)
    assert latch.wheel_twist == Pose2D(0.02, 0.0, 0.0)
    assert latch.body_twist == Pose2D(0.0, 0.0, 0.0)
    state = dict(latch.state)
    assert state["linear_elapsed"] == pytest.approx(0.3)
    assert state["linear_wheel"] == pytest.approx(0.06)
    assert state["linear_body"] == pytest.approx(0.0)
    assert state["command_progress"] == pytest.approx(0.06)
    assert state["command_body"] == pytest.approx(0.0)
    assert state["stall_distance"] == pytest.approx(0.04)
    assert tuple(monitor.recent_intervals)[-1] == latch

    # A safety-forced zero cannot overwrite the evidence explaining the latch.
    assert monitor.update(Velocity(0.0, 0.0), wheel, _body(Pose2D(0.0, 0.0, 0.0))).anomalous
    assert monitor.first_latch == latch
    monitor.reset()
    assert monitor.first_latch is None
    assert not monitor.recent_intervals


def test_interval_history_is_bounded_and_reports_independent_lateral_cause():
    monitor = _monitor()
    for index in range(150):
        wheel = WheelOdom(Pose2D(0.001, 0.0, 0.0), 1.0 + index * 0.1, 0.1)
        assert not monitor.update(Velocity(0.01, 0.0), wheel, _body(wheel.pose_delta)).anomalous
    assert len(monitor.recent_intervals) == 128
    assert monitor.first_latch is None

    wheel = WheelOdom(Pose2D(0.0, 0.0, 0.0), 20.0, 0.1)
    assert monitor.update(Velocity(0.01, 0.0), wheel, _body(Pose2D(0.0, 0.05, 0.0))).anomalous
    assert monitor.first_latch is not None
    assert "lateral" in monitor.first_latch.causes
    assert len(monitor.recent_intervals) == 128
