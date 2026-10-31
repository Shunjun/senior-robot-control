"""Source intervals must remain complete before slip budgets are updated."""

import math
from dataclasses import replace

import pytest

from senior_control.models import Pose2D, Velocity
from senior_control.safety.motion_estimator import MotionEstimator, WheelOdom
from senior_control.safety.motion_runtime import (
    ImuSample,
    ProgressInterval,
    ProgressQueue,
    ProgressRequalification,
    ScanInterval,
    TimedCommandBuffer,
    TimedImuBuffer,
    TimedPoseBuffer,
    TimedPoseSample,
    align_interval,
    progress_interval_fresh,
    qualify_interval,
    stationary_interval,
)
from senior_control.safety.slip import SlipMonitor


def _interval(start, end, *, epoch="scan-a", received=100.0):
    scan = ScanInterval(start / 1e9, end / 1e9, Pose2D(0.0, 0.0, 0.0), (0.001,) * 3, 0.8)
    return ProgressInterval(epoch, start, end, scan, received, end)


def test_contiguous_received_intervals_are_retained_in_source_order():
    queue = ProgressQueue(max_pending=3)
    first, second = _interval(100, 200), _interval(200, 300)
    assert queue.offer(first)
    assert queue.offer(second)
    assert queue.diagnostic_state() == {
        "source_epoch": "scan-a",
        "last_end_ns": 300,
        "pending_count": 2,
        "unqualified": False,
        "head": first,
    }
    assert queue.peek() == first
    assert queue.consume() == first
    assert queue.consume() == second
    assert queue.peek() is None and not queue.unqualified


def test_sensor_buffer_diagnostics_preserve_original_boundaries():
    wheel = TimedPoseBuffer(max_gap=0.3, max_samples=4, max_variance=0.1)
    imu = TimedImuBuffer(max_gap=0.3, max_samples=4, max_variance=0.1)
    command = TimedCommandBuffer(max_gap=0.3, max_samples=4)
    assert wheel.diagnostic_bounds() == (None, None, 0)
    assert imu.diagnostic_bounds() == (None, None, 0)
    assert command.diagnostic_bounds() == (None, None, 0)
    for stamp in (1.0, 1.1):
        assert wheel.add(TimedPoseSample(stamp, Pose2D(0, 0, 0), (0.001,) * 3, "odom"))
        assert imu.add(ImuSample(stamp, 0.0, 0.001))
        assert command.add(stamp, Velocity(0, 0))
    assert wheel.diagnostic_bounds() == imu.diagnostic_bounds() == (1.0, 1.1, 2)
    assert command.diagnostic_bounds() == (1.0, 1.1, 2)
    assert wheel.at(1.0) is not None and command.integrated(1.0, 1.1) is not None


def test_missing_interval_or_epoch_change_cannot_be_silently_skipped():
    for discontinuity in (_interval(201, 300), _interval(200, 300, epoch="scan-b")):
        queue = ProgressQueue(max_pending=3)
        assert queue.offer(_interval(100, 200))
        assert not queue.offer(discontinuity)
        assert queue.unqualified and queue.peek() is None
        assert not queue.offer(_interval(300, 400))
        queue.reset_after_requalification()
        assert not queue.unqualified and queue.offer(_interval(300, 400))


def test_overflow_and_explicit_invalidation_fail_closed():
    queue = ProgressQueue(max_pending=1)
    assert queue.offer(_interval(100, 200))
    assert not queue.offer(_interval(200, 300))
    assert queue.unqualified and queue.peek() is None
    queue.reset_after_requalification()
    assert queue.offer(_interval(300, 400))
    queue.invalidate()
    assert queue.unqualified and queue.peek() is None


def test_older_queue_item_keeps_original_steady_deadline_when_ros_time_freezes():
    old = _interval(19_900_000_000, 20_000_000_000, received=100.0)
    newer = _interval(20_000_000_000, 20_100_000_000, received=100.2)
    options = {"now_ros_ns": 20_100_000_000, "max_age": 0.8, "pending_wait": 0.25}
    assert progress_interval_fresh(old, now_steady=100.24, **options)
    assert not progress_interval_fresh(old, now_steady=100.26, **options)
    assert progress_interval_fresh(newer, now_steady=100.26, **options)
    assert not progress_interval_fresh(
        newer, now_steady=100.26, now_ros_ns=20_900_000_001, max_age=0.8, pending_wait=0.25
    )


def test_stationary_candidate_uses_full_qualified_alignment_without_mutating_slip():
    wheel = TimedPoseBuffer(max_gap=0.2, max_samples=4, max_variance=0.1)
    imu = TimedImuBuffer(max_gap=0.2, max_samples=4, max_variance=0.1)
    for stamp in (1.0, 1.1):
        assert wheel.add(TimedPoseSample(stamp, Pose2D(0, 0, 0), (0.001,) * 3, "odom"))
        assert imu.add(ImuSample(stamp, 0.0, 0.001))
    estimator = MotionEstimator(
        max_skew=0.001, min_quality=0.4, max_yaw_error=0.1, max_covariance=0.1
    )
    scan = ScanInterval(1.0, 1.1, Pose2D(0, 0, 0), (0.001,) * 3, 0.8)
    aligned = align_interval(scan, wheel, imu, estimator)
    assert aligned is not None
    assert aligned.wheel.pose_delta == Pose2D(0, 0, 0)
    assert aligned.estimate.pose_delta == Pose2D(0, 0, 0)
    assert aligned.inertial.yaw_delta == 0.0 and aligned.inertial.variance > 0.0
    assert (
        align_interval(
            ScanInterval(0.9, 1.1, scan.body_delta, scan.covariance, 0.8), wheel, imu, estimator
        )
        is None
    )

    commands = TimedCommandBuffer(max_gap=0.2, max_samples=4)
    assert commands.add(1.0, Velocity(0.0, 0.0))
    assert commands.add(1.1, Velocity(0.0, 0.0))
    command = commands.integrated(1.0, 1.1)
    assert command is not None and command.all_zero
    assert stationary_interval(aligned, command, max_linear_speed=0.001, max_yaw_rate=0.001)
    assert not stationary_interval(aligned, command, max_linear_speed=-1.0, max_yaw_rate=0.001)
    moving = TimedCommandBuffer(max_gap=0.2, max_samples=4)
    assert moving.add(1.0, Velocity(0.01, 0.0))
    assert moving.add(1.05, Velocity(0.0, 0.0))
    assert moving.add(1.1, Velocity(0.0, 0.0))
    moving_command = moving.integrated(1.0, 1.1)
    assert moving_command is not None and not moving_command.all_zero
    assert not stationary_interval(
        aligned, moving_command, max_linear_speed=0.001, max_yaw_rate=0.001
    )
    moving_wheel = replace(aligned, wheel=WheelOdom(Pose2D(0.01, 0, 0), 1.1, 0.1))
    moving_body = replace(
        aligned, estimate=replace(aligned.estimate, pose_delta=Pose2D(0, 0.01, 0))
    )
    rotating_imu = replace(aligned, inertial=replace(aligned.inertial, yaw_delta=0.01))
    for moving_observation in (moving_wheel, moving_body, rotating_imu):
        assert not stationary_interval(
            moving_observation, command, max_linear_speed=0.001, max_yaw_rate=0.001
        )


def test_requalification_requires_fresh_post_request_zero_and_contiguous_stationary_intervals():
    gate = ProgressRequalification(max_gap=0.3, required_intervals=2)
    zero = Velocity(0.0, 0.0)
    gate.requested(zero, steady=9.9, ros_ns=900)
    gate.applied(zero, steady=9.9, ros_ns=900)
    gate.arm(steady=10.0, ros_ns=1000)
    assert not gate.observe("epoch", 1000, 1100, stationary=True, steady=10.1, ros_ns=1100)
    gate.requested(zero, steady=10.11, ros_ns=1110)
    gate.applied(zero, steady=10.12, ros_ns=1120)
    assert not gate.observe("epoch", 1000, 1200, stationary=True, steady=10.2, ros_ns=1200)
    assert not gate.observe("epoch", 1120, 1200, stationary=True, steady=10.2, ros_ns=1200)
    gate.requested(zero, steady=10.21, ros_ns=1210)
    gate.applied(zero, steady=10.22, ros_ns=1220)
    assert gate.observe("epoch", 1200, 1300, stationary=True, steady=10.3, ros_ns=1300)


def test_requalification_rejects_motion_timeout_and_source_discontinuity():
    gate = ProgressRequalification(max_gap=0.3, required_intervals=2)
    zero, moving = Velocity(0.0, 0.0), Velocity(0.01, 0.0)
    gate.arm(steady=10.0, ros_ns=1000)
    gate.requested(zero, steady=10.01, ros_ns=1010)
    gate.applied(zero, steady=10.02, ros_ns=1020)
    assert not gate.observe("epoch-a", 1020, 1100, stationary=True, steady=10.1, ros_ns=1100)
    gate.applied(moving, steady=10.11, ros_ns=1110)
    gate.applied(zero, steady=10.12, ros_ns=1120)
    assert not gate.observe("epoch-a", 1100, 1200, stationary=True, steady=10.2, ros_ns=1200)
    assert not gate.observe("epoch-a", 1120, 1200, stationary=False, steady=10.2, ros_ns=1200)
    assert not gate.observe("epoch-a", 1120, 1200, stationary=True, steady=10.2, ros_ns=1200)
    assert not gate.observe("epoch-b", 1200, 1300, stationary=True, steady=10.3, ros_ns=1300)
    assert not gate.observe("epoch-b", 1300, 1400, stationary=True, steady=10.6, ros_ns=1400)


@pytest.mark.parametrize("delay_frame", [False, True])
@pytest.mark.parametrize("body_stalled", [False, True])
def test_ordered_progress_uses_every_received_delta(delay_frame, body_stalled):
    queue = ProgressQueue(max_pending=8)
    wheel = TimedPoseBuffer(max_gap=0.2, max_samples=256, max_variance=0.1)
    imu = TimedImuBuffer(max_gap=0.2, max_samples=256, max_variance=0.1)
    commands = TimedCommandBuffer(max_gap=0.2, max_samples=256)
    estimator = MotionEstimator(
        max_skew=0.001, min_quality=0.4, max_yaw_error=0.1, max_covariance=0.1
    )
    monitor = SlipMonitor(
        startup_window=0.6,
        wheel_motion_limit=0.004,
        stall_distance=0.04,
        lateral_limit=0.04,
        yaw_limit=0.12,
    )
    start_ns, step_ns, step_distance = 10_000_000_000, 100_000_000, 0.0021
    previous_local = 0.0
    anomalies = []
    for index in range(101):
        stamp_ns = start_ns + index * step_ns
        stamp = stamp_ns / 1e9
        assert wheel.add(
            TimedPoseSample(stamp, Pose2D(index * step_distance, 0, 0), (0.001,) * 3, "odom")
        )
        assert imu.add(ImuSample(stamp, 0.0, 0.001))
        assert commands.add(stamp, Velocity(0.02, 0.0))
        if index == 0:
            continue
        local = 0.0 if body_stalled else math.floor((index * step_distance + 0.01) / 0.02) * 0.02
        scan = ScanInterval(
            (stamp_ns - step_ns) / 1e9,
            stamp,
            Pose2D(local - previous_local, 0, 0),
            (0.001,) * 3,
            0.8,
        )
        previous_local = local
        assert queue.offer(
            ProgressInterval("scan-a", stamp_ns - step_ns, stamp_ns, scan, stamp, stamp_ns)
        )
        if delay_frame and index == 5:
            continue
        while (item := queue.consume()) is not None:
            command = commands.integrated(item.scan.start, item.scan.end)
            assert command is not None
            result = qualify_interval(item.scan, wheel, imu, command.velocity, estimator, monitor)
            assert result is not None
            anomalies.append(result.anomalous)
    assert any(anomalies) is body_stalled
