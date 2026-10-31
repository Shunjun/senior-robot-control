"""Real Fusion callback inputs for a fresh, explicitly stationary startup."""

import time


def first_zero_acknowledgements(requests, outputs, *, intent_steady):
    """Find observed request zero, then observed safety-output zero after intent."""
    request_ack = next(
        (
            received
            for received, message in requests
            if received >= intent_steady and message.linear.x == 0 and message.angular.z == 0
        ),
        None,
    )
    if request_ack is None:
        return None
    output_ack = next(
        (
            received
            for received, message in outputs
            if received >= request_ack and message.linear.x == 0 and message.angular.z == 0
        ),
        None,
    )
    return None if output_ack is None else (request_ack, output_ack)


def prime_stationary_progress(
    fusion,
    clock,
    base_ns,
    *,
    expect_qualified=True,
    wheel_step=0.0,
    body_step=0.0,
    imu_rate=0.0,
):
    from geometry_msgs.msg import Twist
    from rclpy.time import Time
    from senior_motion_msgs.msg import ScanMotion

    from senior_control.models import Pose2D
    from senior_control.safety.motion_runtime import ImuSample, TimedPoseSample

    step_ns = 100_000_000
    for index in range(7):
        stamp_ns = base_ns + index * step_ns
        clock["now"] = stamp_ns + (10_000_000 if index < 6 else 0)
        stamp = stamp_ns / 1e9
        assert fusion._wheel.add(
            TimedPoseSample(stamp, Pose2D(index * wheel_step, 0, 0), (0.001,) * 3, "odom")
        )
        assert fusion._imu.add(ImuSample(stamp, imu_rate, 0.001))
        fusion._wheel_received = fusion._imu_received = time.monotonic()
        if index < 6:
            fusion._on_requested(Twist())
            fusion._on_command(Twist())
        if index:
            message = ScanMotion()
            message.header.frame_id = "base_link"
            message.header.stamp = Time(nanoseconds=stamp_ns).to_msg()
            message.interval_start = Time(nanoseconds=stamp_ns - step_ns).to_msg()
            message.source_epoch = "stable-independent-scan"
            message.valid = True
            message.reason = "qualified"
            message.response = 0.8
            message.valid_rays = 720
            message.body_delta.x = float(body_step)
            message.local_pose.x = float(index * body_step)
            message.covariance[0] = message.covariance[4] = message.covariance[8] = 0.001
            fusion._on_scan(message)
    assert fusion._progress_started is expect_qualified
    assert not fusion._progress.unqualified
    return base_ns + 6 * step_ns
