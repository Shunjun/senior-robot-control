"""Actual ROS watchdog zeros cannot be an explicit motion-fault reset."""

import time

import pytest


def test_idle_watchdog_and_applied_zero_do_not_clear_real_motion_latch():
    rclpy = pytest.importorskip("rclpy")
    from geometry_msgs.msg import Twist
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from senior_robot.command_node import CommandNode
    from senior_robot.motion_fusion import MotionFusion

    from senior_control.models import BodyTwist, Pose2D, Velocity
    from senior_control.safety.motion_estimator import MotionEstimate, WheelOdom

    values = dict(
        sensor_max_age=0.5,
        alignment_max_gap=0.2,
        pending_wait=0.1,
        max_variance=0.1,
        min_quality=0.3,
        max_yaw_error=0.1,
        startup_window=0.6,
        wheel_motion_limit=0.004,
        stall_distance=0.04,
        lateral_limit=0.04,
        yaw_limit=0.12,
    )
    rclpy.init()
    fusion = MotionFusion(parameter_overrides=[Parameter(k, value=v) for k, v in values.items()])
    command = CommandNode()
    guard = rclpy.create_node("test_owned_guard_zero")
    pub = guard.create_publisher(Twist, "/cmd_vel", 10)
    executor = SingleThreadedExecutor()
    for node in (fusion, command, guard):
        executor.add_node(node)
    observed = []
    guard.create_subscription(
        Twist, "/safety/command_in", lambda m: observed.append((m.linear.x, m.angular.z)), 10
    )
    try:
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
        estimate = MotionEstimate(
            Pose2D(0, 0, 0),
            Velocity(0, 0),
            (0.001,) * 3,
            0.8,
            dt=1.0,
            source="scan_match",
            stamp=1.0,
            degenerate=False,
            aligned=True,
            max_covariance=0.1,
            body_twist=BodyTwist(0, 0, 0),
        )
        anomaly = fusion._slip.update(
            Velocity(0.05, 0), WheelOdom(Pose2D(0.1, 0, 0), 1.0, 1.0), estimate
        )
        assert anomaly.anomalous and fusion._slip._latched
        fusion._stop_acknowledgement.trip()
        deadline = time.monotonic() + 0.4
        while time.monotonic() < deadline:
            command.publish_command()  # No active owner: real arbiter/watchdog zero.
            pub.publish(Twist())  # Safety output is zero, no reset request.
            executor.spin_once(timeout_sec=0.01)
        assert observed and all(x == (0.0, 0.0) for x in observed)
        assert fusion._slip._latched, (
            "bare watchdog/protective zeros silently cleared a real slip latch"
        )
    finally:
        executor.shutdown()
        for node in (fusion, command, guard):
            node.destroy_node()
        rclpy.shutdown()
