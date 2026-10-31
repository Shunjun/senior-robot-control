"""Command ownership publishes into the safety gate and nowhere else.

The monotonic clock, not ROS time, expires a command while /clock is frozen.
This file is skipped without rclpy; a skip is not a pass on macOS.
"""

import pytest


def _publisher_topics(node: object) -> set[str]:
    publishers = getattr(node, "publishers", None)
    if publishers is None:
        publishers = getattr(node, "_publishers", ())
    return {publisher.topic_name for publisher in publishers}


def _spin_until(node: object, ready) -> None:
    rclpy = pytest.importorskip("rclpy")
    deadline_spins = 20
    for _ in range(deadline_spins):
        if ready():
            return
        rclpy.spin_once(node, timeout_sec=0.05)
    assert ready()


def test_sim_pause_expires_command():
    """Breaks if the command node lets frozen ROS time keep the last Twist alive."""
    rclpy = pytest.importorskip("rclpy")
    from geometry_msgs.msg import Twist
    from rclpy.parameter import Parameter
    from senior_robot.command_node import CommandNode

    from senior_control.protocol import (
        EmptyPayload,
        SimpleEnvelope,
        VelocityEnvelope,
        VelocityPayload,
    )
    from senior_control.safety.arbiter import Arbiter

    class FrozenClock:
        def __init__(self) -> None:
            self.now = 1000.0

        def monotonic(self) -> float:
            return self.now

    def envelope(
        kind: str, seq: int, vx: float = 0.0, wz: float = 0.0
    ) -> SimpleEnvelope | VelocityEnvelope:
        common = {
            "v": 1,
            "robot_id": "robot-1",
            "session_id": "session-1",
            "connection_epoch": 1,
            "request_id": f"req-{seq}",
            "seq": seq,
            "expires_at": 42.0,
        }
        if kind == "drive.velocity":
            return VelocityEnvelope(
                type="drive.velocity",
                payload=VelocityPayload(vx=vx, wz=wz),
                **common,  # type: ignore[arg-type]
            )
        return SimpleEnvelope(type=kind, payload=EmptyPayload(), **common)  # type: ignore[arg-type]

    rclpy.init()
    clock = FrozenClock()
    arbiter = Arbiter(
        "robot-1",
        heartbeat_seconds=0.25,
        disconnect_seconds=5.0,
        command_ttl_seconds=0.5,
    )
    node = CommandNode(
        arbiter=arbiter,
        clock=clock,
        parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
    )
    listener = rclpy.create_node("velocity_chain_listener")
    heard: list[Twist] = []
    cmd_vel: list[Twist] = []
    listener.create_subscription(Twist, "/safety/command_in", heard.append, 10)
    listener.create_subscription(Twist, "/cmd_vel", cmd_vel.append, 10)
    try:
        node.submit(envelope("control.acquire", 1))
        node.submit(envelope("drive.velocity", 2, vx=0.25, wz=-0.1))
        _spin_until(listener, lambda: bool(heard) and heard[-1].linear.x == 0.25)
        assert heard[-1].angular.z == -0.1
        frozen = node.get_clock().now().nanoseconds
        clock.now = 1000.5 + 1e-9
        node.publish_command()
        _spin_until(
            listener,
            lambda: heard[-1].linear.x == 0.0 and heard[-1].angular.z == 0.0,
        )
        assert node.get_clock().now().nanoseconds == frozen
        assert cmd_vel == []
        clock.now = 1001.0
        node.submit(envelope("drive.velocity", 3, vx=0.4, wz=0.1))
        _spin_until(listener, lambda: heard[-1].linear.x == 0.4 and heard[-1].angular.z == 0.1)
        node.disconnect(1)
        _spin_until(listener, lambda: heard[-1].linear.x == 0.0 and heard[-1].angular.z == 0.0)
        topics = _publisher_topics(node)
        assert "/safety/command_in" in topics
        assert "/cmd_vel" not in topics
    finally:
        listener.destroy_node()
        node.destroy_node()
        rclpy.shutdown()


def test_safety_gate_alone_publishes_cmd_vel():
    """Breaks if command_node takes /cmd_vel or the safety gate stops publishing it."""
    rclpy = pytest.importorskip("rclpy")
    from senior_robot.command_node import CommandNode
    from senior_robot.safety_node import SafetyGate

    rclpy.init()
    safety = SafetyGate()
    command = CommandNode()
    try:
        assert safety._profile().footprint == []
        assert "/cmd_vel" in _publisher_topics(safety)
        command_topics = _publisher_topics(command)
        assert "/safety/command_in" in command_topics
        assert "/cmd_vel" not in command_topics
    finally:
        command.destroy_node()
        safety.destroy_node()
        rclpy.shutdown()


def test_safety_scan_uses_timestamped_tf_and_fails_closed_without_measurement():
    rclpy = pytest.importorskip("rclpy")
    from geometry_msgs.msg import TransformStamped, TwistStamped
    from rclpy.parameter import Parameter
    from senior_robot.safety_node import SafetyGate
    from sensor_msgs.msg import LaserScan

    rclpy.init()
    gate = SafetyGate(
        parameter_overrides=[
            Parameter("observation_max_age", value=0.5),
            Parameter("measurement_max_age", value=0.5),
            Parameter("transform_static", value=True),
            Parameter("self_mask_calibrated", value=True),
        ]
    )
    try:
        tf = TransformStamped()
        tf.header.frame_id = "base_link"
        tf.child_frame_id = "laser_link"
        tf.transform.translation.x = 0.25
        tf.transform.rotation.w = 1.0
        gate._tf_buffer.set_transform_static(tf, "test")
        scan = LaserScan()
        scan.header.frame_id = "laser_link"
        scan.header.stamp = gate.get_clock().now().to_msg()
        scan.angle_increment = 0.1
        scan.range_min, scan.range_max = 0.1, 10.0
        scan.ranges = [1.0]
        gate._on_scan(scan)
        assert gate._filtered is not None
        assert gate._filtered.rays[0].origin == (0.25, 0.0)
        assert gate._filtered.rays[0].endpoint == (1.25, 0.0)
        now = gate.get_clock().now().nanoseconds / 1e9
        import time

        assert not gate._freshness.usable(now, time.monotonic())
        measured = TwistStamped()
        measured.header.frame_id = "base_link"
        measured.header.stamp = gate.get_clock().now().to_msg()
        gate._on_measured(measured)
        now = gate.get_clock().now().nanoseconds / 1e9
        assert gate._freshness.usable(now, time.monotonic())
        assert not gate._freshness.usable(now, time.monotonic() + 1.0)
    finally:
        gate.destroy_node()
        rclpy.shutdown()


def test_safety_steady_timer_publishes_zero_while_ros_time_is_frozen():
    rclpy = pytest.importorskip("rclpy")
    from geometry_msgs.msg import Twist
    from rclpy.parameter import Parameter
    from senior_robot.safety_node import SafetyGate

    rclpy.init()
    gate = SafetyGate(parameter_overrides=[Parameter("use_sim_time", value=True)])
    listener = rclpy.create_node("safety_timer_listener")
    heard = []
    listener.create_subscription(Twist, "/cmd_vel", heard.append, 10)
    try:
        frozen = gate.get_clock().now().nanoseconds
        for _ in range(40):
            rclpy.spin_once(gate, timeout_sec=0.05)
            rclpy.spin_once(listener, timeout_sec=0.05)
            if heard:
                break
        assert heard and heard[-1].linear.x == 0.0 and heard[-1].angular.z == 0.0
        assert gate.get_clock().now().nanoseconds == frozen
    finally:
        listener.destroy_node()
        gate.destroy_node()
        rclpy.shutdown()


def test_safety_unconfigured_arrays_fail_closed_without_callback_exception():
    rclpy = pytest.importorskip("rclpy")
    from senior_robot.safety_node import SafetyGate

    rclpy.init()
    gate = SafetyGate()
    try:
        profile = gate._profile()
        assert not profile.footprint
        assert not profile.self_mask
        gate._on_timer()
    finally:
        gate.destroy_node()
        rclpy.shutdown()


def test_unstamped_autonomous_twist_has_no_motion_input():
    rclpy = pytest.importorskip("rclpy")
    from senior_robot.command_node import CommandNode

    rclpy.init()
    node = CommandNode()
    try:
        subscriptions = getattr(node, "subscriptions", None)
        if subscriptions is None or callable(subscriptions):
            subscriptions = node._subscriptions
        topics = {subscription.topic_name for subscription in subscriptions}
        assert "/control/navigation_cmd" not in topics
        assert "/control/recovery_cmd" not in topics
    finally:
        node.destroy_node()
        rclpy.shutdown()
