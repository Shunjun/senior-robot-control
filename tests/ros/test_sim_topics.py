"""Linux integration: launch simulator separately before running this file."""

import copy
import math
import struct
import subprocess
import time

import pytest

rclpy = pytest.importorskip("rclpy", reason="requires sourced ROS 2 Humble and running simulator")
from geometry_msgs.msg import Twist  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from rosgraph_msgs.msg import Clock  # noqa: E402
from sensor_msgs.msg import CameraInfo, Image, Imu, LaserScan  # noqa: E402
from tf2_msgs.msg import TFMessage  # noqa: E402


@pytest.fixture
def node():
    rclpy.init()
    instance = rclpy.create_node("senior_sim_contract_test")
    yield instance
    instance.destroy_node()
    rclpy.shutdown()


def wait_for(node, condition, seconds=30):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline and not condition():
        rclpy.spin_once(node, timeout_sec=0.1)
    assert condition(), "timed out waiting for ROS data"


def test_sim_sensor_contract(node):
    expected = {
        "/scan": (LaserScan, "laser_link"),
        "/camera/color/image_raw": (Image, "camera_optical_frame"),
        "/camera/depth/image_raw": (Image, "camera_optical_frame"),
        "/camera/color/camera_info": (CameraInfo, "camera_optical_frame"),
        "/camera/depth/camera_info": (CameraInfo, "camera_optical_frame"),
        "/imu/data": (Imu, "imu_link"),
        "/wheel/odom": (Odometry, "odom"),
        "/ground_truth/odom": (Odometry, "world"),
    }
    received = {}
    subscriptions = []
    for topic, (kind, _) in expected.items():
        subscriptions.append(
            node.create_subscription(
                kind, topic, lambda msg, t=topic: received.update({t: msg}), qos_profile_sensor_data
            )
        )
    clock = []
    subscriptions.append(
        node.create_subscription(Clock, "/clock", clock.append, qos_profile_sensor_data)
    )
    transforms = []
    subscriptions.append(
        node.create_subscription(
            TFMessage, "/tf", lambda msg: transforms.extend(msg.transforms), 10
        )
    )
    wait_for(node, lambda: len(received) == len(expected) and bool(clock))
    assert clock[-1].clock.sec or clock[-1].clock.nanosec
    for topic, (_, frame) in expected.items():
        msg = received[topic]
        assert msg.header.frame_id == frame
        assert msg.header.stamp.sec or msg.header.stamp.nanosec
    scan = received["/scan"]
    assert any(math.isfinite(r) and 0.1 < r < 4.0 for r in scan.ranges)
    depth = received["/camera/depth/image_raw"]
    assert depth.encoding == "32FC1" and depth.width and depth.height
    endian = ">" if depth.is_bigendian else "<"
    center_offset = (depth.height // 2) * depth.step + (depth.width // 2) * 4
    center_depth = struct.unpack_from(endian + "f", depth.data, center_offset)[0]
    assert math.isfinite(center_depth) and 0.1 < center_depth < 4.0
    assert received["/ground_truth/odom"].child_frame_id == "base_link"
    assert not any(
        t.header.frame_id == "world" and t.child_frame_id == "base_link" for t in transforms
    )
    assert subscriptions


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec / 1e9


def spin_for(node, duration):
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)


def set_sim_pose(pose):
    position, rotation = pose.position, pose.orientation
    request = (
        f"name: 'senior' position: {{x: {position.x}, y: {position.y}, z: {position.z}}} "
        f"orientation: {{x: {rotation.x}, y: {rotation.y}, z: {rotation.z}, w: {rotation.w}}}"
    )
    result = subprocess.run(
        [
            "ign",
            "service",
            "-s",
            "/world/baseline/set_pose",
            "--reqtype",
            "ignition.msgs.Pose",
            "--reptype",
            "ignition.msgs.Boolean",
            "--timeout",
            "5000",
            "--req",
            request,
        ],
        capture_output=True,
        text=True,
        timeout=8,
        check=True,
    )
    assert "data: true" in result.stdout.lower(), result.stdout + result.stderr


def test_truth_independent_and_tf_stream(node):
    truth, wheel, transforms, scans = [], [], [], []
    subscriptions = [
        node.create_subscription(Odometry, "/ground_truth/odom", truth.append, 10),
        node.create_subscription(Odometry, "/wheel/odom", wheel.append, 10),
        node.create_subscription(
            TFMessage, "/tf", lambda msg: transforms.extend(msg.transforms), 10
        ),
        node.create_subscription(LaserScan, "/scan", scans.append, qos_profile_sensor_data),
    ]
    wait_for(node, lambda: bool(truth) and bool(wheel) and bool(scans))
    assert any(p.node_name == "drive" for p in node.get_publishers_info_by_topic("/wheel/odom"))
    spin_for(node, 2.0)
    before_scan_count = len(scans)
    before_tf_count = len(transforms)
    original = copy.deepcopy(truth[-1].pose.pose)
    original_wheel = copy.deepcopy(wheel[-1].pose.pose)
    shifted = copy.deepcopy(original)
    shifted.position.y += 1.0
    try:
        set_sim_pose(shifted)
        wait_for(node, lambda: abs(truth[-1].pose.pose.position.y - shifted.position.y) < 0.2)
        spin_for(node, 2.0)
        assert abs(truth[-1].pose.pose.position.y - original.position.y) > 0.8
        assert abs(wheel[-1].pose.pose.position.y - original_wheel.position.y) < 0.2
        assert len(scans) >= before_scan_count + 3
        assert len(transforms) >= before_tf_count + 3
        assert len({stamp_seconds(msg.header.stamp) for msg in scans}) >= 3
        normal = [
            t for t in transforms if t.header.frame_id == "odom" and t.child_frame_id == "base_link"
        ]
        assert len(normal) >= 3 and len({stamp_seconds(t.header.stamp) for t in normal}) >= 3
        assert not any(
            t.header.frame_id == "world" or t.child_frame_id == "world" for t in transforms
        )
    finally:
        set_sim_pose(original)
    assert subscriptions


def test_watchdog_stops(node):
    observations, clocks = [], []
    subscriptions = [
        node.create_subscription(Odometry, "/ground_truth/odom", observations.append, 10),
        node.create_subscription(Clock, "/clock", clocks.append, qos_profile_sensor_data),
    ]
    publisher = node.create_publisher(Twist, "/cmd_vel", 10)
    wait_for(node, lambda: bool(clocks) and publisher.get_subscription_count() > 0)
    assert any(s.node_name == "drive" for s in node.get_subscriptions_info_by_topic("/cmd_vel"))
    command = Twist()
    command.linear.x = 0.2
    deadline = time.monotonic() + 15
    moving = False
    while time.monotonic() < deadline:
        publisher.publish(command)
        rclpy.spin_once(node, timeout_sec=0.05)
        if observations and clocks:
            recent = stamp_seconds(clocks[-1].clock) - stamp_seconds(observations[-1].header.stamp)
            if 0 <= recent < 0.2 and observations[-1].twist.twist.linear.x > 0.05:
                moving = True
                break
    assert moving, "no recent measured movement while commands were published"
    # Final command and latest clock establish a lower bound for post-command samples.
    publisher.publish(command)
    last_command_time = stamp_seconds(clocks[-1].clock)
    wait_for(
        node,
        lambda: (
            observations and stamp_seconds(observations[-1].header.stamp) >= last_command_time + 2.0
        ),
        seconds=30,
    )
    samples = [
        m
        for m in observations
        if last_command_time + 1.2 <= stamp_seconds(m.header.stamp) <= last_command_time + 2.0
    ]
    assert len(samples) >= 5
    assert stamp_seconds(samples[-1].header.stamp) - stamp_seconds(samples[0].header.stamp) >= 0.5
    assert all(
        abs(m.twist.twist.linear.x) < 0.02
        and abs(m.twist.twist.linear.y) < 0.02
        and abs(m.twist.twist.angular.z) < 0.05
        for m in samples
    )
    assert subscriptions
