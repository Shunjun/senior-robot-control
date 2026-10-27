"""Linux integration: launch simulator separately before running this file."""

import math
import struct
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
    wheel_publishers = node.get_publishers_info_by_topic("/wheel/odom")
    truth_publishers = node.get_publishers_info_by_topic("/ground_truth/odom")
    assert wheel_publishers and truth_publishers
    assert {bytes(p.endpoint_gid) for p in wheel_publishers}.isdisjoint(
        bytes(p.endpoint_gid) for p in truth_publishers
    )
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


def test_watchdog_stops(node):
    velocities = []
    subscription = node.create_subscription(
        Odometry, "/ground_truth/odom", lambda msg: velocities.append(msg.twist.twist.linear.x), 10
    )
    publisher = node.create_publisher(Twist, "/cmd_vel", 10)
    command = Twist()
    command.linear.x = 0.2
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        publisher.publish(command)
        rclpy.spin_once(node, timeout_sec=0.05)
    assert any(abs(v) > 0.05 for v in velocities), "robot never moved"
    velocities.clear()
    wait_for(node, lambda: len(velocities) > 10 and abs(velocities[-1]) < 0.02, seconds=10)
    assert subscription
