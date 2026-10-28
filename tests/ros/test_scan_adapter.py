"""Runtime gate plus a local negative control for the evidence predicate."""

import math
import time
from types import SimpleNamespace

import pytest


def endpoint(raw, i, distance, origin, yaw):
    angle = yaw + raw.angle_min + i * raw.angle_increment
    return (origin[0] + distance * math.cos(angle), origin[1] + distance * math.sin(angle))


def attachment_evidence(raw, filtered, origin, yaw):
    # URDF attachment and configured laser-plane mask: x [-.33,-.23], y [-.24,.24].
    for i, (x, y) in enumerate(zip(raw.ranges, filtered.ranges, strict=False)):
        if not (math.isfinite(x) and raw.range_min <= x <= raw.range_max and math.isnan(y)):
            continue
        px, py = endpoint(raw, i, x, origin, yaw)
        if abs(px + 0.23) <= 0.025 and abs(py) <= 0.235:
            return True
    return False


def test_invalid_finite_values_do_not_prove_attachment():
    raw = SimpleNamespace(ranges=[0.01, 25.0], range_min=0.08, range_max=20.0)
    filtered = SimpleNamespace(ranges=[float("nan"), float("nan")])
    assert not attachment_evidence(raw, filtered, (0.15699, 0.00201), 0.0)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wall_evidence(raw, filtered, origin, laser_yaw, pose):
    body_yaw = yaw_of(pose.orientation)
    for i, (x, y) in enumerate(zip(raw.ranges, filtered.ranges, strict=False)):
        if not (math.isfinite(x) and raw.range_min <= x <= raw.range_max and x == y):
            continue
        bx, by = endpoint(raw, i, x, origin, laser_yaw)
        wx = pose.position.x + bx * math.cos(body_yaw) - by * math.sin(body_yaw)
        wy = pose.position.y + bx * math.sin(body_yaw) + by * math.cos(body_yaw)
        # Front face of the fixed world obstacle: x=1.9, y in [-1.5, 1.5].
        if abs(wx - 1.9) <= 0.04 and abs(wy) < 1.45:
            return True
    return False


def test_simulated_self_and_wall_evidence():
    rclpy = pytest.importorskip("rclpy")
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import LaserScan
    from tf2_ros import Buffer, TransformException, TransformListener

    rclpy.init()
    node = rclpy.create_node("scan_filter_runtime_test")
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    raw, filtered, statuses, poses = {}, {}, {}, {}

    def key(m):
        return (m.header.stamp.sec, m.header.stamp.nanosec)

    def seconds(stamp):
        return stamp[0] + stamp[1] / 1e9

    subscriptions = [
        node.create_subscription(
            LaserScan, "/scan", lambda m: raw.update({key(m): m}), qos_profile_sensor_data
        ),
        node.create_subscription(
            LaserScan,
            "/perception/scan_filtered",
            lambda m: filtered.update({key(m): m}),
            qos_profile_sensor_data,
        ),
        node.create_subscription(
            DiagnosticArray, "/perception/scan_status", lambda m: statuses.update({key(m): m}), 10
        ),
        node.create_subscription(
            Odometry, "/ground_truth/odom", lambda m: poses.update({key(m): m}), 10
        ),
    ]
    try:
        deadline = time.monotonic() + 40
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            for stamp in raw.keys() & filtered.keys() & statuses.keys():
                a, b, status = raw[stamp], filtered[stamp], statuses[stamp]
                if (
                    not status.status
                    or not any(
                        s.name == "scan_adapter"
                        and s.level == DiagnosticStatus.OK
                        and s.message == "usable"
                        for s in status.status
                    )
                    or not poses
                ):
                    continue
                closest = min(poses, key=lambda p: abs(seconds(p) - seconds(stamp)))
                if abs(seconds(closest) - seconds(stamp)) > 0.1:
                    continue
                try:
                    tf = buffer.lookup_transform(
                        "base_link", a.header.frame_id, Time.from_msg(a.header.stamp)
                    )
                except TransformException:
                    continue
                t = tf.transform.translation
                origin = (t.x, t.y)
                laser_yaw = yaw_of(tf.transform.rotation)
                if attachment_evidence(a, b, origin, laser_yaw) and wall_evidence(
                    a, b, origin, laser_yaw, poses[closest].pose.pose
                ):
                    assert subscriptions and listener
                    return
        pytest.fail(
            "no same-stamp usable scan showed a valid attachment hit masked "
            "and world-wall hit preserved"
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()
