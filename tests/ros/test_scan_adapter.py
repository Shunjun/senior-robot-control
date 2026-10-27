"""Run against the separately launched baseline simulator and scan adapter."""

import math
import time

import pytest

rclpy = pytest.importorskip("rclpy")
from diagnostic_msgs.msg import DiagnosticArray  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402


def test_simulated_self_and_wall_evidence():
    rclpy.init()
    node = rclpy.create_node("scan_filter_runtime_test")
    raw, filtered, statuses = {}, {}, []

    def key(m):
        return (m.header.stamp.sec, m.header.stamp.nanosec)

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
        node.create_subscription(DiagnosticArray, "/perception/scan_status", statuses.append, 10),
    ]
    try:
        deadline = time.monotonic() + 40
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            for stamp in raw.keys() & filtered.keys():
                a, b = raw[stamp], filtered[stamp]
                masked = [
                    i
                    for i, (x, y) in enumerate(zip(a.ranges, b.ranges, strict=False))
                    if math.isfinite(x) and math.isnan(y)
                ]
                preserved = [
                    i
                    for i, (x, y) in enumerate(zip(a.ranges, b.ranges, strict=False))
                    if math.isfinite(x)
                    and x == y
                    and abs(a.angle_min + i * a.angle_increment) < 0.3
                ]
                if (
                    masked
                    and preserved
                    and any(s.status and s.status[0].level == 0 for s in statuses)
                ):
                    assert all(math.isnan(b.ranges[i]) for i in masked)
                    assert all(a.ranges[i] < 4 for i in preserved)
                    assert subscriptions
                    return
        pytest.fail("no matched scan demonstrated masked self hits and preserved front wall")
    finally:
        node.destroy_node()
        rclpy.shutdown()
