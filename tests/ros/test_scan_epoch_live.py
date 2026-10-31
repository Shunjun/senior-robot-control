"""Exercise a real matcher process across a sustained invalid sensor interval."""

import math
import os
import signal
import subprocess
import time
from pathlib import Path

import pytest

pytestmark = pytest.mark.skipif(
    os.environ.get("SENIOR_SCAN_EPOCH_LIVE") != "1", reason="explicit ROS node integration only"
)
rclpy = pytest.importorskip("rclpy")
from geometry_msgs.msg import TransformStamped  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from senior_motion_msgs.msg import ScanMotion  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402
from tf2_ros import StaticTransformBroadcaster  # noqa: E402

from senior_control.safety.motion_runtime import SourceEpochFence  # noqa: E402


def _room_scan():
    scan = LaserScan()
    scan.header.frame_id = "laser_link"
    scan.angle_min = -math.pi
    scan.angle_increment = 2 * math.pi / 720
    scan.angle_max = scan.angle_min + 719 * scan.angle_increment
    scan.range_min, scan.range_max = 0.08, 8.0
    for index in range(720):
        a = scan.angle_min + index * scan.angle_increment
        dx, dy = math.cos(a), math.sin(a)
        distances = []
        for wall in (-2.3, 3.1):
            t = wall / dx
            if t > 0 and -2.0 < t * dy < 2.7:
                distances.append(t)
        for wall in (-2.0, 2.7):
            if abs(dy) < 1e-9:
                continue
            t = wall / dy
            if t > 0 and -2.3 < t * dx < 3.1:
                distances.append(t)
        scan.ranges.append(min(distances, default=float("nan")))
    return scan


def test_invalid_stream_does_not_exhaust_epochs_and_recovers(tmp_path, monkeypatch):
    domain = os.environ.get("SENIOR_MOTION_DOMAIN", "")
    assert domain.isdecimal() and 0 <= int(domain) <= 101, "choose an isolated ROS domain"
    monkeypatch.setenv("ROS_DOMAIN_ID", domain)
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    params = Path(__file__).parents[2] / "ros2/src/senior_robot/config/simulation_motion.yaml"
    log_path = tmp_path / "scan-motion.log"
    with log_path.open("w", encoding="utf-8") as log:
        process = subprocess.Popen(
            [
                "ros2",
                "run",
                "senior_scan_matcher",
                "scan_motion_node",
                "--ros-args",
                "--params-file",
                str(params),
                "-p",
                "use_sim_time:=false",
            ],
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        producer = listener = executor = None
        try:
            rclpy.init()
            producer = rclpy.create_node("scan_adapter")
            listener = rclpy.create_node("scan_epoch_test_listener")
            executor = SingleThreadedExecutor()
            executor.add_node(producer)
            executor.add_node(listener)
            publisher = producer.create_publisher(
                LaserScan, "/perception/scan_filtered", qos_profile_sensor_data
            )
            received = []
            listener.create_subscription(ScanMotion, "/motion/scan", received.append, 10)
            broadcaster = StaticTransformBroadcaster(producer)
            transform = TransformStamped()
            transform.header.stamp = producer.get_clock().now().to_msg()
            transform.header.frame_id = "base_link"
            transform.child_frame_id = "laser_link"
            transform.transform.rotation.w = 1.0
            broadcaster.sendTransform(transform)
            scan = _room_scan()

            def exchange(until, *, bad, timeout):
                deadline = time.monotonic() + timeout
                while time.monotonic() < deadline and not until():
                    assert process.poll() is None, "matcher process exited"
                    scan.header.stamp = producer.get_clock().now().to_msg()
                    scan.time_increment = 0.001 if bad else 0.0
                    publisher.publish(scan)
                    executor.spin_once(timeout_sec=0.01)
                    time.sleep(0.003)
                log.flush()
                assert until(), (
                    [(m.valid, m.reason) for m in received[-10:]],
                    log_path.read_text(encoding="utf-8", errors="replace")[-4000:],
                )

            exchange(lambda: any(m.valid for m in received), bad=False, timeout=20)
            old_epoch = next(m.source_epoch for m in reversed(received) if m.valid)
            received.clear()
            exchange(lambda: len(received) >= 300, bad=True, timeout=20)
            invalid = [m for m in received if m.reason == "time_or_deskew_unqualified"]
            assert len(invalid) >= 257
            epochs = {m.source_epoch for m in invalid}
            assert len(epochs) == 1 and old_epoch not in epochs
            new_epoch = next(iter(epochs))
            fence = SourceEpochFence(max_retired=256)
            assert fence.accept(old_epoch)
            assert all(fence.accept(m.source_epoch) for m in invalid)
            assert not fence.accept(old_epoch)
            received.clear()
            exchange(lambda: any(m.valid for m in received), bad=False, timeout=20)
            assert next(m.source_epoch for m in reversed(received) if m.valid) == new_epoch
        finally:
            if executor is not None:
                executor.shutdown()
            if producer is not None:
                producer.destroy_node()
            if listener is not None:
                listener.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                pass
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.wait(timeout=5)
