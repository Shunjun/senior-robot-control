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
from senior_motion_msgs.msg import BoundedRawLaserScan, FencedFilteredScan, ScanMotion  # noqa: E402
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


class ControlledFencedScanPublisher:
    """Test-owned receipt stream for matcher-only invalid/epoch regressions."""

    def __init__(self, node):
        self.node = node
        self.publisher = node.create_publisher(
            FencedFilteredScan, "/perception/scan_filtered", qos_profile_sensor_data
        )
        self.raw_publisher = node.create_publisher(
            BoundedRawLaserScan, "/safety/raw/scan", qos_profile_sensor_data
        )
        self.sequence = 0

    def publish(self, scan):
        endpoints = self.node.get_publishers_info_by_topic("/perception/scan_filtered")
        raw = self.node.get_publishers_info_by_topic("/safety/raw/scan")
        if len(endpoints) != 1 or len(raw) != 1:
            return
        self.sequence += 1
        envelope = FencedFilteredScan()
        owner, body = envelope.owner, envelope.body
        owner.relay_gid = bytes(endpoints[0].endpoint_gid).hex()
        owner.relay_boot, owner.owner_gid, owner.owner_epoch = "relay", "dd" * 24, "owner"
        owner.owner_generation, owner.owner_sequence = 1, self.sequence
        owner.relay_sequence, owner.valid = self.sequence, True
        body.source_epoch, body.sequence, body.valid = "owner", self.sequence, True
        body.angle_min, body.angle_max = scan.angle_min, scan.angle_max
        body.angle_increment, body.time_increment = scan.angle_increment, scan.time_increment
        body.scan_time, body.range_min, body.range_max = (
            scan.scan_time,
            scan.range_min,
            scan.range_max,
        )
        body.ranges = scan.ranges
        capture = body.raw_capture
        capture.stamp, capture.frame_id = scan.header.stamp, scan.header.frame_id
        capture.received_ros = self.node.get_clock().now().to_msg()
        capture.received_steady = time.monotonic()
        capture.raw_gid = "cc" * 24
        capture.bridge_gid = bytes(raw[0].endpoint_gid).hex()
        capture.bridge_boot_id, capture.source_epoch = "scan-boot", "raw-epoch"
        capture.capture_generation, capture.sequence, capture.valid = 1, self.sequence, True
        self.publisher.publish(envelope)


def test_unrelated_invalid_owner_cannot_revoke_active_matcher(tmp_path, monkeypatch):
    monkeypatch.setenv("ROS_DOMAIN_ID", "100")
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    params = Path(__file__).parents[2] / "ros2/src/senior_robot/config/simulation_motion.yaml"
    with (tmp_path / "owner-invalid.log").open("w", encoding="utf-8") as log:
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
            producer = rclpy.create_node("scan_receipt")
            listener = rclpy.create_node("matcher_owner_fixture")
            executor = SingleThreadedExecutor()
            executor.add_node(producer)
            executor.add_node(listener)
            publisher = ControlledFencedScanPublisher(producer)
            received = []
            listener.create_subscription(ScanMotion, "/motion/scan", received.append, 10)
            broadcaster = StaticTransformBroadcaster(producer)
            transform = TransformStamped()
            transform.header.frame_id, transform.child_frame_id = "base_link", "laser_link"
            transform.transform.rotation.w = 1.0
            broadcaster.sendTransform(transform)
            scan = _room_scan()
            deadline = time.monotonic() + 15
            while time.monotonic() < deadline and not any(item.valid for item in received):
                assert process.poll() is None
                scan.header.stamp = producer.get_clock().now().to_msg()
                publisher.publish(scan)
                executor.spin_once(timeout_sec=0.01)
            assert any(item.valid for item in received)
            received.clear()
            publisher.sequence += 1
            invalid = FencedFilteredScan()
            invalid.owner.relay_gid = bytes(
                producer.get_publishers_info_by_topic("/perception/scan_filtered")[0].endpoint_gid
            ).hex()
            invalid.owner.relay_boot, invalid.owner.relay_sequence = "relay", publisher.sequence
            invalid.owner.owner_gid, invalid.owner.owner_epoch = "ee" * 24, "unrelated-owner"
            invalid.owner.owner_generation, invalid.owner.owner_sequence = 1, publisher.sequence - 1
            invalid.owner.upstream_invalid, invalid.owner.reason = True, "unrelated_invalid"
            publisher.publisher.publish(invalid)
            deadline = time.monotonic() + 0.5
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.01)
            assert not any(item.reason == "unrelated_invalid" for item in received)
            # The actual active owner's new invalid candidate must revoke it.
            publisher.sequence += 1
            invalid.owner.relay_sequence = publisher.sequence
            invalid.owner.owner_gid, invalid.owner.owner_epoch = "dd" * 24, "owner"
            invalid.owner.owner_sequence = publisher.sequence
            invalid.owner.upstream_invalid = False
            invalid.body.source_epoch, invalid.body.sequence = "owner", publisher.sequence
            invalid.owner.reason = "current_invalid"
            publisher.publisher.publish(invalid)
            deadline = time.monotonic() + 2
            while time.monotonic() < deadline and not any(
                item.reason == "current_invalid" for item in received
            ):
                executor.spin_once(timeout_sec=0.01)
            assert any(not item.valid and item.reason == "current_invalid" for item in received)
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
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)


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
            producer = rclpy.create_node("scan_receipt")
            listener = rclpy.create_node("scan_epoch_test_listener")
            executor = SingleThreadedExecutor()
            executor.add_node(producer)
            executor.add_node(listener)
            publisher = ControlledFencedScanPublisher(producer)
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
