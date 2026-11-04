"""Actual native raw receipt, Python filter and native matcher integration."""

import os
import signal
import subprocess
import time
from pathlib import Path

import pytest

pytestmark = pytest.mark.skipif(
    os.environ.get("SENIOR_SCAN_EPOCH_LIVE") != "1", reason="explicit ROS integration only"
)


def test_real_native_scan_filter_receipt_reaches_matcher(native_scan_process, tmp_path):
    rclpy = pytest.importorskip("rclpy")
    from geometry_msgs.msg import TransformStamped
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from senior_motion_msgs.msg import FencedFilteredScan, ScanMotion
    from senior_robot.scan_adapter import ScanAdapter
    from sensor_msgs.msg import LaserScan
    from test_scan_epoch_live import _room_scan
    from tf2_ros import StaticTransformBroadcaster

    params = Path(__file__).parents[2] / "ros2/src/senior_robot/config/simulation_motion.yaml"
    with (tmp_path / "native-matcher.log").open("w", encoding="utf-8") as log:
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
        adapter = producer = executor = None
        try:
            rclpy.init()
            producer = rclpy.create_node("actual_raw_fixture")
            adapter = ScanAdapter(
                parameter_overrides=[
                    Parameter("transform_static", value=True),
                    Parameter("self_mask_calibrated", value=True),
                    Parameter("max_age_seconds", value=0.8),
                ]
            )
            executor = SingleThreadedExecutor()
            executor.add_node(producer)
            executor.add_node(adapter)
            publisher = producer.create_publisher(LaserScan, "/scan", qos_profile_sensor_data)
            filtered, motions = [], []
            producer.create_subscription(
                FencedFilteredScan,
                "/perception/scan_filtered",
                filtered.append,
                qos_profile_sensor_data,
            )
            producer.create_subscription(ScanMotion, "/motion/scan", motions.append, 10)
            broadcaster = StaticTransformBroadcaster(producer)
            transform = TransformStamped()
            transform.header.frame_id, transform.child_frame_id = "base_link", "laser_link"
            transform.transform.rotation.w = 1.0
            adapter.buffer.set_transform_static(transform, "fixture")
            broadcaster.sendTransform(transform)
            scan = _room_scan()
            deadline = time.monotonic() + 20
            while time.monotonic() < deadline and not any(item.valid for item in motions):
                assert process.poll() is None and native_scan_process.poll() is None
                scan.header.stamp = producer.get_clock().now().to_msg()
                publisher.publish(scan)
                for _ in range(5):
                    executor.spin_once(timeout_sec=0.01)
            assert any(item.valid for item in motions), [
                (item.valid, item.reason) for item in motions[-10:]
            ]
            assert any(item.owner.valid and item.body.raw_capture.valid for item in filtered)
            valid = next(item for item in reversed(filtered) if item.owner.valid)
            assert valid.body.raw_capture.frame_id == "laser_link"
            assert valid.body.raw_capture.raw_gid and valid.body.raw_capture.received_steady > 0
            motions.clear()
            scan.time_increment = 0.001
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline and not any(not item.valid for item in motions):
                scan.header.stamp = producer.get_clock().now().to_msg()
                publisher.publish(scan)
                for _ in range(5):
                    executor.spin_once(timeout_sec=0.01)
            assert any(not item.valid for item in motions)
        finally:
            if executor is not None:
                executor.shutdown()
            if adapter is not None:
                adapter.destroy_node()
            if producer is not None:
                producer.destroy_node()
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
