"""ROS ingress lifecycle, provenance and cross-hop freshness regressions."""

import copy
import time
from types import SimpleNamespace

import pytest


def test_raw_source_replacement_retires_old_epoch_and_keeps_original_receipt(monkeypatch):
    rclpy = pytest.importorskip("rclpy")
    from geometry_msgs.msg import TransformStamped
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from senior_robot.safety_node import SafetyGate
    from senior_robot.safety_scan_ingress import SafetyScanIngress
    from sensor_msgs.msg import LaserScan

    def wait_for(predicate):
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.01)
        pytest.fail("raw ROS producer topology did not converge")

    rclpy.init()
    values = [
        Parameter("observation_max_age", value=1.0),
        Parameter("measurement_max_age", value=1.0),
        Parameter("transform_static", value=True),
        Parameter("self_mask_calibrated", value=True),
    ]
    ingress = SafetyScanIngress(parameter_overrides=values)
    gate = SafetyGate(parameter_overrides=values)
    raw = rclpy.create_node("fixture_raw_scan")
    raw_publisher = raw.create_publisher(LaserScan, "/scan", qos_profile_sensor_data)
    duplicate = None
    messages = []
    ingress._publisher = SimpleNamespace(publish=messages.append)
    monkeypatch.setattr(gate, "_scan_topology_ready", lambda: True)
    transform = TransformStamped()
    transform.header.frame_id = "base_link"
    transform.child_frame_id = "laser_link"
    transform.transform.translation.x = 0.25
    transform.transform.rotation.w = 1.0
    ingress._tf_buffer.set_transform_static(transform, "fixture")

    def raw_scan():
        scan = LaserScan()
        scan.header.frame_id = "laser_link"
        scan.header.stamp = raw.get_clock().now().to_msg()
        scan.angle_min = -1.0
        scan.angle_increment = 0.1
        scan.range_min, scan.range_max = 0.1, 8.0
        scan.ranges = [4.0] * 20
        ingress._on_scan(scan)
        assert messages[-1].valid
        return messages[-1]

    try:
        wait_for(ingress._check_source)
        first = raw_scan()
        gate._on_scan(first)
        assert gate._freshness.scan_received == first.ingress_steady
        assert gate._scans[-1].stamp == pytest.approx(first.stamp.sec + first.stamp.nanosec / 1e9)
        assert gate._scans[-1].message.raw_source == first.raw_source
        assert first.sensor_x == pytest.approx(0.25)
        duplicate = rclpy.create_node("duplicate_raw_scan")
        duplicate_publisher = duplicate.create_publisher(
            LaserScan, "/scan", qos_profile_sensor_data
        )
        wait_for(lambda: not ingress._check_source())
        gate._on_scan(messages[-1])
        assert gate._freshness.scan_received is None
        duplicate.destroy_node()
        duplicate = None
        wait_for(ingress._check_source)
        second = raw_scan()
        gate._on_scan(second)
        assert gate._freshness.scan_received == second.ingress_steady
        assert second.source_epoch != first.source_epoch
        generation = gate._scan_generation
        gate._on_scan(first)
        assert gate._scan_generation == generation
        assert gate._freshness.scan_received == second.ingress_steady
        old_invalid = copy.deepcopy(first)
        old_invalid.sequence += 100
        old_invalid.valid = False
        gate._on_scan(old_invalid)
        assert gate._scan_generation == generation
        assert gate._freshness.scan_received == second.ingress_steady

        # A delayed hop does not replace original ingress receipt with gate receipt.
        delayed = raw_scan()
        delayed.ingress_steady -= 2.0
        gate._on_scan(delayed)
        assert gate._freshness.scan_received is None
        assert raw_publisher is not None and duplicate_publisher is not None
    finally:
        if duplicate is not None:
            duplicate.destroy_node()
        raw.destroy_node()
        gate.destroy_node()
        ingress.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize("dimension", ["ranges", "intensities", "frame"])
def test_oversized_raw_dimensions_emit_only_a_bounded_invalid_event(monkeypatch, dimension):
    rclpy = pytest.importorskip("rclpy")
    from rclpy.parameter import Parameter
    from senior_robot.safety_scan_ingress import SafetyScanIngress
    from sensor_msgs.msg import LaserScan

    rclpy.init()
    ingress = SafetyScanIngress(
        parameter_overrides=[
            Parameter("observation_max_age", value=1.0),
            Parameter("transform_static", value=True),
        ]
    )
    captured = []
    ingress._source = "one-qualified-raw-source"
    ingress._publisher = SimpleNamespace(publish=captured.append)
    monkeypatch.setattr(ingress, "_check_source", lambda: True)
    try:
        scan = LaserScan()
        scan.header.stamp = ingress.get_clock().now().to_msg()
        scan.header.frame_id = "f" * 129 if dimension == "frame" else "laser_link"
        scan.ranges = [3.0] * (2049 if dimension == "ranges" else 10)
        if dimension == "intensities":
            scan.intensities = [1.0] * 2049
        ingress._on_scan(scan)
        assert len(captured) == 1
        result = captured[0]
        assert not result.valid and result.reason == "raw_scan_dimensions"
        assert not result.ranges and result.raw_source == ingress._source
        assert result.stamp == scan.header.stamp
        assert result.ingress_steady > 0
    finally:
        ingress.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize("failure", ["empty_epoch", "retirement_exhaustion"])
def test_unqualified_or_exhausted_scan_epoch_revokes_freshness(monkeypatch, failure):
    rclpy = pytest.importorskip("rclpy")
    from rclpy.parameter import Parameter
    from senior_motion_msgs.msg import BoundedSafetyScan
    from senior_robot.safety_node import SafetyGate

    from senior_control.safety.motion_runtime import SourceEpochFence

    rclpy.init()
    gate = SafetyGate(
        parameter_overrides=[
            Parameter("observation_max_age", value=1.0),
            Parameter("measurement_max_age", value=1.0),
        ]
    )
    gate._scan_fence = SourceEpochFence(max_retired=1)
    monkeypatch.setattr(gate, "_scan_topology_ready", lambda: True)

    def observation(epoch, *, valid=True):
        message = BoundedSafetyScan()
        message.stamp = gate.get_clock().now().to_msg()
        message.frame_id = "laser_link"
        message.source_epoch = epoch
        message.raw_source = "raw"
        message.sequence = 1
        message.ingress_steady = time.monotonic()
        message.ingress_ros = gate.get_clock().now().nanoseconds / 1e9
        message.valid = valid
        if valid:
            message.ranges = [2.0]
        return message

    try:
        gate._on_scan(observation("a"))
        assert gate._freshness.scan_received is not None
        if failure == "retirement_exhaustion":
            gate._on_scan(observation("b"))
            assert gate._freshness.scan_received is not None
            trigger = observation("c")
        else:
            trigger = observation("", valid=False)
        generation = gate._scan_generation
        gate._on_scan(trigger)
        assert gate._freshness.scan_received is None
        assert not gate._scans
        assert gate._scan_generation > generation
        if failure == "empty_epoch":
            recovered = observation("a")
            recovered.sequence = 2
            gate._on_scan(recovered)
            assert gate._freshness.scan_received == recovered.ingress_steady
        else:
            gate._on_scan(observation("b"))
            gate._on_scan(observation("d"))
            assert gate._freshness.scan_received is None
            assert not gate._scans
    finally:
        gate.destroy_node()
        rclpy.shutdown()
