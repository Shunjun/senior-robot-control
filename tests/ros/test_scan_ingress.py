"""ROS ingress lifecycle, provenance and cross-hop freshness regressions."""

import copy
import time
from types import SimpleNamespace

import pytest


@pytest.mark.parametrize("survivor", ["old", "foreign"])
def test_raw_source_replacement_retires_old_epoch_and_keeps_original_receipt(
    native_scan_process, survivor
):
    rclpy = pytest.importorskip("rclpy")
    from geometry_msgs.msg import TransformStamped
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from senior_motion_msgs.msg import BoundedRawLaserScan, FencedSafetyScan
    from senior_robot.safety_node import SafetyGate
    from senior_robot.safety_scan_ingress import SafetyScanIngress
    from sensor_msgs.msg import LaserScan

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
    raw_messages = []
    raw.create_subscription(FencedSafetyScan, "/safety/bounded_scan", messages.append, 10)
    raw.create_subscription(
        BoundedRawLaserScan, "/safety/raw/scan", raw_messages.append, qos_profile_sensor_data
    )
    transform = TransformStamped()
    transform.header.frame_id, transform.child_frame_id = "base_link", "laser_link"
    transform.transform.translation.x, transform.transform.rotation.w = 0.25, 1.0
    ingress._tf_buffer.set_transform_static(transform, "fixture")
    executor = SingleThreadedExecutor()
    for node in (raw, ingress, gate):
        executor.add_node(node)

    def publish():
        scan = LaserScan()
        scan.header.frame_id, scan.header.stamp = "laser_link", raw.get_clock().now().to_msg()
        scan.angle_min, scan.angle_max, scan.angle_increment = -1.0, 0.9, 0.1
        scan.range_min, scan.range_max = 0.1, 8.0
        scan.ranges = [4.0] * 20
        raw_publisher.publish(scan)

    def spin_until(predicate):
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline and not predicate():
            assert native_scan_process.poll() is None
            publish()
            for _ in range(5):
                executor.spin_once(timeout_sec=0.01)
        assert predicate()

    try:

        def matching_observation():
            return any(
                value.owner.valid and value.body.ingress_steady == gate._freshness.scan_received
                for value in messages
            )

        spin_until(matching_observation)
        first = next(
            value
            for value in reversed(messages)
            if value.owner.valid and value.body.ingress_steady == gate._freshness.scan_received
        )
        assert gate._freshness.scan_received == first.body.ingress_steady
        assert gate._scans[-1].stamp == pytest.approx(
            first.body.stamp.sec + first.body.stamp.nanosec / 1e9
        )
        assert gate._scans[-1].message.raw_source == first.body.raw_source
        assert first.body.sensor_x == pytest.approx(0.25)
        duplicate = rclpy.create_node("duplicate_raw_scan")
        duplicate_publisher = duplicate.create_publisher(
            LaserScan, "/scan", qos_profile_sensor_data
        )
        spin_until(lambda: gate._freshness.scan_received is None)
        # A foreign callback observed during ambiguity is conservatively retired too.
        endpoints = raw.get_publishers_info_by_topic("/scan")
        foreign = next(value for value in endpoints if value.node_name == "duplicate_raw_scan")
        foreign_gid = bytes(foreign.endpoint_gid).hex()
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline and not any(
            value.capture.reason == "raw_gid_mismatch" and value.capture.raw_gid == foreign_gid
            for value in raw_messages
        ):
            scan = LaserScan()
            scan.header.frame_id, scan.header.stamp = "laser_link", raw.get_clock().now().to_msg()
            scan.angle_min, scan.angle_max, scan.angle_increment = -1.0, 0.9, 0.1
            scan.range_min, scan.range_max, scan.ranges = 0.1, 8.0, [4.0] * 20
            duplicate_publisher.publish(scan)
            for _ in range(5):
                executor.spin_once(timeout_sec=0.01)
        assert any(
            value.capture.reason == "raw_gid_mismatch" and value.capture.raw_gid == foreign_gid
            for value in raw_messages
        )
        if survivor == "old":
            duplicate.destroy_node()
            duplicate = None
        else:
            raw.destroy_publisher(raw_publisher)
        deadline = time.monotonic() + 0.3
        while time.monotonic() < deadline:
            if survivor == "old":
                publish()
            else:
                scan.header.stamp = raw.get_clock().now().to_msg()
                duplicate_publisher.publish(scan)
            executor.spin_once(timeout_sec=0.01)
        assert gate._freshness.scan_received is None
        if survivor == "old":
            raw.destroy_publisher(raw_publisher)
        else:
            duplicate.destroy_node()
            duplicate = None
        raw_publisher = raw.create_publisher(LaserScan, "/scan", qos_profile_sensor_data)
        spin_until(matching_observation)
        second = next(
            value
            for value in reversed(messages)
            if value.owner.valid and value.body.ingress_steady == gate._freshness.scan_received
        )
        assert second.body.source_epoch != first.body.source_epoch
        generation = gate._scan_generation
        gate._on_scan(first)
        assert gate._scan_generation == generation
        assert gate._freshness.scan_received == second.body.ingress_steady
        old_invalid = copy.deepcopy(first)
        old_invalid.owner.relay_sequence += 1000
        old_invalid.owner.owner_sequence += 1000
        old_invalid.owner.valid = False
        gate._on_scan(old_invalid)
        assert gate._scan_generation == generation
        assert gate._freshness.scan_received == second.body.ingress_steady
        delayed = copy.deepcopy(second)
        delayed.owner.relay_sequence += 1000
        delayed.owner.owner_sequence += 1000
        delayed.body.sequence += 1000
        delayed.body.ingress_steady -= 2.0
        gate._on_scan(delayed)
        assert gate._freshness.scan_received is None
        assert duplicate_publisher is not None
    finally:
        executor.shutdown()
        if duplicate is not None:
            duplicate.destroy_node()
        for node in (raw, gate, ingress):
            node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize("dimension", ["ranges", "intensities", "frame"])
def test_oversized_raw_dimensions_emit_only_a_bounded_invalid_event(native_scan_process, dimension):
    rclpy = pytest.importorskip("rclpy")
    from rclpy.qos import qos_profile_sensor_data
    from senior_motion_msgs.msg import BoundedRawLaserScan
    from sensor_msgs.msg import LaserScan

    rclpy.init()
    node = rclpy.create_node("oversized_scan_fixture")
    publisher = node.create_publisher(LaserScan, "/scan", qos_profile_sensor_data)
    captured = []
    node.create_subscription(
        BoundedRawLaserScan, "/safety/raw/scan", captured.append, qos_profile_sensor_data
    )
    try:
        scan = LaserScan()
        scan.header.frame_id = "f" * 129 if dimension == "frame" else "laser_link"
        scan.angle_min, scan.angle_max, scan.angle_increment = -1.0, 1.0, 0.1
        scan.range_min, scan.range_max = 0.1, 8.0
        scan.ranges = [3.0] * (2049 if dimension == "ranges" else 10)
        if dimension == "intensities":
            scan.intensities = [1.0] * 2049
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline and not any(
            not item.capture.valid and item.capture.reason == "raw_scan_invalid"
            for item in captured
        ):
            assert native_scan_process.poll() is None
            scan.header.stamp = node.get_clock().now().to_msg()
            publisher.publish(scan)
            rclpy.spin_once(node, timeout_sec=0.02)
        result = next(item for item in captured if item.capture.reason == "raw_scan_invalid")
        assert not result.capture.valid and not result.ranges
        assert result.capture.raw_gid and result.capture.stamp.sec > 0
        assert result.capture.received_steady > 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize("failure", ["empty_epoch", "retirement_exhaustion"])
def test_unqualified_or_exhausted_scan_epoch_revokes_freshness(monkeypatch, failure):
    rclpy = pytest.importorskip("rclpy")
    from rclpy.parameter import Parameter
    from senior_motion_msgs.msg import BoundedSafetyScan, FencedSafetyScan
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
    gate._scan_relay_gid = "aa" * 24
    monkeypatch.setattr(
        gate,
        "get_publishers_info_by_topic",
        lambda _: [SimpleNamespace(endpoint_gid=bytes.fromhex("bb" * 24))],
    )
    relay_sequence = 0
    raw_sequence = 0
    generations = {}

    def observation(epoch, *, valid=True):
        nonlocal relay_sequence, raw_sequence
        relay_sequence += 1
        raw_sequence += 1
        if epoch and epoch not in generations:
            generations[epoch] = len(generations) + 1
        message = BoundedSafetyScan()
        message.stamp = gate.get_clock().now().to_msg()
        message.frame_id, message.source_epoch = "laser_link", epoch
        message.raw_source, message.sequence = "cc" * 24, raw_sequence
        message.ingress_steady = time.monotonic()
        message.ingress_ros = gate.get_clock().now().nanoseconds / 1e9
        message.valid = valid
        message.ranges = [2.0] if valid else []
        capture = message.raw_capture
        capture.stamp = message.stamp
        capture.received_ros = Time(nanoseconds=int(message.ingress_ros * 1e9)).to_msg()
        message.ingress_ros = Time.from_msg(capture.received_ros).nanoseconds / 1e9
        capture.received_steady, capture.frame_id = message.ingress_steady, message.frame_id
        capture.raw_gid, capture.bridge_gid = message.raw_source, "bb" * 24
        capture.source_epoch, capture.bridge_boot_id = "raw-epoch", "bridge-boot"
        capture.capture_generation, capture.sequence, capture.valid = 1, raw_sequence, True
        envelope = FencedSafetyScan()
        envelope.body = message
        envelope.owner.relay_gid, envelope.owner.relay_boot = gate._scan_relay_gid, "relay-boot"
        envelope.owner.relay_sequence = relay_sequence
        envelope.owner.owner_gid, envelope.owner.owner_epoch = "dd" * 24, epoch
        envelope.owner.owner_generation = generations.get(epoch, len(generations) + 1)
        envelope.owner.owner_sequence, envelope.owner.valid = raw_sequence, valid
        return envelope

    from rclpy.time import Time

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
            gate._on_scan(recovered)
            assert gate._freshness.scan_received == recovered.body.ingress_steady
        else:
            gate._on_scan(observation("b"))
            gate._on_scan(observation("d"))
            assert gate._freshness.scan_received is None
            assert not gate._scans
    finally:
        gate.destroy_node()
        rclpy.shutdown()


def test_scan_consumers_require_fenced_and_original_bounded_sources():
    rclpy = pytest.importorskip("rclpy")
    from rclpy.parameter import Parameter
    from senior_motion_msgs.msg import BoundedRawLaserScan, FencedSafetyScan
    from senior_robot.safety_node import SafetyGate
    from senior_robot.safety_scan_ingress import SafetyScanIngress
    from senior_robot.scan_adapter import ScanAdapter

    rclpy.init()
    ingress = SafetyScanIngress(parameter_overrides=[Parameter("observation_max_age", value=0.5)])
    adapter = ScanAdapter()
    gate = SafetyGate(
        parameter_overrides=[
            Parameter("observation_max_age", value=0.5),
            Parameter("measurement_max_age", value=0.5),
        ]
    )
    try:
        ingress_types = {item.topic_name: item.msg_type for item in ingress._subscriptions}
        adapter_types = {item.topic_name: item.msg_type for item in adapter._subscriptions}
        gate_types = {item.topic_name: item.msg_type for item in gate._subscriptions}
        assert ingress_types["/safety/raw/scan"] is BoundedRawLaserScan
        assert adapter_types["/safety/raw/scan"] is BoundedRawLaserScan
        assert gate_types["/safety/bounded_scan"] is FencedSafetyScan
    finally:
        gate.destroy_node()
        adapter.destroy_node()
        ingress.destroy_node()
        rclpy.shutdown()
