"""ROS typed ingress with real message buffers and measurement-time TF."""

import math
import struct
from types import SimpleNamespace

import pytest


def test_depth_ingress_publishes_original_typed_geometry_and_invalid(monkeypatch):
    rclpy = pytest.importorskip("rclpy")
    from geometry_msgs.msg import TransformStamped
    from rclpy.parameter import Parameter
    from senior_motion_msgs.msg import BoundedDepthEvidence, BoundedRawCameraInfo, BoundedRawImage
    from senior_robot.depth_ingress import DepthIngress

    rclpy.init()
    node = DepthIngress(parameter_overrides=[
        Parameter("min_range_m", value=0.1), Parameter("max_range_m", value=8.0),
        Parameter("range_error_m", value=0.01), Parameter("row_stride", value=1),
        Parameter("column_stride", value=1), Parameter("max_age_s", value=0.5),
        Parameter("transform_static", value=True),
    ])
    captured = []
    node._publisher = SimpleNamespace(publish=captured.append)
    gids = {node._depth_topic: "01" * 24, node._info_topic: "02" * 24}
    monkeypatch.setattr(node, "_source", lambda topic: gids[topic])
    tf = TransformStamped()
    tf.header.frame_id = "base_link"
    tf.child_frame_id = "camera_optical_frame"
    tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = (
        0.2316, 0.00224, 0.1282
    )
    tf.transform.rotation.x, tf.transform.rotation.y = -0.5, 0.5
    tf.transform.rotation.z, tf.transform.rotation.w = -0.5, 0.5
    node._tf_buffer.set_transform_static(tf, "fixture")
    try:
        stamp = node.get_clock().now().to_msg()
        info = BoundedRawCameraInfo()
        info.capture.stamp, info.capture.frame_id = stamp, "camera_optical_frame"
        info.width = info.height = 3
        info.distortion_model = "plumb_bob"
        info.d = [0.0] * 5
        info.k = [2.0, 0.0, 1.0, 0.0, 4.0, 1.0, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [2.0, 0.0, 1.0, 0.0, 0.0, 4.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        image = BoundedRawImage()
        image.capture.stamp, image.capture.frame_id = stamp, "camera_optical_frame"
        image.width = image.height = 3
        image.encoding, image.step = "32FC1", 16
        image.data = b"".join(struct.pack("<fff", *row) + b"pad!" for row in (
            (math.nan, 1.0, math.nan), (math.nan, 0.6, math.nan),
            (math.nan, 0.4, math.nan),
        ))
        import time
        for message, bridge_gid, raw_gid in (
            (image, gids[node._depth_topic], "03" * 24),
            (info, gids[node._info_topic], "04" * 24),
        ):
            capture = message.capture
            capture.received_ros, capture.received_steady = stamp, time.monotonic()
            capture.bridge_gid, capture.raw_gid = bridge_gid, raw_gid
            capture.source_epoch, capture.bridge_boot_id = raw_gid, "bridge-boot"
            capture.capture_generation, capture.sequence, capture.valid = 1, 1, True
        node._on_info(info)
        node._on_depth(image)
        result = next(value for value in captured if value.valid)
        assert isinstance(result, BoundedDepthEvidence)
        assert result.valid and result.stamp == stamp and result.camera_info_stamp == stamp
        assert result.frame_id == "camera_optical_frame" and result.body_frame == "base_link"
        assert len(result.samples) == 3
        assert result.samples[-1].z == pytest.approx(0.0282)
        assert result.sensor_origin == pytest.approx([0.2316, 0.00224, 0.1282])
        assert result.received_steady == image.capture.received_steady
        assert result.camera_info_received_steady == info.capture.received_steady
        assert result.depth_capture == image.capture
        assert result.camera_info_capture == info.capture
        count = sum(item.valid for item in captured)
        image.capture.sequence += 1
        image.capture.bridge_gid = "09" * 24
        node._on_depth(image)
        assert sum(item.valid for item in captured) == count
        image.capture.bridge_gid = gids[node._depth_topic]
        info.capture.capture_generation = 2
        info.capture.sequence += 1
        node._on_info(info)
        image.capture.sequence += 1
        node._on_depth(image)
        assert sum(item.valid for item in captured) == count
        image.capture.capture_generation = 2
        image.capture.sequence += 1
        node._on_depth(image)
        assert sum(item.valid for item in captured) == count + 1
        info.capture.capture_generation = 3
        info.capture.sequence += 1
        info.capture.valid, info.capture.reason = False, "raw_source_changed"
        node._on_info(info)
        assert not captured[-1].valid and captured[-1].reason == "raw_source_changed"
        assert captured[-1].camera_info_capture == info.capture
        image.capture.capture_generation = 3
        image.capture.sequence += 1
        image.encoding = "unsupported"
        node._on_depth(image)
        assert not captured[-1].valid
        assert captured[-1].source_epoch != result.source_epoch
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_depth_gate_revokes_tickets_on_new_invalid_and_replaced_epoch(monkeypatch):
    import copy
    import time

    rclpy = pytest.importorskip("rclpy")
    from rclpy.parameter import Parameter
    from rclpy.time import Time
    from senior_motion_msgs.msg import BoundedDepthEvidence, DepthSample, FencedDepthEvidence
    from senior_robot.safety_node import SafetyGate

    rclpy.init()
    gate = SafetyGate(parameter_overrides=[
        Parameter("observation_max_age", value=0.5),
        Parameter("measurement_max_age", value=0.5),
        Parameter("depth_policy", value="required"),
    ])
    monkeypatch.setattr(gate, "_depth_topology_ready", lambda: True)
    gate._depth_relay_gid = "aa" * 24
    monkeypatch.setattr(gate, "get_publishers_info_by_topic", lambda topic: [SimpleNamespace(
        endpoint_gid=bytes.fromhex("11" * 24 if topic == gate._raw_depth_topic else "22" * 24)
    )])
    relay_sequence = 0
    stops = []
    monkeypatch.setattr(gate, "_publish", lambda velocity, reason: stops.append((velocity, reason)))

    def message(epoch, sequence):
        nonlocal relay_sequence
        relay_sequence += 1
        now = gate.get_clock().now()
        msg = BoundedDepthEvidence()
        msg.source_epoch, msg.sequence = epoch, sequence
        msg.raw_source, msg.camera_info_source = "33" * 24, "44" * 24
        msg.calibration_id = "a" * 64
        msg.frame_id, msg.body_frame = "camera_optical_frame", "base_link"
        msg.stamp = msg.camera_info_stamp = msg.received_ros = msg.camera_info_received_ros = (
            Time(nanoseconds=now.nanoseconds).to_msg()
        )
        msg.received_steady = msg.camera_info_received_steady = time.monotonic()
        msg.transform_static, msg.valid = True, True
        msg.width = msg.height = 3
        msg.row_stride = msg.column_stride = 1
        msg.fx, msg.fy, msg.cx, msg.cy = 2.0, 4.0, 1.0, 1.0
        msg.min_range_m, msg.max_range_m, msg.range_error_m = 0.1, 8.0, 0.01
        msg.sensor_origin = [0.2316, 0.00224, 0.1282]
        msg.optical_to_body_quaternion = [-0.5, 0.5, -0.5, 0.5]
        sample = DepthSample()
        sample.u, sample.v, sample.depth_m = 1, 2, 0.4
        sample.x, sample.y, sample.z = 0.6316, 0.00224, 0.0282
        msg.samples = [sample]
        for capture, raw, bridge in (
            (msg.depth_capture, msg.raw_source, "11" * 24),
            (msg.camera_info_capture, msg.camera_info_source, "22" * 24),
        ):
            capture.stamp, capture.received_ros = msg.stamp, msg.received_ros
            capture.received_steady, capture.frame_id = msg.received_steady, msg.frame_id
            capture.raw_gid, capture.bridge_gid = raw, bridge
            capture.source_epoch, capture.bridge_boot_id = raw, "raw-boot"
            capture.capture_generation, capture.sequence, capture.valid = 1, sequence, True
        value = FencedDepthEvidence()
        value.body = msg
        owner = value.owner
        owner.relay_gid, owner.relay_boot, owner.relay_sequence = (
            gate._depth_relay_gid, "relay", relay_sequence
        )
        owner.owner_gid, owner.owner_epoch = "55" * 24, epoch
        owner.owner_generation, owner.owner_sequence = (1 if epoch == "epoch-a" else 2), sequence
        owner.valid = True
        return value

    try:
        first = message("epoch-a", 1)
        gate._on_depth(first)
        assert gate._depth_admission.latest is not None
        old_generation = gate._depth_admission.generation
        assert gate._depth_still_usable(
            old_generation, gate.get_clock().now().nanoseconds, time.monotonic()
        )
        second = message("epoch-a", 2)
        gate._on_depth(second)
        assert gate._depth_admission.generation > old_generation
        assert not gate._depth_still_usable(
            old_generation, gate.get_clock().now().nanoseconds, time.monotonic()
        )
        assert stops[-1][1] == "depth_observation_changed"
        invalid = message("epoch-a", 3)
        invalid.owner.valid = False
        invalid.body.valid = False
        gate._on_depth(invalid)
        assert gate._depth_admission.latest is None
        recovery = message("epoch-a", 4)
        gate._on_depth(recovery)
        assert gate._depth_admission.latest is not None
        assert gate._depth_admission.latest.source_epoch == "epoch-a"
        replacement = message("epoch-b", 1)
        gate._on_depth(replacement)
        generation = gate._depth_admission.generation
        replay = copy.deepcopy(first)
        replay.owner.owner_sequence = 999
        replay.owner.relay_sequence = 999
        replay.owner.valid = False
        gate._on_depth(replay)
        assert gate._depth_admission.generation == generation
        assert gate._depth_admission.latest.source_epoch == "epoch-b"
        frozen_ros = gate.get_clock().now().nanoseconds
        assert not gate._depth_still_usable(
            generation, frozen_ros, replacement.body.received_steady + 0.501
        )
    finally:
        gate.destroy_node()
        rclpy.shutdown()


def test_real_ros_depth_publication_reaches_gate_and_invalid_revokes():
    import time

    rclpy = pytest.importorskip("rclpy")
    from geometry_msgs.msg import TransformStamped
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from senior_motion_msgs.msg import (
        BoundedDepthEvidence,
        BoundedRawCameraInfo,
        BoundedRawImage,
        FencedDepthEvidence,
    )
    from senior_robot.depth_ingress import DepthIngress
    from senior_robot.safety_node import SafetyGate

    rclpy.init()
    ingress = DepthIngress(parameter_overrides=[
        Parameter("min_range_m", value=0.1), Parameter("max_range_m", value=8.0),
        Parameter("range_error_m", value=0.01), Parameter("row_stride", value=1),
        Parameter("column_stride", value=1), Parameter("max_age_s", value=1.0),
        Parameter("transform_static", value=True),
    ])
    gate = SafetyGate(parameter_overrides=[
        Parameter("observation_max_age", value=1.0),
        Parameter("measurement_max_age", value=1.0),
        Parameter("depth_policy", value="required"),
    ])
    fixture = rclpy.create_node("raw_receipt")
    depth_pub = fixture.create_publisher(
        BoundedRawImage, "/safety/raw/depth", qos_profile_sensor_data
    )
    info_pub = fixture.create_publisher(
        BoundedRawCameraInfo, "/safety/raw/depth_info", qos_profile_sensor_data
    )
    sequence = 0
    seen = []
    fenced_pub = fixture.create_publisher(
        FencedDepthEvidence, "/safety/bounded_depth", qos_profile_sensor_data
    )
    relay_sequence = 0
    owner_generation = 0
    previous_epoch = ""

    def forward_fixture(body):
        nonlocal relay_sequence, owner_generation, previous_epoch
        seen.append(body)
        relay = fixture.get_publishers_info_by_topic("/safety/bounded_depth")
        producers = fixture.get_publishers_info_by_topic("/safety/depth_candidate")
        if len(relay) != 1 or len(producers) != 1 or not body.source_epoch:
            return
        if body.source_epoch != previous_epoch:
            previous_epoch = body.source_epoch
            owner_generation += 1
        relay_sequence += 1
        value = FencedDepthEvidence()
        value.body = body
        value.owner.relay_gid = bytes(relay[0].endpoint_gid).hex()
        value.owner.relay_boot, value.owner.relay_sequence = "fixture-relay", relay_sequence
        value.owner.owner_gid = bytes(producers[0].endpoint_gid).hex()
        value.owner.owner_epoch, value.owner.owner_sequence = body.source_epoch, body.sequence
        value.owner.owner_generation, value.owner.valid = owner_generation, body.valid
        value.owner.reason = body.reason
        fenced_pub.publish(value)

    fixture.create_subscription(BoundedDepthEvidence, "/safety/depth_candidate", forward_fixture,
                                qos_profile_sensor_data)
    tf = TransformStamped()
    tf.header.frame_id, tf.child_frame_id = "base_link", "camera_optical_frame"
    tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = (
        0.2316, 0.00224, 0.1282
    )
    tf.transform.rotation.x, tf.transform.rotation.y = -0.5, 0.5
    tf.transform.rotation.z, tf.transform.rotation.w = -0.5, 0.5
    ingress._tf_buffer.set_transform_static(tf, "fixture")
    executor = SingleThreadedExecutor()
    for node in (fixture, ingress, gate):
        executor.add_node(node)

    def publish_pair():
        nonlocal sequence
        sequence += 1
        stamp = fixture.get_clock().now().to_msg()
        info = BoundedRawCameraInfo()
        info.capture.stamp, info.capture.frame_id = stamp, "camera_optical_frame"
        info.width = info.height = 3
        info.distortion_model = "plumb_bob"
        info.d = [0.0] * 5
        info.k = [2.0, 0.0, 1.0, 0.0, 4.0, 1.0, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [2.0, 0.0, 1.0, 0.0, 0.0, 4.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        image = BoundedRawImage()
        image.capture.stamp, image.capture.frame_id = stamp, "camera_optical_frame"
        image.width = image.height = 3
        image.encoding, image.step = "32FC1", 16
        image.data = b"".join(struct.pack("<fff", *row) + b"pad!" for row in (
            (math.nan, 1.0, math.nan), (math.nan, 0.6, math.nan),
            (math.nan, 0.4, math.nan),
        ))
        for message, topic, raw in (
            (image, "/safety/raw/depth", "01" * 24),
            (info, "/safety/raw/depth_info", "02" * 24),
        ):
            endpoints = fixture.get_publishers_info_by_topic(topic)
            if len(endpoints) != 1:
                return image
            capture = message.capture
            capture.received_ros, capture.received_steady = stamp, time.monotonic()
            capture.raw_gid, capture.bridge_gid = raw, bytes(endpoints[0].endpoint_gid).hex()
            capture.source_epoch, capture.bridge_boot_id = raw, "bridge-boot"
            capture.capture_generation, capture.sequence, capture.valid = 1, sequence, True
        info_pub.publish(info)
        depth_pub.publish(image)
        return image

    try:
        deadline = time.monotonic() + 6
        last = None
        while time.monotonic() < deadline and (
            gate._depth_admission.latest is None or not any(msg.valid for msg in seen)
        ):
            last = publish_pair()
            for _ in range(8):
                executor.spin_once(timeout_sec=0.01)
        assert last is not None and any(msg.valid for msg in seen)
        assert gate._depth_admission.latest is not None
        msg = next(msg for msg in reversed(seen) if msg.valid)
        assert any(sample.z == pytest.approx(0.0282) for sample in msg.samples)
        assert msg.depth_capture.raw_gid == "01" * 24
        assert msg.camera_info_capture.raw_gid == "02" * 24
        assert msg.depth_capture.bridge_boot_id == msg.camera_info_capture.bridge_boot_id
        assert msg.depth_capture.capture_generation == msg.camera_info_capture.capture_generation
        assert gate._depth_usable(gate.get_clock().now().nanoseconds, time.monotonic()) is not None
        last.capture.stamp = fixture.get_clock().now().to_msg()
        last.capture.received_ros = last.capture.stamp
        last.capture.received_steady = time.monotonic()
        last.capture.sequence += 1
        last.encoding = "unsupported"
        depth_pub.publish(last)
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline and gate._depth_admission.latest is not None:
            executor.spin_once(timeout_sec=0.02)
        assert gate._depth_admission.latest is None
        assert any(not msg.valid for msg in seen)
    finally:
        executor.shutdown()
        for node in (fixture, ingress, gate):
            node.destroy_node()
        rclpy.shutdown()


def test_default_required_depth_missing_stops_after_freshness_checks(monkeypatch):
    import time

    rclpy = pytest.importorskip("rclpy")
    from rclpy.parameter import Parameter
    from senior_robot.safety_node import SafetyGate

    from senior_control.models import Velocity

    rclpy.init()
    gate = SafetyGate(parameter_overrides=[
        Parameter("observation_max_age", value=0.5),
        Parameter("measurement_max_age", value=0.5),
        Parameter("command_ttl_seconds", value=0.5),
    ])
    emitted = []
    monkeypatch.setattr(
        gate, "_publish", lambda velocity, reason, **_: emitted.append((velocity, reason))
    )
    monkeypatch.setattr(gate, "_topology_ready", lambda: True)
    monkeypatch.setattr(gate, "_command_topology_ready", lambda: True)
    monkeypatch.setattr(gate, "_poll_command_guard", lambda: None)
    monkeypatch.setattr(gate, "_scan_topology_ready", lambda: True)
    gate._command = Velocity(0.05, 0.0)
    gate._command_monotonic = time.monotonic()
    gate._command_deadline = (
        gate._command_monotonic + gate.get_parameter("command_ttl_seconds").value
    )
    now_ros = gate.get_clock().now().nanoseconds / 1e9
    assert gate._freshness.scan(now_ros, gate._command_monotonic, now_ros)
    assert gate._freshness.measured(now_ros, gate._command_monotonic, now_ros)
    try:
        assert gate._depth_policy == "required"
        assert gate._bounded_depth_topic in {item.topic_name for item in gate._subscriptions}
        gate._on_timer()
        assert emitted[-1] == (Velocity(0.0, 0.0), "depth_freshness_unqualified")
    finally:
        gate.destroy_node()
        rclpy.shutdown()


def test_depth_ingress_uses_only_bounded_bridge_types_and_embeds_captures():
    rclpy = pytest.importorskip("rclpy")
    from rclpy.parameter import Parameter
    from senior_motion_msgs.msg import BoundedDepthEvidence, BoundedRawCameraInfo, BoundedRawImage
    from senior_robot.depth_ingress import DepthIngress

    rclpy.init()
    node = DepthIngress(parameter_overrides=[
        Parameter("min_range_m", value=0.1), Parameter("max_range_m", value=8.0),
        Parameter("range_error_m", value=0.01), Parameter("max_age_s", value=0.5),
    ])
    try:
        subscriptions = {item.topic_name: item.msg_type for item in node._subscriptions}
        assert subscriptions["/safety/raw/depth"] is BoundedRawImage
        assert subscriptions["/safety/raw/depth_info"] is BoundedRawCameraInfo
        evidence = BoundedDepthEvidence()
        assert evidence.depth_capture is not None
        assert evidence.camera_info_capture is not None
    finally:
        node.destroy_node()
        rclpy.shutdown()


@pytest.fixture
def native_bridge_process():
    import subprocess
    from pathlib import Path

    pytest.importorskip("rclpy")
    from ament_index_python.packages import get_package_prefix

    executable = Path(get_package_prefix("senior_sensor_ingress")) / "lib" / (
        "senior_sensor_ingress/raw_receipt_node"
    )
    bridge = subprocess.Popen(
        [str(executable)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    )
    try:
        yield bridge
    finally:
        bridge.terminate()
        try:
            bridge.wait(timeout=3)
        except subprocess.TimeoutExpired:
            bridge.kill()
            bridge.wait(timeout=3)


def test_native_bridge_depth_ingress_gate_end_to_end_and_raw_failure(native_bridge_process):
    import time

    rclpy = pytest.importorskip("rclpy")
    from geometry_msgs.msg import TransformStamped
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from senior_robot.depth_ingress import DepthIngress
    from senior_robot.safety_node import SafetyGate
    from sensor_msgs.msg import CameraInfo, Image

    bridge = native_bridge_process
    rclpy.init()
    ingress = DepthIngress(parameter_overrides=[
        Parameter("min_range_m", value=0.1), Parameter("max_range_m", value=8.0),
        Parameter("range_error_m", value=0.01), Parameter("row_stride", value=1),
        Parameter("column_stride", value=1), Parameter("max_age_s", value=1.0),
        Parameter("transform_static", value=True),
    ])
    gate = SafetyGate(parameter_overrides=[
        Parameter("observation_max_age", value=1.0),
        Parameter("measurement_max_age", value=1.0),
        Parameter("depth_policy", value="required"),
    ])
    fixture = rclpy.create_node("depth_native_e2e_fixture")
    depth_pub = fixture.create_publisher(Image, "/camera/depth/image_raw", qos_profile_sensor_data)
    info_pub = fixture.create_publisher(CameraInfo, "/camera/depth/camera_info",
                                        qos_profile_sensor_data)
    tf = TransformStamped()
    tf.header.frame_id, tf.child_frame_id = "base_link", "camera_optical_frame"
    tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = (
        0.2316, 0.00224, 0.1282
    )
    tf.transform.rotation.x, tf.transform.rotation.y = -0.5, 0.5
    tf.transform.rotation.z, tf.transform.rotation.w = -0.5, 0.5
    ingress._tf_buffer.set_transform_static(tf, "fixture")
    executor = SingleThreadedExecutor()
    for node in (fixture, ingress, gate):
        executor.add_node(node)
    try:
        deadline = time.monotonic() + 8
        last = None
        while time.monotonic() < deadline and gate._depth_admission.latest is None:
            assert bridge.poll() is None
            stamp = fixture.get_clock().now().to_msg()
            info = CameraInfo()
            info.header.stamp, info.header.frame_id = stamp, "camera_optical_frame"
            info.width = info.height = 3
            info.distortion_model, info.d = "plumb_bob", [0.0] * 5
            info.k = [2.0, 0.0, 1.0, 0.0, 4.0, 1.0, 0.0, 0.0, 1.0]
            info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            info.p = [2.0, 0.0, 1.0, 0.0, 0.0, 4.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0]
            last = Image()
            last.header.stamp, last.header.frame_id = stamp, "camera_optical_frame"
            last.width = last.height = 3
            last.encoding, last.step = "32FC1", 12
            last.data = struct.pack("<9f", *([0.4] * 9))
            info_pub.publish(info)
            depth_pub.publish(last)
            for _ in range(8):
                executor.spin_once(timeout_sec=0.01)
        assert last is not None and gate._depth_admission.latest is not None
        assert gate._depth_usable(gate.get_clock().now().nanoseconds, time.monotonic()) is not None
        last.header.stamp = fixture.get_clock().now().to_msg()
        last.encoding = "unsupported"
        depth_pub.publish(last)
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline and gate._depth_admission.latest is not None:
            executor.spin_once(timeout_sec=0.02)
        assert gate._depth_admission.latest is None
    finally:
        executor.shutdown()
        for node in (fixture, ingress, gate):
            node.destroy_node()
        rclpy.shutdown()
