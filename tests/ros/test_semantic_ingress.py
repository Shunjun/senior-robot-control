"""Bounded semantic ROS wiring; detector fixture does not claim model quality."""

import time

import pytest


def fixture_detector(frame):
    return [("person", 0.9, (0.0, 0.0, 2.0, 2.0))]


def test_semantic_node_reports_missing_weights_disabled():
    rclpy = pytest.importorskip("rclpy")
    from senior_robot.semantic_ingress import SemanticIngress

    rclpy.init()
    node = SemanticIngress()
    from types import SimpleNamespace

    captured = []
    node._publisher = SimpleNamespace(publish=captured.append)
    try:
        node._on_timer()
        assert any(not item.valid and item.reason == "weights_not_configured" for item in captured)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_real_ros_rgb_depth_inference_returns_original_typed_frame(tmp_path):
    rclpy = pytest.importorskip("rclpy")
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from senior_motion_msgs.msg import (
        BoundedDepthEvidence,
        BoundedRawCameraInfo,
        BoundedRawImage,
        BoundedSemanticEvidence,
        DepthSample,
        FencedDepthEvidence,
    )
    from senior_robot.semantic_ingress import SemanticIngress

    from senior_control.vision.spawn_detector import SpawnInferenceWorker

    rclpy.init()
    weights = tmp_path / "fixture.pt"
    weights.write_bytes(b"fixture")
    worker = SpawnInferenceWorker(weights_path=weights, processor=fixture_detector, timeout_s=2)
    node = SemanticIngress(
        parameter_overrides=[
            Parameter("registered_profile", value=True),
            Parameter("max_age_s", value=0.3),
        ],
        _worker=worker,
    )
    fixture = rclpy.create_node("semantic_fixture")
    rgb_pub = fixture.create_publisher(BoundedRawImage, "/safety/raw/rgb", qos_profile_sensor_data)
    info_pub = fixture.create_publisher(
        BoundedRawCameraInfo, "/safety/raw/rgb_info", qos_profile_sensor_data
    )
    raw_depth_pub = fixture.create_publisher(
        BoundedRawImage, "/safety/raw/depth", qos_profile_sensor_data
    )
    raw_info_pub = fixture.create_publisher(
        BoundedRawCameraInfo, "/safety/raw/depth_info", qos_profile_sensor_data
    )
    assert raw_depth_pub is not None and raw_info_pub is not None
    depth_pub = fixture.create_publisher(
        FencedDepthEvidence, "/safety/bounded_depth", qos_profile_sensor_data
    )
    received = []
    fixture.create_subscription(
        BoundedSemanticEvidence,
        "/safety/semantic_candidate_unfenced",
        received.append,
        qos_profile_sensor_data,
    )
    executor = SingleThreadedExecutor()
    executor.add_node(fixture)
    executor.add_node(node)
    seq = 0

    def publish_pair(rgb_info_generation=1):
        nonlocal seq
        seq += 1
        stamp = fixture.get_clock().now().to_msg()
        info = BoundedRawCameraInfo()
        info.capture.stamp, info.capture.frame_id = stamp, "optical"
        info.width = info.height = 3
        info.distortion_model, info.d = "plumb_bob", [0.0] * 5
        info.k = [2.0, 0.0, 1.0, 0.0, 4.0, 1.0, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [2.0, 0.0, 1.0, 0.0, 0.0, 4.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        image = BoundedRawImage()
        image.capture.stamp, image.capture.frame_id = stamp, "optical"
        image.width = image.height = 3
        image.encoding, image.step, image.data = "rgb8", 10, bytes(30)
        depth = BoundedDepthEvidence()
        depth.stamp = depth.camera_info_stamp = depth.received_ros = (
            depth.camera_info_received_ros
        ) = stamp
        depth.received_steady = depth.camera_info_received_steady = time.monotonic()
        depth.valid, depth.transform_static, depth.sequence = True, True, seq
        depth.frame_id, depth.body_frame = "optical", "base_link"
        depth.raw_source, depth.camera_info_source = "depth-gid", "depth-info-gid"
        depth.source_epoch, depth.calibration_id = "depth-epoch", "a" * 64
        depth.width = depth.height = 3
        depth.row_stride = depth.column_stride = 1
        depth.fx, depth.fy, depth.cx, depth.cy = 2.0, 4.0, 1.0, 1.0
        depth.min_range_m, depth.max_range_m, depth.range_error_m = 0.1, 8.0, 0.01
        depth.sensor_origin = [0.0] * 3
        depth.optical_to_body_quaternion = [0.0, 0.0, 0.0, 1.0]
        sample = DepthSample()
        sample.u = sample.v = 1
        sample.depth_m = sample.z = 1.0
        depth.samples = [sample]
        for capture, topic, raw in (
            (image.capture, "/safety/raw/rgb", "01" * 24),
            (info.capture, "/safety/raw/rgb_info", "02" * 24),
            (depth.depth_capture, "/safety/raw/depth", "03" * 24),
            (depth.camera_info_capture, "/safety/raw/depth_info", "04" * 24),
        ):
            endpoints = fixture.get_publishers_info_by_topic(topic)
            if len(endpoints) != 1:
                return depth
            capture.stamp, capture.received_ros = stamp, stamp
            capture.received_steady, capture.frame_id = depth.received_steady, "optical"
            capture.bridge_gid, capture.raw_gid = bytes(endpoints[0].endpoint_gid).hex(), raw
            capture.source_epoch, capture.bridge_boot_id = raw, "bridge-boot"
            capture.capture_generation = rgb_info_generation if topic.endswith("rgb_info") else 1
            capture.sequence, capture.valid = seq, True
        depth.raw_source, depth.camera_info_source = "03" * 24, "04" * 24
        endpoints = fixture.get_publishers_info_by_topic("/safety/bounded_depth")
        if len(endpoints) != 1:
            return depth
        envelope = FencedDepthEvidence()
        envelope.body = depth
        envelope.owner.relay_gid = bytes(endpoints[0].endpoint_gid).hex()
        envelope.owner.relay_boot, envelope.owner.relay_sequence = "relay-boot", seq
        envelope.owner.owner_gid, envelope.owner.owner_epoch = "05" * 24, depth.source_epoch
        envelope.owner.owner_generation, envelope.owner.owner_sequence = 1, seq
        envelope.owner.valid = True
        info_pub.publish(info)
        depth_pub.publish(envelope)
        rgb_pub.publish(image)
        return depth

    try:
        end, next_send = time.monotonic() + 7.0, 0.0
        while time.monotonic() < end and not any(item.valid for item in received):
            if time.monotonic() >= next_send:
                publish_pair()
                next_send = time.monotonic() + 0.08
            executor.spin_once(timeout_sec=0.01)
        result = next(item for item in received if item.valid)
        assert result.rgb_stamp == result.rgb_camera_info_stamp == result.depth.stamp
        assert result.depth.camera_info_stamp == result.rgb_stamp
        assert result.depth.source_epoch == "depth-epoch"
        assert result.rgb_source and result.rgb_camera_info_source
        captures = (
            result.rgb_capture,
            result.rgb_camera_info_capture,
            result.depth.depth_capture,
            result.depth.camera_info_capture,
        )
        assert [capture.raw_gid for capture in captures] == [f"{i:02x}" * 24 for i in range(1, 5)]
        assert all(capture.bridge_boot_id == "bridge-boot" for capture in captures)
        assert all(capture.capture_generation == 1 for capture in captures)
        assert all(capture.stamp == result.rgb_stamp for capture in captures)
        assert result.rgb_received_steady == result.rgb_capture.received_steady
        assert result.depth.received_steady == result.depth.depth_capture.received_steady
        assert len(result.registration_id) == 64
        assert len(result.objects) == len(result.associations) == 1
        assert result.associations[0].sample_index == 0
        assert result.associations[0].object_mask == 1
        assert result.depth.samples[0].z == pytest.approx(1.0)
        prior_epoch = result.source_epoch
        mixed = publish_pair(rgb_info_generation=2)
        end = time.monotonic() + 0.4
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.01)
        assert not any(item.valid and item.rgb_stamp == mixed.stamp for item in received)
        assert any(
            not item.valid and item.reason == "raw_capture_generation_changed" for item in received
        )
        invalid = FencedDepthEvidence()
        invalid.owner.relay_gid = bytes(
            fixture.get_publishers_info_by_topic("/safety/bounded_depth")[0].endpoint_gid
        ).hex()
        invalid.owner.relay_boot, invalid.owner.relay_sequence = "relay-boot", seq + 1
        invalid.owner.owner_gid, invalid.owner.owner_epoch = "05" * 24, "depth-epoch"
        invalid.owner.owner_generation, invalid.owner.owner_sequence = 1, seq + 1
        invalid.owner.valid, invalid.owner.reason = False, "depth_failure"
        depth_pub.publish(invalid)
        end = time.monotonic() + 2.0
        while time.monotonic() < end and not any(
            not item.valid and item.source_epoch != prior_epoch and item.reason == "depth_invalid"
            for item in received
        ):
            executor.spin_once(timeout_sec=0.01)
        assert any(
            not item.valid and item.source_epoch != prior_epoch and item.reason == "depth_invalid"
            for item in received
        )
    finally:
        executor.shutdown()
        node.destroy_node()
        fixture.destroy_node()
        rclpy.shutdown()


def test_timer_accepts_advancing_clock_and_rejects_real_rollback(monkeypatch):
    rclpy = pytest.importorskip("rclpy")
    from types import SimpleNamespace

    from senior_robot.semantic_ingress import SemanticIngress

    rclpy.init()
    node = SemanticIngress()
    now = [1_000_000_000]
    resets = []
    drains = []

    def read_clock():
        now[0] += 1
        return SimpleNamespace(nanoseconds=now[0])

    try:
        node._coordinator.close()
        node._coordinator = SimpleNamespace(
            status=("ready", ""), take_result=lambda **_: None, close=lambda: None
        )
        node._registered = True
        monkeypatch.setattr(node, "get_clock", lambda: SimpleNamespace(now=read_clock))
        monkeypatch.setattr(node, "_check_sources", lambda: True)
        monkeypatch.setattr(node, "_reset", lambda reason, **_: resets.append(reason))
        monkeypatch.setattr(node, "_drain", lambda: drains.append(True))
        node._on_timer()
        assert drains == [True]
        assert not resets
        now[0] = 1
        node._on_timer()
        assert resets == ["clock_rollback"]
        assert drains == [True]
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_semantic_ingress_uses_bounded_bridge_and_embeds_four_capture_headers():
    rclpy = pytest.importorskip("rclpy")
    from senior_motion_msgs.msg import (
        BoundedRawCameraInfo,
        BoundedRawImage,
        BoundedSemanticEvidence,
    )
    from senior_robot.semantic_ingress import SemanticIngress

    rclpy.init()
    node = SemanticIngress()
    try:
        subscriptions = {item.topic_name: item.msg_type for item in node._subscriptions}
        assert subscriptions["/safety/raw/rgb"] is BoundedRawImage
        assert subscriptions["/safety/raw/rgb_info"] is BoundedRawCameraInfo
        result = BoundedSemanticEvidence()
        assert result.rgb_capture is not None and result.rgb_camera_info_capture is not None
        assert result.depth.depth_capture is not None
        assert result.depth.camera_info_capture is not None
    finally:
        node.destroy_node()
        rclpy.shutdown()


@pytest.fixture
def semantic_native_bridge():
    import subprocess
    from pathlib import Path

    pytest.importorskip("rclpy")
    from ament_index_python.packages import get_package_prefix

    executable = (
        Path(get_package_prefix("senior_sensor_ingress"))
        / "lib"
        / ("senior_sensor_ingress/raw_receipt_node")
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


def test_native_bridge_semantic_egress_returns_fenced_original_frames(
    semantic_native_bridge, tmp_path
):
    import struct

    rclpy = pytest.importorskip("rclpy")
    from geometry_msgs.msg import TransformStamped
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from senior_motion_msgs.msg import (
        BoundedSemanticEvidence,
        FencedDepthEvidence,
        FencedSemanticEvidence,
    )
    from senior_robot.depth_ingress import DepthIngress
    from senior_robot.semantic_ingress import SemanticIngress
    from sensor_msgs.msg import CameraInfo, Image

    from senior_control.vision.spawn_detector import SpawnInferenceWorker

    rclpy.init()
    weights = tmp_path / "fixture.pt"
    weights.write_bytes(b"fixture")
    worker = SpawnInferenceWorker(weights_path=weights, processor=fixture_detector, timeout_s=2)
    semantic = SemanticIngress(
        parameter_overrides=[
            Parameter("registered_profile", value=True),
            Parameter("max_age_s", value=0.3),
        ],
        _worker=worker,
    )
    depth = DepthIngress(
        parameter_overrides=[
            Parameter("min_range_m", value=0.1),
            Parameter("max_range_m", value=8.0),
            Parameter("range_error_m", value=0.01),
            Parameter("row_stride", value=1),
            Parameter("column_stride", value=1),
            Parameter("max_age_s", value=0.3),
            Parameter("transform_static", value=True),
        ]
    )
    fixture = rclpy.create_node("semantic_native_e2e_fixture")
    rgb_pub = fixture.create_publisher(Image, "/camera/color/image_raw", qos_profile_sensor_data)
    depth_pub = fixture.create_publisher(Image, "/camera/depth/image_raw", qos_profile_sensor_data)
    rgb_info_pub = fixture.create_publisher(
        CameraInfo, "/camera/color/camera_info", qos_profile_sensor_data
    )
    depth_info_pub = fixture.create_publisher(
        CameraInfo, "/camera/depth/camera_info", qos_profile_sensor_data
    )
    received = []
    depth_seen = []
    candidate_seen = []
    fixture.create_subscription(
        FencedDepthEvidence, "/safety/bounded_depth", depth_seen.append, qos_profile_sensor_data
    )
    fixture.create_subscription(
        BoundedSemanticEvidence,
        "/safety/semantic_candidate_unfenced",
        candidate_seen.append,
        qos_profile_sensor_data,
    )
    fixture.create_subscription(
        FencedSemanticEvidence,
        "/safety/semantic_candidates",
        received.append,
        qos_profile_sensor_data,
    )
    tf = TransformStamped()
    tf.header.frame_id, tf.child_frame_id = "base_link", "optical"
    tf.transform.rotation.w = 1.0
    depth._tf_buffer.set_transform_static(tf, "fixture")
    executor = SingleThreadedExecutor()
    for node in (fixture, depth, semantic):
        executor.add_node(node)
    try:
        end, next_send = time.monotonic() + 9, 0.0
        while time.monotonic() < end and not any(item.owner.valid for item in received):
            assert semantic_native_bridge.poll() is None
            if time.monotonic() >= next_send:
                stamp = fixture.get_clock().now().to_msg()
                info = CameraInfo()
                info.header.stamp, info.header.frame_id = stamp, "optical"
                info.width = info.height = 3
                info.distortion_model, info.d = "plumb_bob", [0.0] * 5
                info.k = [2.0, 0.0, 1.0, 0.0, 4.0, 1.0, 0.0, 0.0, 1.0]
                info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
                info.p = [2.0, 0.0, 1.0, 0.0, 0.0, 4.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0]
                image = Image()
                image.header.stamp, image.header.frame_id = stamp, "optical"
                image.width = image.height = 3
                image.encoding, image.step, image.data = "rgb8", 9, bytes(27)
                depth_image = Image()
                depth_image.header.stamp, depth_image.header.frame_id = stamp, "optical"
                depth_image.width = depth_image.height = 3
                depth_image.encoding, depth_image.step = "32FC1", 12
                depth_image.data = struct.pack("<9f", *([1.0] * 9))
                rgb_info_pub.publish(info)
                depth_info_pub.publish(info)
                rgb_pub.publish(image)
                depth_pub.publish(depth_image)
                next_send = time.monotonic() + 0.08
            executor.spin_once(timeout_sec=0.01)
        valid = [item for item in received if item.owner.valid]
        depth_error = ""
        if depth_seen:
            from senior_control.vision.semantic_runtime import depth_snapshot

            try:
                depth_snapshot(depth_seen[-1].body)
            except ValueError as error:
                depth_error = str(error)
        diagnostics = {
            "depth_decode_error": depth_error,
            "reasons": [item.owner.reason for item in received[-20:]],
            "sources": semantic._sources,
            "worker": semantic._coordinator.status,
            "sync_sizes": (
                len(semantic._sync._rgb),
                len(semantic._sync._info),
                len(semantic._sync._depth),
            ),
            "depth_epoch": semantic._depth_epoch,
            "bridge_alive": semantic_native_bridge.poll() is None,
            "depth_status": [(item.owner.valid, item.owner.reason) for item in depth_seen[-10:]],
            "candidate_status": [(item.valid, item.reason) for item in candidate_seen[-10:]],
        }
        if not valid:
            print(diagnostics)
        assert valid, repr(diagnostics)
        result = valid[-1]
        assert result.body.depth_owner.valid
        assert result.body.rgb_capture.stamp == result.body.depth.depth_capture.stamp
        assert (
            result.body.rgb_capture.bridge_boot_id == result.body.depth.depth_capture.bridge_boot_id
        )
        assert len(result.body.objects) == 1
        previous_sequence = result.owner.relay_sequence
        depth_image.header.stamp = fixture.get_clock().now().to_msg()
        depth_image.encoding = "unsupported"
        depth_pub.publish(depth_image)
        end = time.monotonic() + 3
        while time.monotonic() < end and not any(
            not item.owner.valid and item.owner.relay_sequence > previous_sequence
            for item in received
        ):
            executor.spin_once(timeout_sec=0.01)
        assert any(
            not item.owner.valid and item.owner.relay_sequence > previous_sequence
            for item in received
        )
    finally:
        executor.shutdown()
        for node in (fixture, depth, semantic):
            node.destroy_node()
        rclpy.shutdown()
