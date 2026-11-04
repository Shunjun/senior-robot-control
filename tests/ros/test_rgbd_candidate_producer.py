"""Real DDS lifecycle tests for the isolated RGB-D diagnostic producer."""

import threading
import time

import pytest

rclpy = pytest.importorskip("rclpy")
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from senior_motion_msgs.msg import RgbdMotionCandidate  # noqa: E402
from senior_robot.rgbd_candidate_producer import RgbdCandidateProducer  # noqa: E402


def test_disabled_static_domain_emits_unqualified_invalid_and_closes():
    rclpy.init()
    producer = RgbdCandidateProducer()
    observer = rclpy.create_node("rgbd_producer_test_observer")
    received = []
    sub = observer.create_subscription(
        RgbdMotionCandidate, "/rgbd_diagnostic/candidate", received.append, qos_profile_sensor_data
    )
    assert sub is not None
    executor = MultiThreadedExecutor()
    executor.add_node(producer)
    executor.add_node(observer)
    thread = threading.Thread(target=executor.spin)
    thread.start()
    try:
        end = time.monotonic() + 5
        while time.monotonic() < end and not received:
            time.sleep(0.01)
        assert received
        assert not received[-1].valid
        assert not received[-1].hard_error_qualified
        assert received[-1].reason == "static_domain_unavailable"
    finally:
        executor.shutdown()
        thread.join(timeout=5)
        producer.destroy_node()
        observer.destroy_node()
        rclpy.shutdown()
        assert not thread.is_alive()


def test_real_four_streams_and_tf_produce_nonzero_unqualified_candidate():
    import os
    import subprocess

    import numpy as np
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from senior_motion_msgs.msg import (
        BoundedRawCameraInfo,
        BoundedRawImage,
        BoundedStaticTf,
        FencedRgbdCandidate,
        StaticTransformEdge,
    )

    rclpy.init(
        args=["--ros-args", "-p", "static_background_declared:=true", "-p", "camera_frame:=optical"]
    )
    producer = RgbdCandidateProducer()
    raw = rclpy.create_node("raw_receipt")
    tf = rclpy.create_node("static_tf_receipt")
    observer = rclpy.create_node("rgbd_live_observer")
    topics = ("rgb", "depth", "rgb_info", "depth_info")
    pubs = [
        raw.create_publisher(
            BoundedRawImage if i < 2 else BoundedRawCameraInfo,
            "/rgbd_diagnostic/" + topic,
            qos_profile_sensor_data,
        )
        for i, topic in enumerate(topics)
    ]
    tf_pub = tf.create_publisher(
        BoundedStaticTf,
        "/rgbd_diagnostic/tf",
        QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        ),
    )
    received = []
    fenced = []
    sub = observer.create_subscription(
        RgbdMotionCandidate, "/rgbd_diagnostic/candidate", received.append, qos_profile_sensor_data
    )
    fence_sub = observer.create_subscription(
        FencedRgbdCandidate, "/rgbd_diagnostic/fenced", fenced.append, qos_profile_sensor_data
    )
    assert sub is not None and fence_sub is not None
    executable = os.environ.get("RGBD_RECEIPT_EXECUTABLE")
    native = (
        subprocess.Popen([executable, "--ros-args", "-p", "camera_frame:=optical"])
        if executable
        else None
    )
    executor = MultiThreadedExecutor()
    for node in (producer, raw, tf, observer):
        executor.add_node(node)
    thread = threading.Thread(target=executor.spin)
    thread.start()
    try:
        end = time.monotonic() + 5
        while time.monotonic() < end and not all(
            p.get_subscription_count() for p in [*pubs, tf_pub]
        ):
            time.sleep(0.01)
        assert all(p.get_subscription_count() for p in [*pubs, tf_pub])
        time.sleep(0.15)

        def publisher_gid(node, topic):
            owners = node.get_publishers_info_by_topic(topic)
            assert len(owners) == 1
            return bytes(owners[0].endpoint_gid).hex()

        status = BoundedStaticTf()
        status.valid = status.admitted = True
        status.bridge_boot_id = "tfboot"
        status.bridge_gid = publisher_gid(observer, "/rgbd_diagnostic/tf")
        status.raw_gid = "01"
        status.generation = status.sequence = status.first_sequence = 1
        status.received_ros = status.first_received_ros = status.admitted_from_ros = (
            observer.get_clock().now().to_msg()
        )
        status.received_steady = status.first_received_steady = status.admitted_from_steady = (
            time.monotonic()
        )
        edge = StaticTransformEdge()
        edge.parent_frame = "base_link"
        edge.child_frame = "optical"
        edge.rotation_xyzw = [0.0, 0.0, 0.0, 1.0]
        status.edges = [edge]
        from senior_control.vision.static_tf_binding import StaticBinding
        from senior_control.vision.static_tf_wire import observe_verified_tf

        probe = StaticBinding("base_link", "optical")
        assert observe_verified_tf(probe, status, current_relay_gid=status.bridge_gid)
        tf_pub.publish(status)
        time.sleep(0.8)
        assert producer._binding.current is not None, (producer._enabled, producer._binding.camera)
        import copy

        prior = producer._binding.current
        stale = copy.deepcopy(status)
        stale.bridge_gid = "ff"
        before_stale = len(received)
        producer._enqueue((4, producer._tf_epoch, stale))
        time.sleep(0.1)
        assert producer._binding.current == prior
        assert not any(x.reason == "static_tf_unavailable" for x in received[before_stale:])

        for sequence in range(1, 51):
            rng = np.random.default_rng(7)
            mono = np.zeros((240, 320), np.uint8)
            depth = np.zeros((240, 320), np.float32)
            shift = sequence % 2
            for row in range(3):
                for col in range(4):
                    x, y = 42 + col * 70 + shift, 42 + row * 70
                    mono[y - 22 : y + 23, x - 22 : x + 23] = rng.integers(
                        0, 256, (45, 45), dtype=np.uint8
                    )
                    depth[y - 22 : y + 23, x - 22 : x + 23] = 2 + (row + col) % 3
            rgb = np.repeat(mono[:, :, None], 3, axis=2)
            stamp = observer.get_clock().now().to_msg()
            steady = time.monotonic()
            for index, pub in enumerate(pubs):
                value = BoundedRawImage() if index < 2 else BoundedRawCameraInfo()
                h = value.capture
                h.valid = True
                h.bridge_boot_id = "rawboot"
                h.bridge_gid = publisher_gid(observer, "/rgbd_diagnostic/" + topics[index])
                h.raw_gid = "10"
                h.source_epoch = topics[index]
                h.capture_generation = 1
                h.sequence = sequence
                h.frame_id = "optical"
                h.stamp = h.received_ros = stamp
                h.received_steady = steady
                value.width, value.height = 320, 240
                if index == 0:
                    value.step, value.encoding, value.data = 960, "rgb8", rgb.tobytes()
                elif index == 1:
                    value.step, value.encoding, value.data = (
                        1280,
                        "32FC1",
                        depth.astype("<f4").tobytes(),
                    )
                else:
                    value.distortion_model = "plumb_bob"
                    value.d = [0.0] * 5
                    value.k = [250.0, 0.0, 160.0, 0.0, 250.0, 120.0, 0.0, 0.0, 1.0]
                    value.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
                    value.p = [250.0, 0.0, 160.0, 0.0, 0.0, 250.0, 120.0, 0.0, 0.0, 0.0, 1.0, 0.0]
                pub.publish(value)
            time.sleep(0.1)
            candidate_ready = any(x.valid and abs(x.camera_transform[3]) > 1e-5 for x in received)
            fence_ready = native is None or any(
                x.owner.valid and abs(x.body.camera_transform[3]) > 1e-5 for x in fenced
            )
            if candidate_ready and fence_ready:
                break
        assert any(
            x.valid and not x.hard_error_qualified and abs(x.camera_transform[3]) > 1e-5
            for x in received
        ), [(x.reason, x.valid) for x in received[-10:]]
        if native is not None:
            assert any(
                x.owner.valid
                and not x.body.hard_error_qualified
                and abs(x.body.camera_transform[3]) > 1e-5
                for x in fenced
            ), [(x.owner.reason, x.owner.valid) for x in fenced[-10:]]
    finally:
        if native is not None:
            native.terminate()
            native.wait(timeout=5)
        executor.shutdown()
        thread.join(timeout=5)
        for node in (producer, raw, tf, observer):
            node.destroy_node()
        rclpy.shutdown()
        assert not thread.is_alive()


def test_remapped_input_graph_uses_resolved_topic():
    from senior_motion_msgs.msg import BoundedRawImage

    rclpy.init(args=["--ros-args", "-r", "/rgbd_diagnostic/rgb:=/safety/raw/rgb"])
    producer = RgbdCandidateProducer()
    raw = rclpy.create_node("raw_receipt")
    publisher = raw.create_publisher(BoundedRawImage, "/safety/raw/rgb", qos_profile_sensor_data)
    try:
        end = time.monotonic() + 5
        while time.monotonic() < end and not publisher.get_subscription_count():
            time.sleep(0.01)
        assert publisher.get_subscription_count()
        assert producer._owner(producer._topics[0], "raw_receipt")
    finally:
        producer.destroy_node()
        raw.destroy_node()
        rclpy.shutdown()


def test_native_upstream_and_fence_emit_nonzero_candidate():
    import os
    import signal
    import subprocess

    import numpy as np
    from geometry_msgs.msg import TransformStamped
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from senior_motion_msgs.msg import BoundedStaticTf, FencedRgbdCandidate
    from sensor_msgs.msg import CameraInfo, Image
    from tf2_msgs.msg import TFMessage

    root = os.environ.get("RGBD_NATIVE_BUILD")
    if not root:
        pytest.skip("native build path not supplied")
    rclpy.init(
        args=["--ros-args", "-p", "static_background_declared:=true", "-p", "camera_frame:=optical"]
    )
    via_launch = os.environ.get("RGBD_TEST_VIA_LAUNCH") == "1"
    producer = None if via_launch else RgbdCandidateProducer()
    camera = rclpy.create_node("synthetic_rgbd_camera")
    rsp = rclpy.create_node("robot_state_publisher")
    observer = rclpy.create_node("rgbd_upstream_observer")
    input_topics = (
        "/camera/color/image_raw",
        "/camera/depth/image_raw",
        "/camera/color/camera_info",
        "/camera/depth/camera_info",
    )
    pubs = [
        camera.create_publisher(Image if i < 2 else CameraInfo, topic, qos_profile_sensor_data)
        for i, topic in enumerate(input_topics)
    ]
    tf_pub = rsp.create_publisher(
        TFMessage,
        "/tf_static",
        QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        ),
    )
    fenced = []
    candidate = []
    static_status = []
    static_sub = observer.create_subscription(
        BoundedStaticTf,
        "/rgbd_diagnostic/tf",
        static_status.append,
        QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        ),
    )
    assert static_sub is not None
    sub = observer.create_subscription(
        FencedRgbdCandidate, "/rgbd_diagnostic/fenced", fenced.append, qos_profile_sensor_data
    )
    proposed = observer.create_subscription(
        RgbdMotionCandidate, "/rgbd_diagnostic/candidate", candidate.append, qos_profile_sensor_data
    )
    assert sub is not None and proposed is not None
    opts = ["--ros-args", "-p", "camera_frame:=optical"]
    raw_opts = ["--ros-args"]
    for name, stream in zip(
        ("rgb_output", "depth_output", "rgb_info_output", "depth_info_output"),
        ("rgb", "depth", "rgb_info", "depth_info"),
        strict=True,
    ):
        prefix = "/safety/raw" if via_launch else "/rgbd_diagnostic"
        raw_opts.extend(["-p", f"{name}:={prefix}/{stream}"])
    children = [subprocess.Popen([root + "/raw_receipt_node", *raw_opts])]
    if via_launch:
        children.append(
            subprocess.Popen(
                [
                    "ros2",
                    "launch",
                    "senior_robot",
                    "rgbd_diagnostic.launch.py",
                    "use_sim_time:=false",
                    "camera_frame:=optical",
                    "static_background_declared:=true",
                ],
                start_new_session=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
            )
        )
    else:
        children.extend(
            [
                subprocess.Popen(
                    [
                        root + "/static_tf_receipt_node",
                        *opts,
                        "-p",
                        "output_topic:=/rgbd_diagnostic/tf",
                    ]
                ),
                subprocess.Popen([root + "/rgbd_candidate_receipt_node", *opts]),
            ]
        )
    executor = MultiThreadedExecutor()
    nodes = tuple(node for node in (producer, camera, rsp, observer) if node is not None)
    for node in nodes:
        executor.add_node(node)
    thread = threading.Thread(target=executor.spin)
    thread.start()
    try:
        end = time.monotonic() + 8
        while time.monotonic() < end and not all(
            p.get_subscription_count() for p in [*pubs, tf_pub]
        ):
            time.sleep(0.01)
        assert all(p.get_subscription_count() for p in [*pubs, tf_pub])
        time.sleep(0.2)
        transform = TransformStamped()
        transform.header.frame_id = "base_link"
        transform.child_frame_id = "optical"
        transform.transform.rotation.w = 1.0
        tf_pub.publish(TFMessage(transforms=[transform]))
        end = time.monotonic() + 5
        while time.monotonic() < end and not any(x.valid and x.admitted for x in static_status):
            time.sleep(0.01)
        assert any(x.valid and x.admitted for x in static_status)
        time.sleep(0.3)
        target_samples = int(os.environ.get("RGBD_TEST_SAMPLE_COUNT", "3"))
        exercise_overflow = os.environ.get("RGBD_TEST_OVERFLOW_RECOVERY") == "1"
        assert not exercise_overflow or producer is not None
        recovery_marker = None
        recovery_floor = None
        scene_by_stamp = {}
        for sequence in range(1, max(81, target_samples * 8 + 1)):
            rng = np.random.default_rng(7)
            mono = np.zeros((240, 320), np.uint8)
            depth = np.zeros((240, 320), np.float32)
            for row in range(3):
                for col in range(4):
                    z = 2 + (row + col) % 3
                    # fx=250 px and C2<-C1 x=0.048 m give shifts 6, 4, 3 px.
                    shift = (12 // z) if sequence % 2 else 0
                    x, y = 42 + col * 70 + shift, 42 + row * 70
                    mono[y - 22 : y + 23, x - 22 : x + 23] = rng.integers(
                        0, 256, (45, 45), dtype=np.uint8
                    )
                    depth[y - 22 : y + 23, x - 22 : x + 23] = 2 + (row + col) % 3
            rgb = np.repeat(mono[:, :, None], 3, axis=2)
            stamp = observer.get_clock().now().to_msg()
            scene_by_stamp[(stamp.sec, stamp.nanosec)] = sequence % 2
            for index, pub in enumerate(pubs):
                value = Image() if index < 2 else CameraInfo()
                value.header.stamp = stamp
                value.header.frame_id = "optical"
                value.width, value.height = 320, 240
                if index == 0:
                    value.step, value.encoding, value.data = 960, "rgb8", rgb.tobytes()
                elif index == 1:
                    value.step, value.encoding, value.data = (
                        1280,
                        "32FC1",
                        depth.astype("<f4").tobytes(),
                    )
                else:
                    value.distortion_model = "plumb_bob"
                    value.d = [0.0] * 5
                    value.k = [250.0, 0.0, 160.0, 0.0, 250.0, 120.0, 0.0, 0.0, 1.0]
                    value.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
                    value.p = [250.0, 0.0, 160.0, 0.0, 0.0, 250.0, 120.0, 0.0, 0.0, 0.0, 1.0, 0.0]
                pub.publish(value)
            time.sleep(0.1)
            live_count = sum(
                x.owner.valid and abs(x.body.camera_transform[3]) > 1e-5 for x in fenced
            )
            if live_count >= target_samples and exercise_overflow and recovery_marker is None:
                old_engine = producer._engine
                old_epoch = producer._tf_epoch
                old_joiner = old_engine.joiner
                old_ledger = old_engine.coordinator.ledger
                old_worker_generation = old_engine.coordinator.worker.generation
                old_worker_sequence = old_engine.coordinator.worker.sequence_frontier
                assert old_worker_sequence > 0
                producer._inbox.revoke("overflow")
                producer._wake.set()
                deadline = time.monotonic() + 5
                while time.monotonic() < deadline:
                    if (
                        producer._tf_epoch > old_epoch
                        and producer._engine is not None
                        and producer._engine is not old_engine
                    ):
                        break
                    time.sleep(0.01)
                assert producer._engine is not None and producer._engine is not old_engine
                assert producer._engine.joiner is old_joiner
                assert producer._engine.coordinator.ledger is old_ledger
                assert producer._engine.coordinator.worker.generation > old_worker_generation
                assert producer._engine.coordinator.worker.sequence_frontier >= old_worker_sequence
                recovery_floor = producer._engine.coordinator.ledger._fresh_after_ros_ns
                recovery_marker = len(fenced)
            elif recovery_marker is not None:
                if (
                    sum(
                        x.owner.valid and abs(x.body.camera_transform[3]) > 1e-5
                        for x in fenced[recovery_marker:]
                    )
                    >= 3
                ):
                    break
            elif live_count >= target_samples:
                break
        assert all(child.poll() is None for child in children)
        valid = [
            x
            for x in fenced
            if x.owner.valid
            and not x.body.hard_error_qualified
            and abs(x.body.camera_transform[3]) > 1e-5
        ]
        if exercise_overflow:
            assert recovery_marker is not None and recovery_floor is not None
            recovered = [x for x in fenced[recovery_marker:] if x.owner.valid]
            assert len(recovered) >= 3
            assert all(
                x.body.first_rgb.stamp.sec * 1_000_000_000 + x.body.first_rgb.stamp.nanosec
                > recovery_floor
                for x in recovered
            )
            assert any(not x.valid and x.reason == "overflow" for x in candidate)
        assert len(valid) >= target_samples, (
            [(x.reason, x.valid) for x in candidate[-10:]],
            [(x.owner.reason, x.owner.valid) for x in fenced[-10:]],
            [(x.reason, x.valid, x.admitted, x.generation) for x in static_status[-10:]],
        )
        for value in valid:
            first_stamp = value.body.first_rgb.stamp
            second_stamp = value.body.second_rgb.stamp
            expected = 0.048 * (
                scene_by_stamp[(second_stamp.sec, second_stamp.nanosec)]
                - scene_by_stamp[(first_stamp.sec, first_stamp.nanosec)]
            )
            assert abs(value.body.camera_transform[3] - expected) < 0.012, (
                value.body.first_rgb.sequence,
                value.body.second_rgb.sequence,
                value.body.camera_transform[3],
                expected,
            )
        print(
            "native fenced diagnostic:",
            len(valid),
            "samples, x=",
            valid[-1].body.camera_transform[3],
            "inliers=",
            valid[-1].body.inliers,
        )
        if not via_launch:
            loss_start = len(fenced)
            proposal_loss_start = len(candidate)
            children[1].terminate()
            children[1].wait(timeout=5)
            end = time.monotonic() + 2
            while time.monotonic() < end:
                revoked = any(not x.owner.valid for x in fenced[loss_start:])
                producer_revoked = any(
                    not x.valid and x.reason == "static_tf_unavailable"
                    for x in candidate[proposal_loss_start:]
                )
                if revoked and producer_revoked:
                    break
                time.sleep(0.01)
            assert revoked
            assert producer_revoked
    finally:
        for child in children:
            if child.poll() is None:
                if via_launch and child is children[1]:
                    os.killpg(child.pid, signal.SIGINT)
                else:
                    child.terminate()
        for child in children:
            if via_launch and child is children[1]:
                launch_output, _ = child.communicate(timeout=10)
            else:
                child.wait(timeout=5)
        executor.shutdown()
        thread.join(timeout=5)
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()
        assert not thread.is_alive()
        if via_launch:
            assert children[1].returncode == 0, launch_output
            assert "Traceback" not in launch_output, launch_output
            assert "Process SpawnProcess" not in launch_output, launch_output
            assert "KeyboardInterrupt" not in launch_output, launch_output
            assert "Exception ignored" not in launch_output, launch_output
