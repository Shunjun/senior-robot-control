"""Generated ROS messages retain numeric values at RGB-D Python boundaries."""

import pytest

pytest.importorskip("rclpy")
import numpy as np  # noqa: E402
from senior_motion_msgs.msg import (  # noqa: E402
    BoundedRawCameraInfo,
    BoundedRawImage,
    BoundedStaticTf,
    StaticTransformEdge,
)
from senior_motion_msgs.msg import (
    RawCaptureHeader as RosHeader,
)

from senior_control.vision.raw_capture import capture_from_message  # noqa: E402
from senior_control.vision.rgbd_frame_builder import build_frame  # noqa: E402
from senior_control.vision.static_tf_binding import StaticBinding  # noqa: E402
from senior_control.vision.static_tf_wire import observe_verified_tf  # noqa: E402


def test_generated_static_tf_float_arrays_are_admitted():
    edge = StaticTransformEdge()
    edge.parent_frame = "base_link"
    edge.child_frame = "camera"
    edge.translation = np.array([0.1, 0.2, 0.3], dtype=np.float64)
    edge.rotation_xyzw = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    assert isinstance(edge.translation[0], np.float64)
    value = BoundedStaticTf()
    value.bridge_boot_id = "boot"
    value.bridge_gid = "02"
    value.raw_gid = "01"
    value.generation = value.sequence = value.first_sequence = 1
    value.valid = value.admitted = True
    value.admitted_from_ros.nanosec = 200
    value.admitted_from_steady = 0.2
    value.edges = [edge]
    binding = StaticBinding("base_link", "camera")
    assert observe_verified_tf(binding, value, current_relay_gid="02")
    assert binding.current is not None
    assert binding.current.edges[0].translation == (0.1, 0.2, 0.3)
    assert binding.admitted_from == (200, 0.2)


def test_generated_camera_info_builds_frame_and_native_float_snapshot():
    capture = RosHeader()
    capture.stamp.nanosec = 100
    capture.received_ros.nanosec = 101
    capture.received_steady = 1.1
    capture.frame_id = "optical"
    capture.raw_gid = "01"
    capture.bridge_gid = "02"
    capture.source_epoch = "source"
    capture.bridge_boot_id = "boot"
    capture.capture_generation = capture.sequence = 1
    capture.valid = True
    rgb = BoundedRawImage()
    depth = BoundedRawImage()
    for image in (rgb, depth):
        image.capture = capture
        image.width = image.height = 80
    rgb.step, rgb.encoding = 240, "rgb8"
    rgb.data = [255, 0, 0] * (80 * 80)
    depth.step, depth.encoding = 320, "32FC1"
    depth.data = list(np.ones((80, 80), dtype="<f4").tobytes())
    info = BoundedRawCameraInfo()
    info.capture = capture
    info.width = info.height = 80
    info.distortion_model = "plumb_bob"
    info.d = [0.0] * 5
    info.k = np.array([80.0, 0.0, 40.0, 0.0, 80.0, 40.0, 0.0, 0.0, 1.0])
    info.r = np.eye(3, dtype=np.float64).flatten()
    info.p = np.array([80.0, 0.0, 40.0, 0.0, 0.0, 80.0, 40.0, 0.0, 0.0, 0.0, 1.0, 0.0])
    assert isinstance(info.k[0], np.float64)
    header = capture_from_message(capture)
    built = build_frame(
        (header,) * 4,
        rgb,
        depth,
        info,
        info,
        calibration_epoch="cal",
        registration="registered",
        tf_epoch="tf",
        dependencies=("tf",),
        exclusion=bytes(80 * 80),
        static_sim_declared=True,
    )
    assert isinstance(info.roi[0], np.uint32)
    roundtrip = BoundedRawCameraInfo()
    roundtrip.roi = built.rgb_info.roi
    assert tuple(roundtrip.roi) == (0, 0, 0, 0)
    assert all(type(value) is int for value in built.rgb_info.roi)
    assert type(built.frame.calibration.fx) is float
    assert all(
        type(value) is float
        for name in ("d", "k", "r", "p")
        for value in getattr(built.rgb_info, name)
    )
