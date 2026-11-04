"""Copy a compact, unqualified RGB-D diagnostic into a bounded ROS message."""

from collections.abc import Callable
from typing import Any

from senior_control.vision.raw_capture import RawCaptureHeader
from senior_control.vision.rgbd_frame_builder import CameraInfoSnapshot, ImageLayoutSnapshot
from senior_control.vision.rgbd_spawn_worker import CompactResult
from senior_control.vision.static_tf_binding import Receipt


def _set_time(out: Any, value: int) -> None:
    if type(value) is not int or value < 0:
        raise ValueError("invalid_time")
    out.sec, out.nanosec = divmod(value, 1_000_000_000)


def _header(out: Any, value: RawCaptureHeader) -> None:
    _set_time(out.stamp, value.stamp_ns)
    _set_time(out.received_ros, value.received_ros_ns)
    for name in (
        "received_steady",
        "frame_id",
        "raw_gid",
        "bridge_gid",
        "source_epoch",
        "bridge_boot_id",
        "capture_generation",
        "sequence",
        "valid",
        "reason",
    ):
        setattr(out, name, getattr(value, name))


def _layout(out: Any, value: ImageLayoutSnapshot) -> None:
    _header(out.capture, value.capture)
    for name in ("width", "height", "step", "encoding", "is_bigendian", "byte_length"):
        setattr(out, name, getattr(value, name))
    if len(value.sha256) != 32:
        raise ValueError("invalid_image_digest")
    out.sha256 = list(value.sha256)


def _info(out: Any, value: CameraInfoSnapshot) -> None:
    _header(out.capture, value.capture)
    for name in ("width", "height", "distortion_model", "binning_x", "binning_y", "roi_do_rectify"):
        setattr(out, name, getattr(value, name))
    for name in ("d", "k", "r", "p", "roi"):
        setattr(out, name, list(getattr(value, name)))


def _tf(
    out: Any, value: Receipt, admitted: tuple[int, float], edge_factory: Callable[[], Any]
) -> None:
    out.bridge_boot_id, out.bridge_gid, out.raw_gid = value.boot, value.relay_gid, value.source_gid
    out.generation, out.sequence, out.first_sequence = (
        value.generation,
        value.sequence,
        value.sequence,
    )
    _set_time(out.received_ros, value.received_ros_ns)
    _set_time(out.first_received_ros, value.received_ros_ns)
    out.received_steady = out.first_received_steady = value.received_steady
    out.valid, out.admitted, out.reason = True, True, ""
    _set_time(out.admitted_from_ros, admitted[0])
    out.admitted_from_steady = admitted[1]
    edges = []
    for edge in value.edges:
        item = edge_factory()
        item.parent_frame, item.child_frame = edge.parent, edge.child
        _set_time(item.stamp, edge.stamp_ns)
        item.translation, item.rotation_xyzw = list(edge.translation), list(edge.quaternion)
        edges.append(item)
    out.edges = edges


def fill_candidate(
    out: Any,
    result: CompactResult,
    *,
    producer_gid: str,
    producer_boot: str,
    owner_generation: int,
    output_sequence: int,
    body_frame: str,
    edge_factory: Callable[[], Any],
) -> None:
    """Populate the native-fenced proposal; does not authenticate or qualify it."""
    if result.hard_error_qualified is not False:
        raise ValueError("qualification_forbidden")
    out.producer_gid, out.producer_boot_id = producer_gid, producer_boot
    out.owner_generation, out.worker_generation, out.sequence = (
        owner_generation,
        result.generation,
        output_sequence,
    )
    out.hard_error_qualified = False
    out.valid = False
    out.reason = result.reason[:64]
    if result.reason != "candidate":
        return
    if (
        len(result.headers) != 2
        or any(len(group) != 4 for group in result.headers)
        or len(result.image_layouts) != 2
        or any(len(group) != 2 for group in result.image_layouts)
        or len(result.camera_info) != 2
        or any(len(group) != 2 for group in result.camera_info)
        or len(result.tf_receipts) != 2
        or len(result.tf_admitted_from) != 2
        or result.camera_transform is None
        or len(result.raw_rgb_encodings) != 2
        or len(result.preprocessing_versions) != 2
        or result.raw_rgb_encodings[0] != result.raw_rgb_encodings[1]
        or result.preprocessing_versions[0] != result.preprocessing_versions[1]
    ):
        raise ValueError("incomplete_candidate")
    out.profile_id, out.calibration_id, out.registration_id = (
        result.profile_name,
        result.calibration_epoch,
        result.registration,
    )
    out.raw_rgb_encoding, out.preprocessing_version = (
        result.raw_rgb_encodings[0],
        result.preprocessing_versions[0],
    )
    out.body_frame = body_frame
    out.matched, out.inliers = result.matched, result.inliers
    out.median_reprojection_px, out.median_depth_residual_m = (
        result.median_reprojection_px,
        result.median_depth_residual_m,
    )
    out.camera_transform = [value for row in result.camera_transform for value in row]
    _set_time(out.interval_start, result.headers[0][0].stamp_ns)
    _set_time(out.interval_end, result.headers[1][0].stamp_ns)
    for index, prefix in enumerate(("first", "second")):
        for stream, value in zip(
            ("rgb", "depth", "rgb_info", "depth_info"), result.headers[index], strict=True
        ):
            _header(getattr(out, f"{prefix}_{stream}"), value)
        for position, stream in enumerate(("rgb", "depth")):
            _layout(
                getattr(out, f"{prefix}_{stream}_layout"), result.image_layouts[index][position]
            )
            _info(
                getattr(out, f"{prefix}_{stream}_calibration"), result.camera_info[index][position]
            )
        _tf(
            getattr(out, f"{prefix}_tf"),
            result.tf_receipts[index],
            result.tf_admitted_from[index],
            edge_factory,
        )
    out.valid = True
