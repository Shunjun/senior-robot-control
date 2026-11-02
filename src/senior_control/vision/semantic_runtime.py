"""Registered RGB-D semantic candidates, never a free-space or laser mask claim."""

import hashlib
import math
from dataclasses import dataclass
from typing import TypeVar

from senior_control.vision.depth import GeometrySample
from senior_control.vision.depth_runtime import CameraInfoFrame, _intrinsics
from senior_control.vision.detector import Detection
from senior_control.vision.synchronizer import ImageFrame


@dataclass(frozen=True)
class SemanticAssociation:
    """A sampled endpoint inside a 2-D box, pending actual 3-D association."""

    sample: GeometrySample
    object_mask: int


@dataclass(frozen=True)
class SampledGeometry:
    """Only actual 3-D endpoints, with no invented range rays or free space."""

    stamp: float
    samples: tuple[GeometrySample, ...]


def _text(value: str, limit: int) -> bool:
    try:
        return bool(value) and len(value.encode("utf-8")) <= limit
    except (AttributeError, UnicodeError):
        return False


def qualify_registration(
    info: CameraInfoFrame,
    depth: "DepthSnapshot",
    *,
    rgb_stamp_ns: int,
    rgb_frame: str,
    registered_profile: bool,
    now_ros_ns: int,
    now_steady: float,
    max_age_s: float,
) -> str:
    """Require an explicitly qualified same-grid registration and first receipts.

    Equal calibration alone does not prove registration; ``registered_profile``
    must be set from an independently qualified source/profile. Identical stamps
    avoid assigning an asynchronously detected box to a different capture.
    """
    if (
        not registered_profile
        or not math.isfinite(now_steady)
        or not math.isfinite(max_age_s)
        or not 0 < max_age_s <= 0.3
        or now_ros_ns <= 0
        or min(rgb_stamp_ns, depth.stamp_ns, depth.info_stamp_ns, info.stamp_ns) <= 0
        or len({rgb_stamp_ns, depth.stamp_ns, depth.info_stamp_ns, info.stamp_ns}) != 1
        or not _text(rgb_frame, 128)
        or info.frame_id != rgb_frame or depth.frame_id != rgb_frame
        or not _text(info.raw_source, 128) or not _text(depth.raw_source, 128)
        or not _text(depth.info_source, 128) or not _text(depth.source_epoch, 64)
        or not _text(depth.calibration_id, 64)
        or info.width != depth.width or info.height != depth.height
        or not 1 <= info.width <= 4096 or not 1 <= info.height <= 4096
        or info.width * info.height > 2_073_600
    ):
        raise ValueError("registration_identity_unqualified")
    age_ns = max_age_s * 1e9
    if (
        not 0 <= now_ros_ns - rgb_stamp_ns <= age_ns
        or any(not 0 <= now_ros_ns - receipt <= age_ns for receipt in (
            info.received_ros_ns, depth.received_ros_ns, depth.info_received_ros_ns,
        ))
        or any(not math.isfinite(receipt) or not 0 <= now_steady - receipt <= max_age_s
               for receipt in (
                   info.received_steady, depth.received_steady, depth.info_received_steady,
               ))
    ):
        raise ValueError("registration_expired")
    intrinsics = _intrinsics(info)
    other = (depth.fx, depth.fy, depth.cx, depth.cy)
    if (
        not all(math.isfinite(value) for value in other)
        or intrinsics[0] <= 0 or intrinsics[1] <= 0
        or not 0 <= intrinsics[2] <= info.width or not 0 <= intrinsics[3] <= info.height
        or any(abs(a - b) > 1e-8 for a, b in zip(intrinsics, other, strict=True))
    ):
        raise ValueError("registration_calibration_mismatch")
    identity = (
        info.frame_id, info.raw_source, depth.raw_source, depth.info_source,
        depth.calibration_id, info.width, info.height, info.distortion_model, info.d,
        info.k, info.r, info.p, info.binning_x, info.binning_y, info.roi,
    )
    return hashlib.sha256(repr(identity).encode("utf-8")).hexdigest()


def associate_samples(
    detections: tuple[Detection, ...], samples: tuple[GeometrySample, ...]
) -> tuple[SemanticAssociation, ...]:
    """Bound box/pixel membership, preserving every depth and no occlusion claims."""
    if len(detections) > 32 or len(samples) > 8192:
        raise ValueError("semantic_size")
    for detection in detections:
        if (
            not _text(detection.label, 64)
            or not math.isfinite(detection.confidence)
            or not 0 <= detection.confidence <= 1
            or len(detection.box) != 4
            or not all(math.isfinite(value) for value in detection.box)
            or detection.box[0] > detection.box[2] or detection.box[1] > detection.box[3]
        ):
            raise ValueError("semantic_detection_invalid")
    matches = []
    for sample in samples:
        mask = 0
        for index, detection in enumerate(detections):
            x0, y0, x1, y1 = detection.box
            if x0 <= sample.u <= x1 and y0 <= sample.v <= y1:
                mask |= 1 << index
        if mask:
            matches.append(SemanticAssociation(sample, mask))
    return tuple(matches)


@dataclass(frozen=True)
class ColorCapture:
    stamp_ns: int
    received_ros_ns: int
    received_steady: float
    frame_id: str
    raw_source: str
    image: "ImageFrame"


@dataclass(frozen=True)
class DepthSnapshot:
    stamp_ns: int
    info_stamp_ns: int
    received_ros_ns: int
    info_received_ros_ns: int
    transform_stamp_ns: int
    transform_static: bool
    received_steady: float
    info_received_steady: float
    frame_id: str
    body_frame: str
    raw_source: str
    info_source: str
    source_epoch: str
    calibration_id: str
    sequence: int
    width: int
    height: int
    column_stride: int
    row_stride: int
    fx: float
    fy: float
    cx: float
    cy: float
    min_range_m: float
    max_range_m: float
    range_error_m: float
    origin: tuple[float, float, float]
    quaternion: tuple[float, float, float, float]
    samples: tuple[GeometrySample, ...]


def _nanoseconds(value: object) -> int:
    sec, nano = int(value.sec), int(value.nanosec)  # type: ignore[attr-defined]
    if sec < 0 or not 0 <= nano < 1_000_000_000:
        raise ValueError("invalid_timestamp")
    return sec * 1_000_000_000 + nano


def depth_snapshot(message: object) -> DepthSnapshot:
    """Copy and verify every wire endpoint against its pixel ray and measured TF."""
    try:
        return _depth_snapshot(message)
    except (AttributeError, IndexError, OverflowError, TypeError) as error:
        raise ValueError("depth_wire_invalid") from error


def _depth_snapshot(message: object) -> DepthSnapshot:
    # ROS-generated messages are intentionally confined to this adapter.
    from typing import Any, cast

    wire = cast(Any, message)
    stamp, info_stamp = _nanoseconds(wire.stamp), _nanoseconds(wire.camera_info_stamp)
    received, info_received = _nanoseconds(wire.received_ros), _nanoseconds(
        wire.camera_info_received_ros
    )
    tf_stamp = _nanoseconds(wire.transform_stamp)
    width, height = int(wire.width), int(wire.height)
    rows, cols = int(wire.row_stride), int(wire.column_stride)
    fx, fy, cx, cy = map(float, (wire.fx, wire.fy, wire.cx, wire.cy))
    minimum, maximum, error = map(float, (wire.min_range_m, wire.max_range_m, wire.range_error_m))
    origin = tuple(float(x) for x in wire.sensor_origin)
    quaternion = tuple(float(x) for x in wire.optical_to_body_quaternion)
    steady, info_steady = float(wire.received_steady), float(wire.camera_info_received_steady)
    if (
        not wire.valid or stamp <= 0 or info_stamp != stamp
        or received <= 0 or info_received <= 0 or int(wire.sequence) <= 0
        or not _text(wire.frame_id, 128) or not _text(wire.body_frame, 128)
        or not _text(wire.raw_source, 128) or not _text(wire.camera_info_source, 128)
        or not _text(wire.source_epoch, 64) or not _text(wire.calibration_id, 64)
        or not 1 <= width <= 4096 or not 1 <= height <= 4096 or width * height > 2_073_600
        or rows <= 0 or cols <= 0 or not 1 <= len(wire.samples) <= 8192
        or len(origin) != 3 or len(quaternion) != 4
        or not all(math.isfinite(x) for x in (
            fx, fy, cx, cy, minimum, maximum, error, steady, info_steady, *origin, *quaternion
        ))
        or fx <= 0 or fy <= 0 or not 0 <= cx <= width or not 0 <= cy <= height
        or not 0 < minimum < maximum <= 100 or error <= 0
        or abs(sum(x * x for x in quaternion) - 1) > 0.001
        or (not wire.transform_static and not 0 <= stamp - tf_stamp <= 200_000_000)
    ):
        raise ValueError("depth_wire_invalid")
    norm = math.sqrt(sum(x * x for x in quaternion))
    qx, qy, qz, qw = (x / norm for x in quaternion)
    samples = []
    previous = (-1, -1)
    for sample in wire.samples:
        u, v, distance = int(sample.u), int(sample.v), float(sample.depth_m)
        actual = tuple(float(x) for x in (sample.x, sample.y, sample.z))
        if (
            not 0 <= u < width or not 0 <= v < height or u % cols or v % rows
            or (u, v) <= previous or not minimum <= distance <= maximum
            or not all(math.isfinite(x) for x in (distance, *actual))
        ):
            raise ValueError("depth_sample_invalid")
        previous = (u, v)
        px, py, pz = (u - cx) * distance / fx, (v - cy) * distance / fy, distance
        tx, ty, tz = 2 * (qy * pz - qz * py), 2 * (qz * px - qx * pz), 2 * (qx * py - qy * px)
        expected = (px + qw * tx + qy * tz - qz * ty + origin[0],
                    py + qw * ty + qz * tx - qx * tz + origin[1],
                    pz + qw * tz + qx * ty - qy * tx + origin[2])
        if any(abs(a - b) > 1e-4 * max(1.0, distance)
               for a, b in zip(actual, expected, strict=True)):
            raise ValueError("depth_sample_transform_mismatch")
        samples.append(GeometrySample(u // cols, u, v, distance, *actual))
    return DepthSnapshot(
        stamp, info_stamp, received, info_received, tf_stamp, bool(wire.transform_static),
        steady, info_steady, str(wire.frame_id), str(wire.body_frame), str(wire.raw_source),
        str(wire.camera_info_source), str(wire.source_epoch), str(wire.calibration_id),
        int(wire.sequence), width, height, cols, rows, fx, fy, cx, cy, minimum, maximum, error,
        (origin[0], origin[1], origin[2]),
        (quaternion[0], quaternion[1], quaternion[2], quaternion[3]), tuple(samples),
    )


_T = TypeVar("_T")


class SemanticSynchronizer:
    """Bounded exact-frame pairing; wait for future timestamps without restamping."""

    def __init__(self, *, max_age_s: float) -> None:
        if not math.isfinite(max_age_s) or not 0 < max_age_s <= 0.3:
            raise ValueError("invalid_age")
        self._age = max_age_s
        self._rgb: dict[int, ColorCapture] = {}
        self._info: dict[int, CameraInfoFrame] = {}
        self._depth: dict[int, DepthSnapshot] = {}
        self._last: dict[str, int] = {}
        self._last_ros: int | None = None

    def clear(self) -> None:
        self._rgb.clear()
        self._info.clear()
        self._depth.clear()
        self._last.clear()
        self._last_ros = None

    def _offer(self, name: str, stamp: int, value: _T, target: dict[int, _T]) -> None:
        if stamp <= self._last.get(name, 0):
            self.clear()
            raise ValueError("source_out_of_order")
        self._last[name] = stamp
        if len(target) >= 4:
            target.pop(next(iter(target)))
        target[stamp] = value

    def offer_rgb(self, frame: ColorCapture) -> None:
        self._offer("rgb", frame.stamp_ns, frame, self._rgb)

    def offer_info(self, frame: CameraInfoFrame) -> None:
        self._offer("info", frame.stamp_ns, frame, self._info)

    def offer_depth(self, frame: DepthSnapshot) -> None:
        self._offer("depth", frame.stamp_ns, frame, self._depth)

    def take_ready(
        self, *, now_ros_ns: int, now_steady: float
    ) -> tuple[tuple[ColorCapture, CameraInfoFrame, DepthSnapshot] | None, str | None]:
        if (now_ros_ns <= 0 or not math.isfinite(now_steady)
            or (self._last_ros is not None and now_ros_ns < self._last_ros)):
            self.clear()
            self._last_ros = now_ros_ns
            return None, "clock_unqualified"
        self._last_ros = now_ros_ns
        common = self._rgb.keys() & self._info.keys() & self._depth.keys()
        if not common:
            return None, None
        stamp = min(common)
        rgb, info, depth = self._rgb[stamp], self._info[stamp], self._depth[stamp]
        receipts = (rgb.received_steady, info.received_steady, depth.received_steady,
                    depth.info_received_steady)
        age = now_steady - min(receipts)
        if stamp > now_ros_ns and 0 <= age <= min(0.02, self._age):
            return None, None
        for queue in (self._rgb, self._info, self._depth):
            for old in tuple(queue):
                if old <= stamp:
                    del queue[old]
        if stamp > now_ros_ns:
            return None, "future_expired"
        if not 0 <= age <= self._age or not 0 <= now_ros_ns - stamp <= self._age * 1e9:
            return None, "frame_expired"
        return (rgb, info, depth), None
