"""Bounded depth production from explicit calibration and measurement-time TF.

A projected sample is an obstacle candidate, never a free-space claim. In
particular neither a valid pixel nor a column hit qualifies a swept 3-D volume.
"""

import hashlib
import math
from collections import deque
from dataclasses import dataclass

from senior_control.vision.depth import (
    CameraCalibration,
    GeometryConfig,
    GeometrySample,
    body_extrinsic_from_optical_tf,
    project_depth_image,
)
from senior_control.vision.image_codec import decode_depth
from senior_control.vision.runtime_evidence import DepthHeader


@dataclass(frozen=True)
class DepthPacket:
    stamp_ns: int
    received_ros_ns: int
    received_steady: float
    source_epoch: str
    raw_source: str
    sequence: int
    frame_id: str
    width: int
    height: int
    step: int
    encoding: str
    big_endian: bool
    data: bytes


@dataclass(frozen=True)
class CameraInfoFrame:
    stamp_ns: int
    received_ros_ns: int
    received_steady: float
    frame_id: str
    raw_source: str
    width: int
    height: int
    distortion_model: str
    d: tuple[float, ...]
    k: tuple[float, ...]
    r: tuple[float, ...]
    p: tuple[float, ...]
    binning_x: int
    binning_y: int
    roi: tuple[int, int, int, int, bool]


@dataclass(frozen=True)
class OpticalTransform:
    stamp_ns: int
    translation: tuple[float, float, float]
    rotation_xyzw: tuple[float, float, float, float]
    is_static: bool


@dataclass(frozen=True)
class DepthGeometrySettings:
    min_range_m: float
    max_range_m: float
    range_error_m: float
    row_stride: int
    column_stride: int
    body_frame: str
    tf_max_age_ns: int = 200_000_000


@dataclass(frozen=True)
class ProjectedDepthEvidence:
    header: DepthHeader
    info: CameraInfoFrame
    transform: OpticalTransform
    settings: DepthGeometrySettings
    origin: tuple[float, float, float]
    samples: tuple[GeometrySample, ...]
    coverage_qualified: bool = False


class DepthPairBuffer:
    """Bounded exact-stamp CameraInfo pairing with original receipt deadlines."""

    def __init__(self, *, max_age_s: float, pair_wait_s: float) -> None:
        if (
            not math.isfinite(max_age_s)
            or not math.isfinite(pair_wait_s)
            or not 0 < pair_wait_s <= max_age_s
        ):
            raise ValueError("invalid depth pairing limits")
        self.max_age_s = max_age_s
        self.pair_wait_s = pair_wait_s
        self._depth: deque[DepthPacket] = deque()
        self._info: deque[CameraInfoFrame] = deque()
        self._failure: str | None = None
        self._last_depth_ns = 0
        self._last_info_ns = 0
        self._last_ros_ns: int | None = None
        self.last_failure_packet: DepthPacket | None = None

    def clear(self) -> None:
        self._depth.clear()
        self._info.clear()
        self._failure = None
        self._last_depth_ns = 0
        self._last_info_ns = 0
        self.last_failure_packet = None
        self._last_ros_ns = None

    def offer_depth(self, packet: DepthPacket) -> bool:
        if packet.stamp_ns <= self._last_depth_ns:
            self.last_failure_packet = packet
            self._depth.clear()
            self._failure = "depth_out_of_order"
            return False
        self._last_depth_ns = packet.stamp_ns
        if len(self._depth) >= 4:
            self.last_failure_packet = packet
            self._depth.clear()
            self._failure = "depth_overflow"
            return False
        self._depth.append(packet)
        return True

    def offer_info(self, info: CameraInfoFrame) -> bool:
        if info.stamp_ns <= self._last_info_ns:
            self.last_failure_packet = self._depth[-1] if self._depth else None
            self._depth.clear()
            self._info.clear()
            self._failure = "camera_info_out_of_order"
            return False
        self._last_info_ns = info.stamp_ns
        if len(self._info) >= 16:
            self.last_failure_packet = self._depth[-1] if self._depth else None
            self._depth.clear()
            self._info.clear()
            self._failure = "camera_info_overflow"
            return False
        self._info.append(info)
        return True

    def take_ready(
        self, *, now_ros_ns: int, now_steady: float
    ) -> tuple[tuple[DepthPacket, CameraInfoFrame] | None, str | None]:
        if (
            now_ros_ns <= 0
            or not math.isfinite(now_steady)
            or (self._last_ros_ns is not None and now_ros_ns < self._last_ros_ns)
        ):
            self.clear()
            self._last_ros_ns = now_ros_ns
            return None, "clock_unqualified"
        self._last_ros_ns = now_ros_ns
        if self._failure is not None:
            failure = self._failure
            self._failure = None
            return None, failure
        if not self._depth:
            return None, None
        packet = self._depth[0]
        age = now_steady - packet.received_steady
        if not 0 <= age <= self.max_age_s:
            self.last_failure_packet = packet
            self._depth.clear()
            return None, "depth_receipt_expired"
        if packet.stamp_ns > now_ros_ns:
            if age > min(0.02, self.max_age_s):
                self.last_failure_packet = packet
                self._depth.clear()
                return None, "future_expired"
            return None, None
        if not 0 <= now_ros_ns - packet.stamp_ns <= self.max_age_s * 1e9:
            self.last_failure_packet = packet
            self._depth.clear()
            return None, "depth_stamp_expired"
        while self._info and self._info[0].stamp_ns < packet.stamp_ns:
            self._info.popleft()
        if not self._info or self._info[0].stamp_ns != packet.stamp_ns:
            if age > self.pair_wait_s:
                self.last_failure_packet = packet
                self._depth.popleft()
                return None, "camera_info_missing"
            return None, None
        info = self._info.popleft()
        self._depth.popleft()
        if (
            not 0 <= now_steady - info.received_steady <= self.max_age_s
            or not 0 <= now_ros_ns - info.received_ros_ns <= self.max_age_s * 1e9
        ):
            self.last_failure_packet = packet
            return None, "camera_info_receipt_expired"
        return (packet, info), None


def _identity(value: str, limit: int) -> bool:
    try:
        return bool(value) and len(value.encode("utf-8")) <= limit
    except UnicodeError:
        return False


def _intrinsics(info: CameraInfoFrame) -> tuple[float, float, float, float]:
    if (
        info.distortion_model != "plumb_bob"
        or len(info.d) > 16
        or len(info.k) != 9
        or len(info.r) != 9
        or len(info.p) != 12
        or not all(math.isfinite(value) for value in (*info.d, *info.k, *info.r, *info.p))
        or any(abs(value) > 1e-9 for value in info.d)
        or info.binning_x not in (0, 1)
        or info.binning_y not in (0, 1)
        or info.roi not in ((0, 0, 0, 0, False), (0, 0, info.width, info.height, False))
    ):
        raise ValueError("camera_info_unsupported")
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    expected_k = (fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0)
    expected_r = (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)
    expected_p = (fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0)
    if any(abs(a - b) > 1e-8 for a, b in zip(info.k, expected_k, strict=True)):
        raise ValueError("camera_info_intrinsic")
    if any(abs(a - b) > 1e-8 for a, b in zip(info.r, expected_r, strict=True)):
        raise ValueError("camera_info_rectification")
    if any(abs(a - b) > 1e-8 for a, b in zip(info.p, expected_p, strict=True)):
        raise ValueError("camera_info_projection")
    return fx, fy, cx, cy


def produce_depth_evidence(
    packet: DepthPacket,
    info: CameraInfoFrame,
    transform: OpticalTransform,
    settings: DepthGeometrySettings,
) -> ProjectedDepthEvidence:
    """Validate one exact-stamp pair and project its bounded actual samples."""
    if (
        packet.stamp_ns <= 0
        or packet.received_ros_ns <= 0
        or info.received_ros_ns <= 0
        or not math.isfinite(packet.received_steady)
        or not math.isfinite(info.received_steady)
        or packet.sequence <= 0
        or not _identity(packet.source_epoch, 64)
        or not _identity(packet.raw_source, 128)
        or not _identity(info.raw_source, 128)
        or not _identity(packet.frame_id, 128)
        or not _identity(settings.body_frame, 128)
        or packet.frame_id != info.frame_id
        or packet.stamp_ns != info.stamp_ns
        or (packet.width, packet.height) != (info.width, info.height)
    ):
        raise ValueError("depth_info_identity")
    if (
        not all(math.isfinite(v) for v in (
            settings.min_range_m, settings.max_range_m, settings.range_error_m
        ))
        or not 0 < settings.min_range_m < settings.max_range_m <= 100
        or settings.range_error_m <= 0
        or settings.row_stride <= 0
        or settings.column_stride <= 0
        or settings.tf_max_age_ns <= 0
        or (not transform.is_static and not 0 <= packet.stamp_ns - transform.stamp_ns
            <= settings.tf_max_age_ns)
    ):
        raise ValueError("depth_profile_or_transform")
    count = (
        ((packet.width + settings.column_stride - 1) // settings.column_stride)
        * ((packet.height + settings.row_stride - 1) // settings.row_stride)
    )
    if not 1 <= count <= 8192:
        raise ValueError("depth_sample_limit")
    fx, fy, cx, cy = _intrinsics(info)
    extrinsic = body_extrinsic_from_optical_tf(transform.translation, transform.rotation_xyzw)
    unit = {"16UC1": 0.001, "32FC1": 1.0}.get(packet.encoding)
    if unit is None:
        raise ValueError("unsupported depth encoding")
    calibration = CameraCalibration(
        packet.width, packet.height, fx, fy, cx, cy, unit,
        settings.min_range_m, settings.max_range_m, extrinsic,
    )
    frame = decode_depth(packet.stamp_ns / 1e9, packet.width, packet.height, packet.step,
                         packet.encoding, packet.data, packet.big_endian)
    observation = project_depth_image(frame, calibration, GeometryConfig(
        column_stride=settings.column_stride, row_stride=settings.row_stride,
    ))
    if not observation.samples:
        raise ValueError("depth_no_valid_samples")
    calibration_identity = (
        info.raw_source, info.frame_id, info.width, info.height, info.distortion_model,
        info.d, info.k, info.r, info.p, info.binning_x, info.binning_y, info.roi,
        transform.translation, transform.rotation_xyzw, transform.is_static,
        settings, packet.encoding,
    )
    calibration_id = hashlib.sha256(repr(calibration_identity).encode("utf-8")).hexdigest()
    header = DepthHeader(packet.source_epoch, packet.raw_source, packet.sequence, packet.stamp_ns,
                         packet.received_ros_ns, packet.received_steady, packet.frame_id,
                         calibration_id, True)
    return ProjectedDepthEvidence(header, info, transform, settings, transform.translation,
                                  observation.samples)
