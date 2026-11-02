"""Build a research frame from four already authenticated bounded captures.

The caller owns relay/GID authentication, static binding lifetime, capture
frontiers and admission. Neither this conversion nor its strings authenticate
or qualify a source, scene, mount, or motion result.
"""

import hashlib
import math
from dataclasses import dataclass
from typing import Any

from senior_control.vision.independent_motion import Calibration, Frame
from senior_control.vision.raw_capture import (
    RawCaptureHeader,
    capture_from_message,
    compatible_captures,
)


@dataclass(frozen=True)
class ImageLayoutSnapshot:
    capture: RawCaptureHeader
    width: int
    height: int
    step: int
    encoding: str
    is_bigendian: bool
    byte_length: int
    sha256: bytes


def _snapshot_layout(image: Any, capture: RawCaptureHeader, body: bytes) -> ImageLayoutSnapshot:
    return ImageLayoutSnapshot(
        capture,
        image.width,
        image.height,
        image.step,
        image.encoding,
        image.is_bigendian,
        len(body),
        hashlib.sha256(body).digest(),
    )


@dataclass(frozen=True)
class CameraInfoSnapshot:
    capture: RawCaptureHeader
    width: int
    height: int
    distortion_model: str
    d: tuple[float, ...]
    k: tuple[float, ...]
    r: tuple[float, ...]
    p: tuple[float, ...]
    binning_x: int
    binning_y: int
    roi: tuple[int, ...]
    roi_do_rectify: bool


def _snapshot_info(info: Any, capture: RawCaptureHeader) -> CameraInfoSnapshot:
    return CameraInfoSnapshot(
        capture,
        info.width,
        info.height,
        info.distortion_model,
        tuple(info.d),
        tuple(info.k),
        tuple(info.r),
        tuple(info.p),
        info.binning_x,
        info.binning_y,
        tuple(info.roi),
        info.roi_do_rectify,
    )


@dataclass(frozen=True)
class BuiltFrame:
    frame: Frame
    raw_rgb_encoding: str
    preprocessing_version: str
    rgb_layout: ImageLayoutSnapshot
    depth_layout: ImageLayoutSnapshot
    rgb_info: CameraInfoSnapshot
    depth_info: CameraInfoSnapshot


def _info(info: Any, width: int, height: int) -> tuple[float, float, float, float]:
    try:
        d, k, r, p = tuple(info.d), tuple(info.k), tuple(info.r), tuple(info.p)
        if (
            info.width != width
            or info.height != height
            or info.distortion_model != "plumb_bob"
            or not 0 < len(d) <= 16
            or len(k) != 9
            or len(r) != 9
            or len(p) != 12
            or not all(math.isfinite(x) for x in (*d, *k, *r, *p))
            or any(abs(x) > 1e-9 for x in d)
            or info.binning_x not in (0, 1)
            or info.binning_y not in (0, 1)
            or tuple(info.roi) not in ((0, 0, 0, 0), (0, 0, width, height))
            or info.roi_do_rectify
        ):
            raise ValueError("unsupported_camera_info")
        fx, fy, cx, cy = k[0], k[4], k[2], k[5]
        expected = (
            (fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0),
            (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0),
            (fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0),
        )
        if any(
            abs(a - b) > 1e-8
            for actual, want in zip((k, r, p), expected, strict=True)
            for a, b in zip(actual, want, strict=True)
        ):
            raise ValueError("invalid_calibration")
        if not (
            0.1 * width <= fx <= 10 * width
            and 0.1 * height <= fy <= 10 * height
            and 0 <= cx < width
            and 0 <= cy < height
        ):
            raise ValueError("invalid_calibration")
        return fx, fy, cx, cy
    except (AttributeError, TypeError, OverflowError) as exc:
        raise ValueError("invalid_camera_info") from exc


def build_frame(
    headers: tuple[RawCaptureHeader, ...],
    rgb: Any,
    depth: Any,
    rgb_info: Any,
    depth_info: Any,
    *,
    calibration_epoch: str,
    registration: str,
    tf_epoch: str,
    dependencies: tuple[str, ...],
    exclusion: bytes,
    static_sim_declared: bool,
) -> BuiltFrame:
    """Decode exact RGB8 and registered rectified 32FC1, retaining raw identities."""
    import numpy as np

    if not registration or not calibration_epoch or not tf_epoch:
        raise ValueError("registration_or_calibration_unavailable")
    if (
        type(headers) is not tuple
        or len(headers) != 4
        or not all(type(h) is RawCaptureHeader for h in headers)
        or not compatible_captures(*headers)
        or len({h.frame_id for h in headers}) != 1
    ):
        raise ValueError("capture_mismatch")
    try:
        for header, body in zip(headers, (rgb, depth, rgb_info, depth_info), strict=True):
            raw = body.capture
            copied = raw if type(raw) is RawCaptureHeader else capture_from_message(raw)
            if copied != header:
                raise ValueError("capture_body_mismatch")
        width, height = rgb.width, rgb.height
        if (
            type(width) is not int
            or type(height) is not int
            or not 80 <= width <= 640
            or not 80 <= height <= 480
            or depth.width != width
            or depth.height != height
            or rgb.encoding != "rgb8"
            or depth.encoding != "32FC1"
            or type(rgb.is_bigendian) is not bool
            or type(depth.is_bigendian) is not bool
            or type(rgb.step) is not int
            or type(depth.step) is not int
            or rgb.step < width * 3
            or depth.step < width * 4
            or rgb.step * height > 16_777_216
            or depth.step * height > 16_777_216
            or len(rgb.data) != rgb.step * height
            or len(depth.data) != depth.step * height
        ):
            raise ValueError("image_layout")
        if type(exclusion) is not bytes or len(exclusion) != width * height:
            raise ValueError("invalid_exclusion")
        if type(static_sim_declared) is not bool or (
            not any(exclusion) and not static_sim_declared
        ):
            raise ValueError("static_sim_domain_unavailable")
        intrinsics = _info(rgb_info, width, height)
        if _info(depth_info, width, height) != intrinsics:
            raise ValueError("registration_mismatch")
        rgb_body, depth_body = bytes(rgb.data), bytes(depth.data)
        if len(rgb_body) != rgb.step * height or len(depth_body) != depth.step * height:
            raise ValueError("image_layout")
        color = np.frombuffer(rgb_body, dtype=np.uint8).reshape(height, rgb.step)[
            :, : width * 3
        ]
        color = color.reshape(height, width, 3).astype(np.uint32)
        mono = (
            (77 * color[:, :, 0] + 150 * color[:, :, 1] + 29 * color[:, :, 2] + 128) >> 8
        ).astype(np.uint8)
        raw_depth = (
            np.frombuffer(depth_body, dtype=np.uint8)
            .reshape(height, depth.step)[:, : width * 4]
            .copy()
        )
        dtype = ">f4" if depth.is_bigendian else "<f4"
        z = raw_depth.reshape(height, width * 4).view(dtype).astype(np.float32)
    except (AttributeError, TypeError, OverflowError) as exc:
        raise ValueError("capture_or_image_layout") from exc
    calibration = Calibration(
        *intrinsics,
        calibration_epoch,
        registration,
        True,
        width,
        height,
        "mono8",
        "32FC1",
        True,
        "rectified",
        (0, 0, width, height),
        (1, 1),
    )
    frame = Frame(
        *headers,
        mono,
        z,
        calibration,
        tf_epoch,
        dependencies,
        np.frombuffer(exclusion, dtype=np.uint8).reshape(height, width),
    )
    return BuiltFrame(
        frame,
        "rgb8",
        "rgb8-mono-v1",
        _snapshot_layout(rgb, headers[0], rgb_body),
        _snapshot_layout(depth, headers[1], depth_body),
        _snapshot_info(rgb_info, headers[2]),
        _snapshot_info(depth_info, headers[3]),
    )


def offer_capture(
    estimator: Any,
    headers: tuple[RawCaptureHeader, ...],
    rgb: Any,
    depth: Any,
    rgb_info: Any,
    depth_info: Any,
    *,
    calibration_epoch: str,
    registration: str,
    tf_epoch: str,
    dependencies: tuple[str, ...],
    exclusion: bytes,
    static_sim_declared: bool,
    now_ros_ns: int,
    now_steady: float,
) -> Any:
    """Route authenticated captures exactly once; call only outside safety executor.

    Parent-side capture frontiers must survive worker death independently.
    """
    from senior_control.vision.independent_motion import Result

    try:
        built = build_frame(
            headers,
            rgb,
            depth,
            rgb_info,
            depth_info,
            calibration_epoch=calibration_epoch,
            registration=registration,
            tf_epoch=tf_epoch,
            dependencies=dependencies,
            exclusion=exclusion,
            static_sim_declared=static_sim_declared,
        )
    except ValueError:
        estimator.observe_unusable_headers(headers, now_ros_ns=now_ros_ns, now_steady=now_steady)
        return Result("invalid_frame"), None
    return estimator.offer(built.frame, now_ros_ns=now_ros_ns, now_steady=now_steady), built
