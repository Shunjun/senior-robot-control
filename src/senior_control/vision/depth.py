"""Depth projection into robot-base geometry.

Free space is never inferred here. A column becomes a hit ray only when a valid
depth pixel lands inside the obstacle height band. Invalid, out-of-range, or
band-empty columns yield invalid rays with ``no_return_free`` False, so
downstream consumers cannot clear map cells from missing depth.
"""

import math
from collections.abc import Mapping
from dataclasses import dataclass
from typing import Literal

from senior_control.mapping.evidence import RangeRay
from senior_control.vision.synchronizer import AlignedRgbdFrame

ColumnStatus = Literal["hit", "no_obstacle", "invalid"]


class CalibrationError(ValueError):
    """Camera data that cannot be turned into base-frame geometry."""


def _finite(*values: float) -> bool:
    return all(math.isfinite(value) for value in values)


@dataclass(frozen=True)
class CameraExtrinsic:
    """Pose of the camera body frame in ``base_link``.

    The camera body frame is x forward, y left, z up. Rotation is roll about x,
    pitch about y (positive tilts the optical axis down), then yaw about z.
    """

    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float

    def __post_init__(self) -> None:
        if not _finite(self.x, self.y, self.z, self.roll, self.pitch, self.yaw):
            raise CalibrationError("extrinsic values must be finite")


@dataclass(frozen=True)
class CameraCalibration:
    """Pinhole intrinsics for a depth image registered to color.

    ``depth_unit_m`` converts a raw depth value to meters (0.001 for 16UC1
    millimeters, 1.0 for 32FC1 meters). Every field is required: there is no
    default intrinsic or mounting pose.
    """

    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    depth_unit_m: float
    min_range_m: float
    max_range_m: float
    extrinsic: CameraExtrinsic

    def __post_init__(self) -> None:
        numbers = (
            self.fx,
            self.fy,
            self.cx,
            self.cy,
            self.depth_unit_m,
            self.min_range_m,
            self.max_range_m,
        )
        if not _finite(*numbers):
            raise CalibrationError("calibration values must be finite")
        if self.width <= 0 or self.height <= 0:
            raise CalibrationError("calibration image size must be positive")
        if self.fx <= 0.0 or self.fy <= 0.0:
            raise CalibrationError("focal length must be positive")
        if not 0.0 <= self.cx <= self.width or not 0.0 <= self.cy <= self.height:
            raise CalibrationError("principal point lies outside the image")
        if self.depth_unit_m <= 0.0:
            raise CalibrationError("depth unit must be positive")
        if self.min_range_m < 0.0 or self.max_range_m <= self.min_range_m:
            raise CalibrationError("depth range must satisfy 0 <= min < max")

    @classmethod
    def from_mapping(cls, values: Mapping[str, object]) -> "CameraCalibration":
        """Build from ROS parameters or YAML. A missing field is rejected, not defaulted."""
        extrinsic_values = values.get("extrinsic")
        if not isinstance(extrinsic_values, Mapping):
            raise CalibrationError("missing calibration field: extrinsic")
        try:
            extrinsic = CameraExtrinsic(
                **{name: _number(extrinsic_values, name) for name in _EXTRINSIC_FIELDS}
            )
            return cls(
                width=int(_number(values, "width")),
                height=int(_number(values, "height")),
                fx=_number(values, "fx"),
                fy=_number(values, "fy"),
                cx=_number(values, "cx"),
                cy=_number(values, "cy"),
                depth_unit_m=_number(values, "depth_unit_m"),
                min_range_m=_number(values, "min_range_m"),
                max_range_m=_number(values, "max_range_m"),
                extrinsic=extrinsic,
            )
        except (TypeError, ValueError) as error:
            if isinstance(error, CalibrationError):
                raise
            raise CalibrationError(f"invalid calibration: {error}") from error


_EXTRINSIC_FIELDS = ("x", "y", "z", "roll", "pitch", "yaw")


def _number(values: Mapping[str, object], name: str) -> float:
    if name not in values:
        raise CalibrationError(f"missing calibration field: {name}")
    value = values[name]
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise CalibrationError(f"calibration field is not a number: {name}")
    return float(value)


@dataclass(frozen=True)
class GeometryConfig:
    column_stride: int = 4
    row_stride: int = 4
    min_obstacle_z_m: float = 0.05
    max_obstacle_z_m: float = 1.5
    max_invalid_fraction: float = 0.5

    def __post_init__(self) -> None:
        if self.column_stride <= 0 or self.row_stride <= 0:
            raise ValueError("strides must be positive")
        if not _finite(self.min_obstacle_z_m, self.max_obstacle_z_m, self.max_invalid_fraction):
            raise ValueError("geometry thresholds must be finite")
        if self.max_obstacle_z_m <= self.min_obstacle_z_m:
            raise ValueError("obstacle band must have positive height")
        if not 0.0 <= self.max_invalid_fraction <= 1.0:
            raise ValueError("max_invalid_fraction must be within [0, 1]")


@dataclass(frozen=True)
class GeometryPoint:
    ray_index: int
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class GeometryObservation:
    """One depth frame as per-column rays in ``base_link``.

    ``rays``, ``status``, ``occlusion_mask`` and ``columns`` are index-aligned.
    ``occlusion_mask`` marks columns whose depth is too incomplete to clear
    anything behind them. ``points`` holds the nearest in-band return per hit
    column and is what a real-time safety consumer should treat as obstacles.
    """

    stamp: float
    skew_s: float
    columns: tuple[int, ...]
    rays: tuple[RangeRay, ...]
    status: tuple[ColumnStatus, ...]
    occlusion_mask: tuple[bool, ...]
    points: tuple[GeometryPoint, ...]
    quality: float


class _Rotation:
    def __init__(self, extrinsic: CameraExtrinsic) -> None:
        cr, sr = math.cos(extrinsic.roll), math.sin(extrinsic.roll)
        cp, sp = math.cos(extrinsic.pitch), math.sin(extrinsic.pitch)
        cy, sy = math.cos(extrinsic.yaw), math.sin(extrinsic.yaw)
        self.rows = (
            (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp, cp * sr, cp * cr),
        )
        self.offset = (extrinsic.x, extrinsic.y, extrinsic.z)

    def rotate(self, p: tuple[float, float, float]) -> tuple[float, float, float]:
        r = self.rows
        return (
            r[0][0] * p[0] + r[0][1] * p[1] + r[0][2] * p[2],
            r[1][0] * p[0] + r[1][1] * p[1] + r[1][2] * p[2],
            r[2][0] * p[0] + r[2][1] * p[1] + r[2][2] * p[2],
        )

    def to_base(self, p: tuple[float, float, float]) -> tuple[float, float, float]:
        x, y, z = self.rotate(p)
        return (x + self.offset[0], y + self.offset[1], z + self.offset[2])


def project_depth(
    frame: AlignedRgbdFrame,
    calibration: CameraCalibration,
    config: GeometryConfig | None = None,
) -> GeometryObservation:
    config = config or GeometryConfig()
    depth = frame.depth
    if (depth.width, depth.height) != (calibration.width, calibration.height):
        raise CalibrationError("depth image size does not match the calibration size")
    rotation = _Rotation(calibration.extrinsic)
    columns = tuple(range(0, depth.width, config.column_stride))
    rows = tuple(range(0, depth.height, config.row_stride))
    rays: list[RangeRay] = []
    statuses: list[ColumnStatus] = []
    occlusion: list[bool] = []
    points: list[GeometryPoint] = []
    valid_total = 0
    for index, u in enumerate(columns):
        best: tuple[float, float, float] | None = None
        best_range = math.inf
        valid_count = 0
        for v in rows:
            raw = depth.values[v * depth.width + u]
            z_cam = raw * calibration.depth_unit_m if math.isfinite(raw) else math.nan
            if not (calibration.min_range_m <= z_cam <= calibration.max_range_m):
                continue
            valid_count += 1
            x_cam = (u - calibration.cx) / calibration.fx * z_cam
            y_cam = (v - calibration.cy) / calibration.fy * z_cam
            # Optical (x right, y down, z forward) to camera body (x forward, y left, z up).
            point = rotation.to_base((z_cam, -x_cam, -y_cam))
            if not config.min_obstacle_z_m <= point[2] <= config.max_obstacle_z_m:
                continue
            horizontal = math.hypot(point[0], point[1])
            if horizontal < best_range:
                best, best_range = point, horizontal
        valid_total += valid_count
        invalid_fraction = 1.0 - valid_count / len(rows)
        occlusion.append(invalid_fraction > config.max_invalid_fraction)
        if best is not None:
            statuses.append("hit")
            angle = math.atan2(best[1], best[0])
            rays.append(RangeRay(angle, best_range, True, False, None))
            points.append(GeometryPoint(index, best[0], best[1], best[2]))
        else:
            statuses.append("no_obstacle" if valid_count else "invalid")
            ahead = rotation.rotate((1.0, -(u - calibration.cx) / calibration.fx, 0.0))
            rays.append(RangeRay(math.atan2(ahead[1], ahead[0]), 0.0, False, False, None))
    return GeometryObservation(
        stamp=frame.stamp,
        skew_s=frame.skew_s,
        columns=columns,
        rays=tuple(rays),
        status=tuple(statuses),
        occlusion_mask=tuple(occlusion),
        points=tuple(points),
        quality=valid_total / (len(columns) * len(rows)),
    )
