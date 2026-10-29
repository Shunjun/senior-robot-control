"""Passive heading observations. This type does not move the robot.

A forward command is not measured motion. Reverse, curves, slip, jumps,
standstill, a short baseline, and an ungeoreferenced map produce no observation.
A long straight run can correct a wrapped angle. The result is for a fusion
estimator to accept or reject. Wheel odometry, static TF, SLAM geometry, and
Home stay untouched.
"""

import math
from dataclasses import dataclass

from senior_control.localization.geo import quaternion_yaw

_MAX_TURN_RAD = 0.2
_AGREE_RAD = 0.12
_TRACK_RATIO = 0.35


@dataclass(frozen=True)
class MotionSample:
    """One straight-line window. ``commanded_forward`` does not prove the window moved."""

    stamp: float
    map_yaw: float
    map_dx: float
    map_dy: float
    enu_east_m: float
    enu_north_m: float
    odom_forward_m: float
    imu_yaw_delta_rad: float
    gnss_trusted: bool
    georeferenced: bool
    commanded_forward: bool
    slip: bool
    jump: bool
    alignment_rotation: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)


@dataclass(frozen=True)
class HeadingObservation:
    """Bounded correction. ``ready`` stays false when the source is unknown."""

    correction_rad: float
    source: str
    ready: bool


class HeadingObserver:
    """Compare ENU travel with map yaw after the caller has already moved."""

    def __init__(self, *, min_baseline_m: float, max_correction_rad: float) -> None:
        self._min_baseline_m = min_baseline_m
        self._max_correction_rad = max_correction_rad

    def observe(self, sample: MotionSample) -> HeadingObservation | None:
        if not _motion_window(sample, self._min_baseline_m):
            return None
        if not sample.gnss_trusted:
            return HeadingObservation(0.0, "gnss", False)
        psi = quaternion_yaw(sample.alignment_rotation)
        enu_course = math.atan2(sample.enu_north_m, sample.enu_east_m)
        map_course = math.atan2(sample.map_dy, sample.map_dx)
        enu_in_map = _wrap(enu_course + psi)
        yaw_error = _wrap(enu_in_map - sample.map_yaw)
        frame_error = _wrap(enu_in_map - map_course)
        if abs(frame_error) <= _AGREE_RAD and abs(yaw_error) > _AGREE_RAD:
            return HeadingObservation(
                _bound(yaw_error, self._max_correction_rad),
                "heading",
                True,
            )
        if abs(_wrap(map_course - sample.map_yaw)) <= _AGREE_RAD and abs(frame_error) > _AGREE_RAD:
            return HeadingObservation(
                _bound(frame_error, self._max_correction_rad),
                "georeference",
                True,
            )
        if abs(yaw_error) <= _AGREE_RAD and abs(frame_error) <= _AGREE_RAD:
            return None
        residual = yaw_error if abs(yaw_error) >= abs(frame_error) else frame_error
        return HeadingObservation(residual, "unknown", False)


def _motion_window(sample: MotionSample, min_baseline_m: float) -> bool:
    if sample.slip or sample.jump or not sample.georeferenced:
        return False
    if sample.odom_forward_m <= 0.0 or abs(sample.imu_yaw_delta_rad) > _MAX_TURN_RAD:
        return False
    if not _finite(sample):
        return False
    baseline = math.hypot(sample.enu_east_m, sample.enu_north_m)
    map_baseline = math.hypot(sample.map_dx, sample.map_dy)
    if baseline < min_baseline_m or map_baseline < min_baseline_m:
        return False
    if abs(sample.odom_forward_m - baseline) / baseline > _TRACK_RATIO:
        return False
    return abs(map_baseline - baseline) / baseline <= _TRACK_RATIO


def _finite(sample: MotionSample) -> bool:
    values = (
        sample.stamp,
        sample.map_yaw,
        sample.map_dx,
        sample.map_dy,
        sample.enu_east_m,
        sample.enu_north_m,
        sample.odom_forward_m,
        sample.imu_yaw_delta_rad,
    )
    return all(math.isfinite(value) for value in values)


def _wrap(delta: float) -> float:
    return math.atan2(math.sin(delta), math.cos(delta))


def _bound(value: float, limit: float) -> float:
    if value > limit:
        return limit
    if value < -limit:
        return -limit
    return value
