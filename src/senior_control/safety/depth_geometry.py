"""Bounded, independently time-aligned depth hits; no free-volume assertion."""

import math
from dataclasses import dataclass
from typing import Any

from senior_control.models import Pose2D
from senior_control.safety.motion_runtime import TimedPoseBuffer, compose_pose, relative_pose


@dataclass(frozen=True)
class DepthCandidateResult:
    reason: str
    aligned_points: tuple[tuple[float, float, float], ...] = ()
    coverage_qualified: bool = False


def _rotate(q: tuple[float, ...], p: tuple[float, float, float]) -> tuple[float, float, float]:
    x, y, z, w = q
    px, py, pz = p
    tx, ty, tz = 2 * (y * pz - z * py), 2 * (z * px - x * pz), 2 * (x * py - y * px)
    return (
        px + w * tx + y * tz - z * ty,
        py + w * ty + z * tx - x * tz,
        pz + w * tz + x * ty - y * tx,
    )


def evaluate_depth_candidates(
    message: Any,
    history: TimedPoseBuffer,
    *,
    now: float,
    max_age: float,
    max_reference_lag: float,
    linear_bound: float,
    yaw_bound: float,
    radius: float,
    footprint: tuple[tuple[float, float], ...],
    collision_z: tuple[float, float],
    extra_margin: float,
) -> DepthCandidateResult:
    """Validate real sampled endpoints and stop on hits or unproven coverage.

    This first consumer deliberately does not clear any 3-D space. It does not
    discard self hits, since an unverified footprint volume cannot prove a
    return is robot rather than an obstacle. Work belongs in the safety worker,
    never the ROS output executor.
    """
    invalid = DepthCandidateResult("depth_geometry_invalid")
    values = (
        now,
        max_age,
        max_reference_lag,
        linear_bound,
        yaw_bound,
        radius,
        extra_margin,
        *collision_z,
    )
    if (
        not all(math.isfinite(value) for value in values)
        or min(max_age, max_reference_lag, linear_bound, yaw_bound, radius) <= 0
        or extra_margin < 0
        or collision_z[0] >= collision_z[1]
        or len(footprint) < 3
        or not all(math.isfinite(v) for p in footprint for v in p)
    ):
        return invalid
    try:
        stamp = (int(message.stamp.sec) * 1_000_000_000 + int(message.stamp.nanosec)) / 1e9
        width, height = int(message.width), int(message.height)
        fx, fy, cx, cy = (
            float(message.fx),
            float(message.fy),
            float(message.cx),
            float(message.cy),
        )
        minimum, maximum, error = (
            float(message.min_range_m),
            float(message.max_range_m),
            float(message.range_error_m),
        )
        origin = tuple(float(v) for v in message.sensor_origin)
        q = tuple(float(v) for v in message.optical_to_body_quaternion)
        rows, cols = int(message.row_stride), int(message.column_stride)
        if (
            not message.frame_id
            or not message.body_frame
            or not 1 <= width <= 4096
            or not 1 <= height <= 4096
            or width * height > 2_073_600
            or rows <= 0
            or cols <= 0
            or not 1 <= len(message.samples) <= 8192
            or len(origin) != 3
            or len(q) != 4
            or not all(
                math.isfinite(v) for v in (fx, fy, cx, cy, minimum, maximum, error, *origin, *q)
            )
            or fx <= 0
            or fy <= 0
            or not 0 <= cx <= width
            or not 0 <= cy <= height
            or not 0 < minimum < maximum <= 100
            or error <= 0
            or abs(sum(v * v for v in q) - 1.0) > 0.001
            or not 0 <= now - stamp <= max_age
        ):
            return invalid
        q_norm = math.sqrt(sum(value * value for value in q))
        q = tuple(value / q_norm for value in q)
        # A source sample is checked against the actual optical TF, not a
        # projected base-origin polar range or a second optical rotation.
        points = []
        previous = (-1, -1)
        for sample in message.samples:
            u, v, depth = int(sample.u), int(sample.v), float(sample.depth_m)
            actual = (float(sample.x), float(sample.y), float(sample.z))
            if (
                not 0 <= u < width
                or not 0 <= v < height
                or u % cols
                or v % rows
                or (u, v) <= previous
                or not math.isfinite(depth)
                or not minimum <= depth <= maximum
                or not all(math.isfinite(x) for x in actual)
            ):
                return invalid
            previous = (u, v)
            optical = ((u - cx) * depth / fx, (v - cy) * depth / fy, depth)
            transformed = _rotate(q, optical)
            expected = tuple(a + b for a, b in zip(origin, transformed, strict=True))
            # The wire carries float32 samples; this is a shape/consistency
            # tolerance, not a calibration uncertainty or free-space margin.
            tolerance = 1e-4 * max(1.0, depth)
            if any(abs(a - b) > tolerance for a, b in zip(actual, expected, strict=True)):
                return invalid
            points.append(actual)
    except (AttributeError, IndexError, TypeError, ValueError, OverflowError):
        return invalid

    reference = history.latest
    measured = history.at(stamp)
    if (
        reference is None
        or measured is None
        or measured.epoch != reference.epoch
        or not 0 <= now - reference.stamp <= max_reference_lag
        or stamp > reference.stamp
    ):
        return DepthCandidateResult("depth_motion_unaligned")
    uncertainty = history.interval_uncertainty(stamp, reference.stamp)
    interpolation = history.interpolation_horizon(stamp)
    if uncertainty is None or interpolation is None:
        return DepthCandidateResult("depth_motion_unaligned")
    transform = relative_pose(reference.pose, measured.pose)
    aligned = []
    for x, y, z in points:
        point = compose_pose(transform, Pose2D(x, y, 0.0))
        aligned.append((point.x, point.y, z))
    margin = (
        extra_margin
        + error
        + 3 * (uncertainty[0] + radius * uncertainty[1])
        + (linear_bound + radius * yaw_bound) * (now - reference.stamp + interpolation)
    )
    xmin = min(p[0] for p in footprint) - margin
    xmax = max(p[0] for p in footprint) + margin
    ymin = min(p[1] for p in footprint) - margin
    ymax = max(p[1] for p in footprint) + margin
    hit = any(
        xmin <= x <= xmax
        and ymin <= y <= ymax
        and collision_z[0] - error <= z <= collision_z[1] + error
        for x, y, z in aligned
    )
    return DepthCandidateResult("depth_hit" if hit else "depth_coverage_unknown", tuple(aligned))
