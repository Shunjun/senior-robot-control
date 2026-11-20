"""Startup-only front/rear angular availability policy; never measured free space."""

import math
from dataclasses import dataclass

from shapely.geometry import Polygon
from shapely.geometry.base import BaseGeometry

from senior_control.safety.self_filter import FilteredObservation
from vehicle_core.values import Pose2D

_HALF = math.pi / 6
_WIDTH = 2 * _HALF


def _union_width(intervals: list[tuple[float, float]]) -> float:
    merged: list[tuple[float, float]] = []
    for low, high in sorted(intervals):
        if merged and low <= merged[-1][1]:
            merged[-1] = merged[-1][0], max(high, merged[-1][1])
        else:
            merged.append((low, high))
    return math.fsum(high - low for low, high in merged)


@dataclass(frozen=True)
class StartupVisibility:
    sensor_origin: tuple[float, float]
    front_occluded: float
    rear_occluded: float
    front_stop: bool
    rear_stop: bool

    @property
    def blocked_reason(self) -> str | None:
        # Only binary64 arithmetic roundoff at the exact 70% boundary.
        rounding = 8 * math.ulp(1.0)
        if self.front_stop and self.front_occluded >= 0.7 - rounding:
            return "learning_front_occluded"
        if self.rear_stop and self.rear_occluded >= 0.7 - rounding:
            return "learning_rear_occluded"
        return None

    @property
    def identity(self) -> tuple[tuple[float, float], bool, bool, str | None, bool, bool]:
        # Cache identity changes only when the applicable policy changes, not
        # with harmless fractional noise below the same approved threshold.
        return (
            self.sensor_origin,
            self.front_stop,
            self.rear_stop,
            self.blocked_reason,
            self.front_occluded > 0,
            self.rear_occluded > 0,
        )

    def remaining_unknown(self, unknown: BaseGeometry, pose: Pose2D) -> BaseGeometry:
        if unknown.is_empty or self.blocked_reason is not None:
            return unknown
        # Remove only the approved directional coverage requirement. Do not
        # add these triangles to free_space or alter any observed obstacle.
        c, s = math.cos(pose.yaw), math.sin(pose.yaw)
        sx, sy = self.sensor_origin
        origin = (pose.x + c * sx - s * sy, pose.y + s * sx + c * sy)
        left, bottom, right, top = unknown.bounds
        radius = 2 * max(math.dist(origin, (x, y)) for x in (left, right) for y in (bottom, top))
        for center, fraction in ((0.0, self.front_occluded), (math.pi, self.rear_occluded)):
            if fraction == 0:
                continue
            angles = (pose.yaw + center - _HALF, pose.yaw + center + _HALF)
            far = [
                (origin[0] + radius * math.cos(a), origin[1] + radius * math.sin(a)) for a in angles
            ]
            unknown = unknown.difference(Polygon((origin, *far)))
        return unknown


def startup_visibility(
    observation: FilteredObservation,
    *,
    front_stop: bool,
    rear_stop: bool,
) -> StartupVisibility:
    """Union angular cells of usable rays; complement is unavailable coverage.

    Self returns, missing/invalid rays and unobserved FOV supply no usable cell.
    Positive infinity means no return within range and supplies angular coverage
    only, without changing measured rays or free space. Unioning cells avoids
    counting an inclusive 360-degree duplicate sample twice.
    """
    scan = observation.scan
    fractions = []
    for center in (0.0, math.pi):
        cells = []
        base = math.remainder(scan.sensor_to_body.yaw + scan.angle_min - center, math.tau)
        for ray in observation.rays:
            usable = ray.kind in {"hit", "clear"} or (
                ray.kind == "invalid" and scan.ranges[ray.index] == math.inf
            )
            if not usable:
                continue
            for turn in (-math.tau, 0.0, math.tau):
                low = max(-_HALF, base + turn + (ray.index - 0.5) * scan.angle_increment)
                high = min(_HALF, base + turn + (ray.index + 0.5) * scan.angle_increment)
                if high > low:
                    cells.append((low, high))
        fractions.append(max(0.0, min(1.0, 1 - _union_width(cells) / _WIDTH)))
    return StartupVisibility(
        (scan.sensor_to_body.x, scan.sensor_to_body.y),
        fractions[0],
        fractions[1],
        front_stop,
        rear_stop,
    )
