"""Explicit beam evidence in body coordinates, preserving unknown regions."""

from dataclasses import dataclass
from math import cos, isfinite, sin
from typing import Literal

from shapely.geometry import LineString, Point, Polygon

from senior_control.config import DeviceProfile
from senior_control.models import Pose2D

Point2D = tuple[float, float]


@dataclass(frozen=True)
class ScanObservation:
    ranges: tuple[float, ...]
    angle_min: float
    angle_increment: float
    range_min: float
    range_max: float
    stamp: float
    clock_domain: str
    sensor_to_body: Pose2D
    transform_stamp: float
    reference_frame: str
    reference_stamp: float
    interpolate: bool = False
    max_angular_gap: float = 0.0
    no_return_clear: bool = False
    transform_static: bool = False

    def __post_init__(self) -> None:
        values = (
            self.angle_min,
            self.angle_increment,
            self.range_min,
            self.range_max,
            self.stamp,
            self.transform_stamp,
            self.reference_stamp,
            self.max_angular_gap,
        )
        if (
            not all(isfinite(v) for v in values)
            or self.angle_increment <= 0
            or self.range_min < 0
            or self.range_max <= self.range_min
            or self.max_angular_gap < 0
            or not self.reference_frame
            or not self.clock_domain
            or not self.ranges
        ):
            raise ValueError("invalid scan geometry or clock")


@dataclass(frozen=True)
class RayEvidence:
    index: int
    origin: Point2D
    endpoint: Point2D | None
    kind: Literal["hit", "clear", "self", "ambiguous_self_overlap", "invalid"]


@dataclass(frozen=True)
class FilteredObservation:
    scan: ScanObservation
    rays: tuple[RayEvidence, ...]
    external_hits: tuple[RayEvidence, ...]
    free_rays: tuple[RayEvidence, ...]
    occluded_regions: tuple[RayEvidence, ...]
    invalid_regions: tuple[RayEvidence, ...]
    self_masks: tuple[tuple[Point2D, ...], ...]


def filter_self(
    scan: ScanObservation, profile: DeviceProfile, *, self_mask_range_uncertainty: float = 0.0
) -> FilteredObservation:
    # //Compatible: callers without an explicit range bound retain exact-mask behavior.
    if not isfinite(self_mask_range_uncertainty) or self_mask_range_uncertainty < 0:
        raise ValueError("invalid self-mask range uncertainty")
    if not profile.self_mask_calibrated:
        raise ValueError("self-mask is uncalibrated")
    masks = []
    for vertices in profile.self_mask:
        polygon = Polygon(vertices)
        if not polygon.is_valid or polygon.is_empty or polygon.area <= 0:
            raise ValueError("invalid self-mask polygon")
        masks.append(polygon)
    origin = (scan.sensor_to_body.x, scan.sensor_to_body.y)
    rays = []
    for i, distance in enumerate(scan.ranges):
        angle = scan.sensor_to_body.yaw + scan.angle_min + i * scan.angle_increment
        # A missing return is not free space, even when a caller asks to treat it as clear.
        if scan.no_return_clear and distance == float("inf"):
            rays.append(RayEvidence(i, origin, None, "invalid"))
            continue
        valid = isfinite(distance) and scan.range_min <= distance <= scan.range_max
        if not valid:
            rays.append(RayEvidence(i, origin, None, "invalid"))
            continue
        endpoint = (origin[0] + distance * cos(angle), origin[1] + distance * sin(angle))
        line = LineString((origin, endpoint))
        if any(mask.intersects(line) or mask.covers(Point(origin)) for mask in masks):
            rays.append(RayEvidence(i, origin, endpoint, "self"))
        elif self_mask_range_uncertainty and masks:
            extended_distance = distance + self_mask_range_uncertainty
            extended = (
                origin[0] + extended_distance * cos(angle),
                origin[1] + extended_distance * sin(angle),
            )
            if any(mask.intersects(LineString((origin, extended))) for mask in masks):
                rays.append(RayEvidence(i, origin, endpoint, "ambiguous_self_overlap"))
            else:
                rays.append(RayEvidence(i, origin, endpoint, "hit"))
        else:
            rays.append(RayEvidence(i, origin, endpoint, "hit"))
    evidence = tuple(rays)
    return FilteredObservation(
        scan,
        evidence,
        tuple(ray for ray in evidence if ray.kind == "hit"),
        tuple(ray for ray in evidence if ray.kind in ("hit", "clear")),
        tuple(ray for ray in evidence if ray.kind == "self"),
        tuple(ray for ray in evidence if ray.kind in ("invalid", "ambiguous_self_overlap")),
        tuple(tuple(vertices) for vertices in profile.self_mask),
    )
