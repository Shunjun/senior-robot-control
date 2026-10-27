"""Conservative whole-area observation coverage, never a motion authorization."""

from dataclasses import dataclass
from math import isfinite

from shapely.geometry import Polygon
from shapely.ops import unary_union

from senior_control.safety.self_filter import FilteredObservation, Point2D


@dataclass(frozen=True)
class SweptRegion:
    """Area requiring observation; caller excludes only known current physical occupancy."""

    vertices: tuple[Point2D, ...]
    reference_frame: str
    reference_stamp: float

    def __post_init__(self) -> None:
        if not self.reference_frame or not isfinite(self.reference_stamp):
            raise ValueError("invalid reference frame or stamp")
        if not all(isfinite(c) for p in self.vertices for c in p):
            raise ValueError("nonfinite sweep")
        polygon = Polygon(self.vertices)
        if not polygon.is_valid or polygon.is_empty or polygon.area <= 0:
            raise ValueError("invalid sweep")


@dataclass(frozen=True)
class CoverageResult:
    supported: bool
    reason: str


def supports(
    sweep: SweptRegion,
    observations: list[FilteredObservation],
    now: float,
    clock_domain: str,
    max_age: float,
) -> CoverageResult:
    if not isfinite(now) or not isfinite(max_age) or max_age < 0 or not clock_domain:
        raise ValueError("invalid time or clock domain")
    polygons: dict[float, list[Polygon]] = {}
    for observation in observations:
        scan = observation.scan
        if (
            scan.reference_frame != sweep.reference_frame
            or scan.reference_stamp != sweep.reference_stamp
            or scan.clock_domain != clock_domain
            or not 0 <= now - scan.stamp <= max_age
            or (not scan.transform_static and not 0 <= scan.stamp - scan.transform_stamp <= max_age)
            or not scan.interpolate
            or scan.angle_increment > scan.max_angular_gap
        ):
            continue
        for a, b in zip(observation.rays, observation.rays[1:], strict=False):
            if a.kind not in ("hit", "clear") or b.kind not in ("hit", "clear"):
                continue
            assert a.endpoint is not None and b.endpoint is not None
            triangle = Polygon((a.origin, a.endpoint, b.endpoint))
            if (
                triangle.is_valid
                and triangle.area > 0
                and not any(triangle.intersects(Polygon(mask)) for mask in observation.self_masks)
            ):
                polygons.setdefault(scan.stamp, []).append(triangle)
    if not polygons:
        return CoverageResult(False, "no usable observation area")
    if any(unary_union(group).covers(Polygon(sweep.vertices)) for group in polygons.values()):
        return CoverageResult(True, "entire required area observed")
    return CoverageResult(False, "required area contains unknown space")
