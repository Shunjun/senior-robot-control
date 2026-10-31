"""Conservative whole-area observation coverage, never a motion authorization."""

from dataclasses import dataclass
from math import atan2, cos, fsum, isfinite, pi
from typing import cast

from shapely.geometry import Point, Polygon
from shapely.geometry.base import BaseGeometry
from shapely.ops import unary_union

from senior_control.safety.geometry_bounds import mask_bounds, strictly_disjoint, triangle_bounds
from senior_control.safety.self_filter import FilteredObservation, Point2D, RayEvidence


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
    *,
    diagnostics: dict[str, object] | None = None,
    cache: dict[tuple[object, ...], tuple[BaseGeometry, ...]] | None = None,
) -> CoverageResult:
    if not isfinite(now) or not isfinite(max_age) or max_age < 0 or not clock_domain:
        raise ValueError("invalid time or clock domain")
    key = (
        sweep.reference_frame,
        sweep.reference_stamp,
        now,
        clock_domain,
        max_age,
        tuple(observations),
    )
    cached = cache.get(key) if cache is not None else None
    if cached is None:
        cached = _visible_areas(sweep, observations, now, clock_domain, max_age)
        if cache is not None:
            cache[key] = cached
    required = Polygon(sweep.vertices)
    unknown: BaseGeometry = required
    for visible in cached:
        if visible.covers(required):
            return CoverageResult(True, "entire required area observed")
        if diagnostics is not None:
            difference = required.difference(visible)
            if difference.area < unknown.area:
                unknown = difference
    if diagnostics is not None:
        diagnostics.clear()
        diagnostics.update(
            required_wkt=required.wkt,
            unknown_wkt=unknown.wkt,
            unknown_area=unknown.area,
            unknown_bounds=unknown.bounds,
        )
    reason = "required area contains unknown space" if cached else "no usable observation area"
    return CoverageResult(False, reason)


def _visible_areas(
    sweep: SweptRegion,
    observations: list[FilteredObservation],
    now: float,
    clock_domain: str,
    max_age: float,
) -> tuple[BaseGeometry, ...]:
    polygons: dict[float, list[BaseGeometry]] = {}
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
        pairs = list(zip(observation.rays, observation.rays[1:], strict=False))
        span = (len(scan.ranges) - 1) * scan.angle_increment
        wrap_gap = 2 * pi - span
        wraps = (
            len(observation.rays) == len(scan.ranges)
            and len(observation.rays) > 1
            and observation.rays[0].index == 0
            and observation.rays[-1].index == len(scan.ranges) - 1
            and 0 < wrap_gap <= scan.max_angular_gap
            and wrap_gap <= 1.001 * scan.angle_increment
        )
        if wraps:
            # The two measured endpoint beams are adjacent only when the
            # actual missing angular interval meets the calibrated gap bound
            # and is no wider than one sample spacing (plus metadata rounding).
            # Their usual validity and self-mask checks below still apply.
            pairs.append((observation.rays[-1], observation.rays[0]))
        masks = [Polygon(mask) for mask in observation.self_masks]
        indexed_masks = [(mask, mask_bounds(mask)) for mask in masks]
        if (
            wraps
            and not masks
            and scan.angle_increment < pi
            and all(
                ray.kind in ("hit", "clear")
                and ray.endpoint is not None
                and ray.origin == observation.rays[0].origin
                for ray in observation.rays
            )
        ):
            # For a complete single-origin ordered fan, the endpoint ring is
            # exactly the union of its adjacent triangles. Validate topology;
            # ambiguous shapes fall back to the individual-triangle path.
            ring: BaseGeometry = Polygon(
                cast(list[Point2D], [ray.endpoint for ray in observation.rays])
            )
            if (
                ring.is_valid
                and not ring.is_empty
                and ring.area > 0
                and ring.covers(Point(observation.rays[0].origin))
            ):
                if scan.range_min > 0:
                    blind = Point(observation.rays[0].origin).buffer(
                        scan.range_min / cos(pi / 64), quad_segs=16
                    )
                    ring = ring.difference(blind)
                if not ring.is_empty:
                    polygons.setdefault(scan.stamp, []).append(ring)
                continue
        fan_eligible = (
            0 < scan.angle_increment < pi
            and 0 < span < 2 * pi
            and len(observation.rays) == len(scan.ranges)
            and all(ray.index == i for i, ray in enumerate(observation.rays))
        )
        fans = _exact_partial_fans(pairs, masks, wraps) if fan_eligible else None
        if fans is not None:
            if fans:
                origin = observation.rays[0].origin
                visible = fans[0] if len(fans) == 1 else unary_union(fans)
                if scan.range_min > 0:
                    blind = Point(origin).buffer(scan.range_min / cos(pi / 64), quad_segs=16)
                    visible = visible.difference(blind)
                if not visible.is_empty:
                    polygons.setdefault(scan.stamp, []).append(visible)
            continue
        triangles_by_origin: dict[tuple[float, float], list[BaseGeometry]] = {}
        for a, b in pairs:
            if a.kind not in ("hit", "clear") or b.kind not in ("hit", "clear"):
                continue
            assert a.endpoint is not None and b.endpoint is not None
            triangle = Polygon((a.origin, a.endpoint, b.endpoint))
            valid = triangle.is_valid
            area = triangle.area if valid else 0.0
            bounds = triangle_bounds((a.origin, a.endpoint, b.endpoint), area)
            if (
                valid
                and area > 0
                and not any(
                    not strictly_disjoint(bounds, mask_box) and triangle.intersects(mask)
                    for mask, mask_box in indexed_masks
                )
            ):
                triangles_by_origin.setdefault(a.origin, []).append(triangle)
        for origin, triangles in triangles_by_origin.items():
            visible = unary_union(triangles)
            if scan.range_min > 0:
                # Union and set difference distribute. Removing the same blind
                # disk once preserves the exact area without 720 repetitions.
                blind = Point(origin).buffer(scan.range_min / cos(pi / 64), quad_segs=16)
                visible = visible.difference(blind)
            if not visible.is_empty:
                polygons.setdefault(scan.stamp, []).append(visible)
    return tuple(group[0] if len(group) == 1 else unary_union(group) for group in polygons.values())


def _exact_partial_fans(
    pairs: list[tuple[RayEvidence, RayEvidence]], masks: list[Polygon], wraps: bool
) -> list[BaseGeometry] | None:
    """Replace only proven contiguous triangle fans with their exact boundary.

    Each accepted triangle keeps the original ray-kind and mask-intersection
    tests. A common origin, strict counterclockwise winding, less than one
    revolution and a valid polygon establish the same union; gaps split runs.
    Any unproven case returns None for the original triangle implementation.
    """
    if not pairs:
        return []
    origin = pairs[0][0].origin
    if any(a.origin != origin or b.origin != origin for a, b in pairs):
        return None
    indexed_masks = [(mask, mask_bounds(mask)) for mask in masks]
    accepted: list[bool] = []
    angles: list[float] = []
    for a, b in pairs:
        if a.kind not in ("hit", "clear") or b.kind not in ("hit", "clear"):
            accepted.append(False)
            angles.append(0.0)
            continue
        assert a.endpoint is not None and b.endpoint is not None
        triangle = Polygon((origin, a.endpoint, b.endpoint))
        valid = triangle.is_valid
        area = triangle.area if valid else 0.0
        bounds = triangle_bounds((origin, a.endpoint, b.endpoint), area)
        keep = (
            valid
            and area > 0
            and not any(
                not strictly_disjoint(bounds, mask_box) and triangle.intersects(mask)
                for mask, mask_box in indexed_masks
            )
        )
        if keep:
            ax, ay = a.endpoint[0] - origin[0], a.endpoint[1] - origin[1]
            bx, by = b.endpoint[0] - origin[0], b.endpoint[1] - origin[1]
            cross = ax * by - ay * bx
            dot = ax * bx + ay * by
            if cross <= 0:
                return None
            angle = atan2(cross, dot)
            if not 0 < angle < pi:
                return None
        else:
            angle = 0.0
        accepted.append(keep)
        angles.append(angle)
    if not any(accepted):
        return []
    if wraps:
        if all(accepted):
            return None
        offset = (accepted.index(False) + 1) % len(accepted)
        pairs = pairs[offset:] + pairs[:offset]
        accepted = accepted[offset:] + accepted[:offset]
        angles = angles[offset:] + angles[:offset]
    result: list[BaseGeometry] = []
    i = 0
    while i < len(pairs):
        if not accepted[i]:
            i += 1
            continue
        end = i
        while end + 1 < len(pairs) and accepted[end + 1]:
            end += 1
        if fsum(angles[i : end + 1]) >= 2 * pi:
            return None
        first = pairs[i][0].endpoint
        if first is None:
            return None
        vertices: list[Point2D] = [first]
        for _, ray in pairs[i : end + 1]:
            if ray.endpoint is None:
                return None
            vertices.append(ray.endpoint)
        polygon = Polygon([origin, *vertices])
        if not polygon.is_valid or polygon.is_empty or polygon.area <= 0:
            return None
        result.append(polygon)
        i = end + 1
    return result
