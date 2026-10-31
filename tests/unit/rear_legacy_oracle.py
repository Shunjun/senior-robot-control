"""Frozen pre-optimization C12 geometry oracle for safety equivalence tests."""

from math import atan2, cos, fsum, isfinite, pi, sin
from typing import cast

from shapely.geometry import LineString, Point, Polygon
from shapely.geometry.base import BaseGeometry
from shapely.ops import unary_union

from senior_control.config import DeviceProfile
from senior_control.safety.coverage import SweptRegion
from senior_control.safety.self_filter import (
    FilteredObservation,
    Point2D,
    RayEvidence,
    ScanObservation,
)


def legacy_filter_self(
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


def legacy_visible_areas(
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
        fans = legacy_exact_partial_fans(pairs, masks, wraps) if fan_eligible else None
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
            if (
                triangle.is_valid
                and triangle.area > 0
                and not any(triangle.intersects(mask) for mask in masks)
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


def legacy_exact_partial_fans(
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
    accepted: list[bool] = []
    angles: list[float] = []
    for a, b in pairs:
        if a.kind not in ("hit", "clear") or b.kind not in ("hit", "clear"):
            accepted.append(False)
            angles.append(0.0)
            continue
        assert a.endpoint is not None and b.endpoint is not None
        triangle = Polygon((origin, a.endpoint, b.endpoint))
        keep = (
            triangle.is_valid
            and triangle.area > 0
            and not any(triangle.intersects(mask) for mask in masks)
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
