"""Independent gate from a commanded velocity to a hard-sweep decision.

Stopping distances come only from explicit profile fields. Unset braking,
reaction, hard margin, or staleness is not a vehicle guarantee and stops motion.
"""

from dataclasses import dataclass

from shapely.geometry import LineString, MultiPolygon, Point, Polygon
from shapely.geometry.base import BaseGeometry
from shapely.ops import split

from senior_control.config import DeviceProfile
from senior_control.models import Velocity
from senior_control.safety.coverage import SweptRegion, supports
from senior_control.safety.geometry import swept_footprint
from senior_control.safety.self_filter import FilteredObservation

Point2D = tuple[float, float]


@dataclass(frozen=True)
class SafetyDecision:
    allowed: bool
    velocity: Velocity
    reason: str


def _stop(reason: str) -> SafetyDecision:
    return SafetyDecision(False, Velocity(0.0, 0.0), reason)


def stopping_distance(speed: float, reaction_time: float, braking_deceleration: float) -> float:
    """Reaction plus ``v^2 / 2a`` for a non-negative speed. Not a vehicle guarantee."""
    return speed * reaction_time + speed * speed / (2.0 * braking_deceleration)


def _directional_distances(
    command: float, measured: float, reaction_time: float, braking_deceleration: float
) -> tuple[float, float]:
    """Return ``(positive, negative)`` travel using the larger speed on each axis."""
    if command * measured < 0.0:
        positive = (
            stopping_distance(abs(command), reaction_time, braking_deceleration)
            if command > 0.0
            else 0.0
        )
        negative = (
            stopping_distance(abs(command), reaction_time, braking_deceleration)
            if command < 0.0
            else 0.0
        )
        if measured > 0.0:
            positive = max(
                positive, stopping_distance(abs(measured), reaction_time, braking_deceleration)
            )
        elif measured < 0.0:
            negative = max(
                negative, stopping_distance(abs(measured), reaction_time, braking_deceleration)
            )
        return positive, negative
    speed = max(abs(command), abs(measured))
    if speed == 0.0:
        return 0.0, 0.0
    distance = stopping_distance(speed, reaction_time, braking_deceleration)
    if command > 0.0 or (command == 0.0 and measured > 0.0):
        return distance, 0.0
    return 0.0, distance


def _rectangles(
    xmin: float,
    ymin: float,
    xmax: float,
    ymax: float,
    forward: float,
    backward: float,
    margin: float,
) -> list[tuple[Point2D, ...]]:
    """Newly entered area around the current rectangle, split so the body is excluded."""
    x_back = xmin - backward - margin
    x_front = xmax + forward + margin
    y_low = ymin - margin
    y_high = ymax + margin
    pieces: list[tuple[Point2D, ...]] = []
    if x_front > xmax:
        pieces.append(((xmax, y_low), (x_front, y_low), (x_front, y_high), (xmax, y_high)))
    if x_back < xmin:
        pieces.append(((x_back, y_low), (xmin, y_low), (xmin, y_high), (x_back, y_high)))
    if y_high > ymax:
        pieces.append(((xmin, ymax), (xmax, ymax), (xmax, y_high), (xmin, y_high)))
    if y_low < ymin:
        pieces.append(((xmin, y_low), (xmax, y_low), (xmax, ymin), (xmin, ymin)))
    return pieces


def _simple_pieces(geometry: BaseGeometry) -> list[Polygon] | None:
    """Open a body-shaped hole with a cut through the origin so coverage can see the ring."""
    if geometry.is_empty:
        return []
    if isinstance(geometry, Polygon):
        polygons = [geometry]
    elif isinstance(geometry, MultiPolygon):
        polygons = list(geometry.geoms)
    else:
        return None
    simple: list[Polygon] = []
    for polygon in polygons:
        if polygon.area <= 1e-9:
            continue
        if not polygon.interiors:
            simple.append(polygon)
            continue
        opened = [
            part
            for part in split(polygon, LineString(((-100.0, 0.0), (100.0, 0.0)))).geoms
            if isinstance(part, Polygon) and part.area > 1e-9
        ]
        if not opened or any(part.interiors for part in opened):
            return None
        simple.extend(opened)
    return simple


def _bounds(footprint: list[tuple[float, float]]) -> tuple[float, float, float, float]:
    xs = [point[0] for point in footprint]
    ys = [point[1] for point in footprint]
    return min(xs), min(ys), max(xs), max(ys)


def _require(
    vertices: tuple[Point2D, ...],
    frame: str,
    stamp: float,
    observations: list[FilteredObservation],
    now: float,
    clock: str,
    max_age: float,
) -> SafetyDecision | None:
    if len(vertices) < 3:
        return _stop("coverage_rejected")
    try:
        result = supports(SweptRegion(vertices, frame, stamp), observations, now, clock, max_age)
    except ValueError:
        return _stop("coverage_rejected")
    if not result.supported:
        return _stop(result.reason)
    return None


def evaluate_motion(
    command: Velocity,
    measured: Velocity,
    observations: list[FilteredObservation],
    profile: DeviceProfile,
    now: float,
) -> SafetyDecision:
    braking = profile.braking_deceleration
    reaction = profile.reaction_time
    margin = profile.hard_margin
    max_age = profile.observation_max_age
    if braking is None or reaction is None or margin is None or max_age is None or braking <= 0:
        return _stop("safety_limits_unset")
    yaw_braking = profile.yaw_braking_deceleration
    if (command.wz != 0.0 or measured.wz != 0.0) and (yaw_braking is None or yaw_braking <= 0):
        return _stop("yaw_braking_unset")
    if len(profile.footprint) < 3:
        return _stop("footprint_missing")
    if not observations:
        return _stop("observation_missing")
    frames = {item.scan.reference_frame for item in observations}
    stamps = {item.scan.reference_stamp for item in observations}
    clocks = {item.scan.clock_domain for item in observations}
    if len(frames) != 1 or len(stamps) != 1 or len(clocks) != 1:
        return _stop("reference_mismatch")
    frame = next(iter(frames))
    stamp = next(iter(stamps))
    clock = next(iter(clocks))
    try:
        xmin, ymin, xmax, ymax = _bounds(profile.footprint)
        body = Polygon(profile.footprint)
        if not body.is_valid or body.is_empty or body.area <= 0:
            return _stop("footprint_invalid")
    except ValueError:
        return _stop("footprint_invalid")

    hits = [
        Point(ray.endpoint)
        for item in observations
        for ray in item.external_hits
        if ray.endpoint is not None
    ]
    contact = Polygon(
        (
            (xmin - margin, ymin - margin),
            (xmax + margin, ymin - margin),
            (xmax + margin, ymax + margin),
            (xmin - margin, ymax + margin),
        )
    )
    if any(contact.intersects(hit) for hit in hits):
        return _stop("hard_contact")

    forward, backward = _directional_distances(command.vx, measured.vx, reaction, braking)
    hard = Polygon(
        (
            (xmin - backward - margin, ymin - margin),
            (xmax + forward + margin, ymin - margin),
            (xmax + forward + margin, ymax + margin),
            (xmin - backward - margin, ymax + margin),
        )
    )
    if any(hard.intersects(hit) for hit in hits):
        return _stop("hard_sweep_collision")

    if yaw_braking is not None and (command.wz != 0.0 or measured.wz != 0.0):
        positive_yaw, negative_yaw = _directional_distances(
            command.wz, measured.wz, reaction, yaw_braking
        )
        for yaw in (positive_yaw, -negative_yaw):
            if yaw == 0.0:
                continue
            spun = swept_footprint(profile.footprint, yaw=yaw, yaw_step=abs(yaw), margin=margin)
            if any(spun.intersects(hit) for hit in hits):
                return _stop("hard_sweep_collision")
            pieces = _simple_pieces(spun.difference(body))
            if pieces is None:
                return _stop("coverage_rejected")
            for piece in pieces:
                coords = tuple(
                    (float(point[0]), float(point[1])) for point in piece.exterior.coords
                )
                if len(coords) > 1 and coords[0] == coords[-1]:
                    coords = coords[:-1]
                decision = _require(coords, frame, stamp, observations, now, clock, max_age)
                if decision is not None:
                    return decision

    for vertices in _rectangles(xmin, ymin, xmax, ymax, forward, backward, margin):
        try:
            region = SweptRegion(vertices, frame, stamp)
            result = supports(region, observations, now, clock, max_age)
        except ValueError:
            return _stop("coverage_rejected")
        if not result.supported:
            return _stop(result.reason)
    return SafetyDecision(True, command, "hard_sweep_clear")
