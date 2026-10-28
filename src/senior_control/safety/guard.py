"""Independent gate from a commanded velocity to a hard-sweep decision.

Stopping distances come only from explicit profile fields. Unset braking,
reaction, hard margin, or staleness is not a vehicle guarantee and stops motion.
The hard sweep is the body travel during that stop. The warning zone extends
the same stopping distance in both longitudinal directions and does not veto a
hard-clear command.
"""

import math
from dataclasses import dataclass

from shapely.geometry import LineString, MultiPolygon, Point, Polygon
from shapely.geometry.base import BaseGeometry
from shapely.ops import split

from senior_control.config import DeviceProfile
from senior_control.models import Velocity
from senior_control.safety.coverage import SweptRegion, supports
from senior_control.safety.geometry import swept_footprint, swept_trajectory
from senior_control.safety.self_filter import FilteredObservation

Point2D = tuple[float, float]


@dataclass(frozen=True)
class SafetyDecision:
    allowed: bool
    velocity: Velocity
    reason: str


def _stop(reason: str) -> SafetyDecision:
    return SafetyDecision(False, Velocity(0.0, 0.0), reason)


def command_expired(command_stamp: float | None, now: float, ttl: float | None) -> bool:
    """True when the last accepted command is older than an explicit TTL.

    A missing stamp, a missing TTL, or a non-positive TTL fails closed. The
    caller measures ``command_stamp`` and ``now`` on one monotonic clock.
    """
    if (
        command_stamp is None
        or ttl is None
        or not math.isfinite(command_stamp)
        or not math.isfinite(now)
        or not math.isfinite(ttl)
        or ttl <= 0.0
        or now < command_stamp
    ):
        return True
    return now - command_stamp > ttl


def _rotation_step(footprint: list[tuple[float, float]], margin: float) -> float:
    """Step whose corner gap stays within the hard margin, or 5 mm in simulation."""
    radius = max(math.hypot(point[0], point[1]) for point in footprint)
    allowance = margin if margin > 0.0 else 0.005
    if radius <= 1e-9:
        return math.pi / 16.0
    ratio = min(1.0, allowance / (2.0 * radius))
    return 4.0 * math.asin(ratio)


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


def _warning_reach(command: float, measured: float, reaction_time: float, braking: float) -> float:
    """Stopping distance of the faster speed, used on both sides of the body."""
    speed = max(abs(command), abs(measured))
    if speed == 0.0:
        return 0.0
    return stopping_distance(speed, reaction_time, braking)


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
    warning_reach = _warning_reach(command.vx, measured.vx, reaction, braking)
    hard = Polygon(
        (
            (xmin - backward - margin, ymin - margin),
            (xmax + forward + margin, ymin - margin),
            (xmax + forward + margin, ymax + margin),
            (xmin - backward - margin, ymax + margin),
        )
    )
    warning = Polygon(
        (
            (xmin - warning_reach - margin, ymin - margin),
            (xmax + warning_reach + margin, ymin - margin),
            (xmax + warning_reach + margin, ymax + margin),
            (xmin - warning_reach - margin, ymax + margin),
        )
    )
    positive_yaw = 0.0
    negative_yaw = 0.0
    if yaw_braking is not None and (command.wz != 0.0 or measured.wz != 0.0):
        positive_yaw, negative_yaw = _directional_distances(
            command.wz, measured.wz, reaction, yaw_braking
        )
    step = _rotation_step(profile.footprint, margin)
    spins: list[BaseGeometry] = []
    for yaw in (positive_yaw, -negative_yaw):
        if yaw == 0.0:
            continue
        spins.append(
            swept_footprint(
                profile.footprint,
                yaw=yaw,
                yaw_step=min(step, abs(yaw)),
                margin=margin,
            )
        )
    arcs: list[BaseGeometry] = []
    for distance, yaw in _coupled_arcs(
        command, measured, forward, backward, positive_yaw, negative_yaw
    ):
        arcs.append(
            swept_trajectory(
                profile.footprint,
                distance=distance,
                yaw=yaw,
                step=min(step, abs(yaw)),
                margin=margin,
            )
        )
    hard_geoms: list[BaseGeometry] = [hard, *spins, *arcs]
    if any(geom.intersects(hit) for geom in hard_geoms for hit in hits):
        return _stop("hard_sweep_collision")
    warning_overlap = any(warning.intersects(hit) for hit in hits)

    for spun in spins:
        decision = _cover_difference(spun, body, frame, stamp, observations, now, clock, max_age)
        if decision is not None:
            return decision
    for arc in arcs:
        decision = _cover_difference(arc, body, frame, stamp, observations, now, clock, max_age)
        if decision is not None:
            return decision
    for vertices in _rectangles(xmin, ymin, xmax, ymax, forward, backward, margin):
        decision = _require(vertices, frame, stamp, observations, now, clock, max_age)
        if decision is not None:
            return decision
    if warning_overlap:
        return SafetyDecision(True, command, "warning_overlap")
    return SafetyDecision(True, command, "hard_sweep_clear")


def _coupled_arcs(
    command: Velocity,
    measured: Velocity,
    forward: float,
    backward: float,
    positive_yaw: float,
    negative_yaw: float,
) -> list[tuple[float, float]]:
    """Pair each signed stop that has both translation and yaw. Pure axes stay separate."""
    arcs: list[tuple[float, float]] = []
    for linear, angular in ((command.vx, command.wz), (measured.vx, measured.wz)):
        if linear == 0.0 or angular == 0.0:
            continue
        distance = forward if linear > 0.0 else -backward
        yaw = positive_yaw if angular > 0.0 else -negative_yaw
        pair = (distance, yaw)
        if distance != 0.0 and yaw != 0.0 and pair not in arcs:
            arcs.append(pair)
    return arcs


def _cover_difference(
    swept: BaseGeometry,
    body: Polygon,
    frame: str,
    stamp: float,
    observations: list[FilteredObservation],
    now: float,
    clock: str,
    max_age: float,
) -> SafetyDecision | None:
    pieces = _simple_pieces(swept.difference(body))
    if pieces is None:
        return _stop("coverage_rejected")
    for piece in pieces:
        coords = tuple((float(point[0]), float(point[1])) for point in piece.exterior.coords)
        if len(coords) > 1 and coords[0] == coords[-1]:
            coords = coords[:-1]
        decision = _require(coords, frame, stamp, observations, now, clock, max_age)
        if decision is not None:
            return decision
    return None
