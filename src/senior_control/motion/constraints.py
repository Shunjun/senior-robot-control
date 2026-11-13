"""Pure bounded reachability certificates; no actuator, authority or task side effects.

The caller supplies independently supported mode/scene envelopes. Statistical
covariance and wheel references alone never establish these physical bounds.
"""

import math
from dataclasses import dataclass
from enum import Enum

from shapely.affinity import rotate, translate
from shapely.geometry import GeometryCollection, MultiPoint, Polygon, box
from shapely.geometry.base import BaseGeometry
from shapely.ops import unary_union

from senior_control.models import Pose2D, Velocity
from senior_control.motion.contracts import (
    BodyMotion,
    BrakeKind,
    BrakeRequest,
    MotionPermit,
    MotionTarget,
    ResponseBounds,
)
from senior_control.motion.source_modes import SourceMode
from senior_control.safety.geometry import swept_footprint

Point = tuple[float, float]


class PatchKind(str, Enum):
    OBSERVED_FREE = "observed_free"
    NO_RETURN = "no_return"


class NoReturnPolicy(str, Enum):
    UNKNOWN = "unknown"
    VALIDATED_FREE_TO_RANGE = "validated_free_to_range"


@dataclass(frozen=True)
class FreePatch:
    vertices: tuple[Point, ...]
    kind: PatchKind


@dataclass(frozen=True)
class Perception:
    free_space: BaseGeometry
    occupied: BaseGeometry
    stamp_steady: float
    deadline_steady: float
    source_epoch: str
    frame_id: str
    revision: int
    forecast_horizon_s: float
    basis: str


def prepare_perception(
    patches: tuple[FreePatch, ...],
    hits: tuple[Point, ...],
    *,
    error_m: float,
    stamp_steady: float,
    deadline_steady: float,
    source_epoch: str,
    frame_id: str,
    revision: int,
    forecast_horizon_s: float,
    no_return_policy: NoReturnPolicy,
    basis: str,
    max_patches: int,
    max_vertices: int,
    max_hits: int,
) -> Perception:
    """Bounded preprocessing, outside the control fast path.

    Free patches are convex observed areas, already transformed into the shared
    local frame. For moving scenes the supplied geometry must enclose obstacle
    reachability over forecast_horizon_s. Unknown/no-return is never implicit free.
    Sensor position error is charged here once, not added to the body again.
    """
    if (
        len(patches) > max_patches
        or len(hits) > max_hits
        or sum(len(p.vertices) for p in patches) > max_vertices
    ):
        raise ValueError("observation complexity budget")
    if (
        not all(
            math.isfinite(x) for x in (error_m, stamp_steady, deadline_steady, forecast_horizon_s)
        )
        or error_m < 0
        or deadline_steady <= stamp_steady
        or forecast_horizon_s <= 0
        or not isinstance(no_return_policy, NoReturnPolicy)
        or type(revision) is not int
        or revision < 0
        or not source_epoch
        or not frame_id
        or not basis
    ):
        raise ValueError("invalid observation evidence")
    free = []
    for patch in patches:
        if not isinstance(patch.kind, PatchKind):
            raise ValueError("invalid free-patch meaning")
        if not all(math.isfinite(v) for point in patch.vertices for v in point):
            raise ValueError("nonfinite free patch")
        polygon = Polygon(patch.vertices)
        if not polygon.is_valid or polygon.area <= 0 or not polygon.equals(polygon.convex_hull):
            raise ValueError("free patches must be valid convex polygons")
        if patch.kind is PatchKind.NO_RETURN and no_return_policy is NoReturnPolicy.UNKNOWN:
            continue
        free.append(polygon.buffer(-error_m, join_style="mitre") if error_m else polygon)
    if not all(math.isfinite(v) for point in hits for v in point):
        raise ValueError("nonfinite hit")
    # Squares circumscribe Euclidean error disks; never use an inscribed circle.
    occupied = (
        unary_union([box(x - error_m, y - error_m, x + error_m, y + error_m) for x, y in hits])
        if error_m and hits
        else MultiPoint(hits)
        if hits
        else GeometryCollection()
    )
    return Perception(
        unary_union(free),
        occupied,
        stamp_steady,
        deadline_steady,
        source_epoch,
        frame_id,
        revision,
        forecast_horizon_s,
        basis,
    )


@dataclass(frozen=True)
class Footprint:
    length_m: float
    width_m: float

    def __post_init__(self) -> None:
        if not all(math.isfinite(x) and x > 0 for x in (self.length_m, self.width_m)):
            raise ValueError("invalid body geometry")

    @property
    def physical_vertices(self) -> tuple[Point, ...]:
        x, y = self.length_m / 2, self.width_m / 2
        return ((-x, -y), (x, -y), (x, y), (-x, y))

    @property
    def vertices(self) -> tuple[Point, ...]:
        x, y = self.length_m * 1.05 / 2, self.width_m * 1.05 / 2
        return ((-x, -y), (x, -y), (x, y), (-x, y))


@dataclass(frozen=True)
class RiskEnvelope:
    scope_id: str
    basis: str
    motion_epoch: str
    modes: tuple[SourceMode, ...]
    response_profile: str
    valid_until_steady: float
    max_horizon_s: float
    linear_interval: tuple[float, float]
    yaw_interval: tuple[float, float]
    reaction_accel_upper: Velocity
    position_error_m: float  # whole-horizon tracking/localization error, excluding sensor error
    yaw_error_rad: float
    lateral_drift_mps: float  # independently bounded lateral slip throughout the horizon
    linear_cap_mps: float
    yaw_cap_rps: float
    synthetic: bool
    # //Compatible: nonzero lateral drift requires an independent settling guarantee.
    lateral_stop_s: float | None = None

    def __post_init__(self) -> None:
        numbers = (
            self.valid_until_steady,
            self.max_horizon_s,
            *self.linear_interval,
            *self.yaw_interval,
            self.reaction_accel_upper.vx,
            self.reaction_accel_upper.wz,
            self.position_error_m,
            self.yaw_error_rad,
            self.lateral_drift_mps,
            self.linear_cap_mps,
            self.yaw_cap_rps,
        )
        if (
            not all(math.isfinite(x) for x in numbers)
            or self.max_horizon_s <= 0
            or self.linear_interval[0] > self.linear_interval[1]
            or self.yaw_interval[0] > self.yaw_interval[1]
            or min(numbers[6:]) < 0
            or not self.scope_id
            or not self.basis
            or not self.motion_epoch
            or not self.response_profile
            or not self.modes
        ):
            raise ValueError("invalid scoped risk envelope")
        if self.lateral_stop_s is not None and (
            not math.isfinite(self.lateral_stop_s) or self.lateral_stop_s < 0
        ):
            raise ValueError("invalid lateral settling guarantee")


@dataclass(frozen=True)
class ConstraintSnapshot:
    target: MotionTarget
    motion: BodyMotion
    mode: SourceMode
    response: ResponseBounds
    footprint: Footprint
    perception: Perception
    pose: Pose2D
    frame_id: str
    scope_id: str
    config_version: str
    envelope: RiskEnvelope | None


@dataclass(frozen=True)
class ConstraintConfig:
    fractions: tuple[float, ...]
    lease_s: float
    yaw_step_rad: float
    max_yaw_steps: int
    sweep_error_m: float

    def __post_init__(self) -> None:
        if (
            not self.fractions
            or len(self.fractions) > 16
            or not all(math.isfinite(x) and 0 < x <= 1 for x in self.fractions)
            or len(set(self.fractions)) != len(self.fractions)
            or not all(
                math.isfinite(x) and x > 0
                for x in (self.lease_s, self.yaw_step_rad, self.sweep_error_m)
            )
            or self.max_yaw_steps < 1
        ):
            raise ValueError("invalid finite candidate/geometry budget")


@dataclass(frozen=True)
class StopPhase:
    duration: float
    jerk: float
    end_velocity: float
    end_acceleration: float
    distance: float


@dataclass(frozen=True)
class AxisStop:
    distance: float
    duration: float
    phases: tuple[StopPhase, ...]


def stop_axis(speed: float, accel_upper: float, decel: float, jerk: float, lag: float) -> AxisStop:
    """Exact positive-speed S-curve upper profile, including reaction and jerk release.

    Initial acceleration is its independently supported nonnegative upper bound.
    Response profile must guarantee this braking family; these are not measured maxima.
    """
    if (
        not all(math.isfinite(x) for x in (speed, accel_upper, decel, jerk, lag))
        or min(speed, accel_upper, lag) < 0
        or min(decel, jerk) <= 0
    ):
        raise ValueError("invalid stopping model")
    if speed == 0 and accel_upper == 0:
        return AxisStop(0, 0, ())
    velocity, accel = speed, accel_upper
    distance = 0.0
    phases = []

    def phase(duration: float, j: float) -> None:
        nonlocal velocity, accel, distance
        if duration <= 0:
            return
        # Multiply coefficients first: a finite long constant-deceleration
        # plateau must not overflow merely from squaring/cubing its duration.
        travel = (
            velocity * duration
            + (accel * duration) * duration / 2
            + ((j * duration) * duration) * duration / 6
        )
        velocity += accel * duration + (j * duration) * duration / 2
        accel += j * duration
        distance += travel
        if not all(math.isfinite(x) for x in (travel, velocity, accel, distance)):
            raise ValueError("stop_model_numeric_range")
        phases.append(StopPhase(duration, j, velocity, accel, travel))

    phase(lag, 0)
    try:
        peak = min(decel, math.sqrt(jerk * velocity + accel**2 / 2))
        if not math.isfinite(peak) or peak <= 0:
            raise ValueError("stop_model_numeric_range")
        plateau = max(0.0, (velocity + accel**2 / (2 * jerk) - peak**2 / jerk) / peak)
    except OverflowError as error:
        raise ValueError("stop_model_numeric_range") from error
    phase((accel + peak) / jerk, -jerk)
    phase(plateau, 0)
    phase(peak / jerk, jerk)
    return AxisStop(distance, sum(p.duration for p in phases), tuple(phases))


@dataclass(frozen=True)
class SweepCertificate:
    velocity: Velocity
    sweep: BaseGeometry  # padded collision/stand-off envelope, body-relative
    physical_sweep: BaseGeometry  # never treat padding as known occupied/free
    linear_domain: tuple[float, float]
    yaw_domain: tuple[float, float]
    horizon_s: float
    risk: RiskEnvelope
    response_key: tuple[float, ...]


@dataclass(frozen=True)
class ConstraintResult:
    permit: MotionPermit | None
    brake: BrakeRequest | None
    certificates: tuple[SweepCertificate, ...] = ()


def _response_key(r: ResponseBounds) -> tuple[float, ...]:
    return (r.accel, r.decel, r.jerk, r.yaw_accel, r.yaw_decel, r.yaw_jerk, r.actuator_lag)


def _identity(s: ConstraintSnapshot) -> tuple[str, ...]:
    return (
        s.motion.source_epoch,
        s.perception.source_epoch,
        s.frame_id,
        s.scope_id,
        s.config_version,
        str(s.footprint.length_m),
        str(s.footprint.width_m),
        s.response.profile_id,
        s.target.source,
        s.target.source_id,
    )


def _world(geometry: BaseGeometry, pose: Pose2D) -> BaseGeometry:
    return translate(rotate(geometry, pose.yaw, origin=(0, 0), use_radians=True), pose.x, pose.y)


def _trig_range(low: float, high: float, sine: bool) -> tuple[float, float]:
    f = math.sin if sine else math.cos
    if high - low >= 2 * math.pi:
        return (-1.0, 1.0)
    values = [f(low), f(high)]
    offset = math.pi / 2 if sine else 0.0
    for k in range(math.ceil((low - offset) / math.pi), math.floor((high - offset) / math.pi) + 1):
        values.append(f(offset + k * math.pi))
    return min(values), max(values)


def _reachable_shape(
    vertices: tuple[Point, ...],
    low: float,
    high: float,
    travel: float,
    signs: tuple[float, ...],
    config: ConstraintConfig,
    *,
    coarse: bool,
) -> BaseGeometry:
    """Minkowski enclosure of body rotation and signed translation intervals."""
    body = Polygon(vertices)
    rotation: BaseGeometry
    if low == high:
        rotation = body
    elif coarse:
        corners: list[Point] = []
        for x, y in vertices:
            radius = math.hypot(x, y)
            angle = math.atan2(y, x)
            c = _trig_range(low + angle, high + angle, False)
            sn = _trig_range(low + angle, high + angle, True)
            corners.extend(((radius * c[0], radius * sn[0]), (radius * c[1], radius * sn[1])))
        rotation = MultiPoint(corners).envelope
    else:
        if math.ceil((high - low) / config.yaw_step_rad) > config.max_yaw_steps:
            raise ValueError("geometry_budget_exceeded")
        rotated = tuple(
            (x * math.cos(low) - y * math.sin(low), x * math.sin(low) + y * math.cos(low))
            for x, y in vertices
        )
        rotation = swept_footprint(
            rotated,
            yaw=high - low,
            yaw_step=config.yaw_step_rad,
            margin=config.sweep_error_m,
        ).convex_hull
    c = _trig_range(low, high, False)
    sn = _trig_range(low, high, True)
    # Integrating signed velocities over any bounded heading history is enclosed
    # by these translation intervals; never sample only arc endpoints/corners.
    xs = [0.0, *(sign * travel * t for sign in signs for t in c)]
    ys = [0.0, *(sign * travel * t for sign in signs for t in sn)]
    shifts = ((min(xs), min(ys)), (min(xs), max(ys)), (max(xs), min(ys)), (max(xs), max(ys)))
    sweep = unary_union([translate(rotation, x, y) for x, y in shifts]).convex_hull
    return sweep


class ConstraintEngine:
    def __init__(self, config: ConstraintConfig):
        self.config = config

    def _context_error(self, s: ConstraintSnapshot, now: float) -> str | None:
        if not math.isfinite(now):
            return "invalid_clock"
        if (
            not s.motion.computed_steady <= now < s.motion.valid_until_steady
            or not s.motion.qualified
        ):
            return "motion_expired_or_unqualified"
        if not s.perception.stamp_steady <= now < s.perception.deadline_steady:
            return "perception_expired"
        r = s.response
        if (
            not all(math.isfinite(x) and x > 0 for x in _response_key(r)[:-1])
            or not math.isfinite(r.actuator_lag)
            or r.actuator_lag < 0
        ):
            return "invalid_response"
        if not now < r.valid_until_steady:
            return "response_expired"
        e = s.envelope
        if e is None:
            return "missing_risk_envelope"
        if (
            e.scope_id != s.scope_id
            or e.motion_epoch != s.motion.source_epoch
            or s.mode not in e.modes
            or e.response_profile != r.profile_id
            or s.frame_id != s.perception.frame_id
        ):
            return "envelope_not_applicable"
        if now >= e.valid_until_steady:
            return "envelope_expired"
        if e.lateral_drift_mps > 0 and e.lateral_stop_s is None:
            return "lateral_stop_model_missing"
        if e.reaction_accel_upper.vx > r.accel or e.reaction_accel_upper.wz > r.yaw_accel:
            return "acceleration_outside_response"
        return None

    def _bounds(self, s: ConstraintSnapshot) -> tuple[tuple[float, float], tuple[float, float]]:
        e = s.envelope
        assert e is not None
        linear, yaw = e.linear_interval, e.yaw_interval
        upper = s.motion.upper
        if upper is not None:
            if min(upper.vx, upper.vy, upper.wz) < 0:
                raise ValueError("invalid_upper")
            linear = (max(linear[0], -upper.vx), min(linear[1], upper.vx))
            yaw = (max(yaw[0], -upper.wz), min(yaw[1], upper.wz))
            if linear[0] > linear[1] or yaw[0] > yaw[1]:
                raise ValueError("inconsistent_bounds")
        return linear, yaw

    def _release(self, s: ConstraintSnapshot) -> tuple[float, float]:
        assert s.envelope is not None
        a, r = s.envelope.reaction_accel_upper, s.response
        # Old outward acceleration can persist through lag, then takes a/J to
        # release. This is a command-transition event, distinct from expiry braking.
        return (
            a.vx * r.actuator_lag + a.vx * a.vx / (2 * r.jerk),
            a.wz * r.actuator_lag + a.wz * a.wz / (2 * r.yaw_jerk),
        )

    def _effective_bounds(
        self, s: ConstraintSnapshot
    ) -> tuple[tuple[float, float], tuple[float, float]]:
        linear, yaw = self._bounds(s)
        dv, dw = self._release(s)
        return ((linear[0] - dv, linear[1] + dv), (yaw[0] - dw, yaw[1] + dw))

    def _sweep(
        self, s: ConstraintSnapshot, target: Velocity, *, stop_only: bool, coarse: bool
    ) -> SweepCertificate:
        e = s.envelope
        assert e is not None
        linear, yaw = self._effective_bounds(s)
        # Braking already integrates reaction/release: don't add it twice.
        speed_linear, speed_yaw = self._bounds(s) if stop_only else (linear, yaw)
        r = s.response
        v = max(abs(speed_linear[0]), abs(speed_linear[1]), abs(target.vx))
        w = max(abs(speed_yaw[0]), abs(speed_yaw[1]), abs(target.wz))
        a = (
            e.reaction_accel_upper
            if stop_only
            else Velocity(r.accel if v else 0, r.yaw_accel if w else 0)
        )
        stop_v = stop_axis(v, a.vx, r.decel, r.jerk, r.actuator_lag)
        stop_w = stop_axis(w, a.wz, r.yaw_decel, r.yaw_jerk, r.actuator_lag)
        drive = 0.0 if stop_only else self.config.lease_s
        horizon = max(stop_v.duration, stop_w.duration, e.lateral_stop_s or 0.0) + drive
        if horizon > e.max_horizon_s:
            raise ValueError("model_horizon_exceeded")
        length = stop_v.distance + v * drive
        turn = stop_w.distance + w * drive
        low = -turn if yaw[0] < 0 or target.wz < 0 else 0.0
        high = turn if yaw[1] > 0 or target.wz > 0 else 0.0
        if turn and low == high:
            low, high = -turn, turn  # residual acceleration, sign unknown
        low -= e.yaw_error_rad
        high += e.yaw_error_rad
        negative = linear[0] < 0 or target.vx < 0
        positive = linear[1] > 0 or target.vx > 0
        if linear == (0.0, 0.0) and e.reaction_accel_upper.vx > 0:
            negative = positive = True
        signs: tuple[float, ...] = tuple(
            sign for sign, possible in ((-1.0, negative), (1.0, positive)) if possible
        ) or (1.0,)
        sweep = _reachable_shape(
            s.footprint.vertices, low, high, length, signs, self.config, coarse=coarse
        )
        physical = _reachable_shape(
            s.footprint.physical_vertices, low, high, length, signs, self.config, coarse=coarse
        )
        error = e.position_error_m + e.lateral_drift_mps * horizon
        if error:
            sweep = sweep.buffer(error, join_style="mitre")
            physical = physical.buffer(error, join_style="mitre")
        # Current-state domains include all velocities up to the certified peak.
        domain_v = (-v if -1.0 in signs else 0.0, v if 1.0 in signs else 0.0)
        domain_w = (
            -w
            if yaw[0] < 0 or target.wz < 0 or (yaw == (0.0, 0.0) and e.reaction_accel_upper.wz > 0)
            else 0.0,
            w
            if yaw[1] > 0 or target.wz > 0 or (yaw == (0.0, 0.0) and e.reaction_accel_upper.wz > 0)
            else 0.0,
        )
        return SweepCertificate(
            target, sweep, physical, domain_v, domain_w, horizon, e, _response_key(r)
        )

    def _clear(self, c: SweepCertificate, s: ConstraintSnapshot, now: float) -> str | None:
        assert s.envelope is not None
        if (
            s.perception.stamp_steady + s.perception.forecast_horizon_s < now + c.horizon_s
            or s.envelope.valid_until_steady < now + c.horizon_s
            or s.response.valid_until_steady < now + c.horizon_s
            or s.envelope.max_horizon_s < c.horizon_s
        ):
            return "forecast_horizon_short"
        world = _world(c.sweep, s.pose)
        if not world.is_valid or not math.isfinite(world.area) or world.area <= 0:
            return "geometry_numeric_range"
        if world.intersects(s.perception.occupied):
            return "sweep_obstructed"
        padded_new = world.difference(_world(Polygon(s.footprint.vertices), s.pose))
        physical_new = _world(c.physical_sweep, s.pose).difference(
            _world(Polygon(s.footprint.physical_vertices), s.pose)
        )
        for required in (padded_new, physical_new):
            if not required.is_empty and not s.perception.free_space.covers(required):
                return "sweep_unknown"
        return None

    def _stop(self, s: ConstraintSnapshot, now: float) -> BrakeRequest:
        error = self._context_error(s, now)
        if error:
            return BrakeRequest(BrakeKind.CONTROLLED, error, False)
        try:
            c = self._sweep(s, Velocity(0, 0), stop_only=True, coarse=True)
            reason = self._clear(c, s, now)
        except ValueError as error:
            return BrakeRequest(BrakeKind.CONTROLLED, str(error), False)
        if reason == "sweep_obstructed":
            return BrakeRequest(BrakeKind.EMERGENCY, "stop_envelope_blocked", False)
        if reason:
            return BrakeRequest(BrakeKind.CONTROLLED, reason, False)
        return BrakeRequest(BrakeKind.CONTROLLED, "controlled_stop", True)

    def identity(self, s: ConstraintSnapshot) -> tuple[str, ...]:
        return _identity(s)

    def _reverse_required(self, s: ConstraintSnapshot) -> bool:
        linear, _ = self._bounds(s)
        return (linear[0] > 0 and s.target.velocity.vx < 0) or (
            linear[1] < 0 and s.target.velocity.vx > 0
        )

    def refine(self, s: ConstraintSnapshot, now: float) -> ConstraintResult:
        stop = self._stop(s, now)
        if not stop.stop_verified:
            return ConstraintResult(None, stop)
        if not math.isfinite(s.target.deadline_steady) or now >= s.target.deadline_steady:
            return ConstraintResult(
                None, BrakeRequest(BrakeKind.CONTROLLED, "target_expired", True)
            )
        if s.target.velocity == Velocity(0, 0):
            return ConstraintResult(None, stop)
        assert s.envelope is not None
        if self._reverse_required(s):
            return ConstraintResult(
                None, BrakeRequest(BrakeKind.CONTROLLED, "reverse_requires_stop", True)
            )
        certs = []
        for fraction in self.config.fractions:
            v = Velocity(s.target.velocity.vx * fraction, s.target.velocity.wz * fraction)
            if abs(v.vx) > s.envelope.linear_cap_mps or abs(v.wz) > s.envelope.yaw_cap_rps:
                continue
            try:
                c = self._sweep(s, v, stop_only=False, coarse=False)
            except ValueError:
                continue
            if self._clear(c, s, now) is None:
                certs.append(c)
        if not certs:
            return ConstraintResult(
                None, BrakeRequest(BrakeKind.CONTROLLED, "no_clear_candidate", True)
            )
        deadline = min(
            now + self.config.lease_s,
            s.target.deadline_steady,
            s.motion.valid_until_steady,
            s.perception.deadline_steady,
            s.response.valid_until_steady,
            s.envelope.valid_until_steady,
        )
        permit = MotionPermit(tuple(c.velocity for c in certs), now, deadline, _identity(s))
        return ConstraintResult(permit, None, tuple(certs))

    def tighten(
        self, result: ConstraintResult | None, s: ConstraintSnapshot, now: float
    ) -> ConstraintResult:
        stop = self._stop(s, now)
        if not stop.stop_verified:
            return ConstraintResult(None, stop)
        if not math.isfinite(s.target.deadline_steady) or now >= s.target.deadline_steady:
            return ConstraintResult(
                None, BrakeRequest(BrakeKind.CONTROLLED, "target_expired", True)
            )
        if result is None or result.permit is None:
            return ConstraintResult(
                None, BrakeRequest(BrakeKind.CONTROLLED, "waiting_refinement", True)
            )
        if self._reverse_required(s):
            return ConstraintResult(
                None, BrakeRequest(BrakeKind.CONTROLLED, "reverse_requires_stop", True)
            )
        p = result.permit
        if now >= p.deadline_steady or _identity(s) != p.source_identity:
            return ConstraintResult(
                None, BrakeRequest(BrakeKind.CONTROLLED, "permit_expired_or_changed", True)
            )
        assert s.envelope is not None
        e = s.envelope
        linear, yaw = self._effective_bounds(s)
        keep = []
        for c in result.certificates:
            v = c.velocity
            target = s.target.velocity
            if not (
                v in p.velocities
                and abs(v.vx) <= abs(target.vx)
                and v.vx * target.vx >= 0
                and abs(v.wz) <= abs(target.wz)
                and v.wz * target.wz >= 0
            ):
                continue
            if abs(v.vx) > e.linear_cap_mps or abs(v.wz) > e.yaw_cap_rps:
                continue
            contained = (
                c.response_key == _response_key(s.response)
                and e.position_error_m <= c.risk.position_error_m
                and e.yaw_error_rad <= c.risk.yaw_error_rad
                and e.lateral_drift_mps <= c.risk.lateral_drift_mps
                and (e.lateral_stop_s or 0.0) <= (c.risk.lateral_stop_s or 0.0)
                and c.linear_domain[0] <= linear[0] <= linear[1] <= c.linear_domain[1]
                and c.yaw_domain[0] <= yaw[0] <= yaw[1] <= c.yaw_domain[1]
            )
            if not contained:
                # Recheck only an already permitted target, with a bounded analytic
                # envelope. No new velocity or slow rotation sampling on this path.
                try:
                    c = self._sweep(s, v, stop_only=False, coarse=True)
                except ValueError:
                    continue
            if self._clear(c, s, now) is None:
                keep.append(c)
        if not keep:
            return ConstraintResult(
                None, BrakeRequest(BrakeKind.CONTROLLED, "latest_observation_veto", True)
            )
        return ConstraintResult(
            MotionPermit(
                tuple(c.velocity for c in keep),
                p.created_steady,
                min(
                    p.deadline_steady,
                    s.target.deadline_steady,
                    s.motion.valid_until_steady,
                    s.perception.deadline_steady,
                    s.response.valid_until_steady,
                    e.valid_until_steady,
                ),
                p.source_identity,
            ),
            None,
            tuple(keep),
        )

    def admit(
        self, result: ConstraintResult, s: ConstraintSnapshot, now: float
    ) -> ConstraintResult:
        return self.tighten(result, s, now)

    def validate_output(
        self, result: ConstraintResult, s: ConstraintSnapshot, velocity: Velocity, now: float
    ) -> bool:
        latest = self.tighten(result, s, now)
        if latest.permit is None:
            return False
        if latest.permit.allows(velocity, now):
            return True
        # No automatic interpolation. One explicit analytic envelope check may
        # certify a ramp proposal bounded by ONE retained target (not mixed axes
        # from unrelated permits). T5 still owns acceleration/jerk/PI feasibility.
        if not any(
            abs(velocity.vx) <= abs(v.vx)
            and velocity.vx * v.vx >= 0
            and abs(velocity.wz) <= abs(v.wz)
            and velocity.wz * v.wz >= 0
            for v in latest.permit.velocities
        ):
            return False
        try:
            proposed = self._sweep(s, velocity, stop_only=False, coarse=True)
        except ValueError:
            return False
        return self._clear(proposed, s, now) is None
