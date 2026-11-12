"""Bounded, causal ZOH resolution of accepted events, not a second health owner."""

import math
from collections import deque
from collections.abc import Callable
from dataclasses import dataclass

from senior_control.models import Pose2D
from senior_control.motion.admission_events import SourceSnapshot
from senior_control.motion.kinematics import body_delta, compose, exponential, logarithm
from senior_control.motion.observations import (
    AdaptedObservation,
    CovarianceKind,
    Frame,
    Observation,
    Source,
    is_current,
)
from senior_control.motion.sample_types import DeltaObservation, PointObservation

Key = tuple[str, str, int]
Values = tuple[float | None, float | None, float | None]
ComponentKey = tuple[Source, int]
Dependencies = tuple[tuple[ComponentKey, ...], tuple[ComponentKey, ...], tuple[ComponentKey, ...]]


def sample_key(sample: AdaptedObservation) -> Key:
    o = sample.original
    return (o.source.value, o.epoch, o.sequence)


def variances(sample: AdaptedObservation) -> tuple[float, ...] | None:
    o = sample.original
    c = o.covariance
    if c is None or o.model_id is None:
        return None
    n = len(o.values)
    return (
        c.values
        if c.kind is CovarianceKind.DIAGONAL_ONLY
        else tuple(c.values[i * n + i] for i in range(n))
    )


@dataclass(frozen=True)
class TemporalConfig:
    history_s: float
    max_samples: int
    max_hold_s: float
    # //Compatible: retain both injected inputs; effective ZOH coverage is their minimum.
    max_gap_s: float
    prediction_horizon_s: float
    assume_nonholonomic: bool

    def __post_init__(self) -> None:
        if self.max_samples < 2 or not all(
            math.isfinite(x) and x > 0
            for x in (self.history_s, self.max_hold_s, self.max_gap_s, self.prediction_horizon_s)
        ):
            raise ValueError("invalid injected temporal policy")


@dataclass(frozen=True)
class ResolvedSource:
    source: Source
    values: Values
    variances: Values
    original_keys: tuple[Key, ...]
    model_ids: tuple[str, ...]
    reason: str
    delta: Pose2D | None = None
    predicted_delta: Pose2D | None = None
    covariance_note: str = "unsupported"
    inputs: tuple[AdaptedObservation, ...] = ()
    kinematic_yaw: float | None = None
    # //Compatible: normalized rate components have no additional transport dependencies.
    dependencies: Dependencies = ((), (), ())


@dataclass(frozen=True)
class ResolvedWindow:
    start_ros: float
    end_ros: float
    sources: tuple[ResolvedSource, ...]
    diagnostics: tuple[str, ...]

    def by_source(self, source: Source) -> ResolvedSource:
        return next(s for s in self.sources if s.source is source)


class CoverageError(ValueError):
    pass


class TemporalResolver:
    def __init__(self, config: TemporalConfig):
        self.config = config
        self.cursor = 0
        self._history: dict[Source, deque[AdaptedObservation]] = {
            s: deque(maxlen=config.max_samples) for s in Source
        }
        self._diagnostics: deque[str] = deque(maxlen=config.max_samples)

    def consume(self, snapshot: SourceSnapshot) -> None:
        reset = snapshot.cursor < self.cursor or (
            snapshot.journal_gap and snapshot.cursor > self.cursor
        )
        start_cursor = 0 if reset else self.cursor
        if reset:
            for history in self._history.values():
                history.clear()
            self._diagnostics.append("admission_journal_gap")
        live = dict(snapshot.sources)
        for event in snapshot.events:
            if event.cursor <= start_cursor:
                continue
            history = self._history[event.source]
            if event.sample is None:
                history.clear()
            else:
                if len(history) == self.config.max_samples:
                    self._diagnostics.append("history_capacity_eviction")
                history.append(event.sample)
        self.cursor = snapshot.cursor
        rules = {rule.source: rule for rule in snapshot.rules}
        for source, history in self._history.items():
            view = live.get(source)
            retained = [
                p
                for p in history
                if view is not None
                and p.original.epoch == view.observation.original.epoch
                and snapshot.now_ros - p.original.end_ros <= self.config.history_s
                and is_current(
                    p, rules[source], now_ros=snapshot.now_ros, now_steady=snapshot.now_steady
                )
            ]
            history.clear()
            history.extend(retained)

    def _ready_history(self, source: Source, snapshot: SourceSnapshot) -> list[AdaptedObservation]:
        view = dict(snapshot.sources).get(source)
        if view is None:
            raise CoverageError(f"dependency_{source.value}_unavailable")
        if not view.ready:
            raise CoverageError(f"dependency_{source.value}_recovering")
        latest = view.observation.original
        history = [
            p
            for p in self._history[source]
            if p.original.epoch == latest.epoch and p.original.sequence <= latest.sequence
        ]
        if not history:
            raise CoverageError(f"dependency_{source.value}_history_missing")
        return history

    def _check_span(self, stamp: float, until: float) -> None:
        # The old two injected ceilings intersect: one effective ZOH support
        # horizon, enforced identically for known intervals, open tails and holds.
        limit = min(self.config.max_gap_s, self.config.max_hold_s)
        if until - stamp > limit:
            reason = (
                "sample_gap" if self.config.max_gap_s < self.config.max_hold_s else "hold_expired"
            )
            raise CoverageError(reason)

    def _segments(
        self, source: Source, a: float, b: float, snapshot: SourceSnapshot
    ) -> list[tuple[float, float, AdaptedObservation]]:
        points = [
            p
            for p in self._ready_history(source, snapshot)
            if isinstance(p.original, PointObservation)
        ]
        left = [p for p in points if p.original.end_ros <= a]
        if not left:
            raise CoverageError("missing_left_coverage")
        current = left[-1]
        cursor = a
        segments = []
        for following in [p for p in points if a < p.original.end_ros < b]:
            end = following.original.end_ros
            self._check_span(current.original.end_ros, end)
            segments.append((cursor, end, current))
            cursor = end
            current = following
        self._check_span(current.original.end_ros, b)
        segments.append((cursor, b, current))
        return segments

    def _held(self, source: Source, t: float, snapshot: SourceSnapshot) -> AdaptedObservation:
        points = [
            p
            for p in self._ready_history(source, snapshot)
            if isinstance(p.original, PointObservation) and p.original.end_ros <= t
        ]
        if not points:
            raise CoverageError("missing_left_coverage")
        p = points[-1]
        self._check_span(p.original.end_ros, t)
        return p

    def _gyro(
        self,
        a: float,
        b: float,
        snapshot: SourceSnapshot,
        override: dict[tuple[Key, int], float] | None = None,
    ) -> float:
        if a == b:
            return 0.0
        if a > b:
            raise CoverageError("noncausal_heading")
        return sum(
            (right - left) * self._value(p, 0, override)
            for left, right, p in self._segments(Source.IMU, a, b, snapshot)
        )

    @staticmethod
    def _value(
        p: AdaptedObservation, i: int, override: dict[tuple[Key, int], float] | None
    ) -> float:
        return (override or {}).get((sample_key(p), i), p.original.values[i])

    @staticmethod
    def _keys(samples: list[AdaptedObservation]) -> tuple[Key, ...]:
        return tuple(dict.fromkeys(sample_key(p) for p in samples))

    @staticmethod
    def _models(samples: list[AdaptedObservation]) -> tuple[str, ...]:
        if any(p.original.model_id is None for p in samples):
            return ()
        return tuple(sorted({str(p.original.model_id) for p in samples}))

    def _numeric_variance(
        self,
        function: Callable[[dict[tuple[Key, int], float]], tuple[float, float, float]],
        samples: list[AdaptedObservation],
    ) -> Values:
        unique = {sample_key(p): p for p in samples}
        if any(variances(p) is None for p in unique.values()):
            return (None, None, None)
        sigma = [0.0, 0.0, 0.0]
        for key, p in unique.items():
            v = variances(p)
            assert v is not None
            parameters = list(enumerate(zip(p.original.values, v, strict=True)))
            if p.heading_context is not None:
                parameters.append(
                    (
                        len(p.original.values),
                        (p.heading_context.yaw_rad, p.heading_context.variance_rad2),
                    )
                )
            for index, (mean, var) in parameters:
                if var == 0:
                    continue
                step = math.ulp(1.0) ** (1 / 3) * max(1.0, abs(mean))
                high = function({(key, index): mean + step})
                low = function({(key, index): mean - step})
                for i in range(3):
                    sigma[i] += abs((high[i] - low[i]) / (2 * step)) * math.sqrt(var)
        return (sigma[0] ** 2, sigma[1] ** 2, sigma[2] ** 2)

    def _independent_interval(
        self, source: Source, a: float, b: float, snapshot: SourceSnapshot
    ) -> ResolvedSource | None:
        for sample in reversed(self._ready_history(source, snapshot)):
            o = sample.original
            if isinstance(o, PointObservation) or o.start_ros != a or o.end_ros != b:
                continue
            if isinstance(o, DeltaObservation):

                def mean(
                    override: dict[tuple[Key, int], float],
                    sample: AdaptedObservation = sample,
                    o: DeltaObservation = o,
                ) -> tuple[float, float, float]:
                    values = tuple(self._value(sample, i, override) for i in range(3))
                    if abs(values[2]) >= math.pi:
                        raise CoverageError("rotation_exceeds_log_domain")
                    pose = Pose2D(*values)
                    return logarithm(
                        body_delta(pose, o.body_from_sensor) if o.frame is Frame.SENSOR else pose,
                        b - a,
                    )

                values = mean({})
                variance = self._numeric_variance(mean, [sample])
                delta = (
                    body_delta(o.delta, o.body_from_sensor) if o.frame is Frame.SENSOR else o.delta
                )
                return ResolvedSource(
                    source,
                    values,
                    variance,
                    (sample_key(sample),),
                    self._models([sample]),
                    "covered",
                    delta,
                    None,
                    "first_order_delta_log_unknown_correlation",
                    (sample,),
                    dependencies=(((source, 2),), ((source, 2),), ()),
                )
            if isinstance(o, Observation) and source is Source.LASER:
                rate_variance = variances(sample)
                packed: Values = (
                    (None, None, None)
                    if rate_variance is None
                    else (rate_variance[0], rate_variance[1], rate_variance[2])
                )
                return ResolvedSource(
                    source,
                    (o.values[0], o.values[1], o.values[2]),
                    packed,
                    (sample_key(sample),),
                    self._models([sample]),
                    "covered",
                    None,
                    None,
                    "declared_rate_covariance",
                    (sample,),
                )
        return None

    def _point_source(
        self, source: Source, a: float, b: float, snapshot: SourceSnapshot
    ) -> ResolvedSource:
        segments = self._segments(source, a, b, snapshot)
        used = [p for _, _, p in segments]
        dt = b - a
        if source is Source.IMU:
            value = self._gyro(a, b, snapshot) / dt
            vs = [variances(p) for _, _, p in segments]
            variance = (
                None
                if any(v is None for v in vs)
                else sum(
                    (right - left) / dt * math.sqrt(v[0])
                    for (left, right, _), v in zip(segments, vs, strict=True)
                    if v is not None
                )
                ** 2
            )
            return ResolvedSource(
                source,
                (None, None, value),
                (None, None, variance),
                self._keys(used),
                self._models(used),
                "covered",
                covariance_note="unknown_temporal_correlation",
                inputs=tuple(used),
            )
        gyro_segments = self._segments(Source.IMU, a, b, snapshot)
        if abs(self._gyro(a, b, snapshot)) >= math.pi:
            raise CoverageError("rotation_exceeds_log_domain")
        used += [p for _, _, p in gyro_segments]
        knots = sorted(
            {a, b, *[right for _, right, _ in segments], *[right for _, right, _ in gyro_segments]}
        )
        if source is Source.WHEEL:
            if not self.config.assume_nonholonomic:
                raise CoverageError("wheel_motion_assumption_missing")
            delta = Pose2D(0.0, 0.0, 0.0)
            for left, right in zip(knots, knots[1:], strict=False):
                vx = self._held(source, left, snapshot).original.values[0]
                w = self._held(Source.IMU, left, snapshot).original.values[0]
                delta = compose(delta, exponential(vx, 0.0, w, right - left))
            vx = logarithm(delta, dt)[0]
            return ResolvedSource(
                source,
                (vx, None, self._gyro(a, b, snapshot) / dt),
                (None, None, None),
                self._keys(used),
                (),
                "covered",
                None,
                delta,
                "wheel_kinematic_reference_not_independent",
                tuple(used),
                sum((right - left) * p.original.values[1] for left, right, p in segments) / dt,
                dependencies=(((Source.IMU, 2),), (), ((Source.IMU, 2),)),
            )
        if source is not Source.GPS:
            raise CoverageError("no_interval")
        # GPS lever arm correction needs same-window gyro and a known heading.
        for p in list(used):
            if p.original.source is Source.GPS:
                h = p.heading_context
                if h is None:
                    raise CoverageError("heading_missing")
                if h.stamp_ros > p.original.end_ros:
                    raise CoverageError("noncausal_heading")
                used += (
                    [x for _, _, x in self._segments(Source.IMU, h.stamp_ros, a, snapshot)]
                    if h.stamp_ros < a
                    else []
                )

        def mean(override: dict[tuple[Key, int], float]) -> tuple[float, float, float]:
            delta = Pose2D(0.0, 0.0, 0.0)
            for left, right in zip(knots, knots[1:], strict=False):
                p = self._held(Source.GPS, left, snapshot)
                o = p.original
                h = p.heading_context
                assert (
                    isinstance(o, PointObservation)
                    and h is not None
                    and o.body_from_sensor is not None
                )
                yaw = override.get((sample_key(p), len(o.values)), h.yaw_rad) + self._gyro(
                    h.stamp_ros, left, snapshot, override
                )
                w = self._value(self._held(Source.IMU, left, snapshot), 0, override)
                theta = w * (right - left)
                east, north = self._value(p, 0, override), self._value(p, 1, override)
                vx, vy = (
                    math.cos(yaw) * east + math.sin(yaw) * north,
                    -math.sin(yaw) * east + math.cos(yaw) * north,
                )
                x, y = o.body_from_sensor.x, o.body_from_sensor.y
                step = Pose2D(
                    vx * (right - left) + x - math.cos(theta) * x + math.sin(theta) * y,
                    vy * (right - left) + y - math.sin(theta) * x - math.cos(theta) * y,
                    theta,
                )
                delta = compose(delta, step)
            return logarithm(delta, dt)

        values = mean({})
        gps_variance = self._numeric_variance(mean, used)
        return ResolvedSource(
            source,
            (values[0], values[1], None),
            (gps_variance[0], gps_variance[1], None),
            self._keys(used),
            self._models(used),
            "covered",
            covariance_note="first_order_heading_SE2_not_a_strict_bound",
            inputs=tuple(used),
            dependencies=(((Source.IMU, 2),), ((Source.IMU, 2),), ()),
        )

    def resolve(self, a: float, b: float, snapshot: SourceSnapshot) -> ResolvedWindow:
        if not (math.isfinite(a) and math.isfinite(b) and a < b <= snapshot.now_ros):
            raise ValueError("invalid causal window")
        live = dict(snapshot.sources)
        sources = []
        for source in Source:
            try:
                if source not in live:
                    raise CoverageError("source_unavailable")
                if not live[source].ready:
                    raise CoverageError("recovering")
                resolved = self._independent_interval(source, a, b, snapshot) or self._point_source(
                    source, a, b, snapshot
                )
            except CoverageError as error:
                resolved = ResolvedSource(
                    source, (None, None, None), (None, None, None), (), (), str(error)
                )
            sources.append(resolved)
        diagnostics = tuple(self._diagnostics)
        self._diagnostics.clear()
        return ResolvedWindow(a, b, tuple(sources), diagnostics)
