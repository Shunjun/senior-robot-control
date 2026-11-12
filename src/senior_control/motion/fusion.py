"""Nonrecursive component CI over resolved windows; no vehicle permission/output."""

import math
from dataclasses import dataclass

from senior_control.models import BodyTwist
from senior_control.motion.admission_events import SourceSnapshot
from senior_control.motion.observations import Covariance, Source
from senior_control.motion.sample_types import PointObservation
from senior_control.motion.source_modes import SourceMode
from senior_control.motion.temporal import (
    ComponentKey,
    Key,
    ResolvedSource,
    ResolvedWindow,
    TemporalResolver,
    Values,
)


def covariance_intersection(m1: float, p1: float, m2: float, p2: float) -> tuple[float, float]:
    if not all(math.isfinite(x) for x in (m1, p1, m2, p2)) or p1 <= 0 or p2 <= 0:
        raise ValueError("CI needs finite means and positive justified variances")
    # .5/.5 CI in scaled form: never take the reciprocal of a tiny variance.
    lo, hi = min(p1, p2), max(p1, p2)
    ratio = lo / hi
    mean = (
        (m1 / (1 + ratio) + m2 * ratio / (1 + ratio))
        if p1 <= p2
        else (m2 / (1 + ratio) + m1 * ratio / (1 + ratio))
    )
    variance = lo * (2 / (1 + ratio))
    if not math.isfinite(mean) or not math.isfinite(variance):
        raise ValueError("unrepresentable CI result")
    return mean, variance


@dataclass(frozen=True)
class FusionConfig:
    trusted_statistical_models: tuple[str, ...]
    normal_cap_mps: float
    degraded_cap_mps: float
    conflict_linear_mps: float
    conflict_yaw_rps: float

    def __post_init__(self) -> None:
        if not all(
            math.isfinite(x) and x > 0
            for x in (
                self.normal_cap_mps,
                self.degraded_cap_mps,
                self.conflict_linear_mps,
                self.conflict_yaw_rps,
            )
        ):
            raise ValueError("invalid injected fusion policy")
        if self.degraded_cap_mps > self.normal_cap_mps:
            raise ValueError("invalid caps")


@dataclass(frozen=True)
class ComponentEstimate:
    value: float | None
    variance: float | None
    original_keys: tuple[Key, ...]
    numerically_fused: bool
    new_information: bool
    reason: str


@dataclass(frozen=True)
class CurrentReference:
    components: Values
    source_times_ros: tuple[float | None, float | None, float | None]
    requested_ros: float
    predicted_at_ros: float | None
    reason: str


@dataclass(frozen=True)
class SourceHealth:
    source: Source
    ready: bool
    age_ros_s: float
    age_steady_s: float
    valid_until_steady: float
    epoch: str
    sequence: int


@dataclass(frozen=True)
class FusionSnapshot:
    start_ros: float
    end_ros: float
    average: BodyTwist | None
    components: tuple[ComponentEstimate, ...]
    statistical_covariance: tuple[float, ...] | None
    supported_components: tuple[bool, bool, bool]
    upper: BodyTwist | None
    mode: SourceMode
    target_cap_mps: float | None
    conflicts: tuple[str, ...]
    residuals: tuple[tuple[str, float], ...]
    failures: tuple[tuple[str, str], ...]
    original_covariances: tuple[tuple[Key, Covariance | None], ...]
    current_reference: CurrentReference
    source_health: tuple[SourceHealth, ...]


@dataclass(frozen=True)
class ConflictDecision:
    blocked: frozenset[ComponentKey]
    residuals: tuple[tuple[str, float], ...]
    conflicts: tuple[str, ...]


def conflict_decision(window: ResolvedWindow, config: FusionConfig) -> ConflictDecision:
    """Reject disputed components and their transport dependents, without source blame.

    Wheel proxies are diagnostics, not independent evidence against body sensors.
    No decision persists beyond this window or changes admission recovery state.
    """
    laser, gps, imu, wheel = (
        window.by_source(source) for source in (Source.LASER, Source.GPS, Source.IMU, Source.WHEEL)
    )
    axes = ("vx", "vy", "yaw")
    residuals: list[tuple[str, float]] = []
    conflicts: list[str] = []
    blocked: set[ComponentKey] = set()
    for axis, left, right in ((0, laser, gps), (1, laser, gps), (2, laser, imu)):
        first, second = left.values[axis], right.values[axis]
        if first is None or second is None:
            continue
        difference = first - second
        residuals.append((f"{left.source.value}_{right.source.value}_{axes[axis]}", difference))
        threshold = config.conflict_yaw_rps if axis == 2 else config.conflict_linear_mps
        if abs(difference) > threshold:
            conflicts.append("independent_sources_inconsistent_attribution_unknown")
            blocked.update(((left.source, axis), (right.source, axis)))
    independent = laser if laser.values[0] is not None else gps
    for name, first, second, threshold, reason in (
        (
            "wheel_independent_vx",
            wheel.values[0],
            independent.values[0],
            config.conflict_linear_mps,
            "wheel_body_inconsistent_attribution_unknown",
        ),
        (
            "wheel_imu_yaw",
            wheel.kinematic_yaw,
            imu.values[2],
            config.conflict_yaw_rps,
            "wheel_imu_inconsistent_attribution_unknown",
        ),
    ):
        if first is not None and second is not None:
            difference = first - second
            residuals.append((name, difference))
            if abs(difference) > threshold:
                conflicts.append(reason)
    # At most three components per source: dependency closure is finite and pure.
    previous_size = -1
    while previous_size != len(blocked):
        previous_size = len(blocked)
        for source in window.sources:
            for axis, dependencies in enumerate(source.dependencies):
                if any(dependency in blocked for dependency in dependencies):
                    blocked.add((source.source, axis))
    return ConflictDecision(frozenset(blocked), tuple(residuals), tuple(dict.fromkeys(conflicts)))


class NumericalFusion:
    def __init__(self, config: FusionConfig):
        self.config = config
        # Only one monotone information watermark per source. The admission owner
        # fences retired epochs; resolver never mixes caches across an interruption.
        self._seen: dict[str, tuple[str, int]] = {}

    def _new_keys(self, keys: tuple[Key, ...]) -> bool:
        return any(
            source not in self._seen
            or self._seen[source][0] != epoch
            or sequence > self._seen[source][1]
            for source, epoch, sequence in keys
        )

    def _eligible(self, r: ResolvedSource, axis: int) -> bool:
        p = r.variances[axis]
        return (
            p is not None
            and p > 0
            and math.isfinite(p)
            and bool(r.model_ids)
            and all(m in self.config.trusted_statistical_models for m in r.model_ids)
        )

    def _component(
        self, axis: int, sources: list[ResolvedSource], blocked: frozenset[ComponentKey]
    ) -> ComponentEstimate:
        candidates = [
            r for r in sources if (r.source, axis) not in blocked and r.values[axis] is not None
        ]
        eligible = [r for r in candidates if self._eligible(r, axis)]
        used = eligible or candidates[:1]
        if not used:
            return ComponentEstimate(
                None,
                None,
                (),
                False,
                False,
                "conflicting_component"
                if any((r.source, axis) in blocked for r in sources)
                else "unsupported",
            )
        value = used[0].values[axis]
        assert value is not None
        variance = used[0].variances[axis] if eligible else None
        for other in used[1:]:
            m, p = other.values[axis], other.variances[axis]
            assert m is not None and p is not None and variance is not None
            value, variance = covariance_intersection(value, variance, m, p)
        keys = tuple(dict.fromkeys(k for r in used for k in r.original_keys))
        return ComponentEstimate(
            value,
            variance,
            keys,
            len(used) > 1,
            self._new_keys(keys),
            "ci"
            if len(used) > 1
            else "single_statistical_source"
            if eligible
            else "insufficient_statistical_model",
        )

    def current_reference(self, snapshot: SourceSnapshot, horizon: float) -> CurrentReference:
        live = dict(snapshot.sources)
        values: list[float | None] = [None, None, None]
        times: list[float | None] = [None, None, None]
        for source, axis, index in [(Source.WHEEL, 0, 0), (Source.IMU, 2, 0)]:
            view = live.get(source)
            if view is None or not view.ready:
                continue
            o = view.observation.original
            if isinstance(o, PointObservation) and 0 <= snapshot.now_ros - o.stamp_ros <= horizon:
                values[axis] = o.values[index]
                times[axis] = o.stamp_ros
        return CurrentReference(
            (values[0], values[1], values[2]),
            (times[0], times[1], times[2]),
            snapshot.now_ros,
            None,
            "held_points_not_a_current_estimate_no_acceleration_model",
        )

    def update(
        self, window: ResolvedWindow, snapshot: SourceSnapshot, resolver: TemporalResolver
    ) -> FusionSnapshot:
        laser, gps, imu, wheel = (
            window.by_source(s) for s in (Source.LASER, Source.GPS, Source.IMU, Source.WHEEL)
        )
        decision = conflict_decision(window, self.config)
        blocked = decision.blocked
        components = (
            self._component(0, [laser, gps], blocked),
            self._component(1, [laser, gps], blocked),
            self._component(2, [imu, laser], blocked),
        )
        if (
            components[0].value is None
            and wheel.values[0] is not None
            and (Source.WHEEL, 0) not in blocked
        ):
            components = (
                ComponentEstimate(
                    wheel.values[0],
                    None,
                    wheel.original_keys,
                    False,
                    self._new_keys(wheel.original_keys),
                    "wheel_kinematic_only",
                ),
                components[1],
                components[2],
            )
        supported = tuple(c.value is not None for c in components)
        cap: float | None
        vx, vy, wz = (c.value for c in components)
        if vx is not None and vy is not None and wz is not None:
            average = BodyTwist(vx, vy, wz)
            mode = (
                SourceMode.FUSED_REFERENCE
                if any(c.numerically_fused for c in components)
                else SourceMode.LASER_REFERENCE
                if any(k[0] == Source.LASER.value for k in components[0].original_keys)
                else SourceMode.GPS_IMU_REFERENCE
            )
            if components[0].reason == "wheel_kinematic_only":
                mode = SourceMode.WHEEL_IMU_LIMITED
            cap = (
                self.config.degraded_cap_mps
                if mode is SourceMode.WHEEL_IMU_LIMITED
                else self.config.normal_cap_mps
            )
        else:
            average = None
            mode = (
                SourceMode.WHEEL_IMU_LIMITED
                if (
                    supported[0]
                    and supported[2]
                    and wheel.values[0] is not None
                    and (Source.WHEEL, 0) not in blocked
                )
                else SourceMode.UNAVAILABLE
            )
            cap = self.config.degraded_cap_mps if mode is SourceMode.WHEEL_IMU_LIMITED else None
        d = sum(supported)
        p = tuple(
            d * c.variance if c.variance is not None else math.nan
            for c in components
            if c.value is not None
        )
        covariance = p if p and all(math.isfinite(x) for x in p) else None
        raw = {
            sample_key: sample.original.covariance
            for r in window.sources
            for sample in r.inputs
            for sample_key in [
                (sample.original.source.value, sample.original.epoch, sample.original.sequence)
            ]
        }
        for c in components:
            for key in c.original_keys:
                source, epoch, sequence = key
                previous = self._seen.get(source)
                if previous is None or previous[0] != epoch or sequence > previous[1]:
                    self._seen[source] = (epoch, sequence)
        return FusionSnapshot(
            window.start_ros,
            window.end_ros,
            average,
            components,
            covariance,
            (supported[0], supported[1], supported[2]),
            None,
            mode,
            cap,
            decision.conflicts,
            decision.residuals,
            tuple((r.source.value, r.reason) for r in window.sources if r.reason != "covered")
            + tuple(
                (f"{source.value}.{('vx', 'vy', 'yaw')[axis]}", "conflicting_component")
                for source, axis in sorted(blocked, key=lambda item: (item[0].value, item[1]))
            ),
            tuple(raw.items()),
            self.current_reference(snapshot, resolver.config.prediction_horizon_s),
            tuple(
                SourceHealth(
                    source,
                    view.ready,
                    snapshot.now_ros - view.observation.original.end_ros,
                    snapshot.now_steady - view.observation.original.received_steady,
                    view.observation.valid_until_steady,
                    view.observation.original.epoch,
                    view.observation.original.sequence,
                )
                for source, view in snapshot.sources
            ),
        )
