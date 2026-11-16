"""Bounded per-axis control estimates from original, authenticated observations.

Weights are software heuristics, never a physical speed/error guarantee. Output
prediction cannot refresh a root timestamp or turn a wheel/model into body truth.
"""

import math
from collections import deque
from dataclasses import dataclass, field, replace
from typing import cast
from uuid import uuid4

from senior_control.models import Pose2D
from senior_control.sensors.consensus import Vote, combine
from senior_control.sensors.policy import Age, SensorTiming, default_policy
from senior_control.sensors.time_proof import MeasurementInterval, MeasurementTime


@dataclass(frozen=True)
class Origin:
    family: str
    epoch: str
    sequence: int
    stamp_ros: float
    received_ros: float
    received_steady: float
    measurement_steady: float = field(kw_only=True)
    measurement: MeasurementTime | None = field(default=None, kw_only=True)

    @property
    def coordinate(self) -> MeasurementTime:
        # An explicit host coordinate is sufficient for pure host-only observations.
        return self.measurement or MeasurementTime("host", None, self.measurement_steady)


def origin_key(roots: tuple[Origin, ...]) -> tuple[tuple[str, str, int, float], ...]:
    """Acquisition identity, independent of processing/delivery sequence."""
    return tuple((r.family, r.epoch, r.sequence, r.measurement_steady) for r in roots)


def validate_origins(roots: tuple[Origin, ...]) -> None:
    if not isinstance(roots, tuple) or not 1 <= len(roots) <= 32:
        raise ValueError("invalid original origin set")
    for r in roots:
        if (
            not isinstance(r, Origin)
            or not isinstance(r.family, str)
            or not r.family
            or len(r.family) > 128
            or not isinstance(r.epoch, str)
            or not r.epoch
            or len(r.epoch) > 4096
            or type(r.sequence) is not int
            or r.sequence < 1
            or not all(
                math.isfinite(v)
                for v in (r.stamp_ros, r.received_ros, r.received_steady, r.measurement_steady)
            )
            or not 0 <= r.stamp_ros <= 9_223_372_036
            or not 0 <= r.received_ros <= 9_223_372_036
            or r.measurement is not None
            and not isinstance(r.measurement, MeasurementTime)
            or r.measurement_steady < 0
            or round(r.measurement_steady * 1e9) > round(r.received_steady * 1e9)
        ):
            raise ValueError("invalid original motion metadata")


@dataclass(frozen=True)
class MotionObservation:
    source: str
    values: tuple[float | None, float | None, float | None]
    measured_ros: float
    roots: tuple[Origin, ...]
    measured_steady: float = field(kw_only=True)
    measurement: MeasurementTime | None = field(default=None, kw_only=True)
    measurement_interval: MeasurementInterval | None = field(default=None, kw_only=True)
    measurement_delta: Pose2D | None = field(default=None, kw_only=True)
    quality: float | None = None
    error: tuple[float | None, float | None, float | None] = (None, None, None)
    independent: tuple[bool, bool, bool] = (False, False, False)
    interval_half_s: float = 0.0
    # Producer/delivery ordering is distinct from each original acquisition.
    epoch: str | None = None
    sequence: int | None = None

    def __post_init__(self) -> None:
        if (
            not self.source
            or len(self.source) > 128
            or not self.roots
            or len(self.roots) > 32
            or len(self.values) != 3
            or len(self.error) != 3
            or len(self.independent) != 3
            or any(type(v) is not bool for v in self.independent)
            or not all(math.isfinite(t) for t in (self.measured_ros, self.measured_steady))
            or self.interval_half_s < 0
            or not math.isfinite(self.interval_half_s)
            or any(v is not None and not math.isfinite(v) for v in self.values)
            or any(e is not None and (not math.isfinite(e) or e < 0) for e in self.error)
            or self.quality is not None
            and (not math.isfinite(self.quality) or not 0 <= self.quality <= 1)
        ):
            raise ValueError("invalid motion observation")
        validate_origins(self.roots)
        if self.measurement is not None and not isinstance(self.measurement, MeasurementTime):
            raise ValueError("invalid physical observation coordinate")
        if self.measurement_interval is not None and not isinstance(
            self.measurement_interval, MeasurementInterval
        ):
            raise ValueError("invalid physical observation support")
        if self.measurement_delta is not None and (
            not isinstance(self.measurement_delta, Pose2D)
            or self.measurement_interval is None
            or self.measurement_interval.duration <= 0
            or self.measurement_interval.end != self.coordinate
        ):
            raise ValueError("body increment needs its original physical interval")
        if (self.epoch is None) != (self.sequence is None) or (
            self.epoch is not None
            and (
                not self.epoch
                or len(self.epoch) > 4096
                or type(self.sequence) is not int
                or self.sequence < 1
            )
        ):
            raise ValueError("invalid motion observation identity")
        if (
            not min(r.measurement_steady for r in self.roots) - 1e-9
            <= self.measured_steady
            <= max(r.measurement_steady for r in self.roots) + 1e-9
        ):
            raise ValueError("measurement time outside original observation support")

    @property
    def coordinate(self) -> MeasurementTime:
        if self.measurement is not None:
            return self.measurement
        root = self.roots[0].coordinate
        if root.clock_domain.startswith("sim_step:"):
            raise ValueError("simulation observation requires an explicit physical coordinate")
        return MeasurementTime(root.clock_domain, root.epoch, self.measured_steady)

    @property
    def identity_epoch(self) -> str:
        return self.epoch or max(self.roots, key=lambda r: (r.measurement_steady, r.family)).epoch

    @property
    def identity_sequence(self) -> int:
        return (
            self.sequence
            if self.sequence is not None
            else max(self.roots, key=lambda r: (r.measurement_steady, r.family)).sequence
        )


@dataclass(frozen=True)
class AxisEstimate:
    value: float | None
    observed: bool
    confidence: float
    sources: tuple[str, ...]
    roots: tuple[Origin, ...]
    band: str
    reason: str
    error: float | None
    independent: bool
    deadline_steady: float
    # //Compatible: pure callers may omit projection metadata; roots still retain age.
    prediction_error: float = 0.0
    support_start_steady: float | None = None
    support_end_steady: float | None = None
    observation_age_s: float | None = None


Axes3 = tuple[AxisEstimate, AxisEstimate, AxisEstimate]


@dataclass(frozen=True)
class MotionPathSegment:
    """Original body increment or bounded point-rate prediction, never an ACK.

    A supplied delta is the measured SE(2) interval. Otherwise each segment
    retains its own zero-order-hold rates/error; coverage alone is not a rate.
    """

    interval: MeasurementInterval
    start_host_steady: float
    end_host_steady: float
    axes: tuple[AxisEstimate, AxisEstimate, AxisEstimate]
    independent_axes: tuple[AxisEstimate, AxisEstimate, AxisEstimate] | None
    delta: Pose2D | None = None
    independent_delta: Pose2D | None = None


@dataclass(frozen=True)
class MotionPath:
    path_id: str
    measurement: MeasurementTime
    host_steady: float
    segments: tuple[MotionPathSegment, ...]
    current_axes: tuple[AxisEstimate, AxisEstimate, AxisEstimate]
    current_independent_axes: tuple[AxisEstimate, AxisEstimate, AxisEstimate] | None

    def tail(self, previous: MeasurementTime) -> tuple[MotionPathSegment, ...]:
        if not previous.same_axis(self.measurement) or previous.seconds > self.measurement.seconds:
            raise ValueError("incompatible motion path cursor")
        cursor = previous
        result = []
        for segment in self.segments:
            if segment.interval.end.seconds <= cursor.seconds:
                continue
            if (
                not segment.interval.start.same_axis(cursor)
                or segment.interval.start.seconds > cursor.seconds
            ):
                raise ValueError("motion path gap or evicted history")
            if cursor != segment.interval.start:
                raise ValueError("motion path cursor has no original host endpoint")
            result.append(segment)
            cursor = segment.interval.end
        if cursor != self.measurement:
            raise ValueError("motion path endpoint not covered")
        return tuple(result)


@dataclass(frozen=True)
class SourceEvaluation:
    observation: MotionObservation
    age: Age | None
    outcomes: tuple[str, ...]


@dataclass(frozen=True)
class MotionEstimate:
    vx: AxisEstimate
    vy: AxisEstimate
    wz: AxisEstimate
    computed_ros: float
    computed_steady: float
    generation: int
    independent_axes: tuple[AxisEstimate, AxisEstimate] | None = None
    # The coordinate actually used to align all returned axes; never host now for simulation.
    measurement: MeasurementTime | None = None
    # Aligned support coverage only; geometric increments live in path.segments.
    measurement_interval: MeasurementInterval | None = None
    measurement_host_steady: float | None = None
    path: MotionPath | None = None

    @property
    def healthy(self) -> bool:
        return self.vx.observed or self.wz.observed

    @property
    def deadline_steady(self) -> float:
        return min(
            (a.deadline_steady for a in (self.vx, self.wz) if a.value is not None),
            default=self.computed_steady,
        )


@dataclass(frozen=True)
class _Candidate:
    observation: MotionObservation
    value: float
    age: Age
    weight: float
    quality: float
    error: float | None
    alignment_error: float


class MotionEstimation:
    """Single serial owner; one latest sample per source/axis, not per topic vote."""

    def __init__(
        self,
        timing: SensorTiming | None = None,
        *,
        linear_tolerance: float = 0.1,
        yaw_tolerance: float = 0.15,
        acceleration: tuple[float, float, float] = (0.3, 0.3, 0.5),
    ):
        if (
            not all(math.isfinite(v) and v > 0 for v in (linear_tolerance, yaw_tolerance))
            or len(acceleration) != 3
            or not all(math.isfinite(v) and v >= 0 for v in acceleration)
        ):
            raise ValueError("invalid motion estimation policy")
        self.timing = timing or default_policy()
        self.tolerances = (linear_tolerance, linear_tolerance, yaw_tolerance)
        self.acceleration = acceleration
        self._samples: dict[str, tuple[deque[MotionObservation], ...]] = {}
        self._frontiers: dict[str, MotionObservation] = {}
        self._positive_frontiers: dict[str, MotionObservation] = {}
        self._retired: dict[str, set[str]] = {}
        self._last_axes: list[AxisEstimate | None] = [None, None, None]
        self._clock: tuple[float, float] | None = None
        self.generation = 1
        self._clock_epochs: dict[str, str] = {}
        self._evaluations: dict[MotionObservation, tuple[Age | None, set[str]]] = {}
        self._measurement_target: MeasurementTime | None = None
        self._last_estimate: MotionEstimate | None = None
        self._path_id = uuid4().hex
        self._path_segments: deque[MotionPathSegment] = deque(maxlen=256)
        self._path_point: MotionPath | None = None

    @property
    def evaluations(self) -> tuple[SourceEvaluation, ...]:
        return tuple(
            SourceEvaluation(o, age, tuple(sorted(outcomes)))
            for o, (age, outcomes) in self._evaluations.items()
        )

    def _assessed(self, o: MotionObservation, age: Age | None, outcome: str) -> None:
        # Two per-axis history slots, 32 sources: at most 192 unique entries/tick.
        entry = self._evaluations.setdefault(o, (age, set()))
        entry[1].add(outcome)

    @property
    def retained_samples(self) -> int:
        return sum(len(history) for rows in self._samples.values() for history in rows)

    def invalidate(self, source: str) -> None:
        self._samples.pop(source, None)

    def ingest(
        self,
        observation: MotionObservation,
        *,
        now_ros: float | None = None,
        now_steady: float | None = None,
    ) -> bool:
        if (now_ros is None) != (now_steady is None):
            raise ValueError("both ingress clock values are required")
        if now_ros is not None and now_steady is not None:
            # Invalid clocks must not poison the current source frontier/history.
            if self._age(observation.roots, now_ros, now_steady).band == "invalid":
                return False
        old = self._frontiers.get(observation.source)
        root = max(observation.roots, key=lambda r: (r.measurement_steady, r.family))
        retired = self._retired.setdefault(observation.source, set())
        if observation.identity_epoch in retired:
            return False
        if old is not None:
            prior = max(old.roots, key=lambda r: (r.measurement_steady, r.family))
            if observation.identity_epoch == old.identity_epoch:
                if observation.identity_sequence <= old.identity_sequence:
                    return False
                if (
                    observation.coordinate.same_axis(old.coordinate)
                    and observation.coordinate.seconds < old.coordinate.seconds
                ):
                    return False
            else:
                if root.received_steady < prior.received_steady or len(retired) >= 32:
                    return False
                retired.add(old.identity_epoch)
                self.invalidate(observation.source)
        elif len(self._frontiers) >= 32:
            raise ValueError("motion source capacity")
        positive = self._positive_frontiers.get(observation.source)
        if observation.quality != 0 and positive is not None:
            original = max(positive.roots, key=lambda r: (r.measurement_steady, r.family))
            if root.epoch == original.epoch and (
                root.measurement_steady <= original.measurement_steady
                or observation.measured_steady <= positive.measured_steady
            ):
                # A new delivery/processing sequence cannot renew an unchanged
                # acquisition, including while the ROS clock is paused.
                return False
        self._frontiers[observation.source] = observation
        if observation.quality == 0:
            self.invalidate(observation.source)
            self._record_path(observation, now_ros, now_steady)
            return True
        self._positive_frontiers[observation.source] = observation
        histories = self._samples.setdefault(
            observation.source, tuple(deque(maxlen=2) for _ in range(3))
        )
        for axis, history in enumerate(histories):
            if observation.values[axis] is not None:
                history.append(observation)
        self._record_path(observation, now_ros, now_steady)
        return True

    def _record_path(
        self, incoming: MotionObservation, ros: float | None, steady: float | None
    ) -> None:
        # Complete body observations own physical displacement support. Fast
        # wheel/IMU callbacks must not subdivide an authentic scan/RGBD interval
        # into invented constant-rate pieces. They provide a bounded fallback
        # after that independent body's original lifetime actually ends.
        ros = incoming.roots[-1].received_ros if ros is None else ros
        steady = incoming.roots[-1].received_steady if steady is None else steady
        point = self._path_point
        complete = all(
            v is not None and independent
            for v, independent in zip(incoming.values, incoming.independent, strict=True)
        )
        full_body = all(v is not None for v in incoming.values)
        if not full_body and point is not None and point.current_independent_axes is not None:
            if all(
                incoming.measured_steady < a.deadline_steady for a in point.current_independent_axes
            ):
                return
        coordinate, host = incoming.coordinate, incoming.measured_steady
        if (
            point is not None
            and not coordinate.same_axis(point.measurement)
            and host < point.host_steady
        ):
            return  # Late input from the previous physical world cannot restore it.
        if (
            point is not None
            and coordinate.same_axis(point.measurement)
            and coordinate.seconds < point.measurement.seconds
        ):
            return  # A late older interval cannot revise already consumed path geometry.
        previous_target, previous_axes = self._measurement_target, list(self._last_axes)
        try:
            self._measurement_target = coordinate
            axes = cast(Axes3, tuple(self._axis(i, ros, steady) for i in range(3)))
            independent_axes = cast(
                Axes3, tuple(self._axis(i, ros, steady, independent_only=True) for i in range(3))
            )
        finally:
            self._measurement_target, self._last_axes = previous_target, previous_axes
        independent: Axes3 | None = (
            independent_axes
            if all(a.observed and a.independent and a.value is not None for a in independent_axes)
            else None
        )
        live = any(a.observed for a in (axes[0], axes[2]))
        same = point is not None and coordinate.same_axis(point.measurement)
        if not same:
            self._path_segments.clear()
            self._path_id = uuid4().hex
        elif point is not None and coordinate.seconds > point.measurement.seconds:
            span = MeasurementInterval(point.measurement, coordinate)
            support = incoming.measurement_interval
            measured_delta = incoming.measurement_delta
            direct = (
                complete
                and incoming.quality != 0
                and measured_delta is not None
                and support is not None
                and support == span
                and self._age(incoming.roots, ros, steady).band != "invalid"
            )
            if direct:
                age = self._age(incoming.roots, ros, steady)
                # The original body SE(2) increment is the interval evidence.
                # Its mean logarithmic twist is not an extrapolated endpoint rate.
                direct_axes = cast(
                    Axes3,
                    tuple(
                        AxisEstimate(
                            incoming.values[i],
                            True,
                            0.5 if incoming.quality is None else incoming.quality,
                            (incoming.source,),
                            incoming.roots,
                            age.band,
                            "measured_body_interval",
                            incoming.error[i],
                            True,
                            age.deadline_steady,
                        )
                        for i in range(3)
                    ),
                )
                segment = MotionPathSegment(
                    span,
                    point.host_steady,
                    host,
                    direct_axes,
                    direct_axes,
                    measured_delta,
                    measured_delta,
                )
            else:

                def bounded(values: Axes3) -> Axes3:
                    return cast(
                        Axes3,
                        tuple(
                            replace(
                                axis,
                                prediction_error=axis.prediction_error
                                + self.acceleration[i] * span.duration,
                                error=None
                                if axis.error is None
                                else axis.error + self.acceleration[i] * span.duration,
                            )
                            for i, axis in enumerate(values)
                        ),
                    )

                known = (
                    independent is not None
                    and point.current_independent_axes is not None
                    and all(host <= a.deadline_steady for a in point.current_independent_axes)
                )
                # A point-rate hold ends at its original host deadline. Beyond
                # that endpoint the fused path is explicitly unknown; clipping
                # the nominal rate retains the already supported displacement.
                fused = list(bounded(point.current_axes))
                if coordinate.clock_domain.startswith("host"):
                    host_span = host - point.host_steady
                    for i, axis in enumerate(fused):
                        if axis.sources and host > axis.deadline_steady:
                            fraction = max(
                                0.0,
                                min(1.0, (axis.deadline_steady - point.host_steady) / host_span),
                            )
                            fused[i] = replace(
                                axis,
                                observed=False,
                                confidence=0.0,
                                value=None if axis.value is None else axis.value * fraction,
                                error=None,
                                reason="expired_path_span",
                            )
                segment = MotionPathSegment(
                    span,
                    point.host_steady,
                    host,
                    cast(Axes3, tuple(fused)),
                    bounded(point.current_independent_axes)
                    if known and point.current_independent_axes is not None
                    else None,
                )
            self._path_segments.append(segment)
        # Retain the bounded history across unknown spans. An unavailable span
        # is represented explicitly, so sparse consumers keep earlier known work.
        self._path_point = MotionPath(
            self._path_id,
            coordinate,
            host,
            tuple(self._path_segments),
            axes,
            independent if live else None,
        )

    def _path_snapshot(self, ros: float, steady: float) -> MotionPath | None:
        point = self._path_point
        if point is None:
            return None
        candidates = (*point.current_axes, *(point.current_independent_axes or ()))
        for axis in candidates:
            if not axis.observed or not axis.roots:
                continue
            try:
                if self._age(axis.roots, ros, steady).band != "invalid":
                    return point
            except ValueError:
                continue
        return None

    def _age(self, roots: tuple[Origin, ...], ros: float, steady: float) -> Age:
        ages = [
            self.timing.age(
                measured_steady=r.measurement_steady,
                received_steady=r.received_steady,
                now_steady=steady,
            )
            for r in roots
        ]
        worst = max(ages, key=lambda a: a.seconds)
        return replace(
            worst,
            deadline_steady=min(
                min(
                    a.deadline_steady,
                    steady + (self.timing.hard_age_ms / 1000) - a.seconds,
                )
                for a in ages
            ),
        )

    def _candidate(
        self, history: deque[MotionObservation], axis: int, ros: float, steady: float
    ) -> _Candidate | None:
        eligible = []
        for item in history:
            try:
                age = self._age(item.roots, ros, steady)
            except ValueError:
                self._assessed(item, None, "invalid_time")
                continue
            if age.band == "invalid":
                self._assessed(item, age, "expired")
            elif item.measured_steady <= steady:
                target = self._measurement_target
                if (
                    target is not None
                    and item.coordinate.same_axis(target)
                    and item.coordinate.seconds <= target.seconds
                ):
                    eligible.append((item, age))
                else:
                    self._assessed(item, age, "incompatible_measurement_axis")
            else:
                self._assessed(item, None, "invalid_time")
        if not eligible:
            return None
        o, age = eligible[-1]
        value = o.values[axis]
        if value is None:
            return None
        accel = 0.0
        if len(eligible) > 1:
            previous = eligible[-2][0]
            self._assessed(previous, eligible[-2][1], "alignment_history")
            v0 = previous.values[axis]
            dt = o.coordinate.elapsed(previous.coordinate)
            try:
                usable = self._age(previous.roots, ros, steady).band != "invalid"
            except ValueError:
                usable = False
            if dt > 0 and v0 is not None and usable:
                accel = max(
                    -self.acceleration[axis], min(self.acceleration[axis], (value - v0) / dt)
                )
        target = self._measurement_target
        assert target is not None
        elapsed = target.elapsed(o.coordinate)
        aligned = value + accel * elapsed
        alignment_error = self.acceleration[axis] * (elapsed + o.interval_half_s)
        raw_error = o.error[axis]
        error = None if raw_error is None else raw_error + alignment_error
        quality = 0.5 if o.quality is None else o.quality
        if error is None:
            quality *= 0.5
        weight = quality * age.freshness / (1 + (alignment_error / self.tolerances[axis]) ** 2)
        return _Candidate(o, aligned, age, weight, quality, error, alignment_error)

    @staticmethod
    def _newest(candidates: list[_Candidate]) -> _Candidate:
        return min(
            candidates,
            key=lambda c: (
                -max(r.measurement_steady for r in c.observation.roots),
                -c.quality,
                c.alignment_error,
                c.observation.source,
            ),
        )

    def _independent_groups(self, candidates: list[_Candidate]) -> list[_Candidate]:
        # Overlapping raw families are one correlated vote, including republishers.
        groups: list[tuple[set[str], list[_Candidate]]] = []
        single_families = {
            c.observation.roots[0].family
            for c in candidates
            if len({r.family for r in c.observation.roots}) == 1
        }
        for c in candidates:
            roots = {r.family for r in c.observation.roots}
            if len(roots) > 1 and roots <= single_families:
                continue  # Raw components already represent all this derived result's votes.
            members = [c]
            changed = True
            while changed:
                changed = False
                remaining = []
                for families, prior in groups:
                    if families & roots:
                        roots |= families
                        members.extend(prior)
                        changed = True
                    else:
                        remaining.append((families, prior))
                groups = remaining
            groups.append((roots, members))
        if len(groups) > 8:
            raise ValueError("independent motion source capacity")
        return [self._newest(group) for _, group in groups]

    def _axis(
        self, axis: int, ros: float, steady: float, *, independent_only: bool = False
    ) -> AxisEstimate:
        candidates = [
            c
            for histories in self._samples.values()
            if (history := histories[axis])
            if (c := self._candidate(history, axis, ros, steady)) is not None
        ]
        if independent_only:
            candidates = [c for c in candidates if c.observation.independent[axis]]
        evaluated = candidates
        candidates = self._independent_groups(candidates)
        if not candidates:
            if axis == 1:
                return AxisEstimate(
                    0.0,
                    False,
                    0.0,
                    (),
                    (),
                    "assumed",
                    "assumed_zero_lateral",
                    None,
                    False,
                    math.inf,
                )
            previous = None if independent_only else self._last_axes[axis]
            if previous is not None and (
                self._measurement_target is None
                or all(r.coordinate.same_axis(self._measurement_target) for r in previous.roots)
            ):
                try:
                    age = self._age(previous.roots, ros, steady)
                    if age.band != "invalid":
                        return replace(
                            previous,
                            observed=False,
                            independent=False,
                            confidence=0.0,
                            band=age.band,
                            reason="model_only_bounded_hold",
                            deadline_steady=age.deadline_steady,
                            observation_age_s=age.seconds,
                            prediction_error=previous.prediction_error
                            + self.acceleration[axis]
                            * max(0.0, age.seconds - (previous.observation_age_s or 0.0)),
                            error=None,
                        )
                except ValueError:
                    pass
            return AxisEstimate(
                None, False, 0.0, (), (), "invalid", "no_axis_observation", None, False, steady
            )
        fused = combine(
            tuple(
                Vote(
                    c.value,
                    max(r.measurement_steady for r in c.observation.roots),
                    c.observation.source,
                    c.weight,
                    c.quality,
                    c.quality * c.age.freshness,
                    c.age.band == "low",
                    c.alignment_error,
                    c.error,
                )
                for c in candidates
            ),
            self.tolerances[axis],
        )
        selected = [candidates[i] for i in fused.indices]
        chosen = {c.observation for c in selected}
        for c in evaluated:
            outcome = "selected" if c.observation in chosen else "not_selected"
            if independent_only:
                outcome += "_independent"
            self._assessed(c.observation, c.age, outcome)
        roots = tuple(dict.fromkeys(r for c in selected for r in c.observation.roots))
        result = AxisEstimate(
            fused.value,
            True,
            fused.confidence,
            tuple(sorted(c.observation.source for c in selected)),
            roots,
            fused.band,
            fused.reason,
            fused.error,
            all(c.observation.independent[axis] for c in selected),
            min(c.age.deadline_steady for c in selected),
            max(c.alignment_error + abs(c.value - fused.value) for c in selected),
            min(c.observation.measured_steady - c.observation.interval_half_s for c in selected),
            max(c.observation.measured_steady + c.observation.interval_half_s for c in selected),
            max(c.age.seconds for c in selected),
        )
        if not independent_only:
            self._last_axes[axis] = result
        return result

    def estimate(self, now_ros: float, now_steady: float) -> MotionEstimate:
        self._evaluations.clear()
        if not all(math.isfinite(v) for v in (now_ros, now_steady)):
            raise ValueError("invalid estimation clock")
        if self._clock is not None and now_steady < self._clock[1]:
            retained = {}
            for source, histories in self._samples.items():
                rows = []
                for history in histories:
                    keep = []
                    for o in history:
                        if o.identity_epoch == self._clock_epochs.get(source):
                            continue
                        try:
                            if (
                                self._age(o.roots, now_ros, now_steady).band != "invalid"
                                and o.measured_steady <= now_steady
                            ):
                                keep.append(o)
                        except ValueError:
                            continue
                    rows.append(deque(keep, maxlen=2))
                if any(rows):
                    retained[source] = tuple(rows)
            self._samples = retained
            self._last_axes = [None, None, None]
            self.generation += 1
        self._clock = now_ros, now_steady
        observations = [h[-1] for histories in self._samples.values() for h in histories if h]
        simulations = [o for o in observations if o.coordinate.clock_domain.startswith("sim_step:")]
        self._measurement_target = None
        measurement_host = None
        if simulations:
            # Latest admitted acquisition chooses the active physical segment. All axes
            # must then align on this exact segment; incompatible worlds cannot vote.
            newest = max(simulations, key=lambda o: o.measured_steady)
            same = [o for o in simulations if o.coordinate.same_axis(newest.coordinate)]
            endpoint = max(same, key=lambda o: o.coordinate.seconds)
            self._measurement_target = endpoint.coordinate
            measurement_host = endpoint.measured_steady
        elif observations:
            c = observations[0].coordinate
            self._measurement_target = MeasurementTime(c.clock_domain, c.epoch, now_steady)
            measurement_host = now_steady
        axes = tuple(self._axis(i, now_ros, now_steady) for i in range(3))
        independent = tuple(
            self._axis(i, now_ros, now_steady, independent_only=True) for i in (0, 2)
        )
        self._clock_epochs = {}
        for source, histories in self._samples.items():
            for history in histories:
                if not history:
                    continue
                o = history[-1]
                try:
                    if (
                        self._age(o.roots, now_ros, now_steady).band != "invalid"
                        and o.measured_steady <= now_steady
                    ):
                        self._clock_epochs[source] = o.identity_epoch
                except ValueError:
                    continue
        previous = self._last_estimate
        interval = None
        if (
            self._measurement_target is not None
            and previous is not None
            and previous.measurement is not None
            and previous.healthy
            and previous.measurement.same_axis(self._measurement_target)
            and self._measurement_target.seconds >= previous.measurement.seconds
            and measurement_host is not None
            and measurement_host <= previous.deadline_steady
        ):
            start = (
                previous.measurement_interval.start
                if previous.measurement_interval is not None
                and previous.measurement_interval.end == previous.measurement
                else previous.measurement
            )
            interval = MeasurementInterval(start, self._measurement_target)
        result = MotionEstimate(
            axes[0],
            axes[1],
            axes[2],
            now_ros,
            now_steady,
            self.generation,
            (independent[0], independent[1]),
            self._measurement_target,
            interval,
            measurement_host,
            self._path_snapshot(now_ros, now_steady),
        )
        self._last_estimate = result
        return result
