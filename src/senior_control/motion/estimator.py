"""Bounded per-axis control estimates from original, authenticated observations.

Weights are software heuristics, never a physical speed/error guarantee. Output
prediction cannot refresh a root timestamp or turn a wheel/model into body truth.
"""

import math
from collections import deque
from dataclasses import dataclass, replace

from senior_control.sensors.consensus import Vote, combine
from senior_control.sensors.policy import Age, SensorTiming, default_policy


@dataclass(frozen=True)
class Origin:
    family: str
    epoch: str
    sequence: int
    stamp_ros: float
    received_ros: float
    received_steady: float


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
            or not all(math.isfinite(v) for v in (r.stamp_ros, r.received_ros, r.received_steady))
            or not 0 <= r.stamp_ros <= 9_223_372_036
            or not 0 <= r.received_ros <= 9_223_372_036
            or r.received_ros < r.stamp_ros
        ):
            raise ValueError("invalid original motion metadata")


@dataclass(frozen=True)
class MotionObservation:
    source: str
    values: tuple[float | None, float | None, float | None]
    measured_ros: float
    roots: tuple[Origin, ...]
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
            or not math.isfinite(self.measured_ros)
            or self.interval_half_s < 0
            or not math.isfinite(self.interval_half_s)
            or any(v is not None and not math.isfinite(v) for v in self.values)
            or any(e is not None and (not math.isfinite(e) or e < 0) for e in self.error)
            or self.quality is not None
            and (not math.isfinite(self.quality) or not 0 <= self.quality <= 1)
        ):
            raise ValueError("invalid motion observation")
        validate_origins(self.roots)
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
            not min(r.stamp_ros for r in self.roots) - 1e-9
            <= self.measured_ros
            <= max(r.stamp_ros for r in self.roots) + 1e-9
        ):
            raise ValueError("measurement time outside original observation support")

    @property
    def identity_epoch(self) -> str:
        return self.epoch or max(self.roots, key=lambda r: (r.stamp_ros, r.family)).epoch

    @property
    def identity_sequence(self) -> int:
        return (
            self.sequence
            if self.sequence is not None
            else max(self.roots, key=lambda r: (r.stamp_ros, r.family)).sequence
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


@dataclass(frozen=True)
class MotionEstimate:
    vx: AxisEstimate
    vy: AxisEstimate
    wz: AxisEstimate
    computed_ros: float
    computed_steady: float
    generation: int
    independent_axes: tuple[AxisEstimate, AxisEstimate] | None = None

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
        if now_ros is not None and now_steady is not None and observation.quality != 0:
            # Invalid clocks must not poison the current source frontier/history.
            self._age(observation.roots, now_ros, now_steady)
        old = self._frontiers.get(observation.source)
        root = max(observation.roots, key=lambda r: (r.stamp_ros, r.family))
        retired = self._retired.setdefault(observation.source, set())
        if observation.identity_epoch in retired:
            return False
        if old is not None:
            prior = max(old.roots, key=lambda r: (r.stamp_ros, r.family))
            if observation.identity_epoch == old.identity_epoch:
                if observation.identity_sequence <= old.identity_sequence:
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
            original = max(positive.roots, key=lambda r: (r.stamp_ros, r.family))
            if root.epoch == original.epoch and (
                root.stamp_ros <= original.stamp_ros
                or observation.measured_ros <= positive.measured_ros
            ):
                # A new delivery/processing sequence cannot renew an unchanged
                # acquisition, including while the ROS clock is paused.
                return False
        self._frontiers[observation.source] = observation
        if observation.quality == 0:
            self.invalidate(observation.source)
            return True
        self._positive_frontiers[observation.source] = observation
        histories = self._samples.setdefault(
            observation.source, tuple(deque(maxlen=2) for _ in range(3))
        )
        for axis, history in enumerate(histories):
            if observation.values[axis] is not None:
                history.append(observation)
        return True

    def _age(self, roots: tuple[Origin, ...], ros: float, steady: float) -> Age:
        ages = [
            self.timing.age(
                stamp_ros=r.stamp_ros,
                received_ros=r.received_ros,
                received_steady=r.received_steady,
                now_ros=ros,
                now_steady=steady,
            )
            for r in roots
        ]
        worst = max(ages, key=lambda a: a.seconds)
        return replace(
            worst,
            deadline_steady=min(
                min(a.deadline_steady, steady + self.timing.hard_age_s - a.seconds) for a in ages
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
                continue
            if age.band != "invalid" and item.measured_ros <= ros:
                eligible.append((item, age))
        if not eligible:
            return None
        o, age = eligible[-1]
        value = o.values[axis]
        if value is None:
            return None
        accel = 0.0
        if len(eligible) > 1:
            previous = eligible[-2][0]
            v0 = previous.values[axis]
            dt = o.measured_ros - previous.measured_ros
            try:
                usable = self._age(previous.roots, ros, steady).band != "invalid"
            except ValueError:
                usable = False
            if dt > 0 and v0 is not None and usable:
                accel = max(
                    -self.acceleration[axis], min(self.acceleration[axis], (value - v0) / dt)
                )
        elapsed = ros - o.measured_ros
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
                -max(r.stamp_ros for r in c.observation.roots),
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
            if previous is not None:
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
                    max(r.stamp_ros for r in c.observation.roots),
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
        )
        if not independent_only:
            self._last_axes[axis] = result
        return result

    def estimate(self, now_ros: float, now_steady: float) -> MotionEstimate:
        if not all(math.isfinite(v) for v in (now_ros, now_steady)):
            raise ValueError("invalid estimation clock")
        if self._clock is not None and (now_ros < self._clock[0] or now_steady < self._clock[1]):
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
                                and o.measured_ros <= now_ros
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
                        and o.measured_ros <= now_ros
                    ):
                        self._clock_epochs[source] = o.identity_epoch
                except ValueError:
                    continue
        return MotionEstimate(
            axes[0],
            axes[1],
            axes[2],
            now_ros,
            now_steady,
            self.generation,
            (independent[0], independent[1]),
        )
