"""Collision-checked recovery with explicit budgets.

Speed, distance, duration, attempts, and observation age come from the caller.
A missing bound refuses the maneuver. Those numbers are not a calibrated
braking distance or a contact guarantee. Travel budgets use body displacement
from ``MotionEstimate``, never wheel integration.

Map evidence is read from the injected provider. This module does not build an
occupancy grid or run SLAM. A live obstacle vetoes historical free space.
Unknown, occupied, stale, over-budget, and no-return space is not entered.
Slip, blockage, and a degenerate motion estimate cancel the recovery and do
not start another attempt.
"""

import math
from collections.abc import Sequence
from dataclasses import dataclass
from typing import Protocol

from senior_control.config import DeviceProfile
from senior_control.models import MapRef, Pose2D, Velocity
from senior_control.safety.coverage import SweptRegion, supports
from senior_control.safety.guard import evaluate_motion, stopping_distance
from senior_control.safety.motion_estimator import (
    MotionEstimate,
    body_displacement,
    supports_degraded_reverse,
)
from senior_control.safety.self_filter import FilteredObservation, RayEvidence

_COLLISION = frozenset({"hard_contact", "hard_sweep_collision"})
Point2D = tuple[float, float]


@dataclass(frozen=True)
class RecoveryBounds:
    """Explicit maneuver limits. ``None`` means the limit was not configured."""

    speed: float | None
    single_distance: float | None
    cumulative_distance: float | None
    duration: float | None
    attempts: int | None
    observation_max_age: float | None
    min_progress: float | None
    progress_window: float | None
    min_localization_quality: float | None


@dataclass(frozen=True)
class _Limits:
    speed: float
    single_distance: float
    cumulative_distance: float
    duration: float
    attempts: int
    observation_max_age: float
    min_progress: float
    progress_window: float
    min_localization_quality: float


@dataclass(frozen=True)
class MapCorridorEvidence:
    """Read-only description of one body-frame corridor. Not a map update."""

    map_ref: MapRef
    known_free: bool
    occupied: bool
    unknown: bool
    observation_age: float | None
    no_return: bool


class MapEvidenceProvider(Protocol):
    """Caller-owned read-only map evidence. Implementations must not write a map."""

    def corridor(
        self,
        map_ref: MapRef,
        pose: Pose2D,
        distance: float,
        now: float,
    ) -> MapCorridorEvidence | None:
        """Return evidence for the corridor, or ``None`` when the caller has none."""


@dataclass(frozen=True)
class RecoveryContext:
    """Candidate commands, body progress, and the observations that cover them."""

    candidates: tuple[Velocity, ...]
    measured: Velocity
    observations: Sequence[FilteredObservation]
    motion: MotionEstimate
    pose: Pose2D
    map_ref: MapRef | None
    localization_quality: float | None
    tf_stamp: float | None
    planning_failed: bool
    slip: bool
    blocked: bool
    wheel_displacement: float


@dataclass(frozen=True)
class RecoveryDecision:
    velocity: Velocity
    reason: str
    finished: bool
    cancelled: bool


def _finite(value: float | None) -> bool:
    return value is not None and math.isfinite(value)


def _resolve(bounds: RecoveryBounds) -> _Limits | None:
    if not all(
        _finite(value)
        for value in (
            bounds.speed,
            bounds.single_distance,
            bounds.cumulative_distance,
            bounds.duration,
            bounds.observation_max_age,
            bounds.min_progress,
            bounds.progress_window,
            bounds.min_localization_quality,
        )
    ):
        return None
    if bounds.attempts is None:
        return None
    speed = bounds.speed
    single = bounds.single_distance
    cumulative = bounds.cumulative_distance
    duration = bounds.duration
    max_age = bounds.observation_max_age
    min_progress = bounds.min_progress
    window = bounds.progress_window
    quality = bounds.min_localization_quality
    if (
        speed is None
        or single is None
        or cumulative is None
        or duration is None
        or max_age is None
        or min_progress is None
        or window is None
        or quality is None
        or bounds.attempts < 1
        or speed <= 0.0
        or single <= 0.0
        or cumulative <= 0.0
        or duration <= 0.0
        or max_age < 0.0
        or min_progress < 0.0
        or window <= 0.0
        or not 0.0 <= quality <= 1.0
    ):
        return None
    return _Limits(
        speed,
        single,
        cumulative,
        duration,
        bounds.attempts,
        max_age,
        min_progress,
        window,
        quality,
    )


def _degenerate(motion: MotionEstimate) -> bool:
    return (
        motion.quality <= 0.0
        or not math.isfinite(motion.quality)
        or motion.dt <= 0.0
        or not math.isfinite(motion.dt)
        or any(not math.isfinite(value) for value in motion.covariance)
    )


def _axis_distances(
    command: float, measured: float, reaction: float, braking: float
) -> tuple[float, float]:
    """Match the hard-sweep split used by ``evaluate_motion`` for one axis."""
    if command * measured < 0.0:
        positive = stopping_distance(abs(command), reaction, braking) if command > 0.0 else 0.0
        negative = stopping_distance(abs(command), reaction, braking) if command < 0.0 else 0.0
        if measured > 0.0:
            positive = max(positive, stopping_distance(abs(measured), reaction, braking))
        elif measured < 0.0:
            negative = max(negative, stopping_distance(abs(measured), reaction, braking))
        return positive, negative
    speed = max(abs(command), abs(measured))
    if speed == 0.0:
        return 0.0, 0.0
    distance = stopping_distance(speed, reaction, braking)
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


def _without_no_return(
    observations: Sequence[FilteredObservation],
) -> list[FilteredObservation]:
    """A no-return ray is not free space, so it cannot support a sweep."""
    rewritten: list[FilteredObservation] = []
    for observation in observations:
        if not any(ray.kind == "clear" for ray in observation.rays):
            rewritten.append(observation)
            continue
        rays = tuple(
            RayEvidence(ray.index, ray.origin, None, "invalid") if ray.kind == "clear" else ray
            for ray in observation.rays
        )
        rewritten.append(
            FilteredObservation(
                observation.scan,
                rays,
                observation.external_hits,
                tuple(ray for ray in rays if ray.kind == "hit"),
                observation.occluded_regions,
                tuple(ray for ray in rays if ray.kind == "invalid"),
                observation.self_masks,
            )
        )
    return rewritten


class RecoveryController:
    def __init__(
        self,
        profile: DeviceProfile,
        bounds: RecoveryBounds,
        maps: MapEvidenceProvider,
        *,
        clock_domain: str,
    ) -> None:
        if not clock_domain.strip():
            raise ValueError("clock domain is required")
        self._profile = profile
        self._bounds = bounds
        self._maps = maps
        self._clock_domain = clock_domain
        self.distance_traveled = 0.0
        self.attempts_used = 0
        self._started_at: float | None = None
        self._attempt_distance = 0.0
        self._commanding = False
        self._terminal: RecoveryDecision | None = None

    def step(self, context: RecoveryContext, now: float) -> RecoveryDecision:
        if self._terminal is not None:
            return self._terminal
        limits = _resolve(self._bounds)
        if limits is None:
            return self._finish("bounds_missing", cancelled=False)
        if not self._tf_fresh(context.tf_stamp, now, limits.observation_max_age):
            return self._finish("expired_tf", cancelled=False)
        if context.slip:
            return self._finish("slip", cancelled=True)
        if _degenerate(context.motion):
            return self._finish("degenerate_motion", cancelled=True)
        if context.blocked:
            return self._finish("blockage", cancelled=True)
        # Wheel integration is intentionally ignored. The budget is body motion.
        self.distance_traveled += body_displacement(context.motion)
        if self._commanding and self._started_at is not None:
            elapsed = now - self._started_at
            progressed = self.distance_traveled - self._attempt_distance
            if elapsed >= limits.progress_window and progressed < limits.min_progress:
                return self._finish("blockage", cancelled=True)
            if elapsed > limits.duration:
                return self._finish("duration_exceeded", cancelled=False)
        if self.distance_traveled > limits.cumulative_distance:
            return self._finish("distance_exceeded", cancelled=False)
        if not self._commanding and self.attempts_used >= limits.attempts:
            return self._finish("attempts_exceeded", cancelled=False)
        return self._select(context, now, limits)

    def _finish(self, reason: str, *, cancelled: bool) -> RecoveryDecision:
        decision = RecoveryDecision(Velocity(0.0, 0.0), reason, True, cancelled)
        self._terminal = decision
        self._commanding = False
        return decision

    def _command(self, velocity: Velocity, reason: str, now: float) -> RecoveryDecision:
        if not self._commanding:
            self.attempts_used += 1
            self._started_at = now
            self._attempt_distance = self.distance_traveled
            self._commanding = True
        return RecoveryDecision(velocity, reason, False, False)

    def _tf_fresh(self, stamp: float | None, now: float, max_age: float) -> bool:
        if stamp is None or not math.isfinite(stamp) or not math.isfinite(now) or now < stamp:
            return False
        return now - stamp <= max_age

    def _travel(self, speed: float) -> float | None:
        braking = self._profile.braking_deceleration
        reaction = self._profile.reaction_time
        if braking is None or reaction is None or braking <= 0.0 or reaction < 0.0:
            return None
        if not math.isfinite(speed):
            return None
        return stopping_distance(abs(speed), reaction, braking)

    def _remaining(self, limits: _Limits) -> tuple[float, float]:
        consumed = 0.0
        if self._commanding:
            consumed = self.distance_traveled - self._attempt_distance
        single = limits.single_distance - consumed
        cumulative = limits.cumulative_distance - self.distance_traveled
        return single, cumulative

    def _within_budget(self, speed: float, limits: _Limits) -> bool:
        travel = self._travel(speed)
        if travel is None:
            return False
        single, cumulative = self._remaining(limits)
        return travel <= single and travel <= cumulative

    def _sensor(
        self, command: Velocity, context: RecoveryContext, now: float, max_age: float
    ) -> str:
        observations = list(context.observations)
        decision = evaluate_motion(command, context.measured, observations, self._profile, now)
        if not decision.allowed:
            return decision.reason
        if any(ray.kind == "clear" for item in observations for ray in item.rays):
            stripped = evaluate_motion(
                command,
                context.measured,
                _without_no_return(observations),
                self._profile,
                now,
            )
            if not stripped.allowed:
                return stripped.reason
        if not self._covered(command, context, now, max_age):
            return "required area contains unknown space"
        return ""

    def _covered(
        self, command: Velocity, context: RecoveryContext, now: float, max_age: float
    ) -> bool:
        observations = list(context.observations)
        if not observations:
            return False
        frames = {item.scan.reference_frame for item in observations}
        stamps = {item.scan.reference_stamp for item in observations}
        clocks = {item.scan.clock_domain for item in observations}
        if len(frames) != 1 or len(stamps) != 1 or len(clocks) != 1:
            return False
        clock = next(iter(clocks))
        if clock != self._clock_domain:
            return False
        pieces = self._sweep_pieces(command.vx, context.measured.vx)
        if not pieces:
            return False
        frame = next(iter(frames))
        stamp = next(iter(stamps))
        for vertices in pieces:
            try:
                result = supports(
                    SweptRegion(vertices, frame, stamp), observations, now, clock, max_age
                )
            except ValueError:
                return False
            if not result.supported:
                return False
        return True

    def _sweep_pieces(self, command: float, measured: float) -> list[tuple[Point2D, ...]]:
        braking = self._profile.braking_deceleration
        reaction = self._profile.reaction_time
        margin = self._profile.hard_margin
        footprint = self._profile.footprint
        if (
            braking is None
            or reaction is None
            or margin is None
            or braking <= 0.0
            or len(footprint) < 3
        ):
            return []
        xs = [point[0] for point in footprint]
        ys = [point[1] for point in footprint]
        forward, backward = _axis_distances(command, measured, reaction, braking)
        return _rectangles(min(xs), min(ys), max(xs), max(ys), forward, backward, margin)

    def _admissible(self, command: Velocity, limits: _Limits) -> bool:
        return (
            command.wz == 0.0
            and abs(command.vx) <= limits.speed
            and self._within_budget(command.vx, limits)
        )

    def _map_reverse(
        self, context: RecoveryContext, now: float, limits: _Limits, distance: float
    ) -> bool:
        quality = context.localization_quality
        if (
            context.map_ref is None
            or quality is None
            or not math.isfinite(quality)
            or quality < limits.min_localization_quality
            or not supports_degraded_reverse(context.motion)
        ):
            return False
        evidence = self._maps.corridor(context.map_ref, context.pose, distance, now)
        if evidence is None or evidence.map_ref != context.map_ref:
            return False
        age = evidence.observation_age
        if (
            evidence.no_return
            or evidence.occupied
            or evidence.unknown
            or not evidence.known_free
            or age is None
            or not math.isfinite(age)
            or age < 0.0
            or age > limits.observation_max_age
        ):
            return False
        return True

    def _select(self, context: RecoveryContext, now: float, limits: _Limits) -> RecoveryDecision:
        reverse_collision = False
        for candidate in context.candidates:
            if not self._admissible(candidate, limits):
                continue
            reason = self._sensor(candidate, context, now, limits.observation_max_age)
            if reason == "":
                label = "safe_reverse" if candidate.vx < 0.0 else "safe_forward"
                return self._command(candidate, label, now)
            if candidate.vx < 0.0 and reason in _COLLISION:
                reverse_collision = True
        if not context.candidates:
            reason = "planning_failure" if context.planning_failed else "protection_refused"
            return self._finish(reason, cancelled=False)
        if reverse_collision:
            return self._finish("protection_refused", cancelled=False)
        travel = self._travel(limits.speed)
        if travel is None or not self._within_budget(limits.speed, limits):
            return self._finish("distance_exceeded", cancelled=False)
        reverse = Velocity(-limits.speed, 0.0)
        reason = self._sensor(reverse, context, now, limits.observation_max_age)
        if reason in _COLLISION:
            return self._finish("protection_refused", cancelled=False)
        if reason == "":
            return self._command(reverse, "safe_reverse", now)
        if self._map_reverse(context, now, limits, travel):
            return self._command(reverse, "map_assisted_reverse", now)
        if context.planning_failed and reason not in _COLLISION:
            # Planning failure stays distinct when the sweep was not vetoed by a hit
            # and the rear is not the occluded case handled above.
            if reason not in {
                "required area contains unknown space",
                "no usable observation area",
                "coverage_rejected",
                "observation_missing",
            }:
                return self._finish("planning_failure", cancelled=False)
        return self._finish("blind_zone_blocked", cancelled=False)
