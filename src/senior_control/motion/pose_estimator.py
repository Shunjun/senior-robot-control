"""Time-aligned pose owner. Absolute fixes and relative dead reckoning stay labelled."""

import math
from collections import deque
from dataclasses import dataclass

from senior_control.models import Pose2D
from senior_control.motion.estimator import MotionEstimate, Origin, validate_origins
from senior_control.motion.kinematics import compose, exponential
from senior_control.sensors.consensus import Vote, combine
from senior_control.sensors.policy import Age, default_policy


def angle(v: float) -> float:
    return math.atan2(math.sin(v), math.cos(v))


@dataclass(frozen=True)
class PoseObservation:
    source: str
    pose: Pose2D
    stamp_ros: float
    roots: tuple[Origin, ...]
    frame: str = "odom"
    quality: float = 0.5
    error_m: float | None = None
    error_yaw_rad: float | None = None


@dataclass(frozen=True)
class PoseEstimate:
    pose: Pose2D
    stamp_ros: float
    roots: tuple[Origin, ...]
    kind: str
    error_m: float | None
    frame: str
    error_yaw_rad: float | None = None
    confidence: tuple[float, float, float] = (0.0, 0.0, 0.0)
    sources: tuple[str, ...] = ()


@dataclass(frozen=True)
class _PoseCandidate:
    observation: PoseObservation
    pose: Pose2D
    roots: tuple[Origin, ...]
    kind: str
    errors: tuple[float | None, float | None, float | None]
    alignment: tuple[float, float, float]
    quality: float
    raw_time: float


class PoseEstimation:
    def __init__(
        self,
        *,
        frame: str = "odom",
        linear_tolerance: float = 0.1,
        angular_tolerance: float = 0.1,
        unknown_speed_mps: float = 1.52,
    ):
        if not frame or not all(
            math.isfinite(v) and v > 0
            for v in (linear_tolerance, angular_tolerance, unknown_speed_mps)
        ):
            raise ValueError("invalid pose estimation policy")
        self.frame = frame
        self.tolerances = (linear_tolerance, linear_tolerance, angular_tolerance)
        self.unknown_speed = unknown_speed_mps
        self._sources: dict[str, deque[PoseObservation]] = {}
        self._positive_frontiers: dict[str, PoseObservation] = {}
        self._relative: deque[PoseEstimate] = deque(maxlen=256)
        self._previous_motion: MotionEstimate | None = None
        self._retired: dict[str, set[str]] = {}
        self.frame_generation = 0
        self._absolute_frame_aligned = False
        self._clock_epochs: dict[str, str] = {}

    @property
    def retained_samples(self) -> int:
        return len(self._relative) + sum(len(history) for history in self._sources.values())

    def ingest(self, o: PoseObservation) -> bool:
        validate_origins(o.roots)
        if (
            not o.source
            or o.stamp_ros < 0
            or not min(r.stamp_ros for r in o.roots) - 1e-9
            <= o.stamp_ros
            <= max(r.stamp_ros for r in o.roots) + 1e-9
            or not o.roots
            or o.frame != self.frame
            or not all(
                math.isfinite(v) for v in (o.pose.x, o.pose.y, o.pose.yaw, o.stamp_ros, o.quality)
            )
            or not 0 <= o.quality <= 1
            or o.error_yaw_rad is not None
            and (not math.isfinite(o.error_yaw_rad) or o.error_yaw_rad < 0)
            or o.error_m is not None
            and (not math.isfinite(o.error_m) or o.error_m < 0)
        ):
            raise ValueError("invalid pose observation or frame")
        if o.source not in self._sources and len(self._sources) >= 8:
            raise ValueError("pose source capacity")
        history = self._sources.setdefault(o.source, deque(maxlen=128))
        if history:
            prior = history[-1]
            root, old = o.roots[0], prior.roots[0]
            retired = self._retired.setdefault(o.source, set())
            if root.epoch in retired:
                return False
            if old.epoch != root.epoch:
                if root.received_steady < old.received_steady or len(retired) >= 32:
                    return False
                retired.add(old.epoch)
                history.clear()
                self._relative.clear()
                self._previous_motion = None
                self.frame_generation += 1
            elif root.sequence <= old.sequence:
                return False
        positive = self._positive_frontiers.get(o.source)
        if (
            o.quality > 0
            and positive is not None
            and o.roots[0].epoch == positive.roots[0].epoch
            and o.stamp_ros <= positive.stamp_ros
        ):
            return False
        if o.quality == 0:
            history.clear()
        else:
            self._positive_frontiers[o.source] = o
        history.append(o)
        return True

    def _live(self, roots: tuple[Origin, ...], ros: float, steady: float) -> Age | None:
        p = default_policy()
        ages = [
            p.age(
                stamp_ros=r.stamp_ros,
                received_ros=r.received_ros,
                received_steady=r.received_steady,
                now_ros=ros,
                now_steady=steady,
            )
            for r in roots
        ]
        worst = max(ages, key=lambda a: a.seconds)
        return None if worst.band == "invalid" else worst

    def advance(self, motion: MotionEstimate) -> None:
        if (
            self._previous_motion is not None
            and motion.generation != self._previous_motion.generation
        ):
            self._relative.clear()
            retained = {}
            for source, history in self._sources.items():
                live = []
                for o in history:
                    if o.roots[0].epoch == self._clock_epochs.get(source):
                        continue
                    try:
                        if (
                            o.quality > 0
                            and self._live(o.roots, motion.computed_ros, motion.computed_steady)
                            is not None
                        ):
                            live.append(o)
                    except ValueError:
                        continue
                if live:
                    retained[source] = deque(live, maxlen=128)
            self._sources = retained
            self._absolute_frame_aligned = False
            self._previous_motion = None
        eligible = []
        self._clock_epochs = {}
        for source, history in self._sources.items():
            for o in history:
                try:
                    if (
                        o.quality > 0
                        and self._live(o.roots, motion.computed_ros, motion.computed_steady)
                        is not None
                    ):
                        eligible.append(o)
                        self._clock_epochs[source] = o.roots[0].epoch
                except ValueError:
                    continue
        if not motion.healthy:
            self._previous_motion = motion
            return
        roots = tuple(
            dict.fromkeys(r for a in (motion.vx, motion.wz) if a.observed for r in a.roots)
        )
        # Keep dead reckoning in the same coordinate origin as observed poses.
        # A switch to relative prediction must not reset odom to (0,0,0).
        try:
            absolute = self.at(
                motion.computed_ros,
                now_ros=motion.computed_ros,
                now_steady=motion.computed_steady,
                motion=motion,
            )
        except ValueError:
            absolute = None
        if absolute is not None and absolute.kind != "relative_dead_reckoning":
            if not self._absolute_frame_aligned and self._relative:
                self.frame_generation += 1
                self._relative.clear()
            self._absolute_frame_aligned = True
            if not self._relative:
                raw_stamp = max(o.stamp_ros for o in eligible if o.stamp_ros <= motion.computed_ros)
                raw_anchor = self.at(
                    raw_stamp,
                    now_ros=motion.computed_ros,
                    now_steady=motion.computed_steady,
                    motion=motion,
                )
                self._relative.append(raw_anchor)
            self._relative.append(
                PoseEstimate(
                    absolute.pose,
                    motion.computed_ros,
                    absolute.roots,
                    "relative_dead_reckoning",
                    absolute.error_m,
                    self.frame,
                    absolute.error_yaw_rad,
                )
            )
            self._previous_motion = motion
            return
        if not self._relative:
            self._relative.append(
                PoseEstimate(
                    Pose2D(0, 0, 0),
                    min(r.stamp_ros for r in roots),
                    roots,
                    "relative_dead_reckoning",
                    0.0,
                    self.frame,
                )
            )
        previous = self._relative[-1]
        dt = motion.computed_ros - previous.stamp_ros
        if dt < 0:
            self._relative.clear()
            return
        if dt == 0:
            self._previous_motion = motion
            return
        # No axis receives an infinite extension from the other axis. The motion
        # owner has already removed expired per-axis values; missing motion grows error.
        prior = self._previous_motion or motion
        full_dt = dt
        dt = min(dt, default_policy().hard_age_s)
        # Integrate each axis only over its own remaining original lifetime.
        start_steady = motion.computed_steady - dt
        axis_dt = [
            max(0.0, min(dt, a.deadline_steady - start_steady)) if a.value is not None else 0.0
            for a in (prior.vx, prior.vy, prior.wz)
        ]
        rates = [
            (a.value or 0.0) * d / dt
            for a, d in zip((prior.vx, prior.vy, prior.wz), axis_dt, strict=True)
        ]
        increment = exponential(rates[0], rates[1], rates[2], dt)
        missing_dt = max(full_dt - axis_dt[0], full_dt - axis_dt[2])
        error = (
            None
            if previous.error_m is None
            or any(a.observed and a.error is None for a in (prior.vx, prior.wz))
            else (previous.error_m + self.unknown_speed * missing_dt)
        )
        self._relative.append(
            PoseEstimate(
                compose(previous.pose, increment),
                motion.computed_ros,
                roots,
                "relative_dead_reckoning",
                error,
                self.frame,
            )
        )
        self._previous_motion = motion

    def at(
        self,
        stamp_ros: float,
        *,
        now_ros: float,
        now_steady: float,
        motion: MotionEstimate | None = None,
    ) -> PoseEstimate:
        if not math.isfinite(stamp_ros) or not 0 <= stamp_ros <= now_ros:
            raise ValueError("invalid pose query time")
        candidates: list[_PoseCandidate] = []
        for history in self._sources.values():
            live = []
            for observation in history:
                try:
                    if (
                        observation.quality > 0
                        and self._live(observation.roots, now_ros, now_steady) is not None
                    ):
                        live.append(observation)
                except ValueError:
                    continue
            left = [o for o in live if o.stamp_ros <= stamp_ros]
            if not left:
                continue
            a = left[-1]
            right = next((o for o in live if o.stamp_ros > stamp_ros), None)
            pose, roots, kind = a.pose, a.roots, "observed_pose"
            errors = (a.error_m, a.error_m, a.error_yaw_rad)
            alignment = (0.0, 0.0, 0.0)
            quality = a.quality
            raw_time = a.stamp_ros
            if right is not None:
                fraction = (stamp_ros - a.stamp_ros) / (right.stamp_ros - a.stamp_ros)
                pose = Pose2D(
                    a.pose.x + (right.pose.x - a.pose.x) * fraction,
                    a.pose.y + (right.pose.y - a.pose.y) * fraction,
                    angle(a.pose.yaw + angle(right.pose.yaw - a.pose.yaw) * fraction),
                )
                roots = tuple(dict.fromkeys(a.roots + right.roots))
                kind = "interpolated_pose"
                linear = (
                    None
                    if a.error_m is None or right.error_m is None
                    else max(a.error_m, right.error_m)
                )
                angular = (
                    None
                    if a.error_yaw_rad is None or right.error_yaw_rad is None
                    else max(a.error_yaw_rad, right.error_yaw_rad)
                )
                errors = (linear, linear, angular)
                quality = min(a.quality, right.quality)
                raw_time = right.stamp_ros
            elif stamp_ros > a.stamp_ros:
                if motion is None or not motion.healthy:
                    continue
                dt = stamp_ros - a.stamp_ros
                if dt >= default_policy().hard_age_s:
                    continue
                pose = compose(
                    a.pose,
                    exponential(
                        motion.vx.value or 0.0, motion.vy.value or 0.0, motion.wz.value or 0.0, dt
                    ),
                )
                # Prediction retains both the absolute fix and every motion input
                # actually used; shared heading/odometry cannot become extra votes.
                dependencies = tuple(
                    r
                    for axis in (motion.vx, motion.vy, motion.wz)
                    if axis.value is not None
                    for r in axis.roots
                )
                roots = tuple(dict.fromkeys(a.roots + dependencies))
                kind = "predicted_pose"
                linear = (
                    None
                    if a.error_m is None
                    or motion.vx.error is None
                    or (motion.vy.observed and motion.vy.error is None)
                    else (a.error_m + dt * (motion.vx.error + (motion.vy.error or 0.0)))
                )
                angular = (
                    None
                    if a.error_yaw_rad is None or motion.wz.error is None
                    else a.error_yaw_rad + dt * motion.wz.error
                )
                errors = (linear, linear, angular)
                # A weighting penalty is not a hard error certificate. Unknown
                # measurement errors remain None and gain no consistency padding.
                alignment = (
                    self.unknown_speed * dt,
                    self.unknown_speed * dt,
                    abs(motion.wz.value or 0.0) * dt,
                )
            try:
                if self._live(roots, now_ros, now_steady) is None:
                    continue
            except ValueError:
                continue
            candidates.append(
                _PoseCandidate(a, pose, roots, kind, errors, alignment, quality, raw_time)
            )
        if candidates:
            anchor = max(candidates, key=lambda c: c.observation.stamp_ros).pose.yaw
            results = []
            selected: set[int] = set()
            for axis in range(3):
                votes = []
                for c in candidates:
                    age = self._live(c.roots, now_ros, now_steady)
                    assert age is not None
                    value = (c.pose.x, c.pose.y, anchor + angle(c.pose.yaw - anchor))[axis]
                    error = c.errors[axis]
                    quality = c.quality * (0.5 if error is None else 1.0)
                    weight = (
                        quality
                        * age.freshness
                        / (1 + (c.alignment[axis] / self.tolerances[axis]) ** 2)
                    )
                    votes.append(
                        Vote(
                            value,
                            c.raw_time,
                            c.observation.source,
                            weight,
                            quality,
                            quality * age.freshness,
                            age.band == "low",
                            c.alignment[axis],
                            error,
                            frozenset(r.family for r in c.roots),
                        )
                    )
                result = combine(tuple(votes), self.tolerances[axis])
                results.append(result)
                selected.update(result.indices)
            roots = tuple(dict.fromkeys(r for i in sorted(selected) for r in candidates[i].roots))
            linear = (
                None
                if any(results[i].error is None for i in (0, 1))
                else max(results[i].error or 0.0 for i in (0, 1))
            )
            sources = tuple(sorted(candidates[i].observation.source for i in selected))
            kind = "fused_pose" if len(selected) > 1 else candidates[next(iter(selected))].kind
            return PoseEstimate(
                Pose2D(results[0].value, results[1].value, angle(results[2].value)),
                stamp_ros,
                roots,
                kind,
                linear,
                self.frame,
                results[2].error,
                (results[0].confidence, results[1].confidence, results[2].confidence),
                sources,
            )
        # Relative integration is not an absolute-fix freshness claim.
        if motion is not None and motion.healthy:
            relative_left = [p for p in self._relative if p.stamp_ros <= stamp_ros]
            if relative_left:
                relative = relative_left[-1]
                dt = stamp_ros - relative.stamp_ros
                if 0 <= dt < default_policy().hard_age_s:
                    return PoseEstimate(
                        compose(
                            relative.pose,
                            exponential(
                                motion.vx.value or 0.0,
                                motion.vy.value or 0.0,
                                motion.wz.value or 0.0,
                                dt,
                            ),
                        ),
                        stamp_ros,
                        relative.roots,
                        "relative_dead_reckoning",
                        relative.error_m,
                        self.frame,
                    )
        raise ValueError("pose result cannot support requested time/frame")
