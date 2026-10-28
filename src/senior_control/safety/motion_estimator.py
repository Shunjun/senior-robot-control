"""Quality-gated body motion from an independent scan match.

Wheel odometry and ground-truth poses are inputs only. They are not body
displacement. A loop-closure jump is not motion. Historical-map reverse
qualification is not decided here.
"""

import math
from dataclasses import dataclass

from senior_control.models import Pose2D, Velocity
from senior_control.safety.guard import SafetyDecision

_INDEPENDENT_SOURCE = "scan_match"
_REJECTED_SOURCES = frozenset({"wheel", "ground_truth"})


@dataclass(frozen=True)
class RelativeMotion:
    """Pose change from one sensor. Independent body motion uses source ``scan_match``."""

    pose_delta: Pose2D
    covariance: tuple[float, float, float]
    quality: float
    degenerate: bool
    stamp: float
    source: str
    loop_closure: bool
    dt: float


@dataclass(frozen=True)
class WheelOdom:
    pose_delta: Pose2D
    stamp: float
    dt: float


@dataclass(frozen=True)
class ImuYaw:
    yaw_delta: float
    stamp: float
    dt: float


@dataclass(frozen=True)
class MotionEstimate:
    pose_delta: Pose2D
    velocity: Velocity
    covariance: tuple[float, float, float]
    quality: float
    dt: float = 0.0


def _unusable() -> MotionEstimate:
    return MotionEstimate(
        Pose2D(0.0, 0.0, 0.0),
        Velocity(0.0, 0.0),
        (math.inf, math.inf, math.inf),
        0.0,
    )


def _finite_covariance(covariance: tuple[float, float, float]) -> bool:
    return all(math.isfinite(value) and value >= 0.0 for value in covariance)


def _aligned(stamp: float, reference: float, skew: float) -> bool:
    return math.isfinite(stamp) and abs(stamp - reference) <= skew


class MotionEstimator:
    def __init__(
        self,
        *,
        max_skew: float,
        min_quality: float,
        max_yaw_error: float,
        max_covariance: float | None = None,
    ) -> None:
        if (
            max_skew < 0
            or not math.isfinite(max_skew)
            or not 0.0 <= min_quality <= 1.0
            or max_yaw_error < 0
            or not math.isfinite(max_yaw_error)
            or (
                max_covariance is not None
                and (not math.isfinite(max_covariance) or max_covariance < 0.0)
            )
        ):
            raise ValueError("invalid motion estimator limits")
        self.max_skew = max_skew
        self.min_quality = min_quality
        self.max_yaw_error = max_yaw_error
        self.max_covariance = max_covariance

    def _covariance_accepted(self, covariance: tuple[float, float, float]) -> bool:
        if self.max_covariance is None:
            return False
        return all(
            math.isfinite(value) and 0.0 <= value <= self.max_covariance for value in covariance
        )

    def _visual_conflicts(self, scan: RelativeMotion, visual: RelativeMotion) -> bool:
        if (
            visual.source in _REJECTED_SOURCES
            or not visual.source.strip()
            or visual.degenerate
            or visual.loop_closure
            or visual.quality < self.min_quality
            or visual.dt <= 0
            or not _finite_covariance(visual.covariance)
            or not _aligned(visual.stamp, scan.stamp, self.max_skew)
        ):
            return True
        deltas = (
            abs(visual.pose_delta.x - scan.pose_delta.x),
            abs(visual.pose_delta.y - scan.pose_delta.y),
            abs(visual.pose_delta.yaw - scan.pose_delta.yaw),
        )
        limits = tuple(
            math.sqrt(scan.covariance[index] + visual.covariance[index]) for index in range(3)
        )
        return any(delta > limit for delta, limit in zip(deltas, limits, strict=False))

    def update(
        self,
        wheel_odom: WheelOdom | None,
        scan_motion: RelativeMotion | None,
        imu: ImuYaw | None,
        visual_motion: RelativeMotion | None,
        now: float,
    ) -> MotionEstimate:
        if (
            scan_motion is None
            or imu is None
            or wheel_odom is None
            or not math.isfinite(now)
            or scan_motion.degenerate
            or scan_motion.loop_closure
            or scan_motion.source != _INDEPENDENT_SOURCE
            or scan_motion.quality < self.min_quality
            or not math.isfinite(scan_motion.quality)
            or scan_motion.dt <= 0
            or imu.dt <= 0
            or not self._covariance_accepted(scan_motion.covariance)
            or not _aligned(scan_motion.stamp, now, self.max_skew)
            or not _aligned(imu.stamp, scan_motion.stamp, self.max_skew)
            or not _aligned(wheel_odom.stamp, scan_motion.stamp, self.max_skew)
            or not math.isfinite(imu.yaw_delta)
            or abs(scan_motion.pose_delta.yaw - imu.yaw_delta) > self.max_yaw_error
        ):
            return _unusable()
        if visual_motion is not None and self._visual_conflicts(scan_motion, visual_motion):
            return _unusable()
        delta = scan_motion.pose_delta
        return MotionEstimate(
            delta,
            Velocity(delta.x / scan_motion.dt, delta.yaw / scan_motion.dt),
            scan_motion.covariance,
            scan_motion.quality,
            scan_motion.dt,
        )


def supports_degraded_reverse(estimate: MotionEstimate) -> bool:
    """Motion-quality precondition only. Map eligibility belongs to a later task."""
    return estimate.quality > 0.0 and all(math.isfinite(value) for value in estimate.covariance)


def degraded_reverse_gate(estimate: MotionEstimate) -> SafetyDecision:
    if not supports_degraded_reverse(estimate):
        return SafetyDecision(False, Velocity(0.0, 0.0), "independent_motion_unavailable")
    return SafetyDecision(False, Velocity(0.0, 0.0), "map_qualification_deferred")


def body_displacement(estimate: MotionEstimate) -> float:
    """Distance budget from accepted body motion, never from wheel integration."""
    if estimate.quality <= 0.0:
        return 0.0
    return math.hypot(estimate.pose_delta.x, estimate.pose_delta.y)
