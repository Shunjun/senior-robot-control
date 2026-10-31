"""Quality-gated body motion from an independent scan match.

Wheel odometry is compared with independent body motion. Ground truth is not
an input here. A loop-closure jump is not motion. Historical-map reverse
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
    """Independent displacement and interval-average body-frame SE(2) twist."""

    pose_delta: Pose2D
    velocity: Velocity
    covariance: tuple[float, float, float]
    quality: float
    dt: float = 0.0
    source: str = ""
    stamp: float = float("nan")
    degenerate: bool = True
    aligned: bool = False
    max_covariance: float | None = None


def body_twist_delta(delta: Pose2D) -> Pose2D:
    """SE(2) logarithm: integrated body twist for a measured relative pose."""
    theta = math.atan2(math.sin(delta.yaw), math.cos(delta.yaw))
    if abs(theta) < 1e-8:
        a, b = 1.0, theta / 2.0
    else:
        a = math.sin(theta) / theta
        b = 2.0 * math.sin(theta / 2.0) ** 2 / theta
    denominator = a * a + b * b
    if denominator <= 1e-12:
        raise ValueError("unobservable body twist")
    return Pose2D(
        (a * delta.x + b * delta.y) / denominator,
        (-b * delta.x + a * delta.y) / denominator,
        theta,
    )


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
            or wheel_odom.dt <= 0
            or not all(math.isfinite(dt) for dt in (scan_motion.dt, imu.dt, wheel_odom.dt))
            or not self._covariance_accepted(scan_motion.covariance)
            or not _aligned(scan_motion.stamp, now, self.max_skew)
            or not _aligned(imu.stamp, scan_motion.stamp, self.max_skew)
            or not _aligned(wheel_odom.stamp, scan_motion.stamp, self.max_skew)
            or not _aligned(imu.stamp - imu.dt, scan_motion.stamp - scan_motion.dt, self.max_skew)
            or not _aligned(
                wheel_odom.stamp - wheel_odom.dt,
                scan_motion.stamp - scan_motion.dt,
                self.max_skew,
            )
            or not math.isfinite(imu.yaw_delta)
            or abs(
                math.atan2(
                    math.sin(scan_motion.pose_delta.yaw - imu.yaw_delta),
                    math.cos(scan_motion.pose_delta.yaw - imu.yaw_delta),
                )
            )
            > self.max_yaw_error
        ):
            return _unusable()
        if visual_motion is not None and self._visual_conflicts(scan_motion, visual_motion):
            return _unusable()
        delta = scan_motion.pose_delta
        twist_delta = body_twist_delta(delta)
        return MotionEstimate(
            delta,
            Velocity(twist_delta.x / scan_motion.dt, twist_delta.yaw / scan_motion.dt),
            scan_motion.covariance,
            scan_motion.quality,
            scan_motion.dt,
            source=scan_motion.source,
            stamp=scan_motion.stamp,
            degenerate=False,
            aligned=True,
            max_covariance=self.max_covariance,
        )


def supports_degraded_reverse(
    estimate: MotionEstimate, *, now: float | None = None, max_age: float | None = None
) -> bool:
    """Independent scan match only. Wheel-wrapped numbers do not qualify a reverse."""
    cap = estimate.max_covariance
    if (
        estimate.source != _INDEPENDENT_SOURCE
        or estimate.degenerate
        or not estimate.aligned
        or estimate.quality <= 0.0
        or not math.isfinite(estimate.quality)
        or estimate.dt <= 0.0
        or not math.isfinite(estimate.dt)
        or cap is None
        or not math.isfinite(cap)
        or cap < 0.0
        or not math.isfinite(estimate.stamp)
        or any(
            not math.isfinite(value) or value < 0.0 or value > cap for value in estimate.covariance
        )
    ):
        return False
    if now is None and max_age is None:
        return True
    if (
        now is None
        or max_age is None
        or not math.isfinite(now)
        or not math.isfinite(max_age)
        or max_age < 0.0
        or not 0.0 <= now - estimate.stamp <= max_age
    ):
        return False
    return True


def degraded_reverse_gate(estimate: MotionEstimate) -> SafetyDecision:
    if not supports_degraded_reverse(estimate):
        return SafetyDecision(False, Velocity(0.0, 0.0), "independent_motion_unavailable")
    return SafetyDecision(False, Velocity(0.0, 0.0), "map_qualification_deferred")


def body_displacement(estimate: MotionEstimate) -> float:
    """Distance budget from accepted body motion, never from wheel integration."""
    if estimate.quality <= 0.0:
        return 0.0
    return math.hypot(estimate.pose_delta.x, estimate.pose_delta.y)
