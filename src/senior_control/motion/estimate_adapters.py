"""Reuse the existing estimator's measured twist without copying its chain."""

from senior_control.motion.observations import (
    Covariance,
    Frame,
    Observation,
    Source,
    SourceStatus,
)
from senior_control.safety.motion_estimator import MotionEstimate


def from_motion_estimate(
    estimate: MotionEstimate,
    *,
    received_steady: float,
    epoch: str,
    sequence: int,
    velocity_covariance: Covariance,
) -> Observation:
    """Caller must supply velocity covariance with an explicit basis.

    MotionEstimate.covariance describes displacement. Dividing its diagonal by
    dt squared would not establish the nonlinear SE(2) twist/correlation model;
    T3a deliberately does not infer that conversion or a deterministic bound.
    """
    twist = estimate.body_twist
    usable = (
        twist is not None
        and estimate.aligned
        and not estimate.degenerate
        and estimate.source == "scan_match"
    )
    values = () if twist is None else (twist.vx, twist.vy, twist.wz)
    return Observation(
        Source.LASER,
        Frame.BODY,
        estimate.stamp - estimate.dt,
        estimate.stamp,
        received_steady,
        epoch,
        sequence,
        estimate.quality,
        SourceStatus.VALID if usable else SourceStatus.UNAVAILABLE,
        values,
        velocity_covariance,
    )
