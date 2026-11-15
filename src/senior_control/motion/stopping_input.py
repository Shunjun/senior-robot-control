"""Software stopping-risk policy over the single current-motion estimate.

These intervals are assumptions for prediction, never measured physical guarantees.
Independent stillness/reset evidence is a separate concern.
"""

from senior_control.models import Velocity
from senior_control.motion.estimator import AxisEstimate, MotionEstimate


def _axis_domain(
    axis: AxisEstimate, trajectory: float, measurement_error: float
) -> tuple[float, float]:
    # Total known error already contains alignment/age and source disagreement.
    # Unknown measurement error uses the deployment assumption plus that same
    # projection ledger once. A bounded model-only hold retains its original TTL.
    error = max(measurement_error + axis.prediction_error, axis.error or 0.0)
    if axis.value is None:
        # This axis has no live estimate: the output owner closes its target.
        # Keep residual trajectory motion for braking, without inventing a sample
        # or renewing expired support from another axis.
        return min(0.0, trajectory - error), max(0.0, trajectory + error)
    low, high = axis.value - error, axis.value + error
    # A zero command is a goal, not evidence of stillness. Including it in a
    # strictly forward measured domain would erase reverse_requires_stop.
    return (min(low, trajectory), max(high, trajectory)) if trajectory else (low, high)


def stopping_domains(
    estimate: MotionEstimate,
    trajectory: Velocity,
    *,
    linear_error: float,
    yaw_error: float,
) -> tuple[tuple[float, float], tuple[float, float]]:
    """Use fused signed vx/wz, including wheel-only and bounded axis holds.

    Missing vy remains the estimator's explicit zero assumption; it does not
    invalidate these axes. The deployment lateral-drift policy stays downstream.
    """
    return (
        _axis_domain(estimate.vx, trajectory.vx, linear_error),
        _axis_domain(estimate.wz, trajectory.wz, yaw_error),
    )
