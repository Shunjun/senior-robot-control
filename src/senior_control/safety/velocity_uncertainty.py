"""Conditional interval-average velocity bounds from a reported pose covariance."""

import math
from dataclasses import dataclass
from fractions import Fraction

from senior_control.models import Pose2D

PHYSICAL_CAP_CONTEXT = "fortress-flat-v1"


@dataclass(frozen=True)
class IntervalVelocityBounds:
    """Unclipped marginal 3-sigma model bounds; neither is an instantaneous cap."""

    linear: float
    yaw: float


def interval_velocity_bounds(
    delta: Pose2D, covariance: tuple[float, ...], dt: float
) -> IntervalVelocityBounds:
    """Bound the SE(2) interval logarithm without assuming independent axes.

    The three marginal 3-sigma intervals are a model assumption, not a
    calibrated joint probability. A measured average does not bound an
    instantaneous velocity without a separately qualified physical model.
    """
    if len(covariance) != 9 or not all(math.isfinite(v) for v in covariance):
        raise ValueError("motion covariance unqualified")
    if not math.isfinite(dt) or dt <= 0:
        raise ValueError("motion interval unqualified")
    xx, xy, xz, yx, yy, yz, zx, zy, zz = covariance
    if min(xx, yy, zz) <= 0:
        raise ValueError("motion covariance marginals unqualified")
    if xy != yx or xz != zx or yz != zy:
        raise ValueError("motion covariance asymmetric")
    # IEEE-754 values have bounded-size exact rational representations. Exact
    # principal minors accept a singular PSD matrix without accepting a
    # slightly indefinite one through a floating-point tolerance.
    sx, sy, sz, sxy, sxz, syz = map(Fraction, (xx, yy, zz, xy, xz, yz))
    if sx * sy < sxy * sxy or sx * sz < sxz * sxz or sy * sz < syz * syz:
        raise ValueError("motion covariance indefinite")
    determinant = (
        sx * sy * sz + 2 * sxy * sxz * syz - sx * syz * syz - sy * sxz * sxz - sz * sxy * sxy
    )
    if determinant < 0:
        raise ValueError("motion covariance indefinite")
    yaw = abs(math.atan2(math.sin(delta.yaw), math.cos(delta.yaw)))
    yaw_upper = yaw + 3 * math.sqrt(zz)
    if yaw_upper >= math.pi:
        raise ValueError("motion logarithm angle unqualified")
    factor = yaw_upper / (2 * math.sin(yaw_upper / 2)) if yaw_upper else 1.0
    position_upper = math.hypot(delta.x, delta.y) + 3 * (math.sqrt(xx) + math.sqrt(yy))
    result = IntervalVelocityBounds(factor * position_upper / dt, yaw_upper / dt)
    if not all(math.isfinite(v) for v in (result.linear, result.yaw)):
        raise ValueError("motion interval uncertainty unqualified")
    return result
