"""Marginal interval bounds are not an instantaneous physical speed guarantee."""

import itertools
import math

import pytest

from senior_control.models import Pose2D
from senior_control.safety.motion_estimator import body_twist_delta
from senior_control.safety.velocity_uncertainty import interval_velocity_bounds


def test_shared_anchor_rank_one_covariance_keeps_unclipped_interval_bound():
    covariance = (0.01, 0.01, 0.0, 0.01, 0.01, 0.0, 0.0, 0.0, 0.00005)
    result = interval_velocity_bounds(Pose2D(0, 0, 0), covariance, 0.1)
    assert result.linear > 6.0
    assert result.yaw == pytest.approx(3 * math.sqrt(0.00005) / 0.1)


def test_covariance_bound_covers_marginal_se2_corner_cases():
    delta = Pose2D(0.02, -0.01, 0.12)
    covariance = (0.0001, 0, 0, 0, 0.0004, 0, 0, 0, 0.000025)
    bound = interval_velocity_bounds(delta, covariance, 0.1)
    for sx, sy, sw in itertools.product((-1, 1), repeat=3):
        sample = Pose2D(
            delta.x + sx * 3 * math.sqrt(covariance[0]),
            delta.y + sy * 3 * math.sqrt(covariance[4]),
            delta.yaw + sw * 3 * math.sqrt(covariance[8]),
        )
        twist = body_twist_delta(sample)
        assert math.hypot(twist.x, twist.y) / 0.1 <= bound.linear
        assert abs(twist.yaw) / 0.1 <= bound.yaw


def test_even_one_float_step_indefinite_is_rejected():
    diagonal = 0.01
    off_diagonal = math.nextafter(diagonal, math.inf)
    covariance = (
        diagonal,
        off_diagonal,
        0.0,
        off_diagonal,
        diagonal,
        0.0,
        0.0,
        0.0,
        0.00005,
    )
    with pytest.raises(ValueError, match="indefinite"):
        interval_velocity_bounds(Pose2D(0, 0, 0), covariance, 0.1)


@pytest.mark.parametrize(
    "covariance, dt, delta",
    [
        ((0.01, 0.02, 0, 0.02, 0.01, 0, 0, 0, 0.001), 0.1, Pose2D(0, 0, 0)),
        ((0.01, 0.005, 0, 0.0, 0.01, 0, 0, 0, 0.001), 0.1, Pose2D(0, 0, 0)),
        ((0.0, 0, 0, 0, 0.01, 0, 0, 0, 0.001), 0.1, Pose2D(0, 0, 0)),
        ((float("nan"), 0, 0, 0, 0.01, 0, 0, 0, 0.001), 0.1, Pose2D(0, 0, 0)),
        ((0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.001), 0.0, Pose2D(0, 0, 0)),
        ((0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.001), 0.1, Pose2D(0, 0, math.pi - 0.01)),
    ],
)
def test_invalid_covariance_interval_or_log_branch_fails_closed(covariance, dt, delta):
    with pytest.raises(ValueError):
        interval_velocity_bounds(delta, covariance, dt)
