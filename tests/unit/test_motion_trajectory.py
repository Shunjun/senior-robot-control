"""Synthetic SI bounds, independent finite-difference and end-state checks."""

import pytest

from senior_control.models import Velocity
from senior_control.motion.contracts import ResponseBounds
from senior_control.motion.trajectory import Trajectory, scale_wheels


def response():
    return ResponseBounds(0.4, 0.5, 2, 0.5, 0.6, 2, 0.1, 100, "synthetic")


def test_accelerate_then_stop_obeys_derivatives_and_reaches_zero():
    t = Trajectory(Velocity(0, 0), Velocity(0, 0))
    prior = 0.0
    prior_a = 0.0
    for i in range(500):
        goal = Velocity(0.5, 0) if i < 150 else Velocity(0, 0)
        step = t.propose(goal, response(), 0.02)
        assert abs(step.acceleration.vx - prior_a) <= 2 * 0.02 + 1e-10
        assert -0.5 - 1e-10 <= step.acceleration.vx <= 0.4 + 1e-10
        assert 0 <= step.velocity.vx <= 0.5 + 1e-10
        assert abs(step.velocity.vx - prior) <= 0.5 * 0.02 + 1e-10
        t.commit(step)
        prior = step.velocity.vx
        prior_a = step.acceleration.vx
    assert prior == pytest.approx(0, abs=1e-10)
    assert prior_a == pytest.approx(0, abs=1e-10)


def test_unavoidable_reverse_and_bad_dt_are_explicit_not_clipped():
    t = Trajectory(Velocity(0.001, 0), Velocity(-0.5, 0))
    with pytest.raises(ValueError, match="reverse"):
        t.propose(Velocity(0, 0), response(), 0.02)
    with pytest.raises(ValueError):
        t.propose(Velocity(0, 0), response(), float("nan"))
    assert t.velocity.vx == 0.001


def test_wheel_limit_is_combined_and_preserves_curvature():
    v = scale_wheels(Velocity(0.8, 1), 0.5, 0.6)
    assert max(abs(v.vx - v.wz * 0.25), abs(v.vx + v.wz * 0.25)) <= 0.6 + 1e-12
    assert v.wz / v.vx == pytest.approx(1 / 0.8)
