"""Independent motion safety regressions from the runtime review."""

import math

import pytest

from senior_control.config import DeviceProfile
from senior_control.models import BodyTwist, Pose2D, Velocity
from senior_control.safety.guard import evaluate_motion
from senior_control.safety.motion_runtime import (
    TimedPoseBuffer,
    TimedPoseSample,
    prepare_safety_observation,
)
from senior_control.safety.self_filter import ScanObservation, filter_self


class LateralProfile(DeviceProfile):
    """Explicit synthetic lateral braking bound; not a hardware calibration."""

    lateral_braking_deceleration: float | None = 0.3


def _profile(**overrides):
    values = dict(
        footprint=[(-0.337, -0.276), (0.337, -0.276), (0.337, 0.276), (-0.337, 0.276)],
        self_mask_calibrated=True,
        braking_deceleration=0.3,
        yaw_braking_deceleration=0.5,
        lateral_braking_deceleration=0.3,
        reaction_time=0.2,
        hard_margin=0.02,
        observation_max_age=0.8,
    )
    values.update(overrides)
    return LateralProfile(**values)


def _scan_with_side_wall(side, *, wall_distance):
    count = 720
    angles = [-math.pi + index * 2 * math.pi / count for index in range(count)]
    wall_y = side * (0.276 + wall_distance)
    ranges = [
        (wall_y - 0.00201) / math.sin(angle) if abs(angle - side * math.pi / 2) < 0.25 else 5.0
        for angle in angles
    ]
    return ScanObservation(
        tuple(ranges),
        -math.pi,
        2 * math.pi / count,
        0.08,
        8.0,
        1.0,
        "ros",
        Pose2D(0.15699, 0.00201, 0.0),
        0.0,
        "base_link",
        1.0,
        True,
        0.01,
        False,
        True,
    )


@pytest.mark.parametrize("side", [-1, 1])
def test_measured_lateral_braking_sweep_stops_for_side_wall(side):
    profile = _profile()
    scan = filter_self(_scan_with_side_wall(side, wall_distance=0.30), profile)
    measured = BodyTwist(vx=0.05, vy=side * 0.39, wz=0.0)
    decision = evaluate_motion(Velocity(0.05, 0), measured, [scan], profile, 1.0)
    # Independently: .39*.2 + .39**2/(2*.3) = .3315 m > .30 m side gap.
    assert not decision.allowed and decision.reason == "hard_sweep_collision"


def test_lateral_motion_requires_an_explicit_braking_bound():
    profile = _profile(lateral_braking_deceleration=None)
    scan = filter_self(_scan_with_side_wall(1, wall_distance=3.0), profile)
    lateral = BodyTwist(vx=0.05, vy=0.01, wz=0.0)
    straight = BodyTwist(vx=0.05, vy=0.0, wz=0.0)
    assert not evaluate_motion(Velocity(0.05, 0), lateral, [scan], profile, 1.0).allowed
    assert evaluate_motion(Velocity(0.05, 0), straight, [scan], profile, 1.0).allowed
    assert not evaluate_motion(
        Velocity(0.05, 0), BodyTwist(0.05, 1e-20, 0), [scan], profile, 1.0
    ).allowed
    assert not evaluate_motion(Velocity(0.05, 0), Velocity(0.05, 0), [scan], profile, 1.0).allowed


@pytest.mark.parametrize("side", [-1, 1])
def test_qualified_lateral_twist_can_pass_when_the_full_sweep_is_clear(side):
    profile = _profile()
    scan = filter_self(_scan_with_side_wall(side, wall_distance=3.0), profile)
    decision = evaluate_motion(
        Velocity(0.05, 0), BodyTwist(0.05, side * 0.1, 0), [scan], profile, 1.0
    )
    assert decision.allowed


def test_aged_rotating_offset_yawed_scan_stops_near_wall_and_allows_far_wall():
    profile = _profile(observation_max_age=1.0)
    count = 720
    history = TimedPoseBuffer(max_gap=1.0, max_samples=3, max_variance=0.1)
    assert history.add(TimedPoseSample(10.0, Pose2D(0, 0, 0), (1e-8,) * 3, "boot"))
    assert history.add(TimedPoseSample(10.8, Pose2D(0.1, 0, math.pi / 6), (1e-8,) * 3, "boot"))

    def decision(front_range):
        ranges = [5.0] * count
        ranges[count // 2] = front_range
        scan = ScanObservation(
            tuple(ranges),
            -math.pi,
            2 * math.pi / count,
            0.02,
            8.0,
            10.0,
            "ros",
            Pose2D(0.2, 0.1, math.pi / 6),
            0.0,
            "base_link",
            10.0,
            True,
            0.01,
            False,
            True,
        )
        prepared = prepare_safety_observation(
            filter_self(scan, profile),
            history,
            now=10.85,
            max_age=1.0,
            max_reference_lag=0.1,
            linear_bound=0.3,
            yaw_bound=0.7,
            radius=0.5,
        )
        assert prepared is not None
        assert prepared.observation.scan.stamp == 10.0
        assert prepared.observation.scan.reference_stamp == 10.8
        # R(-30°) * ((.2,.1)-(.1,0)) = (.13660254,.03660254).
        assert prepared.observation.rays[count // 2].origin == pytest.approx(
            (0.136602540378, 0.036602540378), abs=1e-10
        )
        return evaluate_motion(
            Velocity(0.05, 0),
            BodyTwist(vx=0.05, vy=0.0, wz=0.0),
            [prepared.observation],
            profile,
            10.85,
            uncertainty_margin=prepared.added_margin,
        )

    # At the reference pose the near return is x=.3500, outside the body but in its hard buffer.
    near = decision(0.2134)
    assert not near.allowed and near.reason == "hard_contact"
    assert decision(5.0).allowed
