"""Depth candidates must independently veto motion without inventing coverage."""

from types import SimpleNamespace

import pytest

from senior_control.models import Pose2D
from senior_control.safety.motion_runtime import TimedPoseBuffer, TimedPoseSample


def evidence(x=0.6316, y=0.00224, z=0.0282):
    return SimpleNamespace(
        stamp=SimpleNamespace(sec=1, nanosec=0),
        frame_id="camera_optical_frame",
        body_frame="base_link",
        width=3,
        height=3,
        row_stride=1,
        column_stride=1,
        fx=2.0,
        fy=4.0,
        cx=1.0,
        cy=1.0,
        min_range_m=0.1,
        max_range_m=8.0,
        range_error_m=0.01,
        sensor_origin=(0.2316, 0.00224, 0.1282),
        optical_to_body_quaternion=(-0.5, 0.5, -0.5, 0.5),
        samples=[SimpleNamespace(u=1, v=2, depth_m=0.4, x=x, y=y, z=z)],
    )


def history():
    result = TimedPoseBuffer(max_gap=1.0, max_samples=8, max_variance=0.1)
    assert result.add(TimedPoseSample(1.0, Pose2D(0.0, 0.0, 0.0), (0.0001,) * 3, "motion"))
    assert result.add(TimedPoseSample(1.1, Pose2D(0.02, 0.0, 0.0), (0.0001,) * 3, "motion"))
    return result


def evaluate(message, poses=None):
    from senior_control.safety.depth_geometry import evaluate_depth_candidates

    return evaluate_depth_candidates(
        message,
        history() if poses is None else poses,
        now=1.1,
        max_age=0.5,
        max_reference_lag=0.3,
        linear_bound=0.3,
        yaw_bound=0.5,
        radius=1.0,
        footprint=((-0.337, -0.276), (0.337, -0.276), (0.337, 0.276), (-0.337, 0.276)),
        collision_z=(-0.06, 0.14),
        extra_margin=0.3,
    )


def test_low_hit_is_motion_aligned_and_vetoes():
    result = evaluate(evidence())
    assert result.reason == "depth_hit"
    assert result.aligned_points[0][0] == pytest.approx(0.6116)
    assert result.aligned_points[0][2] == pytest.approx(0.0282)
    assert not result.coverage_qualified


def test_valid_distant_or_occluded_samples_do_not_prove_3d_free():
    msg = evidence()
    # A different valid ray endpoint, still no proof of the rest of the volume.
    msg.samples = [SimpleNamespace(u=1, v=1, depth_m=4.0, x=4.2316, y=0.00224, z=0.1282)]
    assert evaluate(msg).reason == "depth_coverage_unknown"
    assert not evaluate(msg).coverage_qualified


def test_new_unaligned_and_inconsistent_geometry_stop():
    unavailable = TimedPoseBuffer(max_gap=1.0, max_samples=8, max_variance=0.1)
    assert unavailable.add(TimedPoseSample(0.9, Pose2D(0.0, 0.0, 0.0), (0.0001,) * 3, "m"))
    assert evaluate(evidence(), unavailable).reason == "depth_motion_unaligned"
    assert evaluate(evidence(x=1.0)).reason == "depth_geometry_invalid"


def test_required_depth_is_evaluated_inside_safety_worker_and_never_grants_unproven_space():
    from senior_control.config import DeviceProfile
    from senior_control.models import BodyTwist, Velocity
    from senior_control.safety.async_guard import SafetyWork, evaluate_safety_work
    from senior_control.safety.velocity_uncertainty import IntervalVelocityBounds

    profile = DeviceProfile(
        footprint=[(-0.337, -0.276), (0.337, -0.276), (0.337, 0.276), (-0.337, 0.276)],
        self_mask_calibrated=True,
        braking_deceleration=0.3,
        lateral_braking_deceleration=0.3,
        yaw_braking_deceleration=0.5,
        reaction_time=0.2,
        hard_margin=0.02,
        observation_max_age=0.5,
    )
    work = SafetyWork(
        Velocity(0.05, 0.0),
        BodyTwist(0.0, 0.0, 0.0),
        profile,
        (),
        history(),
        1.1,
        10.0,
        0.3,
        0.3,
        0.5,
        1.0,
        False,
        interval_bounds=IntervalVelocityBounds(0.1, 0.1),
        physical_cap_qualified=True,
        depth_required=True,
        depth_message=evidence(),
        depth_collision_z=(-0.06, 0.14),
    )
    result = evaluate_safety_work(work)
    assert not result.decision.allowed and result.decision.reason == "depth_hit"
    assert result.diagnostics["depth_aligned_samples"] == 1
    assert result.diagnostics["depth_coverage_qualified"] is False
    work.depth_message.samples = []
    assert evaluate_safety_work(work).decision.reason == "depth_geometry_invalid"
