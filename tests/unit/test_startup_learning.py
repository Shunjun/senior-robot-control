from dataclasses import replace

from test_startup_learning_feasibility import BOUNDARY, forward_scan

from senior_control.config import DeviceProfile
from senior_control.models import Pose2D
from senior_control.motion.startup_learning import StartupLearning
from senior_control.safety.self_filter import filter_self
from senior_control.safety.self_learning import SelfReturnModel


def observation(distance):
    return filter_self(forward_scan(distance), DeviceProfile(footprint=BOUNDARY))


def side_observation(distance):
    import math

    from test_startup_learning_feasibility import forward_scan

    scan = replace(
        forward_scan(distance),
        ranges=tuple([2.0] * 180 + [distance] + [2.0] * 180),
        angle_min=0.0,
        angle_increment=math.pi / 360,
        interpolate=True,
        max_angular_gap=0.01,
    )
    return filter_self(scan, DeviceProfile(footprint=BOUNDARY))


def record(model, obs, pose):
    # Distinct capture timestamps for the pure model's moving-frame fixtures.
    model.observe(replace(obs, scan=replace(obs.scan, stamp=10 + pose.x)), pose)


def test_provisional_is_endpoint_local_and_preserves_occlusion():
    m = SelfReturnModel(BOUNDARY, error_m=0.01)
    result = m.filter(observation(0.15), provisional=True)
    assert not result.external_hits and not result.free_rays
    assert result.occluded_regions
    assert m.filter(observation(1), provisional=True).external_hits
    assert m.filter(observation(0.295), provisional=True).external_hits
    assert m.filter(observation(0.15), provisional=False).external_hits


def test_model_requires_independent_motion_and_world_hypothesis_rejection():
    m = SelfReturnModel(BOUNDARY, error_m=0.005)
    for i in range(12):
        record(m, side_observation(0.15), Pose2D(i * 0.015, 0, 0))
    assert m.validate(independent_motion=False) == 0
    assert m.validate(independent_motion=True) == 1
    assert m.filter(side_observation(0.15), provisional=False).rays[180].kind == "self"
    assert m.filter(side_observation(0.10), provisional=False).rays[180].kind == "hit"
    assert m.filter(side_observation(0.15), provisional=False).rays[180].kind == "hit"  # revoked


def test_parallel_surface_and_repeated_points_remain_ambiguous():
    m = SelfReturnModel(BOUNDARY, error_m=0.01)
    for _ in range(12):
        record(m, observation(0.15), Pose2D(0, 0, 0))
    assert m.validate(independent_motion=True) == 0


def test_once_only_claim_and_interruption(tmp_path):
    path = tmp_path / "run.claim"
    a = StartupLearning(path)
    assert a.state == "initializing"
    b = StartupLearning(path)
    assert b.state == "interrupted"
    a.interrupt("estop")
    assert not a.provisional and a.state == "interrupted"
    assert StartupLearning(path).state == "interrupted"


def test_program_start_context_is_explicit_and_run_owned(tmp_path):
    from senior_control.motion.startup_learning import start_environment

    env = start_environment({}, tmp_path)
    assert "SENIOR_STARTUP_LEARNING_CLAIM" in env
    assert not (tmp_path / env["SENIOR_STARTUP_LEARNING_CLAIM"]).exists()
    assert StartupLearning(None).state == "disabled"


def test_world_compatible_repeated_returns_never_become_learned():
    from senior_control.safety.self_filter import RayEvidence

    m = SelfReturnModel(BOUNDARY, error_m=0.005)
    base = observation(0.15)
    # A repeated surface contains a body-stable point AND its world-fixed
    # prediction at the first frame; all hits participate in the world test.
    rays = tuple(RayEvidence(i, (0, 0), (0.15 + i * 0.015, 0), "hit") for i in range(20))
    frame = replace(base, rays=rays, external_hits=rays)
    for i in range(12):
        record(m, frame, Pose2D(i * 0.015, 0, 0))
    assert m.validate(independent_motion=True) == 0


def test_outside_following_object_and_device_removal_do_not_leave_a_mask():
    m = SelfReturnModel(BOUNDARY, error_m=0.005)
    for i in range(12):
        record(m, observation(1), Pose2D(i * 0.015, 0, 0))
    assert m.validate(independent_motion=True) == 0
    assert m.filter(observation(1), provisional=True).external_hits


def test_owner_finishes_once_and_marks_wheel_only_motion_ambiguous(tmp_path):
    from types import SimpleNamespace as N

    from senior_control.motion.estimator import Origin

    owner = StartupLearning(tmp_path / "claim")
    m = SelfReturnModel(BOUNDARY, error_m=0.005)

    def estimate(i, vx):
        axis = N(
            value=vx,
            observed=True,
            roots=(Origin("wheel", "a", i, i, i, i),),
            deadline_steady=i + 1,
        )
        return N(
            computed_ros=i / 10,
            healthy=True,
            generation=1,
            vx=axis,
            wz=N(value=0, observed=True, roots=axis.roots, deadline_steady=i + 1),
            independent_axes=None,
        )

    for i in range(1, 11):
        owner.advance(estimate(i, 0), 0.0, now=i / 10, stationary=True)
        obs = observation(0.15)
        owner.frame(m, replace(obs, scan=replace(obs.scan, stamp=i / 10)))
    assert owner.state == "moving"
    owner.motion_emitted(1.0)
    for i in range(11, 42):
        owner.advance(estimate(i, 0.05), 0.05, now=i / 10, stationary=False)
        obs = observation(0.15)
        owner.frame(m, replace(obs, scan=replace(obs.scan, stamp=i / 10)))
    assert owner.distance >= 0.148
    for i in range(42, 48):
        owner.advance(estimate(i, 0), 0.0, now=i / 10, stationary=True)
        obs = observation(0.15)
        owner.frame(m, replace(obs, scan=replace(obs.scan, stamp=i / 10)))
    assert owner.state == "ambiguous" and not owner.provisional
    assert owner.velocity == 0 and not m.learned
    assert StartupLearning(tmp_path / "claim").state == "interrupted"


def test_collision_footprint_preserves_actual_physical_polygon():
    from senior_control.motion.constraints import Footprint

    polygon = ((-0.3, -0.2), (0.4, -0.2), (0.4, 0.2), (-0.3, 0.2))
    shape = Footprint(0.6, 0.4, polygon)
    assert shape.physical_vertices == polygon


def test_zero_error_still_requires_strict_interior_and_never_expands_boundary():
    m = SelfReturnModel(BOUNDARY, error_m=0)
    assert m.interior((0.15, 0))
    assert not m.interior((0.3, 0))
    assert not m.interior((0.31, 0))


def test_learned_local_support_never_hides_an_obstacle_behind_equipment():
    m = SelfReturnModel(BOUNDARY, error_m=0.005)
    for i in range(12):
        record(m, side_observation(0.15), Pose2D(i * 0.015, 0, 0))
    assert m.validate(independent_motion=True) == 1
    hidden = m.filter(side_observation(0.15), provisional=False)
    assert all(ray.index != 180 for ray in hidden.free_rays)
    assert m.filter(side_observation(0.25), provisional=False).rays[180].kind == "hit"


def test_unobserved_world_prediction_cannot_be_rejected_behind_the_device():
    m = SelfReturnModel(BOUNDARY, error_m=0.005)
    for i in range(12):
        record(m, observation(0.15), Pose2D(i * 0.015, 0, 0))
    assert m.validate(independent_motion=True) == 0


def test_owner_completes_verified_side_return_after_target_and_post_frames(tmp_path):
    from types import SimpleNamespace as N

    from senior_control.motion.estimator import Origin

    owner = StartupLearning(tmp_path / "claim")
    model = SelfReturnModel(BOUNDARY, error_m=0.005)

    def estimate(i, vx):
        root = Origin("laser", "a", i, i / 10, i / 10, i / 10)

        def axis(value):
            return N(
                value=value,
                observed=True,
                independent=True,
                error=0.0,
                roots=(root,),
                deadline_steady=i / 10 + 1,
            )

        x, y, w = axis(vx), axis(0), axis(0)
        return N(
            computed_ros=i / 10,
            healthy=True,
            generation=1,
            vx=x,
            vy=y,
            wz=w,
            independent_axes=(x, w),
        )

    for i in range(1, 11):
        owner.advance(estimate(i, 0), 0.0, now=i / 10, stationary=True)
        obs = side_observation(0.15)
        owner.frame(model, replace(obs, scan=replace(obs.scan, stamp=i / 10)))
    owner.motion_emitted(1.0)
    for i in range(11, 42):
        owner.advance(estimate(i, 0.05), 0.05, now=i / 10, stationary=False)
        obs = side_observation(0.15)
        owner.frame(model, replace(obs, scan=replace(obs.scan, stamp=i / 10)))
    for i in range(42, 48):
        owner.advance(estimate(i, 0), 0.0, now=i / 10, stationary=True)
        obs = side_observation(0.15)
        owner.frame(model, replace(obs, scan=replace(obs.scan, stamp=i / 10)))
    assert owner.state == "completed"
    assert owner.velocity == 0 and model.learned


def test_matcher_features_exclude_body_points_without_changing_raw_obstacles(tmp_path):
    from senior_control.safety.external_motion_features import (
        external_feature_ranges,
        write_feature_boundary,
    )

    path = tmp_path / "run.claim"
    write_feature_boundary(path, BOUNDARY, 0.01)
    raw = observation(0.15)
    assert external_feature_ranges(raw, path)[0] != external_feature_ranges(raw, path)[0]  # NaN
    assert raw.external_hits  # obstacle path remains unchanged
    assert external_feature_ranges(observation(1.0), path) == (1.0,)


def test_prior_diagnostic_never_claims_verified_translation(tmp_path):
    owner = StartupLearning(tmp_path / "claim")
    assert not owner.summary()["independent_motion_verified"]
    assert owner.summary()["classification"] == "prior"
