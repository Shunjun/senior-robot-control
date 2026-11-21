from dataclasses import replace

from learning_test_support import (  # noqa: F401
    begin_action,
    bind_core,
    learning_runs,
    startup_for,
    write_geometry_record,
)
from senior_perception.startup_learning import StartupLearning
from sensor_time_test_support import sample_time
from test_startup_learning_feasibility import BOUNDARY, forward_scan

from senior_control.config import DeviceProfile
from senior_control.safety.self_filter import filter_self
from senior_control.safety.self_learning import SelfReturnModel
from senior_control.vision.raw_capture import RawCaptureHeader


def observation(distance):
    return filter_self(forward_scan(distance), DeviceProfile(footprint=BOUNDARY))


def side_observation(distance):
    import math

    from test_startup_learning_feasibility import forward_scan

    scan = replace(
        forward_scan(distance),
        ranges=tuple([2.0] * 179 + [float("nan"), distance, float("nan")] + [2.0] * 179),
        angle_min=0.0,
        angle_increment=math.pi / 360,
        interpolate=True,
        max_angular_gap=0.01,
    )
    return filter_self(scan, DeviceProfile(footprint=BOUNDARY))


def record(model, obs):
    sequence = model._max_sequence + 1
    model.observe(
        obs,
        capture=RawCaptureHeader(
            time=sample_time(
                stamp_ns=round((10 + sequence * 0.1) * 1e9),
                received_ros_ns=round((10 + sequence * 0.1) * 1e9),
                received_steady_ns=round((10 + sequence * 0.1) * 1e9),
                clock_generation=1,
            ),
            frame_id="laser",
            raw_gid="01",
            bridge_gid="02",
            source_epoch="model-test",
            bridge_boot_id="boot",
            capture_generation=1,
            sequence=sequence,
            valid=True,
            reason="",
        ),
        segment_id="model-test",
    )


def test_provisional_is_endpoint_local_and_preserves_occlusion():
    m = SelfReturnModel(BOUNDARY)
    result = m.filter(observation(0.15), provisional=True)
    assert not result.external_hits and not result.free_rays
    assert result.occluded_regions
    assert m.filter(observation(1), provisional=True).external_hits
    assert not m.filter(observation(0.295), provisional=True).external_hits
    assert m.filter(observation(0.30), provisional=True).external_hits
    assert m.filter(observation(0.15), provisional=False).external_hits


def test_model_uses_body_position_stability_and_freezes_reference():
    m = SelfReturnModel(BOUNDARY)
    for _i in range(12):
        record(m, side_observation(0.15))
    assert m.validate() == 1
    assert m.filter(side_observation(0.15), provisional=False).rays[180].kind == "self"
    assert m.filter(side_observation(0.10), provisional=False).rays[180].kind == "hit"
    # The frozen reference survives occlusion.
    assert m.filter(side_observation(0.15), provisional=False).rays[180].kind == "self"


def test_stable_interior_returns_do_not_require_displacement():
    m = SelfReturnModel(BOUNDARY)
    for _ in range(12):
        record(m, observation(0.15))
    assert m.validate() == 1


def test_consumed_record_restores_without_replaying_possible_motion(tmp_path):
    path = tmp_path / "run-record"
    first = startup_for(path)
    first.state = "moving"
    first.advance(0.0, now=10.0, stationary=False)
    assert first.prepare_action()
    first.session.close()
    restored = startup_for(path)
    restored.advance(0.0, now=10.1, stationary=True)
    assert restored.velocity == 0 and restored.motion_withdrawn
    assert not restored.prepare_action()


def test_program_start_context_is_explicit_and_run_owned(tmp_path):
    from senior_perception.startup_learning import start_environment

    env = start_environment({}, tmp_path)
    assert "SENIOR_STARTUP_LEARNING_CLAIM" in env
    assert not (tmp_path / env["SENIOR_STARTUP_LEARNING_CLAIM"]).exists()
    assert StartupLearning(None).state == "disabled"


def test_stable_connected_returns_do_not_require_world_hypothesis():
    from senior_control.safety.self_filter import RayEvidence

    m = SelfReturnModel(BOUNDARY)
    base = observation(0.15)
    # A repeated surface contains a body-stable point AND its world-fixed
    # prediction at the first frame; all hits participate in the world test.
    rays = tuple(RayEvidence(i, (0, 0), (0.15 + i * 0.015, 0), "hit") for i in range(20))
    frame = replace(base, rays=rays, external_hits=rays)
    for _i in range(12):
        record(m, frame)
    assert m.validate() == 20


def test_outside_following_object_and_device_removal_do_not_leave_a_mask():
    m = SelfReturnModel(BOUNDARY)
    for _i in range(12):
        record(m, observation(1))
    assert m.validate() == 0
    assert m.filter(observation(1), provisional=True).external_hits


def test_collision_footprint_preserves_actual_physical_polygon():
    from senior_control.motion.constraints import Footprint

    polygon = ((-0.3, -0.2), (0.4, -0.2), (0.4, 0.2), (-0.3, 0.2))
    shape = Footprint(0.6, 0.4, polygon)
    assert shape.physical_vertices == polygon


def test_geometry_requires_strict_interior_and_never_expands_boundary():
    m = SelfReturnModel(BOUNDARY)
    assert m.interior((0.15, 0))
    assert not m.interior((0.3, 0))
    assert not m.interior((0.31, 0))


def test_learned_local_support_never_hides_an_obstacle_behind_equipment():
    m = SelfReturnModel(BOUNDARY)
    for _i in range(12):
        record(m, side_observation(0.15))
    assert m.validate() == 1
    hidden = m.filter(side_observation(0.15), provisional=False)
    assert all(ray.index != 180 for ray in hidden.free_rays)
    assert m.filter(side_observation(0.25), provisional=False).rays[180].kind == "hit"


def test_stable_body_return_needs_no_world_free_space():
    m = SelfReturnModel(BOUNDARY)
    for _i in range(12):
        record(m, observation(0.15))
    assert m.validate() == 1


def test_matcher_features_exclude_body_points_without_changing_raw_obstacles(tmp_path):
    from learning_test_support import geometry

    from senior_control.motion.learning_record import LearnedCatalog, LearningAnchor
    from senior_control.safety.external_motion_features import (
        external_feature_ranges,
    )
    from senior_control.safety.learning_view import LearningFilterView

    raw = observation(0.15)
    view = LearningFilterView(
        LearnedCatalog(model_id="a" * 32, anchors=(LearningAnchor(id=0, point=(0.15, 0.0)),)),
        geometry(BOUNDARY),
    )
    assert external_feature_ranges(raw, view)[0] != external_feature_ranges(raw, view)[0]
    assert raw.external_hits
    assert external_feature_ranges(observation(1.0), view) == (1.0,)


def test_prior_diagnostic_never_claims_verified_translation(tmp_path):
    owner = startup_for(tmp_path / "claim")
    assert "independent_motion_verified" not in owner.summary()
    assert owner.summary()["classification"] == "prior"
