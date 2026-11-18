"""Global obstacle error budgets do not expand body-return classification."""

import math
from dataclasses import replace
from types import SimpleNamespace

import pytest
from control_process_test_support import generic_motion_profile
from learning_test_support import TestRun, geometry
from senior_robot.motion_core_output import MotionCoreOutput
from senior_robot.raw_safety_observation import RawSafetyObservation
from sensor_time_test_support import capture_fields, sample_time
from test_startup_learning import observation, record, side_observation
from test_startup_learning_feasibility import BOUNDARY

from senior_control.config import DeviceProfile
from senior_control.motion.learning_record import LearnedCatalog, LearningAnchor
from senior_control.motion.startup_learning import StartupLearning
from senior_control.safety.external_motion_features import (
    external_feature_ranges,
)
from senior_control.safety.learning_view import LearningFilterView
from senior_control.safety.self_filter import filter_self
from senior_control.safety.self_learning import SelfReturnModel


@pytest.fixture
def core(tmp_path):
    run = TestRun(tmp_path / "geometry")
    value = MotionCoreOutput.__new__(MotionCoreOutput)
    value.profile = generic_motion_profile(length=0.6, width=0.4)
    value._learning_client = run.client()
    value.startup = StartupLearning(value._learning_client)
    value.sensors = SimpleNamespace(wheel_linear=lambda _: 0.0, stationary=lambda _: True)
    capture = SimpleNamespace(
        frame_id="laser_link",
        raw_gid="01",
        bridge_gid="02",
        source_epoch="scan",
        bridge_boot_id="boot",
        capture_generation=1,
        sequence=1,
        valid=True,
        reason="",
        **capture_fields(
            time=sample_time(
                stamp_ns=10_000_000_000,
                received_ros_ns=10_000_000_000,
                received_steady_ns=10_000_000_000,
            )
        ),
    )
    message = SimpleNamespace(
        source_epoch="scan",
        sequence=1,
        frame_id="laser_link",
        ranges=[0.15],
        sensor_x=0.0,
        sensor_y=0.0,
        sensor_yaw=0.0,
        angle_min=0.0,
        angle_increment=0.01,
        range_min=0.05,
        range_max=5.0,
        transform_stamp=10.0,
        raw_capture=capture,
    )
    value.node = SimpleNamespace(
        _body="base_link", _scans=[RawSafetyObservation(message, 10.0, 10.0, 10.0, 10.0)]
    )
    value._learning_geometry = value._learning_scan_key = value.self_returns = None
    yield value
    run.close()


@pytest.mark.parametrize("sensor_error,orientation_error", [(0.0, 0.0), (0.05, 0.05), (0.5, 0.7)])
def test_global_errors_preserve_model_learning_and_classification(
    core,
    sensor_error,
    orientation_error,
):
    core._learning_observation(10.0)
    model = core.self_returns
    model.begin_segment("model-test")
    for _i in range(12):
        record(model, side_observation(0.15))
    assert model.validate() == 1
    anchors = model.learned
    feature_bytes = core._learning_client.run.path.read_bytes()
    core.profile = core.profile.model_copy(
        update={"sensor_error": sensor_error, "orientation_error_rad": orientation_error}
    )
    core._learning_observation(10.0)
    assert core.self_returns is model and model.learned == anchors
    assert core._learning_client.run.path.read_bytes() == feature_bytes
    assert model.filter(side_observation(0.15), provisional=False).rays[180].kind == "self"
    assert model.filter(observation(0.301), provisional=False).external_hits


def test_actual_geometry_has_only_user_match_tolerance_and_no_outside_mask():
    model = SelfReturnModel(BOUNDARY)
    model.learned = ((0.15, 0.0),)
    # A small changed measurement is within the explicit user tolerance.
    assert not model.filter(observation(0.151), provisional=False).external_hits
    assert model.learned == ((0.15, 0.0),)
    assert model.filter(observation(0.161), provisional=False).external_hits
    assert model.learned
    manifest = geometry(BOUNDARY)
    assert "error_m" not in manifest.model_dump()
    raw = observation(0.15)
    assert external_feature_ranges(raw) == raw.scan.ranges
    view = LearningFilterView(
        LearnedCatalog(model_id="a" * 32, anchors=(LearningAnchor(id=0, point=(0.4, 0.0)),)),
        manifest,
    )
    assert external_feature_ranges(observation(0.301), view) == (0.301,)
    assert math.isnan(external_feature_ranges(observation(0.405), view)[0])
    assert external_feature_ranges(observation(0.411), view) == (0.411,)
    assert raw.external_hits and raw.rays[0].kind == "hit"  # No raw evidence mutation.


def test_mask_does_not_extend_ray_towards_nearby_body_geometry():
    scan = observation(0.149).scan
    profile = DeviceProfile(
        footprint=BOUNDARY,
        self_mask=[[(0.15, -0.01), (0.16, -0.01), (0.16, 0.01), (0.15, 0.01)]],
    )
    result = filter_self(scan, profile)
    assert result.rays[0].kind == "hit"
    assert result.external_hits and not result.invalid_regions


def noisy_model():
    model = SelfReturnModel(BOUNDARY)
    for i in range(12):
        record(model, side_observation(0.15 + (0.0002 if i % 2 else -0.0002)))
    assert model.validate() == 1
    return model


def test_nonzero_measurement_changes_learn_and_remain_usable_without_drift():
    model = noisy_model()
    anchors = model.learned
    for value in (0.1501, 0.1499, 0.151, 0.148, 0.155, 0.1503):
        assert model.filter(side_observation(value), provisional=False).rays[180].kind == "self"
        assert model.learned == anchors
    # A sequence of accepted matches cannot move the frozen anchor towards an obstacle.
    assert model.filter(side_observation(0.161), provisional=False).rays[180].kind == "hit"
    assert model.learned
    assert model.filter(side_observation(0.1502), provisional=False).rays[180].kind == "self"


@pytest.mark.parametrize(
    "offset,expected", [(-0.01, "self"), (0.01, "self"), (-0.01001, "hit"), (0.01001, "hit")]
)
def test_user_one_centimeter_matching_bound_is_inclusive(offset, expected):
    model = SelfReturnModel(BOUNDARY)
    for _i in range(12):
        record(model, side_observation(0.15))
    assert model.validate() == 1
    assert (
        model.filter(side_observation(0.15 + offset), provisional=False).rays[180].kind == expected
    )


@pytest.mark.parametrize("distance", [0.2, 0.201])
def test_frozen_match_applies_across_body_boundary_within_one_centimeter(distance):
    # Set a frozen interior anchor near the actual side boundary, without
    # inventing a world-motion justification for this isolated filter test.
    model = SelfReturnModel(BOUNDARY)
    model.learned = ((0.0, 0.195),)
    result = model.filter(side_observation(distance), provisional=False)
    assert result.rays[180].kind == "self"
    assert model.learned


def test_new_adjacent_and_moving_nearby_external_hits_remain_obstacles():
    model = noisy_model()
    obs = side_observation(0.1501)
    ranges = list(obs.scan.ranges)
    ranges[179] = 0.10
    ranges[181] = 0.22
    current = filter_self(
        replace(obs.scan, ranges=tuple(ranges)), DeviceProfile(footprint=BOUNDARY)
    )
    result = model.filter(current, provisional=False)
    assert result.rays[180].kind == "self"
    assert result.rays[179].kind == result.rays[181].kind == "hit"
    assert model.filter(side_observation(0.139), provisional=False).rays[180].kind == "hit"


@pytest.mark.parametrize("offset,expected", [(0.01, 1), (0.01001, 0)])
def test_learning_uses_same_one_centimeter_range_as_runtime(offset, expected):
    model = SelfReturnModel(BOUNDARY)
    for i in range(12):
        distance = 0.15 + (offset if i % 2 == 0 else 0.0)
        record(model, side_observation(distance))
    assert model.validate() == expected
