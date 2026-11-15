import math
from dataclasses import replace
from types import SimpleNamespace as N

import pytest
from test_motion_core_handoff import core  # noqa: F401
from test_startup_learning import observation, side_observation
from test_startup_learning_feasibility import BOUNDARY

from senior_control.models import Pose2D
from senior_control.motion.estimator import Origin
from senior_control.motion.startup_learning import StartupLearning
from senior_control.safety.self_learning import SelfReturnModel


def estimate(t, vx=0.05):
    root = Origin("laser", "a", round(t * 1000), t, t, t)

    def axis(value):
        return N(
            value=value,
            error=0.001,
            observed=True,
            independent=True,
            roots=(root,),
            deadline_steady=t + 1,
        )

    x, y, w = axis(vx), axis(0), axis(0)
    return N(computed_ros=t, healthy=True, generation=1, vx=x, vy=y, wz=w, independent_axes=(x, w))


def captured(obs, stamp):
    return replace(obs, scan=replace(obs.scan, stamp=stamp, reference_stamp=stamp))


def moving_owner(tmp_path):
    owner = StartupLearning(tmp_path / "claim")
    owner.state = "moving"
    for i in range(1, 12):
        owner.advance(estimate(i * 0.1), 0.05, now=i * 0.1, stationary=False)
    return owner


@pytest.mark.parametrize("delivery", [[0.3, 0.6, 0.9], [0.9, 0.3, 0.6]])
def test_capture_pose_is_independent_of_delivery_order_and_current_tick(tmp_path, delivery):
    owner = moving_owner(tmp_path)
    model = SelfReturnModel(BOUNDARY, error_m=0.005)
    for stamp in delivery:
        owner.frame(model, captured(observation(0.15), stamp))
    assert [p.x for p, _ in model.frames] == pytest.approx([0.01, 0.025, 0.04])
    assert model.frame_stamps == [0.3, 0.6, 0.9]
    assert all(error > 0 for error in model.frame_errors)


def test_old_moving_scans_cannot_fill_post_stop_validation(tmp_path):
    owner = moving_owner(tmp_path)
    owner.state = "braking"
    owner.advance(estimate(1.2, 0), 0.0, now=1.2, stationary=True)
    assert owner.state == "validating"
    model = SelfReturnModel(BOUNDARY, error_m=0.005)
    for stamp in (0.3, 0.4, 0.5, 0.6, 0.7):
        owner.frame(model, captured(observation(0.15), stamp))
    assert owner.validation_samples == 0
    assert owner.state == "validating"


def test_initialization_wait_does_not_consume_learning_budget(tmp_path):
    owner = StartupLearning(tmp_path / "claim")
    owner.advance(None, None, now=0, stationary=False)
    owner.advance(None, None, now=100, stationary=False)
    assert owner.state == "initializing" and owner.started is None
    for i in range(5):
        owner.advance(estimate(100 + i * 0.1, 0), 0.0, now=100 + i * 0.1, stationary=True)
    assert owner.state == "sampling"
    assert owner.started == pytest.approx(100.4)


def test_world_support_crossing_unknown_cannot_be_classified_self():
    model = SelfReturnModel(BOUNDARY, error_m=0.005)
    for i in range(12):
        model.observe(captured(side_observation(0.15), 10 + i * 0.1), Pose2D(i * 0.015, 0, 0))
    # Small endpoint support is observed; full pose error support crosses the
    # equipment ray/unknown area. The uncertainty is intentionally nonzero.
    assert model.validate(independent_motion=True, motion_error_m=0.1) == 0


def test_body_and_error_change_rebuild_model_and_feature_boundary(core, tmp_path):  # noqa: F811
    from senior_control.config import DeviceProfile
    from senior_control.safety.external_motion_features import external_feature_ranges

    core.startup = StartupLearning(tmp_path / "claim")
    core._startup_claim_path = tmp_path / "claim"
    boundary = [BOUNDARY]
    core.node._profile = lambda: DeviceProfile(footprint=boundary[0])
    msg = N(
        source_epoch="scan",
        sensor_x=0.0,
        sensor_y=0.0,
        sensor_yaw=0.0,
        angle_min=0.0,
        angle_increment=0.01,
        ranges=[0.15],
    )
    core._sync_learning_model(msg)
    original = core.self_returns
    original.learned = ((0.15, 0.0),)
    boundary[0] = [(-0.1, -0.1), (0.1, -0.1), (0.1, 0.1), (-0.1, 0.1)]
    core._sync_learning_model(msg)
    assert core.self_returns is not original and not core.self_returns.learned
    assert core.startup.state == "interrupted"
    assert core.self_returns.filter(observation(0.15), provisional=False).external_hits
    assert external_feature_ranges(observation(0.25), core._startup_claim_path) == (0.25,)
    previous = core.self_returns
    core.profile = replace(core.profile, sensor_error=core.profile.sensor_error + 0.01)
    core._sync_learning_model(msg)
    assert core.self_returns is not previous


def test_planned_verified_braking_is_not_permission_loss(core, tmp_path):  # noqa: F811
    from senior_control.motion.contracts import BrakeKind, BrakeRequest

    core.startup = StartupLearning(tmp_path / "claim")
    core.startup.state = "braking"
    core._learning_motion_emitted = True
    permission = N(permit=None, brake=BrakeRequest(BrakeKind.CONTROLLED, "controlled_stop", True))
    assert not core._startup_permission_lost(permission, 10.0)
    assert core.startup.state == "braking"


def test_initial_scan_alignment_wait_does_not_retire_attempt(core, tmp_path):  # noqa: F811
    from senior_control.motion.contracts import MotionEventKind

    core.startup = StartupLearning(tmp_path / "claim")
    core._event(MotionEventKind.FEEDBACK_LOST, 10.0)
    assert core.startup.state == "initializing"


def test_feature_boundary_replacement_is_not_hidden_by_same_mtime(tmp_path):
    import os

    from senior_control.safety.external_motion_features import (
        boundary_path,
        external_feature_ranges,
        write_feature_boundary,
    )

    claim = tmp_path / "run"
    write_feature_boundary(claim, BOUNDARY, 0.01)
    path = boundary_path(claim)
    old = path.stat()
    assert math.isnan(external_feature_ranges(observation(0.25), claim)[0])
    write_feature_boundary(claim, [(-0.1, -0.1), (0.1, -0.1), (0.1, 0.1), (-0.1, 0.1)], 0.01)
    os.utime(path, ns=(old.st_atime_ns, old.st_mtime_ns))
    assert external_feature_ranges(observation(0.25), claim) == (0.25,)


def test_post_count_requires_frame_retained_by_bounded_model(tmp_path):
    owner = moving_owner(tmp_path)
    owner.state = "braking"
    owner.advance(estimate(1.2, 0), 0.0, now=1.2, stationary=True)
    model = SelfReturnModel(BOUNDARY, error_m=0.005)
    for i in range(64):
        model.observe(captured(observation(0.15), i / 100), Pose2D(0, 0, 0), force=True)
    owner.advance(estimate(1.3, 0), 0.0, now=1.3, stationary=True)
    owner.frame(model, captured(observation(0.15), 1.3))
    assert owner.validation_samples == 0


def test_body_update_invalidates_actual_observation_cache(core, tmp_path):  # noqa: F811
    from shapely.geometry import Point, Polygon
    from test_unified_motion_runtime import point

    from senior_control.config import DeviceProfile
    from senior_control.motion.estimator import MotionObservation
    from senior_control.motion.observations import Source

    boundary = [BOUNDARY]
    core.node._profile = lambda: DeviceProfile(footprint=boundary[0])
    core.node._depth_policy = "laser_only"
    core.node._scan_topology_ready = lambda: True
    core.startup = StartupLearning(tmp_path / "claim")
    core.startup.state = "completed"
    for source, values in ((Source.WHEEL, (0.0, 0.0)), (Source.IMU, (0.0,))):
        core.sensors.ingest(point(source, 10.0, 10.0, values), 10.0, 10.0, 10.0)
    core.sensors.ingest_observation(
        MotionObservation(
            "laser",
            (0.0, 0.0, 0.0),
            10.0,
            (Origin("laser", "a", 1, 10.0, 10.0, 10.0),),
            independent=(True, True, True),
        ),
        10.0,
        10.0,
    )
    core._inputs(10.0)
    ranges = [10.0] * 720
    ranges[359] = 0.15
    angle = -math.pi + 359 * 2 * math.pi / 719
    endpoint = (0.15 * math.cos(angle), 0.15 * math.sin(angle))
    core.node._scans = [
        N(
            stamp=10.0,
            received=10.0,
            received_ros=10.0,
            message=N(
                source_epoch="scan",
                sequence=1,
                ranges=ranges,
                frame_id="laser_link",
                angle_min=-math.pi,
                angle_increment=2 * math.pi / 719,
                range_min=0.08,
                range_max=20.0,
                sensor_x=0.0,
                sensor_y=0.0,
                sensor_yaw=0.0,
                transform_stamp=10.0,
            ),
        )
    ]
    core._observation(10.0)
    core.self_returns.learned = (endpoint,)
    core.self_returns.revision += 1
    core._observation(10.0)
    assert core._perception.occupied.distance(Point(endpoint)) > 0
    boundary[0] = [(-0.1, -0.1), (0.1, -0.1), (0.1, 0.1), (-0.1, 0.1)]
    core._observation(10.0)  # identical raw capture/key; only the body changed
    assert core.self_returns.body.equals(Polygon(boundary[0]))
    assert core._perception.occupied.distance(Point(endpoint)) == 0
    assert not core.self_returns.learned and core.startup.state == "interrupted"
