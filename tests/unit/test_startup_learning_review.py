import math
from dataclasses import replace
from types import SimpleNamespace as N

import pytest
from control_process_test_support import read_inputs, read_observation
from learning_test_support import (  # noqa: F401
    begin_action,
    bind_core,
    learning_runs,
    startup_for,
    write_geometry_record,
)
from sensor_time_test_support import capture_fields, sample_time
from test_motion_core_handoff import core  # noqa: F401
from test_startup_learning import observation
from test_startup_learning_feasibility import BOUNDARY

from senior_control.motion.estimator import Origin
from senior_control.safety.self_learning import SelfReturnModel
from senior_control.vision.raw_capture import RawCaptureHeader


def captured(obs, stamp):
    return replace(obs, scan=replace(obs.scan, stamp=stamp, reference_stamp=stamp))


def moving_owner(tmp_path):
    owner = startup_for(tmp_path / "claim")
    owner.state = "moving"
    for i in range(1, 12):
        owner.advance(0.05, now=i * 0.1, stationary=False)
    return owner


@pytest.mark.parametrize("delivery", [[0.3, 0.6, 0.9], [0.9, 0.3, 0.6]])
def test_capture_order_is_independent_of_delivery_order_and_current_tick(tmp_path, delivery):
    owner = moving_owner(tmp_path)
    model = SelfReturnModel(BOUNDARY)
    for stamp in delivery:
        owner.frame(
            model,
            captured(observation(0.15), stamp),
            capture=RawCaptureHeader(
                time=sample_time(
                    stamp_ns=round(((captured(observation(0.15), stamp)).scan.stamp) * 1e9),
                    received_ros_ns=round(((captured(observation(0.15), stamp)).scan.stamp) * 1e9),
                    received_steady_ns=round(
                        ((captured(observation(0.15), stamp)).scan.stamp) * 1e9
                    ),
                    clock_generation=1,
                ),
                frame_id="laser",
                raw_gid="01",
                bridge_gid="02",
                source_epoch="producer",
                bridge_boot_id="boot",
                capture_generation=1,
                sequence=round((captured(observation(0.15), stamp)).scan.stamp * 1000) + 1,
                valid=True,
                reason="",
            ),
        )
    assert len(model.frames) == 3
    assert [m / 1e9 for m in model.frame_measurements] == [0.3, 0.6, 0.9]


def test_old_moving_scans_cannot_fill_post_stop_validation(tmp_path):
    owner = moving_owner(tmp_path)
    owner.state = "braking"
    owner.advance(0.0, now=1.2, stationary=True)
    assert owner.state == "validating"
    model = SelfReturnModel(BOUNDARY)
    for stamp in (0.3, 0.4, 0.5, 0.6, 0.7):
        owner.frame(
            model,
            captured(observation(0.15), stamp),
            capture=RawCaptureHeader(
                time=sample_time(
                    stamp_ns=round(((captured(observation(0.15), stamp)).scan.stamp) * 1e9),
                    received_ros_ns=round(((captured(observation(0.15), stamp)).scan.stamp) * 1e9),
                    received_steady_ns=round(
                        ((captured(observation(0.15), stamp)).scan.stamp) * 1e9
                    ),
                    clock_generation=1,
                ),
                frame_id="laser",
                raw_gid="01",
                bridge_gid="02",
                source_epoch="producer",
                bridge_boot_id="boot",
                capture_generation=1,
                sequence=round((captured(observation(0.15), stamp)).scan.stamp * 1000) + 1,
                valid=True,
                reason="",
            ),
        )
    assert owner.validation_samples == 0
    assert owner.state == "validating"


def test_initialization_wait_does_not_consume_learning_budget(tmp_path):
    owner = startup_for(tmp_path / "claim")
    owner.advance(None, now=0, stationary=False)
    owner.advance(None, now=100, stationary=False)
    assert owner.state == "initializing" and owner.started is None
    for i in range(5):
        owner.advance(0.0, now=100 + i * 0.1, stationary=True)
    assert owner.state == "sampling"
    assert owner.started is None
    begin_action(owner, 100.5)
    assert owner.started == pytest.approx(100.5)


def test_body_change_rebuilds_model_but_global_errors_do_not(core, tmp_path):  # noqa: F811
    from senior_control.config import DeviceProfile
    from senior_control.safety.external_motion_features import external_feature_ranges

    bind_core(core, tmp_path / "claim")

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
        frame_id="laser_link",
        range_min=0.05,
        range_max=5.0,
        raw_capture=N(
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
                    stamp_ns=round((10.0) * 1e9),
                    received_ros_ns=round((10.0) * 1e9),
                    received_steady_ns=round((10.0) * 1e9),
                )
            ),
        ),
    )
    core._sync_learning_model(msg)
    original = core.self_returns
    original.learned = ((0.15, 0.0),)
    boundary[0] = [(-0.1, -0.1), (0.1, -0.1), (0.1, 0.1), (-0.1, 0.1)]
    core._sync_learning_model(msg)
    assert core.self_returns is not original and not core.self_returns.learned
    assert core.startup.state != "interrupted"
    assert core.self_returns.filter(observation(0.15), provisional=False).external_hits
    assert external_feature_ranges(observation(0.25), core._learning_client.run.path) == (0.25,)
    previous = core.self_returns
    core.profile = core.profile.model_copy(
        update={
            "sensor_error": core.profile.sensor_error + 0.1,
            "orientation_error_rad": core.profile.orientation_error_rad + 0.2,
        }
    )
    core._sync_learning_model(msg)
    assert core.self_returns is previous


def test_planned_stopping_keeps_validation_without_new_forward(core, tmp_path):  # noqa: F811
    bind_core(core, tmp_path / "claim")
    begin_action(core.startup, 10.0)
    core.startup.advance(0.0, now=13.0, stationary=True)
    assert core.startup.state == "validating"
    assert core.startup.velocity == 0


def test_initial_scan_alignment_wait_does_not_retire_attempt(core, tmp_path):  # noqa: F811
    from senior_control.motion.contracts import MotionEventKind

    bind_core(core, tmp_path / "claim")
    core._event(MotionEventKind.FEEDBACK_LOST, 10.0)
    assert core.startup.state == "initializing"


def test_feature_boundary_replacement_is_not_hidden_by_same_mtime(tmp_path):
    import os

    from senior_control.safety.external_motion_features import (
        external_feature_ranges,
    )

    claim = tmp_path / "run"
    write_geometry_record(claim, BOUNDARY)
    path = claim
    old = path.stat()
    assert math.isnan(external_feature_ranges(observation(0.25), claim)[0])
    write_geometry_record(claim, [(-0.1, -0.1), (0.1, -0.1), (0.1, 0.1), (-0.1, 0.1)])
    os.utime(path, ns=(old.st_atime_ns, old.st_mtime_ns))
    assert external_feature_ranges(observation(0.25), claim) == (0.25,)


def test_unstarted_action_cannot_use_post_frames_as_moving_evidence(tmp_path):
    owner = moving_owner(tmp_path)
    owner.state = "braking"
    owner.advance(0.0, now=1.2, stationary=True)
    model = SelfReturnModel(BOUNDARY)
    model.begin_segment(owner.segment_id)
    owner.resample_required = False  # Fill THIS established segment, not retired data.
    for i in range(64):
        model.observe(
            captured(observation(0.15), i / 100),
            capture=RawCaptureHeader(
                time=sample_time(
                    stamp_ns=round(((captured(observation(0.15), i / 100)).scan.stamp) * 1e9),
                    received_ros_ns=round(
                        ((captured(observation(0.15), i / 100)).scan.stamp) * 1e9
                    ),
                    received_steady_ns=round(
                        ((captured(observation(0.15), i / 100)).scan.stamp) * 1e9
                    ),
                    clock_generation=1,
                ),
                frame_id="laser",
                raw_gid="01",
                bridge_gid="02",
                source_epoch="test-producer",
                bridge_boot_id="boot",
                capture_generation=1,
                sequence=round((captured(observation(0.15), i / 100)).scan.stamp * 1000) + 1,
                valid=True,
                reason="",
            ),
            segment_id=owner.segment_id,
        )
    owner.advance(0.0, now=1.3, stationary=True)
    owner.frame(
        model,
        captured(observation(0.15), 1.3),
        capture=RawCaptureHeader(
            time=sample_time(
                stamp_ns=round(((captured(observation(0.15), 1.3)).scan.stamp) * 1e9),
                received_ros_ns=round(((captured(observation(0.15), 1.3)).scan.stamp) * 1e9),
                received_steady_ns=round(((captured(observation(0.15), 1.3)).scan.stamp) * 1e9),
                clock_generation=1,
            ),
            frame_id="laser",
            raw_gid="01",
            bridge_gid="02",
            source_epoch="producer",
            bridge_boot_id="boot",
            capture_generation=1,
            sequence=round((captured(observation(0.15), 1.3)).scan.stamp * 1000) + 1,
            valid=True,
            reason="",
        ),
    )
    assert owner.validation_samples == 0
    assert not owner._motion_frames


def test_body_update_preserves_frozen_filter_and_invalidates_geometry_cache(core, tmp_path):  # noqa: F811
    from learning_test_support import learning_point
    from shapely.geometry import Point, Polygon

    from senior_control.config import DeviceProfile
    from senior_control.motion.estimator import MotionObservation
    from senior_control.motion.observations import Source

    boundary = [BOUNDARY]
    core.node._profile = lambda: DeviceProfile(footprint=boundary[0])
    core.node._depth_policy = "laser_only"
    core.node._scan_topology_ready = lambda: True
    bind_core(core, tmp_path / "claim")
    core.startup.state = "completed"
    for source, values in ((Source.WHEEL, (0.0, 0.0)), (Source.IMU, (0.0,))):
        core.sensors.ingest(
            learning_point(source, 10_000_000_000, 10_000_000_000, 10_000_000_000, values),
            10.0,
            10.0,
            10.0,
        )
    core.node._reception_test.sensors.ingest_observation(
        MotionObservation(
            "laser",
            (0.0, 0.0, 0.0),
            10.0,
            (
                Origin(
                    "laser",
                    "a",
                    1,
                    sample_time(
                        stamp_ns=round((10.0) * 1e9),
                        received_ros_ns=round((10.0) * 1e9),
                        received_steady_ns=round((10.0) * 1e9),
                    ),
                ),
            ),
            independent=(True, True, True),
            measured_steady=10.0,
            stamp_ns=RawCaptureHeader(
                time=sample_time(
                    stamp_ns=round((10.0) * 1e9),
                    received_ros_ns=round((10.0) * 1e9),
                    received_steady_ns=round((10.0) * 1e9),
                    clock_generation=1,
                ),
                frame_id="laser",
                raw_gid="01",
                bridge_gid="02",
                source_epoch="producer",
                bridge_boot_id="boot",
                capture_generation=1,
                sequence=1,
                valid=True,
                reason="",
            ).stamp_ns,
        ),
        10.0,
        10.0,
    )
    read_inputs(core, 10.0)
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
                raw_capture=N(
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
                            stamp_ns=round((10.0) * 1e9),
                            received_ros_ns=round((10.0) * 1e9),
                            received_steady_ns=round((10.0) * 1e9),
                        )
                    ),
                ),
            ),
            measurement_steady=10.0,
        )
    ]
    read_observation(core, 10.0)
    core._learning_client.complete((endpoint,))
    read_observation(core, 10.0)
    assert core._perception.occupied.distance(Point(endpoint)) > 0
    boundary[0] = [(-0.1, -0.1), (0.1, -0.1), (0.1, 0.1), (-0.1, 0.1)]
    read_observation(core, 10.0)  # identical raw capture/key; only the body changed
    assert core.self_returns.body.equals(Polygon(boundary[0]))
    assert core._perception.occupied.distance(Point(endpoint)) > 0
    assert core._learning_client.view.catalog.points == (endpoint,)
    assert core.startup.state == "completed"
