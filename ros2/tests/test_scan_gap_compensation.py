"""Direct real-surface compensation and its existing control/learning consumers."""

import math
from dataclasses import asdict, replace
from types import SimpleNamespace

import pytest
from control_process_test_support import generic_motion_profile
from senior_perception.learning_record import LearnedCatalog, LearningAnchor
from senior_perception.learning_view import LearningFilterView
from senior_perception.raw_safety_observation import RawSafetyObservation
from senior_perception.scan_gap_compensation import ScanGapCompensator
from senior_robot.motion_core_perception import scan_observation
from sensor_time_test_support import capture_fields, sample_time
from test_motion_core_handoff import barrier
from test_startup_learning_chain import LearningRig

from vehicle_core.values import Velocity


@pytest.fixture
def profile():
    return generic_motion_profile(length=0.55, width=0.55, sensor_error=0.05, decel=0.5, lag_ms=500)


def wall(*, x=1.0, pose=(0.0, 0.0, 0.0), angle_min=-0.1):
    # Analytic intersection with the world plane x=constant, not a fitted algorithm.
    return [(x - pose[0]) / math.cos(pose[2] + angle_min + i * 0.01) for i in range(21)]


def scan(sequence, ranges, *, angle_min=-0.1):
    ns = 1_700_000_000_000_000_000 + sequence * 100_000_000
    steady_ns = 10_000_000_000 + sequence * 100_000_000
    timing = sample_time(stamp_ns=ns, received_ros_ns=ns, received_steady_ns=steady_ns)
    capture = SimpleNamespace(
        frame_id="laser",
        raw_gid="01",
        bridge_gid="02",
        source_epoch="scan",
        bridge_boot_id="boot",
        capture_generation=1,
        sequence=sequence,
        valid=True,
        reason="",
        **capture_fields(time=timing),
    )
    message = SimpleNamespace(
        raw_capture=capture,
        frame_id="laser",
        ranges=ranges,
        angle_min=angle_min,
        angle_increment=0.01,
        range_min=0.05,
        range_max=20.0,
        sensor_x=0.0,
        sensor_y=0.0,
        sensor_yaw=0.0,
        transform_stamp=ns / 1e9,
    )
    return RawSafetyObservation(message, steady_ns / 1e9, ns / 1e9, ns / 1e9, steady_ns / 1e9)


def poses(raw, pose=(0.0, 0.0, 0.0), *, bracket=False):
    capture = raw.message.raw_capture
    ns = capture.stamp.sec * 1_000_000_000 + capture.stamp.nanosec
    result = []
    for offset in (-10_000_000, 10_000_000) if bracket else (0,):
        result.append(
            dict(
                time=asdict(
                    sample_time(
                        stamp_ns=ns + offset,
                        received_ros_ns=ns + offset,
                        received_steady_ns=capture.received_steady_ns + offset,
                    )
                ),
                x=pose[0],
                y=pose[1],
                yaw=pose[2],
                epoch="odom",
            )
        )
    return result


def test_internal_gap_follows_current_surface_and_real_return_replaces_it(profile):
    engine = ScanGapCompensator(profile)
    first = scan(1, wall())
    assert not engine.update(first, poses(first), now=first.received).predictions
    ranges = wall(x=0.98)
    ranges[10] = math.inf
    raw = scan(2, ranges)
    result = engine.update(raw, poses(raw), now=raw.received)
    (prediction,) = result.predictions
    assert (prediction.index, prediction.missing_frames) == (10, 1)
    assert prediction.distance == pytest.approx(0.98)
    assert prediction.seed_capture.sequence == 1
    assert result.message is raw.message and raw.message.ranges[10] == math.inf
    assert (result.stamp, result.received, result.measurement_steady) == (
        raw.stamp,
        raw.received,
        raw.measurement_steady,
    )
    ranges = wall(x=0.98)
    ranges[10] = 0.9
    real = scan(3, ranges)
    assert not engine.update(real, poses(real), now=real.received).predictions
    assert real.message.ranges[10] == 0.9


def test_edge_includes_original_endpoint_without_crossing_a_break_or_extending_coverage(profile):
    engine = ScanGapCompensator(profile)
    ranges = wall()
    ranges[11:] = [math.nan] * 10
    first = scan(1, ranges)
    engine.update(first, poses(first), now=first.received)
    current = list(ranges)
    current[10] = math.inf
    raw = scan(2, current)
    (prediction,) = engine.update(raw, poses(raw), now=raw.received).predictions
    assert prediction.index == 10 and prediction.distance == pytest.approx(1.0)
    current[11] = math.inf  # Beyond the historical real surface: no new seed exists.
    raw = scan(3, current)
    assert [p.index for p in engine.update(raw, poses(raw), now=raw.received).predictions] == [10]
    current = list(ranges)
    current[10] = math.inf
    current[7:10] = wall(x=2.0)[7:10]  # Real support belongs to another surface.
    raw = scan(4, current)
    assert not engine.update(raw, poses(raw), now=raw.received).predictions


def test_three_new_acquisitions_expire_without_snapshot_or_prediction_renewal(profile):
    ranges = wall()
    ranges[10] = math.inf
    unseeded = ScanGapCompensator(profile)
    for sequence in (1, 2):
        raw = scan(sequence, ranges)
        assert not unseeded.update(raw, poses(raw), now=raw.received).predictions
    engine = ScanGapCompensator(profile)
    first = scan(1, wall())
    engine.update(first, poses(first), now=first.received)
    for sequence in range(2, 6):
        raw = scan(sequence, ranges)
        result = engine.update(raw, poses(raw), now=raw.received)
        assert engine.update(raw, [], now=raw.received + 0.001) is result
        if sequence < 5:
            (prediction,) = result.predictions
            assert prediction.missing_frames == sequence - 1
            assert prediction.seed_capture.sequence == 1
            assert prediction.seed_capture.stamp_ns == 1_700_000_000_100_000_000
        else:
            assert not result.predictions


def test_motion_and_new_grid_use_measurement_pose_with_nonblocking_same_frame_retry(profile):
    engine = ScanGapCompensator(profile)
    first = scan(1, wall())
    engine.update(first, poses(first), now=first.received)
    pose, angle_min = (0.1, 0.0, 0.03), -0.095
    ranges = wall(pose=pose, angle_min=angle_min)
    ranges[10] = math.inf
    raw = scan(2, ranges, angle_min=angle_min)
    assert engine.update(raw, [], now=raw.received) is raw
    result = engine.update(raw, poses(raw, pose, bracket=True), now=raw.received + 0.02)
    (prediction,) = result.predictions
    assert prediction.index == 10 and prediction.missing_frames == 1
    assert prediction.distance == pytest.approx(0.9 / math.cos(0.035))
    assert scan_observation(result, "base_link").scan.ranges[10] == math.inf


def test_abnormal_ranges_are_never_gap_predictions(profile):
    engine = ScanGapCompensator(profile)
    first = scan(1, wall())
    engine.update(first, poses(first), now=first.received)
    for sequence, invalid in enumerate((math.nan, -math.inf, 0.01, 21.0), start=2):
        ranges = wall()
        ranges[10] = invalid
        raw = scan(sequence, ranges)
        assert not engine.update(raw, poses(raw), now=raw.received).predictions


def test_real_core_consumes_prediction_through_same_frozen_filter_but_learning_stays_raw(
    profile, monkeypatch, tmp_path
):
    engine = ScanGapCompensator(profile)
    first = scan(1, wall())
    engine.update(first, poses(first), now=first.received)
    ranges = wall()
    ranges[10] = math.inf
    raw = scan(2, ranges)
    result = engine.update(raw, poses(raw), now=raw.received)
    rig = LearningRig(monkeypatch, tmp_path)
    try:
        core = rig.core
        core.profile = profile

        def emit(velocity, reason, deadline=0.0, *, computed_steady):
            # This consumer exercises manual output, without the rig's learning-only ACK.
            core.node.emitted.append((rig.now, velocity, reason, deadline, core.startup.started))
            return rig.now

        core.node._emit_motion_core = emit
        rig.now = raw.received
        core.node._scans.append(result)
        learned_input = core._learning_observation(rig.now)
        assert learned_input.scan.ranges[10] == math.inf
        assert learned_input.rays[10].kind == "invalid"
        points = [(1.0, math.tan(-0.1 + i * 0.01)) for i in range(21) if i != 10]
        catalog = LearnedCatalog(
            model_id="a" * 32,
            anchors=tuple(LearningAnchor(id=i, point=p) for i, p in enumerate(points)),
        )
        core._learning_client.view = LearningFilterView(catalog, core._learning_geometry)
        assert core.command(barrier("prepared"))
        intent = barrier("intent")
        intent.sequence, intent.vx, intent.manual_linear = 1, 0.8, 1.0
        intent.manual_linear_limit, intent.degraded_linear_limit = 0.8, 0.3
        intent.steady_deadline = rig.now + 0.5
        assert core.command(intent)
        core._output, core._last_tick = Velocity(0.8, 0), rig.now
        assert 0 < rig.step(rig.now + 0.05, new_scan=False).vx < 0.8
        assert core._learning_client.view.catalog == catalog
        assert core.node._scans[-1].message.ranges[10] == math.inf
        core.node._scans[-1] = replace(result, predictions=())
        core._output = Velocity(0.8, 0)
        assert rig.step(rig.now + 0.05, new_scan=False).vx == pytest.approx(0.8)
        # A learned return at the predicted position uses that same frozen filter too.
        complete_catalog = LearnedCatalog(
            model_id=catalog.model_id,
            anchors=catalog.anchors + (LearningAnchor(id=len(points), point=(1.0, 0.0)),),
        )
        core._learning_client.view = LearningFilterView(complete_catalog, core._learning_geometry)
        core.node._scans[-1] = result
        core._output = Velocity(0.8, 0)
        assert rig.step(rig.now + 0.05, new_scan=False).vx == pytest.approx(0.8)
    finally:
        rig.close()
