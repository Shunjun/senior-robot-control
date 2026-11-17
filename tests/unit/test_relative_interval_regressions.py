"""Recent relative uncertainty must not depend on the lifetime of a shared path."""

from dataclasses import replace

import pytest
from control_process_test_support import read_inputs, read_observation
from sensor_time_test_support import capture_fields, sample_time

from senior_control.motion.estimator import MotionEstimation, MotionObservation, Origin
from senior_control.motion.pose_estimator import PoseEstimation


def sample(t, sequence, *, source="laser", yaw=0.0, missing_linear=False, lateral=None):
    return MotionObservation(
        source,
        (None if missing_linear else 0.2, lateral, yaw),
        t,
        (
            Origin(
                source,
                "epoch",
                sequence,
                sample_time(
                    stamp_ns=round((t) * 1e9),
                    received_ros_ns=round((t) * 1e9),
                    received_steady_ns=round((t + 100) * 1e9),
                ),
            ),
        ),
        measured_steady=t + 100,
        stamp_ns=round((t) * 1e9),
    )


def run(duration, *, yaw=0.0, switch=False, lateral=None):
    e = MotionEstimation()
    p = PoseEstimation(linear_error_mps=0.02, angular_error_rps=0.02, lateral_drift_mps=0.03)
    old = current = None
    for i in range(round(duration / 0.05) + 1):
        t = 1 + i * 0.05
        if switch and i == 100:
            e.invalidate("laser")
        source = "wheel" if switch and i >= 100 else "laser"
        e.ingest(sample(t, i + 1, source=source, yaw=yaw, lateral=lateral))
        m = e.estimate(t, t + 100)
        p.advance(m)
        old, current = (
            current,
            p.at(
                t,
                now_ros=t,
                now_steady=t + 100,
                motion=m,
                measurement_steady=(t) + 100.0,
                stamp_ns=round((t) * 1e9),
            ),
        )
    return p, e, old, current


@pytest.mark.parametrize("duration", [10.05, 60.05])
@pytest.mark.parametrize("yaw", [0.0, 0.4])
def test_recent_interval_cancels_shared_history_after_ten_and_sixty_seconds(duration, yaw):
    short, _, a, b = run(0.15, yaw=yaw)
    expected = short.relative_error(a, b)
    p, _, old, current = run(duration, yaw=yaw)
    result = p.relative_error(old, current)
    assert current.prediction_error.software_position_m > b.prediction_error.software_position_m
    assert result.software_position_m == pytest.approx(expected.software_position_m, abs=1e-10)
    assert result.software_yaw_rad == pytest.approx(expected.software_yaw_rad, abs=1e-10)
    assert result.software_position_m < 0.01 and result.software_yaw_rad < 0.01


@pytest.mark.parametrize("duration", [5.0, 5.05, 60.05])
def test_recent_interval_across_motion_source_switch_remains_local(duration):
    p, e, old, current = run(duration, yaw=0.4, switch=True)
    result = p.relative_error(old, current)
    assert e.estimate(current.stamp_ros, current.stamp_ros + 100).vx.sources == ("wheel",)
    assert result.software_position_m < 0.01 and result.software_yaw_rad < 0.01


@pytest.mark.parametrize("missing_axis", ["vx", "wz"])
def test_missing_axis_interval_charges_uncertainty_then_fresh_interval_recovers(missing_axis):
    p, e, old, current = run(10.05)
    t = current.stamp_ros
    e.invalidate("laser")
    for i in range(1, 26):
        stamp = t + i * 0.05
        missing = (
            sample(stamp, i, source="imu", missing_linear=True)
            if missing_axis == "vx"
            else replace(sample(stamp, i, source="gps"), values=(0.2, None, None))
        )
        e.ingest(missing)
        m = e.estimate(stamp, stamp + 100)
        p.advance(m)
        old, current = (
            current,
            p.at(
                stamp,
                now_ros=stamp,
                now_steady=stamp + 100,
                motion=m,
                measurement_steady=(stamp) + 100.0,
                stamp_ns=round((stamp) * 1e9),
            ),
        )
    assert m.healthy
    missing = p.relative_error(old, current)
    if missing_axis == "vx":
        assert m.vx.value is None and m.wz.observed
        assert missing.position_m is None
        assert missing.software_position_m >= p.unknown_speed * 0.05 - 1e-10
    else:
        assert m.wz.value is None and m.vx.observed
        assert missing.yaw_rad is None
        assert missing.software_yaw_rad >= p.unknown_yaw * 0.05 - 1e-10
    for i in range(26, 30):
        stamp = t + i * 0.05
        e.ingest(sample(stamp, i + 1000, source="laser"))
        m = e.estimate(stamp, stamp + 100)
        p.advance(m)
        old, current = (
            current,
            p.at(
                stamp,
                now_ros=stamp,
                now_steady=stamp + 100,
                motion=m,
                measurement_steady=(stamp) + 100.0,
                stamp_ns=round((stamp) * 1e9),
            ),
        )
    fresh = p.relative_error(old, current)
    assert fresh.software_position_m < 0.01
    assert current.prediction_error.software_position_m > fresh.software_position_m


def test_relative_midsegment_uses_the_same_integrated_path():
    p, e, old, current = run(10.05, yaw=0.4)
    stamp = current.stamp_ros - 0.025
    m = e.estimate(current.stamp_ros, current.stamp_ros + 100)
    mid = p.at(
        stamp,
        now_ros=current.stamp_ros,
        now_steady=current.stamp_ros + 100,
        motion=m,
        measurement_steady=(stamp) + 100.0,
        stamp_ns=round((stamp) * 1e9),
    )
    r = p.relative_error(mid, current)
    assert r.software_position_m < 0.005 and r.software_yaw_rad < 0.005


def test_distinct_anchors_do_not_claim_shared_history_cancellation():
    p, _, old, current = run(0.15)
    unrelated = replace(
        current,
        anchor_roots=(
            Origin(
                "gps",
                "different",
                1,
                sample_time(
                    stamp_ns=round((1) * 1e9),
                    received_ros_ns=round((1) * 1e9),
                    received_steady_ns=round((101) * 1e9),
                ),
            ),
        ),
        error_m=None,
        error_yaw_rad=None,
    )
    result = p.relative_error(old, unrelated)
    assert result.position_m is None and result.yaw_rad is None
    assert result.software_position_m >= p.position_error


def test_actual_scan_alignment_has_same_recent_budget_after_ten_and_sixty_seconds(monkeypatch):
    import math
    from types import SimpleNamespace as N

    from senior_robot.motion_sensor_runtime import MotionSensorRuntime
    from test_motion_core_handoff import core as core_fixture

    from senior_control.config import DeviceProfile

    fixture = core_fixture.__wrapped__(monkeypatch)
    c = next(fixture)
    clock = [10.0]
    c.node.get_clock = lambda: N(now=lambda: N(nanoseconds=round(clock[0] * 1e9)))
    c.node._depth_policy = "laser_only"
    c.node._scan_topology_ready = lambda: True
    c.node._profile = lambda: DeviceProfile(self_mask_calibrated=True)
    try:
        results = []
        for duration in (10.0, 60.0):
            c.node._reception_test.sensors = MotionSensorRuntime(c.profile, c.diagnostics)
            for i in range(round(duration / 0.05) + 1):
                t = round((10.0 + i * 0.05) * 1e9) / 1e9
                clock[0] = t
                c.node._reception_test.sensors.ingest_observation(
                    MotionObservation(
                        "laser",
                        (0.2, None, 0.4),
                        t,
                        (
                            Origin(
                                "laser",
                                "fresh",
                                i + 1,
                                sample_time(
                                    stamp_ns=round((t) * 1e9),
                                    received_ros_ns=round((t) * 1e9),
                                    received_steady_ns=round((t) * 1e9),
                                ),
                            ),
                        ),
                        measured_steady=t,
                        stamp_ns=sample_time(
                            stamp_ns=round((t) * 1e9),
                            received_ros_ns=round((t) * 1e9),
                            received_steady_ns=round((t) * 1e9),
                            clock_generation=1,
                        ).stamp_ns,
                    ),
                    t,
                    t,
                )
                read_inputs(c, t)
            stamp = round((t - 0.05) * 1e9) / 1e9
            c.node._scans = [
                N(
                    stamp=stamp,
                    received=stamp,
                    measurement_steady=stamp,
                    received_ros=stamp,
                    message=N(
                        raw_capture=N(
                            frame_id="laser_link",
                            raw_gid="01",
                            bridge_gid="02",
                            source_epoch="source",
                            bridge_boot_id="boot",
                            capture_generation=1,
                            sequence=1,
                            valid=True,
                            reason="",
                            **capture_fields(
                                time=sample_time(
                                    stamp_ns=round((stamp) * 1e9),
                                    received_ros_ns=round((stamp) * 1e9),
                                    received_steady_ns=round((stamp) * 1e9),
                                )
                            ),
                        ),
                        source_epoch="scan",
                        sequence=round(stamp * 100),
                        frame_id="laser_link",
                        ranges=[10.0] * 720,
                        angle_min=-math.pi,
                        angle_increment=2 * math.pi / 719,
                        range_min=0.08,
                        range_max=20.0,
                        sensor_x=0.0,
                        sensor_y=0.0,
                        sensor_yaw=0.0,
                        transform_stamp=stamp,
                    ),
                )
            ]
            perception, _ = read_observation(c, t)
            results.append(
                (c._space_estimate.software_alignment_error_m, perception.free_space.area)
            )
        assert results[1] == pytest.approx(results[0], abs=1e-8)
        assert results[1][0] < 0.03 and results[1][1] > 100
    finally:
        try:
            next(fixture)
        except StopIteration:
            pass


def test_midsegment_pose_uses_recorded_motion_when_endpoint_velocity_changes():
    from senior_control.motion.kinematics import compose, inverse

    p, e, _, old = run(10.05, yaw=0.4)
    t = old.stamp_ros + 0.05
    e.ingest(replace(sample(t, 10000), values=(0.8, None, 0.8)))
    m = e.estimate(t, t + 100)
    p.advance(m)
    current = p.at(
        t,
        now_ros=t,
        now_steady=t + 100,
        motion=m,
        measurement_steady=(t) + 100.0,
        stamp_ns=round((t) * 1e9),
    )
    mid = p.at(
        t - 0.025,
        now_ros=t,
        now_steady=t + 100,
        motion=m,
        measurement_steady=(t - 0.025) + 100.0,
        stamp_ns=round((t - 0.025) * 1e9),
    )
    first = compose(inverse(old.pose), mid.pose)
    second = compose(inverse(mid.pose), current.pose)
    assert (first.x, first.y, first.yaw) == pytest.approx(
        (second.x, second.y, second.yaw), abs=1e-10
    )


def test_partial_expiry_bounds_error_against_nonzero_clipped_nominal_path():
    e = MotionEstimation(acceleration=(0.0, 0.0, 0.0))
    p = PoseEstimation(linear_error_mps=0.0, angular_error_rps=0.0, lateral_drift_mps=0.0)
    e.ingest(sample(1.0, 1, yaw=0.4))
    first = e.estimate(1.9, 101.9)
    p.advance(first)
    e.ingest(sample(2.1, 1, source="imu", yaw=0.4, missing_linear=True))
    current_motion = e.estimate(2.1, 102.1)
    assert current_motion.vx.value is None and current_motion.healthy
    p.advance(current_motion)
    old = p.at(
        2.05,
        now_ros=2.1,
        now_steady=102.1,
        motion=current_motion,
        measurement_steady=(2.05) + 100.0,
        stamp_ns=round((2.05) * 1e9),
    )
    current = p.at(
        2.1,
        now_ros=2.1,
        now_steady=102.1,
        motion=current_motion,
        measurement_steady=(2.1) + 100.0,
        stamp_ns=round((2.1) * 1e9),
    )
    result = p.relative_error(old, current)
    # The old rate .2/.4 expires halfway through the stored .2s step, so the
    # nominal integrated rates are .1/.2. Unknown motion can oppose that nominal.
    assert result.position_m is None and result.yaw_rad is None
    assert result.software_position_m >= (p.unknown_speed + 0.1) * 0.05 - 1e-10
    assert result.software_yaw_rad >= (p.unknown_yaw + 0.2) * 0.05 - 1e-10


def test_exact_three_axis_reproduction_is_bounded_without_resetting_global_error():
    rows = []
    for duration in (0.15, 10.05, 60.05):
        p, _, old, current = run(duration, lateral=0.0)  # Exact input (0.2, 0, 0).
        budget = p.relative_error(old, current)
        rows.append(
            (
                budget.software_position_m,
                budget.software_yaw_rad,
                current.prediction_error.software_position_m,
            )
        )
    assert rows[1][:2] == pytest.approx(rows[0][:2], abs=1e-10)
    assert rows[2][:2] == pytest.approx(rows[0][:2], abs=1e-10)
    assert rows[2][2] > rows[1][2] > rows[0][2]


def test_epoch_nanosecond_recorded_path_requires_exact_continuous_coverage():
    from senior_control.motion.estimator import MotionPath, MotionPathSegment
    from senior_control.sensors.sample_time import StampInterval

    e = MotionEstimation()
    assert e.ingest(sample(1.0, 1, lateral=0.0))
    motion = e.estimate(1.0, 101.0)
    axes = (motion.vx, motion.vy, motion.wz)
    start = 1_700_000_000_000_000_001
    end = start + 100_000_000
    segment = MotionPathSegment(StampInterval(start, end), 101.0, 101.1, axes, None)
    path = MotionPath("exact", end, 101.1, (segment,), axes, None)
    motion = replace(motion, path=path)
    p = PoseEstimation()
    delta, _, roots, travel = p._recorded_prediction(motion, start, end)
    assert delta.x == pytest.approx(0.02)
    assert delta.y == 0 and delta.yaw == 0
    assert travel == pytest.approx(0.02) and roots == motion.vx.roots
    # A real one-nanosecond gap must not disappear through float rounding.
    motion = replace(motion, path=replace(path, stamp_ns=end + 1))
    with pytest.raises(ValueError, match="history was evicted"):
        p._recorded_prediction(motion, start, end + 1)
