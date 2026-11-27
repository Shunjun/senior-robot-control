"""Synthetic analytic tolerances registered before running any bags."""

import math
from dataclasses import replace

import pytest
from senior_motion_estimation.observations import (
    Frame,
    Source,
    SourceStatus,
)
from senior_motion_estimation.sample_types import PointObservation
from senior_motion_estimation.source_admission import AdmissionOwner
from senior_motion_estimation.temporal import TemporalConfig, TemporalResolver
from sensor_time_test_support import sample_time
from test_motion_sources import config


def point(source, t, seq, values, epoch="p", covariance=None, model=None):
    return PointObservation(
        source,
        Frame.BODY,
        sample_time(
            stamp_ns=round((t) * 1e9),
            received_ros_ns=round((t) * 1e9),
            received_steady_ns=round((t + 100) * 1e9),
        ),
        epoch,
        seq,
        0.9,
        SourceStatus.VALID,
        values,
        covariance,
        model,
        None,
    )


def owner():
    return AdmissionOwner(
        replace(config(), recovery_samples=1, recovery_span_ms=0), event_capacity=128
    )


def resolver():
    return TemporalResolver(TemporalConfig(1.0, 128, 80, 80, 80, True))


def feed(o, events):
    for p in sorted(events, key=lambda p: p.end_ros):
        assert o.ingest(p, now_ros=p.end_ros, now_steady=p.received_steady).accepted


def test_burst_events_are_not_lost_and_gyro_only_has_no_translation_delta():
    o = owner()
    r = resolver()
    feed(o, [point(Source.IMU, i * 0.02, i + 1, (0.5,)) for i in range(6)])
    s = o.snapshot(after_cursor=0, now_ros=0.1, now_steady=100.1)
    assert len([e for e in s.events if e.sample is not None]) == 6
    r.consume(s)
    w = r.resolve(round((0.0) * 1e9), round((0.1) * 1e9), s)
    imu = w.by_source(Source.IMU)
    assert imu.values == (None, None, pytest.approx(0.5))
    assert imu.delta is None
    assert imu.original_keys


def test_30_50_hz_causal_zoh_turning_prediction_is_complete_but_not_observed_lateral():
    o = owner()
    r = resolver()
    feed(
        o,
        [point(Source.IMU, i * 0.02, i + 1, (0.5,)) for i in range(6)]
        + [point(Source.WHEEL, i / 30, i + 1, (0.3, 0.5)) for i in range(4)],
    )
    s = o.snapshot(after_cursor=0, now_ros=0.1, now_steady=100.1)
    r.consume(s)
    wheel = r.resolve(round((0.0) * 1e9), round((0.1) * 1e9), s).by_source(Source.WHEEL)
    assert wheel.values[0] == pytest.approx(0.3, abs=1e-9, rel=0)
    assert wheel.values[1] is None
    assert wheel.predicted_delta.x == pytest.approx(0.3 / 0.5 * math.sin(0.05), abs=1e-9, rel=0)
    assert wheel.predicted_delta.y == pytest.approx(
        0.3 / 0.5 * (1 - math.cos(0.05)), abs=1e-9, rel=0
    )
    assert wheel.delta is None


def test_gap_epoch_interruption_and_cursor_overrun_are_explicit():
    o = owner()
    r = resolver()
    feed(o, [point(Source.IMU, 0.0, 1, (0.5,)), point(Source.WHEEL, 0.2, 1, (0.0, 0.0))])
    s = o.snapshot(after_cursor=0, now_ros=0.2, now_steady=100.2)
    r.consume(s)
    assert (
        r.resolve(round((0.1) * 1e9), round((0.2) * 1e9), s).by_source(Source.IMU).reason
        == "hold_expired"
    )
    bad = replace(point(Source.IMU, 0.21, 2, (0.5,)), status=SourceStatus.UNAVAILABLE)
    assert not o.ingest(bad, now_ros=0.21, now_steady=100.21).accepted
    n = o.snapshot(after_cursor=s.cursor, now_ros=0.21, now_steady=100.21)
    r.consume(n)
    assert (
        not r.resolve(round((0.0) * 1e9), round((0.1) * 1e9), n).by_source(Source.IMU).original_keys
    )
    small = AdmissionOwner(
        replace(config(), recovery_samples=1, recovery_span_ms=0), event_capacity=2
    )
    feed(small, [point(Source.IMU, i * 0.02, i + 1, (0.5,)) for i in range(4)])
    assert small.snapshot(after_cursor=0, now_ros=0.06, now_steady=100.06).journal_gap


@pytest.mark.parametrize("yaw,enu", [(0, (0.3, 0)), (math.pi / 2, (0, 0.3))])
def test_raw_gps_uses_each_causal_heading_not_only_window_start(yaw, enu):
    from senior_motion_estimation.observations import Heading

    from vehicle_core.values import Pose2D

    o, r = owner(), resolver()
    events = [point(Source.IMU, i * 0.02, i + 1, (0.0,)) for i in range(6)]
    gps = [
        replace(
            point(Source.GPS, i / 30, i + 1, enu),
            frame=Frame.ENU,
            body_from_sensor=Pose2D(0, 0, 0),
        )
        for i in range(4)
    ]
    for p in sorted(events + gps, key=lambda p: p.end_ros):
        heading = Heading(
            yaw,
            0,
            sample_time(
                stamp_ns=round((p.end_ros) * 1e9),
                received_ros_ns=round((p.end_ros) * 1e9),
                received_steady_ns=round((p.received_steady) * 1e9),
            ),
            "h",
            "synthetic known yaw",
        )
        assert o.ingest(
            p,
            now_ros=p.end_ros,
            now_steady=p.received_steady,
            heading=heading if p.source is Source.GPS else None,
        ).accepted
    s = o.snapshot(after_cursor=0, now_ros=0.1, now_steady=100.1)
    r.consume(s)
    result = r.resolve(round((0) * 1e9), round((0.1) * 1e9), s).by_source(Source.GPS)
    assert result.reason == "covered"
    assert result.values == pytest.approx((0.3, 0, None), abs=1e-9, rel=0)


def test_invalid_new_epoch_interrupts_resolver_and_retires_old_packets():
    o, r = owner(), resolver()
    feed(o, [point(Source.IMU, i * 0.02, i + 1, (0.5,)) for i in range(6)])
    s = o.snapshot(after_cursor=0, now_ros=0.1, now_steady=100.1)
    r.consume(s)
    bad = replace(
        point(Source.IMU, 0.12, 1, (0.5,), epoch='{"boot_id":"p","generation":2}'),
        status=SourceStatus.UNAVAILABLE,
    )
    assert not o.ingest(bad, now_ros=0.12, now_steady=100.12).accepted
    s = o.snapshot(after_cursor=s.cursor, now_ros=0.12, now_steady=100.12)
    assert any(e.sample is None for e in s.events)
    r.consume(s)
    assert (
        r.resolve(round((0) * 1e9), round((0.1) * 1e9), s).by_source(Source.IMU).reason
        == "source_unavailable"
    )
    old = point(Source.IMU, 0.14, 9, (0.5,))
    assert o.ingest(old, now_ros=0.14, now_steady=100.14).reason == "old_source_generation"
    fresh = point(Source.IMU, 0.14, 2, (0.5,), epoch='{"boot_id":"p","generation":2}')
    assert o.ingest(fresh, now_ros=0.14, now_steady=100.14).accepted


def test_se2_delta_preserves_sensor_extrinsic_and_displacement_covariance():
    from senior_motion_estimation.kinematics import compose, exponential, inverse
    from senior_motion_estimation.observations import Covariance, CovarianceKind
    from senior_motion_estimation.sample_types import DeltaObservation

    from vehicle_core.values import Pose2D

    o, r = owner(), resolver()
    extrinsic = Pose2D(0.7, -0.2, 0.4)
    expected = exponential(0.3, 0.02, 0.5, 0.1)
    sensor = compose(compose(inverse(extrinsic), expected), extrinsic)
    covariance = Covariance(
        (0.001, 0.002, 0.0001), CovarianceKind.DIAGONAL_ONLY, "synthetic displacement", None
    )
    p = DeltaObservation(
        Source.LASER,
        Frame.SENSOR,
        sample_time(
            stamp_ns=round((0) * 1e9),
            received_ros_ns=round((0) * 1e9),
            received_steady_ns=round(((100.1) - ((0.1) - (0))) * 1e9),
        ),
        sample_time(
            stamp_ns=round((0.1) * 1e9),
            received_ros_ns=round((0.1) * 1e9),
            received_steady_ns=round((100.1) * 1e9),
        ),
        "p",
        1,
        0.9,
        SourceStatus.VALID,
        sensor,
        covariance,
        "synthetic",
        extrinsic,
    )
    assert o.ingest(p, now_ros=0.1, now_steady=100.1).accepted
    s = o.snapshot(after_cursor=0, now_ros=0.1, now_steady=100.1)
    r.consume(s)
    result = r.resolve(round((0) * 1e9), round((0.1) * 1e9), s).by_source(Source.LASER)
    assert result.values == pytest.approx((0.3, 0.02, 0.5), abs=1e-9, rel=0)
    assert result.delta.x == pytest.approx(expected.x, abs=1e-9, rel=0)
    assert result.inputs[0].original.covariance is covariance
    assert result.variances[0] != covariance.values[0]  # displacement is not velocity


def test_invalid_extrinsic_rejected_after_valid_owner_without_watermark_change():
    from senior_motion_estimation.observations import Heading

    from vehicle_core.values import Pose2D

    o = owner()
    h = Heading(
        0,
        0,
        sample_time(
            stamp_ns=round((0) * 1e9),
            received_ros_ns=round((0) * 1e9),
            received_steady_ns=round((100) * 1e9),
        ),
        "h",
        "synthetic",
    )
    good = replace(
        point(Source.GPS, 0, 1, (0.3, 0)), frame=Frame.ENU, body_from_sensor=Pose2D(0, 0, 0)
    )
    assert o.ingest(good, now_ros=0, now_steady=100, heading=h).accepted
    with pytest.raises(ValueError):
        Pose2D(float("nan"), 0, 0)
    bad = replace(good, sequence=99, body_from_sensor="unknown")
    assert o.ingest(bad, now_ros=0, now_steady=100, heading=h).reason == "invalid_extrinsic"
    next_sample = replace(
        good,
        sequence=2,
        time=sample_time(
            stamp_ns=round((0.02) * 1e9),
            received_ros_ns=round((0.02) * 1e9),
            received_steady_ns=round((100.02) * 1e9),
            clock_generation=1,
        ),
    )
    assert o.ingest(next_sample, now_ros=0.02, now_steady=100.02, heading=h).accepted


def test_point_contract_cannot_admit_valid_laser_by_fabricating_interval():
    o = owner()
    p = point(Source.LASER, 0, 1, (0.3, 0, 0))
    assert p.start_ros == p.end_ros
    assert o.ingest(p, now_ros=0, now_steady=100).reason == "laser_requires_delta_or_interval"


def test_repeated_snapshot_and_clock_rollback_do_not_keep_gyro_history():
    o, r = owner(), resolver()
    feed(o, [point(Source.IMU, i * 0.02, i + 1, (0.5,)) for i in range(6)])
    s = o.snapshot(after_cursor=0, now_ros=0.1, now_steady=100.1)
    for _ in range(3):
        r.consume(s)
        result = r.resolve(round((0) * 1e9), round((0.1) * 1e9), s).by_source(Source.IMU)
        assert len(result.original_keys) == 5
        assert result.values[2] == pytest.approx(0.5)
    reset = o.snapshot(after_cursor=s.cursor, now_ros=0.01, now_steady=100.2)
    r.consume(reset)
    assert (
        r.resolve(round((0) * 1e9), round((0.01) * 1e9), reset).by_source(Source.IMU).values[2]
        is None
    )


def test_capacity_eviction_and_future_left_sample_never_invent_coverage():
    from senior_motion_estimation.temporal import TemporalConfig, TemporalResolver

    o = owner()
    feed(o, [point(Source.IMU, i * 0.02, i + 1, (0.5,)) for i in range(6)])
    s = o.snapshot(after_cursor=0, now_ros=0.1, now_steady=100.1)
    r = TemporalResolver(TemporalConfig(1, 2, 80, 80, 80, True))
    r.consume(s)
    w = r.resolve(round((0) * 1e9), round((0.1) * 1e9), s)
    assert "history_capacity_eviction" in w.diagnostics
    assert w.by_source(Source.IMU).reason == "missing_left_coverage"


def test_interval_rotation_alias_is_explicit_not_a_crash_or_wrapped_prediction():
    from senior_motion_estimation.sample_types import DeltaObservation

    from vehicle_core.values import Pose2D

    o, r = owner(), resolver()
    p = DeltaObservation(
        Source.LASER,
        Frame.SENSOR,
        sample_time(
            stamp_ns=round((0) * 1e9),
            received_ros_ns=round((0) * 1e9),
            received_steady_ns=round(((100.1) - ((0.1) - (0))) * 1e9),
        ),
        sample_time(
            stamp_ns=round((0.1) * 1e9),
            received_ros_ns=round((0.1) * 1e9),
            received_steady_ns=round((100.1) * 1e9),
        ),
        "p",
        1,
        1,
        SourceStatus.VALID,
        Pose2D(0, 0, 3.2),
        None,
        None,
        Pose2D(0, 0, 0),
    )
    assert o.ingest(p, now_ros=0.1, now_steady=100.1).accepted
    s = o.snapshot(after_cursor=0, now_ros=0.1, now_steady=100.1)
    r.consume(s)
    assert (
        r.resolve(round((0) * 1e9), round((0.1) * 1e9), s).by_source(Source.LASER).reason
        == "rotation_exceeds_log_domain"
    )


def test_wheel_without_declared_lateral_motion_assumption_is_not_se2_mean():
    o, r = owner(), resolver()
    r = TemporalResolver(replace(r.config, assume_nonholonomic=False))
    feed(
        o,
        [point(Source.IMU, i * 0.02, i + 1, (0.5,)) for i in range(6)]
        + [point(Source.WHEEL, i / 30, i + 1, (0.3, 0.5)) for i in range(4)],
    )
    s = o.snapshot(after_cursor=0, now_ros=0.1, now_steady=100.1)
    r.consume(s)
    result = r.resolve(round((0) * 1e9), round((0.1) * 1e9), s).by_source(Source.WHEEL)
    assert result.values[0] is None
    assert result.reason == "wheel_motion_assumption_missing"
    assert result.predicted_delta is None


def test_owner_recovery_blocks_both_dependencies_until_same_epoch_gyro_ready():
    from senior_motion_estimation.observations import Heading

    from vehicle_core.values import Pose2D

    o = AdmissionOwner(config(), event_capacity=128)
    r = resolver()

    def batch(t, seq, imu):
        events = [
            point(Source.WHEEL, t, seq, (0.3, 0)),
            replace(
                point(Source.GPS, t, seq, (0.5, 0)),
                frame=Frame.ENU,
                body_from_sensor=Pose2D(0, 0, 0),
            ),
            imu,
        ]
        for p in events:
            h = Heading(
                0,
                0,
                sample_time(
                    stamp_ns=round((t) * 1e9),
                    received_ros_ns=round((t) * 1e9),
                    received_steady_ns=round((100 + t) * 1e9),
                ),
                "h",
                "synthetic known orientation",
            )
            o.ingest(
                p, now_ros=t, now_steady=100 + t, heading=h if p.source is Source.GPS else None
            )
        s = o.snapshot(after_cursor=r.cursor, now_ros=t, now_steady=100 + t)
        r.consume(s)
        return s

    for i in range(11):
        batch(i / 50, i + 1, point(Source.IMU, i / 50, i + 1, (0,)))
    batch(
        0.22,
        12,
        replace(
            point(Source.IMU, 0.22, 1, (0,), epoch='{"boot_id":"p","generation":2}'),
            status=SourceStatus.UNAVAILABLE,
        ),
    )
    batch(0.24, 13, point(Source.IMU, 0.24, 2, (0,), epoch='{"boot_id":"p","generation":2}'))
    cold = batch(0.26, 14, point(Source.IMU, 0.26, 3, (0,), epoch='{"boot_id":"p","generation":2}'))
    assert dict(cold.sources)[Source.GPS].ready and dict(cold.sources)[Source.WHEEL].ready
    assert not dict(cold.sources)[Source.IMU].ready
    for _ in range(3):
        w = r.resolve(round((0.24) * 1e9), round((0.26) * 1e9), cold)
        for src in (Source.GPS, Source.WHEEL):
            assert w.by_source(src).reason == "dependency_imu_recovering"
            assert not w.by_source(src).original_keys
    for i, t in enumerate([0.28, 0.30, 0.32, 0.34]):
        warm = batch(
            t, 15 + i, point(Source.IMU, t, 4 + i, (0,), epoch='{"boot_id":"p","generation":2}')
        )
    assert dict(warm.sources)[Source.IMU].ready
    w = r.resolve(round((0.24) * 1e9), round((0.34) * 1e9), warm)
    for src in (Source.GPS, Source.WHEEL):
        assert w.by_source(src).reason == "covered"
        assert all(
            epoch == '{"boot_id":"p","generation":2}'
            for source, epoch, seq in w.by_source(src).original_keys
            if source == "imu"
        )


@pytest.mark.parametrize(
    "max_gap,max_hold,reason", [(0.04, 0.2, "sample_gap"), (0.2, 0.04, "hold_expired")]
)
def test_single_left_tail_and_held_use_same_coverage_ceiling(max_gap, max_hold, reason):
    from senior_motion_estimation.temporal import CoverageError

    o = owner()
    feed(o, [point(Source.IMU, 0, 1, (0.1,))])
    feed(o, [point(Source.WHEEL, 0.2, 1, (0.0, 0.0))])
    s = o.snapshot(after_cursor=0, now_ros=0.2, now_steady=100.2)
    r = TemporalResolver(TemporalConfig(1, 128, (max_hold) * 1000, (max_gap) * 1000, 80, True))
    r.consume(s)
    limit = min(max_gap, max_hold)
    assert (
        r.resolve(round((0) * 1e9), round((limit) * 1e9), s).by_source(Source.IMU).reason
        == "covered"
    )
    assert r._held(Source.IMU, round((limit) * 1e9), s).original.sequence == 1
    assert r.resolve(round((0) * 1e9), round((0.1) * 1e9), s).by_source(Source.IMU).reason == reason
    with pytest.raises(CoverageError, match=reason):
        r._held(
            Source.IMU, round((0.1) * 1e9), s
        )  # direct consistency check of the helper used by GPS/wheel
