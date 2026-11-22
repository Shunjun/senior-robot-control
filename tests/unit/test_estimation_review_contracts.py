"""Review regressions: original age attempts and uncertainty through actual composition."""

import logging
from dataclasses import replace

import pytest
from control_process_test_support import generic_motion_profile
from senior_motion_estimation.estimator import MotionEstimation
from senior_motion_estimation.motion_sensor_runtime import MotionSensorRuntime
from senior_motion_estimation.pose_estimator import PoseEstimation, PoseObservation
from sensor_time_test_support import sample_time
from test_reference_nanosecond_boundary import replay  # noqa: F401
from test_unified_estimator import observation

from vehicle_core.diagnostics import RuntimeDiagnostics
from vehicle_core.values import Pose2D


def reporter():
    emitted = []
    d = RuntimeDiagnostics(lambda level, data: emitted.append((level, data)))
    d.pump(0)
    return d, emitted


def summary(d, emitted):
    d.pump(5)
    return next(v for level, v in emitted if level == logging.DEBUG)


def test_source_use_includes_conflict_loser_and_expired_without_axis_or_view_duplicates():
    d, emitted = reporter()
    s = MotionSensorRuntime(generic_motion_profile(), d)
    s.ingest_observation(observation("laser", (0.9, None, 0.9), stamp=9.4), 10, 20)
    s.ingest_observation(observation("wheel", (0.1, None, 0.1), stamp=9.9), 10, 20)
    s.estimate(10, 20)
    s.estimate(10.5, 20.5)  # laser expired, wheel still live
    assert s.latest.vx.observed
    out = summary(d, emitted)
    laser = out["streams"]["sensor.use.laser"]
    assert laser["count"] == 2 and laser["over_hard"] == 1
    assert laser["max"] == pytest.approx(1.1)
    assert out["sensor_results"]["use.laser.not_selected"] == 1
    assert out["sensor_results"]["use.laser.expired"] == 1
    assert out["streams"]["sensor.use.wheel"]["count"] == 2


def test_raw_late_and_invalid_clock_are_reported_without_fabricated_age():
    d, emitted = reporter()
    s = MotionSensorRuntime(generic_motion_profile(), d)
    late = observation("laser", (0.2, None, 0.1), stamp=8.9)
    s.ingest_observation(late, 10, 20)
    # The same original observation is not another raw observation on replay.
    s.ingest_observation(late, 10, 20)
    bad = replace(
        observation("imu", (None, None, 0.1)),
        roots=(
            replace(
                late.roots[0],
                family="imu",
                time=sample_time(
                    stamp_ns=round((10.2) * 1e9),
                    received_ros_ns=round((10.2) * 1e9),
                    received_steady_ns=round((20.2) * 1e9),
                ),
            ),
        ),
        measured_ros=10.2,
        measured_steady=20.2,
    )
    with pytest.raises(ValueError):
        s.ingest_observation(bad, 10, 20)
    s.estimate(10, 20)
    out = summary(d, emitted)
    assert out["streams"]["sensor.raw.laser"]["count"] == 1
    assert out["streams"]["sensor.raw.laser"]["over_hard"] == 1
    assert "sensor.raw.imu" not in out["streams"]
    assert out["sensor_results"]["raw.imu.invalid_time"] == 1
    assert out["sensor_results"]["raw.laser.expired"] == 2  # two attempts, one original age sample


def test_relative_integration_accumulates_known_motion_errors():
    e = MotionEstimation()
    o = replace(observation("laser", (0.4, None, 0.1), stamp=10), error=(0.2, None, 0.3))
    e.ingest(o)
    p = PoseEstimation()
    p.advance(e.estimate(10, 20))
    p.advance(e.estimate(10.2, 20.2))
    r = p.at(
        10.2,
        now_ros=10.2,
        now_steady=20.2,
        motion=e.estimate(10.2, 20.2),
        measurement_steady=(10.2) + 10.0,
        stamp_ns=round((10.2) * 1e9),
    )
    assert r.error_m is not None and r.error_m >= 0.04
    assert r.error_yaw_rad is not None and r.error_yaw_rad >= 0.06 - 1e-12


def test_common_unknown_absolute_anchor_cancels_but_motion_error_does_not():
    e = MotionEstimation()
    raw = replace(observation("laser", (0.4, None, 0.1), stamp=9.5), error=(0.2, None, 0.3))
    e.ingest(raw)
    motion = e.estimate(10, 20)
    p = PoseEstimation()
    p.ingest(
        PoseObservation(
            "gps",
            Pose2D(4, 2, 0.2),
            9.5,
            raw.roots,
            measurement_steady=(raw.roots)[0].measurement_steady,
            stamp_ns=(raw.roots)[0].coordinate,
        )
    )
    old = p.at(
        9.5,
        now_ros=10,
        now_steady=20,
        motion=motion,
        measurement_steady=(9.5) + 10.0,
        stamp_ns=round((9.5) * 1e9),
    )
    current = p.at(
        10,
        now_ros=10,
        now_steady=20,
        motion=motion,
        measurement_steady=(10) + 10.0,
        stamp_ns=round((10) * 1e9),
    )
    assert old.error_m is current.error_m is None
    relative = p.relative_error(old, current)
    assert relative.position_m > 0 and relative.yaw_rad > 0
    assert relative.position_m == pytest.approx(current.prediction_error.position_m)
    # Independent fixes have no proof of common-mode cancellation.
    unrelated = replace(current, anchor_roots=(replace(raw.roots[0], family="other"),))
    unknown = p.relative_error(old, unrelated)
    assert unknown.position_m is unknown.yaw_rad is None
    assert unknown.software_position_m > relative.software_position_m


def test_raw_replay_dedup_survives_another_sample_and_quality_zero_is_observed():
    d, emitted = reporter()
    s = MotionSensorRuntime(generic_motion_profile(), d)
    first = observation("imu", (None, None, 0.1), stamp=9.5)
    second = observation("imu", (None, None, 0.1), stamp=9.6, sequence=2, quality=0)
    s.ingest_observation(first, 10, 20)
    s.ingest_observation(second, 10, 20)
    s.ingest_observation(first, 10, 20)
    out = summary(d, emitted)
    assert out["streams"]["sensor.raw.imu"]["count"] == 2
    assert out["sensor_results"]["raw.imu.unavailable"] == 1


def test_invalid_candidate_clock_has_use_rejection_counter_without_age():
    d, emitted = reporter()
    s = MotionSensorRuntime(generic_motion_profile(), d)
    future = observation("laser", (0.3, None, 0.1), stamp=10.2)
    s.estimator.ingest(future)  # Offline/unclocked ingress; owner still checks at evaluation.
    s.estimate(10, 20)
    out = summary(d, emitted)
    assert "sensor.use.laser" not in out["streams"]
    assert out["sensor_results"]["use.laser.invalid_time"] == 1


def test_rejected_native_receipt_uses_original_identity_without_live_state_mutation(monkeypatch):
    from types import SimpleNamespace as N

    from senior_motion_estimation import motion_observation_adapter as adapter
    from senior_motion_estimation.observations import Source

    d, emitted = reporter()
    monkeypatch.setattr("vehicle_core.diagnostics.process_diagnostics", lambda: d)
    monkeypatch.setattr(adapter, "time", N(monotonic=lambda: 21.0))
    p = generic_motion_profile()
    s = MotionSensorRuntime(p, d)
    core = adapter.NativeObservations(p.admission(), p.temporal(), p.fusion(), sample_sink=s.ingest)
    core.receipt_ros = 10
    provenance = adapter.PublisherOrigin(
        b"wheel", b"wheel", "native_message_info_and_writer_binding"
    )
    msg = N(
        header=N(stamp=N(sec=9, nanosec=800_000_000), frame_id="odom"),
        child_frame_id="base_link",
        twist=N(twist=N(linear=N(x=0.1), angular=N(z=0.1))),
    )
    core.wheel(
        msg,
        provenance,
        10,
        20,
        timing=sample_time(
            stamp_ns=round((9.8) * 1e9),
            received_ros_ns=round((9.8) * 1e9),
            received_steady_ns=round((19.8) * 1e9),
            clock_generation=1,
        ),
        now_steady=20,
        generation=1,
        sequence=1,
    )
    bridge = object.__new__(adapter.NativeObservationRosBridge)
    bridge.core = core
    bridge.node = N(get_clock=lambda: N(now=lambda: N(nanoseconds=11_000_000_000)))
    before = core._sequences.copy()
    bridge._record_rejected_receipt(
        msg,
        dict(
            dict(
                publisher_gid=b"wheel",
                received_timestamp=10_000_000_000,
                native_received_steady=20,
                receipt_generation=1,
                receipt_sequence=1,
            ),
            sample_time=sample_time(
                stamp_ns=round((9.8) * 1e9),
                received_ros_ns=round((9.8) * 1e9),
                received_steady_ns=round((19.8) * 1e9),
            ),
        ),
        Source.WHEEL,
    )
    out = summary(d, emitted)
    assert out["streams"]["sensor.raw.wheel"]["count"] == 1
    assert out["streams"]["sensor.use.wheel"]["max"] == pytest.approx(1.2)
    assert core._sequences == before
    assert s.estimator._frontiers["wheel"].identity_sequence == 1


def test_known_anchor_yaw_error_affects_global_pose_but_cancels_in_relative_pose():
    e = MotionEstimation()
    raw = replace(observation("laser", (1.0, None, 0.0), stamp=9.5), error=(0.1, None, 0.1))
    e.ingest(raw)
    motion = e.estimate(10, 20)
    p = PoseEstimation()
    p.ingest(
        PoseObservation(
            "gps",
            Pose2D(0, 0, 0),
            9.5,
            raw.roots,
            error_m=0.0,
            error_yaw_rad=0.2,
            measurement_steady=(raw.roots)[0].measurement_steady,
            stamp_ns=(raw.roots)[0].coordinate,
        )
    )
    old = p.at(
        9.5,
        now_ros=10,
        now_steady=20,
        motion=motion,
        measurement_steady=(9.5) + 10.0,
        stamp_ns=round((9.5) * 1e9),
    )
    current = p.at(
        10,
        now_ros=10,
        now_steady=20,
        motion=motion,
        measurement_steady=(10) + 10.0,
        stamp_ns=round((10) * 1e9),
    )
    relative = p.relative_error(old, current)
    assert current.error_m >= 0.1 + current.prediction_error.position_m - 1e-12
    assert relative.position_m == current.prediction_error.position_m


def test_raw_identity_dedup_and_attempt_results_remain_distinct_across_flush():
    from vehicle_core.sensor_policy import default_policy

    d, emitted = reporter()
    age = default_policy().age(measured_steady=19.5, received_steady=20, now_steady=20)
    d.sensor_attempt("wheel", "raw", age, ("accepted",), key=("boot", 1))
    d.sensor_attempt("wheel", "raw", age, ("rejected",), key=("boot", 1))
    first = summary(d, emitted)
    assert first["streams"]["sensor.raw.wheel"]["count"] == 1
    assert first["sensor_results"]["raw.wheel.rejected"] == 1
    d.sensor_attempt("wheel", "raw", age, ("rejected",), key=("boot", 1))
    d.pump(10)
    second = [v for level, v in emitted if level == logging.DEBUG][-1]
    assert "sensor.raw.wheel" not in second["streams"]
    assert second["sensor_results"] == {"raw.wheel.rejected": 1}


def test_space_positional_api_and_bounded_diagnostic_registrations():
    from senior_motion_estimation.space_estimator import SpaceEstimation, SpaceObservation
    from shapely.geometry import box

    from vehicle_core.sensor_policy import default_policy

    roots = observation("scan", (None, None, None)).roots
    space = SpaceEstimation()
    assert space.ingest(
        SpaceObservation(
            "scan",
            box(0, 0, 1, 1),
            box(-1, -1, 2, 2),
            roots,
            0.1,
            "scan_reference",
            Pose2D(0, 0, 0),
            "default",
        )
    )
    out = []
    d = RuntimeDiagnostics(lambda level, data: out.append(data), max_streams=4)
    d.pump(0)
    age = default_policy().age(measured_steady=19.5, received_steady=20, now_steady=20)
    for n in range(100):
        d.sensor_attempt(f"bounded_{n}", "raw", age, ("rejected",), key=n)
    d.pump(5)
    report = out[-1]
    assert len(report["streams"]) <= 4 and len(report["sensor_results"]) <= 4
    assert report["unregistered"] > 0


def test_unknown_space_errors_use_explicit_model_policy_instead_of_zero():
    from senior_motion_estimation.space_estimator import SpaceEstimation, SpaceObservation
    from shapely.geometry import box

    owner = SpaceEstimation(software_pose_error_m=0.07)
    roots = observation("scan", (None, None, None)).roots
    owner.ingest(
        SpaceObservation(
            "scan",
            box(0, 0, 1, 1),
            box(-1, -1, 2, 2),
            roots,
            error_m=None,
            pose_in_frame=Pose2D(1, 0, 0),
        )
    )
    result = owner.at(10, 20, reference_pose=Pose2D(0, 0, 0), alignment_error_m=None)
    assert result.alignment_error_m is None and result.unknown_pose_sources == ("scan",)
    assert result.software_alignment_error_m == 0.07
    assert result.occupied.area > 1.0
