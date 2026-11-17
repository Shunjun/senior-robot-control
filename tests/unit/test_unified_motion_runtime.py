from types import SimpleNamespace as N

import pytest
from control_process_test_support import generic_motion_profile
from senior_robot.motion_sensor_runtime import MotionSensorRuntime
from sensor_time_test_support import sample_time

from senior_control.models import Pose2D
from senior_control.motion.observations import Frame, Source, SourceStatus
from senior_control.motion.sample_types import PointObservation
from senior_control.runtime.diagnostics import RuntimeDiagnostics


def point(source, stamp, received, values, sequence=1, *, measurement_steady=None):
    # This synthetic source has zero delivery delay unless explicitly supplied.
    return PointObservation(
        source,
        Frame.BODY,
        sample_time(
            stamp_ns=round((stamp) * 1e9),
            received_ros_ns=round((stamp) * 1e9),
            received_steady_ns=round(
                (received if measurement_steady is None else measurement_steady) * 1e9
            ),
        ),
        "boot",
        sequence,
        1.0,
        SourceStatus.VALID,
        values,
        None,
        None,
        Pose2D(0, 0, 0),
    )


def test_actual_runtime_single_wheel_works_without_pair_and_does_not_make_body_truth():
    s = MotionSensorRuntime(generic_motion_profile(), RuntimeDiagnostics(lambda *_: None))
    s.ingest(point(Source.WHEEL, 9.9, 19.9, (0.4, 0.1)), 10, 20, 9.9)
    ref, tracking, _ = s.estimate(10, 20)
    assert tracking is not None and tracking.unified
    assert tracking.velocity.vx == 0.4 and tracking.velocity.wz == 0.1
    assert s.latest.vy.value == 0 and s.latest.healthy
    assert s.independent is None
    seq = tracking.sequence
    assert s.estimate(10.1, 20.1)[1].sequence == seq  # predicted ticks are not new samples
    assert s.estimate(10.9, 20.9)[1] is None


def test_runtime_partial_axis_hard_cutoff_survives_other_axis_updates():
    s = MotionSensorRuntime(generic_motion_profile(), RuntimeDiagnostics(lambda *_: None))
    s.ingest(point(Source.WHEEL, 9.5, 19.5, (0.4, 0.1)), 10, 20, 9.5)
    s.estimate(10, 20)
    s.invalidate(Source.WHEEL)
    s.ingest(point(Source.IMU, 10.3, 20.3, (0.2,)), 10.3, 20.3, 10.3)
    ref, tracking, _ = s.estimate(10.4, 20.4)
    assert s.latest.vx.value == 0.4 and not s.latest.vx.observed
    assert tracking.observed_axes == (False, True)
    s.estimate(10.5, 20.5)
    assert s.latest.vx.value is None and s.latest.wz.observed


def test_leaf_bridge_local_sink_does_not_run_old_numerical_qualification():
    from senior_robot.motion_observation_adapter import NativeObservations

    p = generic_motion_profile()
    seen = []
    core = NativeObservations(
        p.admission(), p.temporal(), p.fusion(), sample_sink=lambda *x: seen.append(x) or True
    )
    assert core.owner is None and core.fusion is None
    assert core.ingest(point(Source.WHEEL, 9.5, 19.5, (0.4, 0.1)), 10, 20)
    assert len(seen) == 1


def test_older_but_live_remaining_source_survives_legacy_controller_receipt_frontier():
    from senior_control.models import Velocity
    from senior_control.motion.constraints import ConstraintConfig, ConstraintEngine
    from senior_control.motion.controller import Feedback, MotionController

    p = generic_motion_profile()
    c = MotionController(
        ConstraintEngine(ConstraintConfig((1.0, 0.5), 0.15, 0.05, 256, 0.005)),
        p.controller(),
        Velocity(0, 0),
        Velocity(0, 0),
    )
    first = Feedback(Velocity(0.2, 0), 19.9, 20.2, "laser+wheel", 1, True, "unified", unified=True)
    remaining = Feedback(Velocity(0.2, 0), 19.8, 20.8, "wheel", 2, True, "unified", unified=True)
    assert c._admit_feedback("body", first, 20) is not None
    assert c._admit_feedback("body", remaining, 20.2) is not None
    assert c._admit_feedback("body", remaining, 20.8) is None


def test_raw_wheel_imu_stillness_does_not_depend_on_independent_samples():
    from test_unified_estimator import observation

    s = MotionSensorRuntime(generic_motion_profile(), RuntimeDiagnostics(lambda *_: None))
    from dataclasses import replace

    laser = replace(
        observation("laser", (0.0, None, 0.0), stamp=9.9), independent=(True, True, True)
    )
    s.ingest_observation(laser, 10, 20)
    for i in range(7):
        t = 10 + i * 0.1
        s.ingest(point(Source.WHEEL, t, t + 10, (0.0, 0.0), sequence=i + 1), t, t + 10, t)
        s.ingest(point(Source.IMU, t, t + 10, (0.0,), sequence=i + 1), t, t + 10, t)
        s.estimate(t, t + 10)
    assert s.latest.healthy and s.stationary(t + 10)
    for i in range(7):
        t = 10.7 + i * 0.1
        o = replace(
            observation("laser", (0.0, None, 0.0), stamp=t, sequence=i + 2),
            independent=(True, True, True),
        )
        s.ingest_observation(o, t, t + 10)
        s.estimate(t, t + 10)
    assert s.stationary(t + 10)


def test_laser_delta_retains_both_raw_capture_receipts_through_processing_delay():
    from senior_robot.motion_observation_adapter import NativeObservations
    from test_optional_motion_inputs import header, stamp

    p = generic_motion_profile()
    runtime = MotionSensorRuntime(p, RuntimeDiagnostics(lambda *_: None))
    ingress = NativeObservations(
        p.admission(), p.temporal(), p.fusion(), sample_sink=runtime.ingest
    )
    ingress.receipt_ros = 9.6
    first, second = header(9.5, 1, "raw-laser"), header(9.6, 2, "raw-laser")
    message = N(
        header=N(stamp=stamp(9.6), frame_id="base_link"),
        interval_start=stamp(9.5),
        source_epoch="matcher",
        valid=True,
        body_delta=N(x=0.04, y=0.0, theta=0.0),
        covariance=[0.0] * 9,
        response=0.8,
        first_capture=first,
        second_capture=second,
    )
    assert ingress.laser(message, b"writer", 9.6, 20.4, now_steady=20.4)
    assert runtime.estimate(9.6, 20.4)[1] is not None
    assert runtime.latest.vx.deadline_steady == pytest.approx(20.5)
    assert runtime.latest.vx.roots[0].received_steady == 19.5
    assert not ingress.laser(message, b"writer", 9.6, 20.45, now_steady=20.45)
    assert runtime.estimate(9.6, 20.5)[1] is None


def test_ros_imu_unavailable_sentinel_removes_only_imu_contribution():
    from senior_robot.motion_observation_adapter import NativeObservations
    from test_optional_motion_inputs import stamp

    profile = generic_motion_profile()
    runtime = MotionSensorRuntime(profile, RuntimeDiagnostics(lambda *_: None))
    ingress = NativeObservations(
        profile.admission(), profile.temporal(), profile.fusion(), sample_sink=runtime.ingest
    )
    runtime.ingest(point(Source.WHEEL, 9.9, 19.9, (0.2, 0.1)), 10, 20)
    msg = N(header=N(stamp=stamp(9.9), frame_id="base_link"), angular_velocity=N(z=0.1))
    assert ingress.imu(
        msg,
        b"imu",
        10,
        20,
        generation=1,
        sequence=1,
        timing=sample_time(
            stamp_ns=9_900_000_000,
            received_ros_ns=10_000_000_000,
            received_steady_ns=20_000_000_000,
        ),
    )
    assert runtime.estimate(10, 20)[1] is not None
    msg.header.stamp = stamp(10.1)
    msg.angular_velocity_covariance = [-1.0] + [0.0] * 8
    msg.angular_velocity.z = float("nan")
    assert ingress.imu(
        msg,
        b"imu",
        10.1,
        20.1,
        generation=1,
        sequence=2,
        timing=sample_time(
            stamp_ns=10_100_000_000,
            received_ros_ns=10_100_000_000,
            received_steady_ns=20_100_000_000,
        ),
    )
    runtime.estimate(10.1, 20.1)
    assert runtime.latest.healthy and runtime.latest.wz.sources == ("wheel",)


def test_bad_future_ingress_clock_does_not_replace_still_valid_earlier_input():
    from dataclasses import replace

    from test_unified_estimator import observation

    s = MotionSensorRuntime(generic_motion_profile(), RuntimeDiagnostics(lambda *_: None))
    old = observation("laser", (0.4, None, 0.1))
    s.ingest_observation(old, 10, 20)
    s.estimate(10, 20)
    future = replace(
        old,
        measured_ros=10.1,
        measured_steady=20.1,
        stamp_ns=10_100_000_000,
        roots=(
            replace(
                old.roots[0],
                time=sample_time(
                    stamp_ns=10_100_000_000,
                    received_ros_ns=10_100_000_000,
                    received_steady_ns=20_100_000_000,
                ),
                sequence=2,
            ),
        ),
    )
    with pytest.raises(ValueError):
        s.ingest_observation(future, 10.0, 20.0)
    assert s.estimate(10, 20.2)[1].velocity.vx == 0.4
    assert s.latest.vx.roots == old.roots


def test_invalid_scan_motion_withdraws_source_without_fabricating_endpoints():
    from senior_robot.motion_observation_adapter import NativeObservations
    from test_unified_estimator import observation

    profile = generic_motion_profile()
    runtime = MotionSensorRuntime(profile, RuntimeDiagnostics(lambda *_: None))
    ingress = NativeObservations(
        profile.admission(),
        profile.temporal(),
        profile.fusion(),
        sample_sink=runtime.ingest,
        source_invalidated=runtime.invalidate,
    )
    runtime.ingest_observation(observation("laser", (0.2, 0.0, 0.0)), 10, 20)
    assert runtime.estimate(10, 20)[1] is not None
    invalid = N(header=N(frame_id="base_link"), valid=False, reason="source_changed")
    assert not ingress.laser(invalid, b"writer", 10, 20)
    assert runtime.estimate(10, 20)[1] is None


def test_ros_rollback_revokes_raw_stillness_and_recovery_until_new_inputs():
    from dataclasses import replace

    s = MotionSensorRuntime(generic_motion_profile(), RuntimeDiagnostics(lambda *_: None))
    wheel = point(Source.WHEEL, 9.9, 19.9, (0.0, 0.0))
    imu = point(Source.IMU, 9.9, 19.9, (0.0,))
    assert s.ingest(wheel, 10, 20)
    assert s.ingest(imu, 10, 20)
    s.estimate(10, 20)
    generation = s.latest.generation
    assert s.stationary(20)
    assert s.recovery(20).manual_rearm_ready

    s.estimate(9, 20.1)
    assert s.latest.generation == generation + 1 and not s.latest.healthy
    assert s.raw_wheel is None and s.raw_imu is None
    assert s.raw_feedback("wheel", 20.1) is None
    assert s.raw_feedback("imu", 20.1) is None
    assert s.wheel_linear(20.1) is None
    assert not s.stationary(20.1) and s.recovery(20.1) is None
    # Even a higher sequence cannot revive the retired source segment.
    assert not s.ingest(replace(wheel, sequence=2), 9, 20.1)
    assert not s.ingest(replace(imu, sequence=2), 9, 20.1)
    s.estimate(9, 20.1)
    assert not s.stationary(20.1) and s.recovery(20.1) is None

    new_wheel = replace(point(Source.WHEEL, 9.1, 20.2, (0.0, 0.0)), epoch="new")
    new_imu = replace(point(Source.IMU, 9.1, 20.2, (0.0,)), epoch="new")
    assert s.ingest(new_wheel, 9.1, 20.2)
    s.estimate(9.1, 20.2)
    assert s.raw_feedback("wheel", 20.2) is not None
    assert not s.stationary(20.2) and s.recovery(20.2) is None
    assert s.ingest(new_imu, 9.1, 20.2)
    s.estimate(9.1, 20.2)
    recovery = s.recovery(20.2)
    assert recovery.healthy and recovery.stationary and recovery.manual_rearm_ready
    assert recovery.stamp_steady == 20.2
    assert recovery.deadline_steady == pytest.approx(21.2)
    s.estimate(9.2, 20.3)
    assert s.recovery(20.3).deadline_steady == recovery.deadline_steady
    assert not s.stationary(21.2) and s.recovery(21.2) is None
