import math

import pytest

from senior_control.config import DeviceProfile
from senior_control.models import BodyTwist, Pose2D, Velocity
from senior_control.safety.guard import evaluate_motion
from senior_control.safety.motion_estimator import MotionEstimator
from senior_control.safety.motion_runtime import (
    ImuSample,
    ProducerTopology,
    ScanInterval,
    SourceEpochFence,
    StampedObservation,
    StopAcknowledgement,
    TimedCommandBuffer,
    TimedImuBuffer,
    TimedPoseBuffer,
    TimedPoseSample,
    compose_pose,
    newer_scan_veto,
    prepare_safety_observation,
    qualify_interval,
    reference_steady_deadline,
    reframe_observation,
    relative_pose,
    select_safety_observation,
)
from senior_control.safety.self_filter import ScanObservation, filter_self
from senior_control.safety.slip import SlipMonitor


def test_reference_steady_deadline_is_bound_to_admission_and_original_age():
    deadline = reference_steady_deadline(20.0, 20.2, 100.0, 0.4)
    assert deadline == pytest.approx(100.2)
    assert reference_steady_deadline(20.0, 20.41, 100.0, 0.4) is None
    assert reference_steady_deadline(20.0, 19.9, 100.0, 0.4) is None
    assert reference_steady_deadline(20.0, 20.2, 100.0, float("nan")) is None


def test_pose_history_interpolates_rotation_wrap_and_uses_both_endpoints():
    history = TimedPoseBuffer(max_gap=0.3, max_samples=5, max_variance=0.1)
    assert history.add(TimedPoseSample(1.0, Pose2D(0, 0, 3.1), (0.01,) * 3, "boot"))
    assert history.add(TimedPoseSample(1.2, Pose2D(0.2, 0, -3.1), (0.01,) * 3, "boot"))
    mid = history.at(1.1)
    assert mid is not None
    assert mid.pose.x == pytest.approx(0.1)
    assert abs(mid.pose.yaw) == pytest.approx(math.pi)
    assert history.at(0.99) is None
    assert history.at(1.21) is None


def test_pose_history_rejects_rollback_epoch_change_and_bad_covariance():
    history = TimedPoseBuffer(max_gap=0.3, max_samples=5, max_variance=0.1)
    assert history.add(TimedPoseSample(1.0, Pose2D(0, 0, 0), (0.01,) * 3, "boot"))
    assert not history.add(TimedPoseSample(0.9, Pose2D(1, 0, 0), (0.01,) * 3, "boot"))
    assert history.at(1.0) is None
    assert history.add(TimedPoseSample(2.0, Pose2D(0, 0, 0), (0.01,) * 3, "new"))
    assert not history.add(TimedPoseSample(2.1, Pose2D(0, 0, 0), (0.01,) * 3, "other"))
    assert history.at(2.0) is None
    assert not history.add(TimedPoseSample(3.0, Pose2D(0, 0, 0), (math.nan,) * 3, "x"))


def test_relative_pose_includes_sensor_offset_and_rotation():
    # A body moves 0.5 m then turns 90 degrees. Its old laser (0.2 m ahead,
    # 90 degrees turned) is now behind and right of the new body frame.
    old_body = Pose2D(0, 0, 0)
    new_body = Pose2D(0.5, 0, math.pi / 2)
    old_sensor = Pose2D(0.2, 0, math.pi / 2)
    body_old_to_new = relative_pose(new_body, old_body)
    from senior_control.safety.motion_runtime import compose_pose

    sensor_in_current = compose_pose(body_old_to_new, old_sensor)
    assert sensor_in_current.x == pytest.approx(0.0, abs=1e-12)
    assert sensor_in_current.y == pytest.approx(0.3)
    assert sensor_in_current.yaw == pytest.approx(0.0)


def test_old_scan_geometry_is_transformed_into_current_body_frame():
    scan = ScanObservation(
        (1.0,),
        0.0,
        0.1,
        0.1,
        10.0,
        1.0,
        "ros",
        Pose2D(0.2, 0, math.pi / 2),
        1.0,
        "base_link",
        1.0,
        transform_static=True,
    )
    filtered = filter_self(scan, DeviceProfile(self_mask_calibrated=True))
    transformed = reframe_observation(filtered, Pose2D(0, 0, 0), Pose2D(0.5, 0, math.pi / 2), 1.2)
    assert transformed.scan.stamp == 1.0
    assert transformed.scan.reference_stamp == 1.2
    assert transformed.rays[0].origin == pytest.approx((0.0, 0.3), abs=1e-12)
    assert transformed.rays[0].endpoint == pytest.approx((1.0, 0.3), abs=1e-12)
    assert transformed.external_hits == transformed.rays


def test_safety_compensates_an_aged_scan_and_bounds_remaining_latency():
    scan = ScanObservation(
        (0.5,),
        0,
        0.1,
        0.1,
        10,
        1.0,
        "ros",
        Pose2D(0.2, 0, 0),
        1.0,
        "base_link",
        1.0,
        transform_static=True,
    )
    profile = DeviceProfile(
        self_mask_calibrated=True,
        footprint=[(-0.337, -0.27), (0.337, -0.27), (0.337, 0.27), (-0.337, 0.27)],
        braking_deceleration=1.0,
        yaw_braking_deceleration=1.0,
        reaction_time=0.1,
        hard_margin=0.02,
        observation_max_age=0.5,
    )
    history = TimedPoseBuffer(max_gap=0.3, max_samples=5, max_variance=0.1)
    history.add(TimedPoseSample(1.0, Pose2D(0, 0, 0), (0.001,) * 3, "boot"))
    history.add(TimedPoseSample(1.2, Pose2D(0.4, 0, 0), (0.001,) * 3, "boot"))
    result = prepare_safety_observation(
        filter_self(scan, profile),
        history,
        now=1.25,
        max_age=0.5,
        max_reference_lag=0.1,
        linear_bound=0.3,
        yaw_bound=0.5,
        radius=0.5,
    )
    assert result is not None
    assert result.observation.scan.stamp == 1.0
    assert result.observation.scan.reference_stamp == 1.2
    assert result.added_margin > 0.3 * 0.05
    decision = evaluate_motion(
        Velocity(0.1, 0), BodyTwist(0.1, 0.0, 0), [result.observation], profile, 1.25
    )
    assert decision.reason == "hard_contact"
    assert (
        prepare_safety_observation(
            filter_self(scan, profile),
            history,
            now=1.31,
            max_age=0.5,
            max_reference_lag=0.1,
            linear_bound=0.3,
            yaw_bound=0.5,
            radius=0.5,
        )
        is None
    )


def test_interpolated_pose_retains_a_bound_for_motion_between_samples():
    history = TimedPoseBuffer(max_gap=0.3, max_samples=5, max_variance=0.1)
    history.add(TimedPoseSample(1.0, Pose2D(0, 0, 0), (1e-8,) * 3, "boot"))
    history.add(TimedPoseSample(1.2, Pose2D(0, 0, 0), (1e-8,) * 3, "boot"))
    scan = ScanObservation(
        (1.0,),
        0,
        0.1,
        0.1,
        5.0,
        1.1,
        "ros",
        Pose2D(0, 0, 0),
        1.1,
        "base_link",
        1.1,
        transform_static=True,
    )
    result = prepare_safety_observation(
        filter_self(scan, DeviceProfile(self_mask_calibrated=True)),
        history,
        now=1.2,
        max_age=0.5,
        max_reference_lag=0.1,
        linear_bound=0.5,
        yaw_bound=0.4,
        radius=0.5,
    )
    assert result is not None
    # Equal endpoints do not establish that the body stayed there in between.
    assert result.added_margin >= 0.5 * 0.1


def test_new_scan_waiting_for_pose_does_not_discard_previous_fresh_geometry():
    history = TimedPoseBuffer(max_gap=0.3, max_samples=5, max_variance=0.1)
    history.add(TimedPoseSample(1.0, Pose2D(0, 0, 0), (0.001,) * 3, "boot"))
    history.add(TimedPoseSample(1.1, Pose2D(0.01, 0, 0), (0.001,) * 3, "boot"))

    def observation(stamp):
        scan = ScanObservation(
            (1.0,),
            0,
            0.1,
            0.1,
            5,
            stamp,
            "ros",
            Pose2D(0, 0, 0),
            stamp,
            "base_link",
            stamp,
            transform_static=True,
        )
        return filter_self(scan, DeviceProfile(self_mask_calibrated=True))

    samples = [
        StampedObservation(observation(1.1), 100.0),
        StampedObservation(observation(1.2), 100.1),
    ]
    kwargs = dict(
        now=1.2, max_age=0.5, max_reference_lag=0.2, linear_bound=0.3, yaw_bound=0.5, radius=0.5
    )
    selected = select_safety_observation(samples, history, steady=100.1, **kwargs)
    assert selected is not None and selected.observation.scan.stamp == 1.1
    assert select_safety_observation(samples, history, steady=100.6, **kwargs) is None


def test_newer_unaligned_scan_still_vetoes_new_hard_obstacle():
    profile = DeviceProfile(
        self_mask_calibrated=True,
        footprint=[(-0.337, -0.27), (0.337, -0.27), (0.337, 0.27), (-0.337, 0.27)],
        braking_deceleration=1.0,
        yaw_braking_deceleration=1.0,
        reaction_time=0.1,
        hard_margin=0.02,
        observation_max_age=0.5,
    )
    scan = ScanObservation(
        (0.19, 2.0),
        0.0,
        0.01,
        0.05,
        5.0,
        1.2,
        "ros",
        Pose2D(0.15699, 0, 0),
        1.2,
        "base_link",
        1.2,
        transform_static=True,
    )
    latest = StampedObservation(filter_self(scan, profile), 100.1)
    assert newer_scan_veto(
        latest,
        1.1,
        Velocity(0.05, 0),
        BodyTwist(0, 0, 0),
        profile,
        now=1.21,
        steady=100.11,
        linear_bound=0.3,
        yaw_bound=0.5,
        radius=0.5,
    )
    assert not newer_scan_veto(
        latest,
        1.2,
        Velocity(0.05, 0),
        BodyTwist(0, 0, 0),
        profile,
        now=1.21,
        steady=100.11,
        linear_bound=0.3,
        yaw_bound=0.5,
        radius=0.5,
    )


def test_newer_scan_veto_includes_the_full_physical_motion_envelope():
    profile = DeviceProfile(
        self_mask_calibrated=True,
        footprint=[(-0.337, -0.27), (0.337, -0.27), (0.337, 0.27), (-0.337, 0.27)],
        braking_deceleration=1.0,
        yaw_braking_deceleration=1.0,
        reaction_time=0.1,
        hard_margin=0.02,
        observation_max_age=0.5,
    )
    scan = ScanObservation(
        (0.74301, 2.0),
        0.0,
        0.01,
        0.05,
        5.0,
        1.2,
        "ros",
        Pose2D(0.15699, 0, 0),
        1.2,
        "base_link",
        1.2,
        transform_static=True,
    )
    latest = StampedObservation(filter_self(scan, profile), 100.1)
    arguments = dict(now=1.21, steady=100.11, linear_bound=0.3, yaw_bound=0.5, radius=0.5)
    assert not newer_scan_veto(
        latest, 1.1, Velocity(0.05, 0), BodyTwist(0, 0, 0), profile, **arguments
    )
    assert newer_scan_veto(
        latest,
        1.1,
        Velocity(0.05, 0),
        BodyTwist(0, 0, 0),
        profile,
        motion_sweep_margin=0.6,
        **arguments,
    )


def test_imu_integrates_the_exact_interval_and_rejects_missing_coverage():
    imu = TimedImuBuffer(max_gap=0.3, max_samples=8, max_variance=0.1)
    for stamp, rate in [(1.0, 0.0), (1.1, 1.0), (1.2, 2.0), (1.3, 3.0)]:
        assert imu.add(ImuSample(stamp, rate, 0.01))
    result = imu.integrate(1.05, 1.25)
    assert result is not None
    assert result.yaw_delta == pytest.approx(0.3)
    assert result.variance > 0
    assert imu.integrate(0.9, 1.2) is None
    assert not imu.add(ImuSample(1.2, 0.0, 0.01))
    assert imu.integrate(1.05, 1.25) is None


def test_imu_rejects_invalid_covariance_and_large_gap():
    imu = TimedImuBuffer(max_gap=0.15, max_samples=5, max_variance=0.1)
    assert imu.add(ImuSample(1.0, 0.0, 0.01))
    assert imu.add(ImuSample(1.2, 0.0, 0.01))
    assert imu.integrate(1.0, 1.2) is None
    assert not imu.add(ImuSample(1.3, 0.0, -1.0))


def test_fusion_uses_wheel_and_imu_at_scan_endpoints_and_detects_slip():
    wheel = TimedPoseBuffer(max_gap=0.2, max_samples=20, max_variance=0.1)
    imu = TimedImuBuffer(max_gap=0.2, max_samples=20, max_variance=0.1)
    for stamp, x in [(1.0, 0), (1.1, 0.05), (1.2, 0.1)]:
        wheel.add(TimedPoseSample(stamp, Pose2D(x, 0, 0), (0.001,) * 3, "odom"))
        imu.add(ImuSample(stamp, 0.0, 0.001))
    estimator = MotionEstimator(
        max_skew=0.001, min_quality=0.4, max_yaw_error=0.1, max_covariance=0.1
    )
    slip = SlipMonitor(
        startup_window=0.0, wheel_motion_limit=0.005, lateral_limit=0.03, yaw_limit=0.1
    )
    scan = ScanInterval(1.0, 1.2, Pose2D(0.1, 0, 0), (0.001,) * 3, 0.8)
    result = qualify_interval(scan, wheel, imu, Velocity(0.5, 0), estimator, slip)
    assert result is not None and not result.anomalous
    assert result.estimate.pose_delta.x == pytest.approx(0.1)
    stalled = ScanInterval(1.0, 1.2, Pose2D(0.0, 0, 0), (0.001,) * 3, 0.8)
    result = qualify_interval(stalled, wheel, imu, Velocity(0.5, 0), estimator, slip)
    assert result is not None and result.anomalous
    uncovered = ScanInterval(0.9, 1.2, Pose2D(0, 0, 0), (0.001,) * 3, 0.8)
    assert qualify_interval(uncovered, wheel, imu, Velocity(0.5, 0), estimator, slip) is None


def test_rejected_turn_is_not_treated_as_an_applied_turn():
    wheel = TimedPoseBuffer(max_gap=0.2, max_samples=4, max_variance=0.1)
    imu = TimedImuBuffer(max_gap=0.2, max_samples=4, max_variance=0.1)
    for stamp in (1.0, 1.1):
        wheel.add(TimedPoseSample(stamp, Pose2D(0, 0, 0), (0.001,) * 3, "odom"))
        imu.add(ImuSample(stamp, 0, 0.001))
    estimator = MotionEstimator(
        max_skew=0.001, min_quality=0.4, max_yaw_error=0.1, max_covariance=0.1
    )
    slip = SlipMonitor(
        startup_window=0, wheel_motion_limit=0.005, lateral_limit=0.03, yaw_limit=0.01
    )
    requested, applied = Velocity(0, 0.5), Velocity(0, 0)
    scan = ScanInterval(1.0, 1.1, Pose2D(0, 0, 0), (0.001,) * 3, 0.8)
    assert requested != applied
    qualified = qualify_interval(scan, wheel, imu, applied, estimator, slip)
    assert qualified is not None and not qualified.anomalous


def test_command_interval_integrates_changes_and_rejects_missing_history_and_rollback():
    commands = TimedCommandBuffer(max_gap=0.15, max_samples=10)
    assert commands.add(1.0, Velocity(0.2, 0.0))
    assert commands.add(1.1, Velocity(0.2, 0.0))
    assert commands.interval(1.02, 1.2).vx == pytest.approx(0.2)
    assert commands.interval(0.99, 1.1) is None
    assert commands.add(1.2, Velocity(0.0, 0.0))
    mixed = commands.interval(1.05, 1.21)
    assert mixed is not None and mixed.vx == pytest.approx(0.1875)
    assert not commands.add(1.1, Velocity(0.2, 0.0))
    assert commands.interval(1.02, 1.2) is None


def test_applied_command_pulses_integrate_without_inventing_requested_motion():
    commands = TimedCommandBuffer(max_gap=0.11, max_samples=10)
    for stamp, velocity in (
        (1.0, Velocity(0, 0)),
        (1.05, Velocity(0.05, 0)),
        (1.10, Velocity(0, 0)),
        (1.15, Velocity(0.05, 0)),
    ):
        assert commands.add(stamp, velocity)
    actual = commands.integrated(1.0, 1.2)
    assert actual is not None
    assert actual.start == 1.0 and actual.end == 1.2
    assert actual.pose_delta.x == pytest.approx(0.005)
    assert actual.velocity.vx == pytest.approx(0.025)
    assert actual.lateral_velocity == pytest.approx(0.0)


@pytest.mark.parametrize("replacement", [Velocity(0.08, 0), Velocity(0, 0), Velocity(0, 0.2)])
def test_ordered_same_instant_command_transition_only_changes_future_integration(replacement):
    commands = TimedCommandBuffer(max_gap=0.15, max_samples=10)
    assert commands.add(1.0, Velocity(0.04, 0))
    assert commands.add(1.1, Velocity(0.04, 0))
    before = commands.integrated(1.0, 1.1)
    bounds = commands.diagnostic_bounds()
    assert commands.replace_latest(1.1, replacement)
    assert commands.diagnostic_bounds() == bounds
    assert commands.integrated(1.0, 1.1) == before
    future = commands.integrated(1.1, 1.2)
    assert future is not None
    assert future.velocity.vx == pytest.approx(replacement.vx)
    assert future.velocity.wz == pytest.approx(replacement.wz)
    assert commands.integrated(1.1, 1.3) is None
    assert not commands.add(1.09, replacement)
    assert commands.diagnostic_bounds() == (None, None, 0)


@pytest.mark.parametrize("stamp", [1.0, 1.2, float("nan"), float("inf")])
def test_command_replacement_cannot_insert_history_or_change_timestamp(stamp):
    commands = TimedCommandBuffer(max_gap=0.15, max_samples=10)
    assert not commands.replace_latest(stamp, Velocity(0, 0))
    assert commands.add(1.1, Velocity(0.08, 0))
    assert not commands.replace_latest(stamp, Velocity(0, 0))
    assert commands.diagnostic_bounds() == (None, None, 0)


def test_changing_curvature_is_composed_on_se2_not_component_averaged():
    commands = TimedCommandBuffer(max_gap=0.51, max_samples=10)
    assert commands.add(1.0, Velocity(0.6, 0.2))
    assert commands.add(1.5, Velocity(0.1, 0.8))
    actual = commands.integrated(1.0, 2.0)
    assert actual is not None
    first = Pose2D(0.6 * math.sin(0.1) / 0.2, 0.6 * (1 - math.cos(0.1)) / 0.2, 0.1)
    second = Pose2D(0.1 * math.sin(0.4) / 0.8, 0.1 * (1 - math.cos(0.4)) / 0.8, 0.4)
    expected = compose_pose(first, second)
    assert actual.pose_delta.x == pytest.approx(expected.x)
    assert actual.pose_delta.y == pytest.approx(expected.y)
    assert actual.pose_delta.yaw == pytest.approx(expected.yaw)
    assert abs(actual.lateral_velocity) > 0.001


@pytest.mark.parametrize("second", [Velocity(-0.1, 0.0), Velocity(0.1, -0.2)])
def test_command_interval_rejects_opposite_direction_or_turn(second):
    commands = TimedCommandBuffer(max_gap=0.51, max_samples=10)
    assert commands.add(1.0, Velocity(0.1, 0.2))
    assert commands.add(1.5, second)
    assert commands.integrated(1.0, 2.0) is None


def test_retired_source_epochs_cannot_return_after_new_epoch_or_clock_reset():
    fence = SourceEpochFence(max_retired=2)
    assert fence.accept("boot-a")
    assert fence.accept("boot-b")
    assert not fence.accept("boot-a")
    assert fence.active == "boot-b"
    fence.retire_active()
    assert not fence.accept("boot-b")
    assert fence.accept("boot-c")
    # Exhaustion fails closed; old identities are never discarded to make room.
    assert not fence.accept("boot-d")
    assert not fence.accept("boot-a")
    assert not fence.accept("boot-c")


def test_repeated_invalid_evidence_from_one_new_epoch_does_not_exhaust_fence():
    fence = SourceEpochFence(max_retired=256)
    assert fence.accept("qualified-old")
    for _ in range(300):
        assert fence.accept("unqualified-new")
    assert not fence.accept("qualified-old")
    assert fence.accept("recovered-newer")
    assert not fence.accept("unqualified-new")


def test_stamp_frontier_blocks_old_or_equal_valid_after_new_invalid():
    from senior_control.safety.motion_runtime import SourceStampFrontier

    frontier = SourceStampFrontier()
    assert frontier.accept(100, eligible=True, receipt_ros_ns=110)
    assert frontier.accept(120, eligible=False, receipt_ros_ns=130)
    assert not frontier.accept(110, eligible=True, receipt_ros_ns=130)
    assert not frontier.accept(120, eligible=True, receipt_ros_ns=130)
    assert frontier.accept(140, eligible=True, receipt_ros_ns=150)
    assert not frontier.accept(130, eligible=False, receipt_ros_ns=150)
    assert frontier.accept(140, eligible=False, receipt_ros_ns=150)
    frontier.reset()
    assert frontier.accept(1, eligible=True, receipt_ros_ns=2)


def test_undated_invalid_demands_a_measurement_newer_than_its_receipt():
    from senior_control.safety.motion_runtime import SourceStampFrontier

    frontier = SourceStampFrontier()
    assert frontier.accept(100, eligible=True, receipt_ros_ns=110)
    assert frontier.accept(0, eligible=False, receipt_ros_ns=130)
    assert not frontier.accept(120, eligible=True, receipt_ros_ns=140)
    assert not frontier.accept(130, eligible=True, receipt_ros_ns=140)
    assert frontier.accept(140, eligible=True, receipt_ros_ns=150)


def test_stamp_frontier_requires_an_interval_start_at_or_after_failure():
    from senior_control.safety.motion_runtime import SourceStampFrontier

    frontier = SourceStampFrontier()
    assert frontier.accept(100, eligible=True, receipt_ros_ns=110)
    assert frontier.accept(120, eligible=False, receipt_ros_ns=125)
    assert frontier.accept(140, eligible=True, receipt_ros_ns=145)
    assert not frontier.interval_after_failure(119)
    assert frontier.interval_after_failure(120)
    assert not frontier.accept(110, eligible=False, receipt_ros_ns=145)
    assert frontier.interval_after_failure(120)
    frontier.reset()
    assert not frontier.interval_after_failure(0)
    assert frontier.interval_after_failure(1)
    assert frontier.accept(0, eligible=False, receipt_ros_ns=150)
    assert not frontier.interval_after_failure(149)
    assert frontier.interval_after_failure(150)


def test_forced_applied_zero_needs_a_new_requested_stop_before_release():
    acknowledgement = StopAcknowledgement()
    acknowledgement.trip()
    acknowledgement.applied(Velocity(0, 0))
    acknowledgement.requested(Velocity(0.2, 0))
    assert not acknowledgement.ready
    acknowledgement.requested(Velocity(0, 0))
    assert acknowledgement.ready
    acknowledgement.clear()
    assert not acknowledgement.ready


def test_producer_topology_rejects_duplicates_and_reports_replacement():
    topology = ProducerTopology("motion_fusion", "/")
    first = [("motion_fusion", "/", bytes([1] * 24))]
    second = [("motion_fusion", "/", bytes([2] * 24))]
    assert topology.observe(first) == (True, True)
    assert topology.observe(first) == (True, False)
    assert topology.observe(first + second) == (False, True)
    assert topology.observe(second) == (True, True)
    assert topology.observe([("other", "/", bytes([2] * 24))]) == (False, True)
