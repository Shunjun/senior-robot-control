"""Original ROS stamp paths and immutable steady deadlines through real consumers.

Retired: arbitrary ROS labels, foreign boot/tick proofs, world-axis voting and
rate/correction matrices. Standard stamps provide one coordinate, source identity
and observable clock generations fence replacement; receipt rates are not inferred.
"""

from dataclasses import replace

import pytest
from senior_motion_estimation.estimator import MotionEstimation, MotionObservation, Origin
from senior_motion_estimation.kinematics import compose, exponential
from senior_motion_estimation.pose_estimator import PoseEstimation, PoseObservation
from sensor_time_test_support import sample_time

from vehicle_core.sample_time import StampInterval
from vehicle_core.values import Pose2D


def observation(
    host,
    stamp,
    sequence,
    *,
    values=(0.1, 0.0, 0.2),
    producer="driver",
    previous=None,
    source="laser",
    delay=0.0,
):
    time = sample_time(
        stamp_ns=round(stamp * 1e9),
        received_ros_ns=round((stamp + delay) * 1e9),
        received_steady_ns=round((host + delay) * 1e9),
    )
    root = Origin(source, producer, sequence, time)
    roots = (root,)
    interval = delta = None
    if previous is not None:
        old = previous.roots[-1]
        roots = (old, root)
        interval = StampInterval(old.time.stamp_ns, time.stamp_ns)
        delta = exponential(*values, (interval.end_ns - interval.start_ns) / 1e9)
    return MotionObservation(
        source,
        values,
        stamp,
        roots,
        quality=1,
        error=(0.0, 0.0, 0.0),
        independent=(True, True, True),
        measured_steady=host,
        stamp_ns=time.stamp_ns,
        measurement_interval=interval,
        measurement_delta=delta,
    )


def pose_at(owner, estimate):
    return owner.at(
        estimate.computed_ros,
        stamp_ns=estimate.path.stamp_ns,
        measurement_steady=estimate.path.host_steady,
        now_ros=estimate.computed_ros,
        now_steady=estimate.computed_steady,
        motion=estimate,
    )


def test_full_pose_path_keeps_distance_turn_and_support_with_sparse_or_dense_consumption():
    motion, dense, sparse = MotionEstimation(), PoseEstimation(), PoseEstimation()
    previous = None
    expected = Pose2D(0, 0, 0)
    for i, velocity in enumerate([(0.1, 0, 0), (0.1, 0, 0.2), (0.2, 0, -0.1), (0, 0, 0)]):
        stamp, host = i / 10, 10 + i / 10
        item = observation(host, stamp, i + 1, values=velocity, previous=previous)
        assert motion.ingest(item, now_ros=stamp, now_steady=host)
        estimate = motion.estimate(stamp, host)
        dense.advance(estimate)
        if i in (0, 3):
            sparse.advance(estimate)
        if previous is not None:
            expected = compose(expected, item.measurement_delta)
        previous = item
    a, b = pose_at(dense, estimate), pose_at(sparse, estimate)
    for result in (a, b):
        assert (result.pose.x, result.pose.y, result.pose.yaw) == pytest.approx(
            (expected.x, expected.y, expected.yaw)
        )
    assert a.prediction_error == b.prediction_error
    assert estimate.generation == 1 and estimate.path.stamp_ns == 300_000_000
    assert estimate.path.host_steady == 10.3


def test_repeated_original_stamp_does_not_integrate_again_and_stream_loss_expires():
    motion, poses = MotionEstimation(), PoseEstimation()
    item = observation(10, 0, 1, values=(0.5, 0, 0.2))
    assert motion.ingest(item, now_ros=0, now_steady=10)
    for i in range(5):
        if i:
            assert not motion.ingest(
                replace(item, sequence=i + 1, epoch="driver"),
                now_ros=i / 10,
                now_steady=10 + i / 10,
            )
        estimate = motion.estimate(i / 10, 10 + i / 10)
        poses.advance(estimate)
        assert pose_at(poses, estimate).pose == Pose2D(0, 0, 0)
        assert not estimate.path.segments
    expired = motion.estimate(1.01, 11.01)
    assert not expired.healthy and expired.path is None
    assert expired.vx.value is None and expired.wz.value is None


def test_queue_age_replay_and_retired_source_cannot_replace_current_path():
    motion = MotionEstimation()
    first = observation(10, 0, 1)
    assert motion.ingest(first, now_ros=0, now_steady=10)
    current = observation(10.1, 0.1, 2, previous=first)
    assert motion.ingest(current, now_ros=0.1, now_steady=10.1)
    before = motion.estimate(0.1, 10.1)
    assert not motion.ingest(current, now_ros=0.2, now_steady=10.2)
    queued = observation(10.2, 0.2, 3, delay=1.1)
    assert not motion.ingest(queued, now_ros=1.3, now_steady=11.3)
    assert motion._path_point.path_id == before.path.path_id
    assert motion._path_point.stamp_ns == before.path.stamp_ns
    replacement = observation(10.3, 0.3, 1, producer="driver-new")
    # The receiving lifecycle confirms replacement; a frame cannot self-authorize a boot.
    motion.confirm_source_boot("laser", "driver-new")
    assert motion.ingest(replacement, now_ros=0.3, now_steady=10.3)
    assert not motion.ingest(observation(10.4, 0.4, 4), now_ros=0.4, now_steady=10.4)


def test_nonfinite_motion_rejects_without_requiring_boot_or_tick_proof():
    good = observation(10, 0, 1)
    with pytest.raises(ValueError):
        replace(good, values=(float("nan"), 0, 0))
    assert good.stamp_ns == 0


def test_absolute_anchor_prediction_consumes_variable_recorded_pieces():
    owner, pose = MotionEstimation(), PoseEstimation()
    first = observation(10, 0, 1, values=(0.0, 0.0, 0.0))
    owner.ingest(first, now_ros=0, now_steady=10)
    pose.ingest(
        PoseObservation(
            "fix",
            Pose2D(2, 3, 0.1),
            0,
            first.roots,
            measurement_steady=10,
            stamp_ns=0,
            error_m=0,
            error_yaw_rad=0,
        )
    )
    expected, previous = Pose2D(2, 3, 0.1), first
    for i, values in enumerate([(0.1, 0, 0.2), (0, 0, 0), (0.2, 0, -0.3)], 1):
        incoming = observation(10 + i / 10, i / 10, i + 1, values=values, previous=previous)
        owner.ingest(incoming, now_ros=i / 10, now_steady=incoming.measured_steady)
        expected = compose(expected, incoming.measurement_delta)
        previous = incoming
    estimate = owner.estimate(0.3, 10.3)
    actual = pose.at(
        0.3,
        stamp_ns=300_000_000,
        measurement_steady=10.3,
        now_ros=0.3,
        now_steady=10.3,
        motion=estimate,
    )
    assert (actual.pose.x, actual.pose.y, actual.pose.yaw) == pytest.approx(
        (expected.x, expected.y, expected.yaw), abs=1e-12
    )
