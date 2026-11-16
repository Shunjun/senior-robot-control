"""Physical measurement paths and host TTL stay independent across real consumers."""

from dataclasses import replace

import pytest
from source_time_test_support import DOMAIN, capture_time, wire_time

from senior_control.models import Pose2D
from senior_control.motion.estimator import MotionEstimation, MotionObservation, Origin
from senior_control.motion.kinematics import compose, exponential
from senior_control.motion.pose_estimator import PoseEstimation
from senior_control.sensors.time_proof import MeasurementInterval, source_time


def observation(
    host,
    tick,
    sequence,
    *,
    values=(0.1, 0.0, 0.2),
    ros=100.0,
    producer="driver",
    world="world:0",
    previous=None,
    source="laser",
    delay=0.0,
):
    proof = capture_time(host, sequence=sequence, epoch=producer, tick=tick, world=world)
    root = Origin(
        source,
        producer,
        sequence,
        ros,
        ros + 9,
        host + delay,
        measurement_steady=host,
        measurement=proof.measurement,
    )
    roots = (root,)
    interval = delta = None
    if previous is not None:
        old = previous.roots[-1]
        roots = (old, root)
        interval = MeasurementInterval(old.coordinate, root.coordinate)
        delta = exponential(*values, interval.duration)
    return MotionObservation(
        source,
        values,
        ros,
        roots,
        quality=1,
        error=(0.0, 0.0, 0.0),
        independent=(True, True, True),
        measured_steady=host,
        measurement=proof.measurement,
        measurement_interval=interval,
        measurement_delta=delta,
    )


def pose_at(owner, estimate, label):
    return owner.at(
        label,
        measurement=estimate.path.measurement,
        measurement_steady=estimate.path.host_steady,
        now_ros=label,
        now_steady=estimate.computed_steady,
        motion=estimate,
    )


@pytest.mark.parametrize("rate", [0.5, 1.0, 2.0])
@pytest.mark.parametrize("correction", [-1e6, 0, 1e6])
def test_full_pose_path_keeps_physical_distance_turn_and_support_at_any_host_rate(rate, correction):
    motion, dense, sparse = MotionEstimation(), PoseEstimation(), PoseEstimation()
    previous = None
    expected = Pose2D(0, 0, 0)
    values = [(0.1, 0, 0), (0.1, 0, 0.2), (0.2, 0, -0.1), (0, 0, 0)]
    for i, velocity in enumerate(values):
        host, tick = 10 + i / 10 / rate, i / 10
        label = 2e6 + correction * (i % 2)
        item = observation(host, tick, i + 1, values=velocity, ros=label, previous=previous)
        assert motion.ingest(item, now_ros=label, now_steady=host)
        estimate = motion.estimate(label, host)
        dense.advance(estimate)
        if i in (0, len(values) - 1):
            sparse.advance(estimate)
        if previous is not None:
            expected = compose(expected, item.measurement_delta)
        previous = item
    a, b = pose_at(dense, estimate, label), pose_at(sparse, estimate, label)
    assert (a.pose.x, a.pose.y, a.pose.yaw) == pytest.approx((expected.x, expected.y, expected.yaw))
    assert (b.pose.x, b.pose.y, b.pose.yaw) == pytest.approx((expected.x, expected.y, expected.yaw))
    assert a.prediction_error == b.prediction_error
    assert estimate.generation == 1
    assert estimate.computed_steady == host
    assert estimate.path.measurement.seconds == 0.3
    assert estimate.path.host_steady == host


def test_simulation_pause_does_not_integrate_host_elapsed_and_true_stream_loss_expires():
    motion, poses = MotionEstimation(), PoseEstimation()
    for i in range(5):
        host = 10 + i / 10
        item = observation(host, 0, i + 1, values=(0.5, 0, 0.2), ros=1000 - i * 10)
        assert motion.ingest(item, now_ros=item.measured_ros, now_steady=host)
        estimate = motion.estimate(item.measured_ros, host)
        poses.advance(estimate)
        pose = pose_at(poses, estimate, item.measured_ros)
        assert pose.pose == Pose2D(0, 0, 0)
        assert not estimate.path.segments
    expired = motion.estimate(1e9, host + (motion.timing.hard_age_ms / 1000) + 0.01)
    assert not expired.healthy
    assert expired.path is None
    assert expired.vx.value is None and expired.wz.value is None


def test_queue_age_replay_and_old_world_cannot_replace_current_path():
    motion = MotionEstimation()
    first = observation(10, 0, 1)
    assert motion.ingest(first, now_ros=100, now_steady=10)
    current = observation(10.1, 0.1, 2, previous=first)
    assert motion.ingest(current, now_ros=1e9, now_steady=10.1)
    before = motion.estimate(1e9, 10.1)
    assert not motion.ingest(current, now_ros=1, now_steady=10.1)
    queued = observation(10.2, 0.2, 3, delay=(motion.timing.hard_age_ms / 1000) + 0.1)
    assert not motion.ingest(queued, now_ros=1, now_steady=queued.roots[0].received_steady)
    # Rejection did not overwrite the acquisition/path frontier or grant a new deadline.
    assert motion._path_point.path_id == before.path.path_id
    assert motion._path_point.measurement == before.path.measurement
    reset = observation(12.0, 0, 4, world="world:1")
    assert motion.ingest(reset, now_ros=100, now_steady=12.0)
    after = motion.estimate(100, 12.0)
    assert after.path.path_id != before.path.path_id
    assert not after.path.segments
    old = observation(10.25, 0.3, 5, world="world:0")
    assert not motion.ingest(old, now_ros=100, now_steady=12.0)
    assert motion._path_point.measurement.epoch == "world:1"


def test_different_worlds_do_not_vote_together_and_producer_restart_keeps_world_axis():
    motion = MotionEstimation()
    a = observation(10, 0, 1, source="laser")
    assert motion.ingest(a, now_ros=100, now_steady=10)
    b = observation(10.1, 0.1, 1, source="rgbd", producer="camera")
    assert motion.ingest(b, now_ros=100, now_steady=10.1)
    same = motion.estimate(100, 10.1)
    assert {r.family for r in same.vx.roots} == {"laser", "rgbd"}
    restarted = observation(10.2, 0.2, 1, source="rgbd", producer="camera-next")
    assert motion.ingest(restarted, now_ros=10, now_steady=10.2)
    assert restarted.coordinate.same_axis(a.coordinate)
    foreign = observation(10.3, 0, 1, source="foreign", producer="other", world="other-world:0")
    assert motion.ingest(foreign, now_ros=1e9, now_steady=10.3)
    different = motion.estimate(1e9, 10.3)
    assert {r.family for r in different.vx.roots} == {"foreign"}


def test_explicit_zero_tick_is_valid_but_missing_tick_wrong_boot_and_nan_reject():
    wire = wire_time(10, tick=0)
    proof = source_time(wire, domain=DOMAIN)
    assert proof.measurement.seconds == 0
    wire.clock_domain = "00000000-0000-0000-0000-000000000002"
    with pytest.raises(ValueError, match="foreign"):
        source_time(wire, domain=DOMAIN)
    wire = wire_time(10, tick=0)
    del wire.measurement_tick_ns
    with pytest.raises((ValueError, AttributeError)):
        source_time(wire, domain=DOMAIN)
    good = observation(10, 0, 1)
    with pytest.raises(ValueError):
        replace(good, values=(float("nan"), 0, 0))
    with pytest.raises(ValueError):
        replace(proof, measurement_tick_ns=None)


@pytest.mark.parametrize("rate", [0.5, 1.0, 2.0])
def test_absolute_anchor_prediction_consumes_variable_recorded_pieces(rate):
    from senior_control.motion.pose_estimator import PoseObservation

    owner = MotionEstimation()
    pose = PoseEstimation()
    first = observation(10.0, 0.0, 1, values=(0.0, 0.0, 0.0))
    owner.ingest(first, now_ros=100.0, now_steady=10.0)
    pose.ingest(
        PoseObservation(
            "fix",
            Pose2D(2.0, 3.0, 0.1),
            100.0,
            first.roots,
            measurement_steady=10.0,
            measurement=first.coordinate,
            error_m=0.0,
            error_yaw_rad=0.0,
        )
    )
    expected = Pose2D(2.0, 3.0, 0.1)
    previous = first
    for i, values in enumerate([(0.1, 0.0, 0.2), (0.0, 0.0, 0.0), (0.2, 0.0, -0.3)], 1):
        incoming = observation(
            10.0 + i * 0.1 / rate, i * 0.1, i + 1, values=values, previous=previous
        )
        owner.ingest(incoming, now_ros=100.0 + i, now_steady=incoming.measured_steady)
        expected = compose(expected, incoming.measurement_delta)
        previous = incoming
    estimate = owner.estimate(1.0, previous.measured_steady)
    actual = pose.at(
        1.0,
        measurement=previous.coordinate,
        measurement_steady=previous.measured_steady,
        now_ros=1.0,
        now_steady=previous.measured_steady,
        motion=estimate,
    )
    assert (actual.pose.x, actual.pose.y, actual.pose.yaw) == pytest.approx(
        (expected.x, expected.y, expected.yaw), abs=1e-12
    )
