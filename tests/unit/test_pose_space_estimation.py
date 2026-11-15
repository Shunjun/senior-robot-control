import math

import pytest
from shapely.geometry import Point, box
from test_unified_estimator import observation

from senior_control.models import Pose2D
from senior_control.motion.estimator import MotionEstimation
from senior_control.motion.pose_estimator import PoseEstimation, PoseObservation
from senior_control.safety.space_estimator import SpaceEstimation, SpaceObservation


def test_pose_interpolation_wrap_and_motion_prediction_without_150ms_gate():
    e = MotionEstimation()
    o = observation("wheel", (0.4, None, 0.0), stamp=9.5)
    e.ingest(o)
    m = e.estimate(10, 20)
    p = PoseEstimation()
    p.ingest(PoseObservation("odom", Pose2D(0, 0, math.radians(179)), 9.5, o.roots))
    p.ingest(
        PoseObservation(
            "odom",
            Pose2D(0.1, 0, math.radians(-179)),
            9.75,
            observation("wheel", (0.4, None, 0.0), stamp=9.75, sequence=2).roots,
        )
    )
    mid = p.at(9.625, now_ros=10, now_steady=20, motion=m)
    assert abs(abs(mid.pose.yaw) - math.pi) < 1e-6
    result = p.at(10, now_ros=10, now_steady=20, motion=m)
    assert result.pose.x < 0.01  # .4m/s along almost-negative world X for .25s
    assert result.kind == "predicted_pose"


def test_space_never_averages_distinct_obstacles_or_fills_unobserved_space():
    o = observation("scan", (0.1, None, 0.0))
    s = SpaceEstimation()
    s.ingest(SpaceObservation("left", box(-1, -1, -0.8, -0.8), box(-2, -2, 0, 2), o.roots))
    s.ingest(SpaceObservation("right", box(1, 1, 1.2, 1.2), box(0, -2, 2, 2), o.roots))
    r = s.at(10, 20)
    assert r.occupied.covers(Point(-0.9, -0.9)) and r.occupied.covers(Point(1.1, 1.1))
    assert not r.free.covers(Point(-0.9, -0.9))
    assert not r.free.covers(Point(5, 5))
    r = s.at(10.9, 20.9)
    assert r.free.is_empty


def test_single_motion_source_builds_relative_pose_without_new_absolute_claim():
    e = MotionEstimation()
    e.ingest(observation("laser", (0.4, None, 0.0), 9.9))
    m = e.estimate(10, 20)
    p = PoseEstimation()
    p.advance(m)
    r = p.at(10, now_ros=10, now_steady=20, motion=m)
    assert r.pose.x == pytest.approx(0.04)
    assert r.kind == "relative_dead_reckoning"


def test_prediction_keeps_motion_dependencies_and_unknown_errors():
    e = MotionEstimation()
    wheel = observation("wheel", (0.4, None, 0.1), stamp=9.8)
    e.ingest(wheel)
    motion = e.estimate(10.0, 20.0)
    p = PoseEstimation()
    gps = observation("gps", (0.1, None, None), stamp=9.5)
    p.ingest(PoseObservation("gps_pose", Pose2D(2, 3, 0), 9.5, gps.roots))
    result = p.at(10.0, now_ros=10.0, now_steady=20.0, motion=motion)
    assert {r.family for r in result.roots} == {"gps", "wheel"}
    assert result.error_m is None and result.error_yaw_rad is None
    assert result.kind == "predicted_pose"
    assert len(result.sources) == 1


def test_absolute_expiry_preserves_coordinate_origin_in_relative_prediction():
    p = PoseEstimation()
    e = MotionEstimation()
    o = observation("wheel", (0.4, None, 0.0), stamp=9.5)
    e.ingest(o)
    p.ingest(PoseObservation("odom", Pose2D(4, 2, 0), 9.5, o.roots))
    first = e.estimate(10.0, 20.0)
    p.advance(first)
    before = p.at(10.0, now_ros=10.0, now_steady=20.0, motion=first)
    assert before.pose.x == pytest.approx(4.2)
    e.ingest(observation("wheel", (0.4, None, 0.0), stamp=10.5, sequence=2))
    second = e.estimate(10.5, 20.5)
    p.advance(second)
    after = p.at(10.5, now_ros=10.5, now_steady=20.5, motion=second)
    assert after.kind == "relative_dead_reckoning"
    assert after.pose.x == pytest.approx(4.4)
    assert after.pose.y == 2


def test_space_aligns_sensor_poses_before_union_and_keeps_frame_epochs_separate():
    roots = observation("scan", (0.1, None, None)).roots
    s = SpaceEstimation(frame="odom", frame_epoch="map1")
    s.ingest(
        SpaceObservation(
            "a",
            box(0, 0, 0.2, 0.2),
            box(-1, -1, 1, 1),
            roots,
            frame="odom",
            pose_in_frame=Pose2D(1, 0, 0),
            frame_epoch="map1",
        )
    )
    r = s.at(10, 20, reference_pose=Pose2D(0, 0, 0))
    assert r.occupied.covers(Point(1.1, 0.1)) and not r.occupied.covers(Point(0.1, 0.1))
    with pytest.raises(ValueError, match="invalid"):
        s.ingest(
            SpaceObservation(
                "b", box(0, 0, 1, 1), box(0, 0, 2, 2), roots, frame="odom", frame_epoch="map2"
            )
        )
    s.reset_frame("map2")
    assert s.at(10, 20).free.is_empty


def test_pose_zero_quality_veto_and_cached_republication_do_not_create_new_fix():
    from dataclasses import replace

    p = PoseEstimation()
    original = PoseObservation(
        "gps_pose", Pose2D(2, 3, 0), 9.9, observation("gps", (0.1, None, None)).roots
    )
    assert p.ingest(original)
    invalid = replace(original, quality=0.0, roots=(replace(original.roots[0], sequence=2),))
    assert p.ingest(invalid)
    with pytest.raises(ValueError, match="cannot support"):
        p.at(10.0, now_ros=10.0, now_steady=20.0)
    assert not p.ingest(replace(original, roots=(replace(original.roots[0], sequence=3),)))
