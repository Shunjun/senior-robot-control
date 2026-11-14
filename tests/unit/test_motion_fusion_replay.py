"""Offline adapters must preserve raw support and never use truth as admission."""

import pytest

from senior_control.motion.observations import Source
from tools.motion_fusion_replay import ReplayChain, replay_policy
from tools.motion_replay_records import NativeAdapter, TruthIndex


def test_native_imu_has_no_fabricated_interval_or_statistical_model():
    adapter = NativeAdapter("recording")
    row = {
        "topic": "/imu/data",
        "message": {
            "header": {"stamp": {"sec": 1, "nanosec": 0}, "frame_id": "imu_link"},
            "angular_velocity": {"z": 0.2},
            "angular_velocity_covariance": [0] * 9,
        },
    }
    p = adapter.convert(row, 5)
    assert p.source is Source.IMU
    assert p.start_ros == p.end_ros == 1
    assert p.received_steady == 5 and p.model_id is None and p.covariance.values == (0.0,)
    assert adapter.convert({"topic": "/ground_truth/odom"}, 5) is None


def test_truth_same_window_turning_and_no_gap_bridge():
    from senior_control.models import Pose2D
    from senior_control.motion.kinematics import exponential

    truth = TruthIndex(0.08, 100)
    for t in (0, 0.05, 0.1):
        truth.add(0, t, exponential(0.3, 0.02, 0.5, t))
    assert truth.average(0, 0, 0.1) == pytest.approx((0.3, 0.02, 0.5), abs=1e-9, rel=0)
    truth.add(0, 0.3, Pose2D(0, 0, 0))
    assert truth.average(0, 0.1, 0.3) is None
    assert truth.average(1, 0, 0.1) is None


def test_bounded_chain_consumes_all_events_without_ros_or_truth():
    from test_motion_temporal import point

    c = ReplayChain(replay_policy())
    for i in range(16):
        t = i * 0.02
        c.ingest(point(Source.IMU, t, i + 1, (0.1,)), t, 100 + t)
        if i % 2 == 0:
            c.ingest(point(Source.WHEEL, t, i + 1, (0.3, 0.1)), t, 100 + t)
    result = c.window(0.2, 0.3, 0.3, 100.3)
    assert result.components[0].value == pytest.approx(0.3)
    assert result.components[2].value == pytest.approx(0.1)
    assert result.upper is None
    assert not any(x.numerically_fused for x in result.components)


def test_native_stamp_matches_integer_bag_clock_without_false_future_ulp():
    from tools.motion_replay_records import stamp

    # Same ROS instant must have one representation before strict future admission.
    assert stamp({"sec": 1, "nanosec": 140000000}) == 1140000000 / 1e9


def test_labeled_synthetic_offline_consumer_fuses_and_recovers_after_combined_failure():
    from tools.motion_fusion_synthetic import synthetic_demo

    rows = synthetic_demo()
    assert all(row["label"] == "synthetic inputs only" for row in rows)
    modes = [r["result"]["mode"].value for r in rows]
    assert modes == [
        "gps_imu_reference",
        "fused_reference",
        "wheel_imu_limited",
        "unavailable",
        "unavailable",
        "fused_reference",
        "fused_reference",
        "fused_reference",
    ]
    assert rows[1]["result"]["components"][0]["value"] == pytest.approx(0.34, abs=1e-9, rel=0)
    assert rows[1]["result"]["components"][0]["variance"] == pytest.approx(0.016, abs=1e-9, rel=0)
    assert all(r["result"]["upper"] is None for r in rows)
    assert rows[2]["result"]["components"][1]["value"] is None


def test_native_wheel_retains_relevant_cross_covariance_without_certifying_it():
    c = [0.0] * 36
    c[0], c[5], c[30], c[35] = 0.02, 0.001, 0.001, 0.03
    row = {
        "topic": "/wheel/odom",
        "message": {
            "header": {"stamp": {"sec": 1, "nanosec": 0}},
            "child_frame_id": "base_link",
            "twist": {"twist": {"linear": {"x": 0.3}, "angular": {"z": 0.2}}, "covariance": c},
        },
    }
    p = NativeAdapter("recording").convert(row, 5)
    assert p.covariance.values == (0.02, 0.001, 0.001, 0.03)
    assert p.model_id is None
