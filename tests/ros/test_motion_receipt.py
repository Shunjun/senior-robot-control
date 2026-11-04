"""Gate must bind motion freshness to Fusion's original scan receipt."""

from types import SimpleNamespace

import pytest


@pytest.mark.parametrize(
    ("receipt_steady", "receipt_ros_ns", "accepted"),
    [
        (100.0, 20_050_000_000, True),
        (99.0, 20_050_000_000, False),
        (100.41, 20_050_000_000, False),
        (0.0, 20_050_000_000, False),
        (float("nan"), 20_050_000_000, False),
        (100.0, 20_210_000_000, False),
        (100.0, 0, False),
        (100.0, 19_300_000_000, False),
    ],
    ids=[
        "delayed-original",
        "expired-steady",
        "future-steady",
        "missing",
        "nan",
        "future-ros",
        "missing-ros",
        "expired-ros",
    ],
)
def test_gate_uses_original_fusion_receipt(monkeypatch, receipt_steady, receipt_ros_ns, accepted):
    rclpy = pytest.importorskip("rclpy")
    from rclpy.parameter import Parameter
    from rclpy.time import Time
    from senior_motion_msgs.msg import MotionEvidence
    from senior_robot import safety_node as module

    rclpy.init()
    gate = module.SafetyGate(
        parameter_overrides=[
            Parameter("observation_max_age", value=0.8),
            Parameter("measurement_max_age", value=0.8),
            Parameter("motion_min_quality", value=0.3),
            Parameter("motion_max_variance", value=0.1),
            Parameter("motion_max_interval", value=0.3),
        ]
    )
    monkeypatch.setattr(module, "time", SimpleNamespace(monotonic=lambda: 100.4))
    monkeypatch.setattr(
        gate,
        "get_clock",
        lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=20_200_000_000)),
    )
    monkeypatch.setattr(gate, "_clock_ok", lambda _now: True)
    gate._last_ros_advance_steady = 100.4
    evidence = MotionEvidence()
    evidence.header.frame_id = "base_link"
    evidence.header.stamp = Time(nanoseconds=20_100_000_000).to_msg()
    evidence.interval_start = Time(nanoseconds=20_000_000_000).to_msg()
    evidence.source_epoch = "test-epoch"
    evidence.valid = evidence.imu_valid = evidence.wheel_valid = True
    evidence.quality = 0.8
    evidence.covariance[0] = evidence.covariance[4] = evidence.covariance[8] = 0.001

    evidence.scan_received_steady = receipt_steady
    evidence.scan_received_ros = Time(nanoseconds=receipt_ros_ns).to_msg()

    try:
        gate._accept_evidence(evidence)
        if accepted:
            assert gate._freshness.measured_received == receipt_steady
            assert gate._reference_admission[2] <= receipt_steady
        else:
            assert gate._freshness.measured_received is None
            assert gate._reference_admission is None
    finally:
        gate.destroy_node()
        rclpy.shutdown()


def test_bad_new_receipt_establishes_failure_frontier_without_reviving_old_frames(monkeypatch):
    rclpy = pytest.importorskip("rclpy")
    from rclpy.parameter import Parameter
    from rclpy.time import Time
    from senior_motion_msgs.msg import MotionEvidence
    from senior_robot import safety_node as module

    rclpy.init()
    gate = module.SafetyGate(
        parameter_overrides=[
            Parameter("observation_max_age", value=0.8),
            Parameter("measurement_max_age", value=0.8),
            Parameter("motion_min_quality", value=0.3),
            Parameter("motion_max_variance", value=0.1),
            Parameter("motion_max_interval", value=0.3),
        ]
    )
    monkeypatch.setattr(module, "time", SimpleNamespace(monotonic=lambda: 100.4))
    monkeypatch.setattr(
        gate,
        "get_clock",
        lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=20_500_000_000)),
    )
    monkeypatch.setattr(gate, "_clock_ok", lambda _now: True)
    gate._last_ros_advance_steady = 100.4

    def evidence(start_ns, end_ns, *, epoch="epoch-a", receipt=100.0):
        msg = MotionEvidence()
        msg.header.frame_id = "base_link"
        msg.header.stamp = Time(nanoseconds=end_ns).to_msg()
        msg.interval_start = Time(nanoseconds=start_ns).to_msg()
        msg.scan_received_ros = Time(nanoseconds=20_300_000_000).to_msg()
        msg.scan_received_steady = receipt
        msg.source_epoch = epoch
        msg.valid = msg.imu_valid = msg.wheel_valid = True
        msg.quality = 0.8
        msg.covariance[0] = msg.covariance[4] = msg.covariance[8] = 0.001
        return msg

    try:
        gate._accept_evidence(evidence(20_000_000_000, 20_100_000_000))
        assert gate._freshness.measured_received == 100.0
        gate._accept_evidence(evidence(20_100_000_000, 20_200_000_000, receipt=99.0))
        assert gate._freshness.measured_received is None
        assert gate._reference_admission is None
        # A delayed valid interval crossing the new failure must not restore it.
        gate._accept_evidence(evidence(20_100_000_000, 20_250_000_000))
        assert gate._freshness.measured_received is None
        gate._accept_evidence(evidence(20_200_000_000, 20_300_000_000))
        assert gate._freshness.measured_received == 100.0
        restored = gate._reference_admission
        # An older invalid sample or retired source cannot undo the successor.
        gate._accept_evidence(evidence(20_000_000_000, 20_100_000_000, receipt=99.0))
        assert gate._freshness.measured_received == 100.0
        assert gate._reference_admission == restored
        gate._accept_evidence(evidence(20_300_000_000, 20_400_000_000, epoch="epoch-b"))
        assert gate._epoch_fence.active == "epoch-b"
        assert gate._freshness.measured_received == 100.0
        successor = gate._reference_admission
        assert successor is not None
        gate._accept_evidence(evidence(20_300_000_000, 20_400_000_000, receipt=99.0))
        assert gate._epoch_fence.active == "epoch-b"
        assert gate._freshness.measured_received == 100.0
        assert gate._reference_admission == successor
    finally:
        gate.destroy_node()
        rclpy.shutdown()
