"""ROS boundary regressions for the independent-motion safety review."""

import json
import time
from types import SimpleNamespace

import pytest
from motion_progress_test_helpers import prime_stationary_progress


def test_fusion_logs_first_latch_after_publish_with_exact_source_interval(monkeypatch):
    rclpy = pytest.importorskip("rclpy")
    from geometry_msgs.msg import Twist
    from rclpy.parameter import Parameter
    from rclpy.time import Time
    from senior_motion_msgs.msg import ScanMotion
    from senior_robot.motion_fusion import MotionFusion

    from senior_control.models import Pose2D, Velocity
    from senior_control.safety.motion_runtime import ImuSample, TimedPoseSample

    values = {
        "sensor_max_age": 2.0,
        "alignment_max_gap": 0.3,
        "pending_wait": 0.1,
        "max_variance": 0.1,
        "min_quality": 0.3,
        "max_yaw_error": 0.1,
        "startup_window": 0.0,
        "wheel_motion_limit": 0.004,
        "stall_distance": 0.04,
        "lateral_limit": 0.04,
        "yaw_limit": 0.12,
        "progress_stationary_max_linear_speed": 0.001,
        "progress_stationary_max_yaw_rate": 0.001,
    }
    rclpy.init()
    fusion = MotionFusion(parameter_overrides=[Parameter(k, value=v) for k, v in values.items()])
    emitted, logs = [], []
    clock = {"now": 100_010_000_000}
    monkeypatch.setattr(fusion, "_scan_topology_ready", lambda: True)
    monkeypatch.setattr(
        fusion,
        "get_clock",
        lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=clock["now"])),
    )
    monkeypatch.setattr(fusion, "_publisher", SimpleNamespace(publish=emitted.append))
    monkeypatch.setattr(
        fusion,
        "get_logger",
        lambda: SimpleNamespace(warning=lambda s: logs.append((len(emitted), s))),
    )
    try:
        base_ns = prime_stationary_progress(fusion, clock, 100_000_000_000)
        emitted.clear()
        logs.clear()
        clock["now"] = base_ns + 1
        command = Twist()
        command.linear.x = 0.5
        fusion._on_requested(command)
        fusion._on_command(command)
        times = [base_ns + 100_000_000 * index for index in range(3)]
        for index, stamp_ns in enumerate(times[1:], start=1):
            stamp = stamp_ns / 1e9
            assert fusion._wheel.add(
                TimedPoseSample(stamp, Pose2D(index * 0.05, 0, 0), (0.001,) * 3, "odom")
            )
            assert fusion._imu.add(ImuSample(stamp, 0.0, 0.001))
            assert fusion._commands.add(stamp, Velocity(0.5, 0.0))
        fusion._wheel_received = fusion._imu_received = time.monotonic()
        for start_ns, end_ns in zip(times, times[1:], strict=False):
            clock["now"] = end_ns + 10_000_000
            scan = ScanMotion()
            scan.header.frame_id = "base_link"
            scan.header.stamp = Time(nanoseconds=end_ns).to_msg()
            scan.interval_start = Time(nanoseconds=start_ns).to_msg()
            scan.source_epoch = "stable-independent-scan"
            scan.valid = True
            scan.response = 0.8
            scan.valid_rays = 720
            scan.covariance[0] = scan.covariance[4] = scan.covariance[8] = 0.001
            fusion._on_scan(scan)
        assert len(emitted) == 2 and all(message.motion_anomaly for message in emitted)
        assert len(logs) == 1 and logs[0][0] == 1  # stop evidence precedes logging
        prefix = "motion_first_latch "
        assert logs[0][1].startswith(prefix)
        snapshot = json.loads(logs[0][1][len(prefix) :])
        assert snapshot["source_epoch"] == "stable-independent-scan"
        assert snapshot["start_ns"] == times[0] and snapshot["end_ns"] == times[1]
        assert snapshot["previous_accepted"]["end_ns"] == times[0]
        assert snapshot["previous_accepted"]["source_epoch"] == "stable-independent-scan"
        assert snapshot["gap_since_previous_ns"] == 0
        assert any(
            item["status"] == "progress_processed" and item["end_ns"] == times[1]
            for item in snapshot["scan_events"]
        )
        assert set(snapshot["latch"]["causes"]) == {"wheel_stall", "command_stall"}
        assert snapshot["latch"]["wheel"]["pose_delta"]["x"] == pytest.approx(0.05)
        assert snapshot["latch"]["actual"]["pose_delta"]["x"] == pytest.approx(0.0)
        assert snapshot["recent_intervals"][-1]["command"]["vx"] == pytest.approx(0.5)
    finally:
        fusion.destroy_node()
        rclpy.shutdown()


def test_real_fusion_and_gate_retain_independent_lateral_twist(monkeypatch, fenced_scan_fixture):
    rclpy = pytest.importorskip("rclpy")
    from geometry_msgs.msg import Twist
    from rclpy.parameter import Parameter
    from rclpy.time import Time
    from senior_motion_msgs.msg import BoundedSafetyScan, ScanMotion
    from senior_robot.motion_fusion import MotionFusion
    from senior_robot.safety_node import SafetyGate

    from senior_control.models import Pose2D, Velocity
    from senior_control.safety.motion_runtime import ImuSample, TimedPoseSample

    values = {
        "sensor_max_age": 2.0,
        "alignment_max_gap": 0.3,
        "pending_wait": 0.1,
        "max_variance": 0.1,
        "min_quality": 0.3,
        "max_yaw_error": 0.1,
        "startup_window": 0.6,
        "wheel_motion_limit": 0.004,
        "stall_distance": 0.04,
        "lateral_limit": 0.04,
        "yaw_limit": 0.12,
        "progress_stationary_max_linear_speed": 0.001,
        "progress_stationary_max_yaw_rate": 0.001,
    }
    rclpy.init()
    fusion = MotionFusion(parameter_overrides=[Parameter(k, value=v) for k, v in values.items()])
    gate = SafetyGate(
        parameter_overrides=[
            Parameter("depth_policy", value="laser_only"),
            Parameter("observation_max_age", value=2.0),
            Parameter("measurement_max_age", value=2.0),
            Parameter("motion_min_quality", value=0.3),
            Parameter("motion_max_variance", value=0.1),
            Parameter("motion_max_interval", value=0.3),
            Parameter("braking_deceleration", value=0.3),
            Parameter("lateral_braking_deceleration", value=0.3),
            Parameter("yaw_braking_deceleration", value=0.5),
            Parameter("physical_cap_context", value="fortress-flat-v1"),
            Parameter("reaction_time", value=0.2),
            Parameter("hard_margin", value=0.02),
            Parameter("max_reference_lag", value=0.4),
            Parameter("max_linear_speed", value=0.3),
            Parameter("max_yaw_rate", value=0.5),
            Parameter("command_ttl_seconds", value=1.0),
            Parameter(
                "footprint_xy",
                value=[-0.337, -0.276, 0.337, -0.276, 0.337, 0.276, -0.337, 0.276],
            ),
        ]
    )
    emitted = []
    monkeypatch.setattr(fusion, "_scan_topology_ready", lambda: True)
    fusion._publisher = SimpleNamespace(publish=emitted.append)
    try:
        base_ns = fusion.get_clock().now().nanoseconds - 800_000_000
        clock = {"now": base_ns}
        monkeypatch.setattr(
            fusion,
            "get_clock",
            lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=clock["now"])),
        )
        start_ns = prime_stationary_progress(fusion, clock, base_ns)
        emitted.clear()
        clock["now"] = start_ns + 1
        command = Twist()
        command.linear.x = 0.05
        fusion._on_requested(command)
        fusion._on_command(command)
        end_ns = start_ns + 100_000_000
        end = end_ns / 1e9
        assert fusion._wheel.add(TimedPoseSample(end, Pose2D(0.005, 0, 0), (0.001,) * 3, "odom"))
        assert fusion._imu.add(ImuSample(end, 0.0, 0.001))
        assert fusion._commands.add(end, Velocity(0.05, 0.0))
        fusion._wheel_received = fusion._imu_received = time.monotonic()
        clock["now"] = end_ns + 10_000_000
        scan = ScanMotion()
        scan.header.frame_id = "base_link"
        scan.header.stamp = Time(nanoseconds=end_ns).to_msg()
        scan.interval_start = Time(nanoseconds=start_ns).to_msg()
        scan.source_epoch = "stable-independent-scan"
        scan.valid = True
        scan.response = 0.8
        scan.valid_rays = 720
        scan.body_delta.x = scan.local_pose.x = 0.005
        scan.body_delta.y = scan.local_pose.y = 0.039
        scan.covariance[0] = scan.covariance[4] = scan.covariance[8] = 0.001
        fusion._on_scan(scan)
        assert emitted and emitted[-1].valid and not emitted[-1].motion_anomaly
        evidence = emitted[-1]
        # Direct callback is a test-owned source seam; Fusion computes actual aligned evidence.
        assert evidence.body_velocity.linear.x == pytest.approx(0.05, rel=1e-5)
        assert evidence.body_velocity.linear.y == pytest.approx(0.39, rel=1e-5)
        gate._accept_evidence(evidence)
        assert gate._freshness.measured_received is not None
        assert gate._measured.vy == pytest.approx(0.39, rel=1e-5)

        # A supplied bounded scan reaches the real Gate callback. The full
        # translational-speed limit must stop before geometry is dispatched.
        monkeypatch.setattr(gate, "_scan_topology_ready", lambda: True)
        monkeypatch.setattr(gate, "_topology_ready", lambda: True)
        monkeypatch.setattr(gate, "_command_topology_ready", lambda: True)
        bounded = BoundedSafetyScan()
        bounded.stamp = scan.header.stamp
        bounded.frame_id = "laser_link"
        bounded.source_epoch = "test-ingress"
        bounded.raw_source = "test-raw-source"
        bounded.sequence = 1
        bounded.ingress_steady = time.monotonic()
        bounded.ingress_ros = gate.get_clock().now().nanoseconds / 1e9
        bounded.valid = True
        bounded.ranges = [5.0]
        gate._on_scan(fenced_scan_fixture(gate, bounded))
        assert gate._freshness.scan_received is not None
        gate._command = Velocity(0.05, 0)
        gate._command_monotonic = time.monotonic()
        decisions = []
        monkeypatch.setattr(
            gate, "_publish", lambda velocity, reason: decisions.append((velocity, reason))
        )
        gate._on_timer()
        assert decisions[-1] == (Velocity(0, 0), "measured_translational_speed_exceeded")

        # A producer cannot hide the lateral component in its advertised twist.
        evidence.header.stamp = gate.get_clock().now().to_msg()
        evidence.interval_start = scan.header.stamp
        evidence.body_delta.x = evidence.body_delta.y = evidence.body_delta.theta = 0.0
        evidence.body_velocity.linear.x = 0.0
        evidence.body_velocity.linear.y = 0.39
        gate._accept_evidence(evidence)
        assert gate._freshness.measured_received is None
    finally:
        gate.destroy_node()
        fusion.destroy_node()
        rclpy.shutdown()


def test_motion_safety_wire_payload_is_bounded():
    pytest.importorskip("rclpy")
    from senior_motion_msgs.msg import BoundedSafetyScan, MotionEvidence

    scan = BoundedSafetyScan()
    with pytest.raises(AssertionError):
        scan.ranges = [1.0] * 2049
    with pytest.raises(AssertionError):
        scan.frame_id = "f" * 129
    evidence = MotionEvidence()
    with pytest.raises(AssertionError):
        evidence.source_epoch = "s" * 129
    with pytest.raises(AssertionError):
        evidence.reason = "r" * 65


@pytest.mark.parametrize("field", ["frame", "epoch"])
def test_fusion_rejects_upstream_oversized_identity_without_truncation(monkeypatch, field):
    rclpy = pytest.importorskip("rclpy")
    from rclpy.parameter import Parameter
    from senior_motion_msgs.msg import ScanMotion
    from senior_robot.motion_fusion import MotionFusion

    rclpy.init()
    values = {
        "sensor_max_age": 2.0,
        "alignment_max_gap": 0.3,
        "pending_wait": 0.1,
        "max_variance": 0.1,
        "min_quality": 0.3,
        "max_yaw_error": 0.1,
        "startup_window": 0.6,
        "wheel_motion_limit": 0.004,
        "lateral_limit": 0.04,
        "yaw_limit": 0.12,
    }
    fusion = MotionFusion(parameter_overrides=[Parameter(k, value=v) for k, v in values.items()])
    emitted = []
    monkeypatch.setattr(fusion, "_publisher", SimpleNamespace(publish=emitted.append))
    try:
        scan = ScanMotion()
        scan.header.stamp = fusion.get_clock().now().to_msg()
        scan.header.frame_id = "f" * 129 if field == "frame" else "base_link"
        scan.source_epoch = "e" * 96 if field == "epoch" else "source-a"
        fusion._on_scan(scan)
        assert len(emitted) == 1 and not emitted[0].valid
        assert emitted[0].reason == "evidence_field_overflow"
        assert emitted[0].header.stamp == scan.header.stamp
        if field == "frame":
            assert emitted[0].header.frame_id == ""
            assert emitted[0].source_epoch.endswith(":source-a")
        else:
            assert emitted[0].source_epoch == ""
            assert emitted[0].header.frame_id == "base_link"
    finally:
        fusion.destroy_node()
        rclpy.shutdown()
