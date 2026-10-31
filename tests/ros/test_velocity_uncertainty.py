"""Actual Gate regressions for the interval-speed confidence qualification."""

import math
import time
import uuid

import pytest


@pytest.mark.parametrize(
    "case",
    [
        "wide_covariance",
        "unqualified_cap",
        "changed_physical_cap",
        "lateral_brake_unset",
        "qualified_far",
        "stand_qualified_far",
    ],
)
def test_real_gate_uses_velocity_uncertainty_for_coverage(monkeypatch, case):
    rclpy = pytest.importorskip("rclpy")
    from rclpy.parameter import Parameter
    from rclpy.time import Time
    from senior_motion_msgs.msg import BoundedSafetyScan, MotionEvidence
    from senior_robot.safety_node import SafetyGate

    from senior_control.models import Velocity
    from senior_control.safety.async_guard import WorkerReply, evaluate_safety_work

    class ImmediateWorker:
        def __init__(self):
            self.reply = None
            self.last_work = None

        @property
        def pending(self):
            return self.reply is not None

        def submit(self, ticket, work):
            self.last_work = work
            self.reply = WorkerReply(ticket, evaluate_safety_work(work), None)
            return True

        def poll(self):
            reply, self.reply = self.reply, None
            return reply

        def close(self):
            pass

    values = {
        "observation_max_age": 2.0,
        "measurement_max_age": 2.0,
        "motion_min_quality": 0.3,
        "motion_max_variance": 0.1,
        "motion_max_interval": 0.3,
        "max_reference_lag": 0.25,
        "max_linear_speed": 0.35 if case == "changed_physical_cap" else 0.3,
        "max_yaw_rate": 0.5,
        "command_ttl_seconds": 1.0,
        "braking_deceleration": 0.3,
        "lateral_braking_deceleration": -1.0 if case == "lateral_brake_unset" else 0.3,
        "yaw_braking_deceleration": 0.5,
        "reaction_time": 0.2,
        "hard_margin": 0.02,
        "self_mask_calibrated": True,
        "transform_static": True,
        "interpolate": True,
        "max_angular_gap": 0.01,
        "footprint_xy": [-0.337, -0.276, 0.337, -0.276, 0.337, 0.276, -0.337, 0.276],
        "physical_cap_context": (
            ""
            if case == "unqualified_cap"
            else "fortress-fixed-stand-v1"
            if case == "stand_qualified_far"
            else "fortress-flat-v1"
        ),
    }
    rclpy.init()
    gate = SafetyGate(parameter_overrides=[Parameter(k, value=v) for k, v in values.items()])
    gate._worker.close()
    gate._worker = ImmediateWorker()
    outputs = []
    monkeypatch.setattr(
        gate, "_publish", lambda velocity, reason: outputs.append((velocity, reason))
    )
    monkeypatch.setattr(gate, "_topology_ready", lambda: True)
    monkeypatch.setattr(gate, "_command_topology_ready", lambda: True)
    monkeypatch.setattr(gate, "_scan_topology_ready", lambda: True)
    try:
        stamp_ns = gate.get_clock().now().nanoseconds - 20_000_000
        evidence = MotionEvidence()
        evidence.header.stamp = Time(nanoseconds=stamp_ns).to_msg()
        evidence.header.frame_id = "base_link"
        evidence.interval_start = Time(nanoseconds=stamp_ns - 100_000_000).to_msg()
        evidence.scan_received_ros = gate.get_clock().now().to_msg()
        evidence.scan_received_steady = time.monotonic()
        evidence.source_epoch = "independent-evidence"
        evidence.valid = evidence.imu_valid = evidence.wheel_valid = True
        evidence.quality = 0.8
        evidence.covariance[0] = evidence.covariance[4] = 0.01
        evidence.covariance[8] = 0.00005
        gate._accept_evidence(evidence)
        assert gate._freshness.measured_received is not None

        scan = BoundedSafetyScan()
        scan.stamp = evidence.header.stamp
        scan.frame_id = "laser_link"
        scan.source_epoch = uuid.uuid4().hex
        scan.raw_source = "raw-source"
        scan.sequence = 1
        scan.ingress_steady = time.monotonic()
        scan.ingress_ros = gate.get_clock().now().nanoseconds / 1e9
        scan.valid = True
        scan.angle_min = -math.pi
        scan.angle_increment = 2 * math.pi / 720
        scan.range_min, scan.range_max = 0.08, 8.0
        scan.sensor_x, scan.sensor_y = 0.15699, 0.00201
        side_wall = 4.0 if case in {"qualified_far", "stand_qualified_far"} else 0.676
        scan.ranges = [
            (side_wall - scan.sensor_y) / math.sin(angle)
            if abs(angle - math.pi / 2) < 0.25
            else 5.0
            for angle in (-math.pi + index * 2 * math.pi / 720 for index in range(720))
        ]
        gate._on_scan(scan)
        assert gate._freshness.scan_received is not None
        gate._command = Velocity(0.05, 0)
        gate._command_monotonic = time.monotonic()
        gate._on_timer()
        gate._on_timer()
        diagnostics = (
            f"outputs={outputs!r} worker={gate._worker_last_result!r} "
            f"finished={gate._worker_finished} discarded={gate._worker_discarded} "
            f"ros_age={gate.get_clock().now().nanoseconds / 1e9 - stamp_ns / 1e9:.6f} "
            f"source_ns={stamp_ns} scan_ns={scan.stamp.sec * 10**9 + scan.stamp.nanosec} "
            f"scan_float={gate._scans[-1].stamp!r} "
            f"history_stamps={[sample.stamp for sample in gate._history._samples]!r} "
            f"work_ros={getattr(gate._worker.last_work, 'evaluation_ros_time', None)!r}"
        )
        assert outputs[-1][0] == (
            Velocity(0.05, 0)
            if case in {"qualified_far", "stand_qualified_far"}
            else Velocity(0, 0)
        ), diagnostics
        expected = {
            "unqualified_cap": {"physical_motion_cap_unqualified"},
            "changed_physical_cap": {"physical_motion_profile_unqualified"},
            "lateral_brake_unset": {"lateral_braking_unqualified"},
            "wide_covariance": {"hard_contact", "hard_sweep_collision"},
            "qualified_far": {"hard_sweep_clear", "warning_overlap"},
            "stand_qualified_far": {"hard_sweep_clear", "warning_overlap"},
        }
        assert outputs[-1][1] in expected[case], diagnostics
        if case in {"wide_covariance", "qualified_far", "stand_qualified_far"}:
            assert gate._status_geometry["interval_linear_upper"] > 6.0
            assert gate._status_geometry["assumed_instantaneous_linear_cap"] == 0.3
            assert gate._status_geometry["motion_uncertainty_sweep_margin"] > 0.42
    finally:
        gate.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize("kind", ["rank_one_psd", "indefinite"])
def test_gate_qualifies_psd_covariance_and_rejects_indefinite(monkeypatch, kind):
    rclpy = pytest.importorskip("rclpy")
    from rclpy.parameter import Parameter
    from rclpy.time import Time
    from senior_motion_msgs.msg import MotionEvidence
    from senior_robot.safety_node import SafetyGate

    rclpy.init()
    gate = SafetyGate(
        parameter_overrides=[
            Parameter("observation_max_age", value=1.0),
            Parameter("measurement_max_age", value=1.0),
            Parameter("motion_min_quality", value=0.3),
            Parameter("motion_max_variance", value=0.1),
            Parameter("motion_max_interval", value=0.3),
        ]
    )
    try:
        stamp_ns = gate.get_clock().now().nanoseconds - 20_000_000
        evidence = MotionEvidence()
        evidence.header.stamp = Time(nanoseconds=stamp_ns).to_msg()
        evidence.header.frame_id = "base_link"
        evidence.interval_start = Time(nanoseconds=stamp_ns - 100_000_000).to_msg()
        evidence.scan_received_ros = gate.get_clock().now().to_msg()
        evidence.scan_received_steady = time.monotonic()
        evidence.source_epoch = "independent-evidence"
        evidence.valid = evidence.imu_valid = evidence.wheel_valid = True
        evidence.quality = 0.8
        evidence.covariance[0] = evidence.covariance[4] = 0.01
        evidence.covariance[1] = evidence.covariance[3] = 0.01 if kind == "rank_one_psd" else 0.02
        evidence.covariance[8] = 0.00005
        gate._accept_evidence(evidence)
        assert (gate._freshness.measured_received is not None) == (kind == "rank_one_psd")
    finally:
        gate.destroy_node()
        rclpy.shutdown()
