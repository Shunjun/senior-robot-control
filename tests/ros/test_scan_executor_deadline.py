"""Real ROS executor response when a raw scan overwhelms its input path."""

import importlib
import math
import threading
import time

import pytest


def test_oversized_mask_is_rejected_before_runtime_geometry():
    rclpy = pytest.importorskip("rclpy")
    from rclpy.parameter import Parameter
    from senior_robot.safety_node import SafetyGate

    vertices = 256
    mask = [
        coordinate
        for index in range(vertices)
        for coordinate in (
            math.cos(2 * math.pi * index / vertices),
            math.sin(2 * math.pi * index / vertices),
        )
    ]
    rclpy.init()
    gate = None
    try:
        with pytest.raises(ValueError, match="(?i)mask|complex"):
            gate = SafetyGate(
                parameter_overrides=[
                    Parameter("self_mask_calibrated", value=True),
                    Parameter("self_mask_xy", value=mask),
                ]
            )
            gate._profile()
    finally:
        if gate is not None:
            gate.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize("event", ["stop", "expiry"])
@pytest.mark.parametrize("input_case", ["oversized", "blocked_ingress"])
def test_nonzero_gate_returns_zero_during_blocked_or_oversized_scan(
    monkeypatch, event, input_case, native_scan_process
):
    rclpy = pytest.importorskip("rclpy")
    from geometry_msgs.msg import TransformStamped, Twist
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from senior_motion_msgs.msg import MotionEvidence
    from senior_robot import safety_node as module
    from senior_robot.command_node import CommandNode
    from sensor_msgs.msg import LaserScan

    from senior_control.protocol import (
        EmptyPayload,
        SimpleEnvelope,
        VelocityEnvelope,
        VelocityPayload,
    )
    from senior_control.safety.arbiter import Arbiter

    gate_values = {
        "depth_policy": "laser_only",
        "observation_max_age": 3.0,
        "measurement_max_age": 3.0,
        "motion_min_quality": 0.3,
        "motion_max_variance": 0.1,
        "motion_max_interval": 1.0,
        "max_reference_lag": 1.0,
        "max_linear_speed": 0.3,
        "max_yaw_rate": 0.5,
        "command_ttl_seconds": 0.3,
        "braking_deceleration": 0.3,
        "lateral_braking_deceleration": 0.3,
        "yaw_braking_deceleration": 0.5,
        "physical_cap_context": "fortress-flat-v1",
        "reaction_time": 0.2,
        "hard_margin": 0.02,
        "self_mask_calibrated": True,
        "transform_static": True,
        "interpolate": True,
        "max_angular_gap": 0.01,
        "footprint_xy": [-0.337, -0.276, 0.337, -0.276, 0.337, 0.276, -0.337, 0.276],
        # Real self-filter work, far outside the measured free-space radius.
        "self_mask_xy": [10.0, 10.0, 11.0, 10.0, 11.0, 11.0, 10.0, 11.0],
    }
    rclpy.init()
    gate = module.SafetyGate(
        parameter_overrides=[Parameter(k, value=v) for k, v in gate_values.items()]
    )
    command = CommandNode(arbiter=Arbiter("robot", 0.1, 10.0, 0.3))
    motion = rclpy.create_node("motion_fusion")
    evidence_publisher = motion.create_publisher(MotionEvidence, "/motion/evidence", 10)
    source = rclpy.create_node("raw_scan_test_source")
    scan_publisher = source.create_publisher(LaserScan, "/scan", qos_profile_sensor_data)
    listener = rclpy.create_node("scan_deadline_listener")
    outputs = []
    listener.create_subscription(
        Twist, "/cmd_vel", lambda m: outputs.append((time.monotonic(), m)), 10
    )
    executor = SingleThreadedExecutor()
    executor.add_node(gate)
    thread = threading.Thread(target=executor.spin, daemon=True)
    ingress = ingress_executor = ingress_thread = None
    release_ingress = threading.Event()
    try:
        try:
            ingress_module = importlib.import_module("senior_robot.safety_scan_ingress")
        except ModuleNotFoundError as exc:
            if exc.name != "senior_robot.safety_scan_ingress":
                raise
        else:
            ingress = ingress_module.SafetyScanIngress(
                parameter_overrides=[Parameter(k, value=v) for k, v in gate_values.items()]
            )
            ingress_executor = SingleThreadedExecutor()
            ingress_executor.add_node(ingress)
            ingress_thread = threading.Thread(target=ingress_executor.spin, daemon=True)
            ingress_thread.start()

        transform = TransformStamped()
        transform.header.frame_id = "base_link"
        transform.child_frame_id = "laser_link"
        transform.transform.rotation.w = 1.0
        for node in (gate, ingress):
            buffer = getattr(node, "_tf_buffer", None)
            if buffer is not None:
                buffer.set_transform_static(transform, "test")
        entered = threading.Event()
        original_filter = getattr(module, "filter_self", None)
        if original_filter is not None:

            def observed_filter(scan, *args, **kwargs):
                if len(scan.ranges) >= 40_000:
                    entered.set()
                return original_filter(scan, *args, **kwargs)

            monkeypatch.setattr(module, "filter_self", observed_filter)

        sequence = 0

        def submit(kind, vx=0.0):
            nonlocal sequence
            sequence += 1
            common = dict(
                v=1,
                robot_id="robot",
                session_id="test",
                connection_epoch=1,
                request_id=f"request-{sequence}",
                seq=sequence,
                expires_at=1.0,
            )
            if kind == "drive.velocity":
                envelope = VelocityEnvelope(
                    type=kind, payload=VelocityPayload(vx=vx, wz=0), **common
                )
            else:
                envelope = SimpleEnvelope(type=kind, payload=EmptyPayload(), **common)
            assert command.submit(envelope).accepted

        last_evidence_stamp = None

        def publish_sensors():
            nonlocal last_evidence_stamp
            stamp_ns = motion.get_clock().now().nanoseconds
            evidence = MotionEvidence()
            evidence.header.frame_id = "base_link"
            evidence.header.stamp = Time(nanoseconds=stamp_ns).to_msg()
            start_ns = (
                stamp_ns - 100_000_000 if last_evidence_stamp is None else last_evidence_stamp
            )
            evidence.interval_start = Time(nanoseconds=start_ns).to_msg()
            last_evidence_stamp = stamp_ns
            evidence.scan_received_ros = motion.get_clock().now().to_msg()
            evidence.scan_received_steady = time.monotonic()
            evidence.source_epoch = "scan-deadline-test"
            evidence.valid = evidence.imu_valid = evidence.wheel_valid = True
            evidence.quality = 0.8
            evidence.covariance[0] = evidence.covariance[4] = evidence.covariance[8] = 1e-8
            evidence_publisher.publish(evidence)
            scan = LaserScan()
            scan.header.frame_id = "laser_link"
            scan.header.stamp = evidence.header.stamp
            scan.angle_min = -math.pi
            scan.angle_increment = 2 * math.pi / 720
            scan.angle_max = math.pi
            scan.range_min, scan.range_max = 0.08, 8.0
            scan.ranges = [5.0] * 720
            scan_publisher.publish(scan)

        thread.start()
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline and (
            len(gate.get_publishers_info_by_topic("/motion/evidence")) != 1
            or len(gate.get_publishers_info_by_topic("/safety/command_in")) != 1
            or scan_publisher.get_subscription_count() < 1
        ):
            rclpy.spin_once(listener, timeout_sec=0.02)
        assert gate._topology_ready() and gate._command_topology_ready()
        submit("control.acquire")
        sensor_at = command_at = 0.0
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now - sensor_at >= 0.7:
                publish_sensors()
                sensor_at = now
            if now - command_at >= 0.07:
                submit("drive.velocity", 0.05)
                command_at = now
            rclpy.spin_once(listener, timeout_sec=0.01)
            if outputs and outputs[-1][1].linear.x > 0.04:
                break
        else:
            pytest.fail(
                "real SafetyGate did not first publish qualified nonzero output; "
                f"reasons={gate._status_counts}, worker={gate._worker_last_result}, "
                f"finished={gate._worker_finished}, discarded={gate._worker_discarded}"
            )
        assert gate._freshness.scan_received is not None
        assert gate._freshness.measured_received is not None
        submit("drive.velocity", 0.05)
        last_request = time.monotonic()
        if input_case == "blocked_ingress":
            assert ingress is not None
            original_check = ingress._check_source

            def blocked_source_check():
                entered.set()
                release_ingress.wait(timeout=3)
                return original_check()

            monkeypatch.setattr(ingress, "_check_source", blocked_source_check)
            blocked_scan = LaserScan()
            blocked_scan.header.frame_id = "laser_link"
            blocked_scan.header.stamp = source.get_clock().now().to_msg()
            blocked_scan.angle_min = -math.pi
            blocked_scan.angle_increment = 2 * math.pi / 720
            blocked_scan.range_min, blocked_scan.range_max = 0.08, 8.0
            blocked_scan.ranges = [5.0] * 720
            scan_publisher.publish(blocked_scan)
            assert entered.wait(timeout=2), "the ingress executor did not enter the test block"
            assert gate._freshness.scan_received is not None
        else:
            large = LaserScan()
            large.header.frame_id = "laser_link"
            large.header.stamp = source.get_clock().now().to_msg()
            large.angle_min, large.angle_increment = -math.pi, 2 * math.pi / 40_000
            large.range_min, large.range_max = 0.08, 8.0
            large.ranges = [2.0] * 40_000
            scan_publisher.publish(large)
            if ingress is None:
                assert entered.wait(timeout=2), "oversized scan did not enter the actual filter"
            else:
                deadline = time.monotonic() + 0.25
                while gate._freshness.scan_received is not None and time.monotonic() < deadline:
                    rclpy.spin_once(listener, timeout_sec=0.005)
                assert gate._freshness.scan_received is None, "oversized scan did not invalidate"
        triggered = time.monotonic()
        if event == "stop":
            submit("drive.stop")
        timeout = 0.25 if event == "stop" else max(0.0, last_request + 0.45 - triggered)
        deadline = triggered + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(listener, timeout_sec=0.005)
            zeros = [
                t for t, m in outputs if t >= triggered and m.linear.x == 0.0 and m.angular.z == 0.0
            ]
            if len(zeros) >= 2 and zeros[-1] - zeros[0] >= 0.04:
                break
        else:
            pytest.fail(f"steady SafetyGate output did not reach zero after {event} during scan")
    finally:
        release_ingress.set()
        executor.shutdown(timeout_sec=3)
        thread.join(timeout=3)
        if ingress_executor is not None:
            ingress_executor.shutdown(timeout_sec=3)
        if ingress_thread is not None:
            ingress_thread.join(timeout=3)
        if ingress is not None:
            ingress.destroy_node()
        listener.destroy_node()
        source.destroy_node()
        motion.destroy_node()
        command.destroy_node()
        gate.destroy_node()
        rclpy.shutdown()
