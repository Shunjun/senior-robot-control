"""Exercise real ROS graph discovery; these tests require Humble and an isolated domain."""

import time

import pytest


def test_duplicate_and_replaced_producers_cannot_restore_old_evidence():
    rclpy = pytest.importorskip("rclpy")
    from rclpy.parameter import Parameter
    from rclpy.time import Time
    from senior_motion_msgs.msg import MotionEvidence
    from senior_robot.safety_node import SafetyGate

    rclpy.init()
    producer = rclpy.create_node("motion_fusion")
    publisher = producer.create_publisher(MotionEvidence, "/motion/evidence", 10)
    gate = SafetyGate(
        parameter_overrides=[
            Parameter("observation_max_age", value=0.5),
            Parameter("measurement_max_age", value=0.5),
            Parameter("motion_min_quality", value=0.3),
            Parameter("motion_max_variance", value=0.1),
            Parameter("motion_max_interval", value=0.3),
        ]
    )
    intruder = None

    def spin_until(predicate):
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            rclpy.spin_once(gate, timeout_sec=0.01)
            if predicate():
                return
        pytest.fail("ROS graph or evidence did not reach expected state")

    def publish(epoch):
        msg = MotionEvidence()
        stamp = producer.get_clock().now().nanoseconds
        msg.header.frame_id = "base_link"
        msg.header.stamp = Time(nanoseconds=stamp).to_msg()
        msg.interval_start = Time(nanoseconds=stamp - 100_000_000).to_msg()
        msg.scan_received_ros = producer.get_clock().now().to_msg()
        msg.scan_received_steady = time.monotonic()
        msg.source_epoch = epoch
        msg.valid = msg.imu_valid = msg.wheel_valid = True
        msg.quality = 0.8
        msg.covariance[0] = msg.covariance[4] = msg.covariance[8] = 0.001
        publisher.publish(msg)

    try:
        spin_until(gate._topology_ready)
        publish("epoch-a")
        spin_until(lambda: gate._freshness.measured_received is not None)
        intruder = rclpy.create_node("unexpected_motion_producer")
        other = intruder.create_publisher(MotionEvidence, "/motion/evidence", 10)
        spin_until(lambda: not gate._topology_ready())
        assert gate._freshness.measured_received is None
        assert other is not None
        intruder.destroy_node()
        intruder = None
        spin_until(gate._topology_ready)
        publish("epoch-a")
        for _ in range(10):
            rclpy.spin_once(gate, timeout_sec=0.02)
        assert gate._freshness.measured_received is None
        publish("epoch-b")
        spin_until(lambda: gate._freshness.measured_received is not None)
    finally:
        if intruder is not None:
            intruder.destroy_node()
        gate.destroy_node()
        producer.destroy_node()
        rclpy.shutdown()


def test_safety_rejects_duplicate_command_producer_and_recovers_with_arbiter():
    rclpy = pytest.importorskip("rclpy")
    from senior_motion_msgs.msg import FinalCommand
    from senior_robot.command_node import CommandNode
    from senior_robot.safety_node import SafetyGate

    from senior_control.protocol import (
        EmptyPayload,
        SimpleEnvelope,
        VelocityEnvelope,
        VelocityPayload,
    )
    from senior_control.safety.arbiter import Arbiter

    rclpy.init()

    def new_command_owner():
        return CommandNode(arbiter=Arbiter("robot", 0.2, 30.0, 1.0))

    command = new_command_owner()
    final_topic = "/safety/final_command"
    gate = SafetyGate()
    duplicate = None

    def spin_until(predicate):
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            if command is not None:
                rclpy.spin_once(command, timeout_sec=0.0)
            rclpy.spin_once(gate, timeout_sec=0.02)
            if predicate():
                return
        endpoints = gate.get_publishers_info_by_topic(final_topic)
        pytest.fail(
            "ROS command producer graph or callback did not converge "
            f"(endpoints={[(e.node_name, bytes(e.endpoint_gid).hex()) for e in endpoints]}, "
            f"guard_ready={gate._command_topology_ready()}, final={gate._final_identity})"
        )

    def submit(kind, seq, vx=0.0):
        common = dict(
            v=1,
            robot_id="robot",
            session_id="lease",
            connection_epoch=1,
            request_id=f"request-{seq}",
            seq=seq,
            expires_at=1.0,
        )
        if kind == "drive.velocity":
            envelope = VelocityEnvelope(type=kind, payload=VelocityPayload(vx=vx, wz=0.0), **common)
        else:
            envelope = SimpleEnvelope(type=kind, payload=EmptyPayload(), **common)
        assert command.submit(envelope).accepted

    try:
        spin_until(lambda: gate._command_topology_ready() and gate._final_identity is not None)
        submit("control.acquire", 1)
        submit("drive.velocity", 2, 0.05)
        spin_until(lambda: gate._command.vx == 0.05)
        duplicate = rclpy.create_node("unexpected_command_producer")
        publisher = duplicate.create_publisher(FinalCommand, final_topic, 10)
        spin_until(
            lambda: (
                len(gate.get_publishers_info_by_topic(final_topic)) == 2
                and not gate._command_topology_ready()
            )
        )
        assert gate._command_monotonic is None and gate._command.vx == 0.0
        injected = FinalCommand(
            robot_id="robot",
            owner_boot_id=command._owner_boot,
            generation=command._generation,
            session_id="lease",
            connection_epoch=1,
            sequence=3,
            steady_deadline=time.monotonic() + 0.3,
            vx=0.2,
        )
        publisher.publish(injected)
        for _ in range(10):
            rclpy.spin_once(gate, timeout_sec=0.02)
        assert gate._command_monotonic is None and gate._command.vx == 0.0
        duplicate.destroy_node()
        duplicate = None
        spin_until(lambda: gate._command_topology_ready())
        submit("drive.velocity", 3, 0.07)
        for _ in range(10):
            rclpy.spin_once(gate, timeout_sec=0.02)
        assert gate._command.vx == 0.0

        command.destroy_node()
        command = None
        spin_until(
            lambda: (
                not gate.get_publishers_info_by_topic(final_topic)
                and not gate._command_topology_ready()
                and gate._final_identity is None
            )
        )
        command = new_command_owner()
        spin_until(
            lambda: (
                gate._command_topology_ready()
                and gate._final_identity is not None
                and gate._final_identity[0] == command._owner_boot
            )
        )
        submit("control.acquire", 1)
        submit("drive.velocity", 2, 0.07)
        spin_until(lambda: gate._command.vx == 0.07)
    finally:
        if duplicate is not None:
            duplicate.destroy_node()
        if command is not None:
            command.destroy_node()
        gate.destroy_node()
        rclpy.shutdown()


def test_new_invalid_epoch_retires_old_valid_and_invalid_evidence():
    rclpy = pytest.importorskip("rclpy")
    from rclpy.parameter import Parameter
    from rclpy.time import Time
    from senior_motion_msgs.msg import MotionEvidence
    from senior_robot.safety_node import SafetyGate

    rclpy.init()
    producer = rclpy.create_node("motion_fusion")
    publisher = producer.create_publisher(MotionEvidence, "/motion/evidence", 10)
    gate = SafetyGate(
        parameter_overrides=[
            Parameter("observation_max_age", value=0.5),
            Parameter("measurement_max_age", value=0.5),
            Parameter("motion_min_quality", value=0.3),
            Parameter("motion_max_variance", value=0.1),
            Parameter("motion_max_interval", value=0.3),
        ]
    )

    def spin_until(predicate):
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            rclpy.spin_once(gate, timeout_sec=0.02)
            if predicate():
                return
        pytest.fail("epoch evidence did not reach expected state")

    def publish(epoch, valid):
        message = MotionEvidence()
        stamp = producer.get_clock().now().nanoseconds
        message.header.frame_id = "base_link"
        message.header.stamp = Time(nanoseconds=stamp).to_msg()
        message.interval_start = Time(nanoseconds=stamp - 100_000_000).to_msg()
        message.scan_received_ros = producer.get_clock().now().to_msg()
        message.scan_received_steady = time.monotonic()
        message.source_epoch = epoch
        message.valid = message.imu_valid = message.wheel_valid = valid
        message.startup_reference_valid = True  # Old claims cannot restore a retired epoch.
        message.quality = 0.8 if valid else 0.0
        message.covariance[0] = message.covariance[4] = message.covariance[8] = 0.001
        publisher.publish(message)

    try:
        spin_until(gate._topology_ready)
        publish("epoch-a", True)
        spin_until(lambda: gate._freshness.measured_received is not None)
        publish("epoch-b", False)
        spin_until(lambda: gate._freshness.measured_received is None)
        assert gate._epoch_fence.active == "epoch-b"
        assert gate._history.latest is None and gate._reference_admission is None
        for valid in (True, False):
            publish("epoch-a", valid)
            for _ in range(8):
                rclpy.spin_once(gate, timeout_sec=0.02)
            assert gate._epoch_fence.active == "epoch-b"
            assert gate._freshness.measured_received is None
            assert gate._reference_admission is None
        publish("epoch-b", True)
        spin_until(lambda: gate._freshness.measured_received is not None)
        assert gate._epoch_fence.active == "epoch-b"
        for valid in (True, False):
            publish("epoch-a", valid)
            for _ in range(8):
                rclpy.spin_once(gate, timeout_sec=0.02)
            assert gate._epoch_fence.active == "epoch-b"
            assert gate._freshness.measured_received is not None
            assert gate._reference_admission is not None
    finally:
        gate.destroy_node()
        producer.destroy_node()
        rclpy.shutdown()


def test_fusion_retires_old_scan_epoch_on_new_invalid_message(monkeypatch):
    from types import SimpleNamespace

    rclpy = pytest.importorskip("rclpy")
    from rclpy.parameter import Parameter
    from rclpy.time import Time
    from senior_motion_msgs.msg import ScanMotion
    from senior_robot.motion_fusion import MotionFusion

    values = {
        "sensor_max_age": 0.5,
        "alignment_max_gap": 0.2,
        "pending_wait": 0.1,
        "max_variance": 0.1,
        "min_quality": 0.3,
        "max_yaw_error": 0.1,
        "startup_window": 0.2,
        "wheel_motion_limit": 0.005,
        "lateral_limit": 0.02,
        "yaw_limit": 0.1,
    }
    rclpy.init()
    fusion = MotionFusion(parameter_overrides=[Parameter(k, value=v) for k, v in values.items()])
    emitted = []
    monkeypatch.setattr(fusion, "_scan_topology_ready", lambda: True)
    fusion._publisher = SimpleNamespace(publish=emitted.append)

    def message(epoch, valid):
        value = ScanMotion()
        value.response = 0.8
        value.valid_rays = 720
        value.covariance[0] = value.covariance[4] = value.covariance[8] = 0.001
        value.header.frame_id = "base_link"
        stamp = fusion.get_clock().now().nanoseconds
        value.header.stamp = Time(nanoseconds=stamp).to_msg()
        value.interval_start = Time(nanoseconds=stamp - 1).to_msg()
        value.source_epoch = epoch
        value.valid = valid
        value.startup_reference_valid = True
        return value

    try:
        fusion._on_scan(message("epoch-a", True))
        assert fusion._scan_epochs.active == "epoch-a" and fusion._pending is not None
        fusion._on_scan(message("epoch-b", False))
        assert fusion._scan_epochs.active == "epoch-b" and fusion._pending is None
        count = len(emitted)
        for valid in (True, False):
            fusion._on_scan(message("epoch-a", valid))
            assert fusion._scan_epochs.active == "epoch-b" and fusion._pending is None
            assert len(emitted) == count
        assert not emitted[-1].valid and not emitted[-1].startup_reference_valid
        fusion._on_scan(message("epoch-b", True))
        assert fusion._scan_epochs.active == "epoch-b" and fusion._pending is not None
    finally:
        fusion.destroy_node()
        rclpy.shutdown()


def test_later_invalid_prevents_older_valid_from_restoring_same_epoch():
    rclpy = pytest.importorskip("rclpy")
    from rclpy.parameter import Parameter
    from rclpy.time import Time
    from senior_motion_msgs.msg import MotionEvidence
    from senior_robot.safety_node import SafetyGate

    rclpy.init()
    producer = rclpy.create_node("motion_fusion")
    publisher = producer.create_publisher(MotionEvidence, "/motion/evidence", 10)
    gate = SafetyGate(
        parameter_overrides=[
            Parameter("observation_max_age", value=2.0),
            Parameter("measurement_max_age", value=2.0),
            Parameter("motion_min_quality", value=0.3),
            Parameter("motion_max_variance", value=0.1),
            Parameter("motion_max_interval", value=0.3),
        ]
    )

    def spin_until(predicate):
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            rclpy.spin_once(gate, timeout_sec=0.01)
            if predicate():
                return
        pytest.fail("same-epoch evidence did not reach expected state")

    def publish(stamp, valid):
        message = MotionEvidence()
        message.header.frame_id = "base_link"
        message.header.stamp = Time(nanoseconds=stamp).to_msg()
        message.interval_start = Time(nanoseconds=stamp - 100_000_000).to_msg()
        message.scan_received_ros = producer.get_clock().now().to_msg()
        message.scan_received_steady = time.monotonic()
        message.source_epoch = "epoch-a"
        message.valid = message.imu_valid = message.wheel_valid = valid
        message.quality = 0.8 if valid else 0.0
        message.covariance[0] = message.covariance[4] = message.covariance[8] = 0.001
        publisher.publish(message)

    def spin_pending():
        for _ in range(12):
            rclpy.spin_once(gate, timeout_sec=0.01)

    try:
        spin_until(gate._topology_ready)
        base = producer.get_clock().now().nanoseconds - 500_000_000
        publish(base, True)
        spin_until(lambda: gate._freshness.measured_received is not None)
        publish(base + 100_000_000, False)
        spin_until(lambda: gate._freshness.measured_received is None)
        publish(base + 50_000_000, True)
        spin_pending()
        assert gate._freshness.measured_received is None
        assert gate._history.latest is None and gate._reference_admission is None
        publish(base + 200_000_000, True)
        spin_until(lambda: gate._freshness.measured_received is not None)
        admission = gate._reference_admission
        publish(base + 150_000_000, False)
        spin_pending()
        assert gate._freshness.measured_received is not None
        assert gate._reference_admission == admission
    finally:
        gate.destroy_node()
        producer.destroy_node()
        rclpy.shutdown()


def test_fusion_later_invalid_blocks_older_same_epoch_scan(monkeypatch):
    from types import SimpleNamespace

    rclpy = pytest.importorskip("rclpy")
    from rclpy.parameter import Parameter
    from rclpy.time import Time
    from senior_motion_msgs.msg import ScanMotion
    from senior_robot.motion_fusion import MotionFusion

    values = {
        "sensor_max_age": 2.0,
        "alignment_max_gap": 0.2,
        "pending_wait": 0.1,
        "max_variance": 0.1,
        "min_quality": 0.3,
        "max_yaw_error": 0.1,
        "startup_window": 0.2,
        "wheel_motion_limit": 0.005,
        "lateral_limit": 0.02,
        "yaw_limit": 0.1,
    }
    rclpy.init()
    fusion = MotionFusion(parameter_overrides=[Parameter(k, value=v) for k, v in values.items()])
    emitted = []
    monkeypatch.setattr(fusion, "_scan_topology_ready", lambda: True)
    fusion._publisher = SimpleNamespace(publish=emitted.append)

    def deliver(stamp, valid):
        value = ScanMotion()
        value.response = 0.8
        value.valid_rays = 720
        value.covariance[0] = value.covariance[4] = value.covariance[8] = 0.001
        value.header.frame_id = "base_link"
        value.header.stamp = Time(nanoseconds=stamp).to_msg()
        value.interval_start = Time(nanoseconds=stamp - 1).to_msg()
        value.source_epoch = "epoch-a"
        value.valid = valid
        value.startup_reference_valid = True
        fusion._on_scan(value)

    try:
        base = fusion.get_clock().now().nanoseconds - 500_000_000
        deliver(base, True)
        assert fusion._pending is not None
        deliver(base + 100_000_000, False)
        assert fusion._pending is None and not emitted[-1].valid
        deliver(base + 50_000_000, True)
        assert fusion._pending is None
        deliver(base + 200_000_000, True)
        assert fusion._pending is not None
        pending = fusion._pending
        deliver(base + 150_000_000, False)
        assert fusion._pending is pending
        assert all(not item.startup_reference_valid for item in emitted)
    finally:
        fusion.destroy_node()
        rclpy.shutdown()


def test_gate_rejects_valid_interval_crossing_same_epoch_failure():
    rclpy = pytest.importorskip("rclpy")
    from rclpy.parameter import Parameter
    from rclpy.time import Time
    from senior_motion_msgs.msg import MotionEvidence
    from senior_robot.safety_node import SafetyGate

    rclpy.init()
    producer = rclpy.create_node("motion_fusion")
    publisher = producer.create_publisher(MotionEvidence, "/motion/evidence", 10)
    gate = SafetyGate(
        parameter_overrides=[
            Parameter("observation_max_age", value=2.0),
            Parameter("measurement_max_age", value=2.0),
            Parameter("motion_min_quality", value=0.3),
            Parameter("motion_max_variance", value=0.1),
            Parameter("motion_max_interval", value=0.3),
        ]
    )

    def spin_until(predicate):
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            rclpy.spin_once(gate, timeout_sec=0.01)
            if predicate():
                return
        pytest.fail("interval-boundary evidence did not reach expected state")

    def publish(start, end, valid):
        message = MotionEvidence()
        message.header.frame_id = "base_link"
        message.header.stamp = Time(nanoseconds=end).to_msg()
        message.interval_start = Time(nanoseconds=start).to_msg()
        message.scan_received_ros = producer.get_clock().now().to_msg()
        message.scan_received_steady = time.monotonic()
        message.source_epoch = "epoch-a"
        message.valid = message.imu_valid = message.wheel_valid = valid
        message.startup_reference_valid = True
        message.quality = 0.8 if valid else 0.0
        message.covariance[0] = message.covariance[4] = message.covariance[8] = 0.001
        publisher.publish(message)

    try:
        spin_until(gate._topology_ready)
        base = producer.get_clock().now().nanoseconds - 500_000_000
        publish(base - 100_000_000, base, True)
        spin_until(lambda: gate._freshness.measured_received is not None)
        failure = base + 100_000_000
        publish(base, failure, False)
        spin_until(lambda: gate._freshness.measured_received is None)
        publish(base + 50_000_000, base + 200_000_000, True)
        for _ in range(12):
            rclpy.spin_once(gate, timeout_sec=0.01)
        assert gate._freshness.measured_received is None
        assert gate._history.latest is None and gate._reference_admission is None
        publish(failure, base + 300_000_000, True)
        spin_until(lambda: gate._freshness.measured_received is not None)
        assert gate._history.at(failure / 1e9) is not None
    finally:
        gate.destroy_node()
        producer.destroy_node()
        rclpy.shutdown()


def test_fusion_rejects_scan_interval_crossing_same_epoch_failure(monkeypatch):
    from types import SimpleNamespace

    rclpy = pytest.importorskip("rclpy")
    from rclpy.parameter import Parameter
    from rclpy.time import Time
    from senior_motion_msgs.msg import ScanMotion
    from senior_robot.motion_fusion import MotionFusion

    values = {
        "sensor_max_age": 2.0,
        "alignment_max_gap": 0.2,
        "pending_wait": 0.1,
        "max_variance": 0.1,
        "min_quality": 0.3,
        "max_yaw_error": 0.1,
        "startup_window": 0.2,
        "wheel_motion_limit": 0.005,
        "lateral_limit": 0.02,
        "yaw_limit": 0.1,
    }
    rclpy.init()
    fusion = MotionFusion(parameter_overrides=[Parameter(k, value=v) for k, v in values.items()])
    emitted = []
    monkeypatch.setattr(fusion, "_scan_topology_ready", lambda: True)
    fusion._publisher = SimpleNamespace(publish=emitted.append)

    def deliver(start, end, valid):
        value = ScanMotion()
        value.response = 0.8
        value.valid_rays = 720
        value.covariance[0] = value.covariance[4] = value.covariance[8] = 0.001
        value.header.frame_id = "base_link"
        value.header.stamp = Time(nanoseconds=end).to_msg()
        value.interval_start = Time(nanoseconds=start).to_msg()
        value.source_epoch = "epoch-a"
        value.valid = valid
        value.startup_reference_valid = True
        fusion._on_scan(value)

    try:
        base = fusion.get_clock().now().nanoseconds - 500_000_000
        deliver(base - 100_000_000, base, True)
        assert fusion._pending is not None
        failure = base + 100_000_000
        deliver(base, failure, False)
        assert fusion._pending is None and not emitted[-1].valid
        deliver(base + 50_000_000, base + 200_000_000, True)
        assert fusion._pending is None
        deliver(failure, base + 300_000_000, True)
        assert fusion._pending is not None
        assert all(not item.startup_reference_valid for item in emitted)
    finally:
        fusion.destroy_node()
        rclpy.shutdown()


def test_fusion_retired_epoch_cannot_cancel_new_epoch_pending(monkeypatch):
    from types import SimpleNamespace

    rclpy = pytest.importorskip("rclpy")
    from rclpy.parameter import Parameter
    from rclpy.time import Time
    from senior_motion_msgs.msg import ScanMotion
    from senior_robot.motion_fusion import MotionFusion

    values = {
        "sensor_max_age": 2.0,
        "alignment_max_gap": 0.2,
        "pending_wait": 0.1,
        "max_variance": 0.1,
        "min_quality": 0.3,
        "max_yaw_error": 0.1,
        "startup_window": 0.2,
        "wheel_motion_limit": 0.005,
        "lateral_limit": 0.02,
        "yaw_limit": 0.1,
    }
    rclpy.init()
    fusion = MotionFusion(parameter_overrides=[Parameter(k, value=v) for k, v in values.items()])
    emitted = []
    monkeypatch.setattr(fusion, "_scan_topology_ready", lambda: True)
    fusion._publisher = SimpleNamespace(publish=emitted.append)

    def deliver(epoch, end, valid):
        value = ScanMotion()
        value.response = 0.8
        value.valid_rays = 720
        value.covariance[0] = value.covariance[4] = value.covariance[8] = 0.001
        value.header.frame_id = "base_link"
        value.header.stamp = Time(nanoseconds=end).to_msg()
        value.interval_start = Time(nanoseconds=end - 1).to_msg()
        value.source_epoch = epoch
        value.valid = valid
        value.startup_reference_valid = True
        fusion._on_scan(value)

    try:
        base = fusion.get_clock().now().nanoseconds - 500_000_000
        deliver("epoch-a", base, True)
        deliver("epoch-b", base + 100_000_000, False)
        deliver("epoch-b", base + 200_000_000, True)
        pending = fusion._pending
        assert pending is not None
        count = len(emitted)
        for valid in (True, False):
            deliver("epoch-a", base + 250_000_000, valid)
            assert fusion._pending is pending
            assert fusion._scan_epochs.active == "epoch-b"
            assert len(emitted) == count
    finally:
        fusion.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize("invalid_start", ["zero", "equal", "after"])
def test_fusion_rejects_nonpositive_or_reversed_interval(monkeypatch, invalid_start):
    from types import SimpleNamespace

    rclpy = pytest.importorskip("rclpy")
    from rclpy.parameter import Parameter
    from rclpy.time import Time
    from senior_motion_msgs.msg import ScanMotion
    from senior_robot.motion_fusion import MotionFusion

    values = {
        "sensor_max_age": 2.0,
        "alignment_max_gap": 0.2,
        "pending_wait": 0.1,
        "max_variance": 0.1,
        "min_quality": 0.3,
        "max_yaw_error": 0.1,
        "startup_window": 0.2,
        "wheel_motion_limit": 0.005,
        "lateral_limit": 0.02,
        "yaw_limit": 0.1,
    }
    rclpy.init()
    fusion = MotionFusion(parameter_overrides=[Parameter(k, value=v) for k, v in values.items()])
    emitted = []
    monkeypatch.setattr(fusion, "_scan_topology_ready", lambda: True)
    fusion._publisher = SimpleNamespace(publish=emitted.append)

    def deliver(start, end):
        value = ScanMotion()
        value.response = 0.8
        value.valid_rays = 720
        value.covariance[0] = value.covariance[4] = value.covariance[8] = 0.001
        value.header.frame_id = "base_link"
        value.header.stamp = Time(nanoseconds=end).to_msg()
        value.interval_start = Time(nanoseconds=start).to_msg()
        value.source_epoch = "epoch-a"
        value.valid = True
        fusion._on_scan(value)

    try:
        end = fusion.get_clock().now().nanoseconds - 200_000_000
        start = {"zero": 0, "equal": end, "after": end + 1}[invalid_start]
        deliver(start, end)
        assert fusion._pending is None and emitted and not emitted[-1].valid
        deliver(end, end + 100_000_000)
        assert fusion._pending is not None
    finally:
        fusion.destroy_node()
        rclpy.shutdown()


def test_fusion_clock_rollback_retires_pending_and_old_epoch(monkeypatch):
    from types import SimpleNamespace

    rclpy = pytest.importorskip("rclpy")
    from rclpy.parameter import Parameter
    from rclpy.time import Time
    from senior_motion_msgs.msg import ScanMotion
    from senior_robot import motion_fusion as module

    from senior_control.models import Velocity

    values = {
        "sensor_max_age": 2.0,
        "alignment_max_gap": 0.2,
        "pending_wait": 0.1,
        "max_variance": 0.1,
        "min_quality": 0.3,
        "max_yaw_error": 0.1,
        "startup_window": 0.2,
        "wheel_motion_limit": 0.005,
        "lateral_limit": 0.02,
        "yaw_limit": 0.1,
    }
    rclpy.init()
    fusion = module.MotionFusion(
        parameter_overrides=[Parameter(k, value=v) for k, v in values.items()]
    )
    emitted = []
    clock = {"now": 101_000_000_000}
    monkeypatch.setattr(fusion, "_scan_topology_ready", lambda: True)
    monkeypatch.setattr(
        fusion,
        "get_clock",
        lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=clock["now"])),
    )
    fusion._publisher = SimpleNamespace(publish=emitted.append)

    def message(epoch, end):
        value = ScanMotion()
        value.header.frame_id = "base_link"
        value.header.stamp = Time(nanoseconds=end).to_msg()
        value.interval_start = Time(nanoseconds=end - 100_000_000).to_msg()
        value.source_epoch = epoch
        value.valid = True
        value.startup_reference_valid = True
        value.response = 0.8
        value.valid_rays = 720
        value.covariance[0] = value.covariance[4] = value.covariance[8] = 0.001
        return value

    try:
        fusion._on_scan(message("epoch-a", 100_900_000_000))
        assert fusion._pending is not None
        fusion._slip._latched = True
        fusion._stop_acknowledgement.trip()
        fusion._wheel_received = fusion._imu_received = time.monotonic()
        monkeypatch.setattr(
            fusion._commands, "integrated", lambda *_: SimpleNamespace(velocity=Velocity(0.0, 0.0))
        )
        monkeypatch.setattr(
            module,
            "qualify_interval",
            lambda *_: SimpleNamespace(
                estimate=SimpleNamespace(velocity=Velocity(0.0, 0.0), quality=0.8), anomalous=False
            ),
        )
        clock["now"] = 100_000_000_000
        fusion._try_pending()
        assert fusion._pending is None and not any(item.valid for item in emitted)
        assert fusion._scan_epochs.active is None
        assert fusion._slip._latched and not fusion._stop_acknowledgement.ready
        # An old frame can again look recent after the rolled-back clock catches up.
        clock["now"] = 101_000_000_000
        fusion._on_scan(message("epoch-a", 100_900_000_000))
        assert fusion._pending is None and fusion._scan_epochs.active is None
        fusion._on_scan(message("epoch-b", 100_950_000_000))
        assert fusion._scan_epochs.active == "epoch-b" and fusion._pending is not None
        assert all(not item.startup_reference_valid for item in emitted)
    finally:
        fusion.destroy_node()
        rclpy.shutdown()


def test_real_fusion_publisher_restart_requalifies_same_matcher_epoch():
    """Exercise ROS discovery; aligned stationary samples are test-owned inputs."""
    rclpy = pytest.importorskip("rclpy")
    from geometry_msgs.msg import Twist
    from rclpy.parameter import Parameter
    from rclpy.time import Time
    from senior_motion_msgs.msg import MotionEvidence, ScanMotion
    from senior_robot.motion_fusion import MotionFusion
    from senior_robot.safety_node import SafetyGate

    from senior_control.models import Pose2D
    from senior_control.safety.motion_runtime import ImuSample, TimedPoseSample

    values = {
        "sensor_max_age": 2.0,
        "alignment_max_gap": 0.2,
        "pending_wait": 0.1,
        "max_variance": 0.1,
        "min_quality": 0.3,
        "max_yaw_error": 0.1,
        "startup_window": 0.2,
        "wheel_motion_limit": 0.005,
        "lateral_limit": 0.02,
        "yaw_limit": 0.1,
        "progress_stationary_max_linear_speed": 0.001,
        "progress_stationary_max_yaw_rate": 0.001,
    }
    rclpy.init()
    matcher = rclpy.create_node("scan_motion_node")
    publisher = matcher.create_publisher(ScanMotion, "/motion/scan", 10)
    observer = rclpy.create_node("fusion_restart_observer")
    heard = []
    subscription = observer.create_subscription(
        MotionEvidence, "/motion/evidence", heard.append, 10
    )
    gate = SafetyGate(
        parameter_overrides=[
            Parameter("observation_max_age", value=2.0),
            Parameter("measurement_max_age", value=2.0),
            Parameter("motion_min_quality", value=0.3),
            Parameter("motion_max_variance", value=0.1),
            Parameter("motion_max_interval", value=0.3),
        ]
    )
    fusion = None

    def spin_until(predicate):
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            for node in (fusion, gate, observer):
                if node is not None:
                    rclpy.spin_once(node, timeout_sec=0.01)
            if predicate():
                return
        pytest.fail("fusion restart graph or evidence did not reach expected state")

    def publish_stationary():
        previous_ns = None
        for _ in range(12):
            fusion._on_requested(Twist())
            fusion._on_command(Twist())
            end_ns = matcher.get_clock().now().nanoseconds
            stamp = end_ns / 1e9
            assert fusion._wheel.add(TimedPoseSample(stamp, Pose2D(0, 0, 0), (0.001,) * 3, "odom"))
            assert fusion._imu.add(ImuSample(stamp, 0.0, 0.001))
            fusion._wheel_received = fusion._imu_received = time.monotonic()
            if previous_ns is not None:
                message = ScanMotion()
                message.header.frame_id = "base_link"
                message.header.stamp = Time(nanoseconds=end_ns).to_msg()
                message.interval_start = Time(nanoseconds=previous_ns).to_msg()
                message.source_epoch = "unchanged-matcher-epoch"
                message.valid = True
                message.startup_reference_valid = True
                message.response = 0.8
                message.valid_rays = 720
                message.covariance[0] = message.covariance[4] = message.covariance[8] = 0.001
                publisher.publish(message)
            previous_ns = end_ns
            for node in (fusion, gate, observer):
                rclpy.spin_once(node, timeout_sec=0.01)
            time.sleep(0.04)
        return previous_ns

    def last_source_sample_consumed(stamp_ns):
        if not heard or not heard[-1].valid or gate._reference_admission is None:
            return False
        header = heard[-1].header.stamp
        observed_ns = header.sec * 1_000_000_000 + header.nanosec
        return (
            observed_ns == stamp_ns
            and gate._reference_admission[0] == stamp_ns / 1e9
            and gate._epoch_fence.active == heard[-1].source_epoch
            and gate._freshness.measured_received is not None
        )

    try:
        fusion = MotionFusion(
            parameter_overrides=[Parameter(k, value=v) for k, v in values.items()]
        )
        spin_until(
            lambda: (
                gate._topology_ready()
                and fusion._scan_topology_ready()
                and publisher.get_subscription_count() == 1
                and fusion._publisher.get_subscription_count() >= 2
            )
        )
        last_stamp = publish_stationary()
        spin_until(lambda: last_source_sample_consumed(last_stamp))
        old = heard[-1]
        assert not old.startup_reference_valid
        fusion.destroy_node()
        fusion = None
        spin_until(lambda: not gate._topology_ready())
        assert gate._freshness.measured_received is None
        fusion = MotionFusion(
            parameter_overrides=[Parameter(k, value=v) for k, v in values.items()]
        )
        spin_until(
            lambda: (
                gate._topology_ready()
                and fusion._scan_topology_ready()
                and publisher.get_subscription_count() == 1
                and fusion._publisher.get_subscription_count() >= 2
            )
        )
        old_count = len(heard)
        last_stamp = publish_stationary()
        spin_until(lambda: len(heard) > old_count and last_source_sample_consumed(last_stamp))
        new = heard[-1]
        assert new.source_epoch != old.source_epoch
        spin_until(lambda: gate._freshness.measured_received is not None)
        assert not new.startup_reference_valid
        admission = gate._reference_admission
        # Deliver a retained pre-restart sample via the sole current endpoint.
        fusion._publisher.publish(old)
        for _ in range(12):
            rclpy.spin_once(gate, timeout_sec=0.01)
        assert gate._epoch_fence.active == new.source_epoch
        assert gate._reference_admission == admission
        assert subscription
    finally:
        if fusion is not None:
            fusion.destroy_node()
        gate.destroy_node()
        observer.destroy_node()
        matcher.destroy_node()
        rclpy.shutdown()
