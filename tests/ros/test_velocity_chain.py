"""Command ownership publishes into the safety gate and nowhere else.

The monotonic clock, not ROS time, expires a command while /clock is frozen.
This file is skipped without rclpy; a skip is not a pass on macOS.
"""

import pytest


def _blocked_guard_processor(work):
    import os
    import time
    from pathlib import Path

    from senior_control.safety.async_guard import SafetyWorkResult
    from senior_control.safety.guard import SafetyDecision

    root = Path(os.environ["SENIOR_GUARD_TEST_PATH"])
    (root / "worker-started").touch()
    while not (root / "worker-release").exists():
        time.sleep(0.01)
    selected = work.observations[-1]
    return SafetyWorkResult(
        SafetyDecision(True, work.command, "hard_sweep_clear"),
        {},
        0.0,
        selected.received,
        selected.observation.scan.stamp,
        selected.observation.scan.reference_stamp,
        0.0,
        (0.0, 0.0, 0.0),
    )


def _publisher_topics(node: object) -> set[str]:
    publishers = getattr(node, "publishers", None)
    if publishers is None:
        publishers = getattr(node, "_publishers", ())
    return {publisher.topic_name for publisher in publishers}


def _spin_until(node: object, ready) -> None:
    rclpy = pytest.importorskip("rclpy")
    deadline_spins = 20
    for _ in range(deadline_spins):
        if ready():
            return
        rclpy.spin_once(node, timeout_sec=0.05)
    assert ready()


def test_sim_pause_expires_command():
    """Breaks if the command node lets frozen ROS time keep the last Twist alive."""
    rclpy = pytest.importorskip("rclpy")
    from geometry_msgs.msg import Twist
    from rclpy.parameter import Parameter
    from senior_robot.command_node import CommandNode

    from senior_control.protocol import (
        EmptyPayload,
        SimpleEnvelope,
        VelocityEnvelope,
        VelocityPayload,
    )
    from senior_control.safety.arbiter import Arbiter

    class FrozenClock:
        def __init__(self) -> None:
            self.now = 1000.0

        def monotonic(self) -> float:
            return self.now

    def envelope(
        kind: str, seq: int, vx: float = 0.0, wz: float = 0.0
    ) -> SimpleEnvelope | VelocityEnvelope:
        common = {
            "v": 1,
            "robot_id": "robot-1",
            "session_id": "session-1",
            "connection_epoch": 1,
            "request_id": f"req-{seq}",
            "seq": seq,
            "expires_at": 42.0,
        }
        if kind == "drive.velocity":
            return VelocityEnvelope(
                type="drive.velocity",
                payload=VelocityPayload(vx=vx, wz=wz),
                **common,  # type: ignore[arg-type]
            )
        return SimpleEnvelope(type=kind, payload=EmptyPayload(), **common)  # type: ignore[arg-type]

    rclpy.init()
    clock = FrozenClock()
    arbiter = Arbiter(
        "robot-1",
        heartbeat_seconds=0.25,
        disconnect_seconds=5.0,
        command_ttl_seconds=0.5,
    )
    node = CommandNode(
        arbiter=arbiter,
        clock=clock,
        parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
    )
    listener = rclpy.create_node("velocity_chain_listener")
    heard: list[Twist] = []
    cmd_vel: list[Twist] = []
    listener.create_subscription(Twist, "/safety/command_in", heard.append, 10)
    listener.create_subscription(Twist, "/cmd_vel", cmd_vel.append, 10)
    try:
        node.submit(envelope("control.acquire", 1))
        node.submit(envelope("drive.velocity", 2, vx=0.25, wz=-0.1))
        _spin_until(listener, lambda: bool(heard) and heard[-1].linear.x == 0.25)
        assert heard[-1].angular.z == -0.1
        frozen = node.get_clock().now().nanoseconds
        clock.now = 1000.5 + 1e-9
        node.publish_command()
        _spin_until(
            listener,
            lambda: heard[-1].linear.x == 0.0 and heard[-1].angular.z == 0.0,
        )
        assert node.get_clock().now().nanoseconds == frozen
        assert cmd_vel == []
        clock.now = 1001.0
        node.submit(envelope("drive.velocity", 3, vx=0.4, wz=0.1))
        _spin_until(listener, lambda: heard[-1].linear.x == 0.4 and heard[-1].angular.z == 0.1)
        node.disconnect(1)
        _spin_until(listener, lambda: heard[-1].linear.x == 0.0 and heard[-1].angular.z == 0.0)
        topics = _publisher_topics(node)
        assert "/safety/command_in" in topics
        assert "/cmd_vel" not in topics
    finally:
        listener.destroy_node()
        node.destroy_node()
        rclpy.shutdown()


def test_safety_gate_alone_publishes_cmd_vel():
    """Breaks if command_node takes /cmd_vel or the safety gate stops publishing it."""
    rclpy = pytest.importorskip("rclpy")
    from senior_robot.command_node import CommandNode
    from senior_robot.safety_node import SafetyGate

    rclpy.init()
    safety = SafetyGate()
    command = CommandNode()
    try:
        assert safety._profile().footprint == []
        assert "/cmd_vel" in _publisher_topics(safety)
        command_topics = _publisher_topics(command)
        assert "/safety/command_in" in command_topics
        assert "/cmd_vel" not in command_topics
    finally:
        command.destroy_node()
        safety.destroy_node()
        rclpy.shutdown()


def test_motion_fusion_compares_the_safety_applied_command():
    rclpy = pytest.importorskip("rclpy")
    from rclpy.parameter import Parameter
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
    node = MotionFusion(parameter_overrides=[Parameter(k, value=v) for k, v in values.items()])
    try:
        topics = {subscription.topic_name for subscription in node._subscriptions}
        assert "/cmd_vel" in topics
        # Requested zero only acknowledges a latched stop; the compared command is /cmd_vel.
        assert "/safety/command_in" in topics
        assert "/cmd_vel" not in _publisher_topics(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_safety_scan_uses_timestamped_tf_and_fails_closed_without_measurement():
    rclpy = pytest.importorskip("rclpy")
    from geometry_msgs.msg import TransformStamped
    from rclpy.parameter import Parameter
    from rclpy.time import Time
    from senior_motion_msgs.msg import MotionEvidence
    from senior_robot.safety_node import SafetyGate
    from sensor_msgs.msg import LaserScan

    rclpy.init()
    gate = SafetyGate(
        parameter_overrides=[
            Parameter("observation_max_age", value=0.5),
            Parameter("measurement_max_age", value=0.5),
            Parameter("transform_static", value=True),
            Parameter("self_mask_calibrated", value=True),
            Parameter("motion_min_quality", value=0.4),
            Parameter("motion_max_variance", value=0.1),
            Parameter("motion_max_interval", value=0.3),
        ]
    )
    try:
        tf = TransformStamped()
        tf.header.frame_id = "base_link"
        tf.child_frame_id = "laser_link"
        tf.transform.translation.x = 0.25
        tf.transform.rotation.w = 1.0
        gate._tf_buffer.set_transform_static(tf, "test")
        scan = LaserScan()
        scan.header.frame_id = "laser_link"
        scan.header.stamp = gate.get_clock().now().to_msg()
        scan.angle_increment = 0.1
        scan.range_min, scan.range_max = 0.1, 10.0
        scan.ranges = [1.0]
        gate._on_scan(scan)
        assert gate._filtered is not None
        assert gate._filtered.rays[0].origin == (0.25, 0.0)
        assert gate._filtered.rays[0].endpoint == (1.25, 0.0)
        now = gate.get_clock().now().nanoseconds / 1e9
        import time

        assert not gate._freshness.usable(now, time.monotonic())
        measured = MotionEvidence()
        measured.header.frame_id = "base_link"
        measured.header.stamp = scan.header.stamp
        start = Time.from_msg(scan.header.stamp).nanoseconds - 100_000_000
        measured.interval_start = Time(nanoseconds=start).to_msg()
        measured.source_epoch = "scan-epoch"
        measured.valid = measured.imu_valid = measured.wheel_valid = True
        measured.quality = 0.8
        measured.covariance[0] = measured.covariance[4] = measured.covariance[8] = 0.001
        gate._accept_evidence(measured)
        now = gate.get_clock().now().nanoseconds / 1e9
        assert gate._freshness.usable(now, time.monotonic())
        assert not gate._freshness.usable(now, time.monotonic() + 1.0)
        measured.quality = 0.0
        gate._accept_evidence(measured)
        assert not gate._freshness.usable(now, time.monotonic())
    finally:
        gate.destroy_node()
        rclpy.shutdown()


def test_safety_steady_timer_publishes_zero_while_ros_time_is_frozen():
    rclpy = pytest.importorskip("rclpy")
    from geometry_msgs.msg import Twist
    from rclpy.parameter import Parameter
    from senior_robot.safety_node import SafetyGate

    rclpy.init()
    gate = SafetyGate(parameter_overrides=[Parameter("use_sim_time", value=True)])
    listener = rclpy.create_node("safety_timer_listener")
    heard = []
    listener.create_subscription(Twist, "/cmd_vel", heard.append, 10)
    try:
        frozen = gate.get_clock().now().nanoseconds
        for _ in range(40):
            rclpy.spin_once(gate, timeout_sec=0.05)
            rclpy.spin_once(listener, timeout_sec=0.05)
            if heard:
                break
        assert heard and heard[-1].linear.x == 0.0 and heard[-1].angular.z == 0.0
        assert gate.get_clock().now().nanoseconds == frozen
    finally:
        listener.destroy_node()
        gate.destroy_node()
        rclpy.shutdown()


def test_safety_unconfigured_arrays_fail_closed_without_callback_exception():
    rclpy = pytest.importorskip("rclpy")
    from senior_robot.safety_node import SafetyGate

    rclpy.init()
    gate = SafetyGate()
    try:
        profile = gate._profile()
        assert not profile.footprint
        assert not profile.self_mask
        gate._on_timer()
    finally:
        gate.destroy_node()
        rclpy.shutdown()


def test_safety_reports_missing_evidence_topology_reason():
    """A zero must expose its bounded reason on the diagnostic channel."""
    import time

    rclpy = pytest.importorskip("rclpy")
    from diagnostic_msgs.msg import DiagnosticArray
    from senior_robot.safety_node import SafetyGate

    rclpy.init()
    gate = SafetyGate()
    listener = rclpy.create_node("safety_diagnostic_listener")
    heard = []
    listener.create_subscription(DiagnosticArray, "/safety/status", heard.append, 10)
    try:
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and not heard:
            gate._on_timer()
            rclpy.spin_once(listener, timeout_sec=0.05)
        assert heard and heard[-1].status[0].message == "evidence_topology_unqualified"
    finally:
        listener.destroy_node()
        gate.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize("change", ["stop", "scan_loss", "epoch_change"])
def test_safety_timer_never_releases_late_result_after_input_change(tmp_path, monkeypatch, change):
    import threading
    import time

    rclpy = pytest.importorskip("rclpy")
    from geometry_msgs.msg import Twist
    from rclpy.parameter import Parameter
    from senior_robot import safety_node as module
    from sensor_msgs.msg import LaserScan

    from senior_control.config import DeviceProfile
    from senior_control.models import Pose2D, Velocity
    from senior_control.safety.async_guard import BoundedSafetyWorker
    from senior_control.safety.guard import SafetyDecision
    from senior_control.safety.motion_runtime import (
        SafetyObservation,
        StampedObservation,
        TimedPoseSample,
    )
    from senior_control.safety.self_filter import ScanObservation, filter_self

    rclpy.init()
    gate = module.SafetyGate(
        parameter_overrides=[
            Parameter("command_ttl_seconds", value=5.0),
            Parameter("observation_max_age", value=10.0),
            Parameter("measurement_max_age", value=10.0),
            Parameter("motion_max_interval", value=5.0),
            Parameter("motion_max_variance", value=1.0),
            Parameter("max_reference_lag", value=5.0),
            Parameter("max_linear_speed", value=1.0),
            Parameter("max_yaw_rate", value=1.0),
        ]
    )
    monkeypatch.setenv("SENIOR_GUARD_TEST_PATH", str(tmp_path))
    worker = BoundedSafetyWorker(_blocked_guard_processor)
    gate._worker = worker
    outputs = []
    monkeypatch.setattr(
        gate, "_publish", lambda velocity, reason: outputs.append((velocity, reason))
    )
    monkeypatch.setattr(gate, "_topology_ready", lambda: True)
    monkeypatch.setattr(gate, "_command_topology_ready", lambda: True)
    monkeypatch.setattr(gate, "_clock_ok", lambda now: True)
    profile = DeviceProfile(
        footprint=[(-0.3, -0.2), (0.3, -0.2), (0.3, 0.2), (-0.3, 0.2)],
        self_mask_calibrated=True,
        braking_deceleration=1.0,
        yaw_braking_deceleration=1.0,
        reaction_time=0.2,
        hard_margin=0.02,
        observation_max_age=10.0,
    )
    monkeypatch.setattr(gate, "_profile", lambda: profile)
    now_ros = gate.get_clock().now().nanoseconds / 1e9
    now_steady = time.monotonic()
    scan = ScanObservation(
        (2.0, 2.0),
        -0.1,
        0.2,
        0.05,
        5.0,
        now_ros,
        "ros",
        Pose2D(0, 0, 0),
        0,
        "base_link",
        now_ros,
        True,
        0.2,
        False,
        True,
    )
    observation = filter_self(scan, profile)
    gate._scans.append(StampedObservation(observation, now_steady))
    gate._filtered = observation
    gate._command = Velocity(0.05, 0.0)
    gate._command_monotonic = now_steady
    gate._epoch_fence.accept("epoch-a")
    gate._history.add(TimedPoseSample(now_ros, Pose2D(0, 0, 0), (0.01, 0.01, 0.01), "epoch-a"))
    gate._reference_admission = (now_ros, now_ros, now_steady)
    gate._freshness.scan_received = gate._freshness.measured_received = now_steady
    gate._freshness.scan_stamp = gate._freshness.measured_stamp = now_ros
    monkeypatch.setattr(
        module,
        "select_safety_observation",
        lambda *args, **kwargs: SafetyObservation(observation, 0.01),
        raising=False,
    )
    monkeypatch.setattr(module, "newer_scan_veto", lambda *args, **kwargs: False, raising=False)

    def blocked_geometry(*args, **kwargs):
        (tmp_path / "worker-started").touch()
        while not (tmp_path / "worker-release").exists():
            time.sleep(0.01)
        return SafetyDecision(True, Velocity(0.05, 0.0), "hard_sweep_clear")

    monkeypatch.setattr(module, "evaluate_motion", blocked_geometry, raising=False)
    timer = threading.Thread(target=gate._on_timer, daemon=True)
    try:
        timer.start()
        timer.join(timeout=0.3)
        assert not timer.is_alive(), "the ROS safety timer blocked on geometry"
        deadline = time.monotonic() + 5
        while not (tmp_path / "worker-started").exists() and time.monotonic() < deadline:
            time.sleep(0.01)
        assert (tmp_path / "worker-started").exists()
        assert outputs and outputs[-1][0] == Velocity(0, 0)
        if change == "stop":
            gate._on_command(Twist())
        elif change == "scan_loss":
            gate._on_scan(LaserScan())
        else:
            gate._epoch_fence.retire_active()
            gate._epoch_fence.accept("epoch-b")
        (tmp_path / "worker-release").touch()
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            gate._on_timer()
            time.sleep(0.01)
        assert all(velocity == Velocity(0, 0) for velocity, _ in outputs)
    finally:
        (tmp_path / "worker-release").touch()
        timer.join(timeout=2)
        worker.close()
        gate.destroy_node()
        rclpy.shutdown()


def test_frozen_ros_reference_cannot_be_renewed_by_new_work_ticket(monkeypatch):
    from types import SimpleNamespace

    rclpy = pytest.importorskip("rclpy")
    from rclpy.parameter import Parameter
    from senior_robot import safety_node as module

    from senior_control.config import DeviceProfile
    from senior_control.models import Pose2D, Velocity
    from senior_control.safety.async_guard import SafetyWorkResult, WorkerReply
    from senior_control.safety.guard import SafetyDecision
    from senior_control.safety.motion_runtime import StampedObservation, TimedPoseSample
    from senior_control.safety.self_filter import ScanObservation, filter_self

    class ImmediateWorker:
        def __init__(self):
            self.reply = None

        @property
        def pending(self):
            return self.reply is not None

        def submit(self, ticket, work):
            selected = work.observations[-1]
            result = SafetyWorkResult(
                SafetyDecision(True, work.command, "hard_sweep_clear"),
                {},
                0.0,
                selected.received,
                selected.observation.scan.stamp,
                selected.observation.scan.reference_stamp,
                0.1,
                (0, 0, 0),
            )
            self.reply = WorkerReply(ticket, result, None)
            return True

        def poll(self):
            reply, self.reply = self.reply, None
            return reply

        def close(self):
            pass

    rclpy.init()
    gate = module.SafetyGate(
        parameter_overrides=[
            Parameter("command_ttl_seconds", value=0.3),
            Parameter("observation_max_age", value=0.8),
            Parameter("measurement_max_age", value=0.8),
            Parameter("motion_max_interval", value=0.4),
            Parameter("motion_max_variance", value=1.0),
            Parameter("motion_min_quality", value=0.3),
            Parameter("max_reference_lag", value=0.4),
            Parameter("max_linear_speed", value=1.0),
            Parameter("max_yaw_rate", value=1.0),
        ]
    )
    values = {"steady": 100.0, "ros": 20.2}
    monkeypatch.setattr(module.time, "monotonic", lambda: values["steady"])
    monkeypatch.setattr(
        gate,
        "get_clock",
        lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=int(values["ros"] * 1e9))),
    )
    monkeypatch.setattr(gate, "_clock_ok", lambda now: True)
    monkeypatch.setattr(gate, "_topology_ready", lambda: True)
    monkeypatch.setattr(gate, "_command_topology_ready", lambda: True)
    profile = DeviceProfile(
        footprint=[(-0.3, -0.2), (0.3, -0.2), (0.3, 0.2), (-0.3, 0.2)],
        self_mask_calibrated=True,
        braking_deceleration=1.0,
        yaw_braking_deceleration=1.0,
        reaction_time=0.2,
        hard_margin=0.02,
        observation_max_age=0.8,
    )
    monkeypatch.setattr(gate, "_profile", lambda: profile)
    outputs = []
    monkeypatch.setattr(gate, "_publish", lambda velocity, reason: outputs.append(velocity))
    gate._worker.close()
    gate._worker = ImmediateWorker()
    scan = ScanObservation(
        (2.0, 2.0),
        -0.1,
        0.2,
        0.05,
        5.0,
        20.0,
        "ros",
        Pose2D(0, 0, 0),
        0,
        "base_link",
        20.0,
        True,
        0.2,
        False,
        True,
    )
    obs = filter_self(scan, profile)
    gate._scans.append(StampedObservation(obs, 100.0))
    gate._filtered = obs
    gate._command = Velocity(0.05, 0.0)
    gate._command_monotonic = 100.0
    gate._epoch_fence.accept("epoch-a")
    gate._history.add(TimedPoseSample(20.0, Pose2D(0, 0, 0), (0.01,) * 3, "epoch-a"))
    gate._freshness.scan_received = gate._freshness.measured_received = 100.0
    gate._freshness.scan_stamp = gate._freshness.measured_stamp = 20.0
    # Reference was already 0.2 s old when received at steady=100.0.
    gate._reference_admission = (20.0, 20.2, 100.0)
    gate._last_ros_advance_steady = 100.0
    try:
        gate._on_timer()
        values["steady"] = 100.05
        gate._command_monotonic = values["steady"]
        gate._on_timer()
        assert outputs[-1] == Velocity(0.05, 0.0)
        # More than the remaining 0.2 s passed, but sensor age is below 0.8 s;
        # fresh repeated commands must not extend this old pose's geometry.
        for steady in (100.21, 100.22, 100.30, 100.31):
            values["steady"] = steady
            gate._command_monotonic = steady
            gate._on_timer()
        assert all(value == Velocity(0, 0) for value in outputs[-4:])
        # A queued old frame arrives late with a newer source stamp, but ROS
        # time has not advanced since steady=100.0. Receipt cannot restart it.
        from rclpy.time import Time
        from senior_motion_msgs.msg import MotionEvidence

        late = MotionEvidence()
        late.header.frame_id = "base_link"
        late.header.stamp = Time(nanoseconds=20_050_000_000).to_msg()
        late.interval_start = Time(nanoseconds=20_000_000_000).to_msg()
        late.source_epoch = "epoch-a"
        late.valid = late.imu_valid = late.wheel_valid = True
        late.quality = 0.8
        late.covariance[0] = late.covariance[4] = late.covariance[8] = 0.01
        gate._accept_evidence(late)
        assert gate._history.latest.stamp == pytest.approx(20.05)
        for steady in (100.32, 100.33, 100.34):
            values["steady"] = steady
            gate._command_monotonic = steady
            gate._on_timer()
        assert all(value == Velocity(0, 0) for value in outputs[-3:])
    finally:
        gate.destroy_node()
        rclpy.shutdown()


def test_unstamped_autonomous_twist_has_no_motion_input():
    rclpy = pytest.importorskip("rclpy")
    from senior_robot.command_node import CommandNode

    rclpy.init()
    node = CommandNode()
    try:
        subscriptions = getattr(node, "subscriptions", None)
        if subscriptions is None or callable(subscriptions):
            subscriptions = node._subscriptions
        topics = {subscription.topic_name for subscription in subscriptions}
        assert "/control/navigation_cmd" not in topics
        assert "/control/recovery_cmd" not in topics
    finally:
        node.destroy_node()
        rclpy.shutdown()
