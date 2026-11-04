"""C6 ROS/Fusion reproduction: a received but superseded scan still matters."""

import json
import math
import time
from types import SimpleNamespace

import pytest
from motion_progress_test_helpers import first_zero_acknowledgements, prime_stationary_progress


def test_zero_ack_barrier_excludes_inflight_nonzero_without_discarding_events():
    def twist(vx):
        return SimpleNamespace(linear=SimpleNamespace(x=vx), angular=SimpleNamespace(z=0.0))

    requests = [(9.9, twist(0.04)), (10.01, twist(0.04)), (10.02, twist(0.0))]
    outputs = [(10.015, twist(0.0)), (10.03, twist(0.0))]
    assert first_zero_acknowledgements(requests, outputs, intent_steady=10.0) == (10.02, 10.03)
    assert not all(message.linear.x == 0 for received, message in requests if received >= 10.0)
    assert len(requests) == 3 and len(outputs) == 2
    assert first_zero_acknowledgements(requests, outputs[:1], intent_steady=10.0) is None


@pytest.mark.parametrize(
    ("qualified_profile", "wheel_step", "body_step", "imu_rate"),
    [(False, 0, 0, 0), (True, 0.001, 0, 0), (True, 0, 0.001, 0), (True, 0, 0, 0.01)],
    ids=["unqualified-profile", "wheel-moving", "body-moving", "imu-rotation"],
)
def test_initial_progress_requires_qualified_stationary_observations(
    monkeypatch, qualified_profile, wheel_step, body_step, imu_rate
):
    rclpy = pytest.importorskip("rclpy")
    from rclpy.parameter import Parameter
    from senior_robot.motion_fusion import MotionFusion

    values = {
        "sensor_max_age": 2.0,
        "alignment_max_gap": 0.3,
        "pending_wait": 0.2,
        "max_variance": 0.1,
        "min_quality": 0.3,
        "max_yaw_error": 0.1,
        "startup_window": 0.6,
        "wheel_motion_limit": 0.004,
        "stall_distance": 0.04,
        "lateral_limit": 0.04,
        "yaw_limit": 0.12,
    }
    if qualified_profile:
        values.update(
            progress_stationary_max_linear_speed=0.001,
            progress_stationary_max_yaw_rate=0.001,
        )
    rclpy.init()
    fusion = MotionFusion(parameter_overrides=[Parameter(k, value=v) for k, v in values.items()])
    emitted = []
    clock = {"now": 100_010_000_000}
    monkeypatch.setattr(fusion, "_scan_topology_ready", lambda: True)
    monkeypatch.setattr(
        fusion,
        "get_clock",
        lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=clock["now"])),
    )
    monkeypatch.setattr(fusion, "_publisher", SimpleNamespace(publish=emitted.append))
    try:
        prime_stationary_progress(
            fusion,
            clock,
            100_000_000_000,
            expect_qualified=False,
            wheel_step=wheel_step,
            body_step=body_step,
            imu_rate=imu_rate,
        )
        assert emitted and all(not item.valid for item in emitted)
    finally:
        fusion.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize(
    ("supersede", "body_stalled", "expected_anomaly"),
    [(False, False, False), (True, False, False), (False, True, True), (True, True, True)],
    ids=["healthy-no-gap", "healthy-received-superseded", "stall-no-gap", "stall-superseded"],
)
def test_progress_uses_all_received_source_intervals(
    monkeypatch, supersede, body_stalled, expected_anomaly
):
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
        "pending_wait": 0.2,
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
    emitted = []
    clock = {"now": 100_010_000_000}
    monkeypatch.setattr(fusion, "_scan_topology_ready", lambda: True)
    monkeypatch.setattr(
        fusion,
        "get_clock",
        lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=clock["now"])),
    )
    monkeypatch.setattr(fusion, "_publisher", SimpleNamespace(publish=emitted.append))
    initial_ns = 100_000_000_000
    step_ns = 100_000_000
    step_distance = 0.0021

    def add_wheel_imu(index):
        stamp = (base_ns + index * step_ns) / 1e9
        assert fusion._wheel.add(
            TimedPoseSample(stamp, Pose2D(index * step_distance, 0.0, 0.0), (0.001,) * 3, "odom")
        )
        assert fusion._imu.add(ImuSample(stamp, 0.0, 0.001))
        fusion._wheel_received = fusion._imu_received = time.monotonic()

    def scan(index, previous_local):
        local = 0.0 if body_stalled else math.floor((index * step_distance + 0.01) / 0.02) * 0.02
        message = ScanMotion()
        message.header.frame_id = "base_link"
        message.header.stamp = Time(nanoseconds=base_ns + index * step_ns).to_msg()
        message.interval_start = Time(nanoseconds=base_ns + (index - 1) * step_ns).to_msg()
        message.source_epoch = "stable-independent-scan"
        message.valid = True
        message.reason = "qualified"
        message.response = 0.8
        message.valid_rays = 720
        message.body_delta.x = local - previous_local
        message.local_pose.x = local
        message.covariance[0] = message.covariance[4] = message.covariance[8] = 0.001
        return message, local

    try:
        base_ns = prime_stationary_progress(fusion, clock, initial_ns)
        emitted.clear()
        clock["now"] = base_ns + 1
        moving = Twist()
        moving.linear.x = 0.02
        fusion._on_requested(moving)
        fusion._on_command(moving)
        previous_local = 0.0
        for index in range(1, 101):
            clock["now"] = base_ns + index * step_ns + 10_000_000
            assert fusion._commands.add((base_ns + index * step_ns) / 1e9, Velocity(0.02, 0.0))
            message, previous_local = scan(index, previous_local)
            if supersede and index in (5, 6):
                # The fifth valid source message arrives while its wheel/IMU
                # endpoint is still in flight. The sixth supersedes publication
                # of the fifth; both physical sensor histories then arrive.
                fusion._on_scan(message)
                if index == 5:
                    assert fusion._pending is not None
                else:
                    add_wheel_imu(5)
                    add_wheel_imu(6)
                    fusion._try_pending()
            else:
                add_wheel_imu(index)
                fusion._on_scan(message)
        if supersede:
            assert any(not item.valid and item.reason == "alignment_superseded" for item in emitted)
        qualified = [item for item in emitted if item.valid]
        assert len(qualified) >= 95
        anomalies = [item for item in qualified if item.motion_anomaly]
        if expected_anomaly:
            assert anomalies, "genuine wheel spin must still latch"
        else:
            assert not anomalies, (
                f"healthy independent progress was classified as slip; "
                f"first={anomalies[0].header.stamp.sec}.{anomalies[0].header.stamp.nanosec:09d} "
                f"local={anomalies[0].local_pose.x} "
                f"diagnostic={fusion._slip.first_latch}"
            )
            assert qualified[-1].local_pose.x >= 0.20
    finally:
        fusion.destroy_node()
        rclpy.shutdown()


def test_missing_source_interval_requires_explicit_fresh_stationary_requalification(monkeypatch):
    rclpy = pytest.importorskip("rclpy")
    from geometry_msgs.msg import Twist
    from rclpy.parameter import Parameter
    from rclpy.time import Time
    from senior_motion_msgs.msg import ScanMotion
    from senior_robot.motion_fusion import MotionFusion
    from std_srvs.srv import Trigger

    from senior_control.models import Pose2D, Velocity
    from senior_control.safety.motion_estimator import MotionEstimate, WheelOdom
    from senior_control.safety.motion_runtime import ImuSample, TimedPoseSample

    values = {
        "sensor_max_age": 2.0,
        "alignment_max_gap": 0.3,
        "pending_wait": 0.2,
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
        lambda: SimpleNamespace(warning=lambda value: logs.append((len(emitted), value))),
    )
    step = 100_000_000
    try:
        base = prime_stationary_progress(fusion, clock, 100_000_000_000)
        emitted.clear()

        def deliver(
            index, *, publish_scan=True, zero_streams=True, wheel_x=0.0, body_x=0.0, imu=0.0
        ):
            stamp_ns = base + index * step
            clock["now"] = stamp_ns + 10_000_000
            stamp = stamp_ns / 1e9
            assert fusion._wheel.add(
                TimedPoseSample(stamp, Pose2D(wheel_x, 0, 0), (0.001,) * 3, "odom")
            )
            assert fusion._imu.add(ImuSample(stamp, imu, 0.001))
            fusion._wheel_received = fusion._imu_received = time.monotonic()
            if zero_streams:
                fusion._on_requested(Twist())
                fusion._on_command(Twist())
            if publish_scan:
                message = ScanMotion()
                message.header.frame_id = "base_link"
                message.header.stamp = Time(nanoseconds=stamp_ns).to_msg()
                message.interval_start = Time(nanoseconds=stamp_ns - step).to_msg()
                message.source_epoch = "stable-independent-scan"
                message.valid = True
                message.response = 0.8
                message.valid_rays = 720
                message.body_delta.x = body_x
                message.local_pose.x = wheel_x
                message.covariance[0] = message.covariance[4] = message.covariance[8] = 0.001
                fusion._on_scan(message)

        deliver(1, publish_scan=False)
        deliver(2)
        assert fusion._progress.unqualified and not emitted[-1].valid
        assert emitted[-1].reason == "progress_history_unqualified"
        assert len(logs) == 1 and logs[0][0] == len(emitted)
        prefix = "motion_progress_break "
        assert logs[0][1].startswith(prefix)
        first_break = json.loads(logs[0][1][len(prefix) :])
        assert first_break["reason"] == "source_gap_or_overlap"
        assert first_break["queue"]["last_end_ns"] == base
        assert first_break["details"]["incoming"]["start_ns"] == base + step
        assert first_break["wheel_bounds"][2] > 0
        assert first_break["imu_bounds"][2] > 0
        assert first_break["command_bounds"][2] > 0
        for index in (3, 4, 5):
            deliver(index)
            assert fusion._progress.unqualified and not emitted[-1].valid
        assert len(logs) == 1

        response = fusion._on_progress_requalification(Trigger.Request(), Trigger.Response())
        assert response.success and fusion._progress.unqualified
        deliver(6, zero_streams=False)
        assert fusion._progress.unqualified and not emitted[-1].valid
        deliver(7, wheel_x=0.02, body_x=0.02)
        assert fusion._progress.unqualified and not emitted[-1].valid
        deliver(8, wheel_x=0.02, imu=0.1)
        assert fusion._progress.unqualified and not emitted[-1].valid
        deliver(9, wheel_x=0.02)
        assert fusion._progress.unqualified and not emitted[-1].valid
        for index in (10, 11, 12):
            deliver(index, wheel_x=0.02)
            if fusion._progress_started:
                break
        assert fusion._progress_started and not fusion._progress.unqualified
        assert emitted[-1].valid and not emitted[-1].motion_anomaly
        fusion._progress.invalidate()
        for index in range(30):
            fusion._slip.update(
                Velocity(0.02, 0),
                WheelOdom(Pose2D(0.002, 0, 0), 200.0 + index * 0.1, 0.1),
                MotionEstimate(Pose2D(0, 0, 0), Velocity(0, 0), (0.001,) * 3, 0.8, 0.1),
            )
        assert fusion._slip.latched
        response = fusion._on_progress_requalification(Trigger.Request(), Trigger.Response())
        assert not response.success and fusion._slip.latched and fusion._progress.unqualified
    finally:
        fusion.destroy_node()
        rclpy.shutdown()


def test_fusion_processes_at_most_eight_queued_intervals_without_publishing_old_ones(monkeypatch):
    rclpy = pytest.importorskip("rclpy")
    from rclpy.parameter import Parameter
    from rclpy.time import Time
    from senior_motion_msgs.msg import ScanMotion
    from senior_robot.motion_fusion import MotionFusion

    from senior_control.models import Pose2D, Velocity
    from senior_control.safety.motion_runtime import ImuSample, TimedPoseSample

    values = {
        "sensor_max_age": 2.0,
        "alignment_max_gap": 0.3,
        "pending_wait": 0.2,
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
    emitted = []
    clock = {"now": 100_010_000_000}
    monkeypatch.setattr(fusion, "_scan_topology_ready", lambda: True)
    monkeypatch.setattr(
        fusion,
        "get_clock",
        lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=clock["now"])),
    )
    monkeypatch.setattr(fusion, "_publisher", SimpleNamespace(publish=emitted.append))
    step = 100_000_000
    try:
        base = prime_stationary_progress(fusion, clock, 100_000_000_000)
        emitted.clear()
        initial_count = len(fusion._slip.recent_intervals)
        for index in range(1, 10):
            stamp_ns = base + index * step
            clock["now"] = stamp_ns + 10_000_000
            assert fusion._commands.add(stamp_ns / 1e9, Velocity(0, 0))
            message = ScanMotion()
            message.header.frame_id = "base_link"
            message.header.stamp = Time(nanoseconds=stamp_ns).to_msg()
            message.interval_start = Time(nanoseconds=stamp_ns - step).to_msg()
            message.source_epoch = "stable-independent-scan"
            message.valid = True
            message.response = 0.8
            message.valid_rays = 720
            message.covariance[0] = message.covariance[4] = message.covariance[8] = 0.001
            fusion._on_scan(message)
        assert not any(message.valid for message in emitted)
        for index in range(1, 10):
            stamp = (base + index * step) / 1e9
            assert fusion._wheel.add(TimedPoseSample(stamp, Pose2D(0, 0, 0), (0.001,) * 3, "odom"))
            assert fusion._imu.add(ImuSample(stamp, 0.0, 0.001))
        fusion._wheel_received = fusion._imu_received = time.monotonic()
        fusion._try_pending()
        assert len(fusion._slip.recent_intervals) - initial_count == 8
        assert not any(message.valid for message in emitted)
        assert fusion._progress.peek().end_ns == base + 9 * step
        fusion._try_pending()
        assert emitted[-1].valid and not emitted[-1].motion_anomaly
    finally:
        fusion.destroy_node()
        rclpy.shutdown()


def test_live_harness_captures_first_valid_to_progress_unqualified_transition(tmp_path):
    pytest.importorskip("rclpy")
    from collections import deque

    from motion_anomaly_trace import AnomalyTrace
    from senior_motion_msgs.msg import MotionEvidence
    from test_motion_live import LiveRun

    live = LiveRun.__new__(LiveRun)
    live.clocks = deque(maxlen=10)
    live.motion = deque(maxlen=10)
    live.anomaly_trace = AnomalyTrace()
    live.first_trace_kind = None
    live.seen_qualified_motion = False
    live.progress_break_at = None
    live.progress_break_events = deque(maxlen=128)
    live.motion_progress_unqualified = False
    live.anomaly_path = tmp_path / "anomaly.json"
    live.progress_break_path = tmp_path / "progress-break.json"
    live.metrics = {}

    valid = MotionEvidence()
    valid.valid = True
    live._record_message("motion", live.motion, valid)
    invalid = MotionEvidence()
    invalid.reason = "progress_history_unqualified"
    live._record_message("motion", live.motion, invalid)
    first = live.progress_break_at
    live._record_message("motion", live.motion, invalid)
    assert first is not None and live.progress_break_at == first
    assert len(live.progress_break_events) == 1
    live._save_anomaly_trace()
    capture = json.loads(live.progress_break_path.read_text())
    assert capture["trigger"] == "progress_break"
    assert len(capture["topics"]["motion"]) == 3
    assert capture["topics"]["motion"][0]["sample"]["valid"]
    assert capture["topics"]["motion"][1]["sample"]["reason"] == "progress_history_unqualified"


@pytest.mark.parametrize("value", [0.0, 0.05])
def test_duplicate_applied_command_at_identical_ros_stamp_is_idempotent(monkeypatch, value):
    import copy

    rclpy = pytest.importorskip("rclpy")
    from geometry_msgs.msg import Twist
    from rclpy.parameter import Parameter
    from senior_robot.motion_fusion import MotionFusion

    from senior_control.models import BodyTwist, Pose2D, Velocity
    from senior_control.safety.motion_estimator import MotionEstimate, WheelOdom

    values = dict(
        sensor_max_age=2.0,
        alignment_max_gap=0.3,
        pending_wait=0.2,
        max_variance=0.1,
        min_quality=0.3,
        max_yaw_error=0.1,
        startup_window=0.6,
        wheel_motion_limit=0.004,
        stall_distance=0.04,
        lateral_limit=0.04,
        yaw_limit=0.12,
    )
    rclpy.init()
    fusion = MotionFusion(parameter_overrides=[Parameter(k, value=v) for k, v in values.items()])
    clock = {"now": 2_312_000_000}
    monkeypatch.setattr(
        fusion,
        "get_clock",
        lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=clock["now"])),
    )
    monkeypatch.setattr(fusion, "_try_pending", lambda: None)
    try:
        fusion._progress_started = True
        fusion._progress_requalification.arm(steady=time.monotonic() - 1, ros_ns=2_000_000_000)
        command = Twist()
        command.linear.x = value
        fusion._on_command(command)
        bounds = fusion._commands.diagnostic_bounds()
        state = copy.deepcopy(fusion._progress_requalification.__dict__)
        estimate = MotionEstimate(
            Pose2D(0, 0, 0),
            Velocity(0, 0),
            (0.001,) * 3,
            0.8,
            dt=1.0,
            source="scan_match",
            stamp=1.0,
            degenerate=False,
            aligned=True,
            max_covariance=0.1,
            body_twist=BodyTwist(0, 0, 0),
        )
        assert fusion._slip.update(
            Velocity(0.05, 0), WheelOdom(Pose2D(0.1, 0, 0), 1.0, 1.0), estimate
        ).anomalous
        fusion._stop_acknowledgement.trip()
        fusion._stop_acknowledgement.requested(Velocity(0.0, 0.0))
        fusion._on_command(command)
        assert fusion._commands.diagnostic_bounds() == bounds
        assert fusion._progress_requalification.__dict__ == state
        assert not fusion._stop_acknowledgement.ready
        assert fusion._slip.latched
        assert not fusion._progress.unqualified
        assert fusion._first_progress_break is None
        # A replay cannot extend the original command history's age.
        assert fusion._commands.integrated(2.7, 2.8) is None
    finally:
        fusion.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize("failure", ["different_value", "rollback", "float_collision"])
def test_same_stamp_conflicting_applied_value_and_rollback_still_break_progress(
    monkeypatch, failure
):
    rclpy = pytest.importorskip("rclpy")
    from geometry_msgs.msg import Twist
    from rclpy.parameter import Parameter
    from senior_robot.motion_fusion import MotionFusion

    values = dict(
        sensor_max_age=2.0,
        alignment_max_gap=0.3,
        pending_wait=0.2,
        max_variance=0.1,
        min_quality=0.3,
        max_yaw_error=0.1,
        startup_window=0.6,
        wheel_motion_limit=0.004,
        stall_distance=0.04,
        lateral_limit=0.04,
        yaw_limit=0.12,
    )
    rclpy.init()
    fusion = MotionFusion(parameter_overrides=[Parameter(k, value=v) for k, v in values.items()])
    clock = {"now": 1_700_000_000_000_000_000 if failure == "float_collision" else 2_312_000_000}
    monkeypatch.setattr(
        fusion,
        "get_clock",
        lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=clock["now"])),
    )
    monkeypatch.setattr(fusion, "_try_pending", lambda: None)
    try:
        fusion._progress_started = True
        fusion._on_command(Twist())
        command = Twist()
        if failure == "different_value":
            command.linear.x = 0.05
        elif failure == "rollback":
            clock["now"] -= 1_000_000
        else:
            clock["now"] += 1  # Distinct integer instants can round to the same float seconds.
        fusion._on_command(command)
        assert fusion._progress.unqualified
        assert fusion._first_progress_break["reason"] == "applied_command_timestamp_rejected"
        assert fusion._commands.diagnostic_bounds() == (None, None, 0)
    finally:
        fusion.destroy_node()
        rclpy.shutdown()
