"""Real Fusion callbacks must wait for a slightly early source clock sample."""

import time
from types import SimpleNamespace

import pytest
from motion_progress_test_helpers import prime_stationary_progress


@pytest.fixture
def active_fusion(monkeypatch):
    rclpy = pytest.importorskip("rclpy")
    from rclpy.parameter import Parameter
    from senior_robot.motion_fusion import MotionFusion

    values = {
        "sensor_max_age": 0.8,
        "alignment_max_gap": 0.25,
        "pending_wait": 0.25,
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
    base = prime_stationary_progress(fusion, clock, 100_000_000_000)
    emitted.clear()
    try:
        yield fusion, clock, base, emitted
    finally:
        fusion.destroy_node()
        rclpy.shutdown()


def test_future_wheel_waits_without_refresh_until_original_stamp_is_current(active_fusion):
    from nav_msgs.msg import Odometry
    from rclpy.time import Time

    fusion, clock, base, emitted = active_fusion
    now = base + 10_000_000
    stamp = now + 2_000_000
    clock["now"] = now
    message = Odometry()
    message.header.frame_id = "odom"
    message.child_frame_id = "base_link"
    message.header.stamp = Time(nanoseconds=stamp).to_msg()
    message.pose.pose.orientation.w = 1.0
    message.pose.covariance[0] = message.pose.covariance[7] = message.pose.covariance[35] = 0.001
    previous = fusion._wheel.latest
    previous_received = fusion._wheel_received
    arrived_before = time.monotonic()
    fusion._on_wheel(message)
    arrived_after = time.monotonic()
    assert fusion._progress_started and not fusion._progress.unqualified
    assert fusion._wheel.latest == previous and fusion._wheel_received == previous_received
    assert not emitted
    clock["now"] = stamp - 1
    fusion._try_pending()
    assert fusion._wheel.latest == previous and fusion._wheel_received == previous_received
    clock["now"] = stamp
    fusion._try_pending()
    assert fusion._wheel.latest.stamp == stamp / 1e9
    assert arrived_before <= fusion._wheel_received <= arrived_after
    assert fusion._progress_started and not fusion._progress.unqualified


def test_future_imu_waits_without_refresh_until_original_stamp_is_current(active_fusion):
    from geometry_msgs.msg import TransformStamped
    from rclpy.time import Time
    from sensor_msgs.msg import Imu

    fusion, clock, base, emitted = active_fusion
    now = base + 10_000_000
    stamp = now + 2_000_000
    clock["now"] = now
    transform = TransformStamped()
    transform.transform.rotation.w = 1.0
    fusion._tf = SimpleNamespace(lookup_transform=lambda *_: transform)
    message = Imu()
    message.header.frame_id = "imu_link"
    message.header.stamp = Time(nanoseconds=stamp).to_msg()
    message.angular_velocity_covariance[0] = message.angular_velocity_covariance[8] = 0.001
    previous_bounds = fusion._imu.diagnostic_bounds()
    previous_received = fusion._imu_received
    arrived_before = time.monotonic()
    fusion._on_imu(message)
    arrived_after = time.monotonic()
    assert fusion._progress_started and not fusion._progress.unqualified
    assert (
        fusion._imu.diagnostic_bounds() == previous_bounds
        and fusion._imu_received == previous_received
    )
    assert not emitted
    clock["now"] = stamp - 1
    fusion._try_pending()
    assert fusion._imu.diagnostic_bounds() == previous_bounds
    clock["now"] = stamp
    fusion._try_pending()
    assert fusion._imu.diagnostic_bounds()[1] == stamp / 1e9
    assert arrived_before <= fusion._imu_received <= arrived_after
    assert fusion._progress_started and not fusion._progress.unqualified


def test_future_scan_waits_without_advancing_source_or_publishing(active_fusion):
    from rclpy.parameter import Parameter
    from rclpy.time import Time
    from senior_motion_msgs.msg import ScanMotion
    from senior_robot.safety_node import SafetyGate

    from senior_control.models import Pose2D, Velocity
    from senior_control.safety.motion_runtime import ImuSample, TimedPoseSample

    fusion, clock, base, emitted = active_fusion
    stamp = base + 100_000_000
    clock["now"] = stamp - 2_000_000
    message = ScanMotion()
    message.header.frame_id = "base_link"
    message.header.stamp = Time(nanoseconds=stamp).to_msg()
    message.interval_start = Time(nanoseconds=base).to_msg()
    message.source_epoch = "stable-independent-scan"
    message.valid = True
    message.response = 0.8
    message.valid_rays = 720
    message.covariance[0] = message.covariance[4] = message.covariance[8] = 0.001
    before = fusion._progress.diagnostic_state()
    received_before = time.monotonic()
    fusion._on_scan(message)
    received_after = time.monotonic()
    assert fusion._progress_started and not fusion._progress.unqualified
    assert fusion._progress.diagnostic_state() == before and not emitted
    clock["now"] = stamp - 1
    fusion._try_pending()
    assert fusion._progress.diagnostic_state() == before and not emitted
    clock["now"] = stamp
    assert fusion._wheel.add(TimedPoseSample(stamp / 1e9, Pose2D(0, 0, 0), (0.001,) * 3, "odom"))
    assert fusion._imu.add(ImuSample(stamp / 1e9, 0.0, 0.001))
    assert fusion._commands.add(stamp / 1e9, Velocity(0, 0))
    fusion._wheel_received = fusion._imu_received = time.monotonic()
    fusion._try_pending()
    assert emitted and emitted[-1].valid and not emitted[-1].motion_anomaly
    assert emitted[-1].header.stamp == message.header.stamp
    assert emitted[-1].interval_start == message.interval_start
    assert received_before <= emitted[-1].scan_received_steady <= received_after
    assert Time.from_msg(emitted[-1].scan_received_ros).nanoseconds == stamp - 2_000_000
    gate = SafetyGate(
        parameter_overrides=[
            Parameter("observation_max_age", value=0.8),
            Parameter("measurement_max_age", value=0.8),
            Parameter("motion_min_quality", value=0.3),
            Parameter("motion_max_variance", value=0.1),
            Parameter("motion_max_interval", value=0.3),
        ]
    )
    try:
        gate.get_clock = fusion.get_clock
        gate._accept_evidence(emitted[-1])
        assert gate._freshness.measured_received == emitted[-1].scan_received_steady
        assert gate._reference_admission[2] <= emitted[-1].scan_received_steady
    finally:
        gate.destroy_node()


def _scan(start_ns, end_ns, *, epoch="stable-independent-scan", valid=True):
    from rclpy.time import Time
    from senior_motion_msgs.msg import ScanMotion

    message = ScanMotion()
    message.header.frame_id = "base_link"
    message.header.stamp = Time(nanoseconds=end_ns).to_msg()
    message.interval_start = Time(nanoseconds=start_ns).to_msg()
    message.source_epoch = epoch
    message.valid = valid
    message.response = 0.8
    message.valid_rays = 720
    message.covariance[0] = message.covariance[4] = message.covariance[8] = 0.001
    return message


def _add_stationary_end(fusion, stamp_ns):
    from senior_control.models import Pose2D, Velocity
    from senior_control.safety.motion_runtime import ImuSample, TimedPoseSample

    assert fusion._wheel.add(TimedPoseSample(stamp_ns / 1e9, Pose2D(0, 0, 0), (0.001,) * 3, "odom"))
    assert fusion._imu.add(ImuSample(stamp_ns / 1e9, 0.0, 0.001))
    assert fusion._commands.add(stamp_ns / 1e9, Velocity(0, 0))
    fusion._wheel_received = fusion._imu_received = time.monotonic()


def test_future_scan_fifo_preserves_exact_adjacent_intervals(active_fusion):
    fusion, clock, base, emitted = active_fusion
    first, second = base + 100_000_000, base + 200_000_000
    clock["now"] = first - 2_000_000
    before = fusion._progress.diagnostic_state()
    fusion._on_scan(_scan(base, first))
    fusion._on_scan(_scan(first, second))
    assert fusion._progress.diagnostic_state() == before
    assert fusion._deferred_scan.peek().stamp_ns == first
    assert fusion._deferred_scan.latest().stamp_ns == second
    assert not emitted
    for stamp in (first, second):
        clock["now"] = stamp
        _add_stationary_end(fusion, stamp)
        fusion._try_pending()
        assert emitted[-1].valid
        assert fusion._progress.diagnostic_state()["last_end_ns"] == stamp
    assert fusion._deferred_scan.peek() is None


def test_future_scan_gap_fails_closed_without_crossing_the_head(active_fusion):
    fusion, clock, base, emitted = active_fusion
    first = base + 100_000_000
    clock["now"] = first - 2_000_000
    fusion._on_scan(_scan(base, first))
    fusion._on_scan(_scan(first + 1, first + 100_000_000))
    assert fusion._progress.unqualified and not fusion._progress_started
    assert fusion._deferred_scan.peek() is None
    assert emitted and not emitted[-1].valid


def test_future_epoch_change_revokes_old_but_retired_traffic_cannot_cancel_new(active_fusion):
    fusion, clock, base, emitted = active_fusion
    stamp = base + 100_000_000
    clock["now"] = stamp - 2_000_000
    fusion._on_scan(_scan(base, stamp, epoch="new-epoch"))
    assert fusion._scan_epochs.is_retired("stable-independent-scan")
    assert fusion._scan_epochs.active is None
    assert fusion._progress.unqualified and emitted and not emitted[-1].valid
    assert fusion._deferred_scan.peek().value.source_epoch == "new-epoch"
    count = len(emitted)
    for valid in (False, True):
        fusion._on_scan(_scan(base, stamp + 1, valid=valid))
        assert len(emitted) == count
        assert fusion._deferred_scan.peek().value.source_epoch == "new-epoch"
    clock["now"] = stamp
    fusion._try_pending()
    assert fusion._scan_epochs.active == "new-epoch"
    assert fusion._progress.unqualified and not emitted[-1].valid


def test_future_invalid_new_epoch_revokes_now_but_does_not_admit_epoch_early(active_fusion):
    fusion, clock, base, emitted = active_fusion
    stamp = base + 100_000_000
    clock["now"] = stamp - 2_000_000
    fusion._on_scan(_scan(base, stamp, epoch="new-epoch", valid=False))
    assert fusion._scan_epochs.is_retired("stable-independent-scan")
    assert fusion._scan_epochs.active is None
    assert fusion._progress.unqualified and emitted and not emitted[-1].valid
    assert fusion._deferred_scan.peek().value.source_epoch == "new-epoch"
    count = len(emitted)
    for valid in (True, False):
        fusion._on_scan(_scan(base, stamp + 1, valid=valid))
        assert len(emitted) == count
        assert fusion._deferred_scan.peek().value.source_epoch == "new-epoch"
    clock["now"] = stamp
    fusion._try_pending()
    assert fusion._scan_epochs.active == "new-epoch"
    assert fusion._progress.unqualified and not emitted[-1].valid


def test_expired_future_epoch_failure_cannot_be_replayed_to_revive(active_fusion, monkeypatch):
    from senior_robot import motion_fusion as module

    fusion, clock, base, emitted = active_fusion
    stamp = base + 100_000_000
    clock["now"] = stamp - 2_000_000
    steady = {"now": time.monotonic()}
    monkeypatch.setattr(module, "time", SimpleNamespace(monotonic=lambda: steady["now"]))
    fusion._on_scan(_scan(base, stamp, epoch="new-epoch", valid=False))
    assert fusion._deferred_scan.peek().value.source_epoch == "new-epoch"
    steady["now"] += fusion._wait + 0.001
    fusion._try_pending()
    assert fusion._scan_epochs.is_retired("new-epoch")
    assert fusion._deferred_scan.peek() is None
    assert fusion._progress.unqualified and not emitted[-1].valid
    count = len(emitted)
    for valid in (False, True):
        fusion._on_scan(_scan(base, stamp, epoch="new-epoch", valid=valid))
        assert fusion._deferred_scan.peek() is None and len(emitted) == count
    fusion._on_scan(_scan(base, stamp, epoch="successor"))
    assert fusion._deferred_scan.peek().value.source_epoch == "successor"
    clock["now"] = stamp
    fusion._try_pending()
    assert fusion._scan_epochs.active == "successor"
    assert fusion._progress.unqualified and not emitted[-1].valid


@pytest.mark.parametrize("kind", ["invalid", "empty_epoch", "wrong_frame"])
def test_future_scan_failure_immediately_revokes_existing_progress(active_fusion, kind):
    fusion, clock, base, emitted = active_fusion
    stamp = base + 100_000_000
    clock["now"] = stamp - 2_000_000
    message = _scan(base, stamp, valid=kind != "invalid")
    if kind == "empty_epoch":
        message.source_epoch = ""
    elif kind == "wrong_frame":
        message.header.frame_id = "wrong-frame"
    fusion._on_scan(message)
    assert fusion._progress.unqualified and not fusion._progress_started
    assert fusion._deferred_scan.peek() is None
    assert emitted and not emitted[-1].valid


def test_future_wheel_expires_on_timer_with_frozen_ros_clock(active_fusion, monkeypatch):
    from nav_msgs.msg import Odometry
    from rclpy.time import Time
    from senior_robot import motion_fusion as module

    fusion, clock, base, emitted = active_fusion
    clock["now"] = base + 10_000_000
    steady = {"now": time.monotonic()}
    monkeypatch.setattr(module, "time", SimpleNamespace(monotonic=lambda: steady["now"]))
    message = Odometry()
    message.header.frame_id, message.child_frame_id = "odom", "base_link"
    message.header.stamp = Time(nanoseconds=clock["now"] + 2_000_000).to_msg()
    message.pose.pose.orientation.w = 1.0
    message.pose.covariance[0] = message.pose.covariance[7] = message.pose.covariance[35] = 0.001
    fusion._on_wheel(message)
    assert fusion._deferred_wheel.peek() is not None
    steady["now"] += fusion._wait + 0.001
    fusion._try_pending()  # Same callback as the 20 ms steady timer, without new data.
    assert fusion._progress.unqualified and not fusion._progress_started
    assert fusion._deferred_wheel.peek() is None
    assert emitted and not emitted[-1].valid


def test_real_steady_timer_expires_future_sample_without_any_new_input(active_fusion):
    rclpy = pytest.importorskip("rclpy")

    fusion, clock, base, emitted = active_fusion
    stamp = base + 100_000_000
    clock["now"] = stamp - 2_000_000
    fusion._on_scan(_scan(base, stamp))
    assert fusion._deferred_scan.peek() is not None
    deadline = time.monotonic() + 1.5
    while time.monotonic() < deadline and not fusion._progress.unqualified:
        rclpy.spin_once(fusion, timeout_sec=0.03)
    assert fusion._progress.unqualified and not fusion._progress_started
    assert fusion._deferred_scan.peek() is None
    assert emitted and not emitted[-1].valid


@pytest.mark.parametrize("stream", ["wheel", "imu"])
def test_future_sensor_with_bad_frame_immediately_revokes(active_fusion, stream):
    from nav_msgs.msg import Odometry
    from rclpy.time import Time
    from sensor_msgs.msg import Imu

    fusion, clock, base, emitted = active_fusion
    clock["now"] = base + 10_000_000
    if stream == "wheel":
        message = Odometry()
        message.child_frame_id = "base_link"
        message.pose.pose.orientation.w = 1.0
        message.pose.covariance[0] = message.pose.covariance[7] = message.pose.covariance[35] = (
            0.001
        )
        callback = fusion._on_wheel
    else:
        message = Imu()
        message.angular_velocity_covariance[0] = message.angular_velocity_covariance[8] = 0.001
        callback = fusion._on_imu
    message.header.frame_id = "wrong-frame"
    message.header.stamp = Time(nanoseconds=clock["now"] + 2_000_000).to_msg()
    callback(message)
    assert fusion._progress.unqualified and not fusion._progress_started
    assert emitted and not emitted[-1].valid


def test_future_wheel_capacity_is_bounded_and_exhaustion_revokes(active_fusion):
    from nav_msgs.msg import Odometry
    from rclpy.time import Time

    fusion, clock, base, emitted = active_fusion
    clock["now"] = base + 10_000_000
    message = Odometry()
    message.header.frame_id, message.child_frame_id = "odom", "base_link"
    message.pose.pose.orientation.w = 1.0
    message.pose.covariance[0] = message.pose.covariance[7] = message.pose.covariance[35] = 0.001
    for index in range(17):
        message.header.stamp = Time(nanoseconds=clock["now"] + (index + 1) * 1_000_000).to_msg()
        fusion._on_wheel(message)
    assert fusion._progress.unqualified and not fusion._progress_started
    assert fusion._deferred_wheel.peek() is None
    assert emitted and not emitted[-1].valid


def test_deferred_sources_share_eight_item_callback_budget_fairly(active_fusion):
    from geometry_msgs.msg import TransformStamped

    from senior_control.models import Pose2D
    from senior_control.safety.motion_runtime import DeferredSample, ImuSample, TimedPoseSample

    fusion, clock, base, emitted = active_fusion
    clock["now"] = base + 100_000_000
    received = time.monotonic()
    transform = TransformStamped()
    transform.transform.rotation.w = 1.0
    fusion._tf = SimpleNamespace(lookup_transform=lambda *_: transform)
    for index in range(12):
        stamp = base + (index + 1) * 1_000_000
        wheel = TimedPoseSample(stamp / 1e9, Pose2D(0, 0, 0), (0.001,) * 3, "odom")
        imu = ImuSample(stamp / 1e9, 0.0, 0.001)
        assert fusion._deferred_wheel.offer(DeferredSample(stamp, clock["now"], received, wheel))
        assert fusion._deferred_imu.offer(DeferredSample(stamp, clock["now"], received, imu))
    wheel_before = fusion._wheel.diagnostic_bounds()[2]
    imu_before = fusion._imu.diagnostic_bounds()[2]
    fusion._try_pending()
    wheel_count = fusion._wheel.diagnostic_bounds()[2] - wheel_before
    imu_count = fusion._imu.diagnostic_bounds()[2] - imu_before
    assert wheel_count + imu_count == 8
    assert wheel_count > 0 and imu_count > 0
    assert fusion._deferred_wheel.peek() is not None and fusion._deferred_imu.peek() is not None
    assert fusion._progress_started and not emitted


def test_mixed_future_drain_does_not_reenter_progress_processing(active_fusion, monkeypatch):
    from geometry_msgs.msg import TransformStamped

    from senior_control.models import Pose2D, Velocity
    from senior_control.safety.motion_runtime import DeferredSample, ImuSample, TimedPoseSample

    fusion, clock, base, _emitted = active_fusion
    clock["now"] = base + 200_000_000
    received = time.monotonic()
    transform = TransformStamped()
    transform.transform.rotation.w = 1.0
    fusion._tf = SimpleNamespace(lookup_transform=lambda *_: transform)
    for index in range(1, 10):
        stamp = base + index * 10_000_000
        wheel = TimedPoseSample(stamp / 1e9, Pose2D(0, 0, 0), (0.001,) * 3, "odom")
        imu = ImuSample(stamp / 1e9, 0.0, 0.001)
        scan = _scan(stamp - 10_000_000, stamp)
        assert fusion._deferred_wheel.offer(DeferredSample(stamp, clock["now"], received, wheel))
        assert fusion._deferred_imu.offer(DeferredSample(stamp, clock["now"], received, imu))
        assert fusion._deferred_scan.offer(DeferredSample(stamp, clock["now"], received, scan))
        assert fusion._commands.add(stamp / 1e9, Velocity(0, 0))

    counts = {"wheel": 0, "imu": 0, "scan": 0, "calls": 0, "depth": 0, "max_depth": 0}
    for stream in ("wheel", "imu", "scan"):
        method = getattr(fusion, f"_admit_{stream}")

        def tracked(*args, _stream=stream, _method=method):
            counts[_stream] += 1
            return _method(*args)

        monkeypatch.setattr(fusion, f"_admit_{stream}", tracked)
    run = fusion._try_pending

    def tracked_run():
        counts["calls"] += 1
        counts["depth"] += 1
        counts["max_depth"] = max(counts["max_depth"], counts["depth"])
        try:
            run()
        finally:
            counts["depth"] -= 1

    monkeypatch.setattr(fusion, "_try_pending", tracked_run)
    before = len(fusion._slip.recent_intervals)
    fusion._try_pending()
    assert sum(counts[stream] for stream in ("wheel", "imu", "scan")) == 8
    assert all(counts[stream] > 0 for stream in ("wheel", "imu", "scan"))
    assert counts["calls"] == counts["max_depth"] == 1
    assert len(fusion._slip.recent_intervals) - before <= 8


def test_future_queue_true_clock_rollback_revokes(active_fusion):
    fusion, clock, base, emitted = active_fusion
    stamp = base + 100_000_000
    clock["now"] = stamp - 2_000_000
    fusion._on_scan(_scan(base, stamp))
    clock["now"] -= 1
    fusion._try_pending()
    assert fusion._progress.unqualified and not fusion._progress_started
    assert fusion._deferred_scan.peek() is None
    assert emitted and not emitted[-1].valid


def test_changed_scan_publisher_revokes_even_while_scan_is_future(active_fusion, monkeypatch):
    from senior_robot.motion_fusion import MotionFusion

    fusion, clock, base, emitted = active_fusion
    fusion._scan_topology.observe([("scan_motion_node", "/", bytes([1]) * 16)])
    monkeypatch.setattr(
        fusion,
        "_scan_topology_ready",
        MotionFusion._scan_topology_ready.__get__(fusion, MotionFusion),
    )
    monkeypatch.setattr(
        fusion,
        "get_publishers_info_by_topic",
        lambda _topic: [
            SimpleNamespace(
                node_name="scan_motion_node", node_namespace="/", endpoint_gid=bytes([2]) * 16
            )
        ],
    )
    stamp = base + 100_000_000
    clock["now"] = stamp - 2_000_000
    fusion._on_scan(_scan(base, stamp))
    assert fusion._progress.unqualified and not fusion._progress_started
    assert fusion._scan_epochs.is_retired("stable-independent-scan")
    assert fusion._deferred_scan.peek() is None
    assert emitted and not emitted[-1].valid
