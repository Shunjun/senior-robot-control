"""Reference observations retain their first receipt while awaiting ROS time."""

import importlib.util
import json
import sys
from collections import deque
from pathlib import Path
from types import ModuleType, SimpleNamespace

import pytest

from senior_control.mapping.simulation import baseline_grid
from senior_control.models import Pose2D
from senior_control.safety.motion_runtime import DeferredSourceQueue


@pytest.fixture
def reference(monkeypatch):
    lifecycle = ModuleType("senior_robot.motion_lifecycle")
    lifecycle.with_dds_info = lambda callback: callback
    monkeypatch.setitem(sys.modules, lifecycle.__name__, lifecycle)
    path = Path("ros2/src/senior_robot/senior_robot/simulation_reference.py")
    spec = importlib.util.spec_from_file_location("reference_admission_under_test", path)
    module = importlib.util.module_from_spec(spec)
    monkeypatch.setitem(sys.modules, spec.name, module)
    spec.loader.exec_module(module)
    clock = SimpleNamespace(ros=10.0, steady=100.0)
    monkeypatch.setattr(module, "time", SimpleNamespace(monotonic=lambda: clock.steady))

    class RosTime:
        @property
        def nanoseconds(self):
            return round(clock.ros * 1e9)

        def __add__(self, _tolerance):
            return self

        def to_msg(self):
            return SimpleNamespace(sec=int(clock.ros))

    sources = {"/ground_truth/odom": b"truth", "/wheel/odom": b"wheel"}
    logs = []
    node = SimpleNamespace(
        get_clock=lambda: SimpleNamespace(now=RosTime),
        get_logger=lambda: SimpleNamespace(warning=logs.append),
    )
    ref = module.SimulationReference.__new__(module.SimulationReference)
    ref._node = node
    ref._grid = baseline_grid(Path("ros2/src/senior_sim/worlds/baseline.sdf"), "original")
    ref._ready = ref._localization_live = True
    ref._last_ros = clock.ros
    ref._pose = Pose2D(0, 0, 0)
    ref._pose_at = clock.steady
    ref._truth = module._Observation(ref._pose, clock.ros, clock.steady, b"truth")
    ref._wheel = module._Observation(ref._pose, clock.ros, clock.steady, b"wheel")
    ref._observation_history = {
        "/ground_truth/odom": deque([ref._truth], maxlen=32),
        "/wheel/odom": deque([ref._wheel], maxlen=32),
    }
    ref._deferred_observations = {
        "/ground_truth/odom": DeferredSourceQueue(max_pending=32),
        "/wheel/odom": DeferredSourceQueue(max_pending=32),
    }
    ref._last_qualification_log = float("-inf")
    ref._future_observations = 0
    ref._future_admitted = 0
    ref._unique_map = lambda: True
    ref._source = lambda topic, _name, info=None: (
        sources[topic]
        if info is None or info.get("publisher_gid", sources[topic]) == sources[topic]
        else None
    )
    ref._transform_tolerance = object()
    ref._transform_type = lambda: SimpleNamespace(
        header=SimpleNamespace(),
        transform=SimpleNamespace(translation=SimpleNamespace(), rotation=SimpleNamespace()),
    )
    ref._tf = SimpleNamespace(sendTransform=lambda _message: None)
    return SimpleNamespace(ref=ref, clock=clock, sources=sources, logs=logs)


def odometry(stamp, *, frame="world", x=0.0):
    ns = round(stamp * 1e9)
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000),
            frame_id=frame,
        ),
        child_frame_id="base_link",
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=x, y=0.0),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            )
        ),
    )


@pytest.mark.parametrize("stream", ["truth", "wheel"])
def test_future_observation_preserves_generation_then_uses_original_receipt(reference, stream):
    ref, clock = reference.ref, reference.clock
    previous = getattr(ref, "_" + stream)
    clock.ros, clock.steady = 10.01, 100.01
    message = odometry(10.02, frame="world" if stream == "truth" else "odom", x=0.01)
    getattr(ref, "_on_" + stream)(message, {})
    assert getattr(ref, "_" + stream) is previous
    ref._tick()
    assert ref.generation == "original" and ref._localization_live
    message.pose.pose.position.x = 99.0
    clock.ros, clock.steady = 10.02, 100.03
    ref._tick()
    topic = "/ground_truth/odom" if stream == "truth" else "/wheel/odom"
    admitted = ref._observation_history[topic][-1]
    assert admitted.stamp == 10.02 and admitted.received == 100.01
    assert admitted.pose.x == 0.01
    assert ref.generation == "original" and ref._localization_live


@pytest.mark.parametrize(
    ("ros", "steady", "source_changed"),
    [(10.01, 100.52, False), (10.6, 100.03, False), (9.9, 100.03, False), (10.02, 100.03, True)],
    ids=["steady-expired", "ros-expired", "clock-rollback", "publisher-changed"],
)
def test_queued_observation_never_survives_expiry_rollback_or_identity_change(
    reference, ros, steady, source_changed
):
    ref, clock = reference.ref, reference.clock
    clock.ros, clock.steady = 10.01, 100.01
    ref._on_truth(odometry(10.02), {})
    if source_changed:
        reference.sources["/ground_truth/odom"] = b"replacement"
    clock.ros, clock.steady = ros, steady
    ref._tick()
    assert ref.generation != "original" and not ref._localization_live
    assert ref._truth is None and ref._wheel is None
    assert ref._deferred_observations["/ground_truth/odom"].peek() is None
    assert all(not history for history in ref._observation_history.values())
    assert len(reference.logs) == 1
    details = json.loads(reference.logs[0].split(" ", 1)[1])
    assert details["reason"] and "truth_ros_age" in details and "skew" in details
    ref._tick()
    assert len(reference.logs) == 1


def test_duplicate_future_sample_cannot_renew_its_receipt(reference):
    ref, clock = reference.ref, reference.clock
    clock.ros, clock.steady = 10.01, 100.01
    ref._on_truth(odometry(10.02), {})
    clock.steady = 100.2
    ref._on_truth(odometry(10.02), {})
    clock.ros, clock.steady = 10.02, 100.3
    ref._tick()
    assert ref._truth.received == 100.01


def test_future_queue_overflow_fails_closed(reference):
    ref = reference.ref
    for index in range(33):
        ref._on_truth(odometry(10.001 + index * 0.001), {})
    assert ref.generation != "original" and not ref._localization_live
    assert ref._deferred_observations["/ground_truth/odom"].peek() is None
    assert all(not history for history in ref._observation_history.values())


def test_future_admission_does_not_relax_pair_skew(reference):
    ref, clock = reference.ref, reference.clock
    ref._on_truth(odometry(10.2), {})
    clock.ros, clock.steady = 10.2, 100.2
    ref._tick()
    assert ref.generation != "original" and not ref._localization_live


def test_future_admission_expires_from_receipt_ros_even_when_measurement_is_fresh(reference):
    ref, clock = reference.ref, reference.clock
    ref._on_truth(odometry(10.6), {})
    clock.ros, clock.steady = 10.6, 100.01
    ref._tick()
    assert ref._truth is None and ref.generation != "original"
    details = json.loads(reference.logs[0].split(" ", 1)[1])
    assert details["reason"] == "future_expired"
    assert details["pending"]["/ground_truth/odom"]["received_ros_ns"] == 10_000_000_000


@pytest.mark.parametrize("failure", ["frame", "orientation", "publisher", "noncanonical-time"])
def test_future_payload_must_pass_original_identity_and_shape_checks(reference, failure):
    ref = reference.ref
    message, info = odometry(10.02), {}
    if failure == "frame":
        message.header.frame_id = "wrong"
    elif failure == "orientation":
        message.pose.pose.orientation.w = 0.0
    elif failure == "publisher":
        info["publisher_gid"] = b"unexpected"
    else:
        message.header.stamp.nanosec = 1_000_000_000
    ref._on_truth(message, info)
    assert ref.generation != "original" and ref._truth is None
    assert ref._deferred_observations["/ground_truth/odom"].peek() is None


def test_ready_future_samples_drain_in_source_order(reference):
    ref, clock = reference.ref, reference.clock
    ref._on_truth(odometry(10.01, x=0.01), {})
    clock.steady = 100.01
    ref._on_truth(odometry(10.02, x=0.02), {})
    clock.ros, clock.steady = 10.02, 100.02
    ref._tick()
    assert ref._truth.pose.x == 0.02 and ref._truth.received == 100.01
    assert ref._future_admitted == 2 and ref.generation == "original"


def test_current_publisher_change_still_invalidates_without_pending_samples(reference):
    ref = reference.ref
    reference.sources["/wheel/odom"] = b"replacement"
    ref._tick()
    assert ref.generation != "original" and not ref._localization_live


def _wheel_ahead_of_delayed_truth(runtime, *, future_truth=False):
    ref, clock = runtime.ref, runtime.clock
    clock.ros, clock.steady = 10.164, 100.328
    ref._on_wheel(odometry(10.16, frame="odom", x=0.16), {})
    if future_truth:
        ref._on_truth(odometry(10.18, x=0.18), {})
    ref._tick()


def test_delayed_stream_keeps_original_coherent_pair_while_future_truth_waits(reference):
    ref, clock = reference.ref, reference.clock
    original_truth, original_wheel = ref._truth, ref._wheel
    _wheel_ahead_of_delayed_truth(reference, future_truth=True)
    assert ref.generation == "original" and ref._localization_live
    assert ref._truth is original_truth and ref._wheel is original_wheel
    assert ref._pose_at == 100.0
    clock.ros, clock.steady = 10.18, 100.35
    ref._tick()
    assert ref.generation == "original" and ref._localization_live
    assert ref._truth.stamp == 10.18 and ref._truth.received == 100.328
    assert ref._wheel.stamp == 10.16 and ref._wheel.received == 100.328
    assert ref._pose_at == 100.328


def test_pair_uses_latest_qualified_truth_and_closest_wheel_not_latest_wheel(reference):
    ref, clock = reference.ref, reference.clock
    clock.ros, clock.steady = 10.12, 100.12
    ref._on_truth(odometry(10.06, x=0.06), {})
    ref._on_wheel(odometry(10.04, frame="odom", x=0.04), {})
    ref._on_wheel(odometry(10.12, frame="odom", x=0.12), {})
    ref._tick()
    assert ref._truth.stamp == 10.06 and ref._wheel.stamp == 10.04
    assert ref._pose.x == 0.06 and ref.generation == "original"


@pytest.mark.parametrize("expired_clock", ["ros", "steady"])
def test_coherent_history_pair_cannot_extend_either_original_age_limit(reference, expired_clock):
    ref, clock = reference.ref, reference.clock
    _wheel_ahead_of_delayed_truth(reference)
    assert ref.generation == "original" and ref._pose_at == 100.0
    setattr(clock, expired_clock, 10.501 if expired_clock == "ros" else 100.501)
    ref._tick()
    assert ref.generation != "original" and ref._pose is None
    assert all(not history for history in ref._observation_history.values())


def test_historical_pair_does_not_move_the_single_stream_order_frontier_backwards(reference):
    ref = reference.ref
    _wheel_ahead_of_delayed_truth(reference)
    assert ref.generation == "original" and ref._wheel.stamp == 10.0
    ref._on_wheel(odometry(10.08, frame="odom"), {})
    assert ref.generation != "original" and not ref._localization_live
    assert not ref._observation_history["/ground_truth/odom"]
    assert all(item.stamp != 10.16 for item in ref._observation_history["/wheel/odom"])


def test_duplicate_latest_sample_does_not_renew_history_receipt(reference):
    ref, clock = reference.ref, reference.clock
    _wheel_ahead_of_delayed_truth(reference)
    assert ref.generation == "original"
    clock.steady = 100.4
    ref._on_wheel(odometry(10.16, frame="odom", x=99.0), {})
    wheel_history = ref._observation_history["/wheel/odom"]
    assert len(wheel_history) == 2
    assert wheel_history[-1].received == 100.328 and wheel_history[-1].pose.x == 0.16
    ref._tick()
    assert ref._pose_at == 100.0 and ref.generation == "original"


@pytest.mark.parametrize("failure", ["publisher", "rollback", "invalid-frame"])
def test_history_is_cleared_immediately_on_source_clock_or_payload_invalidation(reference, failure):
    ref, clock = reference.ref, reference.clock
    if failure == "publisher":
        reference.sources["/wheel/odom"] = b"new-wheel"
        ref._tick()
    elif failure == "rollback":
        clock.ros = 9.99
        ref._tick()
    else:
        ref._on_wheel(odometry(10.01, frame="wrong-frame"), {})
    assert ref.generation != "original" and not ref._localization_live
    assert all(not history for history in ref._observation_history.values())


def test_accepted_history_is_bounded_without_resetting_receipts(reference):
    ref, clock = reference.ref, reference.clock
    for index in range(1, 40):
        clock.ros, clock.steady = 10 + index / 100, 100 + index / 100
        ref._on_truth(odometry(clock.ros, x=index / 100), {})
        ref._on_wheel(odometry(clock.ros, frame="odom", x=index / 100), {})
    ref._tick()
    assert ref.generation == "original" and ref._pose_at == 100.39
    for history in ref._observation_history.values():
        assert len(history) == 32 and history[0].received == 100.08


@pytest.mark.parametrize("expired_clock", ["ros", "steady"])
def test_pose_read_cannot_replay_cached_pair_past_either_age_limit(reference, expired_clock):
    ref, clock = reference.ref, reference.clock
    _wheel_ahead_of_delayed_truth(reference)
    assert ref.pose() is not None
    setattr(clock, expired_clock, 10.501 if expired_clock == "ros" else 100.501)
    assert ref.pose() is None
    assert ref.pose_reply() == {"ok": False, "error": "unavailable"}


@pytest.mark.parametrize("expired_clock", ["ros", "steady"])
def test_expired_closest_wheel_cannot_be_paired_with_a_fresh_truth(reference, expired_clock):
    ref, clock = reference.ref, reference.clock
    clock.ros, clock.steady = 10.2, 100.2
    ref._on_truth(odometry(10.01), {})
    ref._on_wheel(odometry(10.2, frame="odom"), {})
    ref._tick()
    assert ref._truth.stamp == 10.01 and ref._wheel.stamp == 10.0
    assert ref.generation == "original"
    setattr(clock, expired_clock, 10.501 if expired_clock == "ros" else 100.501)
    ref._tick()
    assert ref.generation != "original" and ref._pose is None


def test_callbacks_do_not_mix_new_observation_stamps_into_the_cached_pose_pair(reference):
    ref, clock = reference.ref, reference.clock
    original_truth, original_wheel, original_pose = ref._truth, ref._wheel, ref._pose
    clock.ros, clock.steady = 10.12, 100.12
    ref._on_truth(odometry(10.12, x=0.12), {})
    ref._on_wheel(odometry(10.1, frame="odom", x=0.1), {})
    assert ref._truth is original_truth and ref._wheel is original_wheel
    assert ref.pose() is original_pose and ref._pose_at == 100.0
    ref._tick()
    assert ref._truth.stamp == 10.12 and ref._wheel.stamp == 10.1
    assert ref._pose.x == 0.12 and ref._pose_at == 100.12
