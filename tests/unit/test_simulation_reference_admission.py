"""Reference observations retain their first receipt while awaiting ROS time."""

import importlib.util
import json
import sys
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
    admitted = getattr(ref, "_" + stream)
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
