"""Reference qualification uses original host age and explicit world coordinates."""

import importlib.util
import sys
from collections import deque
from pathlib import Path
from types import ModuleType
from types import SimpleNamespace as N

import pytest
from source_time_test_support import capture_time, synthetic_domain, wire_time  # noqa: F401

from senior_control.mapping.simulation import baseline_grid
from senior_control.models import Pose2D
from senior_control.sensors.time_proof import SourceFrontier


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
    clock = N(ros=10.0, steady=100.0)
    monkeypatch.setattr(module, "time", N(monotonic=lambda: clock.steady))

    class RosTime:
        @property
        def nanoseconds(self):
            return round(clock.ros * 1e9)

        def __add__(self, _tolerance):
            return self

        def to_msg(self):
            return N(sec=int(clock.ros))

    sources = {"/ground_truth/odom/timed": b"truth", "/wheel/odom/timed": b"wheel"}
    logs = []
    ref = module.SimulationReference.__new__(module.SimulationReference)
    ref._node = N(get_clock=lambda: N(now=RosTime), get_logger=lambda: N(warning=logs.append))
    ref._grid = baseline_grid(Path("ros2/src/senior_sim/worlds/baseline.sdf"), "original")
    ref._ready = ref._localization_live = True
    ref._pose, ref._pose_at = Pose2D(0, 0, 0), 100.0
    truth = capture_time(100, sequence=1, epoch="truth", tick=0, world="world:0")
    wheel = capture_time(100, sequence=1, epoch="wheel", tick=0, world="world:0")
    ref._truth = module._Observation(ref._pose, 10, 100, b"truth", truth)
    ref._wheel = module._Observation(ref._pose, 10, 100, b"wheel", wheel)
    ref._observation_history = {
        "/ground_truth/odom/timed": deque([ref._truth], maxlen=32),
        "/wheel/odom/timed": deque([ref._wheel], maxlen=32),
    }
    ref._timing_frontiers = {}
    for topic, proof in (("/ground_truth/odom/timed", truth), ("/wheel/odom/timed", wheel)):
        ref._timing_frontiers[topic] = SourceFrontier()
        assert ref._timing_frontiers[topic].accept(proof)
    ref._last_qualification_log = float("-inf")
    ref._unique_map = lambda: True
    ref._source = lambda topic, _name, info=None: (
        sources[topic]
        if info is None or info.get("publisher_gid", sources[topic]) == sources[topic]
        else None
    )
    ref._transform_tolerance = object()
    ref._transform_type = lambda: N(header=N(), transform=N(translation=N(), rotation=N()))
    ref._tf = N(sendTransform=lambda _message: None)
    return N(ref=ref, clock=clock, sources=sources, logs=logs)


def odometry(host, tick, seq, *, frame="world", x=0, label=10.0, world="world:0", producer=None):
    ns = round(label * 1e9)
    data = N(
        header=N(stamp=N(sec=ns // 10**9, nanosec=ns % 10**9), frame_id=frame),
        child_frame_id="base_link",
        pose=N(pose=N(position=N(x=x, y=0), orientation=N(x=0.0, y=0.0, z=0.0, w=1.0))),
    )
    return N(
        timing=wire_time(
            host,
            tick=tick,
            sequence=seq,
            world=world,
            epoch=producer or ("truth" if frame == "world" else "wheel"),
        ),
        data=data,
    )


@pytest.mark.parametrize("label", [5.0, 15.0])
def test_ros_correction_and_ahead_labels_do_not_delay_or_revoke_current_source(reference, label):
    ref, clock = reference.ref, reference.clock
    clock.ros, clock.steady = label, 100.02
    ref._on_truth(odometry(100.01, 0.01, 2, label=label + 1, x=0.01), {})
    ref._tick()
    assert ref.generation == "original" and ref._localization_live
    assert ref._truth.stamp == label + 1 and ref._truth.received == 100.02
    assert ref._pose_at == 100.0 and ref._pose.x == 0.01


def test_host_expiry_and_real_source_change_invalidate_history(reference):
    ref, clock = reference.ref, reference.clock
    clock.ros = 5
    clock.steady = 100.501
    ref._tick()
    assert ref.generation != "original" and not ref._localization_live
    assert ref.pose() is None
    assert all(not history for history in ref._observation_history.values())


def test_publisher_change_still_rejects_cached_pair(reference):
    ref = reference.ref
    reference.sources["/wheel/odom/timed"] = b"replacement"
    ref._tick()
    assert not ref._localization_live and ref.generation != "original"


def test_duplicates_and_old_production_order_cannot_renew_original_time(reference):
    ref, clock = reference.ref, reference.clock
    clock.steady = 100.1
    message = odometry(100.05, 0.05, 2, x=0.05)
    ref._on_truth(message, {})
    clock.steady = 100.2
    message.data.pose.pose.position.x = 99
    ref._on_truth(message, {})
    original = ref._observation_history["/ground_truth/odom/timed"][-1]
    assert original.received == 100.1 and original.timing.end_steady == 100.05
    assert original.pose.x == 0.05
    ref._on_truth(odometry(100.04, 0.04, 3, x=2), {})
    assert ref._observation_history["/ground_truth/odom/timed"][-1] is original


def test_already_queued_expired_input_is_rejected_without_receive_stamp_renewal(reference):
    ref, clock = reference.ref, reference.clock
    clock.steady = 100.6
    ref._on_truth(odometry(100.0, 0, 2), {})
    assert not ref._localization_live
    assert all(not h for h in ref._observation_history.values())


def test_latest_truth_pairs_with_closest_qualified_wheel_using_physical_ticks(reference):
    ref, clock = reference.ref, reference.clock
    clock.steady = 100.12
    ref._on_truth(odometry(100.06, 0.06, 2, x=0.06, label=1000), {})
    ref._on_wheel(odometry(100.04, 0.04, 2, frame="odom", x=0.04, label=3), {})
    ref._on_wheel(odometry(100.12, 0.12, 3, frame="odom", x=0.12, label=2000), {})
    ref._tick()
    assert ref._truth.pose.x == 0.06 and ref._wheel.pose.x == 0.04
    assert ref._pose.x == 0.06 and ref._pose_at == 100.04
    assert ref.generation == "original"


def test_cross_world_pair_is_not_aligned_and_reset_starts_a_new_generation(reference):
    ref, clock = reference.ref, reference.clock
    clock.steady = 100.1
    ref._on_truth(odometry(100.1, 0, 2, world="world:1"), {})
    ref._tick()
    assert ref._pose is None and not ref._localization_live
    assert ref.generation != "original"


def test_producer_restart_is_separate_from_shared_world(reference):
    ref, clock = reference.ref, reference.clock
    clock.steady = 100.1
    ref._on_truth(odometry(100.1, 0.1, 1, producer="truth-next", x=0.1), {})
    ref._tick()
    assert ref._pose.x == 0.1 and ref.generation == "original"
    assert ref._truth.timing.source_epoch == "truth-next"
    assert ref._truth.timing.measurement.same_axis(ref._wheel.timing.measurement)


@pytest.mark.parametrize("failure", ["frame", "orientation", "publisher", "noncanonical-label"])
def test_original_identity_and_payload_validation_is_preserved(reference, failure):
    ref, clock = reference.ref, reference.clock
    clock.steady = 100.1
    message, info = odometry(100.1, 0.1, 2), {}
    if failure == "frame":
        message.data.header.frame_id = "wrong"
    elif failure == "orientation":
        message.data.pose.pose.orientation.w = 0
    elif failure == "publisher":
        info["publisher_gid"] = b"unexpected"
    else:
        message.data.header.stamp.nanosec = 10**9
    ref._on_truth(message, info)
    assert ref._pose is None and ref.generation != "original"


def test_history_is_bounded_and_ages_original_capture_even_at_delayed_delivery(reference):
    ref, clock = reference.ref, reference.clock
    for i in range(1, 40):
        host, tick = 100 + i / 100, i / 100
        clock.steady = host + 0.01
        ref._on_truth(odometry(host, tick, i + 1), {})
        ref._on_wheel(odometry(host, tick, i + 1, frame="odom"), {})
    ref._tick()
    assert ref.generation == "original" and ref._pose_at == pytest.approx(100.39)
    assert all(len(h) == 32 for h in ref._observation_history.values())
    clock.steady = 100.891
    assert ref.pose() is None
