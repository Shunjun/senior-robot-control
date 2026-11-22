"""Standard pinned localization retains raw age, source and map generations.

Migrates the generic protections from simulation-reference admission; world/tick
pairing and truth-based permission are retired with that production reference.
"""

import importlib.util
import sys
from pathlib import Path
from types import ModuleType
from types import SimpleNamespace as N

import pytest

from vehicle_core.sample_time import map_sample
from vehicle_core.values import Pose2D


@pytest.fixture
def reference(monkeypatch):
    lifecycle = ModuleType("senior_sensor_ingress.ros_lifecycle")
    lifecycle.with_dds_info = lambda callback: callback
    monkeypatch.setitem(sys.modules, lifecycle.__name__, lifecycle)
    receipt = ModuleType("senior_sensor_ingress.native_receipt")
    # Recording boundary for B's mandatory metadata accessor; no time remapping.
    receipt.received_sample = lambda info: info["sample_time"]
    monkeypatch.setitem(sys.modules, receipt.__name__, receipt)
    path = Path("ros2/src/senior_localization/senior_localization/hardware_reference.py")
    spec = importlib.util.spec_from_file_location("standard_reference_under_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    clock = N(ns=100_000_000_000)
    monkeypatch.setattr(module, "time", N(monotonic_ns=lambda: clock.ns))
    ref = module.HardwareReference.__new__(module.HardwareReference)
    ref.config = N(max_age_s=0.5, max_position_variance=0.2, max_yaw_variance=0.2)
    ref._grid = N(map_ref="map", grid_id="generation-a")
    ref._map_serial, ref._pose_map_serial = 1, -1
    ref._pose = ref._pose_time = ref._pose_identity = ref._pose_gid = None
    ref._pose_at = float("-inf")
    ref._pose_stamp_ns = -1
    ref._pose_sequence = 0
    ref._retired_pose_sources = set()
    ref._pose_topic = "/amcl_pose"
    ref.config.pose_publisher = "amcl"
    sources = {"gid": b"pose-a"}
    ref._source = lambda _topic, _name, info=None: (
        sources["gid"] if info is None else info["publisher_gid"]
    )
    ref.grid = lambda: ref._grid

    def message(
        *,
        ns=10_000_000_000,
        received_ns=None,
        receipt_sequence=1,
        receipt_generation=1,
        clock_generation=1,
        gid=None,
        x=1.0,
    ):
        received_ns = ns if received_ns is None else received_ns
        sample = map_sample(
            stamp_ns=ns,
            received_ros_ns=received_ns,
            received_steady_ns=clock.ns,
            clock_generation=clock_generation,
        )
        covariance = [0.0] * 36
        covariance[0] = covariance[7] = covariance[35] = 0.1
        msg = N(
            header=N(stamp=N(sec=ns // 10**9, nanosec=ns % 10**9), frame_id="map"),
            pose=N(
                pose=N(position=N(x=x, y=0.0, z=0.0), orientation=N(x=0, y=0, z=0, w=1)),
                covariance=covariance,
            ),
        )
        return msg, {
            "sample_time": sample,
            "publisher_gid": gid or sources["gid"],
            "receipt_generation": receipt_generation,
            "receipt_sequence": receipt_sequence,
        }

    return N(ref=ref, clock=clock, sources=sources, message=message)


def test_original_age_duplicate_and_queued_delivery_cannot_renew_pose(reference):
    r = reference
    message, info = r.message(received_ns=10_200_000_000)
    r.ref._on_pose(message, info)
    original = r.ref.pose()
    assert original == Pose2D(1, 0, 0)
    r.clock.ns += 100_000_000
    message.pose.pose.position.x = 99
    r.ref._on_pose(message, {**info, "receipt_sequence": 2})
    assert r.ref._pose_sequence == 2
    assert r.ref._pose_time is info["sample_time"]
    assert r.ref.pose() is original
    assert r.ref._pose_at == pytest.approx(99.8)
    r.clock.ns += 200_000_000
    assert r.ref.pose() is None  # Exactly the original 500ms deadline.
    r.ref._on_pose(message, {**info, "receipt_sequence": 3})
    assert r.ref._pose_sequence == 3
    assert r.ref._pose_time is info["sample_time"]
    assert r.ref._pose_at + r.ref.config.max_age_s == pytest.approx(100.3)
    assert r.ref.pose() is None


def test_increasing_native_sequence_with_older_stamp_withdraws_without_renewal(reference):
    r = reference
    original = r.message(received_ns=10_200_000_000)
    r.ref._on_pose(*original)
    r.clock.ns += 100_000_000
    r.ref._on_pose(
        *r.message(
            ns=9_900_000_000,
            received_ns=10_200_000_000,
            receipt_sequence=2,
            x=99,
        )
    )
    assert r.ref._pose_sequence == 2
    assert r.ref._pose is None and r.ref.pose() is None
    assert r.ref._pose_time is original[1]["sample_time"]
    assert r.ref._pose_at == pytest.approx(99.8)
    assert r.ref._pose_stamp_ns == 10_000_000_000


def test_unseen_expired_first_delivery_is_not_stored_and_fresh_sample_recovers(reference):
    r = reference
    original = r.message()
    r.ref._on_pose(*original)
    assert r.ref.pose() == Pose2D(1, 0, 0)
    r.clock.ns += 100_000_000
    queued = r.message(
        ns=10_100_000_000,
        received_ns=10_700_000_000,
        receipt_sequence=2,
        x=99,
    )
    r.ref._on_pose(*queued)
    assert r.ref._pose_sequence == 2
    assert r.ref._pose_stamp_ns == 10_100_000_000
    assert r.ref._pose is None  # Check admission, not only pose()'s independent TTL.
    assert r.ref._pose_time is original[1]["sample_time"]
    assert r.ref._pose_at == pytest.approx(100.0)
    r.clock.ns += 100_000_000
    fresh = r.message(ns=10_800_000_000, receipt_sequence=3, x=2)
    r.ref._on_pose(*fresh)
    assert r.ref.pose() == Pose2D(2, 0, 0)
    assert r.ref._pose_time is fresh[1]["sample_time"]
    assert r.ref._pose_at == pytest.approx(100.2)


def test_source_replacement_retires_queued_old_writer_without_restamping(reference):
    r = reference
    old = r.message()
    r.ref._on_pose(*old)
    r.sources["gid"] = b"pose-b"
    assert r.ref.pose() is None
    new = r.message(ns=10_100_000_000, receipt_generation=2, x=2)
    r.ref._on_pose(*new)
    replacement = r.ref.pose()
    r.ref._on_pose(*old)
    assert r.ref.pose() is replacement and replacement.x == 2
    assert r.ref._pose_time is new[1]["sample_time"]


def test_clock_segment_and_map_change_cannot_reuse_old_task_pose(reference):
    r = reference
    old = r.message()
    r.ref._on_pose(*old)
    new = r.message(ns=1_000_000_000, clock_generation=2, receipt_sequence=2, x=2)
    r.ref._on_pose(*new)
    assert r.ref.pose().x == 2
    assert r.ref._pose_identity == (b"pose-a", 1, 2)
    r.ref._on_pose(old[0], {**old[1], "receipt_sequence": 3})
    assert r.ref.pose().x == 2
    r.ref._invalidate_map()
    assert not r.ref.qualified("map", "generation-a")
    r.ref._on_pose(*new)  # A cached pose cannot qualify a replacement map.
    assert r.ref.pose() is None


@pytest.mark.parametrize("failure", ["frame", "quaternion", "stamp", "covariance", "position"])
def test_standard_pose_payload_validation_preserved(reference, failure):
    r = reference
    message, info = r.message()
    if failure == "frame":
        message.header.frame_id = "odom"
    elif failure == "quaternion":
        message.pose.pose.orientation.w = 0
    elif failure == "stamp":
        message.header.stamp.nanosec = 10**9
    elif failure == "covariance":
        message.pose.covariance[0] = float("nan")
    else:
        message.pose.pose.position.x = float("nan")
    r.ref._on_pose(message, info)
    assert r.ref.pose() is None


def test_standard_reference_pose_reply_satisfies_existing_home_ipc_schema(reference):
    from vehicle_manager.agent.pose_reply import PoseReply

    from vehicle_core.values import MapRef

    r = reference
    r.ref._grid.map_ref = MapRef("map", 1)
    r.ref.config.anchor_id = "home-anchor"
    r.ref.config.anchor_pose = Pose2D(0, 0, 0)
    r.ref._on_pose(*r.message(received_ns=10_200_000_000))
    reply = PoseReply.model_validate(r.ref.pose_reply())
    assert reply.v == 1 and reply.map_id == "map"
    assert reply.map_generation == "generation-a"
    assert reply.sampled_at == pytest.approx(99.8)
