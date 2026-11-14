"""RED proposal witness: actual core and captured snapshots, controlled local clock."""

import json
import sys
from pathlib import Path
from types import ModuleType
from types import SimpleNamespace as N

import pytest
from senior_robot import motion_core_output as module
from senior_robot.simulation_motion_profile import SimulationMotionProfile
from test_motion_reference_backlog import Node

CASE = json.loads(Path("tests/fixtures/reference_clock_frontier.json").read_text())


@pytest.fixture
def witness(monkeypatch):
    message = ModuleType("std_msgs.msg")
    message.String = lambda: N(data="")
    lifecycle = ModuleType("senior_robot.motion_lifecycle")
    lifecycle.with_dds_info = lambda f: f
    monkeypatch.setitem(sys.modules, message.__name__, message)
    monkeypatch.setitem(sys.modules, lifecycle.__name__, lifecycle)
    clock = N(steady=CASE["tick"]["sampled_steady"], ros=CASE["tick"]["ros_before"])
    monkeypatch.setattr(module, "time", N(monotonic=lambda: clock.steady))
    node = Node(CASE, clock)
    node._scans[0].stamp = CASE["scan"]["stamp"]["sec"] + CASE["scan"]["stamp"]["nanosec"] / 1e9
    core = module.MotionCoreOutput(node, SimulationMotionProfile())
    core._on_reference(N(data=json.dumps(CASE["old"])), {"publisher_gid": b"fusion"})
    yield N(core=core, node=node, clock=clock)
    core.close()


def test_future_snapshot_must_not_replace_still_legal_current_reference(witness):
    old_ref, old_body, _ = witness.core._inputs(witness.clock.steady)
    assert old_ref["end_ros"] == 6.1 and old_body is not None
    # Full captured scan still associates to this original pose/reference.
    perception, _ = witness.core._observation(witness.clock.steady)
    assert perception.revision == CASE["scan"]["sequence"]
    witness.node.sub.ready.append(
        (N(data=json.dumps(CASE["future"])), {"publisher_gid": b"fusion"})
    )
    witness.core._consume_ready_reference()
    _, body, _ = witness.core._inputs(witness.clock.steady)
    assert body is not None, "RED: ref227 age=.082 was legal; ref229 end6.2 is future at local6.182"
    assert witness.core._reference["poses"] == CASE["old"]["poses"]
    assert witness.core._reference["sources"] == CASE["old"]["sources"]
    assert body.stamp_steady == old_body.stamp_steady
    assert body.deadline_steady == old_body.deadline_steady


def test_local_clock_promotes_pending_without_new_packet_and_retains_deadline(witness):
    witness.core._on_reference(N(data=json.dumps(CASE["future"])), {"publisher_gid": b"fusion"})
    assert witness.core._reference["sequence"] == 227
    assert witness.core._reference_sequence == 229
    witness.clock.ros = 6.278
    body = witness.core._inputs(witness.clock.steady)[1]
    assert body is not None
    laser = next(s for s in CASE["future"]["sources"] if s["source"] == "laser")
    assert body.stamp_steady == laser["sample"]["received_steady"]
    assert body.deadline_steady == laser["valid_until_steady"]


def test_old_reference_cannot_outlive_original_prediction_horizon(witness):
    witness.clock.ros = 6.251
    assert witness.core._inputs(witness.clock.steady)[1] is None


def offer(witness, packet):
    witness.core._on_reference(N(data=json.dumps(packet)), {"publisher_gid": b"fusion"})


def packet_copy(name="future", sequence=230):
    import copy

    packet = copy.deepcopy(CASE[name])
    packet["sequence"] = sequence
    return packet


def change_epoch(packet, source, epoch):
    for row in packet["sources"]:
        if row["source"] == source:
            row["sample"]["epoch"] = epoch
    for row in packet["reference"]["source_health"]:
        if row["source"] == source:
            row["epoch"] = epoch


def test_negative_update_clears_pending_but_same_epoch_recovery_is_allowed(witness):
    offer(witness, CASE["future"])
    negative = packet_copy()
    negative["sources"] = [row for row in negative["sources"] if row["source"] != "laser"]
    negative["reference"]["source_health"] = [
        row for row in negative["reference"]["source_health"] if row["source"] != "laser"
    ]
    negative["reference"]["average"] = None
    negative["reference"]["mode"] = "wheel_imu_limited"
    offer(witness, negative)
    assert witness.core._pending_reference is None
    assert witness.core._inputs(witness.clock.steady)[1] is None
    offer(witness, packet_copy("old", 231))
    assert witness.core._inputs(witness.clock.steady)[1] is not None


def test_source_epoch_change_and_retired_replay_cannot_reactivate_old_body(witness):
    offer(witness, CASE["future"])
    new = packet_copy()
    change_epoch(new, "laser", "new-laser-epoch")
    offer(witness, new)
    assert witness.core._pending_reference is None
    assert witness.core._inputs(witness.clock.steady)[1] is None
    offer(witness, packet_copy("old", 231))
    assert witness.core._reference_sequence == 231
    with pytest.raises(ValueError):
        witness.core._inputs(witness.clock.steady)
    recovered = packet_copy("old", 232)
    change_epoch(recovered, "laser", "new-laser-epoch")
    offer(witness, recovered)
    assert witness.core._inputs(witness.clock.steady)[1].epoch == "new-laser-epoch"


def test_boot_change_clears_pending_and_retired_boot_cannot_return(witness):
    offer(witness, CASE["future"])
    new = packet_copy("old", 1)
    new["boot_id"] = "new-producer"
    for row in new["sources"]:
        change_epoch(new, row["source"], "new-" + row["sample"]["epoch"])
    offer(witness, new)
    assert witness.core._reference["boot_id"] == "new-producer"
    assert witness.core._pending_reference is None
    offer(witness, packet_copy("old", 9999))
    assert witness.core._reference_boot == "new-producer"
    assert witness.core._reference_sequence == 1
    assert witness.core._inputs(witness.clock.steady)[1] is not None


def test_out_of_order_packet_cannot_replace_pending_and_local_clock_promotes_it(witness):
    offer(witness, CASE["future"])
    offer(witness, packet_copy("old", 228))
    assert witness.core._reference_sequence == 229
    assert witness.core._pending_reference["sequence"] == 229
    witness.clock.ros = 6.278
    witness.core._inputs(witness.clock.steady)
    assert witness.core._reference["sequence"] == 229
    assert witness.core._pending_reference is None


def test_paused_ros_clock_does_not_extend_old_source_or_pending_deadline(witness):
    offer(witness, CASE["future"])
    laser = next(row for row in CASE["old"]["sources"] if row["source"] == "laser")
    witness.clock.steady = laser["valid_until_steady"] + 0.001
    assert witness.core._inputs(witness.clock.steady)[1] is None
    heartbeat = packet_copy(sequence=230)
    heartbeat["sampled_steady"] = witness.clock.steady
    offer(witness, heartbeat)
    assert witness.core._inputs(witness.clock.steady)[1] is None


def test_producer_reset_can_arrive_before_local_clock_without_permanent_retirement(witness):
    from senior_robot.motion_observation_adapter import NativeObservations

    from senior_control.motion.observations import Source

    profile = SimulationMotionProfile()
    producer = NativeObservations(profile.admission(), profile.temporal(), profile.fusion())
    laser_template = json.loads(Path("tests/fixtures/laser_publication_phase.json").read_text())[
        "old_laser"
    ]
    steady = witness.clock.steady

    def feed(end):
        nonlocal steady
        steady += 0.1
        ns = round(end * 1e9)
        stamp = N(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)
        wheel = N(
            child_frame_id="base_link",
            header=N(frame_id="odom", stamp=stamp),
            twist=N(twist=N(linear=N(x=0), angular=N(z=0))),
        )
        imu = N(header=N(frame_id="base_link", stamp=stamp), angular_velocity=N(z=0))
        message = json.loads(json.dumps(laser_template), object_hook=lambda fields: N(**fields))
        message.header.stamp = stamp
        first = round((end - 0.1) * 1e9)
        message.interval_start = N(sec=first // 1_000_000_000, nanosec=first % 1_000_000_000)
        producer.wheel(wheel, b"wheel", end + 0.02, steady)
        producer.imu(imu, b"imu", end + 0.02, steady)
        producer.laser(message, b"laser", end + 0.02, steady)

    for i in range(60, 68):
        feed(i / 10)
    before = producer.identity(Source.LASER, b"laser", laser_template["source_epoch"], 6.72)[0]
    # Real producer handles the ROS rollback and recovers its original five/.5s admission.
    for i in range(50, 58):
        feed(i / 10)
    snapshot = producer.snapshot(5.72, steady)
    assert snapshot["reference"]["average"] is not None
    epoch = next(row["sample"]["epoch"] for row in snapshot["sources"] if row["source"] == "laser")
    assert epoch != before
    # Retiring this already-post-reset identity on the later local callback would deadlock it.
    from senior_control.safety.motion_runtime import SourceEpochFence

    erroneous = SourceEpochFence(max_retired=profile.admission().epoch_history_limit)
    assert erroneous.accept(epoch)
    erroneous.retire_active()
    assert not erroneous.accept(epoch)
    # It reaches this consumer before this consumer's own /clock reset callback.
    witness.clock.steady = steady
    offer(witness, snapshot)
    watermark = witness.core._reference_sequence
    witness.clock.ros = 5.72
    witness.clock.steady += 0.001
    with pytest.raises(ValueError):
        witness.core._inputs(witness.clock.steady)
    assert witness.core._reference_sequence == watermark
    assert witness.core._pending_reference is None
    offer(witness, snapshot)  # The already-admitted pre-reset packet cannot reactivate the cache.
    with pytest.raises(ValueError):
        witness.core._inputs(witness.clock.steady)
    assert witness.core._reference_sequence == watermark
    # Producer does NOT emit another clock epoch just because another node caught up.
    feed(5.8)
    snapshot = producer.snapshot(5.82, steady)
    assert (
        next(row["sample"]["epoch"] for row in snapshot["sources"] if row["source"] == "laser")
        == epoch
    )
    witness.clock.steady = steady
    witness.clock.ros = 5.82
    offer(witness, snapshot)
    assert witness.core._inputs(witness.clock.steady)[1] is not None


def test_without_legal_current_packet_future_reference_still_cannot_supply_body(witness):
    witness.core._clear_reference_cache()
    offer(witness, CASE["future"])
    assert witness.core._pending_reference is None
    assert witness.core._inputs(witness.clock.steady)[1] is None
