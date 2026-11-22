"""RED proposal witness: actual core and captured snapshots, controlled local clock."""

import json
import sys
from pathlib import Path
from types import ModuleType
from types import SimpleNamespace as N

import pytest
import senior_motion_control.motion_core_output as module
from control_process_test_support import (
    capture_inputs,
    generic_motion_profile,
    local_resources,
    make_core,
    read_inputs,
)
from sensor_time_test_support import sample_time
from test_motion_reference_backlog import Node

CASE = json.loads(Path("tests/fixtures/reference_clock_frontier.json").read_text())


@pytest.fixture
def witness(monkeypatch):
    message = ModuleType("std_msgs.msg")
    message.String = lambda: N(data="")
    lifecycle = ModuleType("senior_sensor_ingress.ros_lifecycle")
    lifecycle.with_dds_info = lambda f: f
    monkeypatch.setitem(sys.modules, message.__name__, message)
    monkeypatch.setitem(sys.modules, lifecycle.__name__, lifecycle)
    clock = N(steady=CASE["tick"]["sampled_steady"], ros=CASE["tick"]["ros_before"])
    monkeypatch.setattr(module, "time", N(monotonic=lambda: clock.steady))
    node = Node(CASE, clock)
    node._scans[0].stamp = CASE["scan"]["stamp"]["sec"] + CASE["scan"]["stamp"]["nanosec"] / 1e9
    local_resources(monkeypatch)
    core = make_core(monkeypatch, node, generic_motion_profile())
    capture_inputs(core, N(data=json.dumps(CASE["old"])))
    yield N(core=core, node=node, clock=clock)
    core.close()


def test_future_raw_clock_cannot_destroy_other_live_sources(witness):
    from senior_motion_estimation.estimator import MotionObservation, Origin

    _, body, _ = read_inputs(witness.core, witness.clock.steady)
    assert body is not None
    before = body.velocity
    with pytest.raises(ValueError):
        witness.core.node._reception_test.sensors.estimator.ingest(
            MotionObservation(
                "future",
                (0.9, None, None),
                7.0,
                (
                    Origin(
                        "future",
                        "future",
                        1,
                        sample_time(
                            stamp_ns=round((7.0) * 1e9),
                            received_ros_ns=round((6.2) * 1e9),
                            received_steady_ns=round((witness.clock.steady) * 1e9),
                        ),
                    ),
                ),
                measured_steady=witness.clock.steady,
                stamp_ns=round((7.0) * 1e9),
            )
        )
    assert read_inputs(witness.core, witness.clock.steady)[1].velocity == before


def test_original_age_expiry_uses_paused_ros_and_advancing_steady(witness):
    assert read_inputs(witness.core, witness.clock.steady)[1] is not None
    witness.clock.steady += 1
    assert read_inputs(witness.core, witness.clock.steady)[1] is None


def test_local_ros_rollback_revokes_estimates_and_advances_generation(witness):
    read_inputs(witness.core, witness.clock.steady)
    generation = witness.core.sensors.latest.generation
    witness.clock.ros -= 1
    assert read_inputs(witness.core, witness.clock.steady)[1] is None
    assert witness.core.sensors.latest.generation == generation + 1


def test_retired_raw_epoch_cannot_return(witness):
    from dataclasses import replace

    e = witness.core.node._reception_test.sensors.estimator
    old = e._frontiers["wheel"]
    root = replace(old.roots[0], epoch="replacement", sequence=1)
    new = replace(old, roots=(root,), epoch="replacement", sequence=1)
    assert e.ingest(new)
    assert not e.ingest(old)
    assert e._frontiers["wheel"] == new


def test_control_snapshot_take_is_monotonic_and_never_replays_consumed_version(witness):
    receiver = witness.core.node._reception_test
    version = receiver.channel.version
    state = witness.core.sensors.latest
    receiver.channel.update(witness.node, witness.core.sensors)
    assert receiver.channel.version == version
    assert witness.core.sensors.latest is state
    assert not hasattr(witness.core, "_on_reference")


def test_republishing_cached_raw_rows_does_not_refresh_original_deadline(witness):
    _, before, _ = read_inputs(witness.core, witness.clock.steady)
    packet = json.loads(json.dumps(CASE["old"]))
    packet["sequence"] += 100
    packet["sampled_ros"] += 0.1
    witness.clock.ros += 0.1
    capture_inputs(witness.core, N(data=json.dumps(packet)))
    _, after, _ = read_inputs(witness.core, witness.clock.steady)
    assert after.deadline_steady <= before.deadline_steady
    assert after.sequence == before.sequence
