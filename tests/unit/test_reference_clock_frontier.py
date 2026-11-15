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
    core = module.MotionCoreOutput(node, SimulationMotionProfile(), ingress_factory=None)
    core._on_reference(N(data=json.dumps(CASE["old"])), {"publisher_gid": b"fusion"})
    yield N(core=core, node=node, clock=clock)
    core.close()


def test_future_raw_clock_cannot_destroy_other_live_sources(witness):
    from senior_control.motion.estimator import MotionObservation, Origin

    _, body, _ = witness.core._inputs(witness.clock.steady)
    assert body is not None
    before = body.velocity
    with pytest.raises(ValueError):
        witness.core.sensors.estimator.ingest(
            MotionObservation(
                "future",
                (0.9, None, None),
                7.0,
                (Origin("future", "future", 1, 7.0, 6.2, witness.clock.steady),),
            )
        )
    assert witness.core._inputs(witness.clock.steady)[1].velocity == before


def test_original_age_expiry_uses_paused_ros_and_advancing_steady(witness):
    assert witness.core._inputs(witness.clock.steady)[1] is not None
    witness.clock.steady += 1
    assert witness.core._inputs(witness.clock.steady)[1] is None


def test_local_ros_rollback_clears_estimates_before_new_epoch(witness):
    witness.core._inputs(witness.clock.steady)
    generation = witness.core.sensors.latest.generation
    witness.clock.ros -= 1
    assert witness.core._inputs(witness.clock.steady)[1] is None
    assert witness.core.sensors.latest.generation == generation + 1


def test_retired_raw_epoch_cannot_return(witness):
    from dataclasses import replace

    e = witness.core.sensors.estimator
    old = e._frontiers["wheel"]
    root = replace(old.roots[0], epoch="replacement", sequence=1)
    new = replace(old, roots=(root,))
    assert e.ingest(new)
    assert not e.ingest(old)
    assert e._frontiers["wheel"] == new


def test_retired_transport_boot_cannot_return(witness):
    packet = json.loads(json.dumps(CASE["old"]))
    packet["boot_id"] = "replacement"
    packet["sequence"] = 1
    witness.core._on_reference(N(data=json.dumps(packet)), {"publisher_gid": b"fusion"})
    assert witness.core._reference_boot == "replacement"
    witness.core._on_reference(N(data=json.dumps(CASE["old"])), {"publisher_gid": b"fusion"})
    assert witness.core._reference_boot == "replacement"


def test_republishing_cached_raw_rows_does_not_refresh_original_deadline(witness):
    _, before, _ = witness.core._inputs(witness.clock.steady)
    packet = json.loads(json.dumps(CASE["old"]))
    packet["sequence"] += 100
    packet["sampled_ros"] += 0.1
    witness.clock.ros += 0.1
    witness.core._on_reference(N(data=json.dumps(packet)), {"publisher_gid": b"fusion"})
    _, after, _ = witness.core._inputs(witness.clock.steady)
    assert after.deadline_steady <= before.deadline_steady
    assert after.sequence == before.sequence
