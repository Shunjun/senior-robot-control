"""Captured R6 reference216: the 150ms closed boundary is exact in ROS ns."""

import copy
import json
import sys
from pathlib import Path
from types import ModuleType
from types import SimpleNamespace as N

import pytest
from senior_robot import motion_core_output as module
from senior_robot.simulation_motion_profile import SimulationMotionProfile
from test_motion_reference_backlog import Node

CASE = json.loads(Path("tests/fixtures/reference_nanosecond_boundary.json").read_text())


@pytest.fixture
def replay(monkeypatch):
    message = ModuleType("std_msgs.msg")
    message.String = lambda: N(data="")
    lifecycle = ModuleType("senior_robot.motion_lifecycle")
    lifecycle.with_dds_info = lambda callback: callback
    monkeypatch.setitem(sys.modules, message.__name__, message)
    monkeypatch.setitem(sys.modules, lifecycle.__name__, lifecycle)
    clock = N(steady=CASE["tick"]["sampled_steady"], ros=5.15, ns=5_150_000_000)
    monkeypatch.setattr(module, "time", N(monotonic=lambda: clock.steady))
    node = Node({"scan": CASE["scan"]["body"]}, clock)
    node.get_clock = lambda: N(now=lambda: N(nanoseconds=clock.ns))
    scan = CASE["scan"]["body"]
    node._scans[0].stamp = scan["stamp"]["sec"] + scan["stamp"]["nanosec"] / 1e9
    core = module.MotionCoreOutput(node, SimulationMotionProfile())
    core._on_reference(N(data=json.dumps(CASE["packet"])), {"publisher_gid": b"fusion"})
    core._startup_motion_unconfirmed = False
    core.controller._last_now = clock.steady - 0.05
    yield N(core=core, clock=clock, node=node)
    core.close()


@pytest.mark.parametrize(
    ("ros_ns", "allowed"),
    [(5_150_000_000, True), (5_150_000_001, False), (4_999_999_999, False)],
)
def test_captured_snapshot_body_and_selection_share_exact_closed_window(replay, ros_ns, allowed):
    replay.clock.ns = ros_ns
    # Isolate timestamp qualification from the independent clock-rollback barrier.
    replay.core._reference_clock = None
    usable = replay.core._reference_usable(replay.core._reference, replay.clock.steady, ros_ns)
    body = replay.core._inputs(replay.clock.steady)[1]
    assert (body is not None) is allowed
    assert usable is allowed
    if body is not None:
        laser = next(s for s in CASE["packet"]["sources"] if s["source"] == "laser")
        assert body.stamp_steady == laser["sample"]["received_steady"]
        assert body.deadline_steady == laser["valid_until_steady"]


def test_captured_tick_keeps_body_at_150ms_instead_of_latching_unknown_body_fault(replay):
    replay.core.tick()
    state = json.loads(replay.node.messages[-1].data)
    assert state["body_reference"] is not None
    assert state["fault_revision"] == 0


def test_future_update_keeps_boundary_legal_snapshot_until_original_window_ends(replay):
    future = copy.deepcopy(CASE["packet"])
    future["sequence"] = 217
    future["reference"]["end_ros"] = 5.2
    replay.core._on_reference(N(data=json.dumps(future)), {"publisher_gid": b"fusion"})
    assert replay.core._reference["sequence"] == 216
    assert replay.core._pending_reference["sequence"] == 217
    assert replay.core._inputs(replay.clock.steady)[1] is not None
    replay.clock.ns += 1
    assert replay.core._inputs(replay.clock.steady)[1] is None
