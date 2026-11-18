"""Captured R6 reference216: the 150ms closed boundary is exact in ROS ns."""

import json
import sys
from pathlib import Path
from types import ModuleType
from types import SimpleNamespace as N

import pytest
from control_process_test_support import (
    capture_inputs,
    generic_motion_profile,
    local_resources,
    make_core,
    read_inputs,
)
from senior_robot import motion_core_output as module
from sensor_time_test_support import sample_time
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
    local_resources(monkeypatch)
    core = make_core(monkeypatch, node, generic_motion_profile())
    capture_inputs(core, N(data=json.dumps(CASE["packet"])))
    core._startup_motion_unconfirmed = False
    core.controller._last_now = clock.steady - 0.05
    yield N(core=core, clock=clock, node=node)
    core.close()


@pytest.mark.parametrize(
    ("age_ns", "allowed"),
    [(150_000_000, True), (300_000_000, True), (999_999_999, True), (1_000_000_000, False)],
)
def test_raw_original_nanosecond_boundary_is_shared_one_second(replay, age_ns, allowed):
    from senior_control.motion.estimator import MotionEstimation, MotionObservation, Origin

    replay.core.node._reception_test.sensors.estimator = MotionEstimation()
    replay.core.node._reception_test.sensors.estimator.ingest(
        MotionObservation(
            "laser",
            (0.0, None, 0.0),
            5.0,
            (
                Origin(
                    "laser",
                    "boundary",
                    1,
                    sample_time(
                        stamp_ns=round((5.0) * 1e9),
                        received_ros_ns=round((5.0) * 1e9),
                        received_steady_ns=round((replay.clock.steady) * 1e9),
                    ),
                ),
            ),
            independent=(True, True, True),
            measured_steady=replay.clock.steady,
            stamp_ns=round((5.0) * 1e9),
        )
    )
    original = replay.clock.steady
    replay.clock.steady += age_ns / 1e9
    replay.clock.ns = 5_000_000_000 + age_ns
    body = read_inputs(replay.core, replay.clock.steady)[1]
    assert (body is not None) is allowed
    if body is not None:
        assert body.deadline_steady <= original + 1


def test_captured_raw_rows_remain_usable_beyond_old_150ms_cutoff(replay):
    replay.clock.ns = 5_200_000_000
    _, body, _ = read_inputs(replay.core, replay.clock.steady)
    assert body is not None
    assert replay.core.sensors.independent is not None
