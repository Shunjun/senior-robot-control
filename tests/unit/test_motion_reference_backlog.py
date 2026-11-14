"""Real core + captured full scan; explicitly scheduled DDS-ready snapshots.

The real run proves consumption of seq86, not when seq87/88 reached this
subscriber. This regression isolates the avoidable timer-before-subscription
case, without inferring transport latency from a different bag subscriber.
"""

import copy
import json
import sys
from pathlib import Path
from types import ModuleType
from types import SimpleNamespace as N

import pytest
from motion_reference_test_support import ReadySubscription
from senior_robot import motion_core_output as module
from senior_robot.simulation_motion_profile import SimulationMotionProfile

from senior_control.config import DeviceProfile
from senior_control.models import Velocity


class Node:
    def __init__(self, case, clock):
        self.clock = clock
        self.messages = []
        self.output = []
        self._depth_policy = "laser_only"
        scan = case["scan"]
        self._scans = [N(message=N(**scan), stamp=1.4, received=scan["ingress_steady"])]

    def create_publisher(self, *_args):
        return N(publish=self.messages.append)

    def create_subscription(self, message_type, _topic, callback, _depth):
        self.sub = ReadySubscription(message_type, callback)
        return self.sub

    def get_publishers_info_by_topic(self, _topic):
        return [N(node_name="motion_fusion", node_namespace="/", endpoint_gid=b"fusion")]

    def get_clock(self):
        return N(now=lambda: N(nanoseconds=round(self.clock.ros * 1e9)))

    def get_parameter(self, name):
        return N(value="base_link" if name == "body_frame" else "route-sim")

    def _scan_topology_ready(self):
        return True

    def _profile(self):
        return DeviceProfile(self_mask_calibrated=True)

    def _emit_motion_core(self, velocity, reason):
        self.output.append((velocity, reason))


@pytest.fixture
def replay(monkeypatch):
    message = ModuleType("std_msgs.msg")
    message.String = lambda: N(data="")
    lifecycle = ModuleType("senior_robot.motion_lifecycle")
    lifecycle.with_dds_info = lambda callback: callback
    monkeypatch.setitem(sys.modules, message.__name__, message)
    monkeypatch.setitem(sys.modules, lifecycle.__name__, lifecycle)
    case = json.loads(Path("tests/fixtures/motion_reference_backlog.json").read_text())
    old = case["packets"][0]
    clock = N(steady=old["sampled_steady"], ros=old["sampled_ros"])
    monkeypatch.setattr(module, "time", N(monotonic=lambda: clock.steady))
    node = Node(case, clock)
    core = module.MotionCoreOutput(node, SimulationMotionProfile())
    node.sub.callback(N(data=json.dumps(old)), {"publisher_gid": b"fusion"})
    # Restore the captured one-way phase checkpoint; never invent body speed.
    core._startup_motion_unconfirmed = case["tick"]["startup_unconfirmed"]
    core.controller._last_now = case["previous_tick"]["sampled_steady"]
    clock.steady = case["tick"]["sampled_steady"]
    clock.ros = case["tick"]["projection_clocks"][0]["ros_now"]
    yield N(core=core, node=node, clock=clock, case=case)
    core.close()


def test_ready_latest_snapshot_is_consumed_before_control_tick(replay):
    for packet in replay.case["packets"][1:]:
        replay.node.sub.ready.append((N(data=json.dumps(packet)), {"publisher_gid": b"fusion"}))
    replay.core.tick()
    assert replay.core._reference_sequence == 88
    state = json.loads(replay.node.messages[-1].data)
    assert state["body_reference"] is not None
    assert state["fault_revision"] == 0
    assert replay.node.output[-1][0] == Velocity(0, 0)
    # Original admission owner's time is retained; consuming never refreshes it.
    laser = next(s for s in replay.case["packets"][-1]["sources"] if s["source"] == "laser")
    assert state["body_reference"]["computed_steady"] == laser["sample"]["received_steady"]
    assert state["body_reference"]["valid_until_steady"] == laser["valid_until_steady"]


def test_missing_or_wrong_source_snapshot_does_not_relax_prediction_window(replay):
    packet = copy.deepcopy(replay.case["packets"][-1])
    replay.node.sub.ready.append((N(data=json.dumps(packet)), {"publisher_gid": b"other"}))
    replay.core.tick()
    state = json.loads(replay.node.messages[-1].data)
    assert replay.core._reference_sequence == 86
    assert state["body_reference"] is None
    assert state["mode"] == "execution_fault" and state["fault_revision"] == 1


def test_queued_sequence_cannot_roll_back_the_admitted_frontier(replay):
    for packet in reversed(replay.case["packets"][1:]):
        replay.node.sub.ready.append((N(data=json.dumps(packet)), {"publisher_gid": b"fusion"}))
    replay.core.tick()
    assert replay.core._reference_sequence == 88
    assert json.loads(replay.node.messages[-1].data)["body_reference"] is not None


def test_fresh_packet_does_not_renew_expired_original_laser_evidence(replay):
    packet = copy.deepcopy(replay.case["packets"][-1])
    for source in packet["sources"]:
        if source["source"] == "laser":
            source["valid_until_steady"] = replay.clock.steady - 0.001
    replay.node.sub.ready.append((N(data=json.dumps(packet)), {"publisher_gid": b"fusion"}))
    replay.core.tick()
    state = json.loads(replay.node.messages[-1].data)
    assert replay.core._reference_sequence == 88
    assert state["body_reference"] is None
    assert state["mode"] == "execution_fault" and state["fault_revision"] == 1


def test_ready_take_remains_bounded_when_new_snapshots_arrive_during_sampling(replay):
    packet = copy.deepcopy(replay.case["packets"][-1])
    take = replay.node.sub.take_message

    def arriving(_type, raw):
        packet["sequence"] += 1
        replay.node.sub.ready.append((N(data=json.dumps(packet)), {"publisher_gid": b"fusion"}))
        return take(_type, raw)

    replay.node.sub.take_message = arriving
    replay.core._consume_ready_reference()
    assert replay.node.sub.takes == 10
    assert replay.core._reference_sequence == 98
