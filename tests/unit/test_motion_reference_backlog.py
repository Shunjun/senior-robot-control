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
from control_process_test_support import (
    capture_inputs,
    local_resources,
    make_core,
    publish_inputs,
    read_inputs,
)
from motion_reference_test_support import ReadySubscription
from senior_robot import motion_core_output as module
from senior_robot.simulation_motion_profile import SimulationMotionProfile
from source_time_test_support import synthetic_capture_schedule, wire_time

from senior_control.config import DeviceProfile
from senior_control.models import Velocity


class Node:
    def __init__(self, case, clock):
        self.clock = clock
        self.messages = []
        self.warnings = []
        self.output = []
        self._depth_policy = "laser_only"
        scan = case["scan"]
        measured_label = scan["stamp"]["sec"] + scan["stamp"]["nanosec"] / 1e9
        message = N(**scan)
        message.raw_capture = N(timing=wire_time(scan["ingress_steady"], tick=measured_label))
        self._scans = [
            N(
                message=message,
                stamp=1.4,
                received=scan["ingress_steady"],
                measurement_steady=scan["ingress_steady"],
                raw_capture=N(timing=wire_time(scan["ingress_steady"], tick=1.4)),
                received_ros=scan["ingress_ros"],
            )
        ]

    def create_publisher(self, *_args):
        return N(publish=self.messages.append)

    def get_logger(self):
        return N(warning=self.warnings.append)

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

    def _emit_motion_core(self, velocity, reason, deadline=0.0, *, computed_steady):
        self.output.append((velocity, reason))


@pytest.fixture
def replay(monkeypatch):
    message = ModuleType("std_msgs.msg")
    message.String = lambda: N(data="")
    lifecycle = ModuleType("senior_robot.motion_lifecycle")
    lifecycle.with_dds_info = lambda callback: callback
    monkeypatch.setitem(sys.modules, message.__name__, message)
    monkeypatch.setitem(sys.modules, lifecycle.__name__, lifecycle)
    case = synthetic_capture_schedule(
        json.loads(Path("tests/fixtures/motion_reference_backlog.json").read_text())
    )
    old = case["packets"][0]
    clock = N(steady=old["sampled_steady"], ros=old["sampled_ros"])
    monkeypatch.setattr(module, "time", N(monotonic=lambda: clock.steady))
    node = Node(case, clock)
    local_resources(monkeypatch)
    core = make_core(monkeypatch, node, SimulationMotionProfile())
    capture_inputs(core, N(data=json.dumps(old)))
    # Restore the captured one-way phase checkpoint; never invent body speed.
    core._startup_motion_unconfirmed = case["tick"]["startup_unconfirmed"]
    core.controller._last_now = case["previous_tick"]["sampled_steady"]
    clock.steady = case["tick"]["sampled_steady"]
    clock.ros = case["tick"]["projection_clocks"][0]["ros_now"]
    yield N(core=core, node=node, clock=clock, case=case)
    core.close()


def offer(replay, packet, gid=b"fusion"):
    capture_inputs(replay.core, N(data=json.dumps(packet)))


def test_raw_updates_are_consumed_without_second_fused_snapshot_queue(replay):
    for packet in replay.case["packets"][1:]:
        offer(replay, packet)
    replay.core.tick()
    state = json.loads(replay.node.messages[-1].data)
    assert replay.core.node._reception_test.sensors.estimator._frontiers
    assert state["body_reference"] is not None and state["fault_revision"] == 0
    assert replay.node.output[-1][0] == Velocity(0, 0)
    assert not hasattr(replay.core, "_subscription")
    roots = replay.core.sensors.latest.independent_axes[0].roots
    assert state["body_reference"]["valid_until_steady"] <= min(
        r.measurement_steady + 1 for r in roots
    )


def test_control_has_no_secondary_fused_reference_subscriber(replay):
    assert not hasattr(replay.core, "_on_reference")
    assert not hasattr(replay.core, "_consume_ready_reference")
    before = replay.core.sensors
    channel = replay.core.node._reception_test.channel
    channel.update(replay.node, before)
    assert replay.core.sensors is before
    _, tracking, _ = read_inputs(replay.core, replay.clock.steady)
    assert tracking is not None
    assert read_inputs(replay.core, replay.clock.steady + 1)[1] is None


def test_reversed_updates_cannot_roll_back_raw_frontiers(replay):
    for packet in reversed(replay.case["packets"][1:]):
        offer(replay, packet)
    assert replay.core.node._reception_test.sensors.estimator._frontiers
    _, tracking, _ = read_inputs(replay.core, replay.clock.steady)
    assert tracking is not None


def test_fresh_snapshot_never_renews_original_expired_sensor(replay):
    packet = copy.deepcopy(replay.case["packets"][-1])
    laser = next(r["sample"] for r in packet["sources"] if r["source"] == "laser")
    laser["received_steady"] -= 2
    offer(replay, packet)
    read_inputs(replay.core, replay.clock.steady)
    # The malformed replacement cannot invalidate the previous valid original
    # sample or extend its lifetime; it is still usable until its own deadline.
    prior = replay.core.sensors.independent
    assert prior is not None
    replay.clock.steady = prior.deadline_steady
    read_inputs(replay.core, replay.clock.steady)
    assert replay.core.sensors.independent is None
    assert replay.core.sensors.tracking is not None  # newer wheel/IMU still available


def test_repeated_packet_is_not_new_evidence_or_extra_queued_work(replay):
    packet = replay.case["packets"][-1]
    offer(replay, packet)
    read_inputs(replay.core, replay.clock.steady)
    sequence = replay.core.sensors.tracking.sequence
    for _ in range(100):
        offer(replay, packet)
        publish_inputs(replay.core)
    read_inputs(replay.core, replay.clock.steady)
    assert replay.core.sensors.tracking.sequence == sequence
    assert len(replay.core.node._reception_test.sensors.estimator._frontiers) == 3
