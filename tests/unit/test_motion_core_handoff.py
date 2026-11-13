"""Actual core ownership/state with fake ROS I/O only; no simulator is started."""

import json
import sys
import time
from types import ModuleType
from types import SimpleNamespace as N

import pytest
from senior_robot.motion_core_output import MotionCoreOutput
from senior_robot.simulation_motion_profile import SimulationMotionProfile

from senior_control.models import Velocity
from senior_control.motion.contracts import MotionEventKind as K
from senior_control.motion.contracts import MotionMode, MotionTarget
from senior_control.motion.state_machine import RecoveryObservation
from senior_control.motion.trajectory import Trajectory


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, value):
        self.messages.append(value)


class Node:
    def __init__(self):
        self.publishers = {}
        self.emitted = []

    def create_publisher(self, _type, topic, _qos):
        self.publishers[topic] = Publisher()
        return self.publishers[topic]

    def create_subscription(self, _type, _topic, callback, _qos):
        assert getattr(callback, "_senior_dds_info", False)
        return callback

    def get_clock(self):
        return N(now=lambda: N(nanoseconds=time.monotonic() * 1e9))

    def get_parameter(self, name):
        return N(value="route-sim")

    def _emit_motion_core(self, v, reason):
        self.emitted.append((v, reason))


@pytest.fixture
def core(monkeypatch):
    msg = ModuleType("std_msgs.msg")
    msg.String = lambda: N(data="")
    monkeypatch.setitem(sys.modules, "std_msgs.msg", msg)
    lifecycle = ModuleType("senior_robot.motion_lifecycle")

    def with_info(callback):
        callback._senior_dds_info = True
        return callback

    lifecycle.with_dds_info = with_info
    monkeypatch.setitem(sys.modules, "senior_robot.motion_lifecycle", lifecycle)
    monkeypatch.setattr("senior_robot.motion_core_output.time.monotonic", lambda: 10.0)
    c = MotionCoreOutput(Node(), SimulationMotionProfile())
    yield c
    c.close()


def barrier(reason="release", generation=2, request=""):
    return N(
        robot_id="route-sim",
        owner_boot_id="producer",
        session_id="session",
        request_id=request,
        generation=generation,
        connection_epoch=7,
        sequence=0,
        gesture=0,
        steady_deadline=0.0,
        vx=0.0,
        wz=0.0,
        source_kind="manual",
        source_id="authority",
        source_epoch=42,
        received_steady=9.9,
        close_reason=reason,
    )


def test_release_installs_local_owner_without_emitting_hard_zero_or_resetting_trajectory(core):
    core.target = MotionTarget(Velocity(0.4, 0), "manual", "authority:42", 10.2)
    core.controller.trajectory = Trajectory(Velocity(0.4, 0), Velocity(0.2, 0))
    message = barrier(request="retire-token")
    assert core.command(message)
    assert core.target.source == "idle" and core.target.source_id == core.boot
    assert core.target.deadline_steady == core.model_deadline
    assert core.controller.trajectory.velocity.vx == 0.4
    assert core.controller.trajectory.acceleration.vx == 0.2
    assert core.node.emitted == []
    sequence = core.event_sequence
    for _ in range(10):
        assert core.command(message)
    assert core.event_sequence == sequence


@pytest.mark.parametrize(
    "reason,attribute", [("estop", "estop_revision"), ("fault", "fault_revision")]
)
def test_identical_close_and_latched_source_echo_are_not_new_events(core, reason, attribute):
    message = barrier(reason)
    assert core.command(message)
    revision = getattr(core.machine.state, attribute)
    for _ in range(10):
        assert core.command(message)
    assert getattr(core.machine.state, attribute) == revision
    assert core.command(barrier(reason, generation=3))
    assert getattr(core.machine.state, attribute) == revision
    changed = barrier(reason, generation=3)
    changed.source_epoch = 999
    assert not core.command(changed)
    assert getattr(core.machine.state, attribute) == revision


def test_new_explicit_estop_request_advances_version_and_closed_owner_never_acks(core):
    assert core.command(barrier("estop", request="press-1"))
    rev = core.machine.state.estop_revision
    assert core.command(barrier("estop", generation=3, request="press-2"))
    assert core.machine.state.estop_revision == rev + 1
    core.close()
    assert not core.command(barrier("release", generation=4, request="retire"))


def test_failed_handoff_can_retry_exact_payload_but_cannot_change_it(core, monkeypatch):
    message = barrier(request="retire")
    actual = core._apply_close
    monkeypatch.setattr(core, "_apply_close", lambda *_: False)
    assert not core.command(message)
    assert not core._barrier_complete
    altered = barrier(request="retire")
    altered.close_reason = "expiry"
    assert not core.command(altered)
    monkeypatch.setattr(core, "_apply_close", actual)
    assert core.command(message) and core._barrier_complete


def test_missing_real_reference_publishes_strict_unhealthy_state_not_old_drive(core):
    core.tick()
    state = json.loads(core.node.publishers["/motion/runtime"].messages[-1].data)
    assert state["schema"] == 1 and "schema_version" not in state
    assert state["body_reference"] is None and state["wheel_linear_mps"] is None
    assert not state["healthy"] and not state["stationary"]
    assert core.node.emitted[-1][0] == Velocity(0, 0)


def test_new_physical_fault_after_recovery_is_not_ignored_forever(core):
    assert core.command(barrier("fault"))
    first = core.machine.state.fault_revision
    for i in range(5):
        core.machine.observe(
            RecoveryObservation("body", i, 9.5 + i * 0.125, 10.2, True, True), 10.0
        )
    core._event(K.FAULT_RECOVER, 10.0, revision=first)
    assert core.machine.state.mode is MotionMode.IDLE
    assert core.command(barrier("fault", generation=3))
    assert core.machine.state.fault_revision == first + 1


def test_feedback_pause_retains_task_replan_intent_but_source_is_fenced(core):
    for i in range(5):
        core.machine.observe(
            RecoveryObservation("body", i, 9.5 + i * 0.125, 11.0, True, True), 10.0
        )
    core._event(K.TASK_START, 10.0, "task-1")
    core.target = MotionTarget(Velocity(0.2, 0), "navigation", "task-1:42", 10.2)
    core._event(K.FEEDBACK_LOST, 10.0)
    assert core.target.source == "idle"
    for i in range(5, 10):
        core.machine.observe(
            RecoveryObservation("body", i, 10.1 + (i - 5) * 0.125, 11.5, True, True),
            10.1 + (i - 5) * 0.125,
        )
    core._event(K.FEEDBACK_HEALTHY, 10.6)
    assert "replan_task" in core.machine.state.allowed_actions


def test_reference_missing_gid_is_simulation_only_and_never_refreshes_old_packet(core):
    from dataclasses import replace

    endpoints = [N(node_name="motion_fusion", node_namespace="/", endpoint_gid=b"fusion")]
    core.node.get_publishers_info_by_topic = lambda _: endpoints

    def packet(seq=1, stamp=10.0):
        return N(
            data=json.dumps(
                dict(schema=1, boot_id="fusion-boot", sequence=seq, sampled_steady=stamp)
            )
        )

    core._on_reference(packet(), {"source_timestamp": 123})
    assert core._reference_sequence == 1
    core._on_reference(packet(2), {"publisher_gid": b"wrong"})
    assert core._reference_sequence == 1
    core._on_reference(packet(1, 10.0), {})
    core._on_reference(packet(2, 8.0), {})
    assert core._reference_sequence == 1
    endpoints.append(N(node_name="motion_fusion", node_namespace="/", endpoint_gid=b"other"))
    core._on_reference(packet(2), {})
    assert core._reference_sequence == 1
    endpoints.pop()
    endpoints[0].node_namespace = "/unexpected"
    core._on_reference(packet(2), {})
    assert core._reference_sequence == 1
    endpoints[0].node_namespace = "/"
    core.profile = replace(core.profile, simulation_only=False)
    core._on_reference(packet(2), {})
    assert core._reference_sequence == 1
    core._on_reference(packet(2), {"publisher_gid": b"fusion"})
    assert core._reference_sequence == 2


def test_explicit_idle_estop_uses_original_request_without_inventing_authority(core):
    message = barrier("estop", request="original-transport-request")
    message.source_kind = "idle"
    message.source_id = ""
    message.source_epoch = 0
    message.received_steady = 9.25
    assert core.command(message)
    first = core.machine.state.estop_revision
    assert core.command(message)
    assert core.machine.state.estop_revision == first and not core._hardware
    echo = barrier("estop", generation=3)
    echo.source_kind = "idle"
    echo.source_id = ""
    echo.source_epoch = 0
    assert core.command(echo)
    assert core.machine.state.estop_revision == first
    message.generation = 4
    message.request_id = "next-explicit-press"
    assert core.command(message)
    assert core.machine.state.estop_revision == first + 1
    assert core._barrier_payload[-2] == 9.25


def test_release_uses_one_steady_instant_and_does_not_create_clock_fault(core, monkeypatch):
    for i in range(5):
        core.machine.observe(
            RecoveryObservation("body", i, 9.5 + i * 0.125, 11.0, True, True), 10.0
        )
    core._event(K.MANUAL_START, 10.0, "authority")
    core.target = MotionTarget(Velocity(0.2, 0), "manual", "authority:42", 10.2)
    clock = iter([10.01, 10.02, 10.03])
    monkeypatch.setattr("senior_robot.motion_core_output.time.monotonic", lambda: next(clock))
    assert core.command(barrier("release"))
    assert core.machine.state.fault_revision == 0
    assert core.machine.state.mode is not MotionMode.EXECUTION_FAULT
    assert core.target.source == "idle"


def test_body_projection_uses_ros_window_and_original_owner_expiry_under_slow_rtf(core):
    core._reference = {
        "sampled_steady": 20.2,
        "sampled_ros": 10.1,
        "reference": {"end_ros": 10.0, "average": {"vx": 0.0, "vy": 0.0, "wz": 0.0}},
        "sources": [
            {
                "source": "laser",
                "ready": True,
                "valid_until_steady": 20.8,
                "sample": {"received_steady": 20.0, "epoch": "laser", "sequence": 1},
            }
        ],
    }
    core.node.get_clock = lambda: N(now=lambda: N(nanoseconds=10.1e9))
    _, body, _ = core._inputs(20.2)
    assert body is not None
    assert body.stamp_steady == 20.0 and body.deadline_steady == 20.8
    core.node.get_clock = lambda: N(now=lambda: N(nanoseconds=10.16e9))
    assert core._inputs(20.2)[1] is None
    core.node.get_clock = lambda: N(now=lambda: N(nanoseconds=10.1e9))
    assert core._inputs(20.8)[1] is None
