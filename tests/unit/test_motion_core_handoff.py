"""Actual core ownership/state with fake ROS I/O only; no simulator is started."""

import json
import sys
import time
from types import ModuleType
from types import SimpleNamespace as N

import pytest
from control_process_test_support import (
    generic_motion_profile,
    local_resources,
    make_core,
    read_inputs,
)
from motion_reference_test_support import ReadySubscription
from senior_motion_control.contracts import MotionEventKind as K
from senior_motion_control.contracts import MotionTarget
from senior_motion_control.state_machine import RecoveryObservation
from senior_motion_control.trajectory import Trajectory
from sensor_time_test_support import sample_time

from vehicle_core.motion_state import MotionMode
from vehicle_core.values import Velocity


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, value):
        self.messages.append(value)


class Node:
    def __init__(self):
        self.publishers = {}
        self.emitted = []
        self.warnings = []

    def create_publisher(self, _type, topic, _qos):
        self.publishers[topic] = Publisher()
        return self.publishers[topic]

    def get_logger(self):
        return N(warning=self.warnings.append)

    def create_subscription(self, _type, _topic, callback, _qos):
        assert getattr(callback, "_senior_dds_info", False)
        return ReadySubscription(_type, callback)

    def get_clock(self):
        return N(now=lambda: N(nanoseconds=time.monotonic() * 1e9))

    def get_parameter(self, name):
        return N(value="route-sim")

    def _emit_motion_core(self, v, reason, deadline=0.0, *, computed_steady):
        self.last_computed_steady = computed_steady
        self.emitted.append((v, reason))


@pytest.fixture
def core(monkeypatch):
    msg = ModuleType("std_msgs.msg")
    msg.String = lambda: N(data="")
    monkeypatch.setitem(sys.modules, "std_msgs.msg", msg)
    lifecycle = ModuleType("senior_sensor_ingress.ros_lifecycle")

    def with_info(callback):
        callback._senior_dds_info = True
        return callback

    lifecycle.with_dds_info = with_info
    monkeypatch.setitem(sys.modules, "senior_sensor_ingress.ros_lifecycle", lifecycle)
    monkeypatch.setattr("senior_motion_control.motion_core_output.time.monotonic", lambda: 10.0)
    local_resources(monkeypatch)
    c = make_core(monkeypatch, Node(), generic_motion_profile())
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
        manual_linear=0.0,
        manual_linear_limit=0.0,
        degraded_linear_limit=0.0,
    )


def test_release_installs_local_owner_without_emitting_hard_zero_or_resetting_trajectory(core):
    core.target = MotionTarget(Velocity(0.4, 0), "manual", "authority:42", 10.2)
    core.controller.trajectory = Trajectory(Velocity(0.4, 0), Velocity(0.2, 0))
    message = barrier(request="retire-token")
    assert core.command(message)
    assert core.target.source == "idle" and core.target.source_id == core.boot
    assert core.target.deadline_steady is None
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
    message = barrier(reason, request="explicit" if reason == "estop" else "")
    assert core.command(message)
    revision = getattr(core.machine.state, attribute)
    for _ in range(10):
        assert core.command(message)
    assert getattr(core.machine.state, attribute) == revision
    assert core.command(barrier(reason, generation=3)) is (reason != "estop")
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


def test_control_owner_only_accepts_reception_snapshot_and_has_no_fused_subscription(core):
    assert not hasattr(core, "_on_reference")
    assert not hasattr(core, "_consume_ready_reference")
    assert hasattr(core.sensors, "input_reference")
    assert read_inputs(core, 10)[1] is None
    core.sensors.__dict__.pop("input_reference")
    with pytest.raises(ValueError, match="input_snapshot_missing"):
        core._inputs(10)


def test_explicit_idle_estop_uses_original_request_without_inventing_authority(core):
    message = barrier("estop", request="original-transport-request")
    message.source_kind = "idle"
    message.source_id = ""
    message.source_epoch = 0
    message.received_steady = 9.25
    assert core.command(message)
    first = core.machine.state.estop_revision
    assert core.command(message)
    assert core.machine.state.estop_revision == first
    echo = barrier("estop", generation=3)
    echo.source_kind = "idle"
    echo.source_id = ""
    echo.source_epoch = 0
    assert not core.command(echo)
    assert core.machine.state.estop_revision == first
    message.generation = 4
    message.request_id = "next-explicit-press"
    assert core.command(message)
    assert core.machine.state.estop_revision == first + 1
    assert core._barrier_payload[14] == 9.25


def test_release_uses_one_steady_instant_and_does_not_create_clock_fault(core, monkeypatch):
    for i in range(5):
        core.machine.observe(
            RecoveryObservation("body", i, 9.5 + i * 0.125, 11.0, True, True), 10.0
        )
    core._event(K.MANUAL_START, 10.0, "authority")
    core.target = MotionTarget(Velocity(0.2, 0), "manual", "authority:42", 10.2)
    clock = iter([10.01, 10.02, 10.03])
    monkeypatch.setattr(
        "senior_motion_control.motion_core_output.time.monotonic", lambda: next(clock)
    )
    assert core.command(barrier("release"))
    assert core.machine.state.fault_revision == 0
    assert core.machine.state.mode is not MotionMode.EXECUTION_FAULT
    assert core.target.source == "idle"


@pytest.mark.parametrize("corrected_ros", [1.0, 1_000_000_000.0])
def test_body_projection_keeps_original_monotonic_expiry_across_ros_correction(core, corrected_ros):
    from senior_motion_estimation.estimator import MotionObservation, Origin

    core.node._reception_test.sensors.estimator.ingest(
        MotionObservation(
            "laser",
            (0.0, None, 0.0),
            10.0,
            (
                Origin(
                    "laser",
                    "laser",
                    1,
                    sample_time(
                        stamp_ns=round((10.0) * 1e9),
                        received_ros_ns=round((10.0) * 1e9),
                        received_steady_ns=round((20.0) * 1e9),
                    ),
                ),
            ),
            measured_steady=20.0,
            independent=(True, True, True),
            stamp_ns=round((10.0) * 1e9),
        )
    )
    core.node.get_clock = lambda: N(now=lambda: N(nanoseconds=10.1e9))
    _, body, _ = read_inputs(core, 20.2)
    assert body is not None
    assert body.stamp_steady == 20.0 and body.deadline_steady == 21.0
    before = core.node._reception_test.sensors.latest.generation
    core.node.get_clock = lambda: N(now=lambda: N(nanoseconds=round(corrected_ros * 1e9)))
    assert (read_inputs(core, 20.2)[1] is not None) is (corrected_ros >= 10.1)
    assert core.node._reception_test.sensors.latest.generation == before + (corrected_ros < 10.1)
    assert read_inputs(core, 21.0)[1] is None
