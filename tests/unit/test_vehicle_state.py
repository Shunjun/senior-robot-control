from dataclasses import replace

import pytest

from senior_control.agent.vehicle_state import (
    ConnectionState,
    RuntimeSnapshot,
    project_vehicle_state,
)
from senior_control.motion.contracts import MotionMode, MotionState


def runtime(sequence=1, mode=MotionMode.TASK, actions=("pause_task",)):
    return RuntimeSnapshot(
        sequence=sequence,
        motion=MotionState(mode, "one_reason", "task", 7, 9, False, actions),
        task_status="running",
        wheel_linear_mps=0.5,
    )


def test_disconnect_keeps_runtime_and_never_substitutes_wheels():
    state = project_vehicle_state(runtime(), ConnectionState(connected=False, rtt_ms=900))
    assert state.mode == MotionMode.TASK
    assert state.task_status == "running"
    assert state.last_known and state.task_may_continue
    assert state.body_average is None
    assert state.wheel_linear_mps == 0.5
    assert state.allowed_actions == ()


def test_estop_pending_hardware_and_independent_revisions():
    state = project_vehicle_state(
        replace(
            runtime(mode=MotionMode.ESTOP, actions=("estop_reset", "fault_recover")),
            hardware_estop=True,
        ),
        ConnectionState(True),
        software_estop_pending=True,
    )
    assert state.estop_status == "hardware_latched"
    assert state.allowed_actions == ("fault_recover",)
    assert (state.estop_revision, state.fault_revision) == (7, 9)
    pending = project_vehicle_state(runtime(), ConnectionState(False), software_estop_pending=True)
    assert pending.estop_status == "pending"
    assert pending.mode == MotionMode.TASK


def test_old_snapshot_cannot_regress_but_connection_can_change():
    previous = project_vehicle_state(runtime(10, MotionMode.EXECUTION_FAULT), ConnectionState(True))
    state = project_vehicle_state(runtime(9), ConnectionState(False), previous=previous)
    assert state.sequence == 10
    assert state.mode == MotionMode.EXECUTION_FAULT
    assert state.last_known


def test_reconnect_same_snapshot_restores_allowed_actions():
    previous = project_vehicle_state(runtime(), ConnectionState(True))
    disconnected = project_vehicle_state(runtime(), ConnectionState(False), previous=previous)
    restored = project_vehicle_state(runtime(), ConnectionState(True), previous=disconnected)
    assert restored.allowed_actions == ("pause_task",)


def test_t5_priority_reason_is_consumed_without_rewriting_reducer():
    from senior_control.motion.contracts import MotionEvent, MotionEventKind
    from senior_control.motion.state_machine import MotionStateMachine

    machine = MotionStateMachine()
    for sequence, kind in enumerate(
        (MotionEventKind.OBSTACLE, MotionEventKind.EXECUTION_FAULT, MotionEventKind.SOFTWARE_ESTOP)
    ):
        motion = machine.reduce(
            MotionEvent(kind, "source", 0, sequence), now_steady=float(sequence)
        )
        state = project_vehicle_state(RuntimeSnapshot(sequence, motion), ConnectionState(True))
        assert (
            state.mode
            is (MotionMode.OBSTACLE_HOLD, MotionMode.EXECUTION_FAULT, MotionMode.ESTOP)[sequence]
        )
        assert state.detail_reason == motion.reason
        assert state.reason == (
            "software_estop" if state.mode is MotionMode.ESTOP else state.mode.value
        )
        assert state.allowed_actions == motion.allowed_actions


def test_new_sequence_cannot_regress_independent_safety_revision():
    previous = project_vehicle_state(runtime(5, MotionMode.ESTOP), ConnectionState(True))
    newer = runtime(6)
    newer = replace(newer, motion=replace(newer.motion, estop_revision=6))
    assert (
        project_vehicle_state(newer, ConnectionState(True), previous=previous).mode
        is MotionMode.ESTOP
    )


def test_body_average_source_age_and_staleness_are_independent_of_wheels():
    from senior_control.models import BodyTwist
    from senior_control.motion.contracts import BodyMotion

    body = BodyMotion(BodyTwist(vx=0.2, vy=0.1, wz=0.3), None, 1, 2, 10, 11, "epoch", True, "laser")
    snapshot = replace(runtime(), body=body, body_source="laser_imu", wheel_linear_mps=0.8)
    state = project_vehicle_state(snapshot, ConnectionState(True), now_steady=10.5)
    assert state.body_average == body.average
    assert state.body_source == "laser_imu"
    assert state.body_age_seconds == 0.5
    assert not state.body_stale
    stale = project_vehicle_state(snapshot, ConnectionState(False), now_steady=12)
    assert stale.body_stale
    assert stale.body_average == body.average
    assert stale.wheel_linear_mps == 0.8
    duplicate = project_vehicle_state(
        snapshot,
        ConnectionState(False),
        now_steady=12,
        previous=state,
    )
    assert duplicate.body_age_seconds == 2
    assert duplicate.body_stale


def test_lower_priority_last_event_cannot_replace_primary_fault_reason():
    from senior_control.motion.contracts import MotionEvent, MotionEventKind
    from senior_control.motion.state_machine import MotionStateMachine

    machine = MotionStateMachine()
    machine.reduce(MotionEvent(MotionEventKind.EXECUTION_FAULT, "fault", 0, 0), now_steady=0)
    motion = machine.reduce(MotionEvent(MotionEventKind.OBSTACLE, "obstacle", 0, 1), now_steady=1)
    state = project_vehicle_state(RuntimeSnapshot(1, motion), ConnectionState(True))
    assert state.reason == "execution_fault"
    assert state.detail_reason == "obstacle"


@pytest.mark.parametrize("sequence", [1, 0])
@pytest.mark.parametrize("connected", [True, False])
def test_pending_overlay_clears_without_new_telemetry(sequence, connected):
    previous = project_vehicle_state(runtime(), ConnectionState(True), software_estop_pending=True)
    cleared = project_vehicle_state(
        runtime(sequence),
        ConnectionState(connected),
        previous=previous,
        software_estop_pending=False,
    )
    assert cleared.estop_status == "clear"
    reconnected = project_vehicle_state(runtime(sequence), ConnectionState(True), previous=cleared)
    assert reconnected.estop_status == "clear"


@pytest.mark.parametrize("hardware", [True, False])
@pytest.mark.parametrize("pending", [True, False])
def test_pending_overlay_never_clears_or_relabels_confirmed_estop(hardware, pending):
    confirmed = project_vehicle_state(
        replace(runtime(10, MotionMode.ESTOP), hardware_estop=hardware), ConnectionState(True)
    )
    # Incoming hardware flag belongs only to this rejected old snapshot.
    stale = project_vehicle_state(
        replace(runtime(9), hardware_estop=not hardware),
        ConnectionState(False),
        previous=confirmed,
        software_estop_pending=pending,
    )
    assert stale.estop_status == ("hardware_latched" if hardware else "software_latched")
    same = project_vehicle_state(
        replace(runtime(10), hardware_estop=not hardware),
        ConnectionState(True),
        previous=stale,
        software_estop_pending=False,
    )
    assert same.estop_status == confirmed.estop_status
