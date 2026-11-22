import pytest
from senior_motion_control.contracts import MotionEvent
from senior_motion_control.contracts import MotionEventKind as K
from senior_motion_control.state_machine import MotionStateMachine, RecoveryObservation

from vehicle_core.motion_state import MotionMode


def evidence(t, seq, healthy=True, stationary=True):
    return RecoveryObservation("body", seq, t, t + 0.2, healthy, stationary)


def event(kind, seq, revision=0, source="task"):
    return MotionEvent(kind, source, revision, seq)


def recover(m, start=0):
    for i in range(5):
        m.observe(evidence(start + i * 0.125, int(start * 1000) + 100 + i), start + i * 0.125)


def test_estop_reset_revision_and_fault_are_independent():
    m = MotionStateMachine()
    recover(m)
    m.reduce(event(K.EXECUTION_FAULT, 1), now_steady=0.5)
    m.reduce(event(K.SOFTWARE_ESTOP, 2), now_steady=0.5)
    rev = m.state.estop_revision
    assert m.reduce(event(K.ESTOP_RESET, 6, rev - 1), now_steady=0.5).mode is MotionMode.ESTOP
    m.observe(evidence(0.6, 700), 0.6)
    assert m.reduce(event(K.ESTOP_RESET, 7, rev), now_steady=0.6).mode is MotionMode.EXECUTION_FAULT
    recover(m, 0.7)
    assert (
        m.reduce(event(K.FAULT_RECOVER, 8, m.state.fault_revision), now_steady=1.2).mode
        is MotionMode.IDLE
    )


def test_takeover_release_reconnect_never_autoresume_task():
    m = MotionStateMachine()
    recover(m)
    m.reduce(event(K.TASK_START, 1, source="task-1"), now_steady=0.5)
    assert (
        m.reduce(event(K.MANUAL_START, 2, source="authority"), now_steady=0.5).mode
        is MotionMode.MANUAL
    )
    m.reduce(event(K.MANUAL_EXPIRED, 3, source="authority"), now_steady=0.5)
    m.observe(evidence(0.6, 700), 0.6)
    assert m.state.task_id == "task-1" and m.task_paused
    m.reduce(event(K.FEEDBACK_HEALTHY, 4), now_steady=0.6)
    assert m.state.mode is not MotionMode.TASK
    m.reduce(event(K.TASK_RESUME, 5, source="task-1"), now_steady=0.6)
    assert "replan_task" in m.state.allowed_actions
    assert m.state.mode is not MotionMode.TASK


def test_obstacle_hold_not_estop_and_reset_does_not_restart_sources():
    m = MotionStateMachine()
    recover(m)
    m.reduce(event(K.MANUAL_START, 1, source="a"), now_steady=0.5)
    assert m.reduce(event(K.OBSTACLE, 2), now_steady=0.5).mode is MotionMode.OBSTACLE_HOLD
    assert m.state.estop_revision == 0
    assert m.reduce(event(K.OBSTACLE_CLEAR, 3), now_steady=0.5).mode is MotionMode.MANUAL
    m.reduce(event(K.SOFTWARE_ESTOP, 4), now_steady=0.5)
    assert (
        m.reduce(event(K.ESTOP_RESET, 5, m.state.estop_revision), now_steady=0.5).mode
        is MotionMode.IDLE
    )


def test_reset_is_explicit_revision_matched_and_never_replays_old_source():
    m = MotionStateMachine()
    m.reduce(event(K.MANUAL_START, 1, source="a"), now_steady=0.5)
    m.reduce(event(K.SOFTWARE_ESTOP, 2), now_steady=0.5)
    revision = m.state.estop_revision
    m.observe(evidence(0.8, 1000, stationary=False), 0.8)
    assert m.state.mode is MotionMode.ESTOP
    assert m.reduce(event(K.ESTOP_RESET, 3, revision - 1), now_steady=0.8).mode is MotionMode.ESTOP
    assert m.reduce(event(K.ESTOP_RESET, 4, revision), now_steady=0.9).mode is MotionMode.IDLE
    assert m.reduce(event(K.MANUAL_START, 1, source="a"), now_steady=0.9).mode is MotionMode.IDLE


def test_observation_clock_rollback_reports_execution_fault_not_user_estop():
    m = MotionStateMachine()
    m.observe(evidence(1, 1), 1)
    m.observe(evidence(0.5, 2), 0.5)
    assert m.state.mode is MotionMode.EXECUTION_FAULT
    assert m.state.estop_revision == 0 and not m.state.stationary


@pytest.mark.parametrize("terminal", [K.TASK_PAUSE, K.TASK_DONE])
def test_explicit_task_lifecycle_cancels_feedback_auto_replan(terminal):
    m = MotionStateMachine()
    recover(m)
    m.reduce(event(K.TASK_START, 1, source="task-1"), now_steady=0.5)
    m.reduce(event(K.FEEDBACK_LOST, 2), now_steady=0.5)
    m.reduce(event(terminal, 3, source="task-1"), now_steady=0.5)
    recover(m, 1)
    state = m.reduce(event(K.FEEDBACK_HEALTHY, 4), now_steady=1.5)
    assert "replan_task" not in state.allowed_actions
    resumed = m.reduce(event(K.TASK_RESUME, 5, source="task-1"), now_steady=1.5)
    assert ("replan_task" in resumed.allowed_actions) == (terminal is K.TASK_PAUSE)
