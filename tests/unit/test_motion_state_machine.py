import pytest

from senior_control.motion.contracts import MotionEvent, MotionMode
from senior_control.motion.contracts import MotionEventKind as K
from senior_control.motion.state_machine import MotionStateMachine, RecoveryObservation


def evidence(t, seq, healthy=True, stationary=True):
    return RecoveryObservation("body", seq, t, t + 0.2, healthy, stationary)


def event(kind, seq, revision=0, source="task"):
    return MotionEvent(kind, source, revision, seq)


def recover(m, start=0):
    for i in range(5):
        m.observe(evidence(start + i * 0.125, int(start * 1000) + 100 + i), start + i * 0.125)


def test_recovery_requires_five_distinct_observations_and_half_second():
    m = MotionStateMachine()
    m.reduce(event(K.FEEDBACK_LOST, 1), now_steady=0)
    m.observe(evidence(0, 1), 0)
    for _ in range(9):
        m.observe(evidence(0, 1), 0.1)
    assert m.reduce(event(K.FEEDBACK_HEALTHY, 2), now_steady=0.1).mode is MotionMode.FEEDBACK_STOP
    recover(m, 0.2)
    assert m.reduce(event(K.FEEDBACK_HEALTHY, 3), now_steady=0.7).mode is MotionMode.IDLE


def test_estop_reset_revision_hardware_and_fault_are_independent():
    m = MotionStateMachine()
    recover(m)
    m.reduce(event(K.EXECUTION_FAULT, 1), now_steady=0.5)
    m.reduce(event(K.SOFTWARE_ESTOP, 2), now_steady=0.5)
    rev = m.state.estop_revision
    m.reduce(event(K.HARDWARE_ESTOP, 3), now_steady=0.5)
    assert m.reduce(event(K.ESTOP_RESET, 4, rev), now_steady=0.5).mode is MotionMode.ESTOP
    m.reduce(event(K.HARDWARE_RELEASE, 5), now_steady=0.5)
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


def test_expired_or_moving_evidence_cannot_reset_and_replay_does_not_revive():
    m = MotionStateMachine()
    recover(m)
    m.reduce(event(K.MANUAL_START, 1, source="a"), now_steady=0.5)
    m.reduce(event(K.SOFTWARE_ESTOP, 2), now_steady=0.5)
    revision = m.state.estop_revision
    assert m.reduce(event(K.ESTOP_RESET, 3, revision), now_steady=0.8).mode is MotionMode.ESTOP
    m.observe(evidence(0.8, 1000, stationary=False), 0.8)
    assert m.reduce(event(K.ESTOP_RESET, 4, revision), now_steady=0.8).mode is MotionMode.ESTOP
    m.observe(evidence(0.9, 1001), 0.9)
    assert m.reduce(event(K.ESTOP_RESET, 5, revision), now_steady=0.9).mode is MotionMode.IDLE
    assert m.reduce(event(K.MANUAL_START, 1, source="a"), now_steady=0.9).mode is MotionMode.IDLE


def test_recovery_gap_and_epoch_restart_require_a_new_continuous_run():
    m = MotionStateMachine()
    m.reduce(event(K.FEEDBACK_LOST, 1), now_steady=0)
    recover(m)
    m.observe(RecoveryObservation("new", 1, 1, 1.2, True, True), 1)
    assert m.reduce(event(K.FEEDBACK_HEALTHY, 2), now_steady=1).mode is MotionMode.FEEDBACK_STOP
    recover(m, 2)
    assert m.reduce(event(K.FEEDBACK_HEALTHY, 3), now_steady=2.5).mode is MotionMode.IDLE


def test_running_task_feedback_recovery_requests_replan_but_takeover_does_not():
    m = MotionStateMachine()
    recover(m)
    m.reduce(event(K.TASK_START, 1, source="task-1"), now_steady=0.5)
    m.reduce(event(K.FEEDBACK_LOST, 2), now_steady=0.5)
    recover(m, 1)
    state = m.reduce(event(K.FEEDBACK_HEALTHY, 3), now_steady=1.5)
    assert "replan_task" in state.allowed_actions and m.task_paused
    m.reduce(event(K.TASK_START, 4, source="task-1"), now_steady=1.5)
    m.reduce(event(K.MANUAL_START, 5, source="a"), now_steady=1.5)
    m.reduce(event(K.FEEDBACK_LOST, 6), now_steady=1.5)
    recover(m, 2)
    state = m.reduce(event(K.FEEDBACK_HEALTHY, 7), now_steady=2.5)
    assert "replan_task" not in state.allowed_actions and m.task_paused


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
