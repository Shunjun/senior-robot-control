"""Frozen values shared by motion owners; no reducer or output path here."""

from dataclasses import FrozenInstanceError, replace

import pytest

from senior_control.models import BodyTwist, Velocity
from senior_control.motion.contracts import (
    BodyMotion,
    ControlOutput,
    ManualIntent,
    MotionEvent,
    MotionEventKind,
    MotionMode,
    MotionState,
    MotionTarget,
    ResponseBounds,
    TaskEvent,
    TaskEventKind,
    TaskStatus,
)


def test_accepted_intent_cannot_be_mutated_through_latest_slot() -> None:
    original = ManualIntent("authority", 1, 2, 0.5, -0.25, "low")
    with pytest.raises(FrozenInstanceError):
        original.linear = 1.0  # type: ignore[misc]
    assert replace(original, sequence=3).sequence == 3
    assert original.sequence == 2


def test_motion_target_preserves_shared_velocity_and_steady_deadline() -> None:
    velocity = Velocity(0.25, -0.5)
    target = MotionTarget(velocity, "manual", "authority", 15.0)
    assert target.velocity is velocity
    with pytest.raises(FrozenInstanceError):
        target.deadline_steady = 20.0  # type: ignore[misc]
    with pytest.raises(FrozenInstanceError):
        target.velocity.vx = 2.0  # type: ignore[misc]


def test_motion_upper_is_separate_from_measured_average() -> None:
    motion = BodyMotion(
        BodyTwist(-0.5, 0.1, -0.2),
        BodyTwist(0.6, 0.2, 0.3),
        100.0,
        100.1,
        10.0,
        10.2,
        "scan-epoch",
        True,
        "qualified",
    )
    response = ResponseBounds(1.0, 0.5, 2.0, 0.8, 0.4, 1.5, 0.1, 20.0, "measured-profile")
    assert motion.average.vx == -0.5
    assert motion.upper.vx == 0.6
    with pytest.raises(FrozenInstanceError):
        motion.qualified = False  # type: ignore[misc]
    with pytest.raises(FrozenInstanceError):
        response.decel = 100.0  # type: ignore[misc]


def test_state_and_events_are_values_without_transition_side_effects() -> None:
    event = MotionEvent(MotionEventKind.MANUAL_START, "authority", 4, 8)
    state = MotionState(MotionMode.IDLE, "standby", None, 2, 3, True, ("acquire",))
    task_event = TaskEvent(TaskEventKind.WAYPOINT_DONE, 5, 1)
    output = ControlOutput(Velocity(0.0, 0.0), MotionMode.BRAKING, "released", True)
    assert event.kind.value == "manual_start"
    assert state.mode.value == "idle"
    assert task_event.completed_waypoint == 1
    assert output.mode.value == "braking"
    for value, attribute in (
        (event, "sequence"),
        (state, "reason"),
        (task_event, "revision"),
        (output, "reason"),
    ):
        with pytest.raises(FrozenInstanceError):
            setattr(value, attribute, "changed")


def test_enum_boundaries_reject_undefined_states_and_events() -> None:
    for enum in (MotionMode, MotionEventKind, TaskStatus, TaskEventKind):
        with pytest.raises(ValueError):
            enum("unknown")
    assert TaskStatus("paused_takeover") is TaskStatus.PAUSED_TAKEOVER
    assert TaskStatus("paused_fault") is TaskStatus.PAUSED_FAULT
