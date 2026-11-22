"""Immutable local motion display values; no control permission or state transitions."""

from dataclasses import dataclass
from enum import Enum


class MotionMode(str, Enum):
    IDLE = "idle"
    MANUAL = "manual"
    TASK = "task"
    BRAKING = "braking"
    OBSTACLE_HOLD = "obstacle_hold"
    FEEDBACK_STOP = "feedback_stop"
    EXECUTION_FAULT = "execution_fault"
    ESTOP = "estop"


@dataclass(frozen=True)
class MotionState:
    mode: MotionMode
    reason: str
    task_id: str | None
    estop_revision: int
    fault_revision: int
    stationary: bool
    allowed_actions: tuple[str, ...]
