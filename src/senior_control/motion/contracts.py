"""Immutable values shared by motion owners; units are SI and clocks explicit.

These dataclasses are internal values, not wire schemas. In particular,
ManualSession validates ManualIntent before admitting untrusted intent fields.
No contract maps intent to speed, grants permission, or changes runtime state.
"""

from dataclasses import dataclass
from enum import Enum
from typing import Literal

from senior_control.models import BodyTwist, Velocity


class MotionMode(str, Enum):
    IDLE = "idle"
    MANUAL = "manual"
    TASK = "task"
    BRAKING = "braking"
    OBSTACLE_HOLD = "obstacle_hold"
    FEEDBACK_STOP = "feedback_stop"
    EXECUTION_FAULT = "execution_fault"
    ESTOP = "estop"


class MotionEventKind(str, Enum):
    MANUAL_START = "manual_start"
    MANUAL_RELEASE = "manual_release"
    MANUAL_EXPIRED = "manual_expired"
    TASK_START = "task_start"
    TASK_PAUSE = "task_pause"
    TASK_RESUME = "task_resume"
    TASK_DONE = "task_done"
    OBSTACLE = "obstacle"
    OBSTACLE_CLEAR = "obstacle_clear"
    FEEDBACK_LOST = "feedback_lost"
    FEEDBACK_HEALTHY = "feedback_healthy"
    EXECUTION_FAULT = "execution_fault"
    FAULT_RECOVER = "fault_recover"
    SOFTWARE_ESTOP = "software_estop"
    ESTOP_RESET = "estop_reset"
    HARDWARE_ESTOP = "hardware_estop"
    HARDWARE_RELEASE = "hardware_release"


class TaskStatus(str, Enum):
    ACCEPTED = "accepted"
    RUNNING = "running"
    PAUSED_TAKEOVER = "paused_takeover"
    PAUSED_FAULT = "paused_fault"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    CANCELED = "canceled"


class TaskEventKind(str, Enum):
    ACCEPTED = "accepted"
    START = "start"
    WAYPOINT_DONE = "waypoint_done"
    TAKEOVER = "takeover"
    FEEDBACK_LOST = "feedback_lost"
    FEEDBACK_RECOVERED = "feedback_recovered"
    EXECUTION_FAULT = "execution_fault"
    RECOVER = "recover"
    ESTOP = "estop"
    RESUME = "resume"
    CANCEL = "cancel"
    SUCCESS = "success"
    FAILURE = "failure"


@dataclass(frozen=True)
class ManualIntent:
    """Normalized effort, awaiting session admission; not a velocity command."""

    authority_id: str
    gesture: int
    sequence: int
    linear: float
    turn: float
    gear: Literal["low", "high"]


@dataclass(frozen=True)
class AcceptedManualIntent:
    """One admitted sample and its fixed local receipt/deadline at handoff."""

    intent: ManualIntent
    received_steady: float
    deadline_steady: float


@dataclass(frozen=True)
class MotionTarget:
    velocity: Velocity
    source: Literal["manual", "navigation", "idle"]
    source_id: str
    deadline_steady: float


@dataclass(frozen=True)
class BodyMotion:
    """Interval reference; current absolute bounds may be unsupported (None).

    Qualification concerns the reference, not a T4 permission. Statistical
    covariance alone does not establish a deterministic current-speed bound.
    """

    average: BodyTwist
    # //Compatible: pure T3a references may lack a supported current true-speed bound.
    upper: BodyTwist | None
    interval_start_ros: float
    interval_end_ros: float
    computed_steady: float
    valid_until_steady: float
    source_epoch: str
    qualified: bool
    reason: str


@dataclass(frozen=True)
class ResponseBounds:
    accel: float
    decel: float
    jerk: float
    yaw_accel: float
    yaw_decel: float
    yaw_jerk: float
    actuator_lag: float
    valid_until_steady: float
    profile_id: str


@dataclass(frozen=True)
class ControlOutput:
    velocity: Velocity
    mode: MotionMode
    reason: str
    limited: bool


@dataclass(frozen=True)
class MotionEvent:
    kind: MotionEventKind
    source_id: str
    revision: int
    sequence: int


@dataclass(frozen=True)
class MotionState:
    mode: MotionMode
    reason: str
    task_id: str | None
    estop_revision: int
    fault_revision: int
    stationary: bool
    allowed_actions: tuple[str, ...]


@dataclass(frozen=True)
class TaskEvent:
    kind: TaskEventKind
    revision: int
    completed_waypoint: int | None
