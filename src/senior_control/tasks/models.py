"""Task definitions, scan context, and execution decisions."""

from dataclasses import dataclass, field
from datetime import datetime, timezone
from enum import Enum

from senior_control.models import MapRef
from senior_control.tasks.schedule import Schedule


class IntentAction(str, Enum):
    DISPATCH = "dispatch"
    SKIP = "skip"
    PAUSE = "pause"


class ExecutionStatus(str, Enum):
    DISPATCHING = "dispatching"
    RUNNING = "running"
    PAUSED = "paused"
    SKIPPED = "skipped"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    NEEDS_REVIEW = "needs_review"


@dataclass(frozen=True)
class TaskIntent:
    task_id: str
    action: IntentAction
    reason: str
    occurrence_utc: datetime | None = None
    execution_id: str | None = None
    skip_from: datetime | None = None
    skip_until: datetime | None = None


@dataclass(frozen=True)
class ExecutionContext:
    connected: bool
    localized: bool
    safety_clear: bool
    manual_driving: bool
    active_map: MapRef | None
    invalidated_task_ids: frozenset[str] = field(default_factory=frozenset)
    last_tick_utc: datetime | None = None
    last_tick_mono: float | None = None


@dataclass(frozen=True)
class TaskDefinition:
    task_id: str
    robot_id: str
    map_ref: MapRef
    waypoint_ids: tuple[str, ...]
    schedule: Schedule
    timezone: str
    enabled: bool
    revision: int
    created_at: datetime

    def __post_init__(self) -> None:
        if not self.task_id.strip() or not self.robot_id.strip():
            raise ValueError("task identity is required")
        points = tuple(self.waypoint_ids)
        if not points or any(not point.strip() for point in points):
            raise ValueError("waypoint is required")
        if not isinstance(self.enabled, bool):
            raise ValueError("enabled must be a boolean")
        if (
            isinstance(self.revision, bool)
            or not isinstance(self.revision, int)
            or self.revision < 0
        ):
            raise ValueError("revision must be a non-negative integer")
        if self.created_at.tzinfo is None or self.created_at.utcoffset() is None:
            raise ValueError("created time must be timezone-aware")
        created = self.created_at.astimezone(timezone.utc)
        if self.timezone != self.schedule.timezone:
            raise ValueError("task timezone must match the schedule")
        if self.schedule.kind == "interval" and self.schedule.created_at != created:
            raise ValueError("interval anchor must be the task creation time")
        object.__setattr__(self, "waypoint_ids", points)
        object.__setattr__(self, "created_at", created)


@dataclass(frozen=True)
class ExecutionRecord:
    execution_id: str
    task_id: str
    occurrence_utc: datetime
    status: ExecutionStatus
    navigation_sent: bool
    rollback_confirmed: bool


@dataclass(frozen=True)
class SkipRecord:
    skip_id: str
    task_id: str
    skip_from: datetime
    skip_until: datetime
    reason: str
