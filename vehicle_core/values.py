"""Shared values independent of transport and hardware."""

from dataclasses import dataclass
from enum import Enum
from math import isfinite
from typing import Literal

HitKind = Literal["static", "unknown_static", "person", "moving", "unknown_dynamic"]


@dataclass(frozen=True)
class RangeRay:
    angle: float
    range_m: float
    valid: bool
    no_return_free: bool
    clear_limit_m: float | None


def _finite(*values: float) -> None:
    if not all(isfinite(value) for value in values):
        raise ValueError("values must be finite")


@dataclass(frozen=True)
class Velocity:
    vx: float
    wz: float

    def __post_init__(self) -> None:
        _finite(self.vx, self.wz)


@dataclass(frozen=True)
class BodyTwist:
    """Measured interval-average body twist, including independent lateral motion."""

    vx: float
    vy: float
    wz: float

    def __post_init__(self) -> None:
        _finite(self.vx, self.vy, self.wz)


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float

    def __post_init__(self) -> None:
        _finite(self.x, self.y, self.yaw)


@dataclass(frozen=True)
class MapRef:
    map_id: str
    reference_revision: int

    def __post_init__(self) -> None:
        if not self.map_id.strip() or self.reference_revision < 0:
            raise ValueError("invalid map reference")


class SensorHealth(str, Enum):
    HEALTHY = "healthy"
    DEGRADED = "degraded"
    UNAVAILABLE = "unavailable"


class OperationStatus(str, Enum):
    ACCEPTED = "accepted"
    DISPATCHING = "dispatching"
    RUNNING = "running"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    NEEDS_REVIEW = "needs_review"


@dataclass(frozen=True)
class OperationResult:
    operation_id: str
    status: OperationStatus

    def __post_init__(self) -> None:
        if not self.operation_id.strip():
            raise ValueError("operation id is required")
        object.__setattr__(self, "status", OperationStatus(self.status))
