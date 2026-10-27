"""Shared values independent of transport and hardware."""

from dataclasses import dataclass
from enum import Enum


@dataclass(frozen=True)
class Velocity:
    vx: float
    wz: float


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class MapRef:
    map_id: str
    reference_revision: int


class SensorHealth(str, Enum):
    HEALTHY = "healthy"
    DEGRADED = "degraded"
    UNAVAILABLE = "unavailable"


@dataclass(frozen=True)
class OperationResult:
    operation_id: str
    status: str
