"""Raw time-support contracts. A point is not a fabricated rate interval."""

from dataclasses import dataclass
from typing import TypeAlias

from senior_control.models import Pose2D
from senior_control.motion.observations import Covariance, Frame, Observation, Source, SourceStatus


@dataclass(frozen=True)
class PointObservation:
    source: Source
    frame: Frame
    stamp_ros: float
    received_steady: float
    epoch: str
    sequence: int
    quality: float
    status: SourceStatus
    values: tuple[float, ...]  # wheel: vx,wz; IMU: wz; GPS: east,north
    covariance: Covariance | None
    model_id: str | None
    body_from_sensor: Pose2D | None  # GPS antenna origin, explicitly supplied

    @property
    def start_ros(self) -> float:
        return self.stamp_ros

    @property
    def end_ros(self) -> float:
        return self.stamp_ros


@dataclass(frozen=True)
class DeltaObservation:
    source: Source
    frame: Frame
    start_ros: float
    end_ros: float
    received_steady: float
    epoch: str
    sequence: int
    quality: float
    status: SourceStatus
    delta: Pose2D
    covariance: Covariance | None  # displacement, NOT velocity
    model_id: str | None
    body_from_sensor: Pose2D  # T_body_sensor for conjugation, identity if already body

    @property
    def values(self) -> tuple[float, ...]:
        return (self.delta.x, self.delta.y, self.delta.yaw)


# //Compatible: existing Observation is the normalized rate-interval contract.
RateIntervalObservation = Observation

MotionSample: TypeAlias = Observation | PointObservation | DeltaObservation
