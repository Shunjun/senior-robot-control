"""Raw time-support contracts. A point is not a fabricated rate interval."""

from dataclasses import dataclass, field
from typing import TypeAlias

from senior_control.models import Pose2D
from senior_control.motion.observations import Covariance, Frame, Observation, Source, SourceStatus
from senior_control.sensors.time_proof import MeasurementInterval, MeasurementTime


@dataclass(frozen=True)
class PointObservation:
    source: Source
    frame: Frame
    stamp_ros: float
    measurement_steady: float = field(kw_only=True)
    measurement: MeasurementTime | None = field(default=None, kw_only=True)
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
    def start_steady(self) -> float:
        return self.measurement_steady

    @property
    def end_steady(self) -> float:
        return self.measurement_steady

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
    measurement_interval: MeasurementInterval | None = field(default=None, kw_only=True)
    start_steady: float = field(kw_only=True)
    end_steady: float = field(kw_only=True)
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


def physical_support(sample: MotionSample) -> MeasurementInterval:
    """Original physical support; host receipt remains a separate TTL input."""
    if isinstance(sample, PointObservation):
        coordinate = sample.measurement or MeasurementTime("host", None, sample.measurement_steady)
        return MeasurementInterval(coordinate, coordinate)
    return sample.measurement_interval or MeasurementInterval(
        MeasurementTime("host", None, sample.start_steady),
        MeasurementTime("host", None, sample.end_steady),
    )
