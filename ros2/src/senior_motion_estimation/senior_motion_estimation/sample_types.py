"""Raw time-support contracts. A point is not a fabricated rate interval."""

from dataclasses import dataclass
from typing import TypeAlias

from senior_control.models import Pose2D
from senior_motion_estimation.observations import (
    Covariance,
    Frame,
    Observation,
    Source,
    SourceStatus,
)
from vehicle_core.sample_time import SampleTime, StampInterval, interval_ns


@dataclass(frozen=True)
class PointObservation:
    source: Source
    frame: Frame
    time: SampleTime
    epoch: str
    sequence: int
    quality: float
    status: SourceStatus
    values: tuple[float, ...]  # wheel: vx,wz; IMU: wz; GPS: east,north
    covariance: Covariance | None
    model_id: str | None
    body_from_sensor: Pose2D | None  # GPS antenna origin, explicitly supplied

    @property
    def stamp_ros(self) -> float:
        return self.time.stamp_ns / 1e9

    @property
    def measurement_steady(self) -> float:
        return self.time.sample_steady_ns / 1e9

    @property
    def received_steady(self) -> float:
        return self.time.received_steady_ns / 1e9

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
    first_time: SampleTime
    last_time: SampleTime
    epoch: str
    sequence: int
    quality: float
    status: SourceStatus
    delta: Pose2D
    covariance: Covariance | None  # displacement, NOT velocity
    model_id: str | None
    body_from_sensor: Pose2D  # T_body_sensor for conjugation, identity if already body

    @property
    def start_ros(self) -> float:
        return self.first_time.stamp_ns / 1e9

    @property
    def end_ros(self) -> float:
        return self.last_time.stamp_ns / 1e9

    @property
    def start_steady(self) -> float:
        return self.first_time.sample_steady_ns / 1e9

    @property
    def end_steady(self) -> float:
        return self.last_time.sample_steady_ns / 1e9

    @property
    def received_steady(self) -> float:
        return self.last_time.received_steady_ns / 1e9

    @property
    def values(self) -> tuple[float, ...]:
        return (self.delta.x, self.delta.y, self.delta.yaw)


MotionSample: TypeAlias = Observation | PointObservation | DeltaObservation


def physical_support(sample: MotionSample) -> StampInterval:
    """Original ROS endpoints, with no host-coordinate fallback."""
    if isinstance(sample, PointObservation):
        return StampInterval(sample.time.stamp_ns, sample.time.stamp_ns)
    return interval_ns(sample.first_time, sample.last_time)
