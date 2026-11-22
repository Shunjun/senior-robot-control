"""Shared source modes and admitted views consumed by the fusion journal."""

from dataclasses import dataclass
from enum import Enum

from senior_motion_estimation.observations import AdaptedObservation


class SourceMode(str, Enum):
    UNAVAILABLE = "unavailable"
    RECOVERING = "recovering"
    FUSED_REFERENCE = "fused_reference"
    LASER_REFERENCE = "laser_reference"
    GPS_IMU_REFERENCE = "gps_imu_reference"
    WHEEL_IMU_LIMITED = "wheel_imu_limited"


@dataclass(frozen=True)
class SourceView:
    observation: AdaptedObservation
    ready: bool
