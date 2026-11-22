"""Undecoded scan receipt shared by ingress and motion control."""

from dataclasses import dataclass
from typing import Any

from senior_perception.vision.raw_capture import RawCaptureHeader


@dataclass(frozen=True)
class PredictedReturn:
    """Current avoidance-only range and its unrenewed real acquisition seed."""

    index: int
    distance: float
    missing_frames: int
    seed_capture: RawCaptureHeader


@dataclass(frozen=True)
class RawSafetyObservation:
    """Bounded ingress message retained without decoding its rays in the ROS executor."""

    message: Any
    received: float
    received_ros: float
    stamp: float
    measurement_steady: float
    predictions: tuple[PredictedReturn, ...] = ()
