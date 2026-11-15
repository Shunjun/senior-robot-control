"""Undecoded scan receipt shared by ingress and motion control."""

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class RawSafetyObservation:
    """Bounded ingress message retained without decoding its rays in the ROS executor."""

    message: Any
    received: float
    received_ros: float
    clock_advance_steady: float
    stamp: float
