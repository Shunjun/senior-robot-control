"""Immutable raw-source evidence produced by motion estimation."""

from dataclasses import dataclass


@dataclass(frozen=True)
class RecoveryObservation:
    epoch: str
    sequence: int
    stamp_steady: float
    deadline_steady: float
    healthy: bool
    stationary: bool
    # //Compatible: retained for existing observation producers, never admission.
    manual_rearm_ready: bool = False
