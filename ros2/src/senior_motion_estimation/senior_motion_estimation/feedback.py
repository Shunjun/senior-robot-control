"""Immutable current-motion feedback shared with control consumers."""

import math
from dataclasses import dataclass

from vehicle_core.values import Velocity


@dataclass(frozen=True)
class Feedback:
    velocity: Velocity
    stamp_steady: float
    deadline_steady: float
    epoch: str
    sequence: int
    current_supported: bool
    basis: str
    # //Compatible: legacy pure callers provide complete, non-unified feedback.
    observed_axes: tuple[bool, bool] = (True, True)
    unified: bool = False

    def fresh(self, now: float) -> bool:
        return (
            self.current_supported
            and bool(self.basis)
            and bool(self.epoch)
            and type(self.sequence) is int
            and self.sequence >= 0
            and all(
                math.isfinite(x)
                for x in (
                    self.velocity.vx,
                    self.velocity.wz,
                    self.stamp_steady,
                    self.deadline_steady,
                )
            )
            and self.stamp_steady <= now < self.deadline_steady
        )
