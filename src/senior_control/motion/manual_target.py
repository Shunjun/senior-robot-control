"""Manual linear intent mapping at the output owner's current source mode."""

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class ManualLinearTarget:
    fraction: float
    full_stick_mps: float
    degraded_mps: float

    def __post_init__(self) -> None:
        if (
            not all(
                math.isfinite(v) for v in (self.fraction, self.full_stick_mps, self.degraded_mps)
            )
            or abs(self.fraction) > 1
            or self.full_stick_mps < 0
            or self.degraded_mps < 0
        ):
            raise ValueError("invalid normalized manual linear target")

    def velocity(self, *, degraded: bool, capability_mps: float) -> float:
        """Scale the whole stick range before directional candidate generation.

        The deployment's normal capability ceiling still applies. The saved
        setting is the sole manual degraded policy; neither grants a permit.
        """
        limit = (
            min(self.full_stick_mps, self.degraded_mps, capability_mps)
            if degraded
            else self.full_stick_mps
        )
        return self.fraction * limit
