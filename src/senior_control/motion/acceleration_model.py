"""Shared acceleration policy; deployment profiles supply only numeric limits."""

import math
from dataclasses import dataclass

from senior_control.models import Velocity


@dataclass(frozen=True)
class AccelerationModel:
    linear_limit: float
    yaw_limit: float
    reaction_linear_limit: float
    reaction_yaw_limit: float

    def __post_init__(self) -> None:
        limits = (
            self.linear_limit,
            self.yaw_limit,
            self.reaction_linear_limit,
            self.reaction_yaw_limit,
        )
        if not all(math.isfinite(value) and value >= 0 for value in limits):
            raise ValueError("invalid acceleration limits")

    @property
    def upper(self) -> Velocity:
        """Upper response/prediction limits cover the reaction-phase limits."""
        return Velocity(
            max(self.linear_limit, self.reaction_linear_limit),
            max(self.yaw_limit, self.reaction_yaw_limit),
        )

    def reaction(self, current: Velocity) -> Velocity:
        """Bound persistence of current trajectory acceleration during response lag.

        A configured maximum is a cap, not an acceleration injected into an idle
        trajectory. Sensor error and prediction uncertainty remain separate.
        """
        return Velocity(
            min(self.reaction_linear_limit, abs(current.vx)),
            min(self.reaction_yaw_limit, abs(current.wz)),
        )
