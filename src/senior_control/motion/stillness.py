"""Single stillness rule: current raw wheel odometry and IMU yaw rate only."""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from senior_control.motion.controller import Feedback


def stationary(
    wheel: Feedback | None,
    imu: Feedback | None,
    now: float,
    *,
    linear_limit: float,
    yaw_limit: float,
) -> bool:
    if wheel is None or imu is None or not wheel.fresh(now) or not imu.fresh(now):
        return False
    if not all(wheel.observed_axes) or not imu.observed_axes[1]:
        return False
    values = wheel.velocity.vx, wheel.velocity.wz, imu.velocity.wz
    return (
        all(math.isfinite(v) for v in values)
        and abs(values[0]) <= linear_limit
        and abs(values[1]) <= yaw_limit
        and abs(values[2]) <= yaw_limit
    )
