"""Compare commanded motion, wheel spin, and accepted body motion.

Longitudinal lag inside the configured startup or braking window is not slip.
Lateral motion and unexpected yaw stop immediately. An anomaly stops and does
not request more torque.
"""

import math
from dataclasses import dataclass

from senior_control.models import Velocity
from senior_control.safety.motion_estimator import MotionEstimate, WheelOdom


@dataclass(frozen=True)
class MotionAnomaly:
    anomalous: bool
    velocity: Velocity
    increase_torque: bool
    reason: str


def _stop(reason: str) -> MotionAnomaly:
    return MotionAnomaly(True, Velocity(0.0, 0.0), False, reason)


class SlipMonitor:
    def __init__(
        self,
        *,
        startup_window: float,
        wheel_motion_limit: float,
        lateral_limit: float,
        yaw_limit: float,
    ) -> None:
        limits = (startup_window, wheel_motion_limit, lateral_limit, yaw_limit)
        if any(not math.isfinite(value) or value < 0.0 for value in limits):
            raise ValueError("slip limits must be non-negative")
        self.startup_window = startup_window
        self.wheel_motion_limit = wheel_motion_limit
        self.lateral_limit = lateral_limit
        self.yaw_limit = yaw_limit
        self._elapsed = 0.0
        self._driving: bool | None = None

    def update(
        self, command: Velocity, wheel_odom: WheelOdom, actual: MotionEstimate
    ) -> MotionAnomaly:
        if wheel_odom.dt <= 0.0 or not math.isfinite(wheel_odom.dt):
            return _stop("invalid_wheel_interval")
        if actual.quality <= 0.0:
            return _stop("body_motion_unusable")
        driving = command.vx != 0.0 or command.wz != 0.0
        if self._driving is None or driving != self._driving:
            self._elapsed = 0.0
            self._driving = driving
        self._elapsed += wheel_odom.dt
        in_window = self._elapsed <= self.startup_window
        lateral = abs(actual.pose_delta.y) > self.lateral_limit
        expected_yaw = command.wz * wheel_odom.dt
        unexpected_yaw = abs(actual.pose_delta.yaw - expected_yaw) > self.yaw_limit
        wheels_without_body = (
            abs(wheel_odom.pose_delta.x) > self.wheel_motion_limit
            and abs(actual.pose_delta.x) <= self.wheel_motion_limit * 0.5
        )
        if lateral or unexpected_yaw or (wheels_without_body and not in_window):
            return _stop("motion_anomaly")
        return MotionAnomaly(False, command, False, "within_tolerance")
