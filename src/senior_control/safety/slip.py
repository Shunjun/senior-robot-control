"""Compare commanded motion, wheel spin, and accepted body motion.

Longitudinal lag, yaw that is merely behind a turn command, and longitudinal
coast after a driving command are not slip inside the configured startup or
braking window. Opposite direction, too little body travel after that window,
lateral motion, and yaw against the command all stop. Body motion while the
command was already zero also stops. An anomaly latches until the command
returns to zero or ``reset`` is called, and it never requests more torque.
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
        self._latched = False
        self._braked_from_motion = False

    def reset(self) -> None:
        """Clear the latched anomaly and the startup window."""
        self._elapsed = 0.0
        self._driving = None
        self._latched = False
        self._braked_from_motion = False

    def update(
        self, command: Velocity, wheel_odom: WheelOdom, actual: MotionEstimate
    ) -> MotionAnomaly:
        if wheel_odom.dt <= 0.0 or not math.isfinite(wheel_odom.dt):
            return _stop("invalid_wheel_interval")
        if actual.quality <= 0.0:
            return _stop("body_motion_unusable")
        if actual.dt <= 0.0 or not math.isfinite(actual.dt):
            return _stop("interval_mismatch")
        scale = wheel_odom.dt / actual.dt
        body_x = actual.pose_delta.x * scale
        body_y = actual.pose_delta.y * scale
        body_yaw = actual.pose_delta.yaw * scale
        if not all(math.isfinite(value) for value in (scale, body_x, body_y, body_yaw)):
            return _stop("interval_mismatch")
        driving = command.vx != 0.0 or command.wz != 0.0
        if driving:
            self._braked_from_motion = False
        elif self._driving is True:
            self._braked_from_motion = True
        if self._driving is None or driving != self._driving:
            self._elapsed = 0.0
            self._driving = driving
        self._elapsed += wheel_odom.dt
        in_window = self._elapsed <= self.startup_window
        wheel_x = wheel_odom.pose_delta.x
        expected_yaw = command.wz * wheel_odom.dt
        lagging_turn = (
            command.wz != 0.0
            and body_yaw * command.wz >= 0.0
            and abs(body_yaw) <= abs(expected_yaw)
        )
        unexpected_yaw = abs(body_yaw - expected_yaw) > self.yaw_limit and not (
            in_window and lagging_turn
        )
        opposite = (
            wheel_x * body_x < 0.0
            and abs(wheel_x) > self.wheel_motion_limit
            and abs(body_x) > self.wheel_motion_limit * 0.5
        )
        # Body travel under half the wheel travel is the trip threshold. The
        # latch is the hysteresis: one quiet frame does not restore the command.
        short = abs(wheel_x) > self.wheel_motion_limit and abs(body_x) < abs(wheel_x) * 0.5
        # Longitudinal coast is braking lag only after a real drive. A body that
        # moves while the command was already zero is still uncommanded.
        uncommanded = (not driving) and (
            abs(body_y) > self.lateral_limit
            or abs(body_yaw) > self.yaw_limit
            or (
                abs(body_x) > self.wheel_motion_limit
                and not (in_window and self._braked_from_motion)
            )
        )
        lateral = abs(body_y) > self.lateral_limit
        if lateral or unexpected_yaw or opposite or uncommanded or (short and not in_window):
            self._latched = True
            return _stop("motion_anomaly")
        if not driving:
            self._latched = False
        if self._latched:
            return _stop("motion_anomaly")
        return MotionAnomaly(False, command, False, "within_tolerance")
