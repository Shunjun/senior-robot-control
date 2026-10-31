"""Compare commanded motion, wheel spin, and accepted body motion.

Longitudinal lag, yaw that is merely behind a turn command, and longitudinal
coast after a driving command are not slip inside the configured startup or
braking window. Opposite direction, too little body travel after that window,
lateral motion, and yaw against the command all stop. Body motion while the
command was already zero also stops. An anomaly latches until a stop is
acknowledged through ``reset``, and it never requests more torque.
"""

import math
from dataclasses import dataclass

from senior_control.models import Velocity
from senior_control.safety.motion_estimator import MotionEstimate, WheelOdom, body_twist_delta


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
        stall_distance: float | None = None,
    ) -> None:
        limits = (startup_window, wheel_motion_limit, lateral_limit, yaw_limit)
        if any(not math.isfinite(value) or value < 0.0 for value in limits) or (
            stall_distance is not None
            and (not math.isfinite(stall_distance) or stall_distance <= 0)
        ):
            raise ValueError("slip limits must be non-negative")
        self.startup_window = startup_window
        self.wheel_motion_limit = wheel_motion_limit
        self.lateral_limit = lateral_limit
        self.yaw_limit = yaw_limit
        # //Compatible: legacy callers use their existing wheel-motion limit.
        self.stall_distance = wheel_motion_limit if stall_distance is None else stall_distance
        self._elapsed = 0.0
        self._driving: bool | None = None
        self._latched = False
        self._braked_from_motion = False
        self._wheel_yaw = 0.0
        self._body_yaw = 0.0
        self._yaw_elapsed = 0.0
        self._turn_sign = 0
        self._turn_elapsed = self._turn_command = self._turn_body = 0.0
        self._linear_sign = 0
        self._linear_elapsed = 0.0
        self._linear_zero_elapsed = 0.0
        self._linear_wheel = self._linear_body = 0.0
        self._command_progress = self._command_body = 0.0
        self._lateral_error = self._yaw_error = 0.0

    def reset(self) -> None:
        """Clear the latched anomaly and the startup window."""
        self._elapsed = 0.0
        self._driving = None
        self._latched = False
        self._braked_from_motion = False
        self._wheel_yaw = 0.0
        self._body_yaw = 0.0
        self._yaw_elapsed = 0.0
        self._turn_sign = 0
        self._turn_elapsed = self._turn_command = self._turn_body = 0.0
        self._linear_sign = 0
        self._linear_elapsed = 0.0
        self._linear_zero_elapsed = 0.0
        self._linear_wheel = self._linear_body = 0.0
        self._command_progress = self._command_body = 0.0
        self._lateral_error = self._yaw_error = 0.0

    def update(
        self, command: Velocity, wheel_odom: WheelOdom, actual: MotionEstimate
    ) -> MotionAnomaly:
        if wheel_odom.dt <= 0.0 or not math.isfinite(wheel_odom.dt):
            return _stop("invalid_wheel_interval")
        if actual.quality <= 0.0:
            return _stop("body_motion_unusable")
        if actual.dt <= 0.0 or not math.isfinite(actual.dt):
            return _stop("interval_mismatch")
        if self._latched:
            return _stop("motion_anomaly")
        scale = wheel_odom.dt / actual.dt
        actual_twist = body_twist_delta(actual.pose_delta)
        wheel_twist = body_twist_delta(wheel_odom.pose_delta)
        body_x = actual_twist.x * scale
        body_y = actual_twist.y * scale
        body_yaw = actual_twist.yaw * scale
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
        wheel_x = wheel_twist.x
        if wheel_twist.yaw != 0.0:
            if self._wheel_yaw * wheel_twist.yaw < 0.0:
                self._wheel_yaw = self._body_yaw = 0.0
                self._yaw_elapsed = 0.0
            self._wheel_yaw += wheel_twist.yaw
            self._body_yaw += body_yaw
            self._yaw_elapsed = min(self.startup_window + 1.0, self._yaw_elapsed + wheel_odom.dt)
        elif self._wheel_yaw != 0.0:
            self._body_yaw += body_yaw
        yaw_stall = (
            self._yaw_elapsed > self.startup_window
            and abs(self._wheel_yaw) > self.yaw_limit
            and math.copysign(1.0, self._wheel_yaw) * self._body_yaw < abs(self._wheel_yaw) * 0.5
        )
        if (
            not yaw_stall
            and abs(self._wheel_yaw) > self.yaw_limit
            and math.copysign(1.0, self._wheel_yaw) * self._body_yaw >= abs(self._wheel_yaw) * 0.5
        ):
            self._wheel_yaw = self._body_yaw = 0.0
        expected_yaw = command.wz * wheel_odom.dt
        turn_sign = (1 if command.wz > 0 else -1) if command.wz else 0
        if turn_sign and turn_sign != self._turn_sign:
            self._turn_sign = turn_sign
            self._turn_elapsed = self._turn_command = self._turn_body = 0.0
        if self._turn_sign:
            if turn_sign or body_yaw != 0.0:
                self._turn_elapsed = min(
                    self.startup_window + 1.0, self._turn_elapsed + wheel_odom.dt
                )
            self._turn_command += max(0.0, self._turn_sign * expected_yaw)
            self._turn_body += self._turn_sign * body_yaw
        turn_stall = (
            turn_sign != 0
            and self._turn_elapsed > self.startup_window
            and self._turn_command > 0
            and self._turn_command >= self.yaw_limit
            and self._turn_body < 0.5 * self._turn_command
        )
        if (
            not turn_stall
            and self._turn_command > 0
            and self._turn_command >= self.yaw_limit
            and self._turn_body >= 0.5 * self._turn_command
        ):
            self._turn_command = self._turn_body = 0.0
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
        # Accumulate same-direction applied-command intervals across brief
        # safety-zero pulses. Independent body progress must keep pace after
        # a configured distance and startup window; one quantized zero scan is
        # not a stall, and a quiet frame cannot clear a latched anomaly.
        command_sign = (1 if command.vx > 0 else -1) if command.vx else 0
        if command_sign:
            self._linear_zero_elapsed = 0.0
        else:
            self._linear_zero_elapsed = min(
                self.startup_window + 1.0, self._linear_zero_elapsed + wheel_odom.dt
            )
        if not self._linear_sign and not command_sign and wheel_x:
            self._linear_sign = 1 if wheel_x > 0 else -1
        if command_sign and command_sign != self._linear_sign:
            self._linear_sign = command_sign
            self._linear_elapsed = 0.0
            self._linear_wheel = self._linear_body = 0.0
            self._command_progress = self._command_body = 0.0
        if self._linear_sign:
            if command_sign or wheel_x != 0.0 or body_x != 0.0:
                self._linear_elapsed = min(
                    self.startup_window + 1.0, self._linear_elapsed + wheel_odom.dt
                )
            self._linear_wheel += self._linear_sign * wheel_x
            self._linear_body += self._linear_sign * body_x
            self._command_progress += max(0.0, self._linear_sign * command.vx) * wheel_odom.dt
            self._command_body += self._linear_sign * body_x
        wheel_progress = abs(self._linear_wheel)
        wheel_body_progress = math.copysign(1.0, self._linear_wheel) * self._linear_body
        stalled = (
            (command_sign != 0 or self._linear_zero_elapsed > self.startup_window)
            and self._linear_elapsed > self.startup_window
            and (
                (
                    wheel_progress > 0
                    and wheel_progress >= self.stall_distance
                    and wheel_body_progress < 0.5 * wheel_progress
                )
                or (
                    self._command_progress > 0
                    and self._command_progress >= self.stall_distance
                    and self._command_body < 0.5 * self._command_progress
                )
            )
        )
        if (
            not stalled
            and wheel_progress > 0
            and wheel_progress >= self.stall_distance
            and wheel_body_progress >= 0.5 * wheel_progress
        ):
            self._linear_wheel = self._linear_body = 0.0
        if (
            not stalled
            and self._command_progress > 0
            and self._command_progress >= self.stall_distance
            and self._command_body >= 0.5 * self._command_progress
        ):
            self._command_progress = self._command_body = 0.0
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
        # A changing-curvature differential-drive path can have a lateral
        # component in its interval SE(2) logarithm. Compare independent body
        # motion with the aligned wheel-path shape to detect actual side drift.
        self._lateral_error += body_y - wheel_twist.y
        if not (in_window and lagging_turn):
            self._yaw_error += body_yaw - expected_yaw
        lateral = abs(self._lateral_error) > self.lateral_limit
        accumulated_yaw = abs(self._yaw_error) > self.yaw_limit
        if (
            lateral
            or accumulated_yaw
            or unexpected_yaw
            or opposite
            or uncommanded
            or yaw_stall
            or turn_stall
            or stalled
        ):
            self._latched = True
            return _stop("motion_anomaly")
        if self._latched:
            return _stop("motion_anomaly")
        return MotionAnomaly(False, command, False, "within_tolerance")
