"""Existing braking and ramp algorithm, unchanged in phase one."""

import math

from vehicle_core.values import Velocity


def _approach(current, target, rate, dt):
    delta = rate * dt
    return min(target, current + delta) if target > current else max(target, current - delta)


def _braking_speed_limit(distance, decel, lag_s):
    """Solve distance = speed*lag + speed²/(2*decel), without cancellation."""
    if distance <= 0:
        return 0.0
    if math.isinf(distance):
        return math.inf
    braking_term = 2 * decel * distance
    lag_speed = decel * lag_s
    return braking_term / (math.sqrt(lag_speed**2 + braking_term) + lag_speed)


def limit_velocity(requested, previous, measured_linear, clearance, profile, dt):
    """v*lag + v²/(2*a) <= d: positive clearance permits a positive speed.

    Ramp ordinary requests from the last output; wheel speed selects the braking
    rate when the vehicle is already faster than the requested/capped speed.
    The distance cap takes precedence over the ramp, including zero at boundary.
    """
    linear = max(-profile.cap, min(profile.cap, requested.vx))
    yaw = max(-profile.yaw_cap, min(profile.yaw_cap, requested.wz))
    lag_s = profile.lag_ms / 1000
    distance = clearance.front if linear >= 0 else clearance.rear
    cap = _braking_speed_limit(distance, profile.decel, lag_s)
    desired = math.copysign(min(abs(linear), cap), linear)
    measured = previous.vx if measured_linear is None else measured_linear
    reversing = previous.vx * desired < 0
    braking = reversing or abs(desired) < abs(previous.vx) or measured * desired > desired**2
    ramp_target = 0.0 if reversing else desired
    vx = _approach(previous.vx, ramp_target, profile.decel if braking else profile.accel, dt)
    # Cap the direction actually commanded, also during a requested reversal.
    actual_distance = clearance.front if vx >= 0 else clearance.rear
    actual_cap = _braking_speed_limit(actual_distance, profile.decel, lag_s)
    vx = math.copysign(min(abs(vx), actual_cap), vx)
    if linear == 0:
        vx = 0.0
    elif abs(vx) < abs(linear):
        yaw *= abs(vx / linear)
    yaw_braking = previous.wz * yaw < 0 or abs(yaw) < abs(previous.wz)
    wz = _approach(previous.wz, yaw, profile.yaw_decel if yaw_braking else profile.yaw_accel, dt)
    if linear != 0 and vx == 0:
        wz = 0.0
    return Velocity(vx, wz)


class SpeedCalculation:
    def __init__(self, profile):
        self.profile = profile

    def calculate(self, requested, previous, measured, clearance, dt):
        return limit_velocity(requested, previous, measured, clearance, self.profile, dt)
