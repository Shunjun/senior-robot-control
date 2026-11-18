"""Front/rear obstacle distance and continuous braking speed limits.

The only boundary is the configured physical rectangle enlarged 10% overall,
plus sensor_error once. No pose, learning, sweep or geometry worker is involved.
"""

import math
from dataclasses import dataclass

from senior_control.models import Velocity
from senior_control.sensors.policy import default_policy


@dataclass(frozen=True)
class DirectionalClearance:
    front: float
    rear: float
    deadline: float


def directional_clearance(scans, profile, now):
    if not scans:
        return None
    raw = scans[-1]
    try:
        age = default_policy().age(
            measured_steady=raw.measurement_steady,
            received_steady=raw.received,
            now_steady=now,
        )
    except ValueError:
        return None
    if age.band == "invalid":
        return None
    scan = raw.message
    geometry = (
        scan.angle_min,
        scan.angle_increment,
        scan.range_min,
        scan.range_max,
        scan.sensor_x,
        scan.sensor_y,
        scan.sensor_yaw,
    )
    if not all(math.isfinite(x) for x in geometry) or not scan.ranges:
        return None
    if scan.range_min < 0 or scan.range_max <= scan.range_min or scan.angle_increment == 0:
        return None
    half_length, half_width = profile.length / 2, profile.width / 2
    boundary_x = half_length * 1.10 + profile.sensor_error
    boundary_y = half_width * 1.10 + profile.sensor_error
    front = rear = math.inf
    observed = False
    for index, distance in enumerate(scan.ranges):
        if distance == math.inf:
            observed = True
            continue
        if not math.isfinite(distance) or not scan.range_min <= distance <= scan.range_max:
            continue
        observed = True
        angle = scan.sensor_yaw + scan.angle_min + index * scan.angle_increment
        x = scan.sensor_x + distance * math.cos(angle)
        y = scan.sensor_y + distance * math.sin(angle)
        # Returns inside the physical body are self returns, not external obstacles.
        if abs(x) <= half_length and abs(y) <= half_width:
            continue
        if abs(y) > boundary_y:
            continue
        if x >= 0:
            front = min(front, max(0.0, x - boundary_x))
        else:
            rear = min(rear, max(0.0, -x - boundary_x))
    return DirectionalClearance(front, rear, age.deadline_steady) if observed else None


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
