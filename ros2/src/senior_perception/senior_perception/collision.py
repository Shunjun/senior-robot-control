"""Front/rear obstacle distance and continuous braking speed limits.

The only boundary is the configured physical rectangle enlarged 10% overall,
plus sensor_error once. Learned self returns are excluded without pose/sweep expansion.
"""

import math
from dataclasses import dataclass

from senior_control.sensors.policy import default_policy


@dataclass(frozen=True)
class DirectionalClearance:
    front: float
    rear: float
    deadline: float


def directional_clearance(scans, profile, now, *, filtered=None):
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
    ranges = scan.ranges if filtered is None else filtered.scan.ranges
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
    for index, distance in enumerate(ranges):
        if distance == math.inf:
            observed = True
            continue
        if not math.isfinite(distance) or not scan.range_min <= distance <= scan.range_max:
            continue
        observed = True
        if filtered is not None and filtered.rays[index].kind == "self":
            continue
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


class CollisionPrediction:
    def __init__(self, profile):
        self.profile = profile

    def clearance(self, scans, now, *, filtered=None):
        return directional_clearance(scans, self.profile, now, filtered=filtered)
