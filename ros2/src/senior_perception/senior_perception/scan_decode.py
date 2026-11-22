"""Front/rear obstacle distance and continuous braking speed limits.

The only boundary is the configured physical rectangle enlarged 10% overall,
plus sensor_error once. Learned self returns are excluded without pose/sweep expansion.
"""

import math

from senior_perception.self_filter import ScanObservation, filter_self
from vehicle_core.config import DeviceProfile
from vehicle_core.values import Pose2D


def _scan_observation(raw, body_frame, ranges):
    scan = raw.message
    return filter_self(
        ScanObservation(
            tuple(ranges),
            scan.angle_min,
            scan.angle_increment,
            scan.range_min,
            scan.range_max,
            raw.stamp,
            "ros",
            Pose2D(scan.sensor_x, scan.sensor_y, scan.sensor_yaw),
            scan.transform_stamp,
            body_frame,
            raw.stamp,
        ),
        DeviceProfile(),
    )


def scan_observation(raw, body_frame):
    """Decode only original endpoints for learning and its visibility policy."""
    return _scan_observation(raw, body_frame, raw.message.ranges)


def avoidance_observation(raw, body_frame):
    """Compose the current sparse predictions without modifying the raw message."""
    ranges = list(raw.message.ranges)
    for prediction in raw.predictions:
        if ranges[prediction.index] == math.inf:
            ranges[prediction.index] = prediction.distance
    return _scan_observation(raw, body_frame, ranges)
