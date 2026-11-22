"""Physical frame identity and independently cached raw/avoidance views."""

from senior_perception.vision.raw_capture import capture_from_message


class SensorAccess:
    """Owns sensor access state; context supplies runtime ports."""

    def __init__(self, context):
        self.context = context
        self._scan_result_key = None
        self._scan_results = [None, None]

    def _scan_key(self, raw):
        """Value identity survives snapshot unpickling; missing metadata only skips reuse."""
        try:
            m = raw.message
            ranges = tuple(m.ranges)
            return (
                capture_from_message(m.raw_capture),
                (m.source_epoch, m.sequence, m.raw_source),
                (raw.stamp, raw.received, raw.received_ros, raw.measurement_steady),
                (self.context.node._body, m.frame_id),
                (m.angle_min, m.angle_increment, m.range_min, m.range_max),
                (m.sensor_x, m.sensor_y, m.sensor_yaw, m.transform_stamp),
                (self.context.profile.length, self.context.profile.width, len(ranges)),
                ranges,
            )
        except (AttributeError, TypeError, ValueError, OverflowError):
            return None
