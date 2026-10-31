"""Isolate unbounded ROS LaserScan deserialization from the safety output executor."""

import math
import time
import uuid

from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from senior_motion_msgs.msg import BoundedSafetyScan
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from senior_robot.motion_lifecycle import run

MAX_RAYS = 2048
MAX_FRAME_BYTES = 128


class SafetyScanIngress(Node):
    """Bound a raw input before it crosses into the short-period safety process.

    Endpoint identities are graph consistency checks in a trusted ROS domain;
    Humble's Python executor does not expose a per-message publisher identity.
    """

    def __init__(self, *, parameter_overrides=None):
        super().__init__(
            "safety_scan_ingress",
            parameter_overrides=parameter_overrides or [],
            start_parameter_services=False,
        )
        self.declare_parameter("body_frame", "base_link")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("bounded_scan_topic", "/safety/bounded_scan")
        self.declare_parameter("observation_max_age", -1.0)
        self.declare_parameter("transform_static", False)
        self._body = str(self.get_parameter("body_frame").value)
        self._raw_topic = str(self.get_parameter("scan_topic").value)
        self._max_age = float(self.get_parameter("observation_max_age").value)
        self._static = bool(self.get_parameter("transform_static").value)
        if not self._body or len(self._body.encode()) > MAX_FRAME_BYTES or not self._raw_topic:
            raise ValueError("invalid safety ingress frame or raw topic")
        self._epoch = uuid.uuid4().hex
        self._source = ""
        self._ready = False
        self._sequence = 0
        self._last_ros = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._publisher = self.create_publisher(
            BoundedSafetyScan, str(self.get_parameter("bounded_scan_topic").value), 10
        )
        self.create_subscription(LaserScan, self._raw_topic, self._on_scan, qos_profile_sensor_data)
        self.create_timer(0.05, self._check_source, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def _message(self, reason: str, steady: float, ros: float) -> BoundedSafetyScan:
        self._sequence += 1
        value = BoundedSafetyScan()
        value.source_epoch = self._epoch
        value.raw_source = self._source
        value.sequence = self._sequence
        value.ingress_steady = steady
        value.ingress_ros = ros
        value.reason = reason
        return value

    def _check_source(self) -> bool:
        endpoints = self.get_publishers_info_by_topic(self._raw_topic)
        source = ""
        if len(endpoints) == 1:
            endpoint = endpoints[0]
            gid = bytes(endpoint.endpoint_gid).hex()
            candidate = f"{endpoint.node_namespace}/{endpoint.node_name}:{gid}"
            if gid and any(bytes(endpoint.endpoint_gid)) and len(candidate.encode()) <= 128:
                source = candidate
        ready = bool(source)
        if ready != self._ready or (ready and source != self._source):
            self._epoch = uuid.uuid4().hex
            self._source = source
            self._ready = ready
            self._publisher.publish(
                self._message(
                    "raw_source_changed", time.monotonic(), self.get_clock().now().nanoseconds / 1e9
                )
            )
        return ready

    def _on_scan(self, scan: LaserScan) -> None:
        received = time.monotonic()
        now = self.get_clock().now().nanoseconds / 1e9
        if not self._check_source():
            return
        value = self._message("", received, now)
        value.stamp = scan.header.stamp
        try:
            frame = scan.header.frame_id
            if (
                not frame
                or len(frame) > MAX_FRAME_BYTES
                or len(frame.encode()) > MAX_FRAME_BYTES
                or not 1 <= len(scan.ranges) <= MAX_RAYS
                or len(scan.intensities) > MAX_RAYS
                or not math.isfinite(scan.time_increment)
                or scan.time_increment != 0.0
                or not math.isfinite(self._max_age)
                or self._max_age <= 0
            ):
                raise ValueError("raw_scan_dimensions")
            if self._last_ros is not None and now < self._last_ros:
                self._epoch = uuid.uuid4().hex
                self._last_ros = now
                value = self._message("raw_clock_rollback", received, now)
                value.stamp = scan.header.stamp
                self._publisher.publish(value)
                return
            self._last_ros = now
            stamp_time = Time.from_msg(scan.header.stamp)
            stamp = stamp_time.nanoseconds / 1e9
            if not 0 <= now - stamp <= self._max_age:
                raise ValueError("raw_scan_stale")
            tf = self._tf_buffer.lookup_transform(self._body, frame, stamp_time)
            q = tf.transform.rotation
            t = tf.transform.translation
            if (
                not all(math.isfinite(v) for v in (q.x, q.y, q.z, q.w, t.x, t.y))
                or abs(sum(v * v for v in (q.x, q.y, q.z, q.w)) - 1) > 0.001
                or abs(q.x) > 0.001
                or abs(q.y) > 0.001
            ):
                raise ValueError("raw_transform_invalid")
            tf_stamp = Time.from_msg(tf.header.stamp).nanoseconds / 1e9
            if not self._static and not 0 <= stamp - tf_stamp <= self._max_age:
                raise ValueError("raw_transform_stale")
            value.frame_id = frame
            value.angle_min = scan.angle_min
            value.angle_increment = scan.angle_increment
            value.range_min = scan.range_min
            value.range_max = scan.range_max
            value.transform_stamp = tf_stamp
            value.sensor_x = t.x
            value.sensor_y = t.y
            value.sensor_yaw = math.atan2(
                2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)
            )
            # //Compatible: Humble accepts this float array in a bounded sequence.
            value.ranges = scan.ranges
            value.valid = True
        except (ValueError, OverflowError, UnicodeError, TransformException) as exc:
            value.reason = str(exc)[:64] or "raw_scan_invalid"
        self._publisher.publish(value)


def main() -> None:
    run(SafetyScanIngress, cli=True)
