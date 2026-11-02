"""Convert bounded native scan receipts to safety observations with measured TF."""

import math
import time
import uuid

from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from senior_motion_msgs.msg import BoundedRawLaserScan, BoundedSafetyScan
from tf2_ros import Buffer, TransformException, TransformListener

from senior_control.vision.raw_capture import RawCaptureAdmission, capture_from_message
from senior_robot.motion_lifecycle import run

MAX_RAYS = 2048
MAX_FRAME_BYTES = 128


class SafetyScanIngress(Node):
    """Keep original native receipts while qualifying the safety transform."""

    def __init__(self, *, parameter_overrides=None):
        super().__init__(
            "safety_scan_ingress", parameter_overrides=parameter_overrides or [],
            start_parameter_services=False,
        )
        self.declare_parameter("body_frame", "base_link")
        self.declare_parameter("scan_topic", "/safety/raw/scan")
        self.declare_parameter("bounded_scan_topic", "/safety/scan_candidate")
        self.declare_parameter("observation_max_age", -1.0)
        self.declare_parameter("transform_static", False)
        self._body = str(self.get_parameter("body_frame").value)
        self._raw_topic = str(self.get_parameter("scan_topic").value)
        self._max_age = float(self.get_parameter("observation_max_age").value)
        self._static = bool(self.get_parameter("transform_static").value)
        if not self._body or len(self._body.encode()) > MAX_FRAME_BYTES or not self._raw_topic:
            raise ValueError("invalid safety ingress frame or raw topic")
        self._admission = (
            RawCaptureAdmission(max_age_s=self._max_age, hard_limit_s=5.0)
            if math.isfinite(self._max_age) and 0 < self._max_age <= 5.0 else None
        )
        self._epoch = uuid.uuid4().hex
        self._source = ""
        self._bridge_gid = ""
        self._lineage = None
        self._ready = False
        self._failure_active = False
        self._sequence = 0
        self._last_ros = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._publisher = self.create_publisher(
            BoundedSafetyScan, str(self.get_parameter("bounded_scan_topic").value), 10
        )
        self.create_subscription(
            BoundedRawLaserScan, self._raw_topic, self._on_scan, qos_profile_sensor_data
        )
        self.create_timer(0.05, self._check_source, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def _message(self, reason: str, steady: float, ros: float) -> BoundedSafetyScan:
        self._sequence += 1
        value = BoundedSafetyScan()
        value.source_epoch = self._epoch
        value.raw_source = self._source
        value.sequence = self._sequence
        value.ingress_steady = steady
        value.ingress_ros = ros
        value.reason = reason[:64]
        return value

    def _new_epoch(self, reason: str, steady: float, ros: float, raw=None, *, force=False) -> None:
        if self._failure_active and not force:
            return
        self._failure_active = True
        self._epoch = uuid.uuid4().hex
        self._sequence = 0
        self._lineage = None
        value = self._message(reason, steady, ros)
        if raw is not None:
            value.raw_capture = raw
            value.stamp = raw.stamp
            value.raw_source = raw.raw_gid
        self._publisher.publish(value)

    def _check_source(self) -> bool:
        endpoints = self.get_publishers_info_by_topic(self._raw_topic)
        gid = bytes(endpoints[0].endpoint_gid).hex() if len(endpoints) == 1 else ""
        ready = self._admission is not None and self._admission.observe_bridge(gid)
        if ready != self._ready or (ready and gid != self._bridge_gid):
            self._bridge_gid, self._ready = gid, ready
            self._source = ""
            self._new_epoch("raw_source_changed", time.monotonic(),
                            self.get_clock().now().nanoseconds / 1e9, force=True)
        return ready

    def _on_scan(self, scan: BoundedRawLaserScan) -> None:
        now_ns, now_steady = self.get_clock().now().nanoseconds, time.monotonic()
        if not self._check_source() or self._admission is None:
            return
        try:
            capture = capture_from_message(scan.capture)
            previous = self._admission.generation
            if not self._admission.offer(capture, bridge_gid=self._bridge_gid,
                                         now_ros_ns=now_ns, now_steady=now_steady):
                if self._admission.generation != previous:
                    self._new_epoch(capture.reason or "raw_scan_invalid", capture.received_steady,
                                    capture.received_ros_ns / 1e9, scan.capture)
                return
            lineage = (capture.bridge_boot_id, capture.capture_generation, capture.source_epoch)
            if self._lineage is not None and lineage != self._lineage:
                self._new_epoch("raw_source_changed", capture.received_steady,
                                capture.received_ros_ns / 1e9, scan.capture, force=True)
            self._lineage = lineage
            self._source = capture.raw_gid
            value = self._message("", capture.received_steady, capture.received_ros_ns / 1e9)
            value.raw_capture, value.stamp = scan.capture, scan.capture.stamp
            frame = capture.frame_id
            if (
                not frame or len(frame.encode()) > MAX_FRAME_BYTES
                or not 1 <= len(scan.ranges) <= MAX_RAYS or len(scan.intensities) > MAX_RAYS
                or not math.isfinite(scan.time_increment) or scan.time_increment != 0.0
            ):
                raise ValueError("raw_scan_dimensions")
            now = now_ns / 1e9
            if self._last_ros is not None and now < self._last_ros:
                self._last_ros = now
                self._new_epoch("raw_clock_rollback", capture.received_steady,
                                capture.received_ros_ns / 1e9, scan.capture)
                return
            self._last_ros = now
            stamp_time = Time.from_msg(scan.capture.stamp)
            stamp = stamp_time.nanoseconds / 1e9
            if not 0 <= now - stamp <= self._max_age:
                raise ValueError("raw_scan_stale")
            tf = self._tf_buffer.lookup_transform(self._body, frame, stamp_time)
            q = tf.transform.rotation
            t = tf.transform.translation
            if (
                not all(math.isfinite(v) for v in (q.x, q.y, q.z, q.w, t.x, t.y))
                or abs(sum(v * v for v in (q.x, q.y, q.z, q.w)) - 1) > 0.001
                or abs(q.x) > 0.001 or abs(q.y) > 0.001
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
            # //Compatible: Humble accepts a bounded float sequence here.
            value.ranges = scan.ranges
            value.valid = True
            self._failure_active = False
            self._publisher.publish(value)
        except (ValueError, OverflowError, UnicodeError, TransformException) as exc:
            self._new_epoch(str(exc) or "raw_scan_invalid", now_steady, now_ns / 1e9, scan.capture)


def main() -> None:
    run(SafetyScanIngress, cli=True)
