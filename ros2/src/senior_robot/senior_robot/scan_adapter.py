"""Self-filter a native bounded scan without changing original beam identities."""

import math
import time
import uuid

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rclpy.clock import Clock, ClockType
from rclpy.exceptions import ParameterUninitializedException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from senior_motion_msgs.msg import BoundedFilteredScan, BoundedRawLaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from senior_control.config import DeviceProfile
from senior_control.models import Pose2D
from senior_control.safety.self_filter import ScanObservation, filter_self
from senior_control.vision.raw_capture import RawCaptureAdmission, capture_from_message
from senior_robot.motion_lifecycle import run


class ScanAdapter(Node):
    def __init__(self, *, parameter_overrides=None):
        super().__init__("scan_adapter", parameter_overrides=parameter_overrides or [])
        self.declare_parameter("body_frame", "base_link")
        self.declare_parameter("scan_topic", "/safety/raw/scan")
        self.declare_parameter("filtered_candidate_topic", "/perception/scan_candidate")
        self.declare_parameter("max_age_seconds", 0.5)
        self.declare_parameter("self_mask_xy", rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("no_return_clear", False)
        self.declare_parameter("self_mask_calibrated", False)
        # //Compatible: unset profiles retain exact-mask filtering, uncalibrated by default.
        self.declare_parameter("self_mask_range_uncertainty", 0.0)
        self.declare_parameter("transform_static", False)
        self.body = self.get_parameter("body_frame").value
        self.max_age = self.get_parameter("max_age_seconds").value
        self._raw_topic = str(self.get_parameter("scan_topic").value)
        try:
            coords = self.get_parameter("self_mask_xy").value or []
        except ParameterUninitializedException:
            coords = []
        if len(coords) % 2 or (coords and len(coords) < 6):
            raise ValueError("self_mask_xy requires at least three coordinate pairs")
        self.profile = DeviceProfile(
            self_mask_calibrated=self.get_parameter("self_mask_calibrated").value,
            self_mask=[list(zip(coords[::2], coords[1::2], strict=False))] if coords else [],
        )
        self.transform_static = self.get_parameter("transform_static").value
        self.mask_range_uncertainty = float(self.get_parameter("self_mask_range_uncertainty").value)
        self.no_return_clear = self.get_parameter("no_return_clear").value
        if (
            not self.body
            or not self._raw_topic
            or not math.isfinite(self.max_age)
            or not 0 < self.max_age <= 5.0
            or not math.isfinite(self.mask_range_uncertainty)
            or self.mask_range_uncertainty < 0
        ):
            raise ValueError("invalid body frame or max age")
        self._admission = RawCaptureAdmission(max_age_s=self.max_age, hard_limit_s=5.0)
        self._bridge_gid = ""
        self._lineage = None
        self._epoch = uuid.uuid4().hex
        self._sequence = 0
        self._ready = False
        self._failure_active = False
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.publisher = self.create_publisher(
            BoundedFilteredScan,
            str(self.get_parameter("filtered_candidate_topic").value),
            qos_profile_sensor_data,
        )
        self.status = self.create_publisher(DiagnosticArray, "/perception/scan_status", 10)
        self.create_subscription(
            BoundedRawLaserScan, self._raw_topic, self.on_scan, qos_profile_sensor_data
        )
        self.create_timer(0.05, self._check_source, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def report(self, message, usable, stamp):
        array = DiagnosticArray()
        array.header.stamp = stamp
        status = DiagnosticStatus()
        status.name = "scan_adapter"
        status.hardware_id = "laser"
        status.level = DiagnosticStatus.OK if usable else DiagnosticStatus.ERROR
        status.message = message
        array.status = [status]
        self.status.publish(array)

    def _invalid(self, reason, raw=None, *, force=False):
        if self._failure_active and not force:
            return
        self._failure_active = True
        self._epoch = uuid.uuid4().hex
        self._sequence = 1
        self._lineage = None
        result = BoundedFilteredScan()
        result.source_epoch, result.sequence, result.reason = (
            self._epoch,
            self._sequence,
            reason[:64],
        )
        if raw is not None:
            result.raw_capture = raw
        self.publisher.publish(result)

    def _check_source(self):
        endpoints = self.get_publishers_info_by_topic(self._raw_topic)
        gid = bytes(endpoints[0].endpoint_gid).hex() if len(endpoints) == 1 else ""
        ready = self._admission.observe_bridge(gid)
        if ready != self._ready or (ready and gid != self._bridge_gid):
            self._bridge_gid, self._ready = gid, ready
            self._invalid("raw_source_changed", force=True)
        return ready

    def on_scan(self, msg):
        ros_ns, steady = self.get_clock().now().nanoseconds, time.monotonic()
        if not self._check_source():
            return
        try:
            capture = capture_from_message(msg.capture)
            before = self._admission.generation
            if not self._admission.offer(
                capture, bridge_gid=self._bridge_gid, now_ros_ns=ros_ns, now_steady=steady
            ):
                if self._admission.generation != before:
                    self._invalid(capture.reason or "raw_scan_invalid", msg.capture)
                return
            lineage = (capture.bridge_boot_id, capture.capture_generation, capture.source_epoch)
            if self._lineage is not None and lineage != self._lineage:
                self._invalid("raw_source_changed", msg.capture, force=True)
            self._lineage = lineage
            stamp = Time.from_msg(msg.capture.stamp)
            age = (ros_ns - stamp.nanoseconds) / 1e9
            if not 0 <= age <= self.max_age or not capture.frame_id:
                raise ValueError("stale scan or missing frame")
            tf = self.buffer.lookup_transform(self.body, capture.frame_id, stamp)
            q = tf.transform.rotation
            if (
                not all(math.isfinite(v) for v in (q.x, q.y, q.z, q.w))
                or abs(sum(v * v for v in (q.x, q.y, q.z, q.w)) - 1) > 0.001
            ):
                raise ValueError("invalid transform rotation")
            if abs(q.x) > 0.001 or abs(q.y) > 0.001:
                raise ValueError("non-planar transform")
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            seconds = stamp.nanoseconds / 1e9
            tf_seconds = Time.from_msg(tf.header.stamp).nanoseconds / 1e9
            if not self.transform_static and not 0 <= seconds - tf_seconds <= self.max_age:
                raise ValueError("stale dynamic transform")
            observation = ScanObservation(
                tuple(msg.ranges),
                msg.angle_min,
                msg.angle_increment,
                msg.range_min,
                msg.range_max,
                seconds,
                "ros",
                Pose2D(tf.transform.translation.x, tf.transform.translation.y, yaw),
                tf_seconds,
                self.body,
                seconds,
                no_return_clear=self.no_return_clear,
                transform_static=self.transform_static,
            )
            filtered = filter_self(
                observation, self.profile, self_mask_range_uncertainty=self.mask_range_uncertainty
            )
            ranges = list(msg.ranges)
            for ray in filtered.rays:
                if ray.kind in ("self", "ambiguous_self_overlap", "invalid"):
                    ranges[ray.index] = float("nan")
            result = BoundedFilteredScan()
            self._sequence += 1
            result.source_epoch, result.sequence = self._epoch, self._sequence
            result.raw_capture = msg.capture
            result.angle_min, result.angle_max, result.angle_increment = (
                msg.angle_min,
                msg.angle_max,
                msg.angle_increment,
            )
            result.time_increment, result.scan_time = msg.time_increment, msg.scan_time
            result.range_min, result.range_max = msg.range_min, msg.range_max
            result.ranges = ranges
            result.valid = True
            self._failure_active = False
            self.report("usable", True, msg.capture.stamp)
            self.publisher.publish(result)
        except (ValueError, OverflowError, UnicodeError, TransformException) as exc:
            self._invalid(str(exc) or "scan_invalid", msg.capture)
            self.report(str(exc), False, msg.capture.stamp)


def main():
    run(ScanAdapter, cli=True)
