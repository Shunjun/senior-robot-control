"""Self-filter a native bounded scan without changing original beam identities."""

import math
import os
import time
import uuid
from pathlib import Path

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from senior_interfaces.msg import BoundedFilteredScan, BoundedRawLaserScan
from senior_sensor_ingress.ros_lifecycle import run
from tf2_ros import Buffer, TransformException, TransformListener

from senior_perception.external_motion_features import external_feature_ranges
from senior_perception.learning_client import LearningClient
from senior_perception.self_filter import ScanObservation, filter_self
from senior_perception.vision.raw_capture import RawCaptureAdmission, capture_from_message
from vehicle_core.config import DeviceProfile
from vehicle_core.ros_wiring import TOPICS, publisher, publisher_identity
from vehicle_core.sensor_policy import default_policy
from vehicle_core.values import Pose2D


class ScanAdapter(Node):
    def __init__(self, *, parameter_overrides=None):
        super().__init__(
            publisher("scan_adapter").name,
            namespace=publisher("scan_adapter").namespace,
            parameter_overrides=parameter_overrides or [],
        )
        self.declare_parameter("body_frame", "base_link")
        self.declare_parameter("scan_topic", TOPICS.safety_raw_scan)
        self.declare_parameter("filtered_candidate_topic", TOPICS.perception_scan_candidate)
        self.declare_parameter("no_return_clear", False)
        self.body = self.get_parameter("body_frame").value
        hard_age_ms = default_policy().hard_age_ms
        self.max_age = hard_age_ms / 1000
        self._raw_topic = str(self.get_parameter("scan_topic").value)
        # Self-return models belong to the motion output owner; transport never
        # trusts operator masks or labels all body-interior points as learned.
        self.profile = DeviceProfile()
        claim = os.environ.get("SENIOR_STARTUP_LEARNING_CLAIM")
        self._learning_client = LearningClient(Path(claim), read_only=True) if claim else None
        self.no_return_clear = self.get_parameter("no_return_clear").value
        if (
            not self.body
            or not self._raw_topic
            or not math.isfinite(self.max_age)
            or not 0 < self.max_age <= 5.0
        ):
            raise ValueError("invalid body frame or max age")
        self._admission = RawCaptureAdmission(max_age_ms=hard_age_ms, hard_limit_ms=5000)
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
        self.status = self.create_publisher(DiagnosticArray, TOPICS.perception_scan_status, 10)
        self.create_subscription(
            BoundedRawLaserScan, self._raw_topic, self.on_scan, qos_profile_sensor_data
        )
        self.create_timer(0.05, self._check_source, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def report(self, message, usable, stamp):
        array = DiagnosticArray()
        array.header.stamp = stamp
        status = DiagnosticStatus()
        status.name = publisher("scan_adapter").name, namespace = publisher(
            "scan_adapter"
        ).namespace
        status.hardware_id = "laser"
        status.level = DiagnosticStatus.OK if usable else DiagnosticStatus.ERROR
        status.message = message
        array.status = [status]
        self.status.publish(array)

    def _invalid(self, reason, raw=None, *, force=False):
        if self._failure_active and not force:
            return
        self._failure_active = True
        self._sequence += 1
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
        if self._learning_client is not None:
            self._learning_client.poll()
        endpoints = self.get_publishers_info_by_topic(self._raw_topic)
        gid = publisher_identity(
            endpoints, "scan_receipt", "senior_interfaces/msg/BoundedRawLaserScan"
        )
        ready = self._admission.observe_bridge(gid)
        if ready != self._ready:
            self._bridge_gid, self._ready = gid, ready
            self._invalid("raw_source_changed", force=True)
        self._bridge_gid = gid
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
            lineage = (capture.bridge_boot_id, capture.capture_generation)
            if self._lineage is not None and lineage != self._lineage:
                self._invalid("raw_source_changed", msg.capture, force=True)
            self._lineage = lineage
            stamp = Time.from_msg(msg.capture.stamp)
            if not capture.frame_id:
                raise ValueError("missing scan frame")
            # TF resolves static and dynamic edges at the scan timestamp.
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
            )
            filtered = filter_self(observation, self.profile)
            view = None if self._learning_client is None else self._learning_client.view
            ranges = list(external_feature_ranges(filtered, view))
            for ray in filtered.rays:
                if ray.kind in ("self", "invalid"):
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

    def destroy_node(self):
        if self._learning_client is not None:
            self._learning_client.close()
        return super().destroy_node()


def main():
    """Assemble the shared functional node with the existing bounded ROS lifecycle."""
    run(ScanAdapter, cli=True)
