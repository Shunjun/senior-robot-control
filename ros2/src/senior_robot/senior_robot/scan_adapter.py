"""ROS boundary for timestamped scan evidence; raw scan remains untouched."""

import copy
import math

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from senior_control.config import DeviceProfile
from senior_control.models import Pose2D
from senior_control.safety.self_filter import ScanObservation, filter_self


class ScanAdapter(Node):
    def __init__(self):
        super().__init__("scan_adapter")
        self.declare_parameter("body_frame", "base_link")
        self.declare_parameter("max_age_seconds", 0.5)
        self.declare_parameter("self_mask_xy", rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("no_return_clear", False)
        self.declare_parameter("self_mask_calibrated", False)
        self.declare_parameter("transform_static", False)
        self.body = self.get_parameter("body_frame").value
        self.max_age = self.get_parameter("max_age_seconds").value
        coords = self.get_parameter("self_mask_xy").value or []
        if len(coords) % 2 or (coords and len(coords) < 6):
            raise ValueError("self_mask_xy requires at least three coordinate pairs")
        self.profile = DeviceProfile(
            self_mask_calibrated=self.get_parameter("self_mask_calibrated").value,
            self_mask=[list(zip(coords[::2], coords[1::2], strict=False))] if coords else [],
        )
        self.transform_static = self.get_parameter("transform_static").value
        self.no_return_clear = self.get_parameter("no_return_clear").value
        if not self.body or not math.isfinite(self.max_age) or self.max_age <= 0:
            raise ValueError("invalid body frame or max age")
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.publisher = self.create_publisher(
            LaserScan, "/perception/scan_filtered", qos_profile_sensor_data
        )
        self.status = self.create_publisher(DiagnosticArray, "/perception/scan_status", 10)
        self.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)

    def report(self, message, usable):
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "scan_adapter"
        status.hardware_id = "laser"
        status.level = DiagnosticStatus.OK if usable else DiagnosticStatus.ERROR
        status.message = message
        array.status = [status]
        self.status.publish(array)

    def on_scan(self, msg):
        result = copy.deepcopy(msg)
        try:
            stamp = Time.from_msg(msg.header.stamp)
            age = (self.get_clock().now() - stamp).nanoseconds / 1e9
            if not 0 <= age <= self.max_age or not msg.header.frame_id:
                raise ValueError("stale scan or missing frame")
            tf = self.buffer.lookup_transform(self.body, msg.header.frame_id, stamp)
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
            filtered = filter_self(observation, self.profile)
            for ray in filtered.rays:
                if ray.kind in ("self", "invalid"):
                    result.ranges[ray.index] = float("nan")
            self.report("usable", True)
        except (ValueError, TransformException) as exc:
            result.ranges = [float("nan")] * len(msg.ranges)
            self.report(str(exc), False)
        self.publisher.publish(result)


def main():
    rclpy.init()
    node = ScanAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
