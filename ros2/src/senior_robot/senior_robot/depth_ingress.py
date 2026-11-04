"""Depth geometry ingress. This process never publishes velocity or free space."""

import time
import uuid
from dataclasses import replace

from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from senior_motion_msgs.msg import (
    BoundedDepthEvidence,
    BoundedRawCameraInfo,
    BoundedRawImage,
    DepthSample,
)
from tf2_ros import Buffer, TransformException, TransformListener

from senior_control.vision.depth_runtime import (
    CameraInfoFrame,
    DepthGeometrySettings,
    DepthPacket,
    DepthPairBuffer,
    OpticalTransform,
    ProjectedDepthEvidence,
    produce_depth_evidence,
)
from senior_control.vision.image_codec import _validate_layout
from senior_control.vision.raw_capture import (
    RawCaptureGroup,
    capture_from_message,
    compatible_captures,
)
from senior_robot.motion_lifecycle import run


def _ns(stamp):
    if not 0 <= stamp.nanosec < 1_000_000_000:
        raise ValueError("invalid timestamp")
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _time_msg(stamp_ns):
    return Time(nanoseconds=stamp_ns).to_msg()


class DepthIngress(Node):
    """Receive CameraInfo/depth with bounded memory in a separate ROS process."""

    def __init__(self, *, parameter_overrides=None):
        super().__init__(
            "depth_ingress",
            parameter_overrides=parameter_overrides or [],
            start_parameter_services=False,
        )
        self.declare_parameter("body_frame", "base_link")
        self.declare_parameter("depth_topic", "/safety/raw/depth")
        self.declare_parameter("camera_info_topic", "/safety/raw/depth_info")
        self.declare_parameter("bounded_depth_topic", "/safety/depth_candidate")
        self.declare_parameter("min_range_m", -1.0)
        self.declare_parameter("max_range_m", -1.0)
        self.declare_parameter("range_error_m", -1.0)
        self.declare_parameter("row_stride", 8)
        self.declare_parameter("column_stride", 8)
        self.declare_parameter("max_age_s", -1.0)
        self.declare_parameter("pair_wait_s", 0.1)
        self.declare_parameter("transform_static", False)
        self._depth_topic = str(self.get_parameter("depth_topic").value)
        self._info_topic = str(self.get_parameter("camera_info_topic").value)
        self._settings = DepthGeometrySettings(
            min_range_m=float(self.get_parameter("min_range_m").value),
            max_range_m=float(self.get_parameter("max_range_m").value),
            range_error_m=float(self.get_parameter("range_error_m").value),
            row_stride=int(self.get_parameter("row_stride").value),
            column_stride=int(self.get_parameter("column_stride").value),
            body_frame=str(self.get_parameter("body_frame").value),
        )
        self._max_age = float(self.get_parameter("max_age_s").value)
        self._buffer = DepthPairBuffer(
            max_age_s=self._max_age, pair_wait_s=float(self.get_parameter("pair_wait_s").value)
        )
        self._static = bool(self.get_parameter("transform_static").value)
        self._captures = {"depth": {}, "info": {}}
        self._capture_group = RawCaptureGroup(("depth", "info"), max_age_s=self._max_age)
        self._capture_lineage = None
        self._epoch = uuid.uuid4().hex
        self._depth_source = ""
        self._info_source = ""
        self._ready = False
        self._input_sequence = 0
        self._output_sequence = 0
        self._calibration_id = ""
        self._last_ros_ns = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._publisher = self.create_publisher(
            BoundedDepthEvidence,
            str(self.get_parameter("bounded_depth_topic").value),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            BoundedRawImage, self._depth_topic, self._on_depth, qos_profile_sensor_data
        )
        self.create_subscription(
            BoundedRawCameraInfo, self._info_topic, self._on_info, qos_profile_sensor_data
        )
        self.create_timer(0.01, self._on_timer, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def _message(self, reason, received_ros_ns, received_steady, stamp_ns=0):
        self._output_sequence += 1
        message = BoundedDepthEvidence()
        message.source_epoch = self._epoch
        message.raw_source = self._depth_source
        message.camera_info_source = self._info_source
        message.sequence = self._output_sequence
        message.reason = reason[:64]
        if received_ros_ns > 0:
            message.received_ros = _time_msg(received_ros_ns)
        message.received_steady = received_steady
        if stamp_ns > 0:
            message.stamp = _time_msg(stamp_ns)
        return message

    def _invalid(self, reason, ros_ns, steady, packet=None, stamp_ns=0, capture=None):
        if packet is not None:
            ros_ns, steady, stamp = packet.received_ros_ns, packet.received_steady, packet.stamp_ns
        else:
            stamp = stamp_ns
        message = self._message(reason, ros_ns, steady, stamp)
        if capture is not None:
            stream, original = capture
            if stream == "depth":
                message.depth_capture = original
            else:
                message.camera_info_capture = original
        self._publisher.publish(message)

    def _new_epoch(self, reason, ros_ns, steady, capture=None):
        self._epoch = uuid.uuid4().hex
        self._input_sequence = self._output_sequence = 0
        self._calibration_id = ""
        self._buffer.clear()
        for values in self._captures.values():
            values.clear()
        self._capture_lineage = None
        self._invalid(reason, ros_ns, steady, capture=capture)

    def _source(self, topic):
        endpoints = self.get_publishers_info_by_topic(topic)
        if len(endpoints) != 1:
            return ""
        endpoint = endpoints[0]
        gid_bytes = bytes(endpoint.endpoint_gid)
        if not gid_bytes or not any(gid_bytes):
            return ""
        return gid_bytes.hex()

    def _check_sources(self):
        depth = self._source(self._depth_topic)
        info = self._source(self._info_topic)
        ready_depth = self._capture_group.observe_bridge("depth", depth)
        ready_info = self._capture_group.observe_bridge("info", info)
        ready = ready_depth and ready_info
        if (depth, info, ready) != (self._depth_source, self._info_source, self._ready):
            self._depth_source, self._info_source, self._ready = depth, info, ready
            self._new_epoch(
                "raw_source_changed", self.get_clock().now().nanoseconds, time.monotonic()
            )
        return ready

    def _clock_ok(self, ros_ns, steady):
        if ros_ns <= 0:
            self._buffer.clear()
            self._invalid("clock_unqualified", ros_ns, steady)
            return False
        if self._last_ros_ns is not None and ros_ns < self._last_ros_ns:
            self._last_ros_ns = ros_ns
            self._new_epoch("clock_rollback", ros_ns, steady)
            return False
        self._last_ros_ns = ros_ns
        return True

    def _offer_capture(self, stream, message, ros_ns, steady):
        capture = capture_from_message(message.capture)
        gid = self._depth_source if stream == "depth" else self._info_source
        previous = self._capture_group.generation
        accepted = self._capture_group.offer(
            stream, capture, bridge_gid=gid, now_ros_ns=ros_ns, now_steady=steady
        )
        if not accepted:
            if self._capture_group.generation != previous:
                self._new_epoch(
                    capture.reason or "raw_capture_invalid",
                    capture.received_ros_ns,
                    capture.received_steady,
                    (stream, message.capture),
                )
            return None
        lineage = (capture.bridge_boot_id, capture.capture_generation)
        if self._capture_lineage is not None and lineage != self._capture_lineage:
            self._new_epoch(
                "raw_capture_generation_changed",
                capture.received_ros_ns,
                capture.received_steady,
                (stream, message.capture),
            )
        self._capture_lineage = lineage
        values = self._captures[stream]
        values[capture.stamp_ns] = (capture, message.capture)
        while len(values) > (4 if stream == "depth" else 16):
            del values[next(iter(values))]
        return capture

    def _on_info(self, message):
        steady = time.monotonic()
        ros_ns = self.get_clock().now().nanoseconds
        if not self._clock_ok(ros_ns, steady) or not self._check_sources():
            return
        try:
            capture = self._offer_capture("info", message, ros_ns, steady)
            if capture is None:
                return
            if len(message.d) > 16:
                raise ValueError("camera_info_size")
            roi = message.roi
            info = CameraInfoFrame(
                capture.stamp_ns,
                capture.received_ros_ns,
                capture.received_steady,
                capture.frame_id,
                capture.raw_gid,
                int(message.width),
                int(message.height),
                message.distortion_model,
                tuple(message.d),
                tuple(message.k),
                tuple(message.r),
                tuple(message.p),
                int(message.binning_x),
                int(message.binning_y),
                (int(roi[0]), int(roi[1]), int(roi[2]), int(roi[3]), bool(message.roi_do_rectify)),
            )
            self._buffer.offer_info(info)
            self._drain()
        except (ValueError, OverflowError, UnicodeError):
            self._new_epoch("camera_info_invalid", ros_ns, steady, ("info", message.capture))

    def _on_depth(self, message):
        steady = time.monotonic()
        ros_ns = self.get_clock().now().nanoseconds
        if not self._clock_ok(ros_ns, steady) or not self._check_sources():
            return
        stamp_ns = 0
        try:
            capture = self._offer_capture("depth", message, ros_ns, steady)
            if capture is None:
                return
            stamp_ns = capture.stamp_ns
            size = {"16UC1": 2, "32FC1": 4}.get(str(message.encoding))
            if size is None:
                raise ValueError("depth_encoding_or_frame")
            _validate_layout(
                int(message.width), int(message.height), int(message.step), size, len(message.data)
            )
            self._input_sequence += 1
            packet = DepthPacket(
                stamp_ns,
                capture.received_ros_ns,
                capture.received_steady,
                self._epoch,
                capture.raw_gid,
                self._input_sequence,
                capture.frame_id,
                int(message.width),
                int(message.height),
                int(message.step),
                str(message.encoding),
                bool(message.is_bigendian),
                bytes(message.data),
            )
            self._buffer.offer_depth(packet)
            self._drain()
        except (ValueError, OverflowError, UnicodeError):
            self._new_epoch("depth_invalid", ros_ns, steady, ("depth", message.capture))

    def _on_timer(self):
        ros_ns = self.get_clock().now().nanoseconds
        steady = time.monotonic()
        if self._clock_ok(ros_ns, steady) and self._check_sources():
            self._drain()

    def _drain(self):
        for _ in range(8):
            now_ns = self.get_clock().now().nanoseconds
            steady = time.monotonic()
            pair, failure = self._buffer.take_ready(now_ros_ns=now_ns, now_steady=steady)
            if failure:
                self._invalid(failure, now_ns, steady, self._buffer.last_failure_packet)
                continue
            if pair is None:
                return
            packet, info = pair
            depth_capture = self._captures["depth"].pop(packet.stamp_ns, None)
            info_capture = self._captures["info"].pop(info.stamp_ns, None)
            if (
                depth_capture is None
                or info_capture is None
                or not compatible_captures(depth_capture[0], info_capture[0])
            ):
                self._new_epoch("raw_capture_pair_unqualified", now_ns, steady)
                continue
            try:
                tf = self._tf_buffer.lookup_transform(
                    self._settings.body_frame,
                    packet.frame_id,
                    Time.from_msg(_time_msg(packet.stamp_ns)),
                )
                t, q = tf.transform.translation, tf.transform.rotation
                transform = OpticalTransform(
                    _ns(tf.header.stamp), (t.x, t.y, t.z), (q.x, q.y, q.z, q.w), self._static
                )
                evidence = produce_depth_evidence(packet, info, transform, self._settings)
                if not self._check_sources() or packet.source_epoch != self._epoch:
                    continue
                if self._calibration_id and self._calibration_id != evidence.header.calibration_id:
                    self._new_epoch("calibration_changed", now_ns, steady)
                    packet = replace(packet, source_epoch=self._epoch)
                    evidence = replace(
                        evidence, header=replace(evidence.header, source_epoch=self._epoch)
                    )
                self._calibration_id = evidence.header.calibration_id
                self._publish_evidence(evidence, depth_capture[1], info_capture[1])
            except (ValueError, OverflowError, TransformException):
                self._invalid("depth_projection_unqualified", now_ns, steady, packet)

    def _publish_evidence(self, evidence: ProjectedDepthEvidence, depth_capture, info_capture):
        h, info, tf, profile = evidence.header, evidence.info, evidence.transform, evidence.settings
        value = self._message("", h.received_ros_ns, h.received_steady, h.stamp_ns)
        value.depth_capture, value.camera_info_capture = depth_capture, info_capture
        value.raw_source, value.camera_info_source = depth_capture.raw_gid, info_capture.raw_gid
        value.valid = True
        value.frame_id = h.frame_id
        value.body_frame = profile.body_frame
        value.camera_info_source = info.raw_source
        value.calibration_id = h.calibration_id
        value.camera_info_stamp = _time_msg(info.stamp_ns)
        value.camera_info_received_ros = _time_msg(info.received_ros_ns)
        value.camera_info_received_steady = info.received_steady
        value.transform_static = tf.is_static
        if tf.stamp_ns > 0:
            value.transform_stamp = _time_msg(tf.stamp_ns)
        value.width, value.height = info.width, info.height
        value.column_stride, value.row_stride = profile.column_stride, profile.row_stride
        value.fx, value.fy, value.cx, value.cy = info.k[0], info.k[4], info.k[2], info.k[5]
        value.min_range_m, value.max_range_m, value.range_error_m = (
            profile.min_range_m,
            profile.max_range_m,
            profile.range_error_m,
        )
        value.sensor_origin = list(evidence.origin)
        value.optical_to_body_quaternion = list(tf.rotation_xyzw)
        samples = []
        for source in evidence.samples:
            sample = DepthSample()
            sample.u, sample.v, sample.depth_m = source.u, source.v, source.depth_m
            sample.x, sample.y, sample.z = source.x, source.y, source.z
            samples.append(sample)
        value.samples = samples
        self._publisher.publish(value)


def main():
    run(DepthIngress, cli=True)
