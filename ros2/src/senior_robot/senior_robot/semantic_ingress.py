"""Isolated registered RGB-D proposals. They never publish velocity or free space."""

import time
from pathlib import Path

from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from senior_motion_msgs.msg import (
    BoundedRawCameraInfo,
    BoundedRawImage,
    BoundedSemanticEvidence,
    FencedDepthEvidence,
    SemanticAssociation,
    SemanticObject,
)

from senior_control.vision.depth_runtime import CameraInfoFrame
from senior_control.vision.egress_receipt import (
    EgressAdmission,
    depth_capture_headers,
    egress_from_message,
)
from senior_control.vision.image_codec import _validate_layout, decode_rgb
from senior_control.vision.raw_capture import (
    RawCaptureGroup,
    capture_from_message,
    compatible_captures,
)
from senior_control.vision.semantic_coordinator import SemanticCoordinator
from senior_control.vision.semantic_runtime import (
    ColorCapture,
    SemanticSynchronizer,
    depth_snapshot,
)
from senior_control.vision.spawn_detector import SpawnInferenceWorker
from senior_robot.motion_lifecycle import run


def _time(value):
    return Time(nanoseconds=value).to_msg()


class SemanticIngress(Node):
    """Subscribe in a separate process; retain at most four exact source frames."""

    def __init__(self, *, parameter_overrides=None, _worker=None):
        super().__init__(
            "semantic_ingress", parameter_overrides=parameter_overrides or [],
            start_parameter_services=False,
        )
        self.declare_parameter("rgb_topic", "/safety/raw/rgb")
        self.declare_parameter("rgb_camera_info_topic", "/safety/raw/rgb_info")
        self.declare_parameter("bounded_depth_topic", "/safety/bounded_depth")
        self.declare_parameter("raw_depth_topic", "/safety/raw/depth")
        self.declare_parameter("raw_depth_info_topic", "/safety/raw/depth_info")
        self.declare_parameter("candidate_topic", "/safety/semantic_candidate_unfenced")
        self.declare_parameter("registered_profile", False)
        self.declare_parameter("weights_path", "")
        self.declare_parameter("device", "cpu")
        self.declare_parameter("inference_timeout_s", 5.0)
        self.declare_parameter("max_age_s", 0.3)
        self._topics = (
            str(self.get_parameter("rgb_topic").value),
            str(self.get_parameter("rgb_camera_info_topic").value),
            str(self.get_parameter("bounded_depth_topic").value),
            str(self.get_parameter("raw_depth_topic").value),
            str(self.get_parameter("raw_depth_info_topic").value),
        )
        self._max_age = float(self.get_parameter("max_age_s").value)
        self._registered = bool(self.get_parameter("registered_profile").value)
        self._depth_egress = EgressAdmission(max_age_s=self._max_age)
        self._sync = SemanticSynchronizer(max_age_s=self._max_age)
        self._capture_group = RawCaptureGroup(("rgb", "rgb_info", "depth", "depth_info"),
                                               max_age_s=self._max_age)
        self._capture_lineage = None
        self._captures = {name: {} for name in ("rgb", "rgb_info", "depth", "depth_info")}
        self._depth_messages = {}
        self._issued_captures = {}
        weights = str(self.get_parameter("weights_path").value)
        worker = _worker if _worker is not None else SpawnInferenceWorker(
            weights_path=Path(weights) if weights else None,
            device=str(self.get_parameter("device").value),
            timeout_s=float(self.get_parameter("inference_timeout_s").value),
            max_result_age_s=self._max_age,
        )
        self._coordinator = SemanticCoordinator(
            worker, registered_profile=self._registered, max_age_s=self._max_age
        )
        self._sources = ("", "", "", "", "")
        self._ready = False
        self._last_ros = None
        self._last_status = 0.0
        self._sequence = 0
        self._depth_epoch = ""
        self._depth_lineage = None
        self._depth_sequence = 0
        self._retired_epochs = set()
        self._publisher = self.create_publisher(
            BoundedSemanticEvidence, str(self.get_parameter("candidate_topic").value),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            BoundedRawImage, self._topics[0], self._on_rgb, qos_profile_sensor_data
        )
        self.create_subscription(BoundedRawCameraInfo, self._topics[1], self._on_info,
                                 qos_profile_sensor_data)
        self.create_subscription(FencedDepthEvidence, self._topics[2], self._on_depth,
                                 qos_profile_sensor_data)
        self.create_timer(0.01, self._on_timer, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def _invalid(self, reason):
        self._sequence += 1
        value = BoundedSemanticEvidence()
        value.source_epoch, value.sequence = self._coordinator.epoch, self._sequence
        value.valid, value.reason = False, reason[:64]
        self._publisher.publish(value)

    def _reset(self, reason, *, retire_depth=False):
        if retire_depth and self._depth_epoch:
            self._retired_epochs.add(self._depth_epoch)
            self._depth_epoch = ""
            self._depth_lineage = None
            self._depth_sequence = 0
        self._sync.clear()
        for values in self._captures.values():
            values.clear()
        self._depth_messages.clear()
        self._issued_captures.clear()
        self._capture_lineage = None
        self._coordinator.invalidate()
        self._sequence = 0
        self._invalid(reason)

    def _source(self, topic):
        endpoints = self.get_publishers_info_by_topic(topic)
        if len(endpoints) != 1:
            return ""
        endpoint = endpoints[0]
        gid = bytes(endpoint.endpoint_gid)
        if not gid or not any(gid):
            return ""
        return gid.hex()

    def _check_sources(self):
        current = tuple(self._source(topic) for topic in self._topics)
        graph_ready = all(self._capture_group.observe_bridge(name, current[index])
                          for name, index in (("rgb", 0), ("rgb_info", 1),
                                              ("depth", 3), ("depth_info", 4)))
        relay_ready = self._depth_egress.observe_relay(current[2])
        ready = all(current) and graph_ready and relay_ready and len(self._retired_epochs) < 256
        if current != self._sources or ready != self._ready:
            self._sources, self._ready = current, ready
            self._reset("raw_source_changed", retire_depth=True)
        return ready

    def _clock_ok(self, ros_ns):
        if ros_ns <= 0:
            self._reset("clock_unqualified", retire_depth=True)
            self._last_ros = ros_ns
            return False
        if self._last_ros is not None and ros_ns < self._last_ros:
            self._reset("clock_rollback", retire_depth=True)
            self._last_ros = ros_ns
            return False
        self._last_ros = ros_ns
        return True

    def _input_ready(self, ros_ns):
        return (self._coordinator.status[0] != "disabled" and self._registered
                and self._clock_ok(ros_ns)
                and self._check_sources())

    def _offer_capture(self, stream, wire, ros_ns, steady):
        capture = capture_from_message(wire)
        index = {"rgb": 0, "rgb_info": 1, "depth": 3, "depth_info": 4}[stream]
        prior = self._capture_group.generation
        accepted = self._capture_group.offer(stream, capture, bridge_gid=self._sources[index],
                                              now_ros_ns=ros_ns, now_steady=steady)
        if not accepted:
            if self._capture_group.generation != prior:
                self._reset(capture.reason or "raw_capture_invalid", retire_depth=True)
            return None
        lineage = (capture.bridge_boot_id, capture.capture_generation)
        if self._capture_lineage is not None and lineage != self._capture_lineage:
            self._reset("raw_capture_generation_changed", retire_depth=True)
        self._capture_lineage = lineage
        values = self._captures[stream]
        values[capture.stamp_ns] = (capture, wire)
        while len(values) > 4:
            del values[next(iter(values))]
        return capture

    def _on_rgb(self, message):
        ros_ns, steady = self.get_clock().now().nanoseconds, time.monotonic()
        if not self._input_ready(ros_ns):
            return
        try:
            capture = self._offer_capture("rgb", message.capture, ros_ns, steady)
            if capture is None:
                return
            width, height, step = int(message.width), int(message.height), int(message.step)
            encoding = str(message.encoding)
            channels = {"rgb8": 3, "bgr8": 3, "mono8": 1}.get(encoding)
            if channels is None:
                raise ValueError("rgb_encoding_invalid")
            _validate_layout(width, height, step, channels, len(message.data))
            image = decode_rgb(capture.stamp_ns / 1e9, width, height, step, encoding,
                               bytes(message.data))
            self._sync.offer_rgb(ColorCapture(capture.stamp_ns, capture.received_ros_ns,
                                              capture.received_steady, capture.frame_id,
                                              capture.raw_gid, image))
            self._drain()
        except (ValueError, OverflowError, UnicodeError):
            self._reset("rgb_invalid")

    def _on_info(self, message):
        ros_ns, steady = self.get_clock().now().nanoseconds, time.monotonic()
        if not self._input_ready(ros_ns):
            return
        try:
            capture = self._offer_capture("rgb_info", message.capture, ros_ns, steady)
            if capture is None:
                return
            model = str(message.distortion_model)
            if len(model.encode("utf-8")) > 32 or len(message.d) > 16:
                raise ValueError("rgb_camera_info_invalid")
            roi = message.roi
            info = CameraInfoFrame(
                capture.stamp_ns, capture.received_ros_ns, capture.received_steady,
                capture.frame_id, capture.raw_gid, int(message.width), int(message.height),
                model, tuple(message.d), tuple(message.k), tuple(message.r), tuple(message.p),
                int(message.binning_x), int(message.binning_y),
                (int(roi[0]), int(roi[1]), int(roi[2]), int(roi[3]),
                 bool(message.roi_do_rectify)),
            )
            self._sync.offer_info(info)
            self._drain()
        except (ValueError, OverflowError, UnicodeError):
            self._reset("rgb_camera_info_invalid")

    def _on_depth(self, envelope):
        ros_ns, steady = self.get_clock().now().nanoseconds, time.monotonic()
        if not self._input_ready(ros_ns):
            return
        try:
            owner = egress_from_message(envelope.owner)
            previous = self._depth_egress.generation
            if not self._depth_egress.offer(owner, relay_gid=self._sources[2]):
                if self._depth_egress.generation != previous:
                    self._reset("depth_invalid", retire_depth=owner.owner_lost)
                return
            message = envelope.body
            if (owner.owner_epoch != message.source_epoch
                or owner.owner_sequence != message.sequence
                or depth_capture_headers(message, depth_gid=self._sources[3],
                                         info_gid=self._sources[4], now_ros_ns=ros_ns,
                                         now_steady=steady, max_age_s=self._max_age) is None):
                raise ValueError("depth_owner_mismatch")
            epoch = str(message.source_epoch)
            if epoch in self._retired_epochs:
                return
            if not message.valid:
                sequence = int(message.sequence)
                if epoch and epoch == self._depth_epoch and 0 < sequence <= self._depth_sequence:
                    return
                if epoch == self._depth_epoch and sequence > self._depth_sequence:
                    self._depth_sequence = sequence
                self._reset("depth_invalid")
                return
            depth = depth_snapshot(message)
            depth_capture = self._offer_capture("depth", message.depth_capture, ros_ns, steady)
            if depth_capture is None:
                return
            info_capture = self._offer_capture("depth_info", message.camera_info_capture,
                                                ros_ns, steady)
            if info_capture is None:
                return
            if (not compatible_captures(depth_capture, info_capture)
                or depth_capture.stamp_ns != depth.stamp_ns
                or info_capture.stamp_ns != depth.info_stamp_ns
                or depth_capture.received_ros_ns != depth.received_ros_ns
                or info_capture.received_ros_ns != depth.info_received_ros_ns
                or depth_capture.received_steady != depth.received_steady
                or info_capture.received_steady != depth.info_received_steady
                or depth_capture.raw_gid != depth.raw_source
                or info_capture.raw_gid != depth.info_source
                or depth_capture.frame_id != depth.frame_id
                or info_capture.frame_id != depth.frame_id):
                raise ValueError("depth_capture_mismatch")
            if depth.source_epoch != self._depth_epoch:
                if self._depth_epoch:
                    self._retired_epochs.add(self._depth_epoch)
                    self._reset("depth_epoch_changed")
                self._depth_epoch = depth.source_epoch
                self._depth_lineage = None
                self._depth_sequence = 0
            lineage = (depth.raw_source, depth.info_source, depth.calibration_id,
                       depth.frame_id, depth.body_frame)
            if ((self._depth_lineage is not None and lineage != self._depth_lineage)
                or depth.sequence <= self._depth_sequence):
                self._reset("depth_lineage_invalid", retire_depth=True)
                return
            self._depth_lineage, self._depth_sequence = lineage, depth.sequence
            self._depth_messages[depth.stamp_ns] = (message, envelope.owner)
            while len(self._depth_messages) > 4:
                del self._depth_messages[next(iter(self._depth_messages))]
            self._sync.offer_depth(depth)
            self._drain()
        except (ValueError, OverflowError, UnicodeError):
            self._reset("depth_invalid")

    def _on_timer(self):
        ros_ns, steady = self.get_clock().now().nanoseconds, time.monotonic()
        status, reason = self._coordinator.status
        if steady - self._last_status >= 0.2 and (status in ("disabled", "failed")
                                                 or not self._registered):
            self._invalid(reason if status in ("disabled", "failed")
                          else "registration_unqualified")
            self._last_status = steady
        if not self._input_ready(ros_ns):
            return
        self._drain()
        for _ in range(4):
            candidate = self._coordinator.take_result(
                now_ros_ns=self.get_clock().now().nanoseconds, now_steady=time.monotonic()
            )
            if candidate is None:
                break
            self._publish_candidate(candidate)

    def _drain(self):
        for _ in range(8):
            now, steady = self.get_clock().now().nanoseconds, time.monotonic()
            pair, reason = self._sync.take_ready(now_ros_ns=now, now_steady=steady)
            if reason:
                self._reset(reason, retire_depth=reason == "clock_unqualified")
                return
            if pair is None:
                return
            stamp = pair[0].stamp_ns
            captures = [self._captures[name].get(stamp) for name in
                        ("rgb", "rgb_info", "depth", "depth_info")]
            depth_message = self._depth_messages.get(stamp)
            if (any(value is None for value in captures) or depth_message is None
                or not compatible_captures(*(value[0] for value in captures))):
                self._reset("raw_capture_pair_unqualified", retire_depth=True)
                return
            previous_epoch = self._coordinator.epoch
            submitted = self._coordinator.submit(*pair, now_ros_ns=now, now_steady=steady)
            if submitted:
                self._issued_captures[stamp] = (captures, depth_message)
                while len(self._issued_captures) > 4:
                    del self._issued_captures[next(iter(self._issued_captures))]
            if previous_epoch != self._coordinator.epoch:
                self._sequence = 0
                self._invalid("semantic_generation_changed")
            if not submitted:
                self._invalid("semantic_frame_unqualified")

    def _publish_candidate(self, candidate):
        value = BoundedSemanticEvidence()
        self._sequence += 1
        value.source_epoch, value.sequence = candidate.epoch, self._sequence
        value.registration_id = candidate.registration_id
        rgb, info, depth = candidate.rgb, candidate.info, candidate.depth
        value.rgb_stamp, value.rgb_received_ros = _time(rgb.stamp_ns), _time(rgb.received_ros_ns)
        value.rgb_camera_info_stamp = _time(info.stamp_ns)
        value.rgb_camera_info_received_ros = _time(info.received_ros_ns)
        value.rgb_received_steady, value.rgb_camera_info_received_steady = (
            rgb.received_steady, info.received_steady
        )
        value.rgb_frame_id, value.rgb_source, value.rgb_camera_info_source = (
            rgb.frame_id, rgb.raw_source, info.raw_source
        )
        value.rgb_width, value.rgb_height = info.width, info.height
        value.distortion_model, value.d = info.distortion_model, list(info.d)
        value.k, value.r, value.p = list(info.k), list(info.r), list(info.p)
        value.binning_x, value.binning_y = info.binning_x, info.binning_y
        value.roi, value.roi_do_rectify = list(info.roi[:4]), info.roi[4]
        retained = self._issued_captures.pop(rgb.stamp_ns, None)
        if retained is None:
            self._invalid("semantic_result_expired")
            return
        captures, depth_record = retained
        original_depth, depth_owner = depth_record
        retained_owner = egress_from_message(depth_owner)
        current_owner = self._depth_egress.latest
        if (current_owner is None or retained_owner.owner_gid != current_owner.owner_gid
            or retained_owner.owner_epoch != current_owner.owner_epoch
            or retained_owner.owner_generation != current_owner.owner_generation
            or retained_owner.relay_gid != current_owner.relay_gid
            or retained_owner.relay_boot != current_owner.relay_boot
            or original_depth.sequence != depth.sequence
            or original_depth.source_epoch != depth.source_epoch):
            self._invalid("semantic_result_expired")
            return
        value.rgb_capture, value.rgb_camera_info_capture = captures[0][1], captures[1][1]
        value.depth_owner, value.depth = depth_owner, original_depth
        objects = []
        for detection in candidate.detections:
            item = SemanticObject()
            item.label, item.confidence, item.box = detection.label, detection.confidence, list(
                detection.box
            )
            objects.append(item)
        value.objects = objects
        indices = {(sample.u, sample.v): index for index, sample in enumerate(depth.samples)}
        associations = []
        for association in candidate.associations:
            item = SemanticAssociation()
            item.sample_index = indices[(association.sample.u, association.sample.v)]
            item.object_mask = association.object_mask
            associations.append(item)
        value.associations = associations
        if not self._check_sources() or not self._coordinator.is_current(
            candidate, now_ros_ns=self.get_clock().now().nanoseconds, now_steady=time.monotonic()
        ):
            self._invalid("semantic_result_expired")
            return
        value.valid = True
        self._publisher.publish(value)

    def destroy_node(self):
        self._coordinator.close()
        return super().destroy_node()


def main():
    run(SemanticIngress, cli=True)
