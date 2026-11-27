"""Isolated SIM RGB-D diagnostic producer; native egress owns authentication."""

import threading
import time
import uuid
from contextlib import suppress

from geometry_msgs.msg import TransformStamped
from rclpy._rclpy_pybind11 import RCLError  # //Compatible: Humble exposes RCLError here.
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from senior_interfaces.msg import (
    BoundedRawCameraInfo,
    BoundedRawImage,
    RgbdMotionCandidate,
)
from tf2_ros import Buffer, TransformException, TransformListener

from senior_perception.vision.independent_motion import Profile
from senior_perception.vision.raw_capture import capture_from_message
from senior_perception.vision.rgbd_bounded_inbox import BoundedInbox
from senior_perception.vision.rgbd_candidate_codec import fill_candidate
from senior_perception.vision.rgbd_producer_engine import RetainedFrontiers, RgbdProducerEngine
from senior_perception.vision.static_tf_binding import Edge, StaticBinding, TransformSnapshot
from vehicle_core.diagnostics import process_diagnostics


class RgbdCandidateProducer(Node):
    def __init__(self):
        super().__init__("rgbd_candidate_producer")
        try:
            self._initialize()
        except BaseException:
            with suppress(BaseException):
                if hasattr(self, "_stop"):
                    self._stop.set()
                    self._wake.set()
                if hasattr(self, "_thread") and self._thread.is_alive():
                    self._thread.join(timeout=5)
                super().destroy_node()
            raise

    def _initialize(self):
        self.declare_parameter("static_background_declared", False)
        self.declare_parameter("body_frame", "base_link")
        self.declare_parameter("camera_frame", "camera_color_optical_frame")
        self._enabled = bool(self.get_parameter("static_background_declared").value)
        self._body = str(self.get_parameter("body_frame").value)
        camera = str(self.get_parameter("camera_frame").value)
        self._binding = StaticBinding(self._body, camera)
        self._profile = Profile(
            "sim",
            "explicit_sim_declaration",
            self._enabled,
            "static_scene",
            "sim_calibration",
            0.4,
            0.4,
            2.0,
            0.1,
            0.1,
        )
        self._boot = uuid.uuid4().hex
        self._generation = 1
        self._sequence = 0
        self._inbox = BoundedInbox(capacity=16)
        self._wake = threading.Event()
        self._stop = threading.Event()
        self._engine = None
        self._retained: RetainedFrontiers | None = None
        self._overflow_invalidation_pending = False
        self._topics = (
            "/rgbd_diagnostic/rgb",
            "/rgbd_diagnostic/depth",
            "/rgbd_diagnostic/rgb_info",
            "/rgbd_diagnostic/depth_info",
        )
        self._candidate_topic = "/rgbd_diagnostic/candidate"
        self._publisher = self.create_publisher(
            RgbdMotionCandidate, self._candidate_topic, qos_profile_sensor_data
        )
        self._input_subscriptions = []
        for index, topic in enumerate(self._topics):
            kind = BoundedRawImage if index < 2 else BoundedRawCameraInfo
            self._input_subscriptions.append(
                self.create_subscription(
                    kind,
                    topic,
                    lambda message, i=index: self._enqueue((i, message)),
                    qos_profile_sensor_data,
                )
            )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._fault = None
        self._fault_reported = False
        self._failure_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._failure_timer = self.create_timer(
            0.02, self._check_failure, clock=self._failure_clock
        )
        self._thread = threading.Thread(target=self._run_work, name="rgbd-coordinator", daemon=True)
        self._thread.start()

    def _check_failure(self):
        if self._fault is not None and not self._fault_reported:
            self._fault_reported = True
            raise self._fault

    def _run_work(self):
        try:
            self._work()
        except BaseException as exc:
            if isinstance(exc, Exception):
                self._fault = exc
            else:
                fault = RuntimeError("RGB-D coordinator aborted")
                fault.__cause__ = exc
                self._fault = fault

    def _enqueue(self, item):
        self._inbox.offer(item)
        self._wake.set()

    def _owner(self, topic, name):
        owners = self.get_publishers_info_by_topic(topic)
        if len(owners) != 1 or owners[0].node_name != name or owners[0].node_namespace != "/":
            return ""
        return bytes(owners[0].endpoint_gid).hex()

    def _invalid(self, reason):
        owner = self._owner(self._candidate_topic, "rgbd_candidate_producer")
        self._publish_invalid_with_owner(reason, owner)
        return True

    def _publish_invalid_with_owner(self, reason, owner):
        self._sequence += 1
        message = RgbdMotionCandidate()
        message.producer_boot_id = self._boot
        message.producer_gid = owner
        message.owner_generation = self._generation
        message.worker_generation = (
            1 if self._engine is None else self._engine.coordinator.worker.generation
        )
        message.sequence = self._sequence
        message.reason = reason[:64]
        message.valid = False
        message.hard_error_qualified = False
        self._publisher.publish(message)

    def _commit_candidate(self, result, output, token):
        geometry = self._binding.current

        def commit():
            if result.reason == "candidate" and (
                geometry is None
                or self._binding.current is not geometry
                or result.tf_receipts != (geometry, geometry)
            ):
                self._invalid("tf_geometry_changed")
                return
            self._publisher.publish(output)

        return self._inbox.commit_if_current(token, commit)

    def _sync_tf(self, stamp):
        try:
            actual = self._tf_buffer.lookup_transform(self._body, self._binding.camera, stamp)
            t, q = actual.transform.translation, actual.transform.rotation
            snapshot = TransformSnapshot(
                (
                    Edge(
                        actual.header.frame_id,
                        actual.child_frame_id,
                        Time.from_msg(actual.header.stamp).nanoseconds,
                        (t.x, t.y, t.z),
                        (q.x, q.y, q.z, q.w),
                    ),
                )
            )
            if not self._binding.observe(snapshot):
                raise ValueError("invalid_tf_geometry")
        except (TransformException, ValueError) as error:
            self._binding.revoke()
            if self._engine is not None:
                self._engine.revoke("tf_unavailable")
            self._invalid(str(error) or "tf_unavailable")
            return False
        if self._engine is None:
            retained = self._retained
            self._engine = RgbdProducerEngine(
                self._profile,
                self._binding,
                registration="registered",
                fresh_after_steady=time.monotonic(),
                generation=(
                    self._generation
                    if retained is None
                    else max(self._generation, retained.worker_generation + 1)
                ),
                sequence_floor=0 if retained is None else retained.sequence_frontier,
                retained=retained,
            )
            self._retained = None
        return True

    def _work(self):
        last_invalid = 0.0
        failure = None
        try:
            while not self._stop.is_set():
                self._wake.wait(0.02)
                self._wake.clear()
                now_ros, now_steady = self.get_clock().now().nanoseconds, time.monotonic()
                if not self._enabled:
                    if now_steady - last_invalid >= 0.1:
                        self._invalid("static_domain_unavailable")
                        last_invalid = now_steady
                    continue
                item = self._inbox.take()
                if item is not None:
                    self._wake.set()
                    generation, value, reason = item
                    if reason:
                        self._binding.revoke()
                        if reason == "overflow" and self._engine is not None:
                            self._retained = self._engine.close_for_recovery()
                            self._engine = None
                            self._generation += 1  # Explicit receiving-stream rebuild.
                        elif self._engine is not None:
                            self._engine.revoke(reason)
                        self._invalid(reason)
                    elif value is not None:
                        stream, message = value
                        owner = self._owner(self._topics[stream], "raw_receipt")
                        try:
                            header = capture_from_message(message.capture)
                            if owner and self._sync_tf(Time.from_msg(message.capture.stamp)):
                                self._engine.offer(
                                    stream,
                                    header,
                                    message,
                                    bridge_gid=owner,
                                    now_ros_ns=now_ros,
                                    now_steady=now_steady,
                                )
                            elif not owner and self._engine is not None:
                                self._engine.revoke("raw_owner_unavailable")
                        except (AttributeError, ValueError, TypeError, OverflowError):
                            if self._engine is not None:
                                self._engine.revoke("invalid_capture")
                    if not self._inbox.current(generation):
                        continue
                process_diagnostics().pump(time.monotonic())
                if self._engine is None:
                    continue
                token = self._inbox.snapshot()
                result = self._engine.poll(
                    now_ros_ns=self.get_clock().now().nanoseconds, now_steady=time.monotonic()
                )
                if result is None:
                    continue
                self._sequence += 1
                output = RgbdMotionCandidate()
                try:
                    fill_candidate(
                        output,
                        result,
                        producer_gid=self._owner(self._candidate_topic, "rgbd_candidate_producer"),
                        producer_boot=self._boot,
                        owner_generation=self._generation,
                        output_sequence=self._sequence,
                        body_frame=self._body,
                        edge_factory=TransformStamped,
                    )
                except (AssertionError, AttributeError, OverflowError, TypeError, ValueError):
                    self._inbox.commit_if_current(
                        token, lambda: self._invalid("candidate_encoding_invalid")
                    )
                else:
                    self._commit_candidate(result, output, token)
        except BaseException as exc:
            if not isinstance(exc, RCLError) or self.context.ok():
                failure = exc
                raise
        finally:
            if self._engine is not None:
                try:
                    self._engine.close()
                except BaseException as cleanup:
                    if failure is not None:
                        raise failure from cleanup
                    raise

    def destroy_node(self):
        self._stop.set()
        self._wake.set()
        self._thread.join(timeout=5)
        if self._thread.is_alive():
            raise RuntimeError("rgbd coordinator did not stop")
        destroyed = super().destroy_node()
        self._check_failure()
        return destroyed


def main():
    from senior_sensor_ingress.ros_lifecycle import run

    run(RgbdCandidateProducer, cli=True)
