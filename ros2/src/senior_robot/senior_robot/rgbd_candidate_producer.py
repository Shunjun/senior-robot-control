"""Isolated SIM RGB-D diagnostic producer; native egress owns authentication."""

import threading
import time
import uuid
from contextlib import suppress

from rclpy._rclpy_pybind11 import RCLError  # //Compatible: Humble exposes RCLError here.
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from senior_motion_msgs.msg import (
    BoundedRawCameraInfo,
    BoundedRawImage,
    BoundedStaticTf,
    RgbdMotionCandidate,
    StaticTransformEdge,
)

from senior_control.vision.independent_motion import Profile
from senior_control.vision.raw_capture import capture_from_message
from senior_control.vision.rgbd_bounded_inbox import BoundedInbox
from senior_control.vision.rgbd_candidate_codec import fill_candidate
from senior_control.vision.rgbd_producer_engine import RetainedFrontiers, RgbdProducerEngine
from senior_control.vision.static_tf_binding import StaticBinding
from senior_control.vision.static_tf_wire import observe_verified_tf


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
        self._tf_topic = "/rgbd_diagnostic/tf"
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
        self._tf_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._tf_epoch = 1
        self._tf_epoch_lock = threading.Lock()
        self._tf_subscription = self._create_tf_subscription(self._tf_epoch)
        self._tf_rearm = self.create_guard_condition(self._replace_tf_subscription)
        self._fault = None
        self._fault_reported = False
        self._failure_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._failure_timer = self.create_timer(
            0.02, self._check_failure, clock=self._failure_clock
        )
        self._thread = threading.Thread(target=self._run_work, name="rgbd-coordinator", daemon=True)
        self._thread.start()

    def _create_tf_subscription(self, epoch):
        return self.create_subscription(
            BoundedStaticTf,
            self._tf_topic,
            lambda message: self._enqueue((4, epoch, message)),
            self._tf_qos,
        )

    def _replace_tf_subscription(self):
        if self._stop.is_set():
            return
        old = self._tf_subscription
        self.destroy_subscription(old)
        with self._tf_epoch_lock:
            epoch = self._tf_epoch
        self._tf_subscription = self._create_tf_subscription(epoch)

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
        if item[0] == 4:
            with self._tf_epoch_lock:
                if item[1] != self._tf_epoch:
                    return
                self._inbox.offer(item)
        else:
            self._inbox.offer(item)
        self._wake.set()

    def _owner(self, topic, name):
        owners = self.get_publishers_info_by_topic(topic)
        if len(owners) != 1 or owners[0].node_name != name or owners[0].node_namespace != "/":
            return ""
        return bytes(owners[0].endpoint_gid).hex()

    def _invalid(self, reason):
        owner = self._owner(self._candidate_topic, "rgbd_candidate_producer")
        if not owner:
            return False
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
        owner = self._owner(self._candidate_topic, "rgbd_candidate_producer")
        if not owner or owner != output.producer_gid:
            return False
        binding = self._binding
        current = binding.current
        first, admitted = binding.first_receipt, binding.admitted_from
        dependencies_valid = True
        if result.reason == "candidate":
            dependencies_valid = (
                current is not None
                and self._owner(self._tf_topic, "static_tf_receipt") == current.relay_gid
                and result.tf_receipts == (first, first)
                and result.tf_admitted_from == (admitted, admitted)
                and len(result.headers) == 2
                and all(len(group) == 4 for group in result.headers)
            )
            if dependencies_valid:
                for index, topic in enumerate(self._topics):
                    raw_owner = self._owner(topic, "raw_receipt")
                    if not raw_owner or any(
                        group[index].bridge_gid != raw_owner for group in result.headers
                    ):
                        dependencies_valid = False
                        break

        def commit():
            if result.reason != "candidate":
                self._publisher.publish(output)
                return
            if (
                not dependencies_valid
                or binding.current is not current
                or binding.first_receipt != first
                or binding.admitted_from != admitted
            ):
                self._publish_invalid_with_owner("dependency_unavailable", owner)
                return
            final_ros = self.get_clock().now().nanoseconds
            final_steady = time.monotonic()
            if any(
                not (0 <= final_ros - h.stamp_ns <= 300_000_000)
                or not (0 <= final_ros - h.received_ros_ns <= 300_000_000)
                or not (0 <= final_steady - h.received_steady <= 0.3)
                for group in result.headers
                for h in group
            ):
                self._publish_invalid_with_owner("candidate_expired", owner)
                return
            self._publisher.publish(output)

        return self._inbox.commit_if_current(token, commit)

    def _work(self):
        last_invalid = 0.0
        pending_tf = {}
        allow_tf_reconfirm = False
        last_ros = 0
        last_steady = 0.0
        failure = None
        try:
            while not self._stop.is_set():
                self._wake.wait(0.02)
                self._wake.clear()
                now_ros = self.get_clock().now().nanoseconds
                now_steady = time.monotonic()
                if last_ros > 0 and (now_ros < last_ros or now_steady < last_steady):
                    self._binding.revoke()
                    self._invalid("clock_invalid")
                    raise RuntimeError("RGB-D clock moved backwards; restart the diagnostic chain")
                last_ros, last_steady = now_ros, now_steady
                if not self._enabled:
                    if now_steady - last_invalid >= 0.1:
                        self._invalid("static_domain_unavailable")
                        last_invalid = now_steady
                    continue
                if self._overflow_invalidation_pending:
                    if not self._invalid("overflow"):
                        continue
                    self._overflow_invalidation_pending = False
                current_tf = self._binding.current
                if (
                    current_tf is not None
                    and self._owner(self._tf_topic, "static_tf_receipt") != current_tf.relay_gid
                ):
                    self._binding.revoke()
                    if self._engine is not None:
                        self._engine.revoke("static_tf_unavailable")
                    self._invalid("static_tf_unavailable")
                item = self._inbox.take()
                if item is not None:
                    self._wake.set()
                if item is not None:
                    generation, value, reason = item
                    if reason:
                        pending_tf.clear()
                        if reason == "overflow":
                            self._binding.suspend()
                            allow_tf_reconfirm = True
                            with self._tf_epoch_lock:
                                self._tf_epoch += 1
                            if self._engine is not None:
                                self._retained = self._engine.close_for_recovery()
                                self._engine = None
                            self._generation += 1
                            self._overflow_invalidation_pending = not self._invalid(reason)
                            self._tf_rearm.trigger()
                        else:
                            self._binding.revoke()
                            allow_tf_reconfirm = False
                            if self._engine is not None:
                                self._engine.revoke(reason)
                            self._invalid(reason)
                    elif value is not None:
                        stream = value[0]
                        if stream == 4:
                            _, epoch, message = value
                            if epoch != self._tf_epoch:
                                continue
                            gid = message.bridge_gid
                            # Keep one status per endpoint while graph discovery catches up.
                            # The cap also bounds ambiguous or obsolete DDS endpoints.
                            previous = pending_tf.get(gid)
                            if previous is None or (
                                message.generation,
                                message.sequence,
                                not message.valid,
                            ) >= (
                                previous[1].generation,
                                previous[1].sequence,
                                not previous[1].valid,
                            ):
                                pending_tf.pop(gid, None)
                                if len(pending_tf) == 4:
                                    pending_tf.pop(next(iter(pending_tf)))
                                pending_tf[gid] = (generation, message)
                        elif self._engine is not None:
                            _, message = value
                            owner = self._owner(self._topics[stream], "raw_receipt")
                            if owner:
                                try:
                                    header = capture_from_message(message.capture)
                                    self._engine.offer(
                                        stream,
                                        header,
                                        message,
                                        bridge_gid=owner,
                                        now_ros_ns=now_ros,
                                        now_steady=now_steady,
                                    )
                                except (AttributeError, ValueError, TypeError, OverflowError):
                                    self._engine.revoke("invalid_capture")
                            else:
                                self._engine.revoke("raw_owner_unavailable")
                    if not self._inbox.current(generation):
                        continue
                if pending_tf:
                    owner = self._owner(self._tf_topic, "static_tf_receipt")
                    pending = pending_tf.pop(owner, None)
                    if pending is not None and self._inbox.current(pending[0]):
                        admitted = self._observe_tf(
                            pending[1], owner, allow_tf_reconfirm, now_ros, now_steady
                        )
                        if admitted:
                            allow_tf_reconfirm = False
                if self._engine is not None:
                    token = self._inbox.snapshot()
                    now_ros = self.get_clock().now().nanoseconds
                    now_steady = time.monotonic()
                    result = self._engine.poll(now_ros_ns=now_ros, now_steady=now_steady)
                    if result is not None:
                        owner = self._owner(self._candidate_topic, "rgbd_candidate_producer")
                        if owner:
                            self._sequence += 1
                            output = RgbdMotionCandidate()
                            try:
                                fill_candidate(
                                    output,
                                    result,
                                    producer_gid=owner,
                                    producer_boot=self._boot,
                                    owner_generation=self._generation,
                                    output_sequence=self._sequence,
                                    body_frame=self._body,
                                    edge_factory=StaticTransformEdge,
                                )
                            except (
                                AssertionError,
                                AttributeError,
                                OverflowError,
                                TypeError,
                                ValueError,
                            ):
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

    def _observe_tf(self, message, owner, allow_tf_reconfirm, now_ros, now_steady):
        prior = self._binding.current
        accepted = observe_verified_tf(
            self._binding,
            message,
            current_relay_gid=owner,
            allow_suspended_reconfirm=allow_tf_reconfirm,
        )
        if not accepted:
            if prior is None or self._binding.current is not prior:
                if self._engine is not None:
                    self._engine.revoke("static_tf_unavailable")
                self._invalid("static_tf_unavailable")
            return False
        if self._binding.admitted_from is None or self._engine is not None:
            return False

        retained = self._retained
        self._engine = RgbdProducerEngine(
            self._profile,
            self._binding,
            registration="registered",
            fresh_after_ros_ns=now_ros,
            fresh_after_steady=now_steady,
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
    from senior_robot.motion_lifecycle import run

    run(RgbdCandidateProducer, cli=True)
