"""Native ROS observations into the single T3 owner; no truth or drive feedback.

This adapter publishes an internal reference snapshot. Statistical model IDs are
never invented from ROS covariance. ROS/steady timestamps remain separate.
"""

import json
import math
import time
from collections import deque
from dataclasses import asdict, dataclass
from typing import Any
from uuid import uuid4

from senior_control.models import Pose2D
from senior_control.motion.fusion import FusionConfig, NumericalFusion
from senior_control.motion.observations import (
    AdmissionConfig,
    Covariance,
    CovarianceKind,
    Frame,
    Source,
    SourceStatus,
)
from senior_control.motion.sample_types import DeltaObservation, MotionSample, PointObservation
from senior_control.motion.source_admission import AdmissionOwner
from senior_control.motion.temporal import TemporalConfig, TemporalResolver


def publisher_gid(info: Any) -> bytes:
    # //Compatible: the project's MotionExecutor retains Humble's metadata dict;
    # newer rclpy delivers a MessageInfo object instead.
    raw = (
        info.get("publisher_gid", b"")
        if isinstance(info, dict)
        else getattr(info, "publisher_gid", b"")
    )
    return bytes(raw or b"")


@dataclass(frozen=True)
class PublisherOrigin:
    sample_gid: bytes | None
    topology_gid: bytes
    basis: str


def observation_origin(
    info: Any,
    endpoints: list[Any],
    expected_name: str,
    *,
    allow_topology_only: bool,
    expected_namespace: str = "/",
) -> PublisherOrigin | None:
    """Validate real metadata; never label a discovery GID as a sample GID."""
    if isinstance(info, dict) and info.get("native_receipt_authenticated") is True:
        # The private inherited channel binds configuration to the native
        # consumer, which checked sole-writer topology AND this MessageInfo GID.
        # Python discovery may lag native discovery; it supplies no sample ID.
        gid = publisher_gid(info)
        if (
            gid
            and any(gid)
            and info.get("native_writer_name") == expected_name
            and info.get("native_writer_namespace") == expected_namespace
        ):
            return PublisherOrigin(gid, gid, "native_message_info_and_writer_binding")
        return None
    if (
        len(endpoints) != 1
        or endpoints[0].node_name != expected_name
        or endpoints[0].node_namespace != expected_namespace
    ):
        return None
    topology_gid = bytes(endpoints[0].endpoint_gid)
    if not topology_gid or not any(topology_gid):
        return None
    gid = publisher_gid(info)
    if gid and any(gid):
        return (
            PublisherOrigin(gid, topology_gid, "per_sample_dds_gid")
            if gid == topology_gid
            else None
        )
    # //Compatible: Humble omits per-sample GID. Only the explicit isolated
    # simulation composition may rely on sole expected publisher discovery;
    # this has weaker provenance and remains labelled topology-only downstream.
    if allow_topology_only:
        return PublisherOrigin(None, topology_gid, "simulation_topology_only")
    return None


def stamp_seconds(stamp: Any) -> float:
    if not 0 <= stamp.nanosec < 1_000_000_000:
        raise ValueError("invalid native stamp")
    return float(stamp.sec) + float(stamp.nanosec) / 1e9


class NativeObservations:
    """One source admission/recovery owner. Configuration is supplied by ROS profile."""

    def __init__(
        self,
        admission: AdmissionConfig,
        temporal: TemporalConfig,
        fusion: FusionConfig,
        *,
        wheel_frame: str = "odom",
    ):
        # //Compatible: isolated simulation and pure callers retain odom.
        # ROS supplies its actual configured odometry frame for hardware.
        if not wheel_frame:
            raise ValueError("wheel_frame must be nonempty")
        self.wheel_frame = wheel_frame
        self.owner = AdmissionOwner(admission, event_capacity=4096)
        self.resolver = TemporalResolver(temporal)
        self.fusion = NumericalFusion(fusion)
        self.boot_id = uuid4().hex
        self._sequences = {source: 0 for source in Source}
        self._sequence = 0
        self._window: tuple[float, float] | None = None
        self._last_ros: float | None = None
        self._clock_epoch = 0
        self.poses: deque[dict[str, Any]] = deque(maxlen=64)

    def invalidate_source(self, source: Source) -> None:
        self.owner.interrupt_source(source)
        if source is Source.WHEEL:
            self.poses.clear()
        if source is Source.LASER:
            self._window = None

    def identity(
        self, source: Source, gid: bytes | PublisherOrigin, producer_epoch: str, now_ros: float
    ) -> tuple[str, int]:
        if not gid:
            raise ValueError("native source has no DDS identity")
        if self._last_ros is not None and now_ros < self._last_ros:
            self._clock_epoch += 1
            self._window = None
            self.poses.clear()
        self._last_ros = now_ros
        self._sequences[source] += 1
        # Preserve producer epoch AND DDS identity. Clock restart is a distinct
        # receiver context, not a replacement for either producer identity.
        provenance = (
            {
                "sample_gid": gid.sample_gid.hex() if gid.sample_gid is not None else None,
                "topology_gid": gid.topology_gid.hex(),
                "basis": gid.basis,
            }
            if isinstance(gid, PublisherOrigin)
            else {"sample_gid": gid.hex(), "basis": "per_sample_dds_gid"}
        )
        return json.dumps(
            [self.boot_id, provenance, producer_epoch, self._clock_epoch]
        ), self._sequences[source]

    def ingest(self, sample: MotionSample, now_ros: float, now_steady: float) -> bool:
        decision = self.owner.ingest(sample, now_ros=now_ros, now_steady=now_steady)
        snapshot = self.owner.snapshot(
            after_cursor=self.resolver.cursor, now_ros=now_ros, now_steady=now_steady
        )
        self.resolver.consume(snapshot)
        if decision.accepted and sample.source is Source.LASER:
            self._window = (sample.start_ros, sample.end_ros)
        return decision.accepted

    # //Compatible: isolated pure callers inject one clock; ROS facades separately
    # supply handling time so queued native receipts never refresh evidence TTL.
    def wheel(
        self,
        msg: Any,
        gid: bytes | PublisherOrigin,
        now_ros: float,
        received: float,
        *,
        now_steady: float | None = None,
    ) -> bool:
        if msg.child_frame_id != "base_link":
            raise ValueError("wheel twist is not in body frame")
        epoch, seq = self.identity(Source.WHEEL, gid, "", now_ros)
        sample = PointObservation(
            Source.WHEEL,
            Frame.BODY,
            stamp_seconds(msg.header.stamp),
            received,
            epoch,
            seq,
            1.0,
            SourceStatus.VALID,
            (
                float(msg.twist.twist.linear.x),
                float(msg.twist.twist.angular.z),
            ),
            None,
            None,
            Pose2D(0, 0, 0),
        )
        accepted = self.ingest(sample, now_ros, received if now_steady is None else now_steady)
        if accepted and msg.header.frame_id == self.wheel_frame and hasattr(msg, "pose"):
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            if (
                all(math.isfinite(x) for x in (p.x, p.y, q.x, q.y, q.z, q.w))
                and abs(q.x) < 1e-6
                and abs(q.y) < 1e-6
                and abs(q.z * q.z + q.w * q.w - 1) < 1e-6
            ):
                self.poses.append(
                    {
                        "stamp_ros": sample.stamp_ros,
                        "received_steady": received,
                        "epoch": epoch,
                        "x": float(p.x),
                        "y": float(p.y),
                        "yaw": math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z),
                    }
                )
        return accepted

    def imu(
        self,
        msg: Any,
        gid: bytes | PublisherOrigin,
        now_ros: float,
        received: float,
        *,
        now_steady: float | None = None,
    ) -> bool:
        # A non-body IMU must be transformed by the ROS facade before admission.
        if msg.header.frame_id != "base_link":
            raise ValueError("IMU requires explicit body rotation")
        epoch, seq = self.identity(Source.IMU, gid, "", now_ros)
        sample = PointObservation(
            Source.IMU,
            Frame.BODY,
            stamp_seconds(msg.header.stamp),
            received,
            epoch,
            seq,
            1.0,
            SourceStatus.VALID,
            (float(msg.angular_velocity.z),),
            None,
            None,
            Pose2D(0, 0, 0),
        )
        return self.ingest(sample, now_ros, received if now_steady is None else now_steady)

    def laser(
        self,
        msg: Any,
        gid: bytes | PublisherOrigin,
        now_ros: float,
        received: float,
        *,
        now_steady: float | None = None,
    ) -> bool:
        if msg.header.frame_id != "base_link":
            raise ValueError("scan delta is not body-frame")
        epoch, seq = self.identity(Source.LASER, gid, str(msg.source_epoch), now_ros)
        sample: MotionSample
        if not msg.valid:
            sample = PointObservation(
                Source.LASER,
                Frame.BODY,
                stamp_seconds(msg.header.stamp),
                received,
                epoch,
                seq,
                1.0,
                SourceStatus.UNAVAILABLE,
                (float(msg.body_delta.x), float(msg.body_delta.y), float(msg.body_delta.theta)),
                None,
                None,
                Pose2D(0, 0, 0),
            )
            return self.ingest(sample, now_ros, received if now_steady is None else now_steady)
        covariance = Covariance(
            tuple(float(x) for x in msg.covariance),
            CovarianceKind.FULL,
            "native ScanMotion displacement covariance; statistical model unverified",
            None,
        )
        sample = DeltaObservation(
            Source.LASER,
            Frame.BODY,
            stamp_seconds(msg.interval_start),
            stamp_seconds(msg.header.stamp),
            received,
            epoch,
            seq,
            float(msg.response),
            SourceStatus.VALID if msg.valid else SourceStatus.UNAVAILABLE,
            Pose2D(float(msg.body_delta.x), float(msg.body_delta.y), float(msg.body_delta.theta)),
            covariance,
            None,
            Pose2D(0, 0, 0),
        )
        return self.ingest(sample, now_ros, received if now_steady is None else now_steady)

    def snapshot(self, now_ros: float, now_steady: float) -> dict[str, Any]:
        snapshot = self.owner.snapshot(
            after_cursor=self.resolver.cursor, now_ros=now_ros, now_steady=now_steady
        )
        self.resolver.consume(snapshot)
        a, b = self._window if self._window else (now_ros - 0.1, now_ros)
        result = self.fusion.update(self.resolver.resolve(a, b, snapshot), snapshot, self.resolver)
        self._sequence += 1
        return {
            "schema": 1,
            "boot_id": self.boot_id,
            "sequence": self._sequence,
            "sampled_steady": now_steady,
            "sampled_ros": now_ros,
            "reference": asdict(result),
            "poses": list(self.poses),
            "sources": [
                {
                    "source": source.value,
                    "ready": view.ready,
                    "sample": asdict(view.observation.original),
                    "valid_until_steady": view.observation.valid_until_steady,
                }
                for source, view in snapshot.sources
            ],
        }


def strict_reference_json(value: dict[str, Any]) -> str:
    """Undefined statistical values remain null; never turn them into zero."""

    def clean(obj: Any) -> Any:
        if isinstance(obj, float) and not math.isfinite(obj):
            return None
        if isinstance(obj, dict):
            return {k: clean(v) for k, v in obj.items()}
        if isinstance(obj, (tuple, list)):
            return [clean(v) for v in obj]
        return obj

    return json.dumps(clean(value), allow_nan=False, separators=(",", ":"))


class NativeObservationRosBridge:
    """ROS facade attached to motion_fusion; creates no actuator publisher."""

    def __init__(self, node: Any, profile: Any):
        from nav_msgs.msg import Odometry
        from rclpy.clock import Clock, ClockType
        from rclpy.qos import qos_profile_sensor_data
        from senior_motion_msgs.msg import ScanMotion
        from sensor_msgs.msg import Imu
        from std_msgs.msg import String

        self.node = node
        self.profile = profile
        self.allow_topology_only = bool(profile.simulation_only and profile.unvalidated)
        self.core = NativeObservations(
            profile.admission(),
            profile.temporal(),
            profile.fusion(),
            wheel_frame=str(node.get_parameter("wheel_frame").value),
        )
        self.publisher = node.create_publisher(String, "/motion/core_reference", 10)
        self._message_type = String
        self._warn_at = 0.0
        self._subscriptions = []
        for cls, topic, handler in [
            (Odometry, "/wheel/odom", self.core.wheel if profile.simulation_only else self._wheel),
            (Imu, "/imu/data", self._imu),
            (
                ScanMotion,
                "/motion/scan",
                self.core.laser if profile.simulation_only else self._laser,
            ),
        ]:
            callback = self._callback(handler, topic)
            if profile.simulation_only:
                # //Compatible: keep the isolated simulation subscription path.
                subscription = node.create_subscription(
                    cls, topic, callback, qos_profile_sensor_data
                )
            else:
                from senior_robot.native_receipt import subscribe

                source = {
                    "/wheel/odom": Source.WHEEL,
                    "/imu/data": Source.IMU,
                    "/motion/scan": Source.LASER,
                }[topic]
                writer = {
                    "/wheel/odom": profile.wheel_publisher,
                    "/imu/data": profile.imu_publisher,
                    "/motion/scan": "scan_motion_node",
                }[topic]

                def invalidate(source=source):
                    self.core.invalidate_source(source)

                subscription = subscribe(
                    node,
                    cls,
                    topic,
                    callback,
                    writer=writer,
                    namespace=profile.source_namespace if source is not Source.LASER else "/",
                    qos="sensor" if source is not Source.LASER else "reliable",
                    invalidate=invalidate,
                )
            self._subscriptions.append(subscription)
        self._timer = node.create_timer(
            0.05, self._publish, clock=Clock(clock_type=ClockType.STEADY_TIME)
        )

    def _callback(self, handler: Any, topic: str) -> Any:
        def receive(msg: Any, info: Any) -> None:

            from tf2_ros import TransformException

            from senior_robot.native_receipt import received_steady

            now = received_steady(info)
            expected = {
                "/wheel/odom": self.profile.wheel_publisher,
                "/imu/data": self.profile.imu_publisher,
                "/motion/scan": "scan_motion_node",
            }
            origin = observation_origin(
                info,
                self.node.get_publishers_info_by_topic(self.node.resolve_topic_name(topic)),
                expected[topic],
                allow_topology_only=self.allow_topology_only,
                expected_namespace=self.profile.source_namespace
                if topic != "/motion/scan"
                else "/",
            )
            if origin is None:
                if now - self._warn_at > 1:
                    self._warn_at = now
                    self.node.get_logger().warning(
                        f"native motion source rejected on {topic}: expected sole "
                        f"publisher {expected[topic]} with per-sample DDS identity; "
                        "topology-only fallback is confined to isolated simulation"
                    )
                return
            try:
                handler(msg, origin, self.node.get_clock().now().nanoseconds / 1e9, now)
            except (ValueError, TypeError, AttributeError, OverflowError, TransformException):
                if now - self._warn_at > 1:
                    self._warn_at = now
                    self.node.get_logger().warning("native motion observation unavailable")

        from senior_robot.motion_lifecycle import with_dds_info

        return with_dds_info(receive)

    def _laser(
        self, msg: Any, gid: bytes | PublisherOrigin, now_ros: float, received: float
    ) -> bool:
        return self.core.laser(msg, gid, now_ros, received, now_steady=time.monotonic())

    def _wheel(
        self, msg: Any, gid: bytes | PublisherOrigin, now_ros: float, received: float
    ) -> bool:
        # //Compatible: WHEELTEC defaults to base_footprint; the common core uses
        # base_link. Transform the actual stamped twist/pose, never relabel it.
        if msg.child_frame_id == "base_link":
            return self.core.wheel(msg, gid, now_ros, received, now_steady=time.monotonic())
        from types import SimpleNamespace as N

        from rclpy.time import Time

        from senior_control.motion.kinematics import compose, inverse

        transform = self.node._tf.lookup_transform(
            "base_link", msg.child_frame_id, Time.from_msg(msg.header.stamp)
        )
        q = transform.transform.rotation
        if (
            not all(math.isfinite(x) for x in (q.x, q.y, q.z, q.w))
            or abs(q.x) > 1e-6
            or abs(q.y) > 1e-6
            or abs(q.z * q.z + q.w * q.w - 1) > 1e-6
        ):
            raise ValueError("wheel body transform must be planar and normalized")
        t = transform.transform.translation
        body_from_child = Pose2D(t.x, t.y, math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z))
        v, w = msg.twist.twist.linear, msg.twist.twist.angular
        c, sn = math.cos(body_from_child.yaw), math.sin(body_from_child.yaw)
        vx = c * v.x - sn * v.y + w.z * t.y
        p, rotation = msg.pose.pose.position, msg.pose.pose.orientation
        if (
            not all(math.isfinite(x) for x in (rotation.x, rotation.y, rotation.z, rotation.w))
            or abs(rotation.x) > 1e-6
            or abs(rotation.y) > 1e-6
            or abs(rotation.z * rotation.z + rotation.w * rotation.w - 1) > 1e-6
        ):
            raise ValueError("wheel pose must be planar and normalized")
        odom_child = Pose2D(
            p.x, p.y, math.atan2(2 * rotation.w * rotation.z, 1 - 2 * rotation.z * rotation.z)
        )
        odom_body = compose(odom_child, inverse(body_from_child))
        normalized = N(
            header=msg.header,
            child_frame_id="base_link",
            twist=N(twist=N(linear=N(x=vx), angular=N(z=w.z))),
            pose=N(
                pose=N(
                    position=N(x=odom_body.x, y=odom_body.y),
                    orientation=N(
                        x=0.0, y=0.0, z=math.sin(odom_body.yaw / 2), w=math.cos(odom_body.yaw / 2)
                    ),
                )
            ),
        )
        return self.core.wheel(normalized, gid, now_ros, received, now_steady=time.monotonic())

    def _imu(self, msg: Any, gid: bytes | PublisherOrigin, now_ros: float, received: float) -> bool:
        from types import SimpleNamespace

        from rclpy.time import Time

        if msg.header.frame_id == "base_link":
            return self.core.imu(msg, gid, now_ros, received, now_steady=time.monotonic())
        tf = self.node._tf.lookup_transform(
            "base_link", msg.header.frame_id, Time.from_msg(msg.header.stamp)
        )
        q = tf.transform.rotation
        values = (q.x, q.y, q.z, q.w)
        if not all(math.isfinite(x) for x in values) or abs(sum(x * x for x in values) - 1) > 1e-6:
            raise ValueError("IMU transform quaternion")
        v = msg.angular_velocity
        z = (
            2 * (q.x * q.z - q.y * q.w) * v.x
            + 2 * (q.y * q.z + q.x * q.w) * v.y
            + (1 - 2 * (q.x * q.x + q.y * q.y)) * v.z
        )
        normalized = SimpleNamespace(
            header=SimpleNamespace(stamp=msg.header.stamp, frame_id="base_link"),
            angular_velocity=SimpleNamespace(z=z),
        )
        return self.core.imu(normalized, gid, now_ros, received, now_steady=time.monotonic())

    def _publish(self) -> None:
        import time

        value = self.core.snapshot(self.node.get_clock().now().nanoseconds / 1e9, time.monotonic())
        msg = self._message_type()
        msg.data = strict_reference_json(value)
        self.publisher.publish(msg)
