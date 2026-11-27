"""Opt-in authenticated GPS odometry and RGB-D measurement adapters.

GPS Odometry obeys ROS semantics: pose in header frame, twist in child frame.
RGB-D is a statistical input, never a hard error or stopping-speed certificate.
No publisher, topic discovery, or assumed calibration enables these inputs.
"""

import json
import math
import time
from typing import Any

from senior_perception.vision.raw_capture import capture_from_message
from senior_sensor_ingress.native_receipt import received_sample

from senior_motion_estimation.estimator import MotionObservation, Origin
from senior_motion_estimation.input_config import OptionalMotionInput
from senior_motion_estimation.motion_observation_adapter import observation_origin, stamp_seconds
from senior_motion_estimation.rgbd_body_motion import body_pose_increment, planar_body_twist
from vehicle_core.sample_time import StampInterval
from vehicle_core.values import Pose2D


def static_matrix(status: Any, base: str, camera: str) -> Any:
    import numpy as np

    if not status.transforms:
        raise ValueError("RGB-D static calibration unavailable")
    edges = {e.child_frame_id: e for e in status.transforms}
    if len(edges) != len(status.transforms):
        raise ValueError("ambiguous static frame parent")
    result = np.eye(4)
    visited = set()
    while camera != base:
        if camera in visited or camera not in edges:
            raise ValueError("missing/cyclic camera calibration chain")
        visited.add(camera)
        edge = edges[camera]
        q = edge.transform.rotation
        x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
        if (
            not all(math.isfinite(v) for v in (x, y, z, w))
            or abs(x * x + y * y + z * z + w * w - 1) > 1e-6
        ):
            raise ValueError("invalid static rotation")
        matrix = np.eye(4)
        matrix[:3, :3] = [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
        t = edge.transform.translation
        matrix[:3, 3] = (t.x, t.y, t.z)
        result = matrix @ result
        camera = edge.header.frame_id
    if not np.isfinite(result).all():
        raise ValueError("nonfinite static calibration")
    return result


def rgbd_observation(message: Any) -> MotionObservation:
    import numpy as np

    body = message.body
    if not message.owner.valid or message.owner.owner_lost or not body.valid:
        raise ValueError("RGB-D source unavailable")
    roots = []
    # Preserve all original captures; worker completion cannot renew any dependency.
    for role, header in zip(
        ("rgb", "depth", "rgb_info", "depth_info") * 2,
        (
            body.first_rgb,
            body.first_depth,
            body.first_rgb_info,
            body.first_depth_info,
            body.second_rgb,
            body.second_depth,
            body.second_rgb_info,
            body.second_depth_info,
        ),
        strict=True,
    ):
        if not header.valid:
            raise ValueError("invalid original RGB-D capture")
        roots.append(
            Origin(
                "camera:" + role,
                json.dumps(
                    {"boot_id": header.bridge_boot_id, "generation": header.capture_generation}
                ),
                int(header.sequence),
                capture_from_message(header).time,
            )
        )
    first = static_matrix(body.first_tf, body.body_frame, body.first_rgb.frame_id)
    second = static_matrix(body.second_tf, body.body_frame, body.second_rgb.frame_id)
    start, end = stamp_seconds(body.interval_start), stamp_seconds(body.interval_end)
    delta = body_pose_increment(np.asarray(body.camera_transform).reshape(4, 4), first, second)
    # Initial software planar-domain tolerances, not an installation certificate.
    values = planar_body_twist(
        delta,
        dt=((roots[4].time.stamp_ns - roots[0].time.stamp_ns) / 1e9),
        max_tilt_rad=0.1,
        max_vertical_m=0.1,
    )
    if body.matched <= 0 or not 0 < body.inliers <= body.matched:
        raise ValueError("invalid RGB-D quality")
    return MotionObservation(
        "rgbd",
        values,
        (start + end) / 2,
        tuple(roots),
        min(1.0, body.inliers / body.matched),
        independent=(True, True, True),
        measured_steady=roots[4].measurement_steady,
        interval_half_s=((roots[4].time.stamp_ns - roots[0].time.stamp_ns) / 1e9) / 2,
        stamp_ns=roots[4].time.stamp_ns,
        measurement_interval=StampInterval(roots[0].time.stamp_ns, roots[4].time.stamp_ns),
        measurement_delta=Pose2D(
            float(delta[0][3]),
            float(delta[1][3]),
            math.atan2(float(delta[1][0]), float(delta[0][0])),
        ),
    )


class OptionalMotionInputs:
    def __init__(self, node: Any, profile: Any, runtime: Any):
        self.node, self.runtime = node, runtime
        self._gps_family = None if profile.gps_input is None else profile.gps_input.origin_family
        if profile.gps_input is not None and self._gps_family is None:
            raise ValueError("GPS input requires original raw acquisition family")
        self._course_frame = None if profile.gps_input is None else profile.gps_input.course_frame
        self._subscriptions = []
        from nav_msgs.msg import Odometry
        from senior_interfaces.msg import FencedRgbdCandidate

        for name, cls, handler in (
            ("gps", Odometry, self.gps),
            ("rgbd", FencedRgbdCandidate, self.rgbd),
        ):
            setting = getattr(profile, name + "_input", None)
            if setting is None:
                continue
            self._subscribe(name, cls, handler, setting)

    def _subscribe(self, name: str, cls: Any, handler: Any, setting: OptionalMotionInput) -> None:
        from senior_sensor_ingress.native_receipt import subscribe, with_dds_info

        def receive(message: Any, info: Any) -> None:
            origin = observation_origin(
                info,
                self.node.get_publishers_info_by_topic(setting.topic),
                setting.publisher,
                expected_namespace=setting.namespace,
            )
            if origin is None:
                return
            try:
                handler(message, info, origin)
            except (ValueError, TypeError, AttributeError, OverflowError):
                # A bad optional source never clears other motion sources.
                self._invalidate(name)

        self._subscriptions.append(
            subscribe(
                self.node,
                cls,
                setting.topic,
                with_dds_info(receive),
                writer=setting.publisher,
                namespace=setting.namespace,
                qos="reliable",
                invalidate=lambda: self._invalidate(name),
            )
        )

    def _invalidate(self, name):
        self.runtime.estimator.invalidate(name)
        if name == "gps":
            self.runtime.gps_course.reset()
            self.runtime.gps_heading = None

    def rgbd(self, message: Any, info: Any, origin: Any) -> None:
        self.runtime.ingest_observation(
            rgbd_observation(message),
            self.node.get_clock().now().nanoseconds / 1e9,
            time.monotonic(),
        )

    def gps(self, message: Any, info: Any, origin: Any) -> None:
        sample = received_sample(info)
        # An Odometry twist is expressed in child_frame_id, never silently ENU.
        # //Compatible: body-frame replay adapters may have no course binding.
        if getattr(self, "_course_frame", None) is not None:
            self._course(message, info, origin, sample)
            return
        from rclpy.time import Time
        from tf2_ros import TransformException

        v = message.twist.twist.linear
        w = message.twist.twist.angular
        vx, vy = float(v.x), float(v.y)
        transform_stamp = None
        body_from_child = None
        if message.child_frame_id != "base_link":
            try:
                tf = self.node._tf.lookup_transform(
                    "base_link", message.child_frame_id, Time.from_msg(message.header.stamp)
                )
            except TransformException as exc:
                raise ValueError("GPS body transform unavailable") from exc
            body_from_child = tf
            transform_stamp = stamp_seconds(tf.header.stamp)
            q, t = tf.transform.rotation, tf.transform.translation
            if abs(q.x) > 1e-6 or abs(q.y) > 1e-6 or abs(q.z * q.z + q.w * q.w - 1) > 1e-6:
                raise ValueError("GPS twist requires planar body transform")
            yaw = math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z)
            vx, vy = (
                math.cos(yaw) * vx - math.sin(yaw) * vy + w.z * t.y,
                math.sin(yaw) * vx + math.cos(yaw) * vy - w.z * t.x,
            )
        stamp = stamp_seconds(message.header.stamp)
        ros = self.node.get_clock().now().nanoseconds / 1e9
        root = self._gps_root(message, info, origin, ros, sample)
        roots = (root,)
        if transform_stamp:
            roots += (
                Origin(
                    "gps_transform",
                    root.epoch,
                    root.sequence,
                    root.time,
                ),
            )
        self.runtime.ingest_observation(
            MotionObservation(
                "gps",
                (vx, vy, None),
                stamp,
                roots,
                independent=(self._gps_family != "wheel", self._gps_family != "wheel", False),
                measured_steady=root.measurement_steady,
                stamp_ns=root.time.stamp_ns,
            ),
            ros,
            time.monotonic(),
        )
        self._gps_pose(message, roots, body_from_child)

    def _gps_root(self, message, info, origin, ros, sample):
        return Origin(
            self._gps_family,
            json.dumps(
                {
                    "boot_id": info.get("receipt_boot_id", "legacy-receiver"),
                    "generation": int(info["receipt_generation"]),
                }
            ),
            int(info["receipt_sequence"]),
            sample,
        )

    def _gps_pose(
        self, message: Any, roots: tuple[Origin, ...], body_from_child: Any = None
    ) -> None:
        from rclpy.time import Time
        from tf2_ros import TransformException

        from senior_motion_estimation.kinematics import compose, inverse
        from senior_motion_estimation.pose_estimator import PoseObservation
        from vehicle_core.values import Pose2D

        def planar(p, q):
            values = (p.x, p.y, q.x, q.y, q.z, q.w)
            if (
                not all(math.isfinite(v) for v in values)
                or abs(q.x) > 1e-6
                or abs(q.y) > 1e-6
                or abs(q.z * q.z + q.w * q.w - 1) > 1e-6
            ):
                raise ValueError("GPS pose orientation unavailable")
            return Pose2D(p.x, p.y, math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z))

        try:
            pose = planar(message.pose.pose.position, message.pose.pose.orientation)
            if body_from_child is not None:
                pose = compose(
                    pose,
                    inverse(
                        planar(
                            body_from_child.transform.translation,
                            body_from_child.transform.rotation,
                        )
                    ),
                )
            frame = self.runtime.pose_estimator.frame
            if message.header.frame_id != frame:
                tf = self.node._tf.lookup_transform(
                    frame, message.header.frame_id, Time.from_msg(message.header.stamp)
                )
                pose = compose(planar(tf.transform.translation, tf.transform.rotation), pose)
                tf_stamp = stamp_seconds(tf.header.stamp)
                if tf_stamp:
                    root = roots[0]
                    roots += (
                        Origin(
                            "gps_pose_transform",
                            root.epoch,
                            root.sequence,
                            root.time,
                        ),
                    )
            self.runtime.pose_estimator.ingest(
                PoseObservation(
                    "gps_pose",
                    pose,
                    stamp_seconds(message.header.stamp),
                    roots,
                    measurement_steady=roots[0].measurement_steady,
                    stamp_ns=roots[0].time.stamp_ns,
                    frame=frame,
                )
            )
        except (ValueError, AttributeError, TransformException):
            # Missing absolute heading/frame removes only this pose contribution.
            return

    def _course(self, message: Any, info: Any, origin: Any, sample) -> None:
        """Heading-only ENU path; no static TF can manufacture antenna heading."""
        from senior_motion_estimation.gps_course import CourseFix

        runtime = self.runtime
        runtime.gps_heading = None
        if (
            message.header.frame_id != self._course_frame
            or message.child_frame_id != self._course_frame
            or self._gps_family == "wheel"
        ):
            runtime.gps_course.reset()
            return
        ros = self.node.get_clock().now().nanoseconds / 1e9
        steady = time.monotonic()
        root = self._gps_root(message, info, origin, ros, sample)
        estimate = runtime.latest
        wheel = runtime.wheel_linear(steady, with_deadline=True)
        if estimate is None or wheel is None or estimate.independent_axes is None:
            runtime.gps_course.reset()
            return
        gyro = estimate.independent_axes[1]
        if not gyro.observed or "imu" not in gyro.sources or steady >= gyro.deadline_steady:
            runtime.gps_course.reset()
            return

        def sigma(covariance):
            xx, yy, xy, yx = (float(covariance[i]) for i in (0, 7, 1, 6))
            if (
                not all(math.isfinite(v) for v in (xx, yy, xy, yx))
                or xx <= 0
                or yy <= 0
                or abs(xy - yx) > 1e-9
                or xy * xy > xx * yy
            ):
                return None
            return math.sqrt(max(xx + abs(xy), yy + abs(xy)))

        p, v = message.pose.pose.position, message.twist.twist.linear
        heading = runtime.gps_course.observe(
            CourseFix(
                float(p.x),
                float(p.y),
                sigma(message.pose.covariance),
                float(v.x),
                float(v.y),
                sigma(message.twist.covariance),
                root,
            ),
            wheel_vx=wheel[0],
            imu_wz=gyro.value,
            now_ros=ros,
            now_steady=steady,
        )
        if heading is not None:
            from dataclasses import replace

            heading = replace(
                heading,
                deadline_steady=min(heading.deadline_steady, wheel[1], gyro.deadline_steady),
            )
        runtime.gps_heading = heading
