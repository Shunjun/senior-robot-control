"""Real map/pose facade for the existing task navigation owner; no velocity output.

The operator pins a map reference and expected publishers. An untagged /map,
wheel odometry, or a TF alone never establishes map identity or localization.
"""

import json
import math
import time
from pathlib import Path
from typing import Any

from senior_sensor_ingress.native_receipt import with_dds_info

from senior_localization.navigation_reference import load_navigation_reference
from vehicle_core.sample_time import SampleTime
from vehicle_core.sensor_policy import default_policy
from vehicle_core.source_order import SourceOrder
from vehicle_core.values import Pose2D


class HardwareReference:
    def __init__(self, node: Any, path: Path):
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from nav_msgs.msg import OccupancyGrid

        self._node = node
        self.config = load_navigation_reference(path)
        self._grid = self.config.grid()
        self._map_gid = None
        self._pose_gid = None
        self._pose = None
        self._pose_at = float("-inf")
        self._pose_time = None
        self._pose_identity = None
        self._pose_stamp_ns = -1
        self._pose_sequence = 0
        self._pose_order = SourceOrder()
        self._pose_topic = self.config.pose_topic
        self._map_serial = 0
        self._pose_map_serial = -1
        self.safety_reason = "motion runtime qualification required"
        self.safety_ages = ""
        from senior_sensor_ingress.native_receipt import subscribe

        subscribe(
            node,
            OccupancyGrid,
            "/map",
            self._on_map,
            writer=self.config.map_publisher,
            namespace=self.config.source_namespace,
            qos="latched",
            invalidate=self._invalidate_map,
        )
        subscribe(
            node,
            PoseWithCovarianceStamped,
            self._pose_topic,
            self._on_pose,
            writer=self.config.pose_publisher,
            namespace=self.config.source_namespace,
            # AMCL retains its last pose for late joiners; original stamp/TTL still apply.
            qos="latched",
            invalidate=self._invalidate_pose,
        )

    def _diagnose(
        self, event, reason, *, msg=None, info=None, sample=None, endpoints=None, **fields
    ):
        """Bound each finite event/reason pair to one ROS log per second.

        Receipt fields are observations only; never remap time or renew a sample.
        """
        try:
            now = time.monotonic_ns()
            if sample is None and info is not None:
                original = info.get("sample_time")
                sample = original if isinstance(original, SampleTime) else None
            if event == "pose_callback" and msg is not None:
                stamp = msg.header.stamp
                if type(stamp.sec) is int and type(stamp.nanosec) is int:
                    coordinate = stamp.sec * 1_000_000_000 + stamp.nanosec
                    previous_stamp = getattr(self, "_diagnostic_pose_stamp", None)
                    fields["stamp_interval_ns"] = (
                        coordinate - previous_stamp if previous_stamp is not None else None
                    )
                    self._diagnostic_pose_stamp = coordinate
            rows = getattr(self, "_diagnostic_rows", None)
            if rows is None:
                rows = self._diagnostic_rows = {}
            key = (event, reason)
            previous, suppressed = rows.get(key, (None, 0))
            if previous is not None and now - previous < 1_000_000_000:
                rows[key] = (previous, suppressed + 1)
                return
            rows[key] = (now, 0)
            row = dict(
                event=event,
                reason=reason,
                steady_ns=now,
                map_serial=self._map_serial,
                suppressed=suppressed,
                **fields,
            )
            if endpoints is not None:
                row.update(
                    writer_count=len(endpoints),
                    writers=[
                        dict(
                            name=e.node_name,
                            namespace=e.node_namespace,
                            gid=bytes(e.endpoint_gid).hex(),
                        )
                        for e in endpoints[:4]
                    ],
                )
            if info is not None:
                gid = info.get("publisher_gid")
                row.update(
                    publisher_gid=gid.hex() if isinstance(gid, bytes) else str(gid)[:128],
                    receipt_generation=info.get("receipt_generation"),
                    receipt_sequence=info.get("receipt_sequence"),
                    writer=info.get("native_writer_name"),
                    writer_namespace=info.get("native_writer_namespace"),
                )
            if msg is not None:
                row["frame"] = msg.header.frame_id[:128]
                row["stamp_sec"] = msg.header.stamp.sec
                row["stamp_nanosec"] = msg.header.stamp.nanosec
                if event == "pose_callback":
                    c = msg.pose.covariance
                    row["covariance"] = [
                        c[i]
                        if isinstance(c[i], (int, float)) and math.isfinite(c[i])
                        else str(c[i])[:128]
                        for i in (0, 7, 35)
                        if i < len(c)
                    ]
            if event == "map_callback" and msg is not None:
                row.update(
                    width=msg.info.width, height=msg.info.height, resolution=msg.info.resolution
                )
            if sample is not None:
                row.update(
                    stamp_ns=sample.stamp_ns,
                    sample_steady_ns=sample.sample_steady_ns,
                    clock_generation=sample.clock_generation,
                    received_steady_ns=sample.received_steady_ns,
                    received_ros_ns=sample.received_ros_ns,
                    age_ns=now - sample.sample_steady_ns,
                )
            self._node.get_logger().info("reference_diagnostic " + json.dumps(row, default=str))
        except Exception:
            # Diagnostic construction/JSON/ROS sinks must not enter admission or
            # native payload-fault handling. Do not log a failing logger again.
            return

    def _invalidate_map(self):
        self._map_gid = None
        self._pose = None
        self._map_serial += 1
        self._diagnose("map_callback", "invalidated")

    def _invalidate_pose(self):
        self._pose = None
        self._pose_gid = None
        self._diagnose("pose_callback", "invalidated")

    def _source(self, topic, name, info=None):
        endpoints = self._node.get_publishers_info_by_topic(topic)
        self._diagnose(
            "source_topology",
            "map" if topic == "/map" else "pose",
            topic=topic,
            expected_writer=name,
            endpoints=endpoints,
        )
        if info is None:
            if (
                len(endpoints) != 1
                or endpoints[0].node_name != name
                or endpoints[0].node_namespace != self.config.source_namespace
            ):
                return None
            gid = bytes(endpoints[0].endpoint_gid)
            return gid
        from senior_motion_estimation.motion_observation_adapter import observation_origin

        origin = observation_origin(
            info,
            endpoints,
            name,
            expected_namespace=self.config.source_namespace,
        )
        return origin.topology_gid if origin is not None else None

    @with_dds_info
    def _on_map(self, msg, info):
        self._map_serial += 1
        self._map_gid = None
        self._pose = None
        gid = self._source("/map", self.config.map_publisher, info)
        g = self._grid
        origin = msg.info.origin
        reason = None
        if gid is None:
            reason = "source"
        elif msg.header.frame_id != "map":
            reason = "frame"
        elif (
            (msg.info.width, msg.info.height) != (g.width, g.height)
            or not math.isclose(msg.info.resolution, g.resolution, abs_tol=1e-8)
            or not math.isclose(origin.position.x, g.origin.x, abs_tol=1e-6)
            or not math.isclose(origin.position.y, g.origin.y, abs_tol=1e-6)
            or not math.isclose(origin.position.z, 0.0, abs_tol=1e-6)
            or not self._yaw_matches(origin.orientation, g.origin.yaw)
        ):
            reason = "geometry"
        elif len(msg.data) != len(g.cells) or any(
            int(a) != b for a, b in zip(msg.data, g.cells, strict=True)
        ):
            reason = "cells"
        self._diagnose(
            "map_callback",
            reason or "accepted",
            msg=msg,
            info=info,
            cells_match=True if reason is None else False if reason == "cells" else None,
        )
        if reason is not None:
            return
        self._map_gid = gid

    @staticmethod
    def _yaw_matches(q, yaw):
        return (
            all(math.isfinite(x) for x in (q.x, q.y, q.z, q.w))
            and abs(q.x) < 1e-6
            and abs(q.y) < 1e-6
            and math.isclose(q.z * q.z + q.w * q.w, 1, abs_tol=1e-6)
            and math.isclose(math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z), yaw, abs_tol=1e-6)
        )

    def grid(self):
        return (
            self._grid
            if (
                self._map_gid is not None
                and self._source("/map", self.config.map_publisher) is not None
            )
            else None
        )

    def _accept_pose_time(self, gid: bytes, sample: SampleTime, info: dict) -> bool:
        if not hasattr(self, "_pose_order"):
            self._pose_order = SourceOrder()  # //Compatible: isolated facade fixtures.
        boot = str(info.get("receipt_boot_id", "legacy-receiver"))
        # Only the managed NativeReceipt channel supplies/changes this identity.
        self._pose_order.confirm_boot(boot)
        accepted = self._pose_order.accept(boot, info["receipt_generation"], sample.stamp_ns)
        if accepted:
            self._pose_stamp_ns = sample.stamp_ns
            self._pose_sequence = info.get("receipt_sequence", 0)
            self._pose = None
        return accepted

    @with_dds_info
    def _on_pose(self, msg, info):
        from senior_sensor_ingress.native_receipt import received_sample

        gid = self._source(self._pose_topic, self.config.pose_publisher, info)
        if gid is None:
            self._pose = None
            self._diagnose("pose_callback", "source", msg=msg, info=info)
            return
        sample = None
        try:
            sample = received_sample(info)
            if not self._accept_pose_time(gid, sample, info):
                self._diagnose("pose_callback", "ordering", msg=msg, info=info, sample=sample)
                return
            stamp = msg.header.stamp
            if (
                type(stamp.sec) is not int
                or type(stamp.nanosec) is not int
                or not 0 <= stamp.nanosec < 1_000_000_000
                or stamp.sec * 1_000_000_000 + stamp.nanosec != sample.stamp_ns
            ):
                self._diagnose("pose_callback", "stamp", msg=msg, info=info, sample=sample)
                return
            age = default_policy().sample_age(sample, now_steady_ns=time.monotonic_ns())
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            c = msg.pose.covariance
            reason = None
            if self.grid() is None:
                reason = "map"
            elif msg.header.frame_id != "map":
                reason = "frame"
            elif age.band == "invalid" or age.seconds >= self.config.max_age_s:
                reason = "age"
            elif (
                len(c) != 36
                or not all(math.isfinite(v) for v in (*c, p.x, p.y, p.z))
                or not all(0 < c[i] <= self.config.max_position_variance for i in (0, 7))
                or not 0 < c[35] <= self.config.max_yaw_variance
            ):
                reason = "covariance"
            elif not self._yaw_matches(q, math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z)):
                reason = "quaternion"
            if reason is not None:
                self._diagnose("pose_callback", reason, msg=msg, info=info, sample=sample)
                return
            pose = Pose2D(p.x, p.y, math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z))
        except (ValueError, TypeError, KeyError, OverflowError) as exc:
            self._pose = None
            self._diagnose(
                "pose_callback",
                "receipt",
                msg=msg,
                info=info,
                sample=sample,
                error_type=type(exc).__name__,
            )
            return
        self._pose_time = sample
        self._pose, self._pose_gid = pose, gid
        # This is an existing seconds-based IPC boundary, not a new receipt.
        self._pose_at = sample.sample_steady_ns / 1e9
        self._pose_map_serial = self._map_serial
        self._diagnose("pose_callback", "accepted", msg=msg, info=info, sample=sample)

    def pose(self):
        reason = None
        if self.grid() is None:
            reason = "map"
        elif self._pose_map_serial != self._map_serial:
            reason = "map_serial"
        elif self._pose_time is None:
            reason = "no_sample"
        elif (
            not 0
            <= time.monotonic_ns() - self._pose_time.sample_steady_ns
            < round(min(self.config.max_age_s, default_policy().hard_age_s) * 1e9)
        ):
            reason = "age"
        elif self._source(self._pose_topic, self.config.pose_publisher) is None:
            reason = "source"
        if reason is not None:
            self._diagnose(
                "pose_query", reason, sample=self._pose_time, pose_map_serial=self._pose_map_serial
            )
            return None
        self._diagnose(
            "pose_query",
            "available" if self._pose is not None else "no_pose",
            sample=self._pose_time,
            pose_map_serial=self._pose_map_serial,
        )
        return self._pose

    def qualified(self, ref, generation):
        return (
            ref == self._grid.map_ref
            and generation == self._grid.grid_id
            and self.pose() is not None
        )

    def state(self, mode, execution_status="idle", *, navigation_ready=False):
        pose, grid = self.pose(), self.grid()
        return {
            "v": 1,
            "ok": True,
            "sampled_at": time.monotonic(),
            "localized": pose is not None,
            "safety_clear": self._node.task_motion_allowed(continuing=False),
            "manual_driving": mode == "manual",
            "execution_status": execution_status,
            "map_id": grid.map_ref.map_id if grid else None,
            "reference_revision": grid.map_ref.reference_revision if grid else None,
            "map_generation": grid.grid_id if grid else None,
            "localization_generation": grid.grid_id if grid and pose else None,
            "navigation_ready": navigation_ready,
            "robot_pose": {"x": pose.x, "y": pose.y, "yaw": pose.yaw} if pose else None,
            "scan_points": [],
        }

    def pose_reply(self):
        p = self.pose()
        if p is None:
            return {"ok": False, "error": "unavailable"}
        a = self.config.anchor_pose
        return {
            "v": 1,
            "ok": True,
            "healthy": True,
            "sampled_at": self._pose_at,
            "map_id": self._grid.map_ref.map_id,
            "reference_revision": self._grid.map_ref.reference_revision,
            "map_generation": self._grid.grid_id,
            "anchor_id": self.config.anchor_id,
            "anchor_x": a.x,
            "anchor_y": a.y,
            "anchor_yaw": a.yaw,
            "x": p.x,
            "y": p.y,
            "yaw": p.yaw,
        }
