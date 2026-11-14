"""Real map/pose facade for the existing task navigation owner; no velocity output.

The operator pins a map reference and expected publishers. An untagged /map,
wheel odometry, or a TF alone never establishes map identity or localization.
"""

import math
import time
from pathlib import Path
from typing import Any

from senior_control.hardware_navigation import load_navigation_reference
from senior_control.models import Pose2D
from senior_robot.motion_lifecycle import with_dds_info
from senior_robot.motion_observation_adapter import observation_origin, stamp_seconds


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
        self._pose_ros = float("-inf")
        self._last_ros = float("-inf")
        self._map_serial = 0
        self._pose_map_serial = -1
        self.safety_reason = "motion runtime qualification required"
        self.safety_ages = ""
        from senior_robot.native_receipt import subscribe

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
            self.config.pose_topic,
            self._on_pose,
            writer=self.config.pose_publisher,
            namespace=self.config.source_namespace,
            qos="reliable",
            invalidate=self._invalidate_pose,
        )

    def _invalidate_map(self):
        self._map_gid = None
        self._pose = None
        self._map_serial += 1

    def _invalidate_pose(self):
        self._pose = None
        self._pose_gid = None

    def _source(self, topic, name, info=None):
        endpoints = self._node.get_publishers_info_by_topic(topic)
        if info is None:
            if (
                len(endpoints) != 1
                or endpoints[0].node_name != name
                or endpoints[0].node_namespace != self.config.source_namespace
            ):
                return None
            gid = bytes(endpoints[0].endpoint_gid)
            return gid if gid and any(gid) else None
        origin = observation_origin(
            info,
            endpoints,
            name,
            allow_topology_only=False,
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
        if (
            gid is None
            or msg.header.frame_id != "map"
            or (msg.info.width, msg.info.height) != (g.width, g.height)
            or not math.isclose(msg.info.resolution, g.resolution, abs_tol=1e-8)
            or not math.isclose(origin.position.x, g.origin.x, abs_tol=1e-6)
            or not math.isclose(origin.position.y, g.origin.y, abs_tol=1e-6)
            or not math.isclose(origin.position.z, 0.0, abs_tol=1e-6)
            or not self._yaw_matches(origin.orientation, g.origin.yaw)
            or len(msg.data) != len(g.cells)
            or any(int(a) != b for a, b in zip(msg.data, g.cells, strict=True))
        ):
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
                and self._map_gid == self._source("/map", self.config.map_publisher)
            )
            else None
        )

    @with_dds_info
    def _on_pose(self, msg, info):
        self._pose = None
        gid = self._source(self.config.pose_topic, self.config.pose_publisher, info)
        now_ros = self._node.get_clock().now().nanoseconds / 1e9
        try:
            stamp = stamp_seconds(msg.header.stamp)
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            c = msg.pose.covariance
            if (
                gid is None
                or self.grid() is None
                or msg.header.frame_id != "map"
                or stamp <= self._pose_ros
                or not 0 <= now_ros - stamp <= self.config.max_age_s
                or not all(math.isfinite(v) for v in c)
                or not all(0 < c[i] <= self.config.max_position_variance for i in (0, 7))
                or not 0 < c[35] <= self.config.max_yaw_variance
                or not self._yaw_matches(q, math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z))
            ):
                return
            pose = Pose2D(p.x, p.y, math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z))
        except (ValueError, TypeError, OverflowError):
            return
        self._pose, self._pose_gid = pose, gid
        from senior_robot.native_receipt import received_steady

        self._pose_ros, self._pose_at = stamp, received_steady(info)
        self._pose_map_serial = self._map_serial

    def pose(self):
        now_ros = self._node.get_clock().now().nanoseconds / 1e9
        if now_ros < self._last_ros:
            self._pose = None
            self._pose_ros = float("-inf")
        self._last_ros = now_ros
        if (
            self.grid() is None
            or self._pose_map_serial != self._map_serial
            or not 0 <= now_ros - self._pose_ros <= self.config.max_age_s
            or not 0 <= time.monotonic() - self._pose_at <= self.config.max_age_s
            or self._pose_gid != self._source(self.config.pose_topic, self.config.pose_publisher)
        ):
            return None
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
            "ok": True,
            "healthy": True,
            "sampled_at": self._pose_at,
            "map_id": self._grid.map_ref.map_id,
            "reference_revision": self._grid.map_ref.reference_revision,
            "map_generation": self._grid.grid_id,
            "simulation": False,
            "anchor_id": self.config.anchor_id,
            "anchor_x": a.x,
            "anchor_y": a.y,
            "anchor_yaw": a.yaw,
            "x": p.x,
            "y": p.y,
            "yaw": p.yaw,
        }
