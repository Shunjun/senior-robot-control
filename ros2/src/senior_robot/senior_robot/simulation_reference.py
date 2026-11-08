"""Reference authority for the explicitly isolated, pinned Gazebo baseline world."""

import math
import secrets
import time
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any

from senior_control.mapping.simulation import baseline_grid, map_to_odom
from senior_control.mapping.workspace import WorkspaceGrid
from senior_control.models import MapRef, Pose2D

_MAX_AGE = 0.5
_MAX_SKEW = 0.15
_SAFETY_READY = {
    "command_expired",
    "zero_requested",
    "evaluation_pending",
    "hard_sweep_clear",
    "warning_overlap",
}


@dataclass(frozen=True)
class _Observation:
    pose: Pose2D
    stamp: float
    received: float
    publisher: bytes


class SimulationReference:
    """One ROS-thread owner for the map, its TF relation, and its generation fence."""

    def __init__(self, node: Any, world: Path) -> None:
        from diagnostic_msgs.msg import DiagnosticArray
        from geometry_msgs.msg import TransformStamped
        from nav_msgs.msg import OccupancyGrid, Odometry
        from rclpy.clock import Clock, ClockType
        from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
        from tf2_ros import TransformBroadcaster

        self._node = node
        self._grid = baseline_grid(world, secrets.token_hex(16))
        self._truth: _Observation | None = None
        self._wheel: _Observation | None = None
        self._pose: Pose2D | None = None
        self._pose_at = float("-inf")
        self._safety_at = float("-inf")
        self._safety_gid: bytes | None = None
        self._localization_live = False
        self._ready = False
        self._last_ros = None
        self._transform_type = TransformStamped
        self._tf = TransformBroadcaster(node)
        self._publisher = node.create_publisher(
            OccupancyGrid, "/map", QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        )
        self._map_message = OccupancyGrid()
        self._map_message.header.frame_id = "map"
        info = self._map_message.info
        info.resolution, info.width, info.height = (
            self._grid.resolution,
            self._grid.width,
            self._grid.height,
        )
        info.origin.position.x, info.origin.position.y = self._grid.origin.x, self._grid.origin.y
        info.origin.orientation.w = 1.0
        self._map_message.data = list(self._grid.cells)
        node.create_subscription(
            Odometry, "/ground_truth/odom", self._on_truth, qos_profile_sensor_data
        )
        node.create_subscription(Odometry, "/wheel/odom", self._on_wheel, qos_profile_sensor_data)
        node.create_subscription(DiagnosticArray, "/safety/status", self._on_safety, 10)
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        node.create_timer(0.05, self._tick, clock=steady)

    @property
    def generation(self) -> str:
        return self._grid.grid_id

    def grid(self) -> WorkspaceGrid | None:
        return self._grid if self._ready and self._unique_map() else None

    def pose(self) -> Pose2D | None:
        if (
            self._pose is None
            or time.monotonic() - self._pose_at > _MAX_AGE
            or self.grid() is None
            or self._truth is None
            or self._wheel is None
            or self._source("/ground_truth/odom", "senior_sim_bridge") != self._truth.publisher
            or self._source("/wheel/odom", "drive") != self._wheel.publisher
        ):
            return None
        return self._pose

    def qualified(self, ref: MapRef, generation: str) -> bool:
        return (
            ref == self._grid.map_ref and generation == self.generation and self.pose() is not None
        )

    def state(
        self, mode: str, execution_status: str = "idle", *, navigation_ready: bool = False
    ) -> dict[str, Any]:
        grid, pose = self.grid(), self.pose()
        return {
            "ok": True,
            "sampled_at": time.monotonic(),
            "localized": pose is not None,
            "safety_clear": mode != "estop"
            and (
                time.monotonic() - self._safety_at <= 1.25
                and self._safety_gid is not None
                and self._source("/safety/status", "safety_gate") == self._safety_gid
            ),
            "manual_driving": mode == "manual",
            "map_id": grid.map_ref.map_id if grid else None,
            "reference_revision": grid.map_ref.reference_revision if grid else None,
            "map_generation": grid.grid_id if grid else None,
            "localization_generation": grid.grid_id if grid and pose else None,
            "execution_status": execution_status,
            "navigation_ready": navigation_ready,
        }

    def pose_reply(self) -> dict[str, Any]:
        pose = self.pose()
        if pose is None:
            return {"ok": False, "error": "unavailable"}
        return {
            "ok": True,
            "healthy": True,
            "sampled_at": self._pose_at,
            "map_id": self._grid.map_ref.map_id,
            "reference_revision": self._grid.map_ref.reference_revision,
            "map_generation": self.generation,
            "simulation": True,
            "anchor_id": "sim-world-origin",
            "x": pose.x,
            "y": pose.y,
            "yaw": pose.yaw,
            "anchor_x": 0.0,
            "anchor_y": 0.0,
            "anchor_yaw": 0.0,
        }

    def _source(self, topic: str, name: str, info: dict[str, Any] | None = None) -> bytes | None:
        sources = self._node.get_publishers_info_by_topic(topic)
        if len(sources) != 1 or (sources[0].node_name, sources[0].node_namespace) != (name, "/"):
            return None
        expected = bytes(sources[0].endpoint_gid)
        # //Compatible: Humble rclpy does not expose publisher_gid in every RMW.
        actual = info.get("publisher_gid") if info is not None else None
        return expected if actual is None or bytes(actual) == expected else None

    def _unique_map(self) -> bool:
        return self._source("/map", self._node.get_name()) is not None

    def _observe(
        self,
        msg: Any,
        info: dict[str, Any],
        topic: str,
        name: str,
        frame: str,
        previous: _Observation | None,
    ) -> _Observation | None:
        source = self._source(topic, name, info)
        if source is None or (msg.header.frame_id, msg.child_frame_id) != (frame, "base_link"):
            return None
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        now = self._node.get_clock().now().nanoseconds / 1e9
        if not 0 <= now - stamp <= _MAX_AGE or stamp <= 0:
            return None
        if previous is not None and (source != previous.publisher or stamp < previous.stamp):
            self._invalidate()
        elif previous is not None and stamp == previous.stamp:
            return previous
        position, q = msg.pose.pose.position, msg.pose.pose.orientation
        if not math.isclose(sum(v * v for v in (q.x, q.y, q.z, q.w)), 1, abs_tol=0.02):
            return None
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        try:
            return _Observation(
                Pose2D(position.x, position.y, yaw), stamp, time.monotonic(), source
            )
        except ValueError:
            return None

    def _on_truth(self, msg: Any, info: dict[str, Any]) -> None:
        self._truth = self._observe(
            msg, info, "/ground_truth/odom", "senior_sim_bridge", "world", self._truth
        )

    def _on_wheel(self, msg: Any, info: dict[str, Any]) -> None:
        self._wheel = self._observe(msg, info, "/wheel/odom", "drive", "odom", self._wheel)

    def _on_safety(self, msg: Any, info: dict[str, Any]) -> None:
        self._safety_gid = self._source("/safety/status", "safety_gate", info)
        self._safety_at = (
            time.monotonic()
            if self._safety_gid is not None
            and any(
                status.name == "safety_gate" and status.message in _SAFETY_READY
                for status in msg.status
            )
            else float("-inf")
        )

    def _invalidate(self) -> None:
        if self._localization_live:
            self._grid = replace(self._grid, grid_id=secrets.token_hex(16))
        self._localization_live = False
        self._pose = None
        self._truth = self._wheel = None

    def _tick(self) -> None:
        now = self._node.get_clock().now()
        ros_now = now.nanoseconds / 1e9
        if self._last_ros is not None and ros_now < self._last_ros:
            self._invalidate()
        self._last_ros = ros_now
        if (
            not self._unique_map()
            or self._source("/ground_truth/odom", "senior_sim_bridge") is None
        ):
            self._ready = False
            self._invalidate()
            return
        if not self._ready:
            self._map_message.header.stamp = now.to_msg()
            self._publisher.publish(self._map_message)
            self._ready = True
        truth, wheel = self._truth, self._wheel
        if (
            truth is None
            or wheel is None
            or time.monotonic() - min(truth.received, wheel.received) > _MAX_AGE
            or abs(truth.stamp - wheel.stamp) > _MAX_SKEW
        ):
            if self._localization_live:
                self._invalidate()
            return
        transform = map_to_odom(truth.pose, wheel.pose)
        message = self._transform_type()
        message.header.frame_id = "map"
        message.header.stamp = now.to_msg()
        message.child_frame_id = "odom"
        message.transform.translation.x = transform.x
        message.transform.translation.y = transform.y
        message.transform.rotation.z = math.sin(transform.yaw / 2)
        message.transform.rotation.w = math.cos(transform.yaw / 2)
        self._tf.sendTransform(message)
        self._pose = truth.pose
        self._pose_at = min(truth.received, wheel.received)
        self._localization_live = True
