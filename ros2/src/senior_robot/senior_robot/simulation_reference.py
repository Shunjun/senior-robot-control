"""Reference authority for the explicitly isolated, pinned Gazebo baseline world."""

import json
import math
import secrets
import time
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any

from senior_control.mapping.simulation import baseline_grid, map_to_odom, project_scan
from senior_control.mapping.workspace import WorkspaceGrid
from senior_control.models import MapRef, Pose2D
from senior_control.safety.motion_runtime import DeferredSample, DeferredSourceQueue
from senior_robot.motion_lifecycle import with_dds_info

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
        from rclpy.duration import Duration
        from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
        from sensor_msgs.msg import LaserScan
        from tf2_ros import Buffer, TransformBroadcaster, TransformListener

        self._node = node
        self._grid = baseline_grid(world, secrets.token_hex(16))
        self._scan_points: list[dict[str, float]] = []
        self._scan_received = float("-inf")
        self._scan_stamp = float("-inf")
        self._scan_generation = ""
        self._scan_gid: bytes | None = None
        self._scan_tf = Buffer()
        self._scan_listener = TransformListener(self._scan_tf, node)
        self._truth: _Observation | None = None
        self._wheel: _Observation | None = None
        self._deferred_observations: dict[str, DeferredSourceQueue[_Observation]] = {
            "/ground_truth/odom": DeferredSourceQueue(max_pending=32),
            "/wheel/odom": DeferredSourceQueue(max_pending=32),
        }
        self._future_observations = self._future_admitted = 0
        self._last_qualification_log = float("-inf")
        self._pose: Pose2D | None = None
        self._pose_at = float("-inf")
        self._safety_at = float("-inf")
        self.safety_reason = "no_status"
        self.safety_ages = ""
        self._safety_gid: bytes | None = None
        self._localization_live = False
        self._ready = False
        self._last_ros = None
        self._transform_type = TransformStamped
        self._transform_tolerance = Duration(seconds=0.15)
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
        node.create_subscription(LaserScan, "/scan", self._on_scan, qos_profile_sensor_data)
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

    def safety_ready(self) -> bool:
        return (
            time.monotonic() - self._safety_at <= 1.25
            and self._safety_gid is not None
            and self._source("/safety/status", "safety_gate") == self._safety_gid
        )

    def state(
        self, mode: str, execution_status: str = "idle", *, navigation_ready: bool = False
    ) -> dict[str, Any]:
        grid, pose = self.grid(), self.pose()
        return {
            "ok": True,
            "sampled_at": time.monotonic(),
            "localized": pose is not None,
            "safety_clear": mode != "estop" and self.safety_ready(),
            "manual_driving": mode == "manual",
            "map_id": grid.map_ref.map_id if grid else None,
            "reference_revision": grid.map_ref.reference_revision if grid else None,
            "map_generation": grid.grid_id if grid else None,
            "localization_generation": grid.grid_id if grid and pose else None,
            "execution_status": execution_status,
            "navigation_ready": navigation_ready,
            "robot_pose": {"x": pose.x, "y": pose.y, "yaw": pose.yaw} if pose else None,
            "scan_points": self.scan_points() if pose else [],
        }

    def scan_points(self) -> list[dict[str, float]]:
        now = self._node.get_clock().now().nanoseconds / 1e9
        if (
            self.pose() is None
            or self._truth is None
            or self._scan_generation != self.generation
            or not 0 <= time.monotonic() - self._scan_received <= _MAX_AGE
            or not 0 <= now - self._scan_stamp <= _MAX_AGE
            or abs(self._truth.stamp - self._scan_stamp) > _MAX_SKEW
            or self._scan_gid != self._source("/scan", "senior_sim_bridge")
        ):
            return []
        return self._scan_points

    @with_dds_info
    def _on_scan(self, msg: Any, info: dict[str, Any]) -> None:
        from rclpy.time import Time
        from tf2_ros import TransformException

        self._scan_points = []
        pose = self.pose()
        source = self._source("/scan", "senior_sim_bridge", info)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        now = self._node.get_clock().now().nanoseconds / 1e9
        if (
            pose is None
            or self._truth is None
            or source is None
            or msg.header.frame_id != "laser_link"
            or not 0 <= now - stamp <= _MAX_AGE
            or abs(self._truth.stamp - stamp) > _MAX_SKEW
            or not 0 < len(msg.ranges) <= 4096
            or not all(
                math.isfinite(v)
                for v in (msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max)
            )
            or not 0 < msg.range_min < msg.range_max
        ):
            return
        try:
            transform = self._scan_tf.lookup_transform("base_link", "laser_link", Time())
        except TransformException:
            return
        t, q = transform.transform.translation, transform.transform.rotation
        # Only the planar, fixed laser mount of this qualified simulation is supported.
        if (
            not all(math.isfinite(v) for v in (t.x, t.y, q.x, q.y, q.z, q.w))
            or abs(q.x) > 1e-6
            or abs(q.y) > 1e-6
            or not math.isclose(q.z * q.z + q.w * q.w, 1.0, abs_tol=0.02)
        ):
            return
        sensor = Pose2D(t.x, t.y, 2 * math.atan2(q.z, q.w))
        self._scan_points = project_scan(
            pose,
            sensor,
            msg.ranges,
            msg.angle_min,
            msg.angle_increment,
            msg.range_min,
            msg.range_max,
        )
        self._scan_stamp, self._scan_received = stamp, time.monotonic()
        self._scan_generation, self._scan_gid = self.generation, source

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
        received = time.monotonic()
        now_ns = self._node.get_clock().now().nanoseconds
        source = self._source(topic, name, info)
        if source is None or (msg.header.frame_id, msg.child_frame_id) != (frame, "base_link"):
            self._invalidate("observation_identity_or_frame", details={"topic": topic})
            return None
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        stamp = stamp_ns / 1e9
        if self._last_ros is not None and now_ns / 1e9 < self._last_ros:
            self._invalidate("clock_rollback")
            return None
        if stamp_ns <= 0 or not 0 <= msg.header.stamp.nanosec < 1_000_000_000:
            self._invalidate("observation_time_invalid", details={"topic": topic})
            return None
        queue = self._deferred_observations[topic]
        tail = queue.latest()
        latest = tail.value if tail is not None else previous
        if latest is not None and (source != latest.publisher or stamp < latest.stamp):
            self._invalidate("observation_source_or_stamp_changed", details={"topic": topic})
            previous = latest = None
        elif latest is not None and stamp == latest.stamp:
            return previous
        position, q = msg.pose.pose.position, msg.pose.pose.orientation
        if not math.isclose(sum(v * v for v in (q.x, q.y, q.z, q.w)), 1, abs_tol=0.02):
            self._invalidate("observation_orientation_invalid", details={"topic": topic})
            return None
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        try:
            observation = _Observation(Pose2D(position.x, position.y, yaw), stamp, received, source)
        except ValueError:
            self._invalidate("observation_pose_invalid", details={"topic": topic})
            return None
        if stamp_ns > now_ns or queue.peek() is not None:
            if not queue.offer(DeferredSample(stamp_ns, now_ns, received, observation)):
                self._invalidate("future_queue_rejected", details={"topic": topic})
                return None
            self._future_observations = min(self._future_observations + 1, 2**31 - 1)
            return previous
        if not 0 <= (now_ns - stamp_ns) / 1e9 <= _MAX_AGE:
            self._invalidate("observation_ros_expired", details={"topic": topic})
            return None
        return observation

    def _drain_observations(self, now_ns: int, steady: float) -> bool:
        for topic, name, attribute in (
            ("/ground_truth/odom", "senior_sim_bridge", "_truth"),
            ("/wheel/odom", "drive", "_wheel"),
        ):
            queue = self._deferred_observations[topic]
            for _ in range(32):
                head = queue.peek()
                if head is None:
                    break
                if self._source(topic, name) != head.value.publisher:
                    self._invalidate("future_source_changed", details={"topic": topic})
                    return False
                status = queue.status(
                    now_ros_ns=now_ns, now_steady=steady, max_age=_MAX_AGE, pending_wait=_MAX_AGE
                )
                if status == "waiting":
                    break
                if status != "ready":
                    self._invalidate("future_" + status, details={"topic": topic})
                    return False
                previous = getattr(self, attribute)
                if previous is not None and head.value.stamp <= previous.stamp:
                    self._invalidate("future_stamp_order", details={"topic": topic})
                    return False
                queue.consume()
                setattr(self, attribute, head.value)
                self._future_admitted = min(self._future_admitted + 1, 2**31 - 1)
        return True

    @with_dds_info
    def _on_truth(self, msg: Any, info: dict[str, Any]) -> None:
        self._truth = self._observe(
            msg, info, "/ground_truth/odom", "senior_sim_bridge", "world", self._truth
        )

    @with_dds_info
    def _on_wheel(self, msg: Any, info: dict[str, Any]) -> None:
        self._wheel = self._observe(msg, info, "/wheel/odom", "drive", "odom", self._wheel)

    @with_dds_info
    def _on_safety(self, msg: Any, info: dict[str, Any]) -> None:
        self._safety_gid = self._source("/safety/status", "safety_gate", info)
        status = next((status for status in msg.status if status.name == "safety_gate"), None)
        self.safety_reason = status.message if status is not None else "no_status"
        self.safety_ages = (
            ", ".join(
                f"{item.key}={item.value}"
                for item in status.values
                if item.key
                in {"scan_ros_age", "scan_steady_age", "motion_ros_age", "motion_steady_age"}
            )
            if status is not None
            else ""
        )
        self._safety_at = (
            time.monotonic()
            if self._safety_gid is not None
            and status is not None
            and status.message in _SAFETY_READY
            and any(item.key == "evidence_ready" and item.value == "true" for item in status.values)
            else float("-inf")
        )

    def _invalidate(self, reason: str, *, details: dict[str, object] | None = None) -> None:
        steady = time.monotonic()
        if steady - self._last_qualification_log >= 1.0:
            self._last_qualification_log = steady
            now = self._node.get_clock().now().nanoseconds / 1e9
            truth, wheel = self._truth, self._wheel
            snapshot = {
                "reason": reason,
                "details": details or {},
                "generation": self.generation,
                "localized": self._localization_live,
                "ros_now": now,
                "steady_now": steady,
                "truth_ros_age": now - truth.stamp if truth else None,
                "wheel_ros_age": now - wheel.stamp if wheel else None,
                "truth_steady_age": steady - truth.received if truth else None,
                "wheel_steady_age": steady - wheel.received if wheel else None,
                "skew": abs(truth.stamp - wheel.stamp) if truth and wheel else None,
                "future_observations": self._future_observations,
                "future_admitted": self._future_admitted,
                "pending": {
                    topic: {
                        "stamp_ns": head.stamp_ns,
                        "received_ros_ns": head.received_ros_ns,
                        "steady_age": steady - head.received_steady,
                    }
                    for topic, queue in self._deferred_observations.items()
                    if (head := queue.peek()) is not None
                },
            }
            self._node.get_logger().warning(
                "reference_qualification_lost " + json.dumps(snapshot, sort_keys=True)
            )
        if self._localization_live:
            self._grid = replace(self._grid, grid_id=secrets.token_hex(16))
        self._localization_live = False
        self._pose = None
        self._truth = self._wheel = None
        for queue in self._deferred_observations.values():
            queue.clear()

    def _tick(self) -> None:
        now = self._node.get_clock().now()
        ros_now = now.nanoseconds / 1e9
        if self._last_ros is not None and ros_now < self._last_ros:
            self._invalidate("clock_rollback")
        self._last_ros = ros_now
        truth_source = self._source("/ground_truth/odom", "senior_sim_bridge")
        wheel_source = self._source("/wheel/odom", "drive")
        if not self._unique_map() or truth_source is None or wheel_source is None:
            self._ready = False
            self._invalidate("reference_topology_invalid")
            return
        if (self._truth is not None and self._truth.publisher != truth_source) or (
            self._wheel is not None and self._wheel.publisher != wheel_source
        ):
            self._invalidate("observation_publisher_changed")
            return
        if not self._ready:
            self._map_message.header.stamp = now.to_msg()
            self._publisher.publish(self._map_message)
            self._ready = True
        steady = time.monotonic()
        if not self._drain_observations(now.nanoseconds, steady):
            return
        truth, wheel = self._truth, self._wheel
        if (
            truth is None
            or wheel is None
            or not 0 <= steady - min(truth.received, wheel.received) <= _MAX_AGE
            or not 0 <= ros_now - truth.stamp <= _MAX_AGE
            or not 0 <= ros_now - wheel.stamp <= _MAX_AGE
            or abs(truth.stamp - wheel.stamp) > _MAX_SKEW
        ):
            if self._localization_live:
                self._invalidate("observation_pair_unqualified")
            return
        transform = map_to_odom(truth.pose, wheel.pose)
        message = self._transform_type()
        message.header.frame_id = "map"
        # Nav2 plans at its current simulation time, slightly ahead of our TF timer.
        # The bounded TF tolerance does not extend the measurement's qualification.
        message.header.stamp = (now + self._transform_tolerance).to_msg()
        message.child_frame_id = "odom"
        message.transform.translation.x = transform.x
        message.transform.translation.y = transform.y
        message.transform.rotation.z = math.sin(transform.yaw / 2)
        message.transform.rotation.w = math.cos(transform.yaw / 2)
        self._tf.sendTransform(message)
        self._pose = truth.pose
        self._pose_at = min(truth.received, wheel.received)
        self._localization_live = True
