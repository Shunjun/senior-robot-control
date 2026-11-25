"""Owned native raster/odometry receipts to the injected localization reference.

The caller has already verified child boot and static lidar provenance. Neither
web requests nor arbitrary /map or TF publishers can install a reference here.
"""

import base64
import hashlib
import json
import time
from dataclasses import replace
from typing import Any

from senior_mapping.evidence import anchor_digest
from senior_mapping.shared_scan import QualifiedScan, ReferenceBasis, project_hits
from senior_mapping.workspace import WorkspaceGrid
from vehicle_core.errors import DomainError, ErrorCode
from vehicle_core.values import MapRef, Pose2D


class NativeNavigationBridge:
    def __init__(self, node: Any, reference: Any, call: Any) -> None:
        self.node, self.reference, self.call = node, reference, call
        self.boot = ""
        self.host_boot_id = ""
        self.odom_frame = "odom"
        self._cut: dict[str, Any] | None = None
        self._grid: tuple[int, WorkspaceGrid, Any] | None = None
        self._publisher: Any = None
        self._tf: Any = None
        self._current_publisher: Any = None
        self.slot: Any = None
        self._scan: QualifiedScan | None = None
        self._last_key: Any = None
        self._reference_floor: Any = None
        self._reference_floor_steady_ns = 0
        self._capture: Any = None
        self._retired = False

    def begin(self, boot: str, odom_frame: str = "odom") -> None:
        self._scan = self._last_key = self._reference_floor = self._capture = None
        self._retired = False
        self._reference_floor_steady_ns = time.monotonic_ns()
        self.odom_frame = odom_frame
        self.boot, self._cut, self._grid = boot, None, None
        if self.reference is not None:
            self.reference.begin(boot)

    def withdraw(self) -> None:
        self.invalidate_current("reference_unavailable")
        if self.slot is not None:
            self.slot.retire()
        if self.reference is not None:
            self.reference.withdraw()

    def observe_cut(self, cut: dict[str, Any]) -> None:
        self._cut = cut

    async def raster(self, sequence: int) -> dict[str, Any]:
        result = await self.call("grid", boot_id=self.boot, graph_commit_sequence=sequence)
        if not result.ok:
            return {"pending": True}
        if result.boot_id != self.boot or result.graph_commit_sequence != sequence:
            raise DomainError(ErrorCode.CONFLICT, "native raster watermark changed")
        message = result.grid
        info, origin = message.info, message.info.origin
        if message.header.frame_id != "map" or info.width * info.height > 1048576:
            raise DomainError(ErrorCode.UNAVAILABLE, "native raster exceeds its budget")
        if (
            origin.orientation.x,
            origin.orientation.y,
            origin.orientation.z,
            origin.orientation.w,
        ) != (0.0, 0.0, 0.0, 1.0):
            raise DomainError(ErrorCode.CONFLICT, "native raster axes changed")
        cells = bytes(value % 256 for value in message.data)
        if 0 not in cells:
            return {"pending": True}
        geometry = [info.width, info.height, info.resolution, origin.position.x, origin.position.y]
        identity = hashlib.sha256(json.dumps(geometry).encode() + cells).hexdigest()
        grid = WorkspaceGrid(
            MapRef("unbound-native-cut", 0),
            identity,
            info.width,
            info.height,
            float(info.resolution),
            Pose2D(origin.position.x, origin.position.y, 0.0),
            cells,
            waypoint_writes=True,
        )
        self._grid = (sequence, grid, message)
        return {
            "boot_id": self.boot,
            "sequence": sequence,
            "grid_id": grid.grid_id,
            "width": grid.width,
            "height": grid.height,
            "resolution": grid.resolution,
            "origin": [grid.origin.x, grid.origin.y, grid.origin.yaw],
            "cells_b64": base64.b64encode(cells).decode(),
        }

    def _prepare_publishers(self) -> None:
        if self._publisher is not None:
            return
        from nav_msgs.msg import OccupancyGrid
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from tf2_ros import TransformBroadcaster

        self._publisher = self.node.create_publisher(
            OccupancyGrid,
            "/map",
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self._tf = TransformBroadcaster(self.node)

        def sole_map_owner() -> bool:
            endpoints = self.node.get_publishers_info_by_topic("/map")
            return (
                len(endpoints) == 1
                and endpoints[0].node_name == self.node.get_name()
                and endpoints[0].node_namespace == self.node.get_namespace()
                and endpoints[0].topic_type == "nav_msgs/msg/OccupancyGrid"
            )

        self.reference.set_map_owner(sole_map_owner)

    def install(self, fields: dict[str, Any]) -> None:
        if fields.get("active") is False:
            self.withdraw()
            return
        if self.reference is None or self._cut is None or self._grid is None:
            raise DomainError(ErrorCode.UNAVAILABLE, "qualified native navigation cut unavailable")
        sequence, grid, message = self._grid
        cut = self._cut
        if (
            fields.get("grid_sequence") != sequence
            or fields.get("sequence") != cut["sequence"]
            or cut["boot_id"] != self.boot
        ):
            raise DomainError(ErrorCode.CONFLICT, "navigation cut changed")
        ref = MapRef(fields["map_id"], fields["reference_revision"])
        generation = fields["generation"]
        if not isinstance(generation, str) or not 1 <= len(generation) <= 128:
            raise DomainError(ErrorCode.VALIDATION, "reference generation is required")
        if sequence != cut["sequence"] and generation != self.reference.map_generation:
            raise DomainError(ErrorCode.CONFLICT, "corrected reference needs a new raster")
        # //Compatible: preserve the native id/x/y/yaw graph and capture wire format.
        # Native keyframe IDs are int32; C3 hashes complete string-ID records.
        digest = anchor_digest([
            dict(anchor_id=str(row["id"]), available=True,
                 pose={axis: row[axis] for axis in ("x", "y", "yaw")})
            for row in cut["anchors"]
        ])
        anchor = next(row for row in cut["anchors"] if row["id"] == cut["keyframe_id"])
        self._prepare_publishers()
        self._publisher.publish(message)
        previous = (
            self.reference.grid().map_ref if self.reference.grid() else None,
            self.reference.map_generation,
        )
        self.reference.install(
            self.boot,
            replace(grid, map_ref=ref),
            generation,
            str(anchor["id"]),
            Pose2D(anchor["x"], anchor["y"], anchor["yaw"]),
            Pose2D(*cut["odom"]),
        )

        basis = ReferenceBasis(
            ref.map_id,
            ref.reference_revision,
            generation,
            cut["sequence"],
            digest,
            str(anchor["id"]),
            Pose2D(anchor["x"], anchor["y"], anchor["yaw"]),
            Pose2D(*cut["odom"]),
        )
        changed = previous != (ref, generation)
        if changed:
            self._reference_floor = self._last_key
            self._reference_floor_steady_ns = time.monotonic_ns()
        if self.slot is not None:
            self.slot.install_reference(basis, floor_steady_ns=self._reference_floor_steady_ns)
        if changed:
            self.invalidate_current("reference_changed")

    def observe_scan(self, scan: QualifiedScan, capture: Any) -> None:
        if (
            self._retired
            or scan.key.backend_boot != self.boot
            or scan.capture.time.sample_steady_ns <= self._reference_floor_steady_ns
        ):
            return
        if self._last_key is not None and (
            scan.key.source != self._last_key.source or scan.key.sequence <= self._last_key.sequence
        ):
            return
        self._last_key, self._capture = scan.key, capture
        if time.monotonic_ns() >= scan.valid_until_steady_ns:
            self.invalidate_current("expired")
            return
        if self.reference is None:
            self.invalidate_current("reference_unavailable")
            if self.slot is not None:
                self.slot.write(scan)
            return
        self.reference.observe(self.boot, capture, scan.odom_base)
        alignment = self.reference.map_to_odom()
        if alignment is None:
            self.invalidate_current("reference_unavailable")
            if self.slot is not None:
                self.slot.write(scan)
            return
        self._scan = scan
        points = project_hits(scan, alignment)
        grid = self.reference.grid()
        covered = grid is not None and all(
            grid.origin.x <= x < grid.origin.x + grid.width * grid.resolution
            and grid.origin.y <= y < grid.origin.y + grid.height * grid.resolution
            for x, y in points
        )
        reason = "" if covered else "coverage_pending"
        if self.slot is not None and self.slot.error is not None:
            covered, reason = False, "scan_slot_unavailable"
        self.reference.set_current_coverage(covered, reason or None)
        self._publish_transform(capture.stamp)
        self._publish_current(True, reason, points, scan.valid_until_steady_ns)
        if self.slot is not None:
            self.slot.write(scan)

    def invalidate_current(self, reason: str, *, retired: bool = False) -> None:
        self._scan = None
        self._retired = self._retired or retired
        if self.reference is not None:
            self.reference.invalidate_pose()
            self.reference.set_current_coverage(False, reason)
        if self.slot is not None:
            if retired:
                self.slot.retire()
            else:
                self.slot.invalidate(reason)
        self._publish_current(False, reason, (), 0)

    def check_current(self) -> None:
        if self._scan is not None and time.monotonic_ns() >= self._scan.valid_until_steady_ns:
            self.invalidate_current("expired")

    def _publish_current(self, valid: bool, reason: str, points: tuple, deadline: int) -> None:
        if not self.host_boot_id or not self.boot:
            return
        from geometry_msgs.msg import Point32
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from senior_interfaces.msg import CurrentObstacles

        if self._current_publisher is None:
            self._current_publisher = self.node.create_publisher(
                CurrentObstacles,
                "/senior/mapping/current_obstacles",
                QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE),
            )
        message = CurrentObstacles()
        message.backend_boot = self.boot
        message.host_boot_id = self.host_boot_id
        grid = None if self.reference is None else self.reference.grid()
        if grid is not None:
            message.map_id = grid.map_ref.map_id
            message.reference_revision = grid.map_ref.reference_revision
            message.map_generation = self.reference.map_generation or ""
        if self._capture is not None:
            message.capture = self._capture
        message.valid, message.reason = valid, reason
        message.valid_until_steady_ns = deadline
        message.frame_id = "map"
        message.points = [Point32(x=float(x), y=float(y), z=0.0) for x, y in points]
        self._current_publisher.publish(message)

    def _publish_transform(self, stamp: Any) -> None:
        if self._tf is None:
            return
        alignment = self.reference.map_to_odom()
        if alignment is None:
            return
        import math

        from geometry_msgs.msg import TransformStamped

        message = TransformStamped()
        message.header.stamp = stamp
        message.header.frame_id, message.child_frame_id = "map", self.odom_frame
        message.transform.translation.x, message.transform.translation.y = alignment.x, alignment.y
        message.transform.rotation.z = math.sin(alignment.yaw / 2)
        message.transform.rotation.w = math.cos(alignment.yaw / 2)
        self._tf.sendTransform(message)

    def close(self) -> None:
        self.withdraw()
        if self._current_publisher is not None:
            self.node.destroy_publisher(self._current_publisher)
            self._current_publisher = None
        if self._publisher is not None:
            self.node.destroy_publisher(self._publisher)
            self._publisher = None
