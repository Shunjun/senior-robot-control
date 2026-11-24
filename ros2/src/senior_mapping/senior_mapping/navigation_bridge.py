"""Owned native raster/odometry receipts to the injected localization reference.

The caller has already verified child boot and static lidar provenance. Neither
web requests nor arbitrary /map or TF publishers can install a reference here.
"""

import asyncio
import base64
import hashlib
import json
from dataclasses import replace
from typing import Any

from senior_mapping.workspace import WorkspaceGrid
from vehicle_core.errors import DomainError, ErrorCode
from vehicle_core.values import MapRef, Pose2D


class NativeNavigationBridge:
    def __init__(self, node: Any, reference: Any, call: Any) -> None:
        self.node, self.reference, self.call = node, reference, call
        self.boot = ""
        self.odom_frame = "odom"
        self._cut: dict[str, Any] | None = None
        self._grid: tuple[int, WorkspaceGrid, Any] | None = None
        self._publisher: Any = None
        self._tf: Any = None

    def begin(self, boot: str, odom_frame: str = "odom") -> None:
        self.odom_frame = odom_frame
        self.boot, self._cut, self._grid = boot, None, None
        if self.reference is not None:
            self.reference.begin(boot)

    def withdraw(self) -> None:
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
        anchor = next(row for row in cut["anchors"] if row["id"] == cut["keyframe_id"])
        self._prepare_publishers()
        self._publisher.publish(message)
        self.reference.install(
            self.boot,
            replace(grid, map_ref=ref),
            generation,
            str(anchor["id"]),
            Pose2D(anchor["x"], anchor["y"], anchor["yaw"]),
            Pose2D(*cut["odom"]),
        )

    async def run(self) -> None:
        boot = self.boot
        while self.boot == boot:
            try:
                result = await self.call("pose", boot_id=boot)
                if (
                    not result.ok
                    or result.boot_id != boot
                    or not result.static_lidar_tf_receipt_valid
                ):
                    self.reference.invalidate_pose()
                else:
                    self.reference.observe(
                        boot, result.capture, Pose2D(result.odom_x, result.odom_y, result.odom_yaw)
                    )
                    self._publish_transform(result.capture.stamp)
            except (DomainError, RuntimeError, ValueError):
                self.reference.invalidate_pose()
            await asyncio.sleep(0.05)

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
        if self._publisher is not None:
            self.node.destroy_publisher(self._publisher)
            self._publisher = None
