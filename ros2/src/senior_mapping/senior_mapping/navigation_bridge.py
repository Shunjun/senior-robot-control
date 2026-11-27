"""Owner-installed fixed cuts and independent original scan/odometry receipts.

The caller has already verified child boot and static lidar provenance. Neither
web requests nor arbitrary /map or TF publishers can install a reference here.
"""

import base64
import hashlib
import json
import time
import uuid
from dataclasses import asdict, replace
from pathlib import Path
from typing import Any

from senior_mapping.evidence import SourceIdentity, anchor_digest
from senior_mapping.references import _watermark_payload, validate_fixed_snapshot
from senior_mapping.shared_scan import (
    ContractSlot,
    CostmapGeometry,
    CurrentLayerCoverage,
    QualifiedScan,
    ReferenceAuthorization,
    ReferenceBasis,
    project_hits,
    read_contract_record,
)
from senior_mapping.workspace import WorkspaceGrid
from vehicle_core.errors import DomainError, ErrorCode
from vehicle_core.source_order import SourceOrder
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
        self._scan_order = SourceOrder()
        self._reference_floor: Any = None
        self._reference_floor_steady_ns = 0
        self._capture: Any = None
        self._retired = False
        self._authorized = None
        self._fixed_watermark = None
        self._owner_coverage_reason = None
        self.contract = None
        self.authorization_slot = None
        self._owner_instance = uuid.uuid4().hex
        self._authority_epoch = self._domain_epoch = self._authorization_revision = 0
        self._selection = None
        self._navigation_floor_ns = 0
        self._geometry = None
        self._authority_identity = None

    def configure_contract(self, contract):
        self.contract = contract
        self.authorization_slot = ContractSlot(
            Path(contract["contract_directory"]) / "reference.json", ReferenceAuthorization
        )

    def export_authorization(self):
        self._authorization_revision += 1
        w, scan = self._fixed_watermark, self._scan
        installed = w is not None and self._authorized == (
            w.map_ref.map_id,
            w.map_ref.reference_revision,
            w.reference_generation,
        )
        revoked = self._retired or (self._authorized is not None and not installed)
        selected = self._selection is not None and installed and not revoked
        lease = 0
        if (
            selected
            and scan is not None
            and scan.capture.time.sample_steady_ns > self._navigation_floor_ns
            and w is not None
            and w.source is not None
            and w.source.coordinates == scan.key.source
        ):
            lease = scan.valid_until_steady_ns
        contract = self.contract or {}
        # //Compatible: the frozen consumer rejects identity changes within one epoch.
        # Derive epochs at the publication boundary, including intermediate revoked
        # offers between owner authorization and verified fixed installation.
        identity = (
            self.boot,
            None if w is None else (w.map_ref, w.reference_generation, w.source),
            self._reference_floor,
            self._reference_floor_steady_ns,
            self._navigation_floor_ns,
            self._selection,
            contract.get("global_instance_permit", ""),
            contract.get("local_instance_permit", ""),
        )
        if identity != self._authority_identity:
            self._authority_epoch += 1
            self._authority_identity = identity
        return ReferenceAuthorization(
            run_nonce=contract.get("run_nonce", "unconfigured"),
            host_boot_id=self.host_boot_id or "unknown",
            owner_instance=self._owner_instance,
            revision=self._authorization_revision,
            authority_epoch=self._authority_epoch,
            status="revoked" if revoked else "selected" if selected else "bootstrap",
            reason="reference_unavailable"
            if self._retired
            else "reference_changed"
            if revoked
            else "",
            map_id="" if w is None else w.map_ref.map_id,
            reference_revision=0 if w is None else w.map_ref.reference_revision,
            reference_generation="" if w is None else w.reference_generation,
            backend_boot=self.boot,
            source=None if w is None or w.source is None else asdict(w.source),
            selection_token="" if self._selection is None else self._selection[0],
            selection_generation=0 if self._selection is None else self._selection[1],
            acceptance_floor=self._reference_floor,
            acceptance_floor_steady_ns=self._reference_floor_steady_ns,
            navigation_floor_steady_ns=self._navigation_floor_ns,
            fixed_revision=0 if w is None else w.fixed_revision,
            graph_sequence=0 if w is None else w.graph_sequence,
            anchor_digest="" if w is None else w.anchor_digest,
            expected_grid_id=""
            if self.reference is None or self.reference.grid() is None
            else self.reference.grid().grid_id,
            geometry=self._geometry,
            coverage_domain_epoch=self._domain_epoch,
            global_instance_permit=contract.get("global_instance_permit", ""),
            local_instance_permit=contract.get("local_instance_permit", ""),
            lease_until_steady_ns=lease,
        )

    def _offer_authorization(self):
        row = self.export_authorization()
        if self.reference is not None:
            self.reference.set_navigation_authorization(row, accepted_key=self._last_key)
        if self.authorization_slot is not None:
            self.authorization_slot.offer(row)
            if self.authorization_slot.error is not None and self.reference is not None:
                self.reference.set_current_coverage(False, "authorization_slot_unavailable")

    def authorize_navigation(self, fields):
        if (
            self._retired
            or self._authorized
            != (fields.get("map_id"), fields.get("reference_revision"), fields.get("generation"))
            or self._fixed_watermark is None
            or not fields.get("selection_token")
            or type(fields.get("selection_generation")) is not int
        ):
            raise DomainError(ErrorCode.CONFLICT, "navigation selection lacks installed reference")
        selection = (fields["selection_token"], fields["selection_generation"])
        if self._selection != selection:
            if self._selection is not None:
                raise DomainError(ErrorCode.CONFLICT, "navigation selection changed session")
            self._selection = selection
            self._navigation_floor_ns = time.monotonic_ns()
        self._offer_authorization()

    def poll_contract(self):
        if self.contract is None or self.reference is None:
            return
        for role in ("global", "local"):
            try:
                row = read_contract_record(
                    Path(self.contract["contract_directory"]) / (role + "-coverage.json"),
                    CurrentLayerCoverage,
                )
                if row is None:
                    self.reference.invalidate_costmap_coverage(role)
                else:
                    self.reference.observe_costmap_coverage(role, row)
            except (ValueError, OSError, TypeError):
                self.reference.invalidate_costmap_coverage(role)
        if self.authorization_slot.error is not None:
            self.reference.set_current_coverage(False, "authorization_slot_unavailable")

    def rotate_local_permit(self, permit):
        if self.contract is not None:
            self.contract["local_instance_permit"] = permit
            self._offer_authorization()

    def begin(self, boot: str, odom_frame: str = "odom") -> None:
        self._scan_order = SourceOrder()
        self._scan = self._last_key = self._reference_floor = self._capture = None
        self._retired = False
        self._reference_floor_steady_ns = time.monotonic_ns()
        self._selection = None
        self._geometry = None
        self.odom_frame = odom_frame
        self.boot, self._cut, self._grid = boot, None, None
        self._authorized = self._fixed_watermark = None
        self._owner_coverage_reason = None
        if self.reference is not None:
            self.reference.begin(boot)
        self._offer_authorization()

    def withdraw(self) -> None:
        self._retired = True
        self._authorized = None
        self.invalidate_current("reference_unavailable")
        if self.slot is not None:
            self.slot.retire()
        if self.reference is not None:
            self.reference.withdraw()
        self._offer_authorization()

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

    def authorize_reference(self, fields):
        """Authenticated owner RPC selection, independent of the transferred JSON."""
        if fields.get("navigation_selected") is True:
            self.authorize_navigation(fields)
            return
        cut = self._cut
        if (
            self._retired
            or cut is None
            or fields.get("boot_id") != self.boot
            or fields.get("sequence") != cut["sequence"]
        ):
            raise DomainError(ErrorCode.CONFLICT, "reference selection changed graph")
        token = (fields["map_id"], fields["reference_revision"], fields["generation"])
        if self._authorized != token:
            self._authorized = token
            self._owner_floor_ns = fields["acceptance_floor_steady_ns"]
            self._reference_floor = self._last_key
            self._reference_floor_steady_ns = max(time.monotonic_ns(), self._owner_floor_ns)
            self.invalidate_current("reference_changed")
        if fields.get("coverage_reason") == "coverage_limit":
            self._owner_coverage_reason = "coverage_limit"
            if self.reference is not None:
                self.reference.set_current_coverage(False, "coverage_limit")

    def install_fixed(self, fixed, *, graph_cut):
        validate_fixed_snapshot(fixed)
        w = fixed.watermark
        graph = self._cut
        digest = anchor_digest(
            [
                dict(
                    anchor_id=str(a["id"]),
                    available=True,
                    pose={axis: a[axis] for axis in ("x", "y", "yaw")},
                )
                for a in graph_cut["anchors"]
            ]
        )
        if (
            self._retired
            or graph is not graph_cut
            or w.backend_boot != self.boot
            or w.host_boot_id != self.host_boot_id
            or self._authorized
            != (w.map_ref.map_id, w.map_ref.reference_revision, w.reference_generation)
            or w.graph_sequence != graph["sequence"]
            or w.anchor_digest != digest
        ):
            raise DomainError(ErrorCode.CONFLICT, "fixed install lacks current owner reference")
        capture = graph.get("capture")
        if capture is not None:
            graph_source = SourceIdentity(
                self.boot,
                capture["bridge_boot_id"],
                capture["source_epoch"],
                capture["capture_generation"],
                capture["time"]["clock_generation"],
                capture["raw_gid"],
                capture["bridge_gid"],
            )
            if w.source != graph_source:
                raise DomainError(ErrorCode.CONFLICT, "fixed source differs from verified graph")
        previous = self._fixed_watermark
        if previous is not None and (
            w.fixed_revision < previous.fixed_revision
            or w.evidence_revision < previous.evidence_revision
            or w.change_sequence < previous.change_sequence
            or (
                w.fixed_revision == previous.fixed_revision
                and replace(w, change_sequence=previous.change_sequence) != previous
            )
        ):
            raise DomainError(ErrorCode.CONFLICT, "late fixed install")
        if previous is not None and w.fixed_revision == previous.fixed_revision:
            installed = None if self.reference is None else self.reference.grid()
            if installed is None or installed.grid_id != fixed.grid.grid_id:
                raise DomainError(ErrorCode.CONFLICT, "unchanged fixed revision changed raster")
        if (
            previous is not None
            and previous.evidence_key is not None
            and w.evidence_key is not None
        ):
            if (
                w.evidence_key.stamp_ns is None
                or previous.evidence_key.stamp_ns is None
                or w.evidence_key.stamp_ns < previous.evidence_key.stamp_ns
            ):
                raise DomainError(ErrorCode.CONFLICT, "late fixed evidence key")
        if self._last_key is not None and w.evidence_key is not None:
            if (
                w.evidence_key.stamp_ns is None
                or self._last_key.stamp_ns is None
                or w.evidence_key.stamp_ns > self._last_key.stamp_ns
            ):
                raise DomainError(ErrorCode.CONFLICT, "fixed evidence is ahead of qualified input")
        if self._last_key is not None and (
            w.source is None or w.source != SourceIdentity.from_key(self._last_key)
        ):
            raise DomainError(ErrorCode.CONFLICT, "fixed source retired")
        if self.reference is None:
            raise DomainError(ErrorCode.UNAVAILABLE, "reference owner unavailable")
        anchor = next(a for a in graph["anchors"] if a["id"] == graph["keyframe_id"])
        grid = replace(fixed.grid, display_only=False, waypoint_writes=True)
        self._publish_fixed_grid(grid)
        self.reference.install(
            self.boot,
            grid,
            w.reference_generation,
            str(anchor["id"]),
            Pose2D(anchor["x"], anchor["y"], anchor["yaw"]),
            Pose2D(*graph["odom"]),
        )
        basis = ReferenceBasis(
            w.map_ref.map_id,
            w.map_ref.reference_revision,
            w.reference_generation,
            w.graph_sequence,
            w.anchor_digest,
            str(anchor["id"]),
            Pose2D(anchor["x"], anchor["y"], anchor["yaw"]),
            Pose2D(*graph["odom"]),
        )
        if self.slot is not None:
            self.slot.install_reference(basis, floor_steady_ns=self._reference_floor_steady_ns)
        geometry = CostmapGeometry.of(grid)
        if self._geometry != geometry:
            self._domain_epoch += 1
            self._geometry = geometry
        if previous is not None and previous.source != w.source:
            self._reference_floor_steady_ns = time.monotonic_ns()
            self.invalidate_current("source_changed")
        self._fixed_watermark = w
        self._owner_coverage_reason = None
        self._offer_authorization()
        # This receipt proves bridge publication/reference installation only.
        return dict(ok=True, watermark=_watermark_payload(w), grid_id=grid.grid_id)

    def _publish_fixed_grid(self, grid):
        from nav_msgs.msg import OccupancyGrid

        self._prepare_publishers()
        message = OccupancyGrid()
        message.header.frame_id = "map"
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.info.width, message.info.height = grid.width, grid.height
        message.info.resolution = grid.resolution
        message.info.origin.position.x, message.info.origin.position.y = (
            grid.origin.x,
            grid.origin.y,
        )
        message.info.origin.orientation.w = 1.0
        message.data = [v if v != 255 else -1 for v in grid.cells]
        self._publisher.publish(message)

    def observe_scan(self, scan: QualifiedScan, capture: Any) -> None:
        if (
            self._retired
            or scan.key.backend_boot != self.boot
            or scan.capture.time.sample_steady_ns <= self._reference_floor_steady_ns
        ):
            return
        if not self._scan_order.accept(
            scan.key.bridge_boot_id, scan.key.capture_generation, scan.capture.stamp_ns
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
        # Preserve the already qualified full Q; display sampling never edits navigation input.
        import math

        beams = tuple(b for b in scan.beams if b.kind == "hit" and b.endpoint_odom is not None)
        self.reference.observe_display(
            scan.key,
            scan.capture.time.sample_steady_ns / 1e9,
            scan.valid_until_steady_ns / 1e9,
            len(scan.beams),
            tuple(
                (b.index, math.dist(b.origin_odom, b.endpoint_odom), p)
                for b, p in zip(beams, points, strict=True)
            ),
        )
        grid = self.reference.grid()
        covered = grid is not None and all(
            grid.origin.x <= x < grid.origin.x + grid.width * grid.resolution
            and grid.origin.y <= y < grid.origin.y + grid.height * grid.resolution
            for x, y in points
        )
        reason = "" if covered else "coverage_pending"
        if self._owner_coverage_reason is not None:
            covered, reason = False, self._owner_coverage_reason
        if self.slot is not None and self.slot.error is not None:
            covered, reason = False, "scan_slot_unavailable"
        self.reference.set_current_coverage(covered, reason or None)
        self._offer_authorization()
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
        self._offer_authorization()

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
        if self.authorization_slot is not None:
            self.authorization_slot.close()
            self.authorization_slot = None
        if self._current_publisher is not None:
            self.node.destroy_publisher(self._current_publisher)
            self._current_publisher = None
        if self._publisher is not None:
            self.node.destroy_publisher(self._publisher)
            self._publisher = None
