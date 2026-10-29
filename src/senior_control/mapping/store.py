"""Reconcile long-term occupancy and waypoints with pose-graph anchors."""

import math
from dataclasses import dataclass, replace
from pathlib import Path

from senior_control.errors import DomainError, ErrorCode
from senior_control.mapping.evidence import CellEvidence, OccupancySnapshot
from senior_control.mapping.references import (
    AnchoredWaypoint,
    AnchorPose,
    AnchorSnapshot,
    CellBinding,
    GridExtent,
    ReferenceChange,
    SlamSnapshot,
    WaypointBinding,
)
from senior_control.models import MapRef, Pose2D


@dataclass
class _Record:
    evidence: CellEvidence
    world_x: float
    world_y: float
    anchor_pose: Pose2D


@dataclass
class _Waypoint:
    waypoint: AnchoredWaypoint
    anchor_pose: Pose2D


def _reproject(old: Pose2D, new: Pose2D, x: float, y: float) -> tuple[float, float]:
    dx = x - old.x
    dy = y - old.y
    cosine = math.cos(old.yaw)
    sine = math.sin(old.yaw)
    local_x = cosine * dx + sine * dy
    local_y = -sine * dx + cosine * dy
    cosine_new = math.cos(new.yaw)
    sine_new = math.sin(new.yaw)
    return (
        new.x + cosine_new * local_x - sine_new * local_y,
        new.y + sine_new * local_x + cosine_new * local_y,
    )


def _wrap(yaw: float) -> float:
    return math.atan2(math.sin(yaw), math.cos(yaw))


class MapReferenceStore:
    """One map, one odometry owner, and occupancy cells tied to their anchor.

    Grid width, height, and origin are display extent. They do not change
    ``MapRef``. A loop moves each cell and waypoint by its own anchor pose.
    An anchor that is missing or has no corrected pose leaves those records
    where they are and marks the reference review-required.
    """

    def __init__(self, map_ref: MapRef, odom_owner: str, cell_size_m: float) -> None:
        owner = odom_owner.strip()
        if not owner:
            raise ValueError("odometry owner is required")
        if not math.isfinite(cell_size_m) or cell_size_m <= 0.0:
            raise ValueError("cell size must be positive")
        self.map_ref = map_ref
        self.odom_owner = owner
        self.cell_size_m = float(cell_size_m)
        self.grid = GridExtent(0, 0, 0.0, 0.0)
        self.review_required = False
        self._live: dict[int, _Record] = {}
        self._removed: dict[int, _Record] = {}
        self._waypoints: list[_Waypoint] = []
        self._seq = 0

    def place(
        self,
        evidence: CellEvidence,
        *,
        world: tuple[float, float],
        anchor_pose: Pose2D,
    ) -> None:
        anchor_id = evidence.anchor_id.strip()
        if not anchor_id or self._index(*world) != evidence.cell:
            raise ValueError("cell is not bound to its anchor point")
        record = _Record(replace(evidence, anchor_id=anchor_id), world[0], world[1], anchor_pose)
        found = self._find(self._live, evidence.cell)
        if found is None:
            self._seq += 1
            self._live[self._seq] = record
        else:
            self._live[found[0]] = record
        removed = self._find(self._removed, evidence.cell)
        if removed is not None:
            del self._removed[removed[0]]

    def place_waypoint(self, waypoint: AnchoredWaypoint, *, anchor_pose: Pose2D) -> None:
        anchor_id = waypoint.anchor_id.strip()
        if not waypoint.waypoint_id.strip() or not anchor_id:
            raise ValueError("waypoint anchor is required")
        stored = replace(waypoint, anchor_id=anchor_id)
        for index, item in enumerate(self._waypoints):
            if item.waypoint.waypoint_id == stored.waypoint_id:
                self._waypoints[index] = _Waypoint(stored, anchor_pose)
                return
        self._waypoints.append(_Waypoint(stored, anchor_pose))

    def remove(self, evidence: CellEvidence) -> None:
        found = self._find(self._live, evidence.cell)
        if found is None:
            return
        key, current = found
        tombstone = replace(evidence, anchor_id=current.evidence.anchor_id)
        self._removed[key] = _Record(
            tombstone, current.world_x, current.world_y, current.anchor_pose
        )
        del self._live[key]

    def apply_anchor_update(self, update: AnchorSnapshot) -> ReferenceChange:
        owner = update.odom_owner.strip()
        if update.map_ref.map_id != self.map_ref.map_id or owner != self.odom_owner:
            raise DomainError(ErrorCode.CONFLICT, "one map has one odometry owner")
        poses, review = self._usable_poses(update.anchors)
        self.grid = update.grid
        self._move_records(self._live, poses)
        self._move_records(self._removed, poses)
        if self._move_waypoints(poses) or self._unresolved(poses) or self._cells_collide():
            review = True
        self.review_required = review
        return ReferenceChange(
            self.map_ref,
            self.occupancy().cells,
            tuple(item.waypoint for item in self._waypoints),
            review,
            self.odom_owner,
        )

    def occupancy(self) -> OccupancySnapshot:
        return OccupancySnapshot(self.map_ref, tuple(item.evidence for item in self._live.values()))

    def removals(self) -> tuple[CellEvidence, ...]:
        return tuple(item.evidence for item in self._removed.values())

    def clear_costmap(self) -> None:
        """Local cost clearing does not change the long-term layer or removals."""

    def project_pose_graph(self, cells: tuple[tuple[int, int], ...]) -> OccupancySnapshot:
        """Ignore a pose-graph raster. Deleted scans must not refill occupancy."""

        del cells
        return self.occupancy()

    def is_empty(self) -> bool:
        return not self._live and not self._removed and not self._waypoints

    def snapshot(
        self,
        pose_graph_prefix: Path,
        pose_graph_cells: tuple[tuple[int, int], ...],
    ) -> SlamSnapshot:
        bindings = tuple(self._binding(item, removed=False) for item in self._live.values())
        bindings += tuple(self._binding(item, removed=True) for item in self._removed.values())
        waypoints = tuple(
            WaypointBinding(item.waypoint, item.anchor_pose) for item in self._waypoints
        )
        return SlamSnapshot(
            self.map_ref,
            self.odom_owner,
            self.cell_size_m,
            pose_graph_prefix,
            self.occupancy(),
            self.removals(),
            pose_graph_cells,
            self._anchor_snapshot(),
            self.grid,
            bindings,
            waypoints,
            self.review_required,
        )

    def restore(self, snapshot: SlamSnapshot) -> None:
        if not self.is_empty():
            raise DomainError(ErrorCode.CONFLICT, "occupancy store already has records")
        if snapshot.map_ref != self.map_ref or snapshot.odom_owner != self.odom_owner:
            raise DomainError(ErrorCode.CONFLICT, "one map has one odometry owner")
        if not math.isclose(snapshot.cell_size_m, self.cell_size_m):
            raise DomainError(ErrorCode.CONFLICT, "cell size does not match the store")
        self.grid = snapshot.grid
        self.review_required = snapshot.review_required
        self._install(snapshot)

    def replace(self, snapshot: SlamSnapshot) -> None:
        """Switch identity, resolution, odometry owner, and sequence together."""

        self._reset_identity(snapshot.map_ref, snapshot.odom_owner, snapshot.cell_size_m)
        self.grid = snapshot.grid
        self.review_required = snapshot.review_required
        self._install(snapshot)

    def clear(self, map_ref: MapRef, *, odom_owner: str, cell_size_m: float) -> None:
        """Empty the layer and publish a new identity in the same update."""

        self._reset_identity(map_ref, odom_owner, cell_size_m)
        self.grid = GridExtent(0, 0, 0.0, 0.0)
        self.review_required = False

    def _reset_identity(self, map_ref: MapRef, odom_owner: str, cell_size_m: float) -> None:
        owner = odom_owner.strip()
        if not owner:
            raise ValueError("odometry owner is required")
        if not math.isfinite(cell_size_m) or cell_size_m <= 0.0:
            raise ValueError("cell size must be positive")
        self.map_ref = map_ref
        self.odom_owner = owner
        self.cell_size_m = float(cell_size_m)
        self._seq = 0
        self._live.clear()
        self._removed.clear()
        self._waypoints.clear()

    def _install(self, snapshot: SlamSnapshot) -> None:
        for binding in snapshot.bindings:
            self._seq += 1
            record = _Record(
                binding.evidence,
                binding.world_x,
                binding.world_y,
                binding.anchor_pose,
            )
            target = self._removed if binding.removed else self._live
            target[self._seq] = record
        self._waypoints = [
            _Waypoint(binding.waypoint, binding.anchor_pose)
            for binding in snapshot.waypoint_bindings
        ]

    def _binding(self, record: _Record, *, removed: bool) -> CellBinding:
        return CellBinding(
            record.evidence,
            record.world_x,
            record.world_y,
            record.anchor_pose,
            removed,
        )

    def _anchor_snapshot(self) -> tuple[AnchorPose, ...]:
        found: dict[str, Pose2D] = {}
        for record in list(self._live.values()) + list(self._removed.values()):
            found[record.evidence.anchor_id] = record.anchor_pose
        for item in self._waypoints:
            found[item.waypoint.anchor_id] = item.anchor_pose
        return tuple(AnchorPose(anchor_id, pose, True) for anchor_id, pose in found.items())

    def _usable_poses(self, anchors: tuple[AnchorPose, ...]) -> tuple[dict[str, Pose2D], bool]:
        found: dict[str, Pose2D] = {}
        review = False
        for anchor in anchors:
            anchor_id = anchor.anchor_id.strip()
            pose = anchor.pose if anchor.available else None
            if not anchor_id or pose is None or anchor_id in found:
                review = True
                if anchor_id:
                    found.pop(anchor_id, None)
                continue
            found[anchor_id] = pose
        return found, review

    def _move_records(self, records: dict[int, _Record], poses: dict[str, Pose2D]) -> None:
        for record in records.values():
            pose = poses.get(record.evidence.anchor_id)
            if pose is None:
                continue
            world_x, world_y = _reproject(record.anchor_pose, pose, record.world_x, record.world_y)
            record.world_x = world_x
            record.world_y = world_y
            record.anchor_pose = pose
            record.evidence = replace(record.evidence, cell=self._index(world_x, world_y))

    def _move_waypoints(self, poses: dict[str, Pose2D]) -> bool:
        unresolved = False
        for item in self._waypoints:
            pose = poses.get(item.waypoint.anchor_id)
            if pose is None:
                unresolved = True
                item.waypoint = replace(item.waypoint, review_required=True)
                continue
            world_x, world_y = _reproject(
                item.anchor_pose, pose, item.waypoint.pose.x, item.waypoint.pose.y
            )
            yaw = _wrap(item.waypoint.pose.yaw + (pose.yaw - item.anchor_pose.yaw))
            item.anchor_pose = pose
            item.waypoint = replace(
                item.waypoint,
                pose=Pose2D(world_x, world_y, yaw),
                review_required=False,
            )
        return unresolved

    def _unresolved(self, poses: dict[str, Pose2D]) -> bool:
        identities = {record.evidence.anchor_id for record in self._live.values()}
        identities.update(record.evidence.anchor_id for record in self._removed.values())
        identities.update(item.waypoint.anchor_id for item in self._waypoints)
        return any(anchor_id not in poses for anchor_id in identities)

    def _cells_collide(self) -> bool:
        cells = [record.evidence.cell for record in self._live.values()]
        return len(cells) != len(set(cells))

    def _find(
        self, records: dict[int, _Record], cell: tuple[int, int]
    ) -> tuple[int, _Record] | None:
        for key, record in records.items():
            if record.evidence.cell == cell:
                return key, record
        return None

    def _index(self, x: float, y: float) -> tuple[int, int]:
        return (self._axis(x), self._axis(y))

    def _axis(self, value: float) -> int:
        if not math.isfinite(value):
            raise ValueError("point must be finite")
        scaled = value / self.cell_size_m
        nearest = round(scaled)
        if math.isclose(scaled, nearest, abs_tol=1e-9):
            scaled = float(nearest)
        return math.floor(scaled)
