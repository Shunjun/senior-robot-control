"""Reconcile long-term occupancy and waypoints with pose-graph anchors."""

import math
import uuid
from collections.abc import Iterator
from contextlib import contextmanager
from copy import copy
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

_MAX_INGEST_ANCHORS = 16_384
_MAX_INGEST_CHANGES = 8_192
_MAX_INGEST_RECORDS = 65_536
_MAX_INGEST_SEQUENCE = (1 << 64) - 1


@dataclass(frozen=True)
class OccupancyIngestBinding:
    map_ref: MapRef
    backend_boot: str
    source_epoch: str


@dataclass(frozen=True)
class OccupancyIngestProgress:
    binding: OccupancyIngestBinding
    sequence: int
    keyframe_id: str


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


def _canonical_identity(value: object) -> bool:
    return isinstance(value, str) and bool(value) and value == value.strip()


def _finite_number(value: object) -> bool:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return False
    try:
        return math.isfinite(value)
    except OverflowError:
        return False


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
        self._map_ref = map_ref
        self._odom_owner = owner
        self._cell_size_m = float(cell_size_m)
        self._grid = GridExtent(0, 0, 0.0, 0.0)
        self._review_required = False
        self._guarded = False
        self._map_generation = uuid.uuid4().hex
        self._change_sequence = 0
        self._capture_frozen = False
        self._capture_interrupted = False
        self._ingest_binding: OccupancyIngestBinding | None = None
        self._ingest_progress: OccupancyIngestProgress | None = None
        self._ingest_fenced = False
        self._known_anchors: dict[str, Pose2D] = {}
        self._live: dict[int, _Record] = {}
        self._removed: dict[int, _Record] = {}
        self._waypoints: list[_Waypoint] = []
        self._seq = 0

    @property
    def map_ref(self) -> MapRef:
        return self._map_ref

    @property
    def odom_owner(self) -> str:
        return self._odom_owner

    @property
    def cell_size_m(self) -> float:
        return self._cell_size_m

    @property
    def grid(self) -> GridExtent:
        return self._grid

    @property
    def ingest_progress(self) -> OccupancyIngestProgress | None:
        """In-process occupancy progress, without durable or backend authority."""
        return self._ingest_progress

    def bind_ingest(
        self, map_ref: MapRef, *, backend_boot: str, source_epoch: str
    ) -> OccupancyIngestBinding:
        """Bind a fresh reference layer to an already qualified upstream stream.

        The caller authenticates these identities independently. A binding is
        local to this store and cannot be reconstructed from a native counter.
        """
        self._require_capture_writable()
        if (
            map_ref != self.map_ref
            or self._ingest_binding is not None
            or self._ingest_fenced
            or not self.is_empty()
            or self._known_anchors
            or self.review_required
        ):
            raise DomainError(ErrorCode.CONFLICT, "map cannot bind a new occupancy stream")
        for identity in (backend_boot, source_epoch):
            if not _canonical_identity(identity) or len(identity) > 128:
                raise DomainError(ErrorCode.VALIDATION, "invalid occupancy stream identity")
        binding = OccupancyIngestBinding(map_ref, backend_boot, source_epoch)
        self._ingest_binding = binding
        return binding

    def revoke_ingest(self, binding: OccupancyIngestBinding) -> None:
        """Withdraw this stream when its authenticated source changes or disappears."""
        if binding is not self._ingest_binding:
            raise DomainError(ErrorCode.CONFLICT, "occupancy stream binding is stale")
        self._interrupt_capture()
        self._fence_ingest()

    def commit_ingest(
        self,
        binding: OccupancyIngestBinding,
        sequence: int,
        keyframe_id: str,
        anchors: AnchorSnapshot,
        changes: tuple[CellBinding, ...],
    ) -> OccupancyIngestProgress:
        """Apply one qualified observation synchronously on the owning loop.

        Corrected anchors can move every record. Prepare in a bounded private
        copy, and publish the corrected layer and its watermark without yielding.
        """
        self._require_capture_writable()
        if binding is not self._ingest_binding or binding.map_ref != self.map_ref:
            raise DomainError(ErrorCode.CONFLICT, "occupancy stream binding is stale")
        previous = 0 if self._ingest_progress is None else self._ingest_progress.sequence
        if type(sequence) is int and 0 < sequence <= previous:
            raise DomainError(ErrorCode.CONFLICT, "occupancy observation was already committed")
        try:
            if (
                type(sequence) is not int
                or sequence > _MAX_INGEST_SEQUENCE
                or sequence != previous + 1
            ):
                raise DomainError(ErrorCode.CONFLICT, "occupancy stream sequence is not contiguous")
            staged = self._prepare_ingest(keyframe_id, anchors, changes)
        except Exception:
            self._fence_ingest()
            raise

        self._known_anchors = staged._known_anchors
        self._live = staged._live
        self._removed = staged._removed
        self._waypoints = staged._waypoints
        self._grid, self._seq = staged._grid, staged._seq
        self._change_sequence = staged._change_sequence
        self._map_generation = staged._map_generation
        progress = OccupancyIngestProgress(binding, sequence, keyframe_id)
        self._ingest_progress = progress
        return progress

    def _prepare_ingest(
        self, keyframe_id: str, anchors: AnchorSnapshot, changes: tuple[CellBinding, ...]
    ) -> "MapReferenceStore":
        if (
            len(anchors.anchors) > _MAX_INGEST_ANCHORS
            or len(changes) > _MAX_INGEST_CHANGES
            or len(self._live) + len(self._removed) + len(self._waypoints) + len(changes)
            > _MAX_INGEST_RECORDS
        ):
            raise DomainError(ErrorCode.VALIDATION, "occupancy observation exceeds capture limits")
        grid = anchors.grid
        if (
            not _canonical_identity(keyframe_id)
            or any(not _canonical_identity(anchor.anchor_id) for anchor in anchors.anchors)
            or any(type(size) is not int or size < 0 for size in (grid.width, grid.height))
            or not all(_finite_number(origin) for origin in (grid.origin_x, grid.origin_y))
        ):
            raise DomainError(ErrorCode.VALIDATION, "invalid occupancy anchor identity or grid")
        poses, incomplete = self._usable_poses(anchors.anchors)
        if (
            anchors.map_ref != self.map_ref
            or anchors.odom_owner != self.odom_owner
            or incomplete
            or len(poses) != len(self._known_anchors) + 1
            or not self._known_anchors.keys() <= poses.keys()
            or keyframe_id not in poses
            or keyframe_id in self._known_anchors
        ):
            raise DomainError(ErrorCode.CONFLICT, "corrected anchor set is incomplete or stale")

        staged = copy(self)
        staged._live = {key: replace(record) for key, record in self._live.items()}
        staged._removed = {key: replace(record) for key, record in self._removed.items()}
        staged._waypoints = [replace(waypoint) for waypoint in self._waypoints]
        staged.apply_anchor_update(anchors)
        if staged.review_required:
            raise DomainError(ErrorCode.CONFLICT, "corrected occupancy requires review")
        live_keys = {record.evidence.cell: key for key, record in staged._live.items()}
        removed_keys: dict[tuple[int, int], int] = {}
        for removed_key, record in staged._removed.items():
            removed_keys.setdefault(record.evidence.cell, removed_key)
        seen: set[tuple[int, int]] = set()
        for item in changes:
            evidence = item.evidence
            world = (item.world_x, item.world_y)
            if (
                not _canonical_identity(evidence.anchor_id)
                or not isinstance(evidence.cell, tuple)
                or len(evidence.cell) != 2
                or any(type(coordinate) is not int for coordinate in evidence.cell)
                or evidence.cell in seen
                or type(item.removed) is not bool
                or any(
                    type(count) is not int or count < 0
                    for count in (evidence.hit_count, evidence.free_count)
                )
                or evidence.change_source not in ("geometric_confirmation", "observed_free_space")
                or not all(
                    _finite_number(value)
                    for value in (*world, evidence.last_valid_time, evidence.belief)
                )
                or item.anchor_pose != poses.get(evidence.anchor_id)
                or staged._index(*world) != evidence.cell
            ):
                raise DomainError(ErrorCode.VALIDATION, "invalid anchored occupancy change")
            seen.add(evidence.cell)
            key = live_keys.get(evidence.cell)
            if item.removed:
                if key is None or staged._live[key].evidence.anchor_id != evidence.anchor_id:
                    raise DomainError(ErrorCode.CONFLICT, "occupancy removal anchor does not match")
                staged._remove_cell(key, evidence)
            else:
                record = _Record(evidence, *world, item.anchor_pose)
                staged._put_cell(record, key, removed_keys.get(evidence.cell))
        return staged

    def _fence_ingest(self) -> None:
        self._ingest_binding = None
        self._ingest_progress = None
        self._ingest_fenced = True

    @contextmanager
    def freeze_for_capture(self) -> Iterator[None]:
        """Hold the reference layer stable on its owning event loop.

        A graph owner must acquire its own freeze and reconcile its graph with
        this reference layer before using a snapshot as archive evidence.
        """
        if self._capture_frozen:
            raise DomainError(ErrorCode.CONFLICT, "map reference is frozen for capture")
        self._capture_frozen = True
        self._capture_interrupted = False
        try:
            yield
            if self._capture_interrupted:
                raise DomainError(
                    ErrorCode.OPERATION_UNKNOWN, "map reference capture was interrupted"
                )
        finally:
            self._capture_frozen = False
            self._capture_interrupted = False

    def _require_capture_writable(self) -> None:
        if self._capture_frozen:
            self._capture_interrupted = True
            raise DomainError(ErrorCode.CONFLICT, "map reference is frozen for capture")

    def _interrupt_capture(self) -> None:
        if self._capture_frozen:
            self._capture_interrupted = True

    @property
    def review_required(self) -> bool:
        return self._guarded or self._review_required

    @review_required.setter
    def review_required(self, value: bool) -> None:
        if value != self._review_required:
            self._interrupt_capture()
            self._fence_ingest()
        self._review_required = value

    @property
    def change_sequence(self) -> int:
        return self._change_sequence

    @property
    def map_generation(self) -> str:
        """Reference/localization nonce, not the backend graph capture generation.

        Restart, map switch and changes to existing reference anchors invalidate it.
        """
        return self._map_generation

    def _advance_generation(self) -> None:
        self._map_generation = uuid.uuid4().hex

    def guard_reference(self) -> None:
        self._interrupt_capture()
        self._fence_ingest()
        self._advance_generation()
        self._guarded = True

    def release_reference(self) -> None:
        if self._guarded:
            self._interrupt_capture()
        self._guarded = False

    def place(
        self,
        evidence: CellEvidence,
        *,
        world: tuple[float, float],
        anchor_pose: Pose2D,
    ) -> None:
        self._require_capture_writable()
        self._fence_ingest()
        anchor_id = evidence.anchor_id.strip()
        if not anchor_id or self._index(*world) != evidence.cell:
            raise ValueError("cell is not bound to its anchor point")
        record = _Record(replace(evidence, anchor_id=anchor_id), world[0], world[1], anchor_pose)
        self._notice_anchor(anchor_id, anchor_pose)
        live = self._find(self._live, evidence.cell)
        removed = self._find(self._removed, evidence.cell)
        self._put_cell(
            record, None if live is None else live[0], None if removed is None else removed[0]
        )

    def _put_cell(self, record: _Record, live_key: int | None, removed_key: int | None) -> None:
        self._change_sequence += 1
        if live_key is None:
            self._seq += 1
            live_key = self._seq
        self._live[live_key] = record
        if removed_key is not None:
            del self._removed[removed_key]

    def place_waypoint(self, waypoint: AnchoredWaypoint, *, anchor_pose: Pose2D) -> None:
        self._require_capture_writable()
        self._fence_ingest()
        anchor_id = waypoint.anchor_id.strip()
        if not waypoint.waypoint_id.strip() or not anchor_id:
            raise ValueError("waypoint anchor is required")
        stored = replace(waypoint, anchor_id=anchor_id)
        self._notice_anchor(anchor_id, anchor_pose)
        self._change_sequence += 1
        for index, item in enumerate(self._waypoints):
            if item.waypoint.waypoint_id == stored.waypoint_id:
                self._waypoints[index] = _Waypoint(stored, anchor_pose)
                return
        self._waypoints.append(_Waypoint(stored, anchor_pose))

    def remove(self, evidence: CellEvidence) -> None:
        self._require_capture_writable()
        self._fence_ingest()
        found = self._find(self._live, evidence.cell)
        if found is None:
            return
        self._remove_cell(found[0], evidence)

    def _remove_cell(self, key: int, evidence: CellEvidence) -> None:
        current = self._live[key]
        self._change_sequence += 1
        tombstone = replace(evidence, anchor_id=current.evidence.anchor_id)
        self._removed[key] = _Record(
            tombstone, current.world_x, current.world_y, current.anchor_pose
        )
        del self._live[key]

    def apply_anchor_update(self, update: AnchorSnapshot) -> ReferenceChange:
        self._require_capture_writable()
        self._fence_ingest()
        owner = update.odom_owner.strip()
        if update.map_ref.map_id != self.map_ref.map_id or owner != self.odom_owner:
            raise DomainError(ErrorCode.CONFLICT, "one map has one odometry owner")
        poses, review = self._usable_poses(update.anchors)
        if any(poses.get(anchor_id) != pose for anchor_id, pose in self._known_anchors.items()):
            self._advance_generation()
        self._known_anchors = dict(poses)
        self._change_sequence += 1
        self._grid = update.grid
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
        self._require_capture_writable()
        self._fence_ingest()
        if not self.is_empty():
            raise DomainError(ErrorCode.CONFLICT, "occupancy store already has records")
        if snapshot.map_ref != self.map_ref or snapshot.odom_owner != self.odom_owner:
            raise DomainError(ErrorCode.CONFLICT, "one map has one odometry owner")
        if not math.isclose(snapshot.cell_size_m, self.cell_size_m):
            raise DomainError(ErrorCode.CONFLICT, "cell size does not match the store")
        self._advance_generation()
        self._change_sequence += 1
        self._grid = snapshot.grid
        self.review_required = snapshot.review_required
        self._install(snapshot)

    def replace(self, snapshot: SlamSnapshot) -> None:
        """Switch identity, resolution, odometry owner, and sequence together."""
        self._require_capture_writable()

        self._reset_identity(snapshot.map_ref, snapshot.odom_owner, snapshot.cell_size_m)
        self._advance_generation()
        self._change_sequence += 1
        self._grid = snapshot.grid
        self.review_required = snapshot.review_required
        self._install(snapshot)

    def clear(self, map_ref: MapRef, *, odom_owner: str, cell_size_m: float) -> None:
        """Empty the layer and publish a new identity in the same update."""
        self._require_capture_writable()

        self._reset_identity(map_ref, odom_owner, cell_size_m)
        self._advance_generation()
        self._change_sequence += 1
        self._grid = GridExtent(0, 0, 0.0, 0.0)
        self.review_required = False
        self._fence_ingest()
        self._ingest_fenced = False

    def _reset_identity(self, map_ref: MapRef, odom_owner: str, cell_size_m: float) -> None:
        owner = odom_owner.strip()
        if not owner:
            raise ValueError("odometry owner is required")
        if not math.isfinite(cell_size_m) or cell_size_m <= 0.0:
            raise ValueError("cell size must be positive")
        self._map_ref = map_ref
        self._odom_owner = owner
        self._cell_size_m = float(cell_size_m)
        self._seq = 0
        self._known_anchors.clear()
        self._live.clear()
        self._removed.clear()
        self._waypoints.clear()

    def _install(self, snapshot: SlamSnapshot) -> None:
        self._known_anchors = {
            anchor.anchor_id: anchor.pose
            for anchor in snapshot.anchors
            if anchor.available and anchor.pose is not None
        }
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
        found = dict(self._known_anchors)
        for record in list(self._live.values()) + list(self._removed.values()):
            found[record.evidence.anchor_id] = record.anchor_pose
        for item in self._waypoints:
            found[item.waypoint.anchor_id] = item.anchor_pose
        return tuple(AnchorPose(anchor_id, pose, True) for anchor_id, pose in found.items())

    def _notice_anchor(self, anchor_id: str, pose: Pose2D) -> None:
        previous = self._known_anchors.get(anchor_id)
        if previous is not None and previous != pose:
            self._advance_generation()
        self._known_anchors[anchor_id] = pose

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
