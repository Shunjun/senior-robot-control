"""Reconcile long-term occupancy and waypoints with pose-graph anchors."""

import math
import time
import uuid
from collections import Counter
from collections.abc import Callable, Iterator
from contextlib import contextmanager
from copy import copy
from dataclasses import dataclass, replace
from pathlib import Path

from senior_mapping.evidence import (
    FIXED_CHANGES,
    FIXED_MEMBERS,
    FIXED_TILES,
    CellEvidence,
    FixedDelta,
    FixedWatermark,
    OccupancySnapshot,
    ProfileProvenance,
    SourceIdentity,
    VerifiedBasis,
    anchor_digest,
    fixed_text,
    fixed_uint,
)
from senior_mapping.references import (
    AnchoredWaypoint,
    AnchorPose,
    AnchorSnapshot,
    CellBinding,
    FixedSnapshot,
    GridExtent,
    ReferenceChange,
    SlamSnapshot,
    WaypointBinding,
    fixed_anchor_records,
)
from vehicle_core.errors import DomainError, ErrorCode
from vehicle_core.values import MapRef, Pose2D
from vehicle_core.workspace_limits import MAX_GRID_CELLS, MAX_GRID_SIDE

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
        self._fixed_cut: FixedSnapshot | None = None
        self._fixed_retired = False

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

    @classmethod
    def resume_verified(
        cls, snapshot: SlamSnapshot, *, backend_boot: str, source_epoch: str
    ) -> tuple["MapReferenceStore", OccupancyIngestBinding]:
        """Build a fresh writer after an independently verified load and localization.

        The production caller must first verify the actual loaded graph, its
        source/frame generation and localization; serialized boot names and
        snapshot checksums alone are not that proof. This method validates local
        structure and grants neither pose nor navigation authority. It never
        re-enables a revoked store or weakens bind_ingest's empty-map rule.
        """
        for identity in (backend_boot, source_epoch):
            if not _canonical_identity(identity) or len(identity) > 128:
                raise DomainError(ErrorCode.VALIDATION, "invalid occupancy stream identity")
        if (
            snapshot.review_required
            or len(snapshot.anchors) > _MAX_INGEST_ANCHORS
            or len(snapshot.bindings) + len(snapshot.waypoint_bindings) > _MAX_INGEST_RECORDS
        ):
            raise DomainError(ErrorCode.CONFLICT, "saved reference is not ready to resume")
        grid = snapshot.grid
        if (
            any(
                type(size) is not int or not 0 <= size <= MAX_GRID_SIDE
                for size in (grid.width, grid.height)
            )
            or grid.width * grid.height > MAX_GRID_CELLS
            or not all(_finite_number(v) for v in (grid.origin_x, grid.origin_y))
        ):
            raise DomainError(ErrorCode.VALIDATION, "saved grid is invalid or exceeds limits")
        fresh = cls(snapshot.map_ref, snapshot.odom_owner, snapshot.cell_size_m)
        poses, incomplete = fresh._usable_poses(snapshot.anchors)
        if incomplete or any(not _canonical_identity(a.anchor_id) for a in snapshot.anchors):
            raise DomainError(ErrorCode.CONFLICT, "saved graph anchors are incomplete")
        for item in snapshot.bindings:
            fresh._validate_cell_binding(item, poses)
        for item in snapshot.waypoint_bindings:
            if (
                item.waypoint.review_required
                or not _canonical_identity(item.waypoint.waypoint_id)
                or item.anchor_pose != poses.get(item.waypoint.anchor_id)
            ):
                raise DomainError(ErrorCode.CONFLICT, "saved waypoint anchor is incomplete")
        if (
            snapshot.occupancy.map_ref != snapshot.map_ref
            or snapshot.occupancy.cells
            != tuple(i.evidence for i in snapshot.bindings if not i.removed)
            or snapshot.removed != tuple(i.evidence for i in snapshot.bindings if i.removed)
        ):
            raise DomainError(ErrorCode.CONFLICT, "saved occupancy cut is inconsistent")
        fresh.replace(snapshot)
        if fresh._cells_collide():
            raise DomainError(ErrorCode.CONFLICT, "saved reference has colliding cells")
        binding = OccupancyIngestBinding(snapshot.map_ref, backend_boot, source_epoch)
        fresh._ingest_binding = binding
        fresh._ingest_fenced = False
        fresh._fixed_retired = False  # fresh verified writer; never reopens an old binding
        return fresh, binding

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

        fixed_stage = (
            self._prepare_fixed_graph(staged, sequence, anchors)
            if self._fixed_cut is not None
            else None
        )
        self._known_anchors = staged._known_anchors
        self._live = staged._live
        self._removed = staged._removed
        self._waypoints = staged._waypoints
        self._grid, self._seq = staged._grid, staged._seq
        self._change_sequence = staged._change_sequence
        self._map_generation = staged._map_generation
        progress = OccupancyIngestProgress(binding, sequence, keyframe_id)
        self._ingest_progress = progress
        if fixed_stage is not None:
            self._publish_fixed_graph(fixed_stage)
        return progress

    def commit_waypoints(
        self,
        binding: OccupancyIngestBinding,
        *,
        expected_sequence: int,
        changes: tuple[WaypointBinding, ...],
    ) -> None:
        """Update frozen bindings without revoking the live stream.

        The canonical Home/waypoint transaction owner supplies the changes and
        reconciles its revision; this method does not own that database. The
        watermark prevents an edit prepared against older corrected anchors
        from silently following a newer graph cut.
        """
        self._require_capture_writable()
        progress = self._ingest_progress
        if (
            binding is not self._ingest_binding
            or type(expected_sequence) is not int
            or expected_sequence != (0 if progress is None else progress.sequence)
        ):
            raise DomainError(ErrorCode.CONFLICT, "waypoint edit belongs to an old graph cut")
        if len(changes) > _MAX_INGEST_CHANGES:
            raise DomainError(ErrorCode.VALIDATION, "waypoint edit exceeds capture limits")
        staged = {item.waypoint.waypoint_id: item for item in self._waypoints}
        seen: set[str] = set()
        for item in changes:
            waypoint = item.waypoint
            if (
                not _canonical_identity(waypoint.waypoint_id)
                or waypoint.waypoint_id in seen
                or waypoint.review_required
                or item.anchor_pose != self._known_anchors.get(waypoint.anchor_id)
            ):
                raise DomainError(ErrorCode.VALIDATION, "waypoint edit has no current graph anchor")
            seen.add(waypoint.waypoint_id)
            staged[waypoint.waypoint_id] = _Waypoint(waypoint, item.anchor_pose)
        if (
            len(staged)
            + len(self._live)
            + len(self._removed)
            + (0 if self._fixed_cut is None else self._fixed_member_count - len(self._waypoints))
            > _MAX_INGEST_RECORDS
        ):
            raise DomainError(ErrorCode.VALIDATION, "waypoint edit exceeds reference capacity")
        if changes:
            candidate = None
            if self._fixed_cut is not None:
                self._check_fixed_owner(binding)
                candidate = replace(
                    self._fixed_cut,
                    watermark=replace(
                        self._fixed_cut.watermark, change_sequence=self.change_sequence + 1
                    ),
                    waypoint_bindings=tuple(
                        WaypointBinding(v.waypoint, v.anchor_pose) for v in staged.values()
                    ),
                )
                if self._fixed_encoded_size(candidate, {}, {}) > 8 * 1024 * 1024:
                    raise DomainError(ErrorCode.VALIDATION, "fixed_capacity: waypoint payload")
            self._waypoints = list(staged.values())
            self._change_sequence += 1
            if candidate is not None:
                self._fixed_member_count += len(self._waypoints) - len(
                    self._fixed_cut.waypoint_bindings
                )
                self._fixed_cut = candidate

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
        staged._fixed_cut = None  # prepare the legacy graph members independently
        staged._archived_fixed = None
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
            self._validate_cell_binding(item, poses)
            if evidence.cell in seen:
                raise DomainError(ErrorCode.VALIDATION, "duplicate anchored occupancy change")
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

    def _validate_cell_binding(self, item: CellBinding, poses: dict[str, Pose2D]) -> None:
        evidence = item.evidence
        world = (item.world_x, item.world_y)
        if (
            not _canonical_identity(evidence.anchor_id)
            or not isinstance(evidence.cell, tuple)
            or len(evidence.cell) != 2
            or any(type(coordinate) is not int for coordinate in evidence.cell)
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
            or self._index(*world) != evidence.cell
        ):
            raise DomainError(ErrorCode.VALIDATION, "invalid anchored occupancy change")

    def _fence_ingest(self) -> None:
        self._fixed_retired = True
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
        self._reject_fixed_v1()
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
        self._reject_fixed_v1()
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
        self._reject_fixed_v1()
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
        self._reject_fixed_v1()
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
        return (
            self._fixed_cut is None
            and getattr(self, "_archived_fixed", None) is None
            and not self._live
            and not self._removed
            and not self._waypoints
        )

    def waypoint_bindings(self) -> tuple[WaypointBinding, ...]:
        """Immutable current bindings for the canonical waypoint owner."""
        return tuple(WaypointBinding(item.waypoint, item.anchor_pose) for item in self._waypoints)

    def snapshot(
        self,
        pose_graph_prefix: Path,
        pose_graph_cells: tuple[tuple[int, int], ...],
    ) -> SlamSnapshot:
        fixed = self._fixed_cut or getattr(self, "_archived_fixed", None)
        if fixed is not None:
            from senior_mapping.references import snapshot_from_fixed

            fixed = replace(fixed, waypoint_bindings=self.waypoint_bindings())
            return snapshot_from_fixed(
                fixed,
                odom_owner=self.odom_owner,
                prefix=pose_graph_prefix,
                review_required=self.review_required,
                legacy_json=getattr(self, "_legacy_json", None),
            )
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
        self._validate_archived_fixed(snapshot)
        self._reject_fixed_v1()
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
        self._validate_archived_fixed(snapshot)
        if self._fixed_cut is not None:
            self._reject_fixed_v1()
        self._fence_ingest()
        self._reset_identity(snapshot.map_ref, snapshot.odom_owner, snapshot.cell_size_m)
        self._advance_generation()
        self._change_sequence += 1
        self._grid = snapshot.grid
        self.review_required = snapshot.review_required
        self._install(snapshot)

    def clear(self, map_ref: MapRef, *, odom_owner: str, cell_size_m: float) -> None:
        """Empty the layer and publish a new identity in the same update."""
        self._require_capture_writable()

        self._reject_fixed_v1()
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
        self._archived_fixed = None
        self._legacy_json = None
        self._known_anchors.clear()
        self._live.clear()
        self._removed.clear()
        self._waypoints.clear()

    @staticmethod
    def _validate_archived_fixed(snapshot):
        if snapshot.fixed is None:
            return
        from senior_mapping.references import validate_fixed_snapshot

        validate_fixed_snapshot(snapshot.fixed)
        cut = snapshot.fixed
        if (
            snapshot.map_ref != cut.watermark.map_ref
            or snapshot.cell_size_m != cut.grid.resolution
            or snapshot.anchors != cut.anchors
            or snapshot.waypoint_bindings != cut.waypoint_bindings
            or snapshot.grid
            != GridExtent(cut.grid.width, cut.grid.height, cut.grid.origin.x, cut.grid.origin.y)
            or snapshot.bindings
            or snapshot.removed
            or snapshot.occupancy.cells
        ):
            raise DomainError(ErrorCode.CONFLICT, "archive envelope differs from fixed cut")

    def _install(self, snapshot: SlamSnapshot) -> None:
        self._known_anchors = {
            anchor.anchor_id: anchor.pose
            for anchor in snapshot.anchors
            if anchor.available and anchor.pose is not None
        }
        self._archived_fixed = snapshot.fixed
        self._legacy_json = snapshot.legacy_json
        if snapshot.fixed is not None:
            self._change_sequence = max(
                self._change_sequence, snapshot.fixed.watermark.change_sequence
            )
        for binding in snapshot.bindings if snapshot.fixed is None else ():
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
            if pose is None or pose == record.anchor_pose:
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
            if pose == item.anchor_pose:
                # Unchanged graph vertices must not repeatedly normalize Home
                # heading or accumulate coordinate roundoff on every new scan.
                if item.waypoint.review_required:
                    item.waypoint = replace(item.waypoint, review_required=False)
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

    def _reject_fixed_v1(self) -> None:
        if self._fixed_cut is not None or getattr(self, "_archived_fixed", None) is not None:
            raise DomainError(ErrorCode.CONFLICT, "explicit V2 lifecycle required")

    def bind_fixed_context(
        self,
        binding: OccupancyIngestBinding,
        *,
        source_identity: SourceIdentity,
        host_boot_id: str,
        acceptance_floor,
        acceptance_floor_steady_ns: int,
        accepting: Callable[[], bool],
        clock: Callable[[], int] = time.monotonic_ns,
        profiles: tuple[ProfileProvenance, ...] = (),
        legacy_base=None,
    ) -> FixedSnapshot:
        """Opt-in pure C3 boundary after independently verified graph/source intake.

        The source token owner supplies the live local fence, never a JSON field.
        This grants no physical classification, navigation or production activation.
        Existing V1 records require T4 migration and cannot be silently discarded.
        """
        from senior_mapping.workspace import FixedRaster

        self._require_capture_writable()
        self._check_binding(binding)
        fixed_text(host_boot_id, 36)
        fixed_uint(acceptance_floor_steady_ns)
        if (
            self._fixed_cut is not None
            or self._fixed_retired
            or self._ingest_progress is None
            or self._live
            or self._removed
            or source_identity.backend_boot != binding.backend_boot
            or source_identity.source_epoch != binding.source_epoch
            or not callable(accepting)
            or not callable(clock)
            or not accepting()
            or len(profiles) > 128
            or len({p.profile_id for p in profiles}) != len(profiles)
            or (
                acceptance_floor is not None
                and SourceIdentity.from_key(acceptance_floor) != source_identity
            )
        ):
            raise DomainError(ErrorCode.CONFLICT, "fixed context is not independently qualified")
        anchors = tuple(sorted(self._anchor_snapshot(), key=lambda a: a.anchor_id.encode("utf-8")))
        digest = anchor_digest(fixed_anchor_records(anchors))
        extent = self.grid
        raster = FixedRaster(
            self.map_ref,
            self.cell_size_m,
            Pose2D(extent.origin_x, extent.origin_y, 0.0),
            extent.width,
            extent.height,
            True,
            False,
        )
        restored = getattr(self, "_archived_fixed", None)
        supports = () if restored is None else restored.supports
        tiles = () if restored is None else restored.free_tiles
        if restored is not None:
            legacy_base = restored.legacy_base
            profiles = tuple({p.profile_id: p for p in (*restored.profiles, *profiles)}.values())
        raster.install(anchors, supports, tiles, legacy_base)
        watermark = FixedWatermark(
            self.map_ref,
            self.map_generation,
            binding.backend_boot,
            host_boot_id,
            source_identity,
            self._ingest_progress.sequence,
            digest,
            0 if restored is None else restored.watermark.fixed_revision + 1,
            None,
            0 if restored is None else restored.watermark.evidence_revision,
            self.change_sequence,
        )
        cut = FixedSnapshot(
            watermark,
            raster.grid(),
            anchors,
            None,
            supports,
            tiles,
            legacy_base,
            self.waypoint_bindings(),
            profiles,
        )
        from senior_mapping.references import validate_fixed_snapshot

        validate_fixed_snapshot(cut)
        from senior_mapping.references import fixed_payload_size

        if fixed_payload_size(cut) > 8 * 1024 * 1024:
            raise DomainError(ErrorCode.VALIDATION, "fixed_capacity: encoded transfer")
        self._fixed_accepting, self._fixed_clock = accepting, clock
        self._fixed_floor, self._fixed_floor_ns = acceptance_floor, acceptance_floor_steady_ns
        self._fixed_accepted = None
        self._fixed_last_accepted = acceptance_floor
        self._fixed_supports = {s.support_id: s for s in supports}
        self._fixed_tiles = {t.tile_key: t for t in tiles}
        self._fixed_by_anchor = {}
        for support in supports:
            self._fixed_by_anchor.setdefault(support.anchor_id, set()).add(
                ("s", support.support_id)
            )
        for tile in tiles:
            self._fixed_by_anchor.setdefault(tile.anchor_id, set()).add(("f", tile.tile_key))
        if legacy_base is not None:
            for b in legacy_base.anchor_bindings:
                self._fixed_by_anchor.setdefault(b.anchor_id, set()).update(
                    ("l", i) for i in b.base_cell_indices
                )
        # IDs are owner-generated UUIDs; retained audit IDs need no unbounded tombstone list.
        self._fixed_raster, self._fixed_cut = raster, cut
        self._fixed_member_count = (
            len(cut.waypoint_bindings)
            + sum(s.member_count for s in supports)
            + (
                0
                if legacy_base is None
                else sum(2 * len(b.base_cell_indices) for b in legacy_base.anchor_bindings)
            )
        )
        return cut

    def _check_binding(self, binding) -> None:
        if (
            binding is not self._ingest_binding
            or binding is None
            or binding.map_ref != self.map_ref
            or self.review_required
        ):
            raise DomainError(ErrorCode.CONFLICT, "fixed source binding is stale")

    def _check_fixed_owner(self, binding) -> FixedSnapshot:
        self._require_capture_writable()
        self._check_binding(binding)
        if self._fixed_cut is None or self._fixed_retired:
            raise DomainError(ErrorCode.CONFLICT, "fixed session retired or unbound")
        if not self._fixed_accepting():
            self._fixed_retired = True
            self._fixed_accepted = None
            raise DomainError(ErrorCode.CONFLICT, "fixed owner observed retirement")
        return self._fixed_cut

    def retire_fixed(self, binding) -> None:
        self._check_binding(binding)
        self._interrupt_capture()
        self._fixed_retired = True
        self._fixed_accepted = None

    def fixed_snapshot(self, binding) -> FixedSnapshot:
        self._check_binding(binding)
        if self._fixed_cut is None:
            raise DomainError(ErrorCode.CONFLICT, "fixed mode is not bound")
        # Waypoint owner shares change_sequence; never return a stale CAS watermark.
        if self._fixed_cut.watermark.change_sequence != self.change_sequence:
            self._fixed_cut = replace(
                self._fixed_cut,
                watermark=replace(self._fixed_cut.watermark, change_sequence=self.change_sequence),
                waypoint_bindings=self.waypoint_bindings(),
            )
        return self._fixed_cut

    def accept_scan(self, binding, scan, basis: VerifiedBasis) -> None:
        """Verifier-owned accepted fact, retaining original capture/floor/deadline.

        File readers call this only after original session verification. PENDING is
        not a retirement event. Manager next/capture failure and stop/switch must
        call retire_fixed (not yet wired to NativeSession in this stage).
        """
        from senior_mapping.shared_scan import QualifiedScan

        cut = self._check_fixed_owner(binding)
        w = cut.watermark
        previous = self._fixed_last_accepted
        if (
            not isinstance(scan, QualifiedScan)
            or not isinstance(basis, VerifiedBasis)
            or SourceIdentity.from_key(scan.key) != w.source
            or scan.host_boot_id != w.host_boot_id
            or scan.key.backend_boot != w.backend_boot
            or scan.capture.time.sample_steady_ns <= self._fixed_floor_ns
            or (previous is not None and scan.key.sequence <= previous.sequence)
            or (
                basis.map_ref,
                basis.reference_generation,
                basis.graph_sequence,
                basis.anchor_digest,
            )
            != (w.map_ref, w.reference_generation, w.graph_sequence, w.anchor_digest)
            or self._known_anchors.get(basis.anchor_id) != basis.map_basis
            or not scan.capture.time.sample_steady_ns
            <= self._fixed_clock()
            < scan.valid_until_steady_ns
        ):
            raise DomainError(ErrorCode.CONFLICT, "scan does not belong to live verified cut")
        self._fixed_accepted = (scan, basis, self._observed_bounds(scan, basis))
        self._fixed_last_accepted = scan.key
        self._fixed_cut = replace(cut, basis=basis)

    def _observed_bounds(self, scan, basis):
        from senior_mapping.workspace import _inverse, _transform

        points = []
        for beam in scan.beams:
            if beam.kind not in ("hit", "clear") or beam.endpoint_odom is None:
                continue
            for p in (beam.origin_odom, beam.endpoint_odom):
                points.append(_transform(_inverse(p, basis.odom_basis), basis.map_basis))
        if not points:
            return None
        if not all(_finite_number(v) for p in points for v in p):
            raise DomainError(ErrorCode.VALIDATION, "nonfinite observed bounds")
        return (
            min(p[0] for p in points),
            min(p[1] for p in points),
            max(p[0] for p in points),
            max(p[1] for p in points),
        )

    def _check_fixed_fact(self, binding, expected):
        self._check_fixed_owner(binding)
        cut = self.fixed_snapshot(binding)
        accepted = self._fixed_accepted
        if accepted is None or expected != cut.watermark:
            raise DomainError(ErrorCode.CONFLICT, "fixed full CAS or accepted scan mismatch")
        scan, basis, bounds = accepted
        if (
            scan.host_boot_id != expected.host_boot_id
            or SourceIdentity.from_key(scan.key) != expected.source
            or (
                basis.map_ref,
                basis.reference_generation,
                basis.graph_sequence,
                basis.anchor_digest,
            )
            != (
                expected.map_ref,
                expected.reference_generation,
                expected.graph_sequence,
                expected.anchor_digest,
            )
            or self._known_anchors.get(basis.anchor_id) != basis.map_basis
            or scan.capture.time.sample_steady_ns <= self._fixed_floor_ns
            or (self._fixed_floor is not None and scan.key.sequence <= self._fixed_floor.sequence)
            or not scan.capture.time.sample_steady_ns
            <= self._fixed_clock()
            < scan.valid_until_steady_ns
        ):
            raise DomainError(ErrorCode.CONFLICT, "accepted capture expired or changed cut")
        return cut, scan, basis, bounds

    def ensure_observed_extent(self, binding, *, expected, bounds) -> FixedSnapshot:
        cut, scan, basis, actual = self._check_fixed_fact(binding, expected)
        if actual is None or bounds != actual:
            raise DomainError(ErrorCode.VALIDATION, "extent is not the accepted scan bounds")
        extent = self._fixed_extent(actual)
        if extent == self._fixed_raster.extent:
            return cut
        try:
            patch = self._fixed_raster.prepare({}, extent=extent)
            watermark = replace(
                cut.watermark,
                fixed_revision=cut.watermark.fixed_revision + 1,
                change_sequence=self.change_sequence + 1,
            )
            candidate = replace(cut, watermark=watermark, grid=patch.grid)
            self._fixed_capacity(candidate, {}, {}, 0)
        except ValueError as exc:
            raise DomainError(ErrorCode.VALIDATION, str(exc)) from exc
        self._check_fixed_fact(binding, expected)
        self._fixed_raster.publish(patch)
        self._fixed_cut = candidate
        self._change_sequence = watermark.change_sequence
        return candidate

    def _fixed_extent(self, bounds, updates=None):
        raster = self._fixed_raster
        x0, y0, x1, y1 = raster.extent
        if bounds is not None:
            lo = raster._cell(bounds[:2])
            hi = raster._cell(bounds[2:])
            x0, y0, x1, y1 = min(x0, lo[0]), min(y0, lo[1]), max(x1, hi[0] + 1), max(y1, hi[1] + 1)
        for key, projection in (updates or {}).items():
            if projection is not None:
                covered = projection.occupied | projection.free
                if key[0] == "l":
                    covered = covered.union(cell for cell, _index in projection.free_bits)
                for x, y in covered:
                    x0, y0, x1, y1 = min(x0, x), min(y0, y), max(x1, x + 1), max(y1, y + 1)
        return x0, y0, x1, y1

    def allocate_support_id(self, binding) -> str:
        """Only this owner issues new support identities, never reused on revocation."""
        self._check_fixed_owner(binding)
        identity = uuid.uuid4().hex
        if not hasattr(self, "_fixed_issued"):
            self._fixed_issued = set()
        if len(self._fixed_issued) >= FIXED_CHANGES:
            raise DomainError(ErrorCode.VALIDATION, "fixed_capacity: unconsumed support IDs")
        self._fixed_issued.add(identity)
        return identity

    def commit_evidence(
        self,
        binding,
        *,
        expected_graph_sequence: int,
        expected_anchor_digest: str,
        delta: FixedDelta,
    ) -> FixedSnapshot:
        """One synchronous owning-loop transaction; no externally visible stage writes."""
        cut, scan, basis, bounds = self._check_fixed_fact(binding, delta.expected)
        w = cut.watermark
        if (
            type(expected_graph_sequence) is not int
            or expected_graph_sequence != w.graph_sequence
            or expected_anchor_digest != w.anchor_digest
            or delta.key != scan.key
            or delta.trigger_anchor_id != basis.anchor_id
            or delta.evidence_revision <= w.evidence_revision
            or (w.evidence_key is not None and delta.key.sequence <= w.evidence_key.sequence)
        ):
            raise DomainError(ErrorCode.CONFLICT, "evidence belongs to a stale graph or scan")
        try:
            candidate, supports, tiles, patch, charge = self._prepare_fixed_delta(
                cut, delta, bounds
            )
            self._fixed_capacity(candidate, supports, tiles, charge)
        except (ValueError, OverflowError) as exc:
            raise DomainError(ErrorCode.VALIDATION, str(exc)) from exc
        self._check_fixed_fact(binding, delta.expected)
        self._publish_fixed(candidate, supports, tiles, patch)
        return candidate

    def _stage_support_removals(self, delta):
        supports = {}
        charge = 0
        handled = set()
        free_removals = []
        for removal in (*delta.revocations, *delta.support_cell_removals):
            old = self._fixed_supports.get(removal.support_id)
            if (
                old is None
                or removal.support_id in handled
                or old.updated_evidence_revision != removal.expected_support_revision
                or removal.trigger_key != delta.key
                or removal.trigger_anchor_id != delta.trigger_anchor_id
            ):
                raise ValueError("invalid support removal reference or trigger")
            handled.add(removal.support_id)
            if hasattr(removal, "local_cells"):
                removed = set(removal.local_cells)
                free_removals.append(
                    replace(
                        old,
                        local_cells=removal.local_cells,
                        local_endpoints=tuple(
                            p
                            for p in old.local_endpoints
                            if (
                                math.floor(p[0] / old.resolution_m),
                                math.floor(p[1] / old.resolution_m),
                            )
                            in removed
                        ),
                        local_surface=(),
                    )
                )
                if not removed <= set(old.local_cells):
                    raise ValueError("partial removal references missing cells")
                cells = tuple(c for c in old.local_cells if c not in removed)
                endpoints = tuple(
                    p
                    for p in old.local_endpoints
                    if (math.floor(p[0] / old.resolution_m), math.floor(p[1] / old.resolution_m))
                    not in removed
                )
                new = (
                    None
                    if not cells
                    else replace(
                        old,
                        local_cells=cells,
                        local_endpoints=endpoints,
                        local_surface=endpoints[:32],
                        updated_evidence_revision=delta.evidence_revision,
                    )
                )
                surface = () if new is None else new.local_surface
                charge += (
                    len(removed)
                    + len(old.local_endpoints)
                    - len(endpoints)
                    + 1
                    + sum((Counter(old.local_surface) - Counter(surface)).values())
                    + sum((Counter(surface) - Counter(old.local_surface)).values())
                )
            else:
                if removal.reason == "observed_free_space":
                    free_removals.append(old)
                new = None
                charge += old.member_count
            supports[old.support_id] = new
        return supports, charge, free_removals

    def _stage_support_upserts(self, cut, delta, supports):
        charge = 0
        handled = set(supports)
        profile_lookup = {p.profile_id: p for p in cut.profiles}
        upsert_ids = set()
        for s in delta.upsert_supports:
            old = supports.get(s.support_id, self._fixed_supports.get(s.support_id))
            original = self._fixed_supports.get(s.support_id)
            if (
                s.support_id in upsert_ids
                or (s.support_id in handled and old is None)
                or self._known_anchors.get(s.anchor_id) is None
                or s.resolution_m != self.cell_size_m
                or s.provenance != "geometric"
                or s.source != cut.watermark.source
                or s.last_key != delta.key
                or s.updated_evidence_revision != delta.evidence_revision
                or s.profile_id not in profile_lookup
                or profile_lookup[s.profile_id].evidence_digest != s.evidence_digest
            ):
                raise ValueError("invalid admitted support/provenance")
            if old is not None:
                if (
                    s.anchor_id != old.anchor_id
                    or s.created_evidence_revision != old.created_evidence_revision
                    or s.first_key != old.first_key
                    or not set(old.local_cells) <= set(s.local_cells)
                    or not Counter(old.local_endpoints) <= Counter(s.local_endpoints)
                ):
                    raise ValueError("support update silently shrinks or changes anchor")
            elif original is None:
                if (
                    s.support_id not in getattr(self, "_fixed_issued", ())
                    or s.created_evidence_revision != delta.evidence_revision
                ):
                    raise ValueError("support ID was not issued by this owner")
            upsert_ids.add(s.support_id)
            supports[s.support_id] = s
            charge += (
                s.member_count
                if old is None
                else (
                    1
                    + sum((Counter(s.local_endpoints) - Counter(old.local_endpoints)).values())
                    + len(set(s.local_cells) - set(old.local_cells))
                    + sum((Counter(s.local_surface) - Counter(old.local_surface)).values())
                    + sum((Counter(old.local_surface) - Counter(s.local_surface)).values())
                )
            )
        return charge

    def _stage_free_tiles(self, cut, delta, updates):
        tiles = {}
        raster = self._fixed_raster
        observed_free_regions = []
        for t in delta.free_tiles:
            old = self._fixed_tiles.get(t.tile_key)
            if (
                t.tile_key in tiles
                or t.anchor_id != delta.trigger_anchor_id
                or t.resolution_m != self.cell_size_m
                or t.source != cut.watermark.source
                or t.last_observed_key != delta.key
                or t.evidence_revision != delta.evidence_revision
                or t.tile_revision != (1 if old is None else old.tile_revision + 1)
            ):
                raise ValueError("invalid complete free replacement")
            observed_projection = raster.free_projection(t, self._known_anchors[t.anchor_id])
            observed_free_regions.append(observed_projection)
            xor = int.from_bytes(t.active_mask, "little") ^ (
                0 if old is None else int.from_bytes(old.active_mask, "little")
            )
            if xor == 0:
                # Reobserving already active bits is not a new local generation.
                continue
            tiles[t.tile_key] = t if any(t.active_mask) else None
            updates["f", t.tile_key] = None if tiles[t.tile_key] is None else observed_projection
        return tiles, observed_free_regions

    def _prepare_fixed_delta(self, cut, delta, bounds):
        from senior_mapping.workspace import _Projection

        supports, charge, free_removals = self._stage_support_removals(delta)
        charge += self._stage_support_upserts(cut, delta, supports)
        updates = {}
        raster = self._fixed_raster
        for identity, s in supports.items():
            updates["s", identity] = (
                None
                if s is None
                else raster.support_projection(s, self._known_anchors[s.anchor_id])
            )
        tiles, observed_free_regions = self._stage_free_tiles(cut, delta, updates)
        # Explicit legacy O removal must precede free conflict resolution;
        # otherwise the old base would delete the very free proof replacing it.
        legacy, explicit_dirty, explicit_charge = self._legacy_edits(cut, delta, set(), updates)
        qualified_free = set()
        for projection in observed_free_regions:
            qualified_free.update(projection.free)
        for removed in free_removals:
            projected = raster.support_projection(removed, self._known_anchors[removed.anchor_id])
            if not projected.occupied <= qualified_free:
                raise ValueError("free removal lacks complete qualified region for selected cells")
        # Every occupied write invalidates *all* intersecting older free, including
        # partial rotated bits that did not render as a complete target F cell.
        occupied = raster.occupied_cells(updates)
        # A qualified current hit invalidates older free generations even while
        # physical stable classification is unresolved. It paints no fixed O.
        scan, basis, _ = self._fixed_accepted
        from senior_mapping.references import rebase_pose
        from senior_mapping.workspace import _transform

        alignment = rebase_pose(Pose2D(0, 0, 0), basis.odom_basis, basis.map_basis)
        occupied.update(
            raster._cell(_transform(b.endpoint_odom, alignment))
            for b in scan.beams
            if b.kind == "hit" and b.endpoint_odom is not None
        )
        # New free also cannot survive an independent occupied support.
        for projection in tuple(updates.values()):
            if projection is not None:
                for cell, _local in projection.free_bits:
                    if raster._legacy_value(cell, legacy) == 100:
                        occupied.add(cell)
                    for key in raster.reverse.get(cell, ()):
                        current = updates.get(key, raster.records.get(key, _Projection()))
                        if current is not None and cell in current.occupied:
                            occupied.add(cell)
        self._invalidate_fixed_free(occupied, tiles, updates)
        charge += self._finalize_fixed_tiles(tiles, updates, self._known_anchors)
        legacy, legacy_dirty, legacy_charge = self._legacy_edits(
            replace(cut, legacy_base=legacy), replace(delta, legacy_edits=()), occupied, updates
        )
        legacy_dirty = set(legacy_dirty) | set(explicit_dirty)
        charge += legacy_charge + explicit_charge
        patch = raster.prepare(
            updates,
            extent=self._fixed_extent(bounds, updates),
            legacy=legacy,
            legacy_dirty=legacy_dirty,
        )
        watermark = replace(
            cut.watermark,
            fixed_revision=cut.watermark.fixed_revision + 1,
            evidence_key=delta.key,
            evidence_revision=delta.evidence_revision,
            change_sequence=self.change_sequence + 1,
        )
        candidate = replace(
            cut,
            watermark=watermark,
            grid=patch.grid,
            supports=self._patched_fixed_members(
                self._fixed_supports, supports, cut.supports, lambda s: s.support_id
            ),
            free_tiles=self._patched_fixed_members(
                self._fixed_tiles, tiles, cut.free_tiles, lambda t: t.tile_key
            ),
            legacy_base=legacy,
        )
        return candidate, supports, tiles, patch, charge

    @staticmethod
    def _patched_fixed_members(current, edits, original, sort_key):
        if not edits:
            return original
        members = [edits.get(k, v) for k, v in current.items() if edits.get(k, v) is not None]
        members.extend(v for k, v in edits.items() if k not in current and v is not None)
        return tuple(sorted(members, key=sort_key))

    def _invalidate_fixed_free(self, occupied, tiles, updates, poses=None):
        charge = 0
        raster = self._fixed_raster
        poses = self._known_anchors if poses is None else poses
        for key, cells in raster.overlapping_free(occupied, updates).items():
            tile = tiles.get(key, self._fixed_tiles.get(key))
            if tile is None:
                continue
            mask = bytearray(tile.active_mask)
            changed = 0
            for x, y in cells:
                bit = 32 * (y - 32 * tile.tile_y) + (x - 32 * tile.tile_x)
                if mask[bit // 8] & (1 << (bit % 8)):
                    mask[bit // 8] &= ~(1 << (bit % 8))
                    changed += 1
            if changed:
                charge += changed
                updated = replace(
                    tile, active_mask=bytes(mask), tile_revision=tile.tile_revision + 1
                )
                tiles[key] = updated if any(mask) else None
                updates["f", key] = (
                    None
                    if tiles[key] is None
                    else raster.free_projection(updated, poses[tile.anchor_id])
                )
        return charge

    def _finalize_fixed_tiles(self, tiles, updates, poses):
        """Charge committed bit differences and retain no-op local generations."""
        charge = 0
        for key in tuple(tiles):
            old = self._fixed_tiles.get(key)
            new = tiles[key]
            before = 0 if old is None else int.from_bytes(old.active_mask, "little")
            after = 0 if new is None else int.from_bytes(new.active_mask, "little")
            charge += (before ^ after).bit_count()
            if before == after:
                del tiles[key]
                if old is None:
                    updates.pop(("f", key), None)
                else:
                    updates["f", key] = self._fixed_raster.free_projection(
                        old, poses[old.anchor_id]
                    )
            elif old is not None and new is not None and not after & ~before:
                # Derived clearing must not claim a new free observation.
                tiles[key] = replace(
                    new,
                    last_observed_key=old.last_observed_key,
                    source=old.source,
                    evidence_revision=old.evidence_revision,
                )
        return charge

    def _legacy_edits(self, cut, delta, occupied, updates, poses=None):
        base = cut.legacy_base
        if base is None:
            if delta.legacy_edits:
                raise ValueError("legacy edit without base")
            return None, (), 0
        mask = bytearray(base.live_mask)
        cleared = set()
        for edit in delta.legacy_edits:
            if (
                edit.base_id != base.base_id
                or edit.trigger_key != delta.key
                or edit.trigger_anchor_id != delta.trigger_anchor_id
            ):
                raise ValueError("legacy trigger mismatch")
            for i, value in zip(edit.cell_indices, edit.expected_values, strict=False):
                if (
                    i >= len(base.grid.cells)
                    or i in cleared
                    or base.grid.cells[i] != value
                    or not mask[i // 8] & (1 << (i % 8))
                ):
                    raise ValueError("legacy expected value mismatch")
                cleared.add(i)
        raster = self._fixed_raster
        for indices in raster.overlapping_free(occupied, updates, kind="l").values():
            cleared.update(indices)
        for cell in occupied:
            for i in raster.legacy_indices(cell, base):
                if (
                    i not in raster.legacy_bound
                    and base.grid.cells[i] == 0
                    and mask[i // 8] & (1 << (i % 8))
                ):
                    cleared.add(i)
        for i in cleared:
            mask[i // 8] &= ~(1 << (i % 8))
        new = replace(base, live_mask=bytes(mask)) if cleared else base
        dirty = set()
        bound = []
        for i in cleared:
            if i in raster.legacy_bound:
                bound.append(i)
            else:
                dirty.update(cell for cell, _area in raster.legacy_coverage(i, base))
        updates.update(
            raster.legacy_projections(new, self._known_anchors if poses is None else poses, bound)
        )
        return new, dirty, len(cleared)

    def _fixed_capacity(self, candidate, supports, tiles, charge):
        # Count aggregate full members, including surface summaries, not just IDs.
        count = self._fixed_member_count
        for key, new in supports.items():
            old = self._fixed_supports.get(key)
            count += (0 if new is None else new.member_count) - (
                0 if old is None else old.member_count
            )
        tile_count = len(self._fixed_tiles) + sum(
            (new is not None) - (key in self._fixed_tiles) for key, new in tiles.items()
        )
        if count > FIXED_MEMBERS or tile_count > FIXED_TILES or charge > FIXED_CHANGES:
            raise ValueError(
                "fixed_capacity: complete transaction exceeds member/tile/change bound"
            )
        # Strict actual encoded budgets are checked before publication; a cached
        # component-size ledger avoids serializing anchors/members every scan.
        size = self._fixed_encoded_size(candidate, supports, tiles)
        if size > 8 * 1024 * 1024:
            raise ValueError("fixed_capacity: encoded transfer")
        return count

    def _fixed_encoded_size(self, cut, supports, tiles):
        # Filled by references' explicit codec component encoder; no second schema.
        from senior_mapping.references import fixed_payload_size

        if not hasattr(self, "_fixed_size_cache"):
            self._fixed_size_cache = {}
        return fixed_payload_size(cut, self._fixed_size_cache)

    def _publish_fixed(self, candidate, supports, tiles, patch):
        for key, new in supports.items():
            old = self._fixed_supports.get(key)
            self._fixed_member_count += (0 if new is None else new.member_count) - (
                0 if old is None else old.member_count
            )
            if old is not None:
                self._fixed_by_anchor[old.anchor_id].discard(("s", key))
            if new is None:
                self._fixed_supports.pop(key, None)
            else:
                self._fixed_supports[key] = new
                self._fixed_by_anchor.setdefault(new.anchor_id, set()).add(("s", key))
            getattr(self, "_fixed_issued", set()).discard(key)
        for key, new in tiles.items():
            old = self._fixed_tiles.get(key)
            if old is not None:
                self._fixed_by_anchor[old.anchor_id].discard(("f", key))
            if new is None:
                self._fixed_tiles.pop(key, None)
            else:
                self._fixed_tiles[key] = new
                self._fixed_by_anchor.setdefault(new.anchor_id, set()).add(("f", key))
        self._fixed_raster.publish(patch)
        self._fixed_raster.legacy = candidate.legacy_base
        self._fixed_cut = candidate
        self._change_sequence = candidate.watermark.change_sequence

    def _prepare_fixed_graph(self, staged, sequence, anchors):
        cut = self._fixed_cut
        if self._fixed_retired or not self._fixed_accepting():
            self._fixed_retired = True
            raise DomainError(ErrorCode.CONFLICT, "fixed session retired")
        # Graph correction costs only affected anchors' cached members; graph cut
        # validation itself retains the original complete-graph continuity check.
        changed = {k for k, v in staged._known_anchors.items() if self._known_anchors.get(k) != v}
        raster = self._fixed_raster
        updates = {}
        affected_legacy = []
        for anchor in changed:
            for kind, key in self._fixed_by_anchor.get(anchor, ()):
                if kind == "s":
                    s = self._fixed_supports[key]
                    updates[kind, key] = raster.support_projection(s, staged._known_anchors[anchor])
                elif kind == "f":
                    t = self._fixed_tiles[key]
                    updates[kind, key] = raster.free_projection(t, staged._known_anchors[anchor])
                else:
                    affected_legacy.append(key)
        tiles = {}
        base = cut.legacy_base
        if base is not None:
            updates.update(raster.legacy_projections(base, staged._known_anchors, affected_legacy))
        occupied = raster.occupied_cells(updates)
        for projection in tuple(updates.values()):
            if projection is not None:
                for cell, _local in projection.free_bits:
                    if raster._legacy_value(cell, base) == 100:
                        occupied.add(cell)
                    for key in raster.reverse.get(cell, ()):
                        current = updates.get(key, raster.records.get(key))
                        if current is not None and cell in current.occupied:
                            occupied.add(cell)
        self._invalidate_fixed_free(occupied, tiles, updates, staged._known_anchors)
        charge = self._finalize_fixed_tiles(tiles, updates, staged._known_anchors)
        # Same legacy edit engine, with no synthetic observation or new evidence key.
        from types import SimpleNamespace

        legacy_delta = SimpleNamespace(legacy_edits=())
        base, legacy_dirty, legacy_charge = self._legacy_edits(
            cut, legacy_delta, occupied, updates, staged._known_anchors
        )
        charge += legacy_charge
        grid = anchors.grid
        lo = raster._cell((grid.origin_x, grid.origin_y))
        hi = raster._cell(
            (
                grid.origin_x + grid.width * self.cell_size_m,
                grid.origin_y + grid.height * self.cell_size_m,
            )
        )
        x0, y0, x1, y1 = raster.extent
        ux0, uy0, ux1, uy1 = self._fixed_extent(None, updates)
        patch = raster.prepare(
            updates,
            extent=(min(ux0, lo[0]), min(uy0, lo[1]), max(ux1, hi[0]), max(uy1, hi[1])),
            legacy=base,
            legacy_dirty=legacy_dirty,
        )
        watermark = replace(
            cut.watermark,
            reference_generation=staged.map_generation,
            graph_sequence=sequence,
            anchor_digest=anchor_digest(fixed_anchor_records(anchors.anchors)),
            fixed_revision=cut.watermark.fixed_revision + 1,
            change_sequence=staged.change_sequence,
        )
        tt = tuple(
            tiles.get(k, v) for k, v in self._fixed_tiles.items() if tiles.get(k, v) is not None
        )
        candidate = replace(
            cut,
            watermark=watermark,
            anchors=tuple(sorted(anchors.anchors, key=lambda a: a.anchor_id.encode("utf-8"))),
            basis=None,
            grid=patch.grid,
            free_tiles=tuple(sorted(tt, key=lambda t: t.tile_key)),
            legacy_base=base,
            waypoint_bindings=staged.waypoint_bindings(),
        )
        self._fixed_capacity(candidate, {}, tiles, charge)
        floor = self._fixed_floor_ns
        if cut.watermark.reference_generation != watermark.reference_generation:
            floor = self._fixed_clock()
            fixed_uint(floor)
        self._check_fixed_owner(self._ingest_binding)
        return candidate, {}, tiles, patch, floor

    def _publish_fixed_graph(self, stage):
        self._publish_fixed(*stage[:4])
        self._fixed_floor_ns = stage[4]
        self._fixed_accepted = None
        self._fixed_floor = self._fixed_last_accepted
        # Original graph basis changes require a fresh scan but graph PENDING does not.

    def local_support_candidates(self, binding, bounds, *, limit=8):
        """Query only current spatial contributions; saturation never truncates identity."""
        self._check_fixed_owner(binding)
        raster = self._fixed_raster
        lo, hi = raster._cell(bounds[:2]), raster._cell(bounds[2:])
        if (hi[0] - lo[0] + 1) * (hi[1] - lo[1] + 1) > 8192:
            return None
        identities = set()
        for y in range(lo[1], hi[1] + 1):
            for x in range(lo[0], hi[0] + 1):
                for kind, identity in raster.reverse.get((x, y), ()):
                    if kind == "s":
                        identities.add(identity)
                        if len(identities) > limit:
                            return None
        return tuple(self._fixed_supports[i] for i in sorted(identities))
