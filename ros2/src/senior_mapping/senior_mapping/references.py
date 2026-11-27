"""Map identity, pose-graph anchors, and the saved localization/occupancy split."""

import base64
import hashlib
import json
import math
import os
import shutil
import struct
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import TYPE_CHECKING

from senior_mapping.evidence import (
    FIXED_MEMBERS,
    FIXED_PIXELS,
    FIXED_TILES,
    CellEvidence,
    FixedDelta,
    FixedWatermark,
    FreeTile,
    LegacyMaskEdit,
    OccupancySnapshot,
    ProfileProvenance,
    SourceIdentity,
    StableSupport,
    SupportCellRemoval,
    SupportRevocation,
    VerifiedBasis,
    anchor_digest,
    fixed_digest,
    fixed_pose,
    fixed_real,
    fixed_text,
    fixed_uint,
)
from senior_mapping.shared_scan import ScanKey
from vehicle_core.errors import DomainError, ErrorCode
from vehicle_core.values import MapRef, Pose2D

if TYPE_CHECKING:
    from senior_mapping.workspace import WorkspaceGrid


def rebase_pose(pose: Pose2D, old_anchor: Pose2D, new_anchor: Pose2D) -> Pose2D:
    """Keep a physical pose attached to its original anchor through a graph correction."""
    if old_anchor == new_anchor:
        return pose
    dx, dy = pose.x - old_anchor.x, pose.y - old_anchor.y
    c, s = math.cos(old_anchor.yaw), math.sin(old_anchor.yaw)
    local_x, local_y = c * dx + s * dy, -s * dx + c * dy
    c, s = math.cos(new_anchor.yaw), math.sin(new_anchor.yaw)
    return Pose2D(
        new_anchor.x + c * local_x - s * local_y,
        new_anchor.y + s * local_x + c * local_y,
        math.remainder(pose.yaw + new_anchor.yaw - old_anchor.yaw, math.tau),
    )


FORMAT_VERSION = 2
LEGACY_FORMAT_VERSION = 1
# Humble 2.6.10 graph_visualization publishes node spheres in this namespace.
# Edge lines use slam_toolbox_edges and are not anchors.
NODE_MARKER_NAMESPACE = "slam_toolbox"


@dataclass(frozen=True)
class GridExtent:
    width: int
    height: int
    origin_x: float
    origin_y: float


@dataclass(frozen=True)
class AnchorPose:
    anchor_id: str
    pose: Pose2D | None
    available: bool


@dataclass(frozen=True)
class AnchorSnapshot:
    map_ref: MapRef
    odom_owner: str
    anchors: tuple[AnchorPose, ...]
    grid: GridExtent


@dataclass(frozen=True)
class AnchoredWaypoint:
    waypoint_id: str
    pose: Pose2D
    anchor_id: str
    review_required: bool


@dataclass(frozen=True)
class GraphNodeMarker:
    """One sphere from slam_toolbox 2.6.10 graph_visualization, without ROS types.

    That message carries corrected x/y and an identity quaternion. ``yaw`` is
    None when the marker did not include a corrected heading.
    """

    namespace: str
    marker_id: int
    x: float
    y: float
    yaw: float | None


@dataclass(frozen=True)
class CellBinding:
    evidence: CellEvidence
    world_x: float
    world_y: float
    anchor_pose: Pose2D
    removed: bool


@dataclass(frozen=True)
class WaypointBinding:
    waypoint: AnchoredWaypoint
    anchor_pose: Pose2D


@dataclass(frozen=True)
class ReferenceChange:
    map_ref: MapRef
    cells: tuple[CellEvidence, ...]
    waypoints: tuple[AnchoredWaypoint, ...]
    review_required: bool
    odom_owner: str


@dataclass(frozen=True)
class SlamSnapshot:
    """Pose graph for localization, plus the long-term occupancy layer.

    ``pose_graph_cells`` are cells the localization raster would paint. They
    are not copied into ``occupancy``. ``removed`` stays removed across reload.
    """

    map_ref: MapRef
    odom_owner: str
    cell_size_m: float
    pose_graph_prefix: Path
    occupancy: OccupancySnapshot
    removed: tuple[CellEvidence, ...]
    pose_graph_cells: tuple[tuple[int, int], ...]
    anchors: tuple[AnchorPose, ...]
    grid: GridExtent
    bindings: tuple[CellBinding, ...]
    waypoint_bindings: tuple[WaypointBinding, ...]
    review_required: bool
    fixed: "FixedSnapshot | None" = None
    legacy_json: bytes | None = None

    def write(self, destination: Path) -> None:
        destination.mkdir(parents=True, exist_ok=True)
        if self.fixed is not None:
            payload = encode_snapshot(self)
            temporary = destination / "occupancy.json.tmp"
            temporary.write_bytes(payload)
            durable_replace(temporary, destination / "occupancy.json")
            return
        payload = {
            "format_version": LEGACY_FORMAT_VERSION,
            "map_id": self.map_ref.map_id,
            "reference_revision": self.map_ref.reference_revision,
            "odom_owner": self.odom_owner,
            "cell_size_m": self.cell_size_m,
            "pose_graph_prefix": str(self.pose_graph_prefix),
            "review_required": self.review_required,
            "grid": _grid_payload(self.grid),
            "pose_graph_cells": [list(cell) for cell in self.pose_graph_cells],
            "anchors": [_anchor_payload(anchor) for anchor in self.anchors],
            "bindings": [_binding_payload(binding) for binding in self.bindings],
            "waypoint_bindings": [_waypoint_payload(binding) for binding in self.waypoint_bindings],
        }
        target = destination / "occupancy.json"
        temporary = destination / "occupancy.json.tmp"
        temporary.write_text(json.dumps(payload), encoding="utf-8")
        durable_replace(temporary, target)

    @classmethod
    def read(cls, destination: Path) -> "SlamSnapshot":
        return cls.from_json((destination / "occupancy.json").read_bytes())

    @classmethod
    def from_json(cls, data: bytes) -> "SlamSnapshot":
        body = _mapping(json.loads(data))
        if _whole(body.get("format_version")) != LEGACY_FORMAT_VERSION:
            raise ValueError("unsupported occupancy snapshot")
        bindings = tuple(_binding(_item) for _item in _sequence(body.get("bindings")))
        live = tuple(item.evidence for item in bindings if not item.removed)
        removed = tuple(item.evidence for item in bindings if item.removed)
        map_ref = MapRef(_text(body.get("map_id")), _whole(body.get("reference_revision")))
        return cls(
            map_ref,
            _text(body.get("odom_owner")),
            _real(body.get("cell_size_m")),
            Path(_text(body.get("pose_graph_prefix"))),
            OccupancySnapshot(map_ref, live),
            removed,
            tuple(_cell(item) for item in _sequence(body.get("pose_graph_cells"))),
            tuple(_anchor(item) for item in _sequence(body.get("anchors"))),
            _grid(_mapping(body.get("grid"))),
            bindings,
            tuple(_waypoint(item) for item in _sequence(body.get("waypoint_bindings"))),
            _flag(body.get("review_required")),
        )


def durable_replace(temporary: Path, destination: Path) -> None:
    """Replace a file, then fsync the file and its parent directory."""

    _fsync_path(temporary)
    os.replace(temporary, destination)
    _fsync_path(destination)
    _fsync_path(destination.parent)


def recover_move_tree(destination: Path) -> None:
    """Recover the last complete directory after an interrupted replacement."""
    displaced = destination.with_name(f".{destination.name}.previous")
    if displaced.is_symlink() or destination.is_symlink():
        raise DomainError(ErrorCode.VALIDATION, "map path contains a symlink")
    if not displaced.exists():
        return
    if destination.exists():
        shutil.rmtree(displaced)
    else:
        os.replace(displaced, destination)
    _fsync_path(destination.parent)


def durable_move_tree(temporary: Path, destination: Path) -> None:
    """Publish a complete tree with a recoverable previous generation."""
    for path in sorted(temporary.rglob("*"), key=lambda item: len(item.parts), reverse=True):
        if path.is_symlink():
            raise DomainError(ErrorCode.VALIDATION, "map tree contains a symlink")
        if path.is_file() or path.is_dir():
            _fsync_path(path)
    _fsync_path(temporary)
    durable_mkdir(destination.parent)
    recover_move_tree(destination)
    displaced = destination.with_name(f".{destination.name}.previous")
    if destination.exists():
        os.replace(destination, displaced)
        _fsync_path(destination.parent)
    os.replace(temporary, destination)
    _fsync_path(destination.parent)
    if displaced.exists():
        shutil.rmtree(displaced)
        _fsync_path(destination.parent)


def durable_mkdir(directory: Path) -> None:
    missing: list[Path] = []
    current = directory
    while not current.exists():
        missing.append(current)
        current = current.parent
    directory.mkdir(parents=True, exist_ok=True)
    for path in reversed(missing):
        _fsync_path(path)
        _fsync_path(path.parent)
    _fsync_path(directory)


def _fsync_path(path: Path) -> None:
    fd = os.open(path, os.O_RDONLY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def anchor_from_graph_marker(marker: GraphNodeMarker) -> AnchorPose | None:
    """Turn one Humble graph marker into an anchor, or ignore an edge marker.

    2.6.10 node markers do not carry ``GetCorrectedPose`` heading. A missing
    yaw stays unavailable so the store will not invent a rotation.
    """

    if marker.namespace != NODE_MARKER_NAMESPACE:
        return None
    anchor_id = str(marker.marker_id)
    if marker.yaw is None:
        return AnchorPose(anchor_id, None, False)
    return AnchorPose(anchor_id, Pose2D(marker.x, marker.y, marker.yaw), True)


def _grid_payload(grid: GridExtent) -> dict[str, float | int]:
    return {
        "width": grid.width,
        "height": grid.height,
        "origin_x": grid.origin_x,
        "origin_y": grid.origin_y,
    }


def _anchor_payload(anchor: AnchorPose) -> dict[str, object]:
    pose = anchor.pose
    return {
        "anchor_id": anchor.anchor_id,
        "available": anchor.available and pose is not None,
        "x": None if pose is None else pose.x,
        "y": None if pose is None else pose.y,
        "yaw": None if pose is None else pose.yaw,
    }


def _binding_payload(binding: CellBinding) -> dict[str, object]:
    evidence = binding.evidence
    return {
        "cell": [evidence.cell[0], evidence.cell[1]],
        "anchor_id": evidence.anchor_id,
        "hit_count": evidence.hit_count,
        "free_count": evidence.free_count,
        "last_valid_time": evidence.last_valid_time,
        "belief": evidence.belief,
        "change_source": evidence.change_source,
        "world_x": binding.world_x,
        "world_y": binding.world_y,
        "anchor_x": binding.anchor_pose.x,
        "anchor_y": binding.anchor_pose.y,
        "anchor_yaw": binding.anchor_pose.yaw,
        "removed": binding.removed,
    }


def _waypoint_payload(binding: WaypointBinding) -> dict[str, object]:
    waypoint = binding.waypoint
    return {
        "waypoint_id": waypoint.waypoint_id,
        "x": waypoint.pose.x,
        "y": waypoint.pose.y,
        "yaw": waypoint.pose.yaw,
        "anchor_id": waypoint.anchor_id,
        "review_required": waypoint.review_required,
        "anchor_x": binding.anchor_pose.x,
        "anchor_y": binding.anchor_pose.y,
        "anchor_yaw": binding.anchor_pose.yaw,
    }


def _mapping(value: object) -> dict[str, object]:
    if not isinstance(value, dict):
        raise ValueError("expected object")
    body: dict[str, object] = {}
    for key, item in value.items():
        if not isinstance(key, str):
            raise ValueError("expected string key")
        body[key] = item
    return body


def _sequence(value: object) -> tuple[object, ...]:
    if not isinstance(value, list):
        raise ValueError("expected list")
    return tuple(value)


def _item(value: object) -> dict[str, object]:
    return _mapping(value)


def _text(value: object) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError("expected text")
    return value


def _whole(value: object) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError("expected integer")
    return value


def _real(value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("expected number")
    return float(value)


def _flag(value: object) -> bool:
    if not isinstance(value, bool):
        raise ValueError("expected flag")
    return value


def _cell(value: object) -> tuple[int, int]:
    pair = _sequence(value)
    if len(pair) != 2:
        raise ValueError("expected cell")
    return (_whole(pair[0]), _whole(pair[1]))


def _grid(body: dict[str, object]) -> GridExtent:
    return GridExtent(
        _whole(body.get("width")),
        _whole(body.get("height")),
        _real(body.get("origin_x")),
        _real(body.get("origin_y")),
    )


def _anchor(value: object) -> AnchorPose:
    body = _item(value)
    available = _flag(body.get("available"))
    pose: Pose2D | None
    if body.get("x") is None or body.get("y") is None or body.get("yaw") is None:
        pose = None
        available = False
    else:
        pose = Pose2D(_real(body.get("x")), _real(body.get("y")), _real(body.get("yaw")))
    return AnchorPose(_text(body.get("anchor_id")), pose, available)


def _binding(value: object) -> CellBinding:
    body = _item(value)
    source = body.get("change_source")
    if source not in ("geometric_confirmation", "observed_free_space"):
        raise ValueError("expected change source")
    evidence = CellEvidence(
        _cell(body.get("cell")),
        _text(body.get("anchor_id")),
        _whole(body.get("hit_count")),
        _whole(body.get("free_count")),
        _real(body.get("last_valid_time")),
        _real(body.get("belief")),
        source,
    )
    return CellBinding(
        evidence,
        _real(body.get("world_x")),
        _real(body.get("world_y")),
        Pose2D(
            _real(body.get("anchor_x")),
            _real(body.get("anchor_y")),
            _real(body.get("anchor_yaw")),
        ),
        _flag(body.get("removed")),
    )


def _waypoint(value: object) -> WaypointBinding:
    body = _item(value)
    waypoint = AnchoredWaypoint(
        _text(body.get("waypoint_id")),
        Pose2D(_real(body.get("x")), _real(body.get("y")), _real(body.get("yaw"))),
        _text(body.get("anchor_id")),
        _flag(body.get("review_required")),
    )
    pose = Pose2D(
        _real(body.get("anchor_x")),
        _real(body.get("anchor_y")),
        _real(body.get("anchor_yaw")),
    )
    return WaypointBinding(waypoint, pose)


# C3 V2 records; the explicit legacy parser/writer retain V1 wire compatibility.
# //Compatible: AbsoluteGrid reuses WorkspaceGrid's geometry, flags and dense bytes.


@dataclass(frozen=True)
class LegacyAnchorBinding:
    anchor_id: str
    anchor_pose_at_migration: Pose2D
    base_cell_indices: tuple[int, ...]
    local_quads: tuple[tuple[float, ...], ...]
    original_bindings: tuple[CellBinding, ...]

    def __post_init__(self) -> None:
        fixed_text(self.anchor_id)
        fixed_pose(self.anchor_pose_at_migration)
        if any(
            type(value) is not tuple
            for value in (self.base_cell_indices, self.local_quads, self.original_bindings)
        ):
            raise ValueError("legacy binding members must be immutable tuples")
        # Copy caller-owned inner lists before this C3 record can enter a cut.
        object.__setattr__(self, "local_quads", tuple(tuple(q) for q in self.local_quads))
        n = len(self.base_cell_indices)
        if (
            n > FIXED_MEMBERS
            or n != len(self.local_quads)
            or n != len(self.original_bindings)
            or tuple(sorted(set(self.base_cell_indices))) != self.base_cell_indices
        ):
            raise ValueError("invalid legacy binding lengths/indices")
        for index, quad, binding in zip(
            self.base_cell_indices, self.local_quads, self.original_bindings, strict=False
        ):
            fixed_uint(index, 32)
            # CellEvidence is an unchanged V1 type; close its alias at C3 admission.
            if type(binding.evidence.cell) is not tuple:
                raise ValueError("legacy evidence cell must be an immutable tuple")
            if (
                len(quad) != 8
                or binding.evidence.anchor_id != self.anchor_id
                or binding.anchor_pose != self.anchor_pose_at_migration
            ):
                raise ValueError("invalid legacy original binding")
            for value in quad:
                fixed_real(value)


@dataclass(frozen=True)
class LegacyBase:
    base_id: str
    grid: "WorkspaceGrid"
    provenance: str
    original_archive_digest: str
    migration_id: str
    live_mask: bytes
    anchor_bindings: tuple[LegacyAnchorBinding, ...]

    def __post_init__(self) -> None:
        fixed_digest(self.base_id)
        fixed_digest(self.original_archive_digest)
        fixed_text(self.migration_id)
        if type(self.anchor_bindings) is not tuple or len(self.anchor_bindings) > FIXED_MEMBERS:
            raise ValueError("legacy bindings must be a bounded immutable tuple")
        n = self.grid.width * self.grid.height
        if (
            n > FIXED_PIXELS
            or type(self.live_mask) is not bytes
            or len(self.live_mask) != (n + 7) // 8
            or (n % 8 and self.live_mask[-1] >> (n % 8))
            or self.provenance not in ("legacy", "legacy-derived")
        ):
            raise ValueError("invalid legacy base mask/provenance")


@dataclass(frozen=True)
class FixedSnapshot:
    watermark: FixedWatermark
    grid: "WorkspaceGrid"
    anchors: tuple[AnchorPose, ...]
    basis: VerifiedBasis | None
    supports: tuple[StableSupport, ...]
    free_tiles: tuple[FreeTile, ...]
    legacy_base: LegacyBase | None
    waypoint_bindings: tuple[WaypointBinding, ...]
    profiles: tuple[ProfileProvenance, ...]
    schema_version: int = 2
    localization_role: str = "localization_only"

    def __post_init__(self) -> None:
        if (
            type(self.schema_version) is not int
            or self.schema_version != 2
            or self.localization_role != "localization_only"
        ):
            raise ValueError("unsupported fixed schema/role")
        for members, limit in (
            (self.anchors, 16384),
            (self.supports, FIXED_MEMBERS),
            (self.free_tiles, FIXED_TILES),
            (self.waypoint_bindings, FIXED_MEMBERS),
            (self.profiles, 128),
        ):
            if type(members) is not tuple or len(members) > limit:
                raise ValueError("fixed_capacity: members must be bounded immutable tuples")


def fixed_anchor_records(anchors: tuple[AnchorPose, ...]) -> list[dict]:
    return [
        dict(
            anchor_id=a.anchor_id,
            available=a.available,
            pose=None if a.pose is None else dict(x=a.pose.x, y=a.pose.y, yaw=a.pose.yaw),
        )
        for a in anchors
    ]


def validate_fixed_snapshot(cut: FixedSnapshot, *, render: bool = True) -> None:
    """Validate a complete untrusted cut; never recover one half from the other."""
    from senior_mapping.workspace import FixedRaster, fixed_grid_id, validate_absolute_grid

    w, grid = cut.watermark, cut.grid
    validate_absolute_grid(grid)
    if (
        w.map_ref != grid.map_ref
        or w.anchor_digest != anchor_digest(fixed_anchor_records(cut.anchors))
        or len(cut.anchors) > 16_384
        or len(cut.profiles) > 128
    ):
        raise ValueError("fixed graph identity mismatch")
    poses = {a.anchor_id: a.pose for a in cut.anchors if a.available}
    profiles = {p.profile_id: p for p in cut.profiles}
    if len(profiles) != len(cut.profiles):
        raise ValueError("duplicate profile")
    ids = set()
    members = len(cut.waypoint_bindings)
    for s in cut.supports:
        if (
            s.support_id in ids
            or poses.get(s.anchor_id) is None
            or s.resolution_m != grid.resolution
        ):
            raise ValueError("invalid support reference")
        ids.add(s.support_id)
        members += s.member_count
        if s.updated_evidence_revision > w.evidence_revision:
            raise ValueError("support evidence is ahead of cut")
        if s.provenance == "geometric" and (
            s.profile_id not in profiles
            or profiles[s.profile_id].evidence_digest != s.evidence_digest
        ):
            raise ValueError("missing support profile provenance")
    keys = set()
    for t in cut.free_tiles:
        if (
            t.tile_key in keys
            or poses.get(t.anchor_id) is None
            or t.resolution_m != grid.resolution
            or t.evidence_revision > w.evidence_revision
        ):
            raise ValueError("invalid free reference")
        keys.add(t.tile_key)
    if len(keys) > FIXED_TILES:
        raise ValueError("fixed_capacity: free tiles")
    waypoint_ids = set()
    for binding in cut.waypoint_bindings:
        if binding.waypoint.waypoint_id in waypoint_ids or binding.anchor_pose != poses.get(
            binding.waypoint.anchor_id
        ):
            raise ValueError("invalid waypoint binding")
        _waypoint_v2(_waypoint_payload(binding))
        waypoint_ids.add(binding.waypoint.waypoint_id)
    if cut.basis is not None:
        b = cut.basis
        if (b.map_ref, b.reference_generation, b.graph_sequence, b.anchor_digest) != (
            w.map_ref,
            w.reference_generation,
            w.graph_sequence,
            w.anchor_digest,
        ) or poses.get(b.anchor_id) != b.map_basis:
            raise ValueError("basis does not belong to cut")
    base = cut.legacy_base
    if base is not None:
        validate_absolute_grid(base.grid)
        if (
            base.grid.map_ref != w.map_ref
            or base.grid.resolution != grid.resolution
            or base.base_id != base.grid.grid_id
            or base.grid.grid_id
            != fixed_grid_id(
                base.grid.map_ref,
                base.grid.width,
                base.grid.height,
                base.grid.resolution,
                base.grid.origin,
                base.grid.cells,
            )
        ):
            raise ValueError("legacy base identity mismatch")
        indices = set()
        binding_ids = set()
        for b in base.anchor_bindings:
            if b.anchor_id in binding_ids:
                raise ValueError("duplicate legacy anchor binding")
            binding_ids.add(b.anchor_id)
            if poses.get(b.anchor_id) is None:
                raise ValueError("unavailable legacy anchor")
            members += len(b.base_cell_indices) * 2
            for index, quad, original in zip(
                b.base_cell_indices, b.local_quads, b.original_bindings, strict=False
            ):
                if index in indices or index >= len(base.grid.cells):
                    raise ValueError("duplicate or out of range legacy index")
                indices.add(index)
                _binding_v2(_binding_payload(original))
                if original.removed or original.evidence.cell != (
                    math.floor(original.world_x / grid.resolution),
                    math.floor(original.world_y / grid.resolution),
                ):
                    raise ValueError("invalid original legacy binding")
                x = base.grid.origin.x + (index % base.grid.width) * grid.resolution
                y = base.grid.origin.y + (index // base.grid.width) * grid.resolution
                if not (
                    x <= original.world_x < x + grid.resolution
                    and y <= original.world_y < y + grid.resolution
                ):
                    raise ValueError("original binding does not belong to base index")
                p = b.anchor_pose_at_migration
                c, s = math.cos(p.yaw), math.sin(p.yaw)
                expected = tuple(
                    v
                    for wx, wy in (
                        (x, y),
                        (x + grid.resolution, y),
                        (x + grid.resolution, y + grid.resolution),
                        (x, y + grid.resolution),
                    )
                    for v in (c * (wx - p.x) + s * (wy - p.y), -s * (wx - p.x) + c * (wy - p.y))
                )
                if any(
                    not math.isclose(a, z, rel_tol=0, abs_tol=1e-10)
                    for a, z in zip(quad, expected, strict=False)
                ):
                    raise ValueError("legacy quad changed geometry")
    if members > FIXED_MEMBERS:
        raise ValueError("fixed_capacity: aggregate members")
    if grid.grid_id != fixed_grid_id(
        grid.map_ref, grid.width, grid.height, grid.resolution, grid.origin, grid.cells
    ):
        raise ValueError("absolute grid digest mismatch")
    if render:
        raster = FixedRaster(
            grid.map_ref,
            grid.resolution,
            grid.origin,
            grid.width,
            grid.height,
            grid.display_only,
            grid.waypoint_writes,
        )
        raster.install(cut.anchors, cut.supports, cut.free_tiles, cut.legacy_base)
        if raster.grid().cells != grid.cells:
            raise ValueError("absolute grid and evidence disagree")


# Wire codecs are deliberately explicit. Unknown fields and duplicate JSON keys fail.

_FIXED_PAYLOAD_BYTES = 64 * 1024 * 1024
_FIXED_TRANSFER_BYTES = 8 * 1024 * 1024


def _fixed_json(value: object) -> bytes:
    def normalized(v):
        if type(v) is float:
            return 0.0 if v == 0 else v
        if isinstance(v, dict):
            return {k: normalized(item) for k, item in v.items()}
        if isinstance(v, (tuple, list)):
            return [normalized(item) for item in v]
        return v

    return json.dumps(
        normalized(value), sort_keys=True, separators=(",", ":"), ensure_ascii=True, allow_nan=False
    ).encode("utf-8")


def _fixed_object(value: object, fields: str) -> dict:
    if type(value) is not dict or set(value) != set(fields.split()):
        raise ValueError("invalid fixed record fields")
    return value


def _fixed_b64(data: bytes) -> str:
    return base64.b64encode(data).decode("ascii")


def _fixed_bytes(value: str, size: int) -> bytes:
    if type(value) is not str or len(value) != 4 * ((size + 2) // 3):
        raise ValueError("invalid encoded length")
    data = base64.b64decode(value, validate=True)
    if len(data) != size or _fixed_b64(data) != value:
        raise ValueError("invalid base64")
    return data


def _fixed_array(value: object, limit: int) -> list:
    if type(value) is not list or len(value) > limit:
        raise ValueError("invalid bounded array")
    return value


def _packed(points: tuple, fmt: str) -> dict:
    return dict(
        count=len(points),
        encoding="le-" + fmt,
        data_b64=_fixed_b64(
            b"".join(
                struct.pack("<" + fmt, *(0.0 if type(v) is float and v == 0 else v for v in p))
                for p in points
            )
        ),
    )


def _unpacked(body: dict, fmt: str, limit: int | None = None) -> tuple:
    body = _fixed_object(body, "count encoding data_b64")
    fixed_uint(body["count"])
    if (limit is not None and body["count"] > limit) or body["encoding"] != "le-" + fmt:
        raise ValueError("invalid packed coordinate encoding")
    return tuple(
        struct.iter_unpack(
            "<" + fmt, _fixed_bytes(body["data_b64"], body["count"] * struct.calcsize("<" + fmt))
        )
    )


def _key_payload(key: ScanKey | None) -> dict | None:
    return None if key is None else asdict(key)


def _key_v2(body: dict | None) -> ScanKey | None:
    if body is None:
        return None
    # //Compatible: archived keys have no source stamp. Preserve None as
    # unavailable; never invent a timestamp from their diagnostic sequence.
    fields = dict(body)
    fields.setdefault("stamp_ns", None)
    return ScanKey(
        **_fixed_object(
            fields,
            "backend_boot bridge_boot_id source_epoch capture_generation clock_generation "
            "raw_gid bridge_gid sequence stamp_ns",
        )
    )


def _source_v2(body: dict | None) -> SourceIdentity | None:
    if body is None:
        return None
    return SourceIdentity(
        **_fixed_object(
            body,
            "backend_boot bridge_boot_id source_epoch capture_generation clock_generation "
            "raw_gid bridge_gid",
        )
    )


def _map_v2(body: dict) -> MapRef:
    body = _fixed_object(body, "map_id reference_revision")
    fixed_text(body["map_id"])
    fixed_uint(body["reference_revision"])
    return MapRef(**body)


def _pose_v2(body: dict) -> Pose2D:
    p = Pose2D(**_fixed_object(body, "x y yaw"))
    fixed_pose(p)
    return p


def _absolute_payload(grid: "WorkspaceGrid") -> dict:
    return dict(
        width=grid.width,
        height=grid.height,
        resolution_m=float(grid.resolution),
        origin=[float(grid.origin.x), float(grid.origin.y), 0.0],
        display_only=grid.display_only,
        waypoint_writes=grid.waypoint_writes,
        cells_b64=_fixed_b64(grid.cells),
        encoding="u8-ofu-v1",
        grid_id=grid.grid_id,
    )


def _absolute_v2(body: dict, map_ref: MapRef) -> "WorkspaceGrid":
    from senior_mapping.workspace import WorkspaceGrid, validate_absolute_grid

    body = _fixed_object(
        body,
        "width height resolution_m origin display_only waypoint_writes cells_b64 encoding grid_id",
    )
    for v in (body["width"], body["height"]):
        fixed_uint(v)
        if not 0 < v <= 65536:
            raise ValueError("invalid absolute side")
    n = body["width"] * body["height"]
    origin = _fixed_array(body["origin"], 3)
    if (
        n > FIXED_PIXELS
        or len(origin) != 3
        or type(origin[2]) not in (int, float)
        or origin[2] != 0
        or body["encoding"] != "u8-ofu-v1"
    ):
        raise ValueError("invalid absolute geometry/encoding")
    grid = WorkspaceGrid(
        map_ref,
        body["grid_id"],
        body["width"],
        body["height"],
        body["resolution_m"],
        Pose2D(*origin),
        _fixed_bytes(body["cells_b64"], n),
        body["display_only"],
        body["waypoint_writes"],
    )
    validate_absolute_grid(grid)
    return grid


def _support_payload(s: StableSupport) -> dict:
    return dict(
        support_id=s.support_id,
        anchor_id=s.anchor_id,
        resolution_m=float(s.resolution_m),
        local_surface=s.local_surface,
        local_endpoints=_packed(s.local_endpoints, "dd"),
        local_cells=_packed(s.local_cells, "ii"),
        first_key=_key_payload(s.first_key),
        last_key=_key_payload(s.last_key),
        source=None if s.source is None else asdict(s.source),
        provenance=s.provenance,
        profile_id=s.profile_id,
        evidence_digest=s.evidence_digest,
        created_evidence_revision=s.created_evidence_revision,
        updated_evidence_revision=s.updated_evidence_revision,
    )


def _support_v2(body: dict) -> StableSupport:
    b = dict(
        _fixed_object(
            body,
            "support_id anchor_id resolution_m local_surface local_endpoints local_cells "
            "first_key last_key source provenance profile_id evidence_digest "
            "created_evidence_revision updated_evidence_revision",
        )
    )
    b["local_surface"] = tuple(tuple(p) for p in _fixed_array(b["local_surface"], 32))
    b["local_endpoints"] = _unpacked(b["local_endpoints"], "dd")
    b["local_cells"] = _unpacked(b["local_cells"], "ii")
    b["first_key"], b["last_key"], b["source"] = (
        _key_v2(b["first_key"]),
        _key_v2(b["last_key"]),
        _source_v2(b["source"]),
    )
    return StableSupport(**b)


def encode_fixed_snapshot(cut: FixedSnapshot, *, transfer: bool = False) -> bytes:
    """Explicit, complete V2 payload; not selected by archive/checkpoint V1."""
    validate_fixed_snapshot(cut)
    watermark = _watermark_payload(cut.watermark)
    basis = (
        None
        if cut.basis is None
        else dict(
            cut.basis.__dict__,
            map_basis=asdict(cut.basis.map_basis),
            odom_basis=asdict(cut.basis.odom_basis),
            map_ref=asdict(cut.basis.map_ref),
        )
    )
    tiles = [_tile_payload(t) for t in sorted(cut.free_tiles, key=lambda t: t.tile_key)]
    base = cut.legacy_base
    legacy = (
        None
        if base is None
        else dict(
            base_id=base.base_id,
            grid=_absolute_payload(base.grid),
            provenance=base.provenance,
            original_archive_digest=base.original_archive_digest,
            migration_id=base.migration_id,
            live_mask=_fixed_b64(base.live_mask),
            anchor_bindings=[
                dict(
                    anchor_id=b.anchor_id,
                    anchor_pose_at_migration=asdict(b.anchor_pose_at_migration),
                    base_cell_indices=b.base_cell_indices,
                    local_quads_b64=_fixed_b64(
                        b"".join(struct.pack("<8d", *q) for q in b.local_quads)
                    ),
                    count=len(b.local_quads),
                    original_bindings=[_binding_payload(x) for x in b.original_bindings],
                )
                for b in sorted(base.anchor_bindings, key=lambda b: b.anchor_id)
            ],
        )
    )
    body = dict(
        schema_version=2,
        localization_role=cut.localization_role,
        watermark=watermark,
        grid=_absolute_payload(cut.grid),
        anchors=sorted(
            fixed_anchor_records(cut.anchors), key=lambda a: a["anchor_id"].encode("utf-8")
        ),
        basis=basis,
        supports=[_support_payload(s) for s in sorted(cut.supports, key=lambda s: s.support_id)],
        free_tiles=tiles,
        legacy_base=legacy,
        waypoint_bindings=[
            _waypoint_payload(x)
            for x in sorted(cut.waypoint_bindings, key=lambda x: x.waypoint.waypoint_id)
        ],
        profiles=[
            dict(
                profile_id=p.profile_id,
                evidence_digest=p.evidence_digest,
                parameters=dict(p.parameters),
            )
            for p in sorted(cut.profiles, key=lambda p: p.profile_id)
        ],
    )
    data = _fixed_json(body)
    if len(data) > (_FIXED_TRANSFER_BYTES if transfer else _FIXED_PAYLOAD_BYTES):
        raise ValueError("fixed_capacity: encoded payload")
    return data


def decode_fixed_snapshot(data: bytes, *, transfer: bool = False) -> FixedSnapshot:
    b = _fixed_object(
        _fixed_load(data, transfer=transfer),
        "schema_version localization_role watermark grid anchors basis supports free_tiles "
        "legacy_base waypoint_bindings profiles",
    )
    watermark = _watermark_v2(b["watermark"])
    anchors = []
    for a in _fixed_array(b["anchors"], 16_384):
        a = _fixed_object(a, "anchor_id pose available")
        anchors.append(
            AnchorPose(
                a["anchor_id"], None if a["pose"] is None else _pose_v2(a["pose"]), a["available"]
            )
        )
    basis = b["basis"]
    if basis is not None:
        basis = dict(
            _fixed_object(
                basis,
                "anchor_id map_basis odom_basis graph_sequence anchor_digest map_ref "
                "reference_generation",
            )
        )
        basis["map_ref"], basis["map_basis"], basis["odom_basis"] = (
            _map_v2(basis["map_ref"]),
            _pose_v2(basis["map_basis"]),
            _pose_v2(basis["odom_basis"]),
        )
        basis = VerifiedBasis(**basis)
    tiles = [_tile_v2(t) for t in _fixed_array(b["free_tiles"], FIXED_TILES)]
    legacy = b["legacy_base"]
    if legacy is not None:
        legacy = dict(
            _fixed_object(
                legacy,
                "base_id grid provenance original_archive_digest migration_id live_mask "
                "anchor_bindings",
            )
        )
        legacy["grid"] = _absolute_v2(legacy["grid"], watermark.map_ref)
        legacy["live_mask"] = _fixed_bytes(
            legacy["live_mask"], (len(legacy["grid"].cells) + 7) // 8
        )
        bindings = []
        for item in _fixed_array(legacy["anchor_bindings"], FIXED_MEMBERS):
            item = dict(
                _fixed_object(
                    item,
                    "anchor_id anchor_pose_at_migration base_cell_indices local_quads_b64 count "
                    "original_bindings",
                )
            )
            item["anchor_pose_at_migration"] = _pose_v2(item["anchor_pose_at_migration"])
            item["base_cell_indices"] = tuple(
                _fixed_array(item["base_cell_indices"], FIXED_MEMBERS)
            )
            fixed_uint(item["count"])
            if item["count"] > FIXED_MEMBERS:
                raise ValueError("legacy quad count exceeds cap")
            item["local_quads"] = tuple(
                struct.iter_unpack(
                    "<8d", _fixed_bytes(item.pop("local_quads_b64"), item.pop("count") * 64)
                )
            )
            item["original_bindings"] = tuple(
                _binding_v2(x) for x in _fixed_array(item["original_bindings"], FIXED_MEMBERS)
            )
            bindings.append(LegacyAnchorBinding(**item))
        legacy["anchor_bindings"] = tuple(bindings)
        legacy = LegacyBase(**legacy)
    cut = FixedSnapshot(
        watermark,
        _absolute_v2(b["grid"], watermark.map_ref),
        tuple(anchors),
        basis,
        tuple(_support_v2(s) for s in _fixed_array(b["supports"], FIXED_MEMBERS)),
        tuple(tiles),
        legacy,
        tuple(_waypoint_v2(x) for x in _fixed_array(b["waypoint_bindings"], FIXED_MEMBERS)),
        tuple(
            ProfileProvenance(**_fixed_object(p, "profile_id evidence_digest parameters"))
            for p in _fixed_array(b["profiles"], 128)
        ),
        b["schema_version"],
        b["localization_role"],
    )
    validate_fixed_snapshot(cut)
    return cut


def fixed_payload_size(cut: FixedSnapshot, cache: dict | None = None) -> int:
    """Exact codec length without serializing dense bytes or unchanged members.

    Cache contains object identities and lengths only, no second authoritative
    state. Rejected candidates may replace this disposable cache safely.
    """
    cache = {} if cache is None else cache

    def piece(key, obj, build):
        prior = cache.get(key)
        if prior is not None and prior[0] is obj:
            return prior[1]
        size = len(_fixed_json(build()))
        cache[key] = (obj, size)
        return size

    def array(name, items, identity, build):
        collection_key = ("array", name)
        prior = cache.get(collection_key)
        if prior is not None and prior[0] is items:
            return prior[1]
        lengths = []
        live = set()
        for item in items:
            key = (name, identity(item))
            live.add(key)
            lengths.append(piece(key, item, lambda item=item: build(item)))
        for key in tuple(cache):
            if isinstance(key, tuple) and key[0] == name and key not in live:
                del cache[key]
        total = 2 + sum(lengths) + max(0, len(lengths) - 1)
        cache[collection_key] = (items, total)
        return total

    def grid_size(g):
        b = dict(
            width=g.width,
            height=g.height,
            resolution_m=float(g.resolution),
            origin=[float(g.origin.x), float(g.origin.y), 0.0],
            display_only=g.display_only,
            waypoint_writes=g.waypoint_writes,
            cells_b64="",
            encoding="u8-ofu-v1",
            grid_id=g.grid_id,
        )
        return len(_fixed_json(b)) + 4 * ((len(g.cells) + 2) // 3)

    w = cut.watermark
    sizes = {
        "schema_version": 1,
        "localization_role": len(_fixed_json(cut.localization_role)),
        "watermark": len(
            _fixed_json(
                dict(
                    w.__dict__,
                    map_ref=asdict(w.map_ref),
                    source=None if w.source is None else asdict(w.source),
                    evidence_key=_key_payload(w.evidence_key),
                )
            )
        ),
        "grid": grid_size(cut.grid),
        "anchors": piece(
            "anchors",
            cut.anchors,
            lambda: sorted(
                fixed_anchor_records(cut.anchors), key=lambda a: a["anchor_id"].encode("utf-8")
            ),
        ),
        "basis": 4
        if cut.basis is None
        else len(
            _fixed_json(
                dict(
                    cut.basis.__dict__,
                    map_ref=asdict(cut.basis.map_ref),
                    map_basis=asdict(cut.basis.map_basis),
                    odom_basis=asdict(cut.basis.odom_basis),
                )
            )
        ),
        "supports": array("supports", cut.supports, lambda s: s.support_id, _support_payload),
        "free_tiles": array(
            "tiles",
            cut.free_tiles,
            lambda t: t.tile_key,
            lambda t: dict(
                t.__dict__,
                active_mask=_fixed_b64(t.active_mask),
                last_observed_key=_key_payload(t.last_observed_key),
                source=asdict(t.source),
            ),
        ),
        "waypoint_bindings": piece(
            "waypoints",
            cut.waypoint_bindings,
            lambda: [
                _waypoint_payload(x)
                for x in sorted(cut.waypoint_bindings, key=lambda x: x.waypoint.waypoint_id)
            ],
        ),
        "profiles": piece(
            "profiles",
            cut.profiles,
            lambda: [
                dict(
                    profile_id=p.profile_id,
                    evidence_digest=p.evidence_digest,
                    parameters=dict(p.parameters),
                )
                for p in sorted(cut.profiles, key=lambda p: p.profile_id)
            ],
        ),
    }
    base = cut.legacy_base
    if base is None:
        sizes["legacy_base"] = 4
    else:
        bindings = piece(
            "legacy-bindings",
            base.anchor_bindings,
            lambda: [
                dict(
                    anchor_id=b.anchor_id,
                    anchor_pose_at_migration=asdict(b.anchor_pose_at_migration),
                    base_cell_indices=b.base_cell_indices,
                    local_quads_b64=_fixed_b64(
                        b"".join(struct.pack("<8d", *q) for q in b.local_quads)
                    ),
                    count=len(b.local_quads),
                    original_bindings=[_binding_payload(x) for x in b.original_bindings],
                )
                for b in sorted(base.anchor_bindings, key=lambda b: b.anchor_id)
            ],
        )
        fields = dict(
            base_id=len(_fixed_json(base.base_id)),
            grid=grid_size(base.grid),
            provenance=len(_fixed_json(base.provenance)),
            original_archive_digest=66,
            migration_id=len(_fixed_json(base.migration_id)),
            live_mask=2 + 4 * ((len(base.live_mask) + 2) // 3),
            anchor_bindings=bindings,
        )
        sizes["legacy_base"] = (
            2 + sum(len(_fixed_json(k)) + 1 + v for k, v in fields.items()) + len(fields) - 1
        )
    return 2 + sum(len(_fixed_json(k)) + 1 + v for k, v in sizes.items()) + len(sizes) - 1


def _binding_v2(body: dict) -> CellBinding:
    b = _fixed_object(
        body,
        "cell anchor_id hit_count free_count last_valid_time belief change_source world_x "
        "world_y anchor_x anchor_y anchor_yaw removed",
    )
    fixed_text(b["anchor_id"])
    for k in ("hit_count", "free_count"):
        fixed_uint(b[k])
    for k in (
        "last_valid_time",
        "belief",
        "world_x",
        "world_y",
        "anchor_x",
        "anchor_y",
        "anchor_yaw",
    ):
        fixed_real(b[k])
    pair = _fixed_array(b["cell"], 2)
    if len(pair) != 2 or any(type(v) is not int or not -(1 << 31) <= v < 1 << 31 for v in pair):
        raise ValueError("invalid original legacy cell")
    return _binding(b)


def _waypoint_v2(body: dict) -> WaypointBinding:
    b = _fixed_object(
        body, "waypoint_id x y yaw anchor_id review_required anchor_x anchor_y anchor_yaw"
    )
    for k in ("waypoint_id", "anchor_id"):
        fixed_text(b[k])
    for k in ("x", "y", "yaw", "anchor_x", "anchor_y", "anchor_yaw"):
        fixed_real(b[k])
    return _waypoint(b)


def _watermark_payload(w: FixedWatermark) -> dict:
    return dict(
        w.__dict__,
        map_ref=asdict(w.map_ref),
        source=None if w.source is None else asdict(w.source),
        evidence_key=_key_payload(w.evidence_key),
    )


def _watermark_v2(body: dict) -> FixedWatermark:
    b = dict(
        _fixed_object(
            body,
            "map_ref reference_generation backend_boot host_boot_id source graph_sequence "
            "anchor_digest fixed_revision evidence_key evidence_revision change_sequence",
        )
    )
    b["map_ref"], b["source"], b["evidence_key"] = (
        _map_v2(b["map_ref"]),
        _source_v2(b["source"]),
        _key_v2(b["evidence_key"]),
    )
    return FixedWatermark(**b)


def _tile_payload(t: FreeTile) -> dict:
    return dict(
        t.__dict__,
        active_mask=_fixed_b64(t.active_mask),
        last_observed_key=_key_payload(t.last_observed_key),
        source=asdict(t.source),
    )


def _tile_v2(body: dict) -> FreeTile:
    b = dict(
        _fixed_object(
            body,
            "anchor_id resolution_m tile_x tile_y active_mask tile_revision "
            "last_observed_key source evidence_revision",
        )
    )
    b["active_mask"], b["last_observed_key"], b["source"] = (
        _fixed_bytes(b["active_mask"], 128),
        _key_v2(b["last_observed_key"]),
        _source_v2(b["source"]),
    )
    return FreeTile(**b)


def _fixed_load(data: bytes, *, transfer: bool) -> dict:
    if type(data) is not bytes or len(data) > (
        _FIXED_TRANSFER_BYTES if transfer else _FIXED_PAYLOAD_BYTES
    ):
        raise ValueError("fixed payload exceeds bound")

    def pairs(items):
        result = {}
        for k, v in items:
            if k in result:
                raise ValueError("duplicate JSON key")
            result[k] = v
        return result

    return json.loads(data, object_pairs_hook=pairs)


def encode_fixed_delta(delta: FixedDelta) -> bytes:
    """Same C3 field encodings for a prepared batch; never conveys authorization."""

    def removal(item):
        body = dict(item.__dict__, trigger_key=_key_payload(item.trigger_key))
        if isinstance(item, SupportCellRemoval):
            body["local_cells"] = _packed(item.local_cells, "ii")
        return body

    body = dict(
        schema_version=2,
        expected=_watermark_payload(delta.expected),
        key=_key_payload(delta.key),
        evidence_revision=delta.evidence_revision,
        trigger_anchor_id=delta.trigger_anchor_id,
        upsert_supports=[
            _support_payload(s) for s in sorted(delta.upsert_supports, key=lambda s: s.support_id)
        ],
        revocations=[removal(r) for r in sorted(delta.revocations, key=lambda r: r.support_id)],
        support_cell_removals=[
            removal(r) for r in sorted(delta.support_cell_removals, key=lambda r: r.support_id)
        ],
        free_tiles=[_tile_payload(t) for t in sorted(delta.free_tiles, key=lambda t: t.tile_key)],
        legacy_edits=[
            removal(e)
            for e in sorted(delta.legacy_edits, key=lambda e: (e.base_id, e.cell_indices))
        ],
    )
    data = _fixed_json(body)
    if len(data) > _FIXED_TRANSFER_BYTES:
        raise ValueError("fixed_capacity: encoded delta")
    return data


def decode_fixed_delta(data: bytes) -> FixedDelta:
    b = dict(
        _fixed_object(
            _fixed_load(data, transfer=True),
            "schema_version expected key evidence_revision trigger_anchor_id upsert_supports "
            "revocations support_cell_removals free_tiles legacy_edits",
        )
    )
    version = b.pop("schema_version")
    if type(version) is not int or version != 2:
        raise ValueError("unsupported fixed delta schema")
    b["expected"], b["key"] = _watermark_v2(b["expected"]), _key_v2(b["key"])
    b["upsert_supports"] = tuple(_support_v2(s) for s in _fixed_array(b["upsert_supports"], 8192))
    b["free_tiles"] = tuple(_tile_v2(t) for t in _fixed_array(b["free_tiles"], 1024))
    for name, kind, fields in (
        (
            "revocations",
            SupportRevocation,
            "support_id expected_support_revision reason trigger_key trigger_anchor_id",
        ),
        (
            "support_cell_removals",
            SupportCellRemoval,
            "support_id expected_support_revision local_cells reason trigger_key trigger_anchor_id",
        ),
        (
            "legacy_edits",
            LegacyMaskEdit,
            "base_id cell_indices expected_values reason trigger_key trigger_anchor_id",
        ),
    ):
        items = []
        for item in _fixed_array(b[name], 8192):
            item = dict(_fixed_object(item, fields))
            item["trigger_key"] = _key_v2(item["trigger_key"])
            if kind is SupportCellRemoval:
                item["local_cells"] = _unpacked(item["local_cells"], "ii")
            if kind is LegacyMaskEdit:
                item["cell_indices"] = tuple(_fixed_array(item["cell_indices"], 8192))
                item["expected_values"] = tuple(_fixed_array(item["expected_values"], 8192))
            items.append(kind(**item))
        b[name] = tuple(items)
    return FixedDelta(**b)


def encode_snapshot(snapshot: SlamSnapshot) -> bytes:
    """V2 archive envelope; the retained V1 parser deliberately rejects it."""
    import base64

    cut = snapshot.fixed
    if cut is None:
        raise ValueError("V2 snapshot requires authoritative fixed layers")
    validate_fixed_snapshot(cut)
    body = dict(
        format_version=FORMAT_VERSION,
        odom_owner=snapshot.odom_owner,
        pose_graph_prefix=str(snapshot.pose_graph_prefix),
        review_required=snapshot.review_required,
        fixed=json.loads(encode_fixed_snapshot(cut)),
        legacy_audit_b64=None
        if snapshot.legacy_json is None
        else base64.b64encode(snapshot.legacy_json).decode("ascii"),
    )
    body["checksum"] = hashlib.sha256(_fixed_json(body)).hexdigest()
    return _fixed_json(body)


def snapshot_from_fixed(
    cut, *, odom_owner, prefix=Path("pose_graph"), review_required=False, legacy_json=None
):
    grid = cut.grid
    return SlamSnapshot(
        cut.watermark.map_ref,
        odom_owner,
        grid.resolution,
        prefix,
        OccupancySnapshot(cut.watermark.map_ref, ()),
        (),
        (),
        cut.anchors,
        GridExtent(grid.width, grid.height, grid.origin.x, grid.origin.y),
        (),
        cut.waypoint_bindings,
        review_required,
        cut,
        legacy_json,
    )


def load_snapshot(data: bytes) -> SlamSnapshot:
    """Explicit V1/V2 dispatch. Malformed V2 never tries the legacy parser."""
    import base64

    if len(data) > 64 * 1024 * 1024:
        raise ValueError("snapshot exceeds byte budget")
    body = _fixed_load(data, transfer=False)
    if body.get("format_version") == LEGACY_FORMAT_VERSION:
        from dataclasses import replace

        return replace(SlamSnapshot.from_json(data), legacy_json=data)
    fields = (
        "format_version odom_owner pose_graph_prefix review_required "
        "fixed legacy_audit_b64 checksum"
    )
    _fixed_object(body, fields)
    if _whole(body["format_version"]) != FORMAT_VERSION:
        raise ValueError("unsupported snapshot version")
    checksum = body.pop("checksum")
    if checksum != hashlib.sha256(_fixed_json(body)).hexdigest():
        raise ValueError("snapshot checksum mismatch")
    cut = decode_fixed_snapshot(_fixed_json(body["fixed"]))
    legacy = body["legacy_audit_b64"]
    legacy = None if legacy is None else base64.b64decode(legacy, validate=True)
    if legacy is not None:
        original = SlamSnapshot.from_json(legacy)
        if original.map_ref != cut.watermark.map_ref:
            raise ValueError("legacy audit belongs to another map")
    return snapshot_from_fixed(
        cut,
        odom_owner=_text(body["odom_owner"]),
        prefix=Path(_text(body["pose_graph_prefix"])),
        review_required=_flag(body["review_required"]),
        legacy_json=legacy,
    )


def migrate_legacy(
    snapshot: SlamSnapshot, *, legacy_grid, native_legacy_grid, original_archive_digest=None
) -> FixedSnapshot:
    """One explicit V1 conversion; native raster is admitted only at this boundary.

    Original bindings/removed/Home remain byte exact in the archive audit member.
    Native-only geometry has legacy-derived provenance, never physical stability.
    """
    import tempfile
    import uuid

    from senior_mapping.workspace import (
        FixedRaster,
        WorkspaceGrid,
        fixed_grid_id,
        render_stable_grid,
    )

    if snapshot.fixed is not None:
        raise ValueError("legacy migration accepts only an unmigrated V1 cut")
    if (
        len(snapshot.anchors) > 16384
        or len(snapshot.bindings) + len(snapshot.waypoint_bindings) > FIXED_MEMBERS
    ):
        raise ValueError("fixed_capacity: legacy migration members")
    with tempfile.TemporaryDirectory() as raw:
        snapshot.write(Path(raw))
        original = snapshot.legacy_json or (Path(raw) / "occupancy.json").read_bytes()
    grid = (
        native_legacy_grid
        or legacy_grid
        or render_stable_grid(
            snapshot.map_ref, "migration", snapshot.grid, snapshot.cell_size_m, snapshot.occupancy
        )
    )
    if grid.map_ref != snapshot.map_ref or grid.resolution != snapshot.cell_size_m:
        raise ValueError("legacy raster identity mismatch")
    r = grid.resolution
    lo_x = min(snapshot.grid.origin_x, grid.origin.x, *(b.world_x for b in snapshot.bindings))
    lo_y = min(snapshot.grid.origin_y, grid.origin.y, *(b.world_y for b in snapshot.bindings))
    hi_x = max(
        snapshot.grid.origin_x + snapshot.grid.width * r,
        grid.origin.x + grid.width * r,
        *(b.world_x + r for b in snapshot.bindings),
    )
    hi_y = max(
        snapshot.grid.origin_y + snapshot.grid.height * r,
        grid.origin.y + grid.height * r,
        *(b.world_y + r for b in snapshot.bindings),
    )
    x0, y0 = math.floor((lo_x - grid.origin.x) / r), math.floor((lo_y - grid.origin.y) / r)
    width = math.ceil((hi_x - grid.origin.x) / r) - x0
    height = math.ceil((hi_y - grid.origin.y) / r) - y0
    if width * height > FIXED_PIXELS:
        raise ValueError("fixed_capacity: legacy migration extent")
    cells = bytearray(b"\xff") * (width * height)
    for y in range(grid.height):
        offset = (y - y0) * width - x0
        cells[offset : offset + grid.width] = grid.cells[y * grid.width : (y + 1) * grid.width]
    grid = WorkspaceGrid(
        grid.map_ref,
        "migration-lattice",
        width,
        height,
        r,
        Pose2D(grid.origin.x + x0 * r, grid.origin.y + y0 * r, 0),
        bytes(cells),
        display_only=True,
    )

    def index(x, y):
        ix, iy = (
            math.floor((x - grid.origin.x) / grid.resolution),
            math.floor((y - grid.origin.y) / grid.resolution),
        )
        if not 0 <= ix < grid.width or not 0 <= iy < grid.height:
            raise ValueError("legacy binding is outside migration raster")
        return iy * grid.width + ix

    grouped = {}
    for b in snapshot.bindings:
        i = index(b.world_x, b.world_y)
        cells[i] = 255 if b.removed else 100
        if b.removed:
            continue
        x, y = (
            grid.origin.x + (i % grid.width) * grid.resolution,
            grid.origin.y + (i // grid.width) * grid.resolution,
        )
        p = b.anchor_pose
        c, si = math.cos(p.yaw), math.sin(p.yaw)
        quad = tuple(
            v
            for wx, wy in (
                (x, y),
                (x + grid.resolution, y),
                (x + grid.resolution, y + grid.resolution),
                (x, y + grid.resolution),
            )
            for v in (c * (wx - p.x) + si * (wy - p.y), -si * (wx - p.x) + c * (wy - p.y))
        )
        grouped.setdefault(b.evidence.anchor_id, []).append((i, quad, b))
    bindings = []
    for anchor, rows in sorted(grouped.items()):
        rows.sort(key=lambda row: row[0])
        bindings.append(
            LegacyAnchorBinding(
                anchor,
                rows[0][2].anchor_pose,
                tuple(row[0] for row in rows),
                tuple(row[1] for row in rows),
                tuple(row[2] for row in rows),
            )
        )
    grid = WorkspaceGrid(
        snapshot.map_ref,
        fixed_grid_id(
            snapshot.map_ref, grid.width, grid.height, grid.resolution, grid.origin, bytes(cells)
        ),
        grid.width,
        grid.height,
        grid.resolution,
        grid.origin,
        bytes(cells),
        display_only=True,
    )
    mask = bytearray((len(cells) + 7) // 8)
    for i, value in enumerate(cells):
        if value != 255:
            mask[i // 8] |= 1 << (i % 8)
    base = LegacyBase(
        grid.grid_id,
        grid,
        "legacy-derived",
        original_archive_digest or hashlib.sha256(original).hexdigest(),
        uuid.uuid4().hex,
        bytes(mask),
        tuple(bindings),
    )
    raster = FixedRaster(
        grid.map_ref, grid.resolution, grid.origin, grid.width, grid.height, True, False
    )
    raster.install(snapshot.anchors, (), (), base)
    watermark = FixedWatermark(
        snapshot.map_ref,
        uuid.uuid4().hex,
        "legacy-migration",
        "legacy",
        None,
        0,
        anchor_digest(fixed_anchor_records(snapshot.anchors)),
        0,
        None,
        0,
        0,
    )
    cut = FixedSnapshot(
        watermark,
        raster.grid(),
        snapshot.anchors,
        None,
        (),
        (),
        base,
        snapshot.waypoint_bindings,
        (),
    )
    validate_fixed_snapshot(cut)
    return cut
