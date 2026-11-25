"""Observation and evidence records for the long-term occupancy layer."""

import hashlib
import json
import math
from collections.abc import Iterable, Mapping
from dataclasses import dataclass
from typing import Literal

from senior_mapping.shared_scan import ScanKey

# //Compatible: retain original public and old pickle lookup paths as identical aliases.
from vehicle_core.values import HitKind as HitKind
from vehicle_core.values import MapRef, Pose2D
from vehicle_core.values import RangeRay as RangeRay

ChangeSource = Literal["geometric_confirmation", "observed_free_space"]


def canonical_anchor_records(anchors: Iterable[Mapping[str, object]]) -> list[dict[str, object]]:
    """Validate complete C3 records; return a fresh, UTF-8 ID-sorted binary64 cut.

    Native graph wire rows must be explicitly converted by their owning boundary.
    Unavailable anchors remain in the digest with pose=None, never silently dropped.
    """
    records: list[dict[str, object]] = []
    seen: set[str] = set()
    for anchor in anchors:
        if not isinstance(anchor, Mapping) or set(anchor) != {"anchor_id", "available", "pose"}:
            raise ValueError("invalid anchor record fields")
        identity = anchor["anchor_id"]
        if (
            not isinstance(identity, str)
            or identity != identity.strip()
            or not 1 <= len(identity.encode("utf-8")) <= 128
            or identity in seen
        ):
            raise ValueError("invalid or duplicate canonical anchor ID")
        seen.add(identity)
        available, pose = anchor["available"], anchor["pose"]
        if type(available) is not bool:
            raise ValueError("anchor availability must be bool")
        normalized = None
        if available:
            if not isinstance(pose, Mapping) or set(pose) != {"x", "y", "yaw"}:
                raise ValueError("available anchor requires complete pose")
            normalized = {}
            for axis in ("x", "y", "yaw"):
                value = pose[axis]
                if type(value) not in (int, float):
                    raise ValueError("anchor pose must be finite binary64")
                try:
                    number = float(value)
                except OverflowError as exc:
                    raise ValueError("anchor pose exceeds binary64") from exc
                if not math.isfinite(number):
                    raise ValueError("anchor pose must be finite binary64")
                normalized[axis] = 0.0 if number == 0 else number
        elif pose is not None:
            raise ValueError("unavailable anchor requires null pose")
        records.append(dict(anchor_id=identity, available=available, pose=normalized))
    return sorted(records, key=lambda row: str(row["anchor_id"]).encode("utf-8"))


def anchor_digest(anchors: Iterable[Mapping[str, object]]) -> str:
    """SHA256 of the sole canonical C3 JSON encoding, shared by Python boundaries."""
    payload = json.dumps(
        canonical_anchor_records(anchors),
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
        ensure_ascii=True,
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


@dataclass(frozen=True)
class OccupancyThresholds:
    cell_size_m: float | None = None
    confirm_hits: int | None = None
    confirm_span_s: float | None = None
    clear_free: int | None = None
    clear_span_s: float | None = None
    distance_tolerance_m: float | None = None
    hit_weight: float | None = None
    free_weight: float | None = None
    occupied_belief: float | None = None
    free_belief: float | None = None
    belief_min: float | None = None
    belief_max: float | None = None
    max_age_s: float | None = None
    min_quality: float | None = None


@dataclass(frozen=True)
class SceneHit:
    x: float
    y: float
    kind: HitKind


@dataclass(frozen=True)
class MappingObservation:
    map_ref: MapRef
    anchor_id: str
    pose: Pose2D
    stamp: float
    rays: tuple[RangeRay, ...]
    hits: tuple[SceneHit, ...]
    dynamic_mask: tuple[bool, ...]
    occlusion_mask: tuple[bool, ...]
    quality: float
    localization_trusted: bool
    stale: bool
    age_s: float | None


@dataclass(frozen=True)
class CellEvidence:
    cell: tuple[int, int]
    anchor_id: str
    hit_count: int
    free_count: int
    last_valid_time: float
    belief: float
    change_source: ChangeSource


@dataclass(frozen=True)
class MapDelta:
    added: tuple[CellEvidence, ...]
    removed: tuple[CellEvidence, ...]


@dataclass(frozen=True)
class OccupancySnapshot:
    map_ref: MapRef | None
    cells: tuple[CellEvidence, ...]


# C3 records are facts, not scan acceptance or physical classification authority.
# ScanKey remains the C1 leaf identity; no references/workspace imports here.
U64_MAX = (1 << 64) - 1
FIXED_MEMBERS = 65_536
FIXED_CHANGES = 8_192
FIXED_TILES = 1_024
FIXED_PIXELS = 1_048_576


def fixed_uint(value: int, bits: int = 64) -> None:
    if type(value) is not int or not 0 <= value < 1 << bits:
        raise ValueError("expected unsigned integer")


def fixed_text(value: str, limit: int = 128) -> None:
    if (
        not isinstance(value, str)
        or value != value.strip()
        or not 1 <= len(value.encode("utf-8")) <= limit
    ):
        raise ValueError("invalid fixed identity")


def fixed_digest(value: str) -> None:
    if (
        not isinstance(value, str)
        or len(value) != 64
        or any(c not in "0123456789abcdef" for c in value)
    ):
        raise ValueError("expected SHA256 lowercase hex")


def fixed_real(value: float, *, positive: bool = False) -> None:
    try:
        valid = type(value) in (int, float) and math.isfinite(value) and (not positive or value > 0)
    except OverflowError:
        valid = False
    if not valid:
        raise ValueError("expected finite binary64")


def fixed_pose(pose: Pose2D) -> None:
    if not isinstance(pose, Pose2D):
        raise ValueError("expected Pose2D")
    for v in (pose.x, pose.y, pose.yaw):
        fixed_real(v)


def fixed_cells(cells: tuple[tuple[int, int], ...], limit: int = 2048) -> None:
    if not isinstance(cells, tuple) or len(cells) > limit or len(set(cells)) != len(cells):
        raise ValueError("invalid or duplicate local cells")
    for p in cells:
        if (
            not isinstance(p, tuple)
            or len(p) != 2
            or any(type(v) is not int or not -(1 << 31) <= v < 1 << 31 for v in p)
        ):
            raise ValueError("expected int32 cell pair")


def fixed_points(points: tuple[tuple[float, float], ...], limit: int) -> None:
    if not isinstance(points, tuple) or len(points) > limit:
        raise ValueError("too many local points")
    for p in points:
        if not isinstance(p, tuple) or len(p) != 2:
            raise ValueError("expected binary64 point pair")
        for v in p:
            fixed_real(v)


@dataclass(frozen=True)
class SourceIdentity:
    backend_boot: str
    bridge_boot_id: str
    source_epoch: str
    capture_generation: int
    clock_generation: int
    raw_gid: str
    bridge_gid: str

    def __post_init__(self) -> None:
        ScanKey(**self.__dict__, sequence=0)
        for name in ("backend_boot", "bridge_boot_id", "source_epoch", "raw_gid", "bridge_gid"):
            fixed_text(getattr(self, name), 36 if name == "backend_boot" else 128)

    @classmethod
    def from_key(cls, key: ScanKey) -> "SourceIdentity":
        return cls(*key.source)


@dataclass(frozen=True)
class FixedWatermark:
    map_ref: MapRef
    reference_generation: str
    backend_boot: str
    host_boot_id: str
    source: SourceIdentity | None
    graph_sequence: int
    anchor_digest: str
    fixed_revision: int
    evidence_key: ScanKey | None
    evidence_revision: int
    change_sequence: int

    def __post_init__(self) -> None:
        if not isinstance(self.map_ref, MapRef):
            raise ValueError("expected MapRef")
        fixed_text(self.map_ref.map_id)
        fixed_uint(self.map_ref.reference_revision)
        fixed_text(self.reference_generation)
        fixed_text(self.backend_boot, 36)
        fixed_text(self.host_boot_id, 36)
        fixed_digest(self.anchor_digest)
        for v in (
            self.graph_sequence,
            self.fixed_revision,
            self.evidence_revision,
            self.change_sequence,
        ):
            fixed_uint(v)
        if self.source is not None and (
            not isinstance(self.source, SourceIdentity)
            or self.source.backend_boot != self.backend_boot
        ):
            raise ValueError("watermark source mismatch")
        if self.evidence_key is not None and (
            not isinstance(self.evidence_key, ScanKey)
            or SourceIdentity.from_key(self.evidence_key) != self.source
        ):
            raise ValueError("watermark key mismatch")


@dataclass(frozen=True)
class VerifiedBasis:
    anchor_id: str
    map_basis: Pose2D
    odom_basis: Pose2D
    graph_sequence: int
    anchor_digest: str
    map_ref: MapRef
    reference_generation: str

    def __post_init__(self) -> None:
        fixed_text(self.anchor_id)
        fixed_pose(self.map_basis)
        fixed_pose(self.odom_basis)
        fixed_uint(self.graph_sequence)
        fixed_digest(self.anchor_digest)
        if not isinstance(self.map_ref, MapRef):
            raise ValueError("expected MapRef")
        fixed_text(self.map_ref.map_id)
        fixed_uint(self.map_ref.reference_revision)
        fixed_text(self.reference_generation)


@dataclass(frozen=True)
class ProfileProvenance:
    profile_id: str
    evidence_digest: str
    parameters: dict[str, float | int]

    def __post_init__(self) -> None:
        fixed_text(self.profile_id)
        fixed_digest(self.evidence_digest)
        if not isinstance(self.parameters, Mapping) or len(self.parameters) > 128:
            raise ValueError("invalid profile parameters")
        for k, v in self.parameters.items():
            fixed_text(k)
            fixed_uint(v) if type(v) is int else fixed_real(v)
        # Frozen dataclass must not retain a caller-owned mutable parameter dictionary.
        from types import MappingProxyType

        object.__setattr__(self, "parameters", MappingProxyType(dict(self.parameters)))


@dataclass(frozen=True)
class StableSupport:
    support_id: str
    anchor_id: str
    resolution_m: float
    local_surface: tuple[tuple[float, float], ...]
    local_endpoints: tuple[tuple[float, float], ...]
    local_cells: tuple[tuple[int, int], ...]
    first_key: ScanKey | None
    last_key: ScanKey | None
    source: SourceIdentity | None
    provenance: Literal["geometric", "legacy", "legacy-derived"]
    profile_id: str | None
    evidence_digest: str | None
    created_evidence_revision: int
    updated_evidence_revision: int

    def __post_init__(self) -> None:
        fixed_text(self.support_id)
        fixed_text(self.anchor_id)
        fixed_real(self.resolution_m, positive=True)
        fixed_points(self.local_surface, 32)
        fixed_points(self.local_endpoints, 2048)
        fixed_cells(self.local_cells)
        for v in (self.created_evidence_revision, self.updated_evidence_revision):
            fixed_uint(v)
        if self.created_evidence_revision > self.updated_evidence_revision:
            raise ValueError("support revision moved backwards")
        if self.provenance not in ("geometric", "legacy", "legacy-derived"):
            raise ValueError("invalid provenance")
        if self.profile_id is not None:
            fixed_text(self.profile_id)
        if self.evidence_digest is not None:
            fixed_digest(self.evidence_digest)
        if self.source is not None and not isinstance(self.source, SourceIdentity):
            raise ValueError("invalid support source")
        for key in (self.first_key, self.last_key):
            if key is not None and (
                not isinstance(key, ScanKey) or SourceIdentity.from_key(key) != self.source
            ):
                raise ValueError("support key/source mismatch")
        if (
            self.first_key is not None
            and self.last_key is not None
            and self.first_key.sequence > self.last_key.sequence
        ):
            raise ValueError("support key interval reversed")
        if self.provenance == "geometric" and any(
            v is None
            for v in (
                self.first_key,
                self.last_key,
                self.source,
                self.profile_id,
                self.evidence_digest,
            )
        ):
            raise ValueError("geometric support requires complete provenance")
        endpoint_cells = {
            (math.floor(x / self.resolution_m), math.floor(y / self.resolution_m))
            for x, y in self.local_endpoints
        }
        if (
            not self.local_cells
            or (self.provenance == "geometric" and not self.local_endpoints)
            or (self.local_endpoints and endpoint_cells != set(self.local_cells))
            or not set(self.local_surface) <= set(self.local_endpoints)
        ):
            raise ValueError("support endpoints/cells/summary disagree")

    @property
    def member_count(self) -> int:
        return 1 + len(self.local_surface) + len(self.local_endpoints) + len(self.local_cells)


@dataclass(frozen=True)
class FreeTile:
    anchor_id: str
    resolution_m: float
    tile_x: int
    tile_y: int
    active_mask: bytes
    tile_revision: int
    last_observed_key: ScanKey
    source: SourceIdentity
    evidence_revision: int

    def __post_init__(self) -> None:
        fixed_text(self.anchor_id)
        fixed_real(self.resolution_m, positive=True)
        fixed_cells(((self.tile_x, self.tile_y),))
        if type(self.active_mask) is not bytes or len(self.active_mask) != 128:
            raise ValueError("free mask must be exactly 128 bytes")
        fixed_uint(self.tile_revision)
        fixed_uint(self.evidence_revision)
        if (
            self.tile_revision == 0
            or not isinstance(self.last_observed_key, ScanKey)
            or not isinstance(self.source, SourceIdentity)
            or SourceIdentity.from_key(self.last_observed_key) != self.source
        ):
            raise ValueError("invalid free provenance/revision")

    @property
    def tile_key(self) -> tuple:
        return self.anchor_id, self.resolution_m, self.tile_x, self.tile_y

    def cells(self) -> tuple[tuple[int, int], ...]:
        return tuple(
            (32 * self.tile_x + i % 32, 32 * self.tile_y + i // 32)
            for i in range(1024)
            if self.active_mask[i // 8] & (1 << (i % 8))
        )


@dataclass(frozen=True)
class SupportRevocation:
    support_id: str
    expected_support_revision: int
    reason: Literal["motion_evidenced", "observed_free_space"]
    trigger_key: ScanKey
    trigger_anchor_id: str

    def __post_init__(self) -> None:
        fixed_text(self.support_id)
        fixed_text(self.trigger_anchor_id)
        fixed_uint(self.expected_support_revision)
        if not isinstance(self.trigger_key, ScanKey) or self.reason not in (
            "motion_evidenced",
            "observed_free_space",
        ):
            raise ValueError("invalid support revocation")


@dataclass(frozen=True)
class SupportCellRemoval:
    support_id: str
    expected_support_revision: int
    local_cells: tuple[tuple[int, int], ...]
    reason: Literal["observed_free_space"]
    trigger_key: ScanKey
    trigger_anchor_id: str

    def __post_init__(self) -> None:
        SupportRevocation(
            self.support_id,
            self.expected_support_revision,
            self.reason,
            self.trigger_key,
            self.trigger_anchor_id,
        )
        fixed_cells(self.local_cells)
        if not self.local_cells or self.reason != "observed_free_space":
            raise ValueError("partial removal requires actual free cells")


@dataclass(frozen=True)
class LegacyMaskEdit:
    base_id: str
    cell_indices: tuple[int, ...]
    expected_values: tuple[int, ...]
    reason: str
    trigger_key: ScanKey
    trigger_anchor_id: str

    def __post_init__(self) -> None:
        fixed_digest(self.base_id)
        fixed_text(self.trigger_anchor_id)
        if (
            not isinstance(self.cell_indices, tuple)
            or len(self.cell_indices) > FIXED_CHANGES
            or tuple(sorted(set(self.cell_indices))) != self.cell_indices
            or len(self.cell_indices) != len(self.expected_values)
            or self.reason
            not in ("motion_evidenced", "observed_free_space", "geometric_confirmation")
            or not isinstance(self.trigger_key, ScanKey)
        ):
            raise ValueError("invalid legacy edit")
        for index in self.cell_indices:
            fixed_uint(index, 32)
        for value in self.expected_values:
            fixed_uint(value, 8)


@dataclass(frozen=True)
class FixedDelta:
    expected: FixedWatermark
    key: ScanKey
    evidence_revision: int
    trigger_anchor_id: str
    upsert_supports: tuple[StableSupport, ...] = ()
    revocations: tuple[SupportRevocation, ...] = ()
    support_cell_removals: tuple[SupportCellRemoval, ...] = ()
    free_tiles: tuple[FreeTile, ...] = ()
    legacy_edits: tuple[LegacyMaskEdit, ...] = ()

    def __post_init__(self) -> None:
        fixed_uint(self.evidence_revision)
        fixed_text(self.trigger_anchor_id)
        if not isinstance(self.expected, FixedWatermark) or not isinstance(self.key, ScanKey):
            raise ValueError("invalid evidence identity")
        for name, kind in (
            ("upsert_supports", StableSupport),
            ("revocations", SupportRevocation),
            ("support_cell_removals", SupportCellRemoval),
            ("free_tiles", FreeTile),
            ("legacy_edits", LegacyMaskEdit),
        ):
            items = getattr(self, name)
            if (
                not isinstance(items, tuple)
                or len(items) > FIXED_CHANGES
                or any(not isinstance(x, kind) for x in items)
            ):
                raise ValueError("invalid evidence collection")
        for items, identity in (
            (self.upsert_supports, lambda x: x.support_id),
            (self.revocations + self.support_cell_removals, lambda x: x.support_id),
            (self.free_tiles, lambda x: x.tile_key),
        ):
            keys = [identity(x) for x in items]
            if len(keys) != len(set(keys)):
                raise ValueError("duplicate evidence member")
