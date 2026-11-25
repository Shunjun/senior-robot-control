"""Observation and evidence records for the long-term occupancy layer."""

import hashlib
import json
import math
from collections.abc import Iterable, Mapping
from dataclasses import dataclass
from typing import Literal

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
        canonical_anchor_records(anchors), sort_keys=True, separators=(",", ":"),
        allow_nan=False, ensure_ascii=True,
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
