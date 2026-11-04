"""Observation and evidence records for the long-term occupancy layer."""

from dataclasses import dataclass
from typing import Literal

from senior_control.models import MapRef, Pose2D

HitKind = Literal["static", "unknown_static", "person", "moving", "unknown_dynamic"]
ChangeSource = Literal["geometric_confirmation", "observed_free_space"]


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
class RangeRay:
    angle: float
    range_m: float
    valid: bool
    no_return_free: bool
    clear_limit_m: float | None


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
