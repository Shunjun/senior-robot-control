"""//Compatible: historical map values forward the canonical public API."""

from senior_mapping.evidence import (
    CellEvidence,
    MapDelta,
    MappingObservation,
    OccupancySnapshot,
    OccupancyThresholds,
    RangeRay,
    SceneHit,
)
from senior_mapping.occupancy import OccupancyMaintainer
from senior_mapping.references import (
    AnchoredWaypoint,
    AnchorPose,
    AnchorSnapshot,
    ReferenceChange,
    SlamSnapshot,
)
from senior_mapping.store import MapReferenceStore

__all__ = [
    "AnchorPose",
    "AnchorSnapshot",
    "AnchoredWaypoint",
    "CellEvidence",
    "MapDelta",
    "MapReferenceStore",
    "MappingObservation",
    "OccupancyMaintainer",
    "OccupancySnapshot",
    "OccupancyThresholds",
    "RangeRay",
    "ReferenceChange",
    "SceneHit",
    "SlamSnapshot",
]
