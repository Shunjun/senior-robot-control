"""Conservative semantic candidates; geometry remains an independent obstacle.

Column masks here are diagnostic candidates, not a qualified laser mask. A
consumer must associate the 3-D source samples and both measurement identities
before excluding any scan hit. Missing or empty detections never prove static.
"""

import math
from collections.abc import Sequence
from dataclasses import dataclass

from senior_control.mapping.evidence import HitKind
from senior_control.vision.depth import GeometryObservation
from senior_control.vision.detector import Detection

_MOVING_LABELS = frozenset({"bicycle", "car", "motorcycle", "bus", "truck", "dog", "cat"})


@dataclass(frozen=True)
class TrackingConfig:
    min_confidence: float = 0.5
    max_detection_age_s: float = 0.3
    range_tolerance_m: float = 0.5
    max_object_speed_mps: float = 2.0
    motion_threshold_m: float = 0.2
    motion_min_baseline_s: float = 0.15
    motion_max_baseline_s: float = 0.6
    motion_max_changed_fraction: float = 0.5
    person_labels: frozenset[str] = frozenset({"person"})
    moving_labels: frozenset[str] = _MOVING_LABELS

    def __post_init__(self) -> None:
        values = (
            self.min_confidence,
            self.max_detection_age_s,
            self.range_tolerance_m,
            self.max_object_speed_mps,
            self.motion_threshold_m,
            self.motion_min_baseline_s,
            self.motion_max_baseline_s,
            self.motion_max_changed_fraction,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("tracking thresholds must be finite")
        if self.max_detection_age_s < 0.0 or self.range_tolerance_m <= 0.0:
            raise ValueError("invalid tracking thresholds")
        if self.max_object_speed_mps <= 0.0 or self.motion_threshold_m <= 0.0:
            raise ValueError("speed and motion threshold must be positive")
        if not 0.0 < self.motion_min_baseline_s < self.motion_max_baseline_s:
            raise ValueError("motion baselines must satisfy 0 < min < max")
        if not 0.0 < self.motion_max_changed_fraction <= 1.0:
            raise ValueError("motion_max_changed_fraction must be within (0, 1]")


@dataclass(frozen=True)
class DynamicMask:
    """Index-aligned with ``GeometryObservation.rays``.

    ``semantics_available`` False means no fresh detector output: kinds are
    ``unknown_static`` and consumers must not treat the geometry as confirmed
    static. ``occluded`` is True where geometry cannot vouch for what lies
    behind the ray. ``motion_available`` False means the geometric motion cue
    could not run (no baseline, columns changed, or the whole scene changed as
    if the robot moved): the absence of a dynamic flag then proves nothing.
    """

    semantics_available: bool
    dynamic: tuple[bool, ...]
    occluded: tuple[bool, ...]
    kinds: tuple[HitKind, ...]
    motion_available: bool = False


def track(
    detections: Sequence[Detection],
    geometry: GeometryObservation,
    *,
    config: TrackingConfig | None = None,
    semantics_stamp: float | None = None,
) -> DynamicMask:
    """Associate only the same captured frame; arrival time is never a capture time."""
    config = config or TrackingConfig()
    count = len(geometry.rays)
    stamp = semantics_stamp
    if stamp is None and detections:
        stamp = max(item.stamp for item in detections)
    fresh = (
        stamp is not None
        and math.isfinite(stamp)
        and math.isfinite(geometry.stamp)
        and geometry.stamp == stamp
        and all(item.stamp == stamp for item in detections)
    )
    if not fresh:
        unknown: list[HitKind] = ["unknown_static"] * count
        return DynamicMask(False, (False,) * count, geometry.occlusion_mask, tuple(unknown))

    kinds: list[HitKind] = ["unknown_static"] * count
    dynamic = [False] * count
    # People first so a person box wins over an overlapping moving-object box.
    ordered = sorted(detections, key=lambda item: item.label not in config.person_labels)
    for detection in ordered:
        kind = _kind(detection, config)
        if kind is None:
            continue
        for index in _matching_columns(detection, geometry, config):
            if dynamic[index] and kinds[index] == "person":
                continue
            dynamic[index] = True
            kinds[index] = kind
    occluded = tuple(
        hidden or moving for hidden, moving in zip(geometry.occlusion_mask, dynamic, strict=True)
    )
    return DynamicMask(True, tuple(dynamic), occluded, tuple(kinds))


def _kind(detection: Detection, config: TrackingConfig) -> HitKind | None:
    if not math.isfinite(detection.confidence) or detection.confidence < config.min_confidence:
        return None
    if detection.label in config.person_labels:
        return "person"
    if detection.label in config.moving_labels:
        return "moving"
    return None


def _matching_columns(
    detection: Detection, geometry: GeometryObservation, config: TrackingConfig
) -> list[int]:
    x0, y0, x1, y1 = detection.box
    if not all(math.isfinite(value) for value in detection.box) or x1 < x0 or y1 < y0:
        return []
    selected = {point.ray_index: (point.x, point.y, point.z) for point in geometry.points}
    indices = {
        sample.ray_index
        for sample in geometry.samples
        if x0 <= sample.u <= x1
        and y0 <= sample.v <= y1
        and selected.get(sample.ray_index) == (sample.x, sample.y, sample.z)
    }
    if not indices:
        return []
    reference = min(geometry.rays[index].range_m for index in indices)
    return sorted(
        index
        for index in indices
        if geometry.rays[index].range_m - reference <= config.range_tolerance_m
    )


class Tracker:
    """Semantic candidates; motion is unavailable without independent pose history."""

    def __init__(self, config: TrackingConfig | None = None) -> None:
        self._config = config or TrackingConfig()

    def update(
        self,
        detections: Sequence[Detection],
        geometry: GeometryObservation,
        *,
        semantics_stamp: float | None = None,
    ) -> DynamicMask:
        return track(detections, geometry, config=self._config, semantics_stamp=semantics_stamp)
