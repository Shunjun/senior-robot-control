"""Turns detections plus depth geometry into a per-ray dynamic mask.

A detection box is wider than the object inside it. Columns are marked only
when their measured range matches the object's own range, so a wall seen
through the box stays static geometry. The mask never edits the geometry: a
person still blocks the real-time safety layer, and what the person hides is
reported as occluded rather than free.
"""

import math
import statistics
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
    person_labels: frozenset[str] = frozenset({"person"})
    moving_labels: frozenset[str] = _MOVING_LABELS

    def __post_init__(self) -> None:
        values = (self.min_confidence, self.max_detection_age_s, self.range_tolerance_m)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("tracking thresholds must be finite")
        if self.max_detection_age_s < 0.0 or self.range_tolerance_m <= 0.0:
            raise ValueError("invalid tracking thresholds")


@dataclass(frozen=True)
class DynamicMask:
    """Index-aligned with ``GeometryObservation.rays``.

    ``semantics_available`` False means no fresh detector output: kinds are
    ``unknown_static`` and consumers must not treat the geometry as confirmed
    static. ``occluded`` is True where geometry cannot vouch for what lies
    behind the ray.
    """

    semantics_available: bool
    dynamic: tuple[bool, ...]
    occluded: tuple[bool, ...]
    kinds: tuple[HitKind, ...]


def track(
    detections: Sequence[Detection],
    geometry: GeometryObservation,
    *,
    config: TrackingConfig | None = None,
    semantics_stamp: float | None = None,
) -> DynamicMask:
    """``semantics_stamp`` is the inference time; it defaults to the newest detection."""
    config = config or TrackingConfig()
    count = len(geometry.rays)
    stamp = semantics_stamp
    if stamp is None and detections:
        stamp = max(item.stamp for item in detections)
    fresh = (
        stamp is not None
        and math.isfinite(stamp)
        and math.isfinite(geometry.stamp)
        and abs(geometry.stamp - stamp) <= config.max_detection_age_s
    )
    if not fresh:
        unknown: list[HitKind] = ["unknown_static"] * count
        return DynamicMask(False, (False,) * count, geometry.occlusion_mask, tuple(unknown))

    kinds: list[HitKind] = ["static"] * count
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
    x0, _y0, x1, _y1 = detection.box
    if not all(math.isfinite(value) for value in detection.box) or x1 < x0:
        return []
    inside = [
        index
        for index, column in enumerate(geometry.columns)
        if x0 <= column <= x1 and geometry.status[index] == "hit"
    ]
    if not inside:
        return []
    # The object sits in the middle of its box; the box edges often show background.
    quarter = (x1 - x0) / 4.0
    center = [i for i in inside if x0 + quarter <= geometry.columns[i] <= x1 - quarter] or inside
    reference = statistics.median(geometry.rays[i].range_m for i in center)
    return [
        i for i in inside if abs(geometry.rays[i].range_m - reference) <= config.range_tolerance_m
    ]
