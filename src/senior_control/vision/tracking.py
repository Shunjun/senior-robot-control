"""Turns detections plus depth geometry into a per-ray dynamic mask.

A detection box is wider than the object inside it. Columns are marked only
when their measured range matches the object's own range, so a wall seen
through the box stays static geometry. The mask never edits the geometry: a
person still blocks the real-time safety layer, and what the person hides is
reported as occluded rather than free.

Detections arrive later than the depth they are applied to. A box older than the
geometry is widened by the elapsed time times ``max_object_speed_mps``, in angle
and in range, so a target that walked out of the old box is not confirmed static.
The object's range comes from the nearest return in that widened band, not from the
old box center, which may show the background after a sideways shift.
``Tracker`` adds a geometric motion cue that works with the detector disabled: a
column whose range changed between observations is dynamic. A column with no
observed motion stays ``unknown_static``; motion that is missed stays missed.
"""

import math
from collections import deque
from collections.abc import Sequence
from dataclasses import dataclass, replace

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
    in_box = [i for i, column in enumerate(geometry.columns) if x0 <= column <= x1]
    if not in_box:
        return []
    # An older box can only be stale, never ahead of the geometry. The object may have
    # moved up to speed * elapsed sideways or along the ray since the detection.
    reach = config.max_object_speed_mps * max(0.0, geometry.stamp - detection.stamp)
    angles = [geometry.rays[i].angle for i in in_box]
    low, high = min(angles), max(angles)
    # A column joins the band when its own sideways gap to the old box, in meters at the
    # column's own range, is within reach. The band does not depend on any reference range.
    band = [
        i
        for i, ray in enumerate(geometry.rays)
        if geometry.status[i] == "hit"
        and ray.range_m * max(low - ray.angle, ray.angle - high, 0.0) <= reach
    ]
    if not band:
        return []
    # The object is the nearest return in the band. The old box center is not used: after
    # a sideways shift it often shows the background.
    reference = min(geometry.rays[i].range_m for i in band)
    tolerance = config.range_tolerance_m + reach
    return [i for i in band if geometry.rays[i].range_m - reference <= tolerance]


class Tracker:
    """Detections plus a short geometry history. One instance per camera stream.

    The history gives a detector-free motion cue. It cannot separate object motion
    from robot motion without odometry, so when most columns change at once it
    reports ``motion_available=False`` instead of guessing.
    """

    def __init__(self, config: TrackingConfig | None = None) -> None:
        self._config = config or TrackingConfig()
        self._history: deque[GeometryObservation] = deque()

    def update(
        self,
        detections: Sequence[Detection],
        geometry: GeometryObservation,
        *,
        semantics_stamp: float | None = None,
    ) -> DynamicMask:
        mask = track(detections, geometry, config=self._config, semantics_stamp=semantics_stamp)
        moved = self._moved_columns(geometry)
        self._remember(geometry)
        if moved is None:
            return mask
        dynamic = tuple(a or b for a, b in zip(mask.dynamic, moved, strict=True))
        kinds: list[HitKind] = list(mask.kinds)
        for index, flag in enumerate(moved):
            if flag and not mask.dynamic[index]:
                kinds[index] = "unknown_dynamic"
        occluded = tuple(a or b for a, b in zip(mask.occluded, dynamic, strict=True))
        return replace(
            mask, dynamic=dynamic, occluded=occluded, kinds=tuple(kinds), motion_available=True
        )

    def _moved_columns(self, geometry: GeometryObservation) -> tuple[bool, ...] | None:
        config = self._config
        baseline = None
        for earlier in reversed(self._history):
            age = geometry.stamp - earlier.stamp
            if age < config.motion_min_baseline_s:
                continue
            if age <= config.motion_max_baseline_s and earlier.columns == geometry.columns:
                baseline = earlier
            break
        if baseline is None or not math.isfinite(geometry.stamp):
            return None
        moved: list[bool] = []
        comparable = 0
        for now, before, ray, old in zip(
            geometry.status, baseline.status, geometry.rays, baseline.rays, strict=True
        ):
            both = now == "hit" and before == "hit"
            comparable += both
            moved.append(both and abs(ray.range_m - old.range_m) > config.motion_threshold_m)
        if comparable == 0:
            return None
        if sum(moved) / comparable > config.motion_max_changed_fraction:
            return None
        return tuple(moved)

    def _remember(self, geometry: GeometryObservation) -> None:
        if self._history and geometry.stamp <= self._history[-1].stamp:
            self._history.clear()  # time went backwards: the old baseline is meaningless
        self._history.append(geometry)
        horizon = geometry.stamp - self._config.motion_max_baseline_s
        while self._history and self._history[0].stamp < horizon:
            self._history.popleft()
