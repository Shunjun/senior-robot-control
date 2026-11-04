"""Confirm and clear a long-term occupancy layer from repeated observations."""

import math
from dataclasses import dataclass, replace

from senior_control.mapping.evidence import (
    CellEvidence,
    ChangeSource,
    HitKind,
    MapDelta,
    MappingObservation,
    OccupancySnapshot,
    OccupancyThresholds,
    RangeRay,
)
from senior_control.models import MapRef, Pose2D

_WRITABLE_HITS: frozenset[HitKind] = frozenset({"static", "unknown_static"})


@dataclass(frozen=True)
class _Limits:
    cell_size_m: float
    confirm_hits: int
    confirm_span_s: float
    clear_free: int
    clear_span_s: float
    distance_tolerance_m: float
    hit_weight: float
    free_weight: float
    occupied_belief: float
    free_belief: float
    belief_min: float
    belief_max: float
    max_age_s: float
    min_quality: float


@dataclass
class _Track:
    ix: int
    iy: int
    anchor_id: str
    hit_count: int
    free_count: int
    first_hit_time: float
    last_valid_time: float
    first_free_time: float | None
    belief: float
    ref_x: float
    ref_y: float
    occupied: bool
    source: ChangeSource | None


def _real(value: object) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    number = float(value)
    if not math.isfinite(number):
        return None
    return number


def _whole(value: object) -> int | None:
    if isinstance(value, bool) or not isinstance(value, int):
        return None
    return value


def _parse_limits(raw: OccupancyThresholds | None) -> _Limits | None:
    if raw is None:
        return None
    cell_size_m = _real(raw.cell_size_m)
    confirm_hits = _whole(raw.confirm_hits)
    confirm_span_s = _real(raw.confirm_span_s)
    clear_free = _whole(raw.clear_free)
    clear_span_s = _real(raw.clear_span_s)
    distance_tolerance_m = _real(raw.distance_tolerance_m)
    hit_weight = _real(raw.hit_weight)
    free_weight = _real(raw.free_weight)
    occupied_belief = _real(raw.occupied_belief)
    free_belief = _real(raw.free_belief)
    belief_min = _real(raw.belief_min)
    belief_max = _real(raw.belief_max)
    max_age_s = _real(raw.max_age_s)
    min_quality = _real(raw.min_quality)
    if (
        cell_size_m is None
        or confirm_hits is None
        or confirm_span_s is None
        or clear_free is None
        or clear_span_s is None
        or distance_tolerance_m is None
        or hit_weight is None
        or free_weight is None
        or occupied_belief is None
        or free_belief is None
        or belief_min is None
        or belief_max is None
        or max_age_s is None
        or min_quality is None
    ):
        return None
    gap_holds = occupied_belief - free_weight > free_belief
    bounds_hold = belief_min <= free_belief < occupied_belief <= belief_max
    prior_holds = belief_min <= 0.0 <= belief_max and occupied_belief > hit_weight
    counts_hold = confirm_hits >= 2 and clear_free >= 2
    steps_hold = (
        cell_size_m > 0.0
        and confirm_span_s > 0.0
        and clear_span_s > 0.0
        and distance_tolerance_m >= 0.0
        and hit_weight > 0.0
        and free_weight > 0.0
        and max_age_s > 0.0
    )
    if not (gap_holds and bounds_hold and prior_holds and counts_hold and steps_hold):
        return None
    return _Limits(
        cell_size_m,
        confirm_hits,
        confirm_span_s,
        clear_free,
        clear_span_s,
        distance_tolerance_m,
        hit_weight,
        free_weight,
        occupied_belief,
        free_belief,
        belief_min,
        belief_max,
        max_age_s,
        min_quality,
    )


def _observation_usable(
    observation: MappingObservation,
    limits: _Limits,
    map_ref: MapRef | None,
) -> bool:
    age = observation.age_s
    masks_match = len(observation.dynamic_mask) == len(observation.rays)
    masks_match = masks_match and len(observation.occlusion_mask) == len(observation.rays)
    age_ok = age is not None and math.isfinite(age) and 0.0 <= age <= limits.max_age_s
    quality_ok = math.isfinite(observation.quality) and observation.quality >= limits.min_quality
    return bool(
        observation.anchor_id.strip()
        and not observation.stale
        and observation.localization_trusted
        and math.isfinite(observation.stamp)
        and quality_ok
        and age_ok
        and masks_match
        and (map_ref is None or observation.map_ref == map_ref)
    )


def _segment_cells(
    x0: float,
    y0: float,
    x1: float,
    y1: float,
    size: float,
) -> list[tuple[int, int]]:
    if not all(math.isfinite(value) for value in (x0, y0, x1, y1, size)) or size <= 0.0:
        return []
    ix = math.floor(x0 / size)
    iy = math.floor(y0 / size)
    end_ix = math.floor(x1 / size)
    end_iy = math.floor(y1 / size)
    cells = [(ix, iy)]
    dx = x1 - x0
    dy = y1 - y0
    step_x = 0 if dx == 0.0 else (1 if dx > 0.0 else -1)
    step_y = 0 if dy == 0.0 else (1 if dy > 0.0 else -1)
    if step_x == 0 and step_y == 0:
        return cells
    t_max_x = math.inf if step_x == 0 else ((ix + (step_x > 0)) * size - x0) / dx
    t_max_y = math.inf if step_y == 0 else ((iy + (step_y > 0)) * size - y0) / dy
    t_delta_x = math.inf if dx == 0.0 else abs(size / dx)
    t_delta_y = math.inf if dy == 0.0 else abs(size / dy)
    guard = abs(end_ix - ix) + abs(end_iy - iy) + 2
    steps = 0
    while (ix, iy) != (end_ix, end_iy) and steps < guard:
        if t_max_x < t_max_y:
            ix += step_x
            t_max_x += t_delta_x
        elif t_max_y < t_max_x:
            iy += step_y
            t_max_y += t_delta_y
        else:
            ix += step_x
            iy += step_y
            t_max_x += t_delta_x
            t_max_y += t_delta_y
        cells.append((ix, iy))
        steps += 1
    return cells


class OccupancyMaintainer:
    """Maintain stable occupied cells and clear them only with observed free space.

    Cell coordinates are ``floor(position / cell_size)``. Each observation
    changes one cell by at most one hit or one free count. ``snapshot``
    returns the long-term occupied layer, including the anchor, counts, last
    valid time, belief, and change source needed for a later reprojection.
    """

    def __init__(self, thresholds: OccupancyThresholds | None) -> None:
        self._limits = _parse_limits(thresholds)
        self._cells: dict[tuple[int, int], _Track] = {}
        self._map_ref: MapRef | None = None
        self._layer_open = False

    def update(self, observation: MappingObservation) -> MapDelta:
        limits = self._limits
        if limits is None or not _observation_usable(observation, limits, self._map_ref):
            return MapDelta((), ())
        if self._map_ref is None:
            self._map_ref = observation.map_ref
        added: list[CellEvidence] = []
        removed: list[CellEvidence] = []
        hit_cells = self._integrate_hits(observation, added)
        self._integrate_free(observation, hit_cells, removed)
        return MapDelta(tuple(added), tuple(removed))

    def snapshot(self) -> OccupancySnapshot:
        if not self._layer_open or self._map_ref is None:
            return OccupancySnapshot(None, ())
        occupied = tuple(self._record(track) for track in self._cells.values() if track.occupied)
        return OccupancySnapshot(self._map_ref, occupied)

    def _integrate_hits(
        self,
        observation: MappingObservation,
        added: list[CellEvidence],
    ) -> set[tuple[int, int]]:
        grouped: dict[tuple[int, int], list[tuple[float, float]]] = {}
        for hit in observation.hits:
            if hit.kind not in _WRITABLE_HITS:
                continue
            if not math.isfinite(hit.x) or not math.isfinite(hit.y):
                continue
            grouped.setdefault(self._index(hit.x, hit.y), []).append((hit.x, hit.y))
        touched: set[tuple[int, int]] = set()
        for key, points in grouped.items():
            track = self._cells.get(key)
            selected = self._consistent_point(track, points)
            if selected is None:
                continue
            if track is None:
                track = _Track(
                    key[0],
                    key[1],
                    observation.anchor_id,
                    0,
                    0,
                    observation.stamp,
                    observation.stamp,
                    None,
                    0.0,
                    selected[0],
                    selected[1],
                    False,
                    None,
                )
                self._cells[key] = track
            self._apply_hit(track, observation, added)
            touched.add(key)
        return touched

    def _consistent_point(
        self,
        track: _Track | None,
        points: list[tuple[float, float]],
    ) -> tuple[float, float] | None:
        limits = self._limits
        assert limits is not None
        for point in points:
            if track is None:
                return point
            if math.dist(point, (track.ref_x, track.ref_y)) <= limits.distance_tolerance_m:
                return point
        return None

    def _apply_hit(
        self,
        track: _Track,
        observation: MappingObservation,
        added: list[CellEvidence],
    ) -> None:
        limits = self._limits
        assert limits is not None
        track.hit_count += 1
        track.belief = self._clamp(track.belief + limits.hit_weight)
        track.free_count = 0
        track.first_free_time = None
        track.last_valid_time = observation.stamp
        track.anchor_id = observation.anchor_id
        if track.occupied:
            return
        spanned = observation.stamp - track.first_hit_time >= limits.confirm_span_s
        confirmed = (
            track.hit_count >= limits.confirm_hits and track.belief >= limits.occupied_belief
        )
        if not spanned or not confirmed:
            return
        track.occupied = True
        track.source = "geometric_confirmation"
        self._layer_open = True
        added.append(self._record(track))

    def _integrate_free(
        self,
        observation: MappingObservation,
        hit_cells: set[tuple[int, int]],
        removed: list[CellEvidence],
    ) -> None:
        cleared: set[tuple[int, int]] = set()
        for index, ray in enumerate(observation.rays):
            if observation.dynamic_mask[index] or observation.occlusion_mask[index]:
                continue
            cleared.update(self._free_cells(observation.pose, ray))
        cleared.difference_update(hit_cells)
        for key in cleared:
            track = self._cells.get(key)
            if track is not None:
                self._apply_free(key, track, observation, removed)

    def _free_cells(self, pose: Pose2D, ray: RangeRay) -> tuple[tuple[int, int], ...]:
        limits = self._limits
        assert limits is not None
        if not math.isfinite(ray.angle):
            return ()
        if ray.no_return_free:
            limit = ray.clear_limit_m
            if limit is None or not math.isfinite(limit) or limit <= 0.0:
                return ()
            distance = limit
            drop_endpoint = False
        elif not ray.valid or not math.isfinite(ray.range_m) or ray.range_m < 0.0:
            return ()
        else:
            distance = ray.range_m
            drop_endpoint = True
        heading = pose.yaw + ray.angle
        end = (pose.x + distance * math.cos(heading), pose.y + distance * math.sin(heading))
        cells = _segment_cells(pose.x, pose.y, end[0], end[1], limits.cell_size_m)
        if drop_endpoint and cells:
            cells = cells[:-1]
        return tuple(cells)

    def _apply_free(
        self,
        key: tuple[int, int],
        track: _Track,
        observation: MappingObservation,
        removed: list[CellEvidence],
    ) -> None:
        limits = self._limits
        assert limits is not None
        if track.first_free_time is None:
            track.first_free_time = observation.stamp
        track.free_count += 1
        track.belief = self._clamp(track.belief - limits.free_weight)
        track.last_valid_time = observation.stamp
        started = track.first_free_time
        ready = (
            track.occupied
            and started is not None
            and track.free_count >= limits.clear_free
            and observation.stamp - started >= limits.clear_span_s
            and track.belief <= limits.free_belief
        )
        if not ready:
            return
        track.source = "observed_free_space"
        removed.append(replace(self._record(track), anchor_id=observation.anchor_id))
        del self._cells[key]

    def _clamp(self, belief: float) -> float:
        limits = self._limits
        assert limits is not None
        return min(limits.belief_max, max(limits.belief_min, belief))

    def _index(self, x: float, y: float) -> tuple[int, int]:
        limits = self._limits
        assert limits is not None
        size = limits.cell_size_m
        return (math.floor(x / size), math.floor(y / size))

    def _record(self, track: _Track) -> CellEvidence:
        source = track.source
        assert source is not None
        return CellEvidence(
            (track.ix, track.iy),
            track.anchor_id,
            track.hit_count,
            track.free_count,
            track.last_valid_time,
            track.belief,
            source,
        )
