"""Matcher features from an already loaded frozen view; no file or IPC access."""

from functools import lru_cache

from shapely.geometry import Point, Polygon

from senior_control.safety.learning_view import LearningFilterView
from senior_control.safety.self_filter import FilteredObservation


@lru_cache(maxsize=4)
def _boundary(vertices: tuple[tuple[float, float], ...]) -> Polygon:
    body = Polygon(vertices)
    if not body.is_valid or body.area <= 0:
        raise ValueError("invalid physical boundary")
    return body


def external_feature_ranges(
    observation: FilteredObservation, view: LearningFilterView | None = None
) -> tuple[float, ...]:
    if view is None:
        return observation.scan.ranges
    filtered = view.filter(observation)
    body = _boundary(view.geometry.physical_polygon)
    ranges = list(observation.scan.ranges)
    left, bottom, right, top = body.bounds
    for ray in filtered.rays:
        if ray.kind == "self":
            ranges[ray.index] = float("nan")
        elif ray.endpoint is not None:
            x, y = ray.endpoint
            if left <= x <= right and bottom <= y <= top and body.covers(Point(x, y)):
                ranges[ray.index] = float("nan")
    return tuple(ranges)
