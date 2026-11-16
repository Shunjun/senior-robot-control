"""Matcher-only body exclusion from the run's canonical learning record.

This never clears raw obstacle evidence, and never treats a state file as motion
permission. Only the validated physical polygon is needed by matcher features.
"""

from functools import lru_cache
from pathlib import Path

from shapely.geometry import Point, Polygon

from senior_control.runtime.learning_store import read_record
from senior_control.safety.self_filter import FilteredObservation


@lru_cache(maxsize=4)
def _boundary(vertices: tuple[tuple[float, float], ...]) -> Polygon:
    body = Polygon(vertices)
    if not body.is_valid or body.area <= 0:
        raise ValueError("invalid physical boundary")
    return body


def external_feature_ranges(
    observation: FilteredObservation, record_path: Path
) -> tuple[float, ...]:
    try:
        record = read_record(record_path)
        if record.geometry is None:
            raise ValueError("awaiting_start_physical_boundary")
        body = _boundary(record.geometry.physical_polygon)
    except (OSError, ValueError) as exc:
        raise ValueError("awaiting_start_physical_boundary") from exc
    ranges = list(observation.scan.ranges)
    left, bottom, right, top = body.bounds
    for ray in observation.rays:
        if ray.endpoint is None:
            continue
        x, y = ray.endpoint
        if left <= x <= right and bottom <= y <= top and body.covers(Point(x, y)):
            ranges[ray.index] = float("nan")
    return tuple(ranges)
