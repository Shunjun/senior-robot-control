"""Batched point predicates retain the original GEOS boundary decisions."""

import math

import pytest
from shapely.geometry import Point, Polygon

from senior_control.safety.guard import _intersects_hits


@pytest.mark.parametrize(
    "hits",
    [
        (),
        ((0.0, 0.0),),
        ((1.0, 0.0),),
        ((1.0, 1.0),),
        ((1.0000000000000002, 0.0),),
        ((2.0, 2.0), (0.0, 0.0)),
        ((math.inf, 0.0),),
        ((-math.inf, math.inf), (2.0, 2.0)),
        ((math.nan, math.nan),),
        ((math.nan, 0.0), (1.0, 0.0)),
        ((2.0, 2.0), (math.inf, math.nan), (0.0, 0.0)),
    ],
    ids=[
        "empty",
        "inside",
        "edge",
        "corner",
        "outside-adjacent",
        "mixed-finite",
        "infinite",
        "infinite-outside",
        "nan-empty",
        "nan-boundary",
        "mixed-nonfinite",
    ],
)
def test_batched_hits_match_original_point_intersections(hits):
    polygon = Polygon(((-1, -1), (1, -1), (1, 1), (-1, 1)))
    original_points = [Point(hit) for hit in hits]
    expected = any(polygon.intersects(hit) for hit in original_points)
    assert _intersects_hits(polygon, hits) is expected
