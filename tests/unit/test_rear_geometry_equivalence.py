"""Compare optimized mask geometry with the frozen C12 implementation."""

import json
import math
import random
from pathlib import Path

import pytest
from rear_legacy_oracle import legacy_exact_partial_fans, legacy_filter_self
from shapely.errors import GEOSException
from shapely.geometry import Polygon

from senior_control.config import DeviceProfile
from senior_control.models import Pose2D
from senior_control.safety import coverage, self_filter
from senior_control.safety.self_filter import ScanObservation


def _profile(masks):
    return DeviceProfile(
        mode="simulation",
        footprint=[(-0.337, -0.276), (0.337, -0.276), (0.337, 0.276), (-0.337, 0.276)],
        self_mask=masks,
        self_mask_calibrated=True,
        braking_deceleration=0.3,
        lateral_braking_deceleration=0.3,
        yaw_braking_deceleration=0.5,
        reaction_time=0.2,
        hard_margin=0.02,
        observation_max_age=0.8,
    )


def _captured_case():
    fixture = json.loads(
        (Path(__file__).parents[1] / "fixtures/fortress_c12_rear.json").read_text()
    )
    assert fixture["source_capture_sha256"] == (
        "a8f69abee10a2e7aee318ae4bef8afdf56fa9442e8a573b200e88d5231b5795f"
    )
    raw = fixture["scan"]
    stamp = raw["stamp"]
    scan = ScanObservation(
        tuple(float("nan") if value is None else value for value in raw["ranges"]),
        raw["angle_min"],
        raw["angle_increment"],
        raw["range_min"],
        raw["range_max"],
        stamp,
        "ros",
        Pose2D(*fixture["sensor_origin"]),
        0.0,
        "base_link",
        stamp,
        True,
        0.01,
        False,
        True,
    )
    masks = [[(-0.33, -0.24), (-0.23, -0.24), (-0.23, 0.24), (-0.33, 0.24)]]
    return scan, _profile(masks), fixture["uncertainty_margin"]


def _random_cases():
    rng = random.Random(0xC12)
    masks = [
        [],
        [[(-0.9, -0.4), (-0.4, -0.4), (-0.4, 0.4), (-0.9, 0.4)]],
        [[(-0.3, -0.3), (0.3, -0.3), (0.3, 0.3), (-0.3, 0.3)]],
        [[(0.7, -0.3), (1.1, 0.1), (0.8, 0.5), (0.5, 0.1)]],
        [
            [(-1.2, -0.5), (-0.4, -0.5), (-0.4, -0.1), (-0.8, -0.1), (-0.8, 0.3), (-1.2, 0.3)],
            [(0.6, -0.5), (0.9, -0.5), (0.9, 0.5), (0.6, 0.5)],
        ],
    ]
    result = []
    for index in range(20):
        count = 72
        distances = [rng.uniform(0.08, 5.0) for _ in range(count)]
        distances[0], distances[7], distances[17] = float("nan"), float("inf"), 0.01
        origin = Pose2D(
            rng.uniform(-0.2, 0.2), rng.uniform(-0.2, 0.2), rng.uniform(-math.pi, math.pi)
        )
        scan = ScanObservation(
            tuple(distances),
            -math.pi,
            2 * math.pi / count,
            0.08,
            5.0,
            10.0,
            "ros",
            origin,
            0.0,
            "base_link",
            10.0,
            True,
            0.09,
            False,
            True,
        )
        result.append((scan, _profile(masks[index % len(masks)]), 0.1))
    return result


def _boundary_cases():
    masks = [[(0.9, 0.0), (1.1, 0.0), (1.1, 0.2), (0.9, 0.2)]]
    result = []
    for distance in (0.889, 0.89, 0.899999999999, 0.9, 1.0, 1.1, 1.100000000001):
        scan = ScanObservation(
            (distance, distance, distance),
            -0.01,
            0.01,
            0.08,
            5.0,
            10.0,
            "ros",
            Pose2D(0.0, 0.0, 0.0),
            0.0,
            "base_link",
            10.0,
            True,
            0.01,
            False,
            True,
        )
        result.append((scan, _profile(masks), 0.1))
    return result


@pytest.mark.parametrize("overflow", ["endpoint", "uncertainty_extension"])
def test_nonfinite_geometry_extremes_preserve_legacy_filter_behavior(overflow):
    mask = [[(-2.0, -1.0), (-1.0, -1.0), (-1.0, 1.0), (-2.0, 1.0)]]
    origin = Pose2D(1e308 if overflow == "endpoint" else 0.0, 0.0, 0.0)
    scan = ScanObservation(
        (1e308,), 0.0, 0.01, 0.1, 1e308, 10.0, "ros", origin, 10.0, "base_link", 10.0
    )
    uncertainty = 0.1 if overflow == "endpoint" else 1e308

    def outcome(function):
        try:
            return function(scan, _profile(mask), self_mask_range_uncertainty=uncertainty)
        except (ValueError, RuntimeWarning, GEOSException) as exc:
            # //Compatible: GEOS versions differ in handling nonfinite coordinates.
            return (type(exc), str(exc))

    expected = outcome(legacy_filter_self)
    actual = outcome(self_filter.filter_self)
    assert actual == expected


def test_tiny_segment_and_exact_mask_boundary_keep_legacy_linestring_path(monkeypatch):
    original = self_filter.LineString
    count = 0

    def counted(*args, **kwargs):
        nonlocal count
        count += 1
        return original(*args, **kwargs)

    monkeypatch.setattr(self_filter, "LineString", counted)
    for distance, mask in (
        (1e-16, [[(10.0, -1.0), (11.0, -1.0), (11.0, 1.0), (10.0, 1.0)]]),
        (0.9, [[(0.9, 0.0), (1.1, 0.0), (1.1, 0.2), (0.9, 0.2)]]),
    ):
        scan = ScanObservation(
            (distance,),
            0.0,
            0.01,
            0.0,
            20.0,
            10.0,
            "ros",
            Pose2D(0, 0, 0),
            10.0,
            "base_link",
            10.0,
        )
        expected = legacy_filter_self(scan, _profile(mask))
        before = count
        actual = self_filter.filter_self(scan, _profile(mask))
        assert actual == expected
        assert count > before


def test_near_degenerate_triangle_keeps_legacy_mask_intersection_path(monkeypatch):
    from senior_control.safety.self_filter import RayEvidence

    origin = (0.0, 0.0)
    pairs = [
        (RayEvidence(0, origin, (1.0, 0.0), "hit"), RayEvidence(1, origin, (1.0, 1e-16), "hit"))
    ]
    masks = [Polygon([(10, 10), (11, 10), (11, 11), (10, 11)])]
    reference = legacy_exact_partial_fans(pairs, masks, False)
    original = coverage.Polygon
    count = 0

    class CountedTriangle:
        def __init__(self, polygon):
            self.polygon = polygon

        def __getattr__(self, name):
            return getattr(self.polygon, name)

        def intersects(self, mask):
            nonlocal count
            count += 1
            return self.polygon.intersects(mask)

    def counted(vertices):
        polygon = original(vertices)
        return (
            CountedTriangle(polygon)
            if isinstance(vertices, tuple) and len(vertices) == 3
            else polygon
        )

    monkeypatch.setattr(coverage, "Polygon", counted)
    actual = coverage._exact_partial_fans(pairs, masks, False)
    assert actual is not None and reference is not None
    assert len(actual) == len(reference)
    assert all(old.equals(new) for old, new in zip(reference, actual, strict=True))
    assert count == 1


def test_c12_rear_filter_avoids_unrelated_linestring_construction(monkeypatch):
    scan, profile, _ = _captured_case()
    reference = legacy_filter_self(scan, profile, self_mask_range_uncertainty=0.01)
    original = self_filter.LineString
    count = 0

    def counted(*args, **kwargs):
        nonlocal count
        count += 1
        return original(*args, **kwargs)

    monkeypatch.setattr(self_filter, "LineString", counted)
    actual = self_filter.filter_self(scan, profile, self_mask_range_uncertainty=0.01)
    assert actual == reference
    assert count < 720, f"unnecessary per-ray LineString work: {count}"


def test_c12_rear_fan_avoids_mask_intersection_for_disjoint_triangles(monkeypatch):
    scan, profile, _ = _captured_case()
    observed = legacy_filter_self(scan, profile, self_mask_range_uncertainty=0.01)
    pairs = list(zip(observed.rays, observed.rays[1:], strict=False))
    masks = [Polygon(vertices) for vertices in profile.self_mask]
    reference = legacy_exact_partial_fans(pairs, masks, False)
    original = coverage.Polygon
    count = 0

    class CountedTriangle:
        def __init__(self, polygon):
            self.polygon = polygon

        def __getattr__(self, name):
            return getattr(self.polygon, name)

        def intersects(self, mask):
            nonlocal count
            count += 1
            return self.polygon.intersects(mask)

    def counted(vertices):
        polygon = original(vertices)
        return (
            CountedTriangle(polygon)
            if isinstance(vertices, tuple) and len(vertices) == 3
            else polygon
        )

    monkeypatch.setattr(coverage, "Polygon", counted)
    actual = coverage._exact_partial_fans(pairs, masks, False)
    assert actual is not None and reference is not None
    assert len(actual) == len(reference)
    assert all(old.equals(new) for old, new in zip(reference, actual, strict=True))
    assert count < 300, f"unnecessary triangle-mask intersections: {count}"
