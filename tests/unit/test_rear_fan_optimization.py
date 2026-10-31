"""Partial-fan optimization preserves the exact pre-existing coverage semantics."""

import json
import math
import random
from dataclasses import replace
from pathlib import Path

import pytest
from shapely.geometry import Point, Polygon
from shapely.ops import unary_union as geometric_union

from senior_control.config import DeviceProfile
from senior_control.models import BodyTwist, Pose2D, Velocity
from senior_control.safety import coverage
from senior_control.safety.coverage import SweptRegion
from senior_control.safety.guard import evaluate_motion
from senior_control.safety.motion_runtime import reframe_observation
from senior_control.safety.self_filter import ScanObservation, filter_self


def make_observation(scenario):
    fixture = json.loads((Path(__file__).parents[1] / "fixtures/fortress_c4_rear.json").read_text())
    raw = fixture["scan"]
    values = [float("nan") if value is None else value for value in raw["ranges"]]
    if scenario == "partial_fov":
        values = values[:-60]
    if scenario == "edge_nan":
        values[0] = float("nan")
    if scenario == "interior_nan":
        values[350] = float("nan")
    if scenario == "multiple_gaps":
        values[150:153] = [float("nan")] * 3
        values[410:413] = [float("nan")] * 3
    stamp = raw["stamp"]
    profile = DeviceProfile(
        footprint=[(-0.337, -0.276), (0.337, -0.276), (0.337, 0.276), (-0.337, 0.276)],
        self_mask=[[(-0.33, -0.24), (-0.23, -0.24), (-0.23, 0.24), (-0.33, 0.24)]],
        self_mask_calibrated=True,
        braking_deceleration=0.3,
        lateral_braking_deceleration=0.3,
        yaw_braking_deceleration=0.5,
        reaction_time=0.2,
        hard_margin=0.02,
        observation_max_age=0.8,
    )
    if scenario == "multiple_masks":
        profile = profile.model_copy(
            update={
                "self_mask": [
                    *profile.self_mask,
                    [(0.8, -0.1), (0.9, -0.1), (0.9, 0.1), (0.8, 0.1)],
                ]
            }
        )
    scan = ScanObservation(
        tuple(values),
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
    observation = filter_self(scan, profile, self_mask_range_uncertainty=0.01)
    if scenario == "reframed":
        observation = reframe_observation(
            observation, Pose2D(0, 0, 0), Pose2D(0.03, 0.02, 0.2), stamp + 0.1
        )
    if scenario in {"different_origin", "reversed_rays"}:
        rays = list(observation.rays)
        if scenario == "different_origin":
            rays[340] = replace(rays[340], origin=(0.2, 0.1))
        else:
            a, b = rays[300], rays[301]
            rays[300], rays[301] = replace(a, endpoint=b.endpoint), replace(b, endpoint=a.endpoint)
        observation = replace(observation, rays=tuple(rays))
    return profile, observation


def original_visible(observation):
    scan = observation.scan
    rays = observation.rays
    pairs = list(zip(rays, rays[1:], strict=False))
    wrap_gap = 2 * math.pi - (len(scan.ranges) - 1) * scan.angle_increment
    if (
        len(rays) == len(scan.ranges)
        and len(rays) > 1
        and rays[0].index == 0
        and rays[-1].index == len(scan.ranges) - 1
        and 0 < wrap_gap <= scan.max_angular_gap
        and wrap_gap <= 1.001 * scan.angle_increment
    ):
        pairs.append((rays[-1], rays[0]))
    masks = [Polygon(mask) for mask in observation.self_masks]
    triangles = {}
    for a, b in pairs:
        if a.kind not in ("hit", "clear") or b.kind not in ("hit", "clear"):
            continue
        triangle = Polygon((a.origin, a.endpoint, b.endpoint))
        if (
            triangle.is_valid
            and triangle.area > 0
            and not any(triangle.intersects(m) for m in masks)
        ):
            triangles.setdefault(a.origin, []).append(triangle)
    groups = []
    for origin, pieces in triangles.items():
        visible = geometric_union(pieces)
        if scan.range_min > 0:
            visible = visible.difference(
                Point(origin).buffer(scan.range_min / math.cos(math.pi / 64), quad_segs=16)
            )
        if not visible.is_empty:
            groups.append(visible)
    return geometric_union(groups) if groups else None


@pytest.mark.parametrize(
    "scenario",
    [
        "actual",
        "partial_fov",
        "edge_nan",
        "interior_nan",
        "multiple_gaps",
        "reframed",
        "different_origin",
        "reversed_rays",
        "multiple_masks",
    ],
)
def test_partial_fans_exactly_match_original_pairwise_triangle_union(scenario):
    _, observation = make_observation(scenario)
    scan = observation.scan
    sweep = SweptRegion(
        ((1, 1), (2, 1), (2, 2), (1, 2)), scan.reference_frame, scan.reference_stamp
    )
    result = coverage._visible_areas(sweep, [observation], scan.reference_stamp, "ros", 0.8)
    expected = original_visible(observation)
    assert len(result) == 1 and expected is not None
    assert result[0].equals(expected)
    assert result[0].difference(expected).is_empty
    assert expected.difference(result[0]).is_empty


def test_seeded_gaps_and_unordered_rays_preserve_the_original_observed_set():
    _, baseline = make_observation("actual")
    rng = random.Random(31752)
    for case in range(16):
        rays = list(baseline.rays)
        for index in rng.sample(range(len(rays)), rng.randrange(1, 18)):
            rays[index] = replace(rays[index], kind="invalid", endpoint=None)
        if case % 5 == 0:
            first, second = rays[300], rays[301]
            if first.endpoint is not None and second.endpoint is not None:
                rays[300] = replace(first, endpoint=second.endpoint)
                rays[301] = replace(second, endpoint=first.endpoint)
        if case % 7 == 0:
            rays[400] = replace(rays[400], origin=(0.2, 0.1))
        observation = replace(baseline, rays=tuple(rays))
        scan = observation.scan
        sweep = SweptRegion(
            ((1, 1), (2, 1), (2, 2), (1, 2)), scan.reference_frame, scan.reference_stamp
        )
        result = coverage._visible_areas(sweep, [observation], scan.reference_stamp, "ros", 0.8)
        expected = original_visible(observation)
        assert len(result) == 1 and expected is not None
        assert result[0].equals(expected)


def test_actual_rear_partial_fan_avoids_hundreds_of_union_inputs(monkeypatch):
    profile, observation = make_observation("actual")
    assert len(observation.occluded_regions) == 64
    assert sum(ray.kind == "ambiguous_self_overlap" for ray in observation.rays) == 64
    calls = []
    original = coverage.unary_union

    def counted(items):
        calls.append(len(items))
        return original(items)

    monkeypatch.setattr(coverage, "unary_union", counted)
    decision = evaluate_motion(
        Velocity(0.05, 0),
        BodyTwist(0, 0, 0),
        [observation],
        profile,
        observation.scan.stamp,
        uncertainty_margin=0.8666,
    )
    assert not decision.allowed and decision.reason == "required area contains unknown space"
    assert not calls or max(calls) <= 8, f"per-beam union remains: {calls!r}"
