import json
import math
from pathlib import Path

import pytest
from senior_perception.coverage import SweptRegion, supports
from senior_perception.self_filter import ScanObservation, filter_self

from vehicle_core.config import DeviceProfile
from vehicle_core.values import Pose2D


def sweep_region(vertices):
    return SweptRegion(vertices, "base_link", 10.0)


def observation(ranges, stamp=10.0):
    scan = ScanObservation(
        tuple(ranges),
        -1.0,
        0.5,
        0.1,
        5.0,
        stamp,
        "ros",
        Pose2D(0, 0, 0),
        stamp,
        "base_link",
        stamp,
        True,
        0.51,
    )
    return filter_self(scan, DeviceProfile(self_mask_calibrated=True))


def test_forward_observed_but_reverse_unknown():
    obs = observation([3.0] * 5)
    assert supports(
        sweep_region(((0.5, -0.1), (1.0, -0.1), (1.0, 0.1), (0.5, 0.1))), [obs], 10.1, "ros", 0.5
    ).supported
    assert not supports(
        sweep_region(((-1.0, -0.1), (-0.5, -0.1), (-0.5, 0.1), (-1.0, 0.1))),
        [obs],
        10.1,
        "ros",
        0.5,
    ).supported


def test_interior_blind_wedge_rejected():
    obs = observation([3.0, 3.0, float("nan"), 3.0, 3.0])
    sweep = sweep_region(((0.7317, -0.6816), (1.4634, -1.3632), (1.4634, 1.3632), (0.7317, 0.6816)))
    assert not supports(sweep, [obs], 10.1, "ros", 0.5).supported


def test_unknown_area_diagnostics_reports_actual_required_difference():
    from shapely import wkt

    obs = observation([3.0, 3.0, float("nan"), 3.0, 3.0])
    sweep = sweep_region(((0.5, -0.3), (1.0, -0.3), (1.0, 0.3), (0.5, 0.3)))
    diagnostic = {}
    result = supports(sweep, [obs], 10.1, "ros", 0.5, diagnostics=diagnostic)
    baseline = supports(sweep, [obs], 10.1, "ros", 0.5)
    assert not result.supported and result == baseline
    required = wkt.loads(diagnostic["required_wkt"])
    unknown = wkt.loads(diagnostic["unknown_wkt"])
    assert required.covers(unknown) and unknown.area > 0
    assert diagnostic["unknown_area"] == pytest.approx(unknown.area)


def _captured_fortress_seam(*, drop_tail=0, invalid_edge=None, mask=None, max_gap=0.01):
    from shapely import wkt

    fixture = json.loads(
        (Path(__file__).parents[1] / "fixtures/fortress_scan_seam.json").read_text()
    )
    raw = fixture["scan"]
    assert raw["stamp"] == fixture["selected_stamp_seconds"] == 25.2
    ranges = [float("nan") if value is None else value for value in raw["ranges"]]
    if drop_tail:
        ranges = ranges[:-drop_tail]
    if invalid_edge is not None:
        ranges[invalid_edge] = float("nan")
    sensor = Pose2D(*(float(value) for value in fixture["sensor_origin"]))
    stamp = raw["stamp"]
    scan = ScanObservation(
        tuple(ranges),
        raw["angle_min"],
        raw["angle_increment"],
        raw["range_min"],
        raw["range_max"],
        stamp,
        "ros",
        sensor,
        0.0,
        "base_link",
        float(fixture["reference_stamp"]),
        True,
        max_gap,
        False,
        True,
    )
    profile = DeviceProfile(self_mask_calibrated=True, self_mask=mask or [])
    obs = filter_self(scan, profile)
    required = wkt.loads(fixture["required_wkt"])
    sweep = SweptRegion(tuple(required.exterior.coords)[:-1], "base_link", stamp)
    return scan, obs, sweep


def test_actual_fortress_full_circle_wrap_is_observed():
    scan, obs, sweep = _captured_fortress_seam()
    gap = 2 * math.pi - (len(scan.ranges) - 1) * scan.angle_increment
    assert 0 < gap <= scan.max_angular_gap
    assert obs.rays[0].kind == obs.rays[-1].kind == "hit"
    assert supports(sweep, [obs], scan.stamp, "ros", 0.8).supported


def test_full_fan_fast_path_equals_union_of_real_beam_triangles():
    from senior_perception.coverage import _visible_areas
    from shapely.geometry import Point, Polygon
    from shapely.ops import unary_union

    scan, obs, sweep = _captured_fortress_seam()
    pairs = [*zip(obs.rays, obs.rays[1:], strict=False), (obs.rays[-1], obs.rays[0])]
    triangles = [Polygon((a.origin, a.endpoint, b.endpoint)) for a, b in pairs]
    blind = Point(obs.rays[0].origin).buffer(scan.range_min / math.cos(math.pi / 64), quad_segs=16)
    reference = unary_union(triangles).difference(blind)
    visible = _visible_areas(sweep, [obs], scan.stamp, "ros", 0.8)
    assert len(visible) == 1 and visible[0].equals(reference)


@pytest.mark.parametrize("invalid_edge", [0, -1])
def test_full_circle_wrap_with_invalid_edge_stays_unknown(invalid_edge):
    scan, obs, sweep = _captured_fortress_seam(invalid_edge=invalid_edge)
    assert not supports(sweep, [obs], scan.stamp, "ros", 0.8).supported


def test_partial_fov_does_not_fill_large_wrap_gap():
    scan, obs, sweep = _captured_fortress_seam(drop_tail=10)
    assert 2 * math.pi - (len(scan.ranges) - 1) * scan.angle_increment > scan.max_angular_gap
    assert not supports(sweep, [obs], scan.stamp, "ros", 0.8).supported


def test_partial_fov_does_not_bridge_missing_beams_even_with_loose_gap_parameter():
    scan, obs, sweep = _captured_fortress_seam(drop_tail=10, max_gap=0.1)
    gap = 2 * math.pi - (len(scan.ranges) - 1) * scan.angle_increment
    assert scan.angle_increment < gap <= scan.max_angular_gap
    assert not supports(sweep, [obs], scan.stamp, "ros", 0.8).supported


def test_full_circle_wrap_cannot_use_self_masked_edges():
    mask = [[(-0.11, -0.01), (-0.05, -0.01), (-0.05, 0.014), (-0.11, 0.014)]]
    scan, obs, sweep = _captured_fortress_seam(mask=mask)
    assert obs.rays[0].kind == obs.rays[-1].kind == "self"
    assert not supports(sweep, [obs], scan.stamp, "ros", 0.8).supported


def test_stale_or_wrong_clock_is_unusable():
    sweep = sweep_region(((0.5, -0.1), (1.0, -0.1), (1.0, 0.1), (0.5, 0.1)))
    assert not supports(sweep, [observation([3.0] * 5)], 11.0, "ros", 0.5).supported
    assert not supports(sweep, [observation([3.0] * 5)], 10.1, "monotonic", 0.5).supported


def test_stale_transform_is_unusable():
    scan = ScanObservation(
        (3.0, 3.0, 3.0),
        -0.5,
        0.5,
        0.1,
        5.0,
        10.0,
        "ros",
        Pose2D(0, 0, 0),
        9.0,
        "base_link",
        10.0,
        True,
        0.51,
    )
    obs = filter_self(scan, DeviceProfile(self_mask_calibrated=True))
    sweep = sweep_region(((0.5, -0.1), (1.0, -0.1), (1.0, 0.1), (0.5, 0.1)))
    assert not supports(sweep, [obs], 10.1, "ros", 0.5).supported


def test_attachment_between_rays_does_not_make_free_fan():
    scan = ScanObservation(
        (3.0, 3.0),
        -0.5,
        1.0,
        0.1,
        5.0,
        10.0,
        "ros",
        Pose2D(0, 0, 0),
        10.0,
        "base_link",
        10.0,
        True,
        1.01,
    )
    profile = DeviceProfile(
        self_mask_calibrated=True, self_mask=[[(0.9, -0.1), (1.1, -0.1), (1.1, 0.1), (0.9, 0.1)]]
    )
    obs = filter_self(scan, profile)
    sweep = sweep_region(((1.5, -0.1), (2.0, -0.1), (2.0, 0.1), (1.5, 0.1)))
    assert not supports(sweep, [obs], 10.1, "ros", 0.5).supported


def test_static_transform_stamp_zero_remains_usable():
    scan = ScanObservation(
        (3.0, 3.0, 3.0),
        -0.5,
        0.5,
        0.1,
        5.0,
        10.0,
        "ros",
        Pose2D(0, 0, 0),
        0.0,
        "base_link",
        10.0,
        True,
        0.51,
        transform_static=True,
    )
    obs = filter_self(scan, DeviceProfile(self_mask_calibrated=True))
    sweep = sweep_region(((0.5, -0.1), (1.0, -0.1), (1.0, 0.1), (0.5, 0.1)))
    assert supports(sweep, [obs], 10.1, "ros", 0.5).supported


def test_reference_epoch_mismatch_is_unusable():
    obs = observation([3.0] * 5)
    sweep = SweptRegion(((0.5, -0.1), (1.0, -0.1), (1.0, 0.1), (0.5, 0.1)), "base_link", 10.1)
    assert not supports(sweep, [obs], 10.1, "ros", 0.5).supported


@pytest.mark.parametrize(
    "vertices",
    [
        ((0.2, -0.05), (0.3, -0.05), (0.3, 0.05), (0.2, 0.05)),
        ((0.9, -0.05), (1.2, -0.05), (1.2, 0.05), (0.9, 0.05)),
    ],
)
def test_minimum_range_blind_disk_is_not_covered(vertices):
    scan = ScanObservation(
        (3.0, 3.0, 3.0),
        -0.5,
        0.5,
        1.0,
        5.0,
        10.0,
        "ros",
        Pose2D(0, 0, 0),
        10.0,
        "base_link",
        10.0,
        True,
        0.51,
    )
    obs = filter_self(scan, DeviceProfile(self_mask_calibrated=True))
    assert not supports(sweep_region(vertices), [obs], 10.1, "ros", 0.5).supported
