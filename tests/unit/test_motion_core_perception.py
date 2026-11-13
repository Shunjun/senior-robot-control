"""Measured free windows preserve blind/no-return gaps, including the 360 seam."""

import math
from dataclasses import replace

from senior_robot.motion_core_perception import scan_free_patches
from shapely.geometry import Point, Polygon
from shapely.ops import unary_union

from senior_control.config import DeviceProfile
from senior_control.models import Pose2D
from senior_control.safety.self_filter import ScanObservation, filter_self


def observation(ranges):
    return filter_self(
        ScanObservation(
            tuple(ranges),
            -math.pi,
            2 * math.pi / 1260,
            0.05,
            20.0,
            10.0,
            "ros",
            Pose2D(0, 0, 0),
            10.0,
            "base_link",
            10.0,
            True,
            0.01,
            False,
            True,
        ),
        DeviceProfile(self_mask_calibrated=True),
    )


def area(o):
    return unary_union([Polygon(p.vertices).buffer(-0.02) for p in scan_free_patches(o)])


def test_full_ring_seam_is_covered_after_error_erosion_but_blind_center_is_not():
    f = area(observation([10.0] * 1260))
    assert f.covers(Point(-0.4, 0))
    assert not f.covers(Point(0, 0))


def test_missing_and_no_return_rays_are_never_bridged_to_free():
    values = [10.0] * 1260
    for i in list(range(20)) + list(range(1240, 1260)):
        values[i] = float("inf")
    f = area(observation(values))
    assert not f.covers(Point(-0.4, 0))
    assert f.covers(Point(0.4, 0))
    original = observation([10.0] * 1260)
    sparse = replace(original, scan=replace(original.scan, angle_increment=0.02))
    assert not scan_free_patches(sparse)
    assert not scan_free_patches(replace(original, scan=replace(original.scan, interpolate=False)))


def test_inclusive_end_ray_layout_matches_actual_gazebo_lidar():
    original = observation([10.0] * 720)
    scan = replace(original.scan, angle_increment=2 * math.pi / 719)
    measured = filter_self(scan, DeviceProfile(self_mask_calibrated=True))
    assert area(measured).covers(Point(-0.4, 0))


def test_verified_no_return_contract_has_separate_evidence_not_finite_hits():
    from senior_control.motion.constraints import NoReturnPolicy, PatchKind

    original = observation([float("inf")] * 1260)
    assert not scan_free_patches(original)
    assert not original.external_hits
    patches = scan_free_patches(original, no_return_policy=NoReturnPolicy.VALIDATED_FREE_TO_RANGE)
    assert patches and all(p.kind is PatchKind.NO_RETURN for p in patches)
    geometry = unary_union([Polygon(p.vertices).buffer(-0.02) for p in patches])
    assert geometry.covers(Point(0.4, 0)) and geometry.covers(Point(-0.4, 0))
    assert not geometry.covers(Point(0, 0)) and not geometry.covers(Point(21, 0))
    assert all(x == float("inf") for x in original.scan.ranges)
    assert all(r.kind == "invalid" for r in original.rays)


def test_no_return_contract_never_clears_invalid_missing_or_self_occluded_rays():
    from senior_control.motion.constraints import NoReturnPolicy

    policy = NoReturnPolicy.VALIDATED_FREE_TO_RANGE
    for invalid in [float("nan"), -float("inf"), 0.0]:
        values = [float("inf")] * 1260
        for i in list(range(20)) + list(range(1240, 1260)):
            values[i] = invalid
        patches = scan_free_patches(observation(values), no_return_policy=policy)
        geometry = unary_union([Polygon(p.vertices).buffer(-0.02) for p in patches])
        assert not geometry.covers(Point(-0.4, 0))
        assert geometry.covers(Point(0.4, 0))
    masked = filter_self(
        observation([float("inf")] * 1260).scan,
        DeviceProfile(
            self_mask_calibrated=True,
            self_mask=[[(0.2, -0.2), (0.3, -0.2), (0.3, 0.2), (0.2, 0.2)]],
        ),
    )
    geometry = unary_union(
        [
            Polygon(p.vertices).buffer(-0.02)
            for p in scan_free_patches(masked, no_return_policy=policy)
        ]
    )
    assert not geometry.covers(Point(0.5, 0))
    assert geometry.covers(Point(-0.5, 0))


def test_candidate_contract_is_bound_to_known_sensor_frame_and_range():
    from dataclasses import replace

    from senior_robot.simulation_motion_profile import SimulationMotionProfile

    from senior_control.motion.constraints import NoReturnPolicy

    p = SimulationMotionProfile()
    assert (
        p.perception_no_return_policy("laser_link", 20.0) is NoReturnPolicy.VALIDATED_FREE_TO_RANGE
    )
    assert p.perception_no_return_policy("unknown_sensor", 20.0) is NoReturnPolicy.UNKNOWN
    assert p.perception_no_return_policy("laser_link", 30.0) is NoReturnPolicy.UNKNOWN
    assert (
        replace(p, simulation_only=False).perception_no_return_policy("laser_link", 20.0)
        is NoReturnPolicy.UNKNOWN
    )
