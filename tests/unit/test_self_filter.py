import json
import math
from pathlib import Path

from senior_control.config import DeviceProfile
from senior_control.models import Pose2D, Velocity
from senior_control.safety.guard import evaluate_motion
from senior_control.safety.self_filter import ScanObservation, filter_self


def scan(ranges, *, pose=None):
    pose = pose or Pose2D(0, 0, 0)
    return ScanObservation(
        tuple(ranges), -0.5, 0.5, 0.1, 5.0, 10.0, "ros", pose, 10.0, "base_link", 10.0
    )


def test_attached_occluder_never_creates_free_ray():
    profile = DeviceProfile(
        self_mask_calibrated=True, self_mask=[[(0.9, -0.2), (1.1, -0.2), (1.1, 0.2), (0.9, 0.2)]]
    )
    result = filter_self(scan([2.0, 1.0, 2.0]), profile)
    assert len(result.occluded_regions) == 1
    assert len(result.free_rays) == 2
    assert len(result.external_hits) == 2
    assert result.rays[1].kind == "self"


def test_nearby_wall_not_self():
    profile = DeviceProfile(
        self_mask_calibrated=True, self_mask=[[(0.9, -0.2), (1.1, -0.2), (1.1, 0.2), (0.9, 0.2)]]
    )
    result = filter_self(scan([0.5, 0.5, 0.5]), profile)
    assert len(result.external_hits) == 3


def test_self_overlap_within_range_uncertainty_is_unknown_not_free_or_hit():
    profile = DeviceProfile(
        self_mask_calibrated=True, self_mask=[[(0.9, -0.2), (1.1, -0.2), (1.1, 0.2), (0.9, 0.2)]]
    )

    def forward(distance, angle=0.0):
        return ScanObservation(
            (distance,),
            angle,
            0.1,
            0.05,
            5.0,
            10.0,
            "ros",
            Pose2D(0, 0, 0),
            10.0,
            "base_link",
            10.0,
        )

    uncertain = filter_self(forward(0.895), profile, self_mask_range_uncertainty=0.01)
    assert uncertain.rays[0].kind == "ambiguous_self_overlap"
    assert not uncertain.external_hits and not uncertain.free_rays
    assert uncertain.invalid_regions == uncertain.rays
    outside = filter_self(forward(0.889), profile, self_mask_range_uncertainty=0.01)
    assert outside.rays[0].kind == "hit" and outside.external_hits
    safety_profile = DeviceProfile(
        footprint=[(-0.5, -0.2), (0.88, -0.2), (0.88, 0.2), (-0.5, 0.2)],
        self_mask_calibrated=True,
        braking_deceleration=1.0,
        yaw_braking_deceleration=1.0,
        reaction_time=0.2,
        hard_margin=0.02,
        observation_max_age=1.0,
    )
    decision = evaluate_motion(Velocity(-0.05, 0), Velocity(0, 0), [outside], safety_profile, 10.0)
    assert not decision.allowed and decision.reason == "hard_contact"
    different_ray = filter_self(
        forward(0.895, math.pi / 2), profile, self_mask_range_uncertainty=0.01
    )
    assert different_ray.rays[0].kind == "hit"
    invalid = filter_self(forward(float("nan")), profile, self_mask_range_uncertainty=0.01)
    assert invalid.rays[0].kind == "invalid" and not invalid.free_rays


def test_actual_rear_fixture_overlap_remains_unknown_coverage():
    path = Path(__file__).parents[1] / "fixtures/fortress_rear_overlap.json"
    fixture = json.loads(path.read_text())
    raw = fixture["scan"]
    stamp = raw["stamp"]
    profile = DeviceProfile(
        mode="simulation",
        footprint=[(-0.337, -0.276), (0.337, -0.276), (0.337, 0.276), (-0.337, 0.276)],
        self_mask=[[(-0.33, -0.24), (-0.23, -0.24), (-0.23, 0.24), (-0.33, 0.24)]],
        self_mask_calibrated=True,
        braking_deceleration=0.3,
        yaw_braking_deceleration=0.5,
        reaction_time=0.2,
        hard_margin=0.02,
        observation_max_age=0.8,
    )
    observation = ScanObservation(
        tuple(float("nan") if r is None else r for r in raw["ranges"]),
        raw["angle_min"],
        raw["angle_increment"],
        raw["range_min"],
        raw["range_max"],
        stamp,
        "ros",
        Pose2D(*map(float, fixture["sensor_origin"])),
        0,
        "base_link",
        stamp,
        True,
        0.01,
        False,
        True,
    )
    filtered = filter_self(observation, profile, self_mask_range_uncertainty=0.01)
    assert sum(ray.kind == "ambiguous_self_overlap" for ray in filtered.rays) == 64
    decision = evaluate_motion(
        Velocity(0.05, 0),
        Velocity(0, 0),
        [filtered],
        profile,
        stamp,
        uncertainty_margin=float(fixture["uncertainty_margin"]),
    )
    assert not decision.allowed and decision.reason == "required area contains unknown space"


def test_no_return_clear_does_not_paint_inf_as_free():
    raw = scan([math.inf, 1.0])
    forced = ScanObservation(
        raw.ranges,
        raw.angle_min,
        raw.angle_increment,
        raw.range_min,
        raw.range_max,
        raw.stamp,
        raw.clock_domain,
        raw.sensor_to_body,
        raw.transform_stamp,
        raw.reference_frame,
        raw.reference_stamp,
        raw.interpolate,
        raw.max_angular_gap,
        True,
        raw.transform_static,
    )
    result = filter_self(forced, DeviceProfile(self_mask_calibrated=True))
    assert all(ray.kind != "clear" for ray in result.rays)
    assert result.rays[0].kind == "invalid"


def test_unverified_no_return_unknown():
    result = filter_self(scan([math.inf, math.nan, 1.0]), DeviceProfile(self_mask_calibrated=True))
    assert len(result.invalid_regions) == 2
    assert len(result.free_rays) == 1


def test_uncalibrated_mask_is_unusable():
    import pytest

    with pytest.raises(ValueError, match="uncalibrated"):
        filter_self(scan([1.0]), DeviceProfile())


def test_transformed_sensor_origin_applies_to_mask():
    profile = DeviceProfile(
        self_mask_calibrated=True, self_mask=[[(1.9, -0.2), (2.1, -0.2), (2.1, 0.2), (1.9, 0.2)]]
    )
    result = filter_self(scan([1.0], pose=Pose2D(1, 0, 0.5)), profile)
    assert result.rays[0].kind == "self"
