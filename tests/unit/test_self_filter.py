import math

from senior_control.config import DeviceProfile
from senior_control.models import Pose2D
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
