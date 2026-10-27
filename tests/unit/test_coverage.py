from senior_control.config import DeviceProfile
from senior_control.models import Pose2D
from senior_control.safety.coverage import SweptRegion, supports
from senior_control.safety.self_filter import ScanObservation, filter_self


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
