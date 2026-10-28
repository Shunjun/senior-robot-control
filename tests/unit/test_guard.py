"""Gate limits below are explicit simulation configuration, not a vehicle braking guarantee."""

import math

from senior_control.config import DeviceProfile
from senior_control.models import Pose2D, Velocity
from senior_control.safety.guard import evaluate_motion
from senior_control.safety.self_filter import ScanObservation, filter_self

RECT = ((-0.337, -0.27), (0.337, -0.27), (0.337, 0.27), (-0.337, 0.27))


def limits(**overrides: object) -> DeviceProfile:
    values: dict[str, object] = {
        "mode": "simulation",
        "footprint": list(RECT),
        "self_mask_calibrated": True,
        "braking_deceleration": 1.0,
        "yaw_braking_deceleration": 1.0,
        "reaction_time": 0.2,
        "hard_margin": 0.02,
        "observation_max_age": 0.5,
    }
    values.update(overrides)
    return DeviceProfile(**values)  # type: ignore[arg-type]


def observation(
    angle_min: float,
    *,
    hit: float = 0.6,
    stamp: float = 10.0,
    reference_stamp: float = 10.0,
    transform_static: bool = False,
    transform_stamp: float | None = None,
    nan_rear: bool = False,
    nan_center: float | None = None,
    nan_width: float = 0.0,
):
    steps = 180
    increment = 2 * math.pi / steps
    ranges: list[float] = []
    best = 1e9
    forward_index = 0
    for index in range(steps + 1):
        angle = (angle_min + index * increment + math.pi) % (2 * math.pi) - math.pi
        distance = 5.0
        if nan_rear and abs(abs(angle) - math.pi) < 0.4:
            distance = float("nan")
        if nan_center is not None:
            delta = (angle - nan_center + math.pi) % (2 * math.pi) - math.pi
            if abs(delta) < nan_width:
                distance = float("nan")
        if abs(angle) < best and distance == distance:
            best = abs(angle)
            forward_index = index
        ranges.append(distance)
    if ranges[forward_index] == ranges[forward_index]:
        ranges[forward_index] = hit
    scan = ScanObservation(
        tuple(ranges),
        angle_min,
        increment,
        0.05,
        8.0,
        stamp,
        "sim",
        Pose2D(0.0, 0.0, 0.0),
        stamp if transform_stamp is None else transform_stamp,
        "base_link",
        reference_stamp,
        True,
        increment,
        False,
        transform_static,
    )
    return filter_self(scan, DeviceProfile(self_mask_calibrated=True))


def scan_pair(**kwargs: object):
    return [observation(-math.pi / 2, **kwargs), observation(math.pi / 2, **kwargs)]  # type: ignore[arg-type]


def point_hit(x: float, y: float):
    angle = math.atan2(y, x)
    scan = ScanObservation(
        (math.hypot(x, y), 5.0),
        angle,
        0.01,
        0.05,
        8.0,
        10.0,
        "sim",
        Pose2D(0.0, 0.0, 0.0),
        10.0,
        "base_link",
        10.0,
        True,
        0.01,
        False,
        False,
    )
    return filter_self(scan, DeviceProfile(self_mask_calibrated=True))


def test_warning_overlap_allows_safe_reverse():
    # Speed 0.8 m/s stops in 0.48 m. The forward warning reaches x=0.837 and
    # contains the 0.60 m hit. The reverse hard sweep stays at x=0.357.
    observations = scan_pair()
    profile = limits()
    reverse = evaluate_motion(Velocity(-0.8, 0.0), Velocity(0.0, 0.0), observations, profile, 10.2)
    forward = evaluate_motion(Velocity(0.8, 0.0), Velocity(0.0, 0.0), observations, profile, 10.2)
    clear = evaluate_motion(
        Velocity(-0.8, 0.0), Velocity(0.0, 0.0), scan_pair(hit=5.0), profile, 10.2
    )
    assert reverse.allowed
    assert reverse.velocity == Velocity(-0.8, 0.0)
    assert reverse.reason == "warning_overlap"
    assert not forward.allowed
    assert forward.velocity == Velocity(0.0, 0.0)
    assert forward.reason == "hard_sweep_collision"
    assert clear.allowed
    assert clear.reason == "hard_sweep_clear"


def test_forward_measured_warning_overlap_allows_reverse():
    # Measured 0.3 m/s still coasts forward 0.105 m, so the hard front is
    # 0.462 m. Commanded 0.8 m/s puts the forward warning out to 0.837 m,
    # which contains the 0.60 m hit without putting it in the hard sweep.
    decision = evaluate_motion(Velocity(-0.8, 0.0), Velocity(0.3, 0.0), scan_pair(), limits(), 10.2)
    assert decision.allowed
    assert decision.velocity == Velocity(-0.8, 0.0)
    assert decision.reason == "warning_overlap"


def test_invalid_depth_is_unknown():
    profile = limits()
    valid = evaluate_motion(
        Velocity(-0.4, 0.0), Velocity(0.0, 0.0), scan_pair(hit=5.0), profile, 10.2
    )
    unknown = evaluate_motion(
        Velocity(-0.4, 0.0),
        Velocity(0.0, 0.0),
        scan_pair(hit=5.0, nan_rear=True),
        profile,
        10.2,
    )
    assert valid.allowed
    assert valid.velocity == Velocity(-0.4, 0.0)
    assert not unknown.allowed
    assert unknown.velocity == Velocity(0.0, 0.0)


def test_stale_sensor_stops():
    observations = scan_pair()
    profile = limits()
    command = Velocity(0.4, 0.0)
    measured = Velocity(0.4, 0.0)
    fresh = evaluate_motion(command, measured, observations, profile, 10.2)
    stale = evaluate_motion(command, measured, observations, profile, 11.0)
    assert fresh.allowed
    assert fresh.velocity == command
    assert not stale.allowed
    assert stale.velocity == Velocity(0.0, 0.0)


def test_measured_velocity_extends_braking():
    # 0.4 m/s stops in 0.16 m; hard front is 0.517 m, short of the 0.60 m hit.
    # Measured 0.9 m/s stops in 0.585 m; hard front is 0.942 m and contains it.
    observations = scan_pair()
    profile = limits()
    command = Velocity(0.4, 0.0)
    held = evaluate_motion(command, Velocity(0.4, 0.0), observations, profile, 10.2)
    overrun = evaluate_motion(command, Velocity(0.9, 0.0), observations, profile, 10.2)
    assert held.allowed
    assert held.velocity == command
    assert not overrun.allowed
    assert overrun.velocity == Velocity(0.0, 0.0)


def test_unset_safety_limits_stop():
    observations = scan_pair()
    command = Velocity(-0.4, 0.0)
    measured = Velocity(0.0, 0.0)
    configured = evaluate_motion(command, measured, observations, limits(), 10.2)
    assert configured.allowed
    assert configured.velocity == command
    for field in (
        "braking_deceleration",
        "reaction_time",
        "hard_margin",
        "observation_max_age",
    ):
        decision = evaluate_motion(command, measured, observations, limits(**{field: None}), 10.2)
        assert not decision.allowed
        assert decision.velocity == Velocity(0.0, 0.0)
    translation = evaluate_motion(
        command, measured, observations, limits(yaw_braking_deceleration=None), 10.2
    )
    assert translation.allowed
    assert translation.velocity == command


def test_hard_contact_does_not_command_escape():
    profile = limits(hard_margin=0.0)
    command = Velocity(-0.4, 0.0)
    measured = Velocity(0.0, 0.0)
    contact = evaluate_motion(command, measured, scan_pair(hit=0.2), profile, 10.2)
    clear = evaluate_motion(command, measured, scan_pair(hit=0.6), profile, 10.2)
    assert not contact.allowed
    assert contact.velocity == Velocity(0.0, 0.0)
    assert clear.allowed
    assert clear.velocity == command


def test_reference_mismatch_rejected():
    profile = limits(hard_margin=0.0)
    command = Velocity(-0.4, 0.0)
    measured = Velocity(0.0, 0.0)
    aligned = [observation(-math.pi / 2, hit=5.0)]
    mismatched = aligned + [observation(math.pi / 2, hit=5.0, reference_stamp=10.5)]
    accepted = evaluate_motion(command, measured, aligned, profile, 10.2)
    rejected = evaluate_motion(command, measured, mismatched, profile, 10.2)
    assert accepted.allowed
    assert accepted.velocity == command
    assert not rejected.allowed
    assert rejected.velocity == Velocity(0.0, 0.0)


def test_mid_arc_corner_hit_blocks_rotation():
    # 0.8 rad/s stops in 0.48 rad. The hit is the reviewer's mid-arc sample.
    radius = math.hypot(0.337, 0.27)
    distance = 0.995 * radius
    angle = 0.908
    observations = scan_pair(hit=5.0) + [
        point_hit(distance * math.cos(angle), distance * math.sin(angle))
    ]
    decision = evaluate_motion(
        Velocity(0.0, 0.8),
        Velocity(0.0, 0.0),
        observations,
        limits(hard_margin=0.0),
        10.2,
    )
    assert not decision.allowed
    assert decision.velocity == Velocity(0.0, 0.0)
    assert decision.reason == "hard_sweep_collision"


def test_combined_arc_carries_body_into_obstacle():
    # v=0.8 and w=0.8 stop after 0.48 m and 0.48 rad. Body point (0.325, 0.259)
    # is inside the footprint at that arc pose and outside the straight box.
    yaw = 0.48
    travel_x = math.sin(yaw)
    travel_y = 1.0 - math.cos(yaw)
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    body_x = 0.325
    body_y = 0.259
    world_x = travel_x + cosine * body_x - sine * body_y
    world_y = travel_y + sine * body_x + cosine * body_y
    observations = scan_pair(hit=5.0) + [point_hit(world_x, world_y)]
    decision = evaluate_motion(Velocity(0.8, 0.8), Velocity(0.0, 0.0), observations, limits(), 10.2)
    assert not decision.allowed
    assert decision.velocity == Velocity(0.0, 0.0)
    assert decision.reason == "hard_sweep_collision"


def test_unobserved_rotation_stops():
    # Zero hard margin leaves no translation sliver, so only the corner arc is new.
    # 0.8 rad/s, reaction 0.2 s, yaw braking 1 rad/s^2 stops in 0.48 rad.
    profile = limits(hard_margin=0.0)
    command = Velocity(0.0, 0.8)
    measured = Velocity(0.0, 0.0)
    clear = evaluate_motion(command, measured, scan_pair(hit=5.0), profile, 10.2)
    unknown = evaluate_motion(
        command,
        measured,
        scan_pair(hit=5.0, nan_center=0.9, nan_width=0.35),
        profile,
        10.2,
    )
    assert clear.allowed
    assert clear.velocity == command
    assert not unknown.allowed
    assert unknown.velocity == Velocity(0.0, 0.0)


def test_static_extrinsic_stamp_zero_allows_motion():
    profile = limits()
    command = Velocity(-0.4, 0.0)
    measured = Velocity(0.0, 0.0)
    static = scan_pair(transform_static=True, transform_stamp=0.0)
    dynamic = scan_pair(transform_static=False, transform_stamp=0.0)
    allowed = evaluate_motion(command, measured, static, profile, 10.2)
    rejected = evaluate_motion(command, measured, dynamic, profile, 10.2)
    assert allowed.allowed
    assert allowed.velocity == command
    assert not rejected.allowed
    assert rejected.velocity == Velocity(0.0, 0.0)
