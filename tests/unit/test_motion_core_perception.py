"""Directional distance limits following the vehicle's real driving verification."""

import math
from types import SimpleNamespace

import pytest
from control_process_test_support import generic_motion_profile
from senior_robot.motion_core_perception import directional_clearance, limit_velocity
from senior_robot.raw_safety_observation import RawSafetyObservation

from senior_control.models import Velocity


@pytest.fixture
def profile():
    return generic_motion_profile(length=0.8, width=0.4, sensor_error=0.03, decel=0.5)


def scan(ranges, *, angle_min=0.0, sensor=(0.0, 0.0, 0.0)):
    message = SimpleNamespace(
        ranges=ranges,
        angle_min=angle_min,
        angle_increment=math.pi,
        range_min=0.05,
        range_max=20.0,
        sensor_x=sensor[0],
        sensor_y=sensor[1],
        sensor_yaw=sensor[2],
    )
    return (RawSafetyObservation(message, 9.9, 9.9, 9.8, 9.8),)


def test_configured_dimensions_and_sensor_transform_set_the_only_boundary(profile):
    # 0.8 x 0.4 body: boundary half extents are 0.47 x 0.25, including error once.
    # Sensor +90 degrees and ray -90 degrees give body hit (1.0, 0.249).
    inside = directional_clearance(
        scan([0.9], angle_min=-math.pi / 2, sensor=(0.1, 0.249, math.pi / 2)),
        profile,
        10.0,
    )
    outside = directional_clearance(
        scan([0.9], angle_min=-math.pi / 2, sensor=(0.1, 0.251, math.pi / 2)),
        profile,
        10.0,
    )
    assert inside.front == pytest.approx(0.53)
    assert inside.rear == math.inf
    assert outside.front == math.inf


def test_forward_approach_remains_positive_until_the_boundary(profile):
    requested = previous = Velocity(0.4, 0.0)
    outputs = []
    # One approach: far away, two closer positions, then the boundary.
    for front_range in (2.47, 0.49, 0.475, 0.47):
        clearance = directional_clearance(scan([front_range]), profile, 10.0)
        previous = limit_velocity(requested, previous, previous.vx, clearance, profile, 0.05)
        outputs.append(previous.vx)
    assert outputs == pytest.approx([0.4, 0.1414213562, 0.0707106781, 0.0])


def test_rear_approach_stops_at_boundary_and_still_allows_forward(profile):
    requested = previous = Velocity(-0.4, 0.0)
    outputs = []
    for rear_range in (2.47, 0.49, 0.475, 0.47):
        clearance = directional_clearance(scan([2.47, rear_range]), profile, 10.0)
        previous = limit_velocity(requested, previous, previous.vx, clearance, profile, 0.05)
        outputs.append(previous.vx)
    assert outputs == pytest.approx([-0.4, -0.1414213562, -0.0707106781, 0.0])
    forward = limit_velocity(Velocity(0.4, 0.0), previous, 0.0, clearance, profile, 0.05)
    assert forward.vx == pytest.approx(0.015)


def test_in_place_turn_uses_yaw_ramp_and_cap_at_linear_obstacle_boundaries(profile):
    clearance = directional_clearance(scan([0.47, 0.47]), profile, 10.0)
    assert clearance.front == clearance.rear == 0.0
    previous = Velocity(0.0, 0.0)
    outputs = []
    for _ in range(25):
        previous = limit_velocity(Velocity(0.0, 2.0), previous, None, clearance, profile, 0.1)
        outputs.append(previous)
    assert outputs[0] == Velocity(0.0, 0.05)
    assert outputs[-1] == Velocity(0.0, profile.yaw_cap)
    assert all(output.vx == 0.0 and 0.0 < output.wz <= profile.yaw_cap for output in outputs)
