"""Recorded Fortress test-oracle trajectory; never used as a runtime motion source."""

import bisect
import hashlib
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

from senior_control.safety.guard import stopping_distance

ROOT = Path(__file__).parents[2]
FIXTURE = ROOT / "tests/fixtures/fortress_lateral_impulses.json"
PROFILE = ROOT / "ros2/src/senior_robot/config/simulation_motion_flat_lateral.yaml"
BASE_PROFILE = ROOT / "ros2/src/senior_robot/config/simulation_motion.yaml"
ROBOT = ROOT / "ros2/src/senior_sim/urdf/senior.urdf.xacro"
WORLD = ROOT / "tests/fixtures/fortress_lateral_calibration_world.sdf"
BASE_WORLD = ROOT / "ros2/src/senior_sim/worlds/motion_qualification.sdf"


def test_empirical_flat_fortress_lateral_stopping_envelope():
    capture = json.loads(FIXTURE.read_text())
    calibrated = yaml.safe_load(PROFILE.read_text())["safety_gate"]["ros__parameters"]
    defaults = yaml.safe_load(BASE_PROFILE.read_text())["safety_gate"]["ros__parameters"]
    braking = calibrated["lateral_braking_deceleration"]
    reaction = defaults["reaction_time"]
    assert braking == pytest.approx(0.3) and reaction == pytest.approx(0.2)
    assert calibrated["physical_cap_context"] == "fortress-flat-v1"
    assert "lateral_braking_deceleration" not in defaults
    assert "physical_cap_context" not in defaults
    assert hashlib.sha256(ROBOT.read_bytes()).hexdigest() == capture["robot_xacro_sha256"]
    assert hashlib.sha256(WORLD.read_bytes()).hexdigest() == capture["world_sha256"]
    measured_world = ET.parse(WORLD).getroot()
    world = measured_world.find("world")
    assert world is not None
    plugin = world.find("./plugin[@name='ignition::gazebo::systems::ApplyLinkWrench']")
    assert plugin is not None
    world.remove(plugin)
    assert ET.tostring(measured_world) == ET.tostring(ET.parse(BASE_WORLD).getroot())
    assert (
        capture["raw_sha256"] == "6b888279825d712968a00ac0ee9abf2f63d6ccbf3027dd10a898478fcd521abb"
    )
    assert capture["owned_cleanup_empty"]
    assert len(capture["cmd_samples"]) == 331
    assert all(all(value == 0.0 for value in cmd) for cmd in capture["cmd_samples"])
    assert [pulse["force_N"] for pulse in capture["pulses"]] == [
        6000,
        -6000,
        12000,
        -12000,
        18000,
        -18000,
    ]
    nominal_period = capture["truth_period_seconds"]
    assert nominal_period == pytest.approx(0.02)

    for pulse in capture["pulses"]:
        samples = capture["truth"][pulse["start"] : pulse["end"]]
        assert len(samples) >= 100
        gaps = [b[0] - a[0] for a, b in zip(samples, samples[1:], strict=False)]
        assert max(gaps) <= nominal_period + 1e-9
        peak_index = max(range(len(samples)), key=lambda index: abs(samples[index][3]))
        peak_speed = abs(samples[peak_index][3])
        assert peak_speed > 0.04
        assert samples[peak_index][3] * pulse["force_N"] > 0
        assert abs(samples[-1][3]) < 0.005
        for _, _, _, _, qx, qy, qz, qw in samples:
            roll = math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
            assert abs(roll) < 0.02

        # Include the entire measured pulse, including pre-peak displacement.
        # Two sample periods at the observed peak are a disclosed sampling
        # allowance, not a proven bound on an unseen instantaneous peak.
        entire_excursion = max(abs(sample[2] - samples[0][2]) for sample in samples)
        empirical_uncertainty = 2 * nominal_period * peak_speed
        assert entire_excursion + empirical_uncertainty < stopping_distance(
            peak_speed, reaction, braking
        )
        for index in range(peak_index, len(samples)):
            speed = abs(samples[index][3])
            if speed < 0.005:
                continue  # Only classifies test-oracle settled samples.
            excursion = max(abs(later[2] - samples[index][2]) for later in samples[index:])
            assert excursion + nominal_period * speed < stopping_distance(speed, reaction, braking)


def test_recorded_short_pulse_intervals_fit_reported_marginal_covariance():
    capture = json.loads(FIXTURE.read_text())
    truth = capture["truth"]
    timestamps = [sample[0] for sample in truth]
    assert len(truth) == 711 and len(capture["motion"]) == 115
    assert max(b - a for a, b in zip(timestamps, timestamps[1:], strict=False)) <= 0.020000001

    def state(stamp):
        index = bisect.bisect_left(timestamps, stamp)
        assert 0 <= index < len(truth)
        if abs(timestamps[index] - stamp) < 1e-8:
            return truth[index]
        assert index > 0
        before, after = truth[index - 1], truth[index]
        fraction = (stamp - before[0]) / (after[0] - before[0])
        return [a + fraction * (b - a) for a, b in zip(before, after, strict=True)]

    def yaw(sample):
        qx, qy, qz, qw = sample[4:]
        return math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))

    for start, end, dx, dy, dtheta, var_x, var_y, var_yaw in capture["motion"]:
        first, last = state(start), state(end)
        heading = yaw(first)
        x, y = last[1] - first[1], last[2] - first[2]
        observed = (
            math.cos(heading) * x + math.sin(heading) * y,
            -math.sin(heading) * x + math.cos(heading) * y,
            math.atan2(math.sin(yaw(last) - heading), math.cos(yaw(last) - heading)),
        )
        for actual, estimate, variance in zip(
            observed, (dx, dy, dtheta), (var_x, var_y, var_yaw), strict=True
        ):
            assert variance > 0
            assert abs(actual - estimate) <= 3 * math.sqrt(variance)
