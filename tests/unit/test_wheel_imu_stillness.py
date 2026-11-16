"""All stillness consumers share raw wheel/IMU evidence, without localization."""

import math
from dataclasses import replace
from types import SimpleNamespace

import pytest
from senior_robot.motion_sensor_runtime import MotionSensorRuntime
from senior_robot.simulation_motion_profile import SimulationMotionProfile
from test_unified_motion_runtime import point

from senior_control.models import Velocity
from senior_control.motion.controller import Feedback, MotionController
from senior_control.motion.observations import Source
from senior_control.motion.stillness import stationary
from senior_control.runtime.diagnostics import RuntimeDiagnostics


def sample(vx=0.0, wz=0.0, *, epoch="wheel", axes=(True, True)):
    return Feedback(Velocity(vx, wz), 10, 11, epoch, 1, True, "raw test input", axes)


def test_stationary_without_laser_gps_vision_or_independent_body():
    wheel, imu = sample(0.015, 0.02), sample(wz=0.015, epoch="imu", axes=(False, True))
    assert stationary(wheel, imu, 10.1, linear_limit=0.02, yaw_limit=0.03)
    profile = SimulationMotionProfile()
    sensors = MotionSensorRuntime(profile, RuntimeDiagnostics(lambda *_: None))
    sensors.ingest(point(Source.WHEEL, 10, 10, (0.015, 0.02)), 10, 10, 10)
    sensors.ingest(point(Source.IMU, 10, 10, (0.015,)), 10, 10, 10)
    sensors.estimate(10, 10)
    assert sensors.independent is None
    assert sensors.stationary(10)
    assert sensors.recovery(10).stationary
    from test_directional_constraints import engine

    controller = MotionController(engine(), profile.controller(), Velocity(0.2, 0), Velocity(0, 0))
    assert controller.reset_at_stillness(wheel, imu, 10.1)
    assert controller.trajectory.velocity == Velocity(0, 0)


@pytest.mark.parametrize("axis", ["wheel_linear", "wheel_yaw", "imu_yaw"])
@pytest.mark.parametrize("value", [0.1, -0.1, math.nan, math.inf, -math.inf])
def test_motion_or_nonfinite_raw_input_never_claims_stillness(axis, value):
    wheel, imu = sample(), sample(epoch="imu", axes=(False, True))
    if axis == "wheel_linear":
        wheel = replace(wheel, velocity=SimpleNamespace(vx=value, wz=0))
    elif axis == "wheel_yaw":
        wheel = replace(wheel, velocity=SimpleNamespace(vx=0, wz=value))
    else:
        imu = replace(imu, velocity=SimpleNamespace(vx=0, wz=value))
    assert not stationary(wheel, imu, 10, linear_limit=0.02, yaw_limit=0.03)


@pytest.mark.parametrize("which", ["wheel", "imu"])
@pytest.mark.parametrize("failure", ["missing", "expired", "future", "unsupported", "unobserved"])
def test_absent_or_unusable_evidence_is_not_zero(which, failure):
    values = {"wheel": sample(), "imu": sample(epoch="imu", axes=(False, True))}
    value = values[which]
    values[which] = {
        "missing": None,
        "expired": replace(value, deadline_steady=10),
        "future": replace(value, stamp_steady=10.1),
        "unsupported": replace(value, current_supported=False),
        "unobserved": replace(value, observed_axes=(False, False)),
    }[failure]
    assert not stationary(values["wheel"], values["imu"], 10, linear_limit=0.02, yaw_limit=0.03)
