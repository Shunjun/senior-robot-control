"""Environment profile uses the common numeric schema, without control special cases."""

import json
from pathlib import Path

from senior_motion_control.hardware_profile import HardwareMotionProfile


def test_hardware_uses_requested_linear_rates():
    values = json.loads(Path("config/hardware-motion.json").read_text())
    profile = HardwareMotionProfile.model_validate(values)
    assert profile.accel == 0.5
    assert profile.decel == 0.8
    assert profile.reaction_linear_accel == 0.5


def test_standard_simulation_uses_shared_core_profile():
    path = Path("ros2/src/senior_sim/config/motion_profile.json")
    values = json.loads(path.read_text())
    profile = HardwareMotionProfile.model_validate(values)
    assert profile.wheel_topic == "/wheel/odom"
    assert profile.imu_topic == "/imu/data"
    assert profile.cap == 25 / 18
    assert profile.accel == 0.5
    assert profile.decel == 0.8
    assert profile.reaction_linear_accel == 0.5
    assert profile.yaw_accel == 0.5
    assert profile.lag_ms == 200
    assert profile.max_age_ms == 1000
