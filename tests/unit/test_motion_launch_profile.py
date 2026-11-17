"""Environment profile uses the common numeric schema, without control special cases."""

import json
from pathlib import Path

from senior_control.motion.hardware_profile import HardwareMotionProfile


def test_standard_simulation_uses_shared_core_profile():
    path = Path("ros2/src/senior_sim/config/motion_profile.json")
    values = json.loads(path.read_text())
    profile = HardwareMotionProfile.model_validate(values)
    assert profile.wheel_topic == "/wheel/odom"
    assert profile.imu_topic == "/imu/data"
    assert profile.cap == 25 / 18
    assert profile.accel == 0.3
    assert profile.yaw_accel == 0.5
    assert profile.lag_ms == 200
    assert profile.max_age_ms == 1000
