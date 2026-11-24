"""Actual output snapshot + constraint engine, with synthetic observed geometry only."""

from pathlib import Path

from senior_motion_control.hardware_profile import load_hardware_profile

ROOT = Path(__file__).resolve().parents[2]


def test_repository_hardware_deceleration_is_operator_assumption():
    p = load_hardware_profile(ROOT / "config/hardware-motion.json")
    assert p.decel == 0.8
    assert "ASSUMED" in p.evidence_basis and "no physical measurements" in p.evidence_basis


def test_hardware_profile_schema_and_launch_resolver_preserve_other_parameters():
    import json

    from senior_motion_control.hardware_profile import HardwareMotionProfile
    from senior_motion_control.motion_profile import select_motion_profile

    schema = json.loads((ROOT / "config/hardware-motion.schema.json").read_text())
    assert schema == HardwareMotionProfile.model_json_schema()
    p = select_motion_profile(
        hardware_path=str(ROOT / "config/hardware-motion.json"),
    )
    assert p.decel == 0.8
    assert (p.accel, p.yaw_decel, p.jerk, p.horizon_s) == (0.5, 0.15, 0.4, 12)
    assert p.reaction_linear_accel == 0.5
