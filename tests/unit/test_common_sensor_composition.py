import json
from pathlib import Path

from senior_robot.simulation_motion_profile import SimulationMotionProfile

from senior_control.motion.hardware_profile import HardwareMotionProfile
from senior_control.real_runtime import load_real_parameters


def test_old_age_copies_migrate_in_memory_to_shared_policy_not_command_ttl():
    p = HardwareMotionProfile.model_validate(
        json.loads(Path("config/hardware-motion.json").read_text())
    )
    sim = SimulationMotionProfile()
    assert p.max_age == sim.max_age == 1.0
    assert p.temporal().prediction_horizon_s == sim.temporal().prediction_horizon_s == 1.0
    loaded = load_real_parameters(Path("config/hardware-safety.yaml").resolve(), p)
    assert loaded["scan_motion_node"]["scan_max_age"] == 1.0
    assert "measurement_max_age" not in loaded["safety_gate"]
    assert loaded["command_node"]["command_ttl_seconds"] == 0.3
    assert "disconnect_seconds" not in loaded["command_node"]


def test_nested_legacy_sensor_policy_migrates_without_changing_response_limits():
    from dataclasses import asdict

    from senior_control.motion.hardware_profile import HardwareMotionProfile

    raw = json.loads(Path("config/hardware-motion.json").read_text())
    current = HardwareMotionProfile.model_validate(raw)
    old = asdict(current.admission())
    old.update(
        recovery_samples=5,
        recovery_span_s=0.5,
        heading_max_age_s=0.3,
        alignment_max_skew_s=0.15,
        clock_pause_s=0.3,
    )
    old["rules"] = [{**row, "max_age_s": 0.5} for row in old["rules"]]
    raw["admission_config"] = old
    raw["temporal_config"] = {
        **asdict(current.temporal()),
        "max_hold_s": 0.15,
        "max_gap_s": 0.15,
        "prediction_horizon_s": 0.15,
    }
    migrated = HardwareMotionProfile.model_validate(raw)
    assert migrated.admission() == current.admission()
    assert migrated.temporal() == current.temporal()
    assert migrated.decel == raw["decel"]
    assert raw["admission_config"]["recovery_samples"] == 5  # input remains untouched


def test_legacy_age_migration_does_not_hide_invalid_or_contradictory_config():
    import pytest
    from pydantic import ValidationError

    raw = json.loads(Path("config/hardware-motion.json").read_text())
    with pytest.raises(ValidationError):
        HardwareMotionProfile.model_validate({**raw, "max_age": float("nan")})
    from dataclasses import asdict

    current = HardwareMotionProfile.model_validate(raw)
    admission = asdict(current.admission())
    admission["normal_cap_mps"] += 1
    with pytest.raises(ValidationError, match="contradicts"):
        HardwareMotionProfile.model_validate({**raw, "admission_config": admission})
