import json
from dataclasses import asdict
from pathlib import Path

import pytest
from pydantic import ValidationError

from senior_control.motion.hardware_profile import HardwareMotionProfile
from senior_control.real_runtime import load_real_parameters


def test_shared_sensor_policy_stays_independent_of_command_ttl():
    p = HardwareMotionProfile.model_validate(
        json.loads(Path("config/hardware-motion.json").read_text())
    )
    sim = HardwareMotionProfile.model_validate_json(
        Path("ros2/src/senior_sim/config/motion_profile.json").read_text()
    )
    assert p.max_age_ms == sim.max_age_ms == 1000
    assert p.lag_ms == 500
    assert sim.lag_ms == 200
    assert p.lateral_stop_ms == 1000
    assert sim.lateral_stop_ms == 0
    assert p.temporal().prediction_horizon_ms == sim.temporal().prediction_horizon_ms == 1000
    assert p.temporal().history_s == sim.temporal().history_s == 2.0
    loaded = load_real_parameters(Path("config/hardware-safety.yaml").resolve(), p)
    motion = loaded["scan_motion_node"]
    assert "scan_max_age" not in motion
    assert motion["max_interval_ms"] == 400.0
    assert "measurement_max_age" not in loaded["safety_gate"]
    assert "observation_max_age" not in loaded["safety_gate"]
    assert loaded["command_node"]["command_ttl_ms"] == 300.0
    assert "disconnect_seconds" not in loaded["command_node"]
    assert "heartbeat_seconds" not in loaded["command_node"]
    assert "heartbeat_ms" not in loaded["command_node"]


def test_retired_profile_copies_are_rejected_without_changing_derived_limits():
    raw = json.loads(Path("config/hardware-motion.json").read_text())
    current = HardwareMotionProfile.model_validate(raw)
    payload = {
        **raw,
        "max_age": 0.5,
        "lag": 0.5,
        "lateral_stop_s": 1.0,
        "admission_config": {"recovery_samples": 5, "recovery_span_ms": 500},
        "temporal_config": {
            "max_hold_ms": 150,
            "max_gap_ms": 150,
            "prediction_horizon_ms": 150,
        },
    }
    with pytest.raises(ValidationError):
        HardwareMotionProfile.model_validate(payload)
    assert payload["admission_config"]["recovery_samples"] == 5
    assert current.admission().recovery_span_ms == 0
    assert current.admission().heading_max_age_ms == 1000
    assert current.temporal().history_s == 2.0
    assert current.temporal().max_hold_ms == 1000
    assert current.lag_ms == 500
    assert current.lateral_stop_ms == 1000
    assert current.decel == raw["decel"]


def test_legacy_age_migration_does_not_hide_invalid_or_contradictory_config():
    raw = json.loads(Path("config/hardware-motion.json").read_text())
    with pytest.raises(ValidationError):
        HardwareMotionProfile.model_validate({**raw, "max_age": float("nan")})
    with pytest.raises(ValidationError):
        HardwareMotionProfile.model_validate({**raw, "lag": 0.5, "lateral_stop_s": 1.0})
    current = HardwareMotionProfile.model_validate(raw)
    fusion = asdict(current.fusion())
    fusion["normal_cap_mps"] += 1
    with pytest.raises(ValidationError, match="contradicts"):
        HardwareMotionProfile.model_validate({**raw, "fusion_config": fusion})
