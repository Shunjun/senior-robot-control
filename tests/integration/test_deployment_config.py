from __future__ import annotations

import importlib.util
import re
import subprocess
import sys
from pathlib import Path
from types import ModuleType

import pytest
import yaml

ROOT = Path(__file__).parents[2]
COMPOSE = ROOT / "deploy/compose.yaml"
ENV_EXAMPLE = ROOT / "deploy/env.example"
DOCTOR = ROOT / "tools/doctor.py"


def _doctor_module() -> ModuleType:
    assert DOCTOR.is_file(), "tools/doctor.py must provide read-only diagnostics"
    spec = importlib.util.spec_from_file_location("senior_doctor", DOCTOR)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_missing_secrets_fail_closed() -> None:
    assert COMPOSE.is_file(), "deploy/compose.yaml must require deployment secrets"
    text = COMPOSE.read_text(encoding="utf-8")
    for secret in ("SENIOR_SECRET", "ADMIN_PASSWORD_HASH", "TURN_SHARED_SECRET"):
        assert f"${{{secret}:?" in text


def test_deploy_never_exposes_dds() -> None:
    assert COMPOSE.is_file(), "deploy/compose.yaml must not expose the ROS graph"
    compose = yaml.safe_load(COMPOSE.read_text(encoding="utf-8"))
    assert all(service.get("network_mode") != "host" for service in compose["services"].values())
    published = " ".join(
        str(port)
        for service in compose["services"].values()
        for port in service.get("ports", ())
    )
    assert not re.search(r"\b(?:7400|7410|11811)\b", published)
    assert "robot" not in compose["services"]
    assert "agent" not in compose["services"]


def test_example_has_no_real_credentials() -> None:
    assert ENV_EXAMPLE.is_file(), "deploy/env.example must contain placeholders"
    values = {
        key: value
        for line in ENV_EXAMPLE.read_text(encoding="utf-8").splitlines()
        if line and not line.startswith("#")
        for key, value in [line.split("=", 1)]
    }
    sensitive = ("SENIOR_SECRET", "ADMIN_PASSWORD_HASH", "TURN_SHARED_SECRET")
    assert all(
        values.get(key, "").startswith("<") and values[key].endswith(">") for key in sensitive
    )
    assert not re.search(
        r"\b(?:10\.\d+\.\d+\.\d+|192\.168\.\d+\.\d+|172\.(?:1[6-9]|2\d|3[01])\.\d+\.\d+)\b",
        ENV_EXAMPLE.read_text(encoding="utf-8"),
    )


def test_unassembled_agent_entry_fails_loudly() -> None:
    completed = subprocess.run(
        (sys.executable, "-m", "senior_control.agent.main"),
        cwd=ROOT,
        capture_output=True,
        text=True,
        check=False,
    )

    assert completed.returncode != 0, "an unassembled agent entry must not report success"
    assert "connector" in completed.stderr
    assert "database" in completed.stderr
    assert "ROS IPC" in completed.stderr


def test_doctor_detects_duplicate_cmd_vel_and_tf(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    module = _doctor_module()
    profile = tmp_path / "hardware.yaml"
    profile.write_text((ROOT / "config/hardware.example.yaml").read_text(), encoding="utf-8")
    inspection = module.InspectionData(
        versions={"ros": "jazzy"},
        topics=("/cmd_vel", "/tf"),
        publishers={"/cmd_vel": ("safety_node", "rogue_node")},
        transforms=(("map", "base_link", "localization"), ("odom", "base_link", "odometry")),
        clock="system",
        chassis_watchdog=True,
        model_state="ready",
    )
    monkeypatch.setattr(module, "_inspect_live_system", lambda: inspection)

    report = module.doctor(profile)

    assert not report.ok
    assert report.checks["publisher_counts"].status == "failed"
    assert report.checks["tf"].status == "failed"
    assert "safety_node" in report.checks["publisher_counts"].detail
    assert "base_link" in report.checks["tf"].detail


def test_sim_real_profiles_cannot_mix(tmp_path: Path) -> None:
    module = _doctor_module()
    mixed = tmp_path / "mixed.yaml"
    mixed.write_text(
        """
mode: simulation
mode: hardware
footprint: [[0, 0], [1, 0], [0, 1]]
topics: {scan: /scan}
frames: {base: base_link}
sensor_coverage: {scan: 6.28}
""",
        encoding="utf-8",
    )

    report = module.doctor(mixed)

    assert not report.ok
    assert report.checks["profile"].status == "failed"
    assert "duplicate" in report.checks["profile"].detail.lower()
