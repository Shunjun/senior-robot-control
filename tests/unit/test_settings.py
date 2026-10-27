import pytest

from senior_control.config import SettingsSource, load_settings


def test_settings_precedence_and_redaction(tmp_path):
    path = tmp_path / "config.yaml"
    path.write_text("heartbeat_seconds: 0.4\nsecret: yaml-secret\n")
    settings = load_settings(
        SettingsSource(
            yaml_path=path,
            environment={"SENIOR_HEARTBEAT_SECONDS": "0.3", "SENIOR_SECRET": "env-secret"},
            cli={"heartbeat_seconds": 0.2},
        )
    )
    assert settings.heartbeat_seconds == 0.2
    assert settings.secret.get_secret_value() == "env-secret"
    with pytest.raises(ValueError) as exc:
        load_settings(
            SettingsSource(
                environment={
                    "SENIOR_SECRET": "sensitive-value",
                    "SENIOR_HEARTBEAT_SECONDS": "sensitive-value",
                }
            )
        )
    assert "sensitive-value" not in str(exc.value)


def test_timing_constraints():
    with pytest.raises(ValueError):
        load_settings(
            SettingsSource(cli={"secret": "x", "heartbeat_seconds": 2, "disconnect_seconds": 1})
        )


def test_import_has_no_side_effects(tmp_path):
    import os
    import subprocess
    import sys

    result = subprocess.run(
        [sys.executable, "-c", "import senior_control.config, senior_control.storage.database"],
        cwd=tmp_path,
        env={"PATH": os.environ["PATH"]},
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr
    assert list(tmp_path.iterdir()) == []


def test_log_redacts_secrets():
    import logging

    from senior_control.observability import JsonFormatter

    record = logging.LogRecord(
        "test", logging.INFO, "", 0, "token=very-secret password=hunter2", (), None
    )
    record.operation_id = "op-1"
    result = JsonFormatter().format(record)
    assert "very-secret" not in result
    assert "hunter2" not in result
    assert "op-1" in result


def test_configuration_traceback_does_not_expose_input():
    import traceback

    with pytest.raises(ValueError) as exc:
        load_settings(
            SettingsSource(
                environment={"SENIOR_SECRET": "x", "SENIOR_HEARTBEAT_SECONDS": "trace-secret"}
            )
        )
    assert "trace-secret" not in "".join(traceback.format_exception(exc.value))


def test_mounted_secrets_and_nested_env_precedence(tmp_path):
    from senior_control.config import AgentSettings

    config = tmp_path / "config.yaml"
    config.write_text("secret: yaml\ndevice:\n  mode: simulation\n  topics:\n    cmd: /yaml\n")
    secrets = tmp_path / "secrets"
    secrets.mkdir()
    (secrets / "SENIOR_SECRET").write_text("mounted")
    result = load_settings(
        SettingsSource(
            yaml_path=config,
            secrets_dir=secrets,
            environment={"SENIOR_DEVICE__TOPICS__CMD": "/env"},
            cli={"device": {"topics": {"scan": "/cli"}}},
        )
    )
    assert isinstance(result, AgentSettings)
    assert result.secret.get_secret_value() == "mounted"
    assert result.device.topics == {"cmd": "/env", "scan": "/cli"}


def test_cloud_settings_rejects_device_and_empty_secret():
    from senior_control.config import CloudSettings

    result = load_settings(SettingsSource(cli={"role": "cloud", "secret": "abc"}, environment={}))
    assert isinstance(result, CloudSettings)
    for values in ({"role": "cloud", "secret": "abc", "device": {}}, {"secret": ""}):
        with pytest.raises(ValueError):
            load_settings(SettingsSource(cli=values, environment={}))


def test_hardware_profile_requires_geometry_and_ros_mapping():
    with pytest.raises(ValueError):
        load_settings(
            SettingsSource(cli={"secret": "x", "device": {"mode": "hardware"}}, environment={})
        )
    settings = load_settings(
        SettingsSource(
            cli={
                "secret": "x",
                "device": {
                    "mode": "hardware",
                    "footprint": [[0, 0], [1, 0], [0, 1]],
                    "frames": {"base": "base_link"},
                    "topics": {"cmd": "/cmd_vel", "scan": "/scan"},
                    "sensor_coverage": {"scan": 6.0},
                },
            },
            environment={},
        )
    )
    assert settings.device.mode == "hardware"


def test_nested_wrong_type_rejected():
    with pytest.raises(ValueError):
        load_settings(
            SettingsSource(environment={"SENIOR_SECRET": "x", "SENIOR_DEVICE__TOPICS": "not-a-map"})
        )


def test_unstructured_and_exception_logs_never_emit_input():
    import json
    import logging

    from senior_control.observability import JsonFormatter

    record = logging.LogRecord(
        "test", logging.ERROR, "", 0, 'authorization: Bearer "super secret" position=1,2', (), None
    )
    record.event = "not-an-approved-event"
    record.operation_id = "op-23"
    formatted = json.loads(JsonFormatter().format(record))
    assert formatted["event"] == "unstructured_log"
    assert formatted["operation_id"] == "op-23"
    assert "super secret" not in str(formatted)
    assert "position" not in str(formatted)


def test_approved_event_keeps_identifiers():
    import json
    import logging

    from senior_control.observability import JsonFormatter

    record = logging.LogRecord("test", logging.INFO, "", 0, "token: secret", (), None)
    record.event = "operation_failed"
    record.operation_id = "op-23"
    result = json.loads(JsonFormatter().format(record))
    assert result["event"] == "operation_failed"
    assert result["operation_id"] == "op-23"
    assert "secret" not in str(result)


def test_clock_is_injected_without_sleep():
    from senior_control.clock import Clock

    class FakeClock:
        now = 10.0

        def monotonic(self) -> float:
            return self.now

    def expired(clock: Clock, deadline: float) -> bool:
        return clock.monotonic() >= deadline

    clock = FakeClock()
    assert not expired(clock, 11.0)
    clock.now = 11.0
    assert expired(clock, 11.0)


def test_malformed_yaml_traceback_does_not_expose_secret(tmp_path, monkeypatch):
    import traceback

    import yaml

    parse = yaml.safe_load
    monkeypatch.setattr(yaml, "safe_load", lambda stream: parse(stream.read()))

    path = tmp_path / "invalid.yaml"
    path.write_text("secret: private-yaml-token: invalid\n", encoding="utf-8")
    with pytest.raises(Exception) as exc:
        load_settings(SettingsSource(yaml_path=path, environment={}))
    assert "private-yaml-token" not in "".join(traceback.format_exception(exc.value))


@pytest.mark.parametrize(
    "coverage",
    [
        {},
        {"scan": -0.1},
        {"scan": 0},
        {"scan": 6.3},
        {"other": 1.0},
        {"scan": 1.0, "other": 1.0},
        {"scan": float("inf")},
    ],
)
def test_hardware_rejects_missing_or_invalid_coverage(coverage):
    from senior_control.config import DeviceProfile

    with pytest.raises(ValueError):
        DeviceProfile(
            mode="hardware",
            footprint=[(0, 0), (1, 0), (0, 1)],
            frames={"base": "base"},
            topics={"scan": "/scan"},
            sensor_coverage=coverage,
        )


def test_simulation_coverage_optional_and_full_circle_valid():
    from math import pi

    from senior_control.config import DeviceProfile

    assert DeviceProfile().sensor_coverage == {}
    profile = DeviceProfile(topics={"scan": "/scan"}, sensor_coverage={"scan": 2 * pi})
    assert profile.sensor_coverage["scan"] == 2 * pi
    with pytest.raises(ValueError):
        DeviceProfile(sensor_coverage={"scan": 1.0})
