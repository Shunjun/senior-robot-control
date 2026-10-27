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
