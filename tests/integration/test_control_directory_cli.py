"""Directory selection, deployment boundaries and container process ownership."""

import json
import tempfile
from pathlib import Path

import pytest

from senior_control import cli


@pytest.fixture
def control_dir():
    with tempfile.TemporaryDirectory(prefix="senior-cli-") as directory:
        yield Path(directory).resolve()


def configure(root, **fields):
    config = {
        "kind": "real",
        "project_dir": "/opt/robot",
        "cloud_url": "wss://example.test/ws/agent",
    }
    config.update(fields)
    (root / "config.json").write_text(json.dumps(config))
    return root / "config.json"


def test_default_directory_reads_config_and_derives_data_paths(control_dir, monkeypatch):
    monkeypatch.setenv("HOME", str(control_dir))
    root = control_dir / "senior-control"
    root.mkdir(mode=0o700)
    configure(root)
    seen = []

    def doctor(profile):
        seen.append(profile)
        return 0

    monkeypatch.setattr(cli, "_doctor", doctor)
    assert cli.main(["doctor"]) == 0
    assert seen[0].credentials == root / "device.json"
    assert seen[0].database == root / "agent.db"
    assert seen[0].socket == root / "agent.sock"
    assert seen[0].estop_state == root / "estop-state.json"


def test_explicit_directory_does_not_read_default_home(control_dir, monkeypatch):
    configure(control_dir)
    monkeypatch.setenv("HOME", "/nonexistent")
    monkeypatch.setattr(cli, "_doctor", lambda p: 0 if p.control_dir == control_dir else 1)
    assert cli.main(["doctor", "--control-dir", str(control_dir)]) == 0


def test_directory_profile_rejects_scattered_data_paths(control_dir, monkeypatch):
    configure(control_dir, database="/elsewhere/agent.db")
    monkeypatch.setattr(cli, "_doctor", lambda _: pytest.fail("accepted scattered data"))
    assert cli.main(["doctor", "--control-dir", str(control_dir)]) == 1


def test_container_flag_defaults_to_ros2(control_dir, monkeypatch):
    configure(control_dir)
    seen = []
    monkeypatch.setattr(cli, "_deploy", lambda p, path, **kw: seen.append(p) or 0)
    assert cli.main(["deploy", "--control-dir", str(control_dir), "--container", "--dry-run"]) == 0
    assert seen[0].container == "ros2"
    assert (
        cli.main(["deploy", "--control-dir", str(control_dir), "--container", "other", "--dry-run"])
        == 0
    )
    assert seen[1].container == "other"


def test_profile_and_directory_cannot_be_selected_together(control_dir):
    config = configure(control_dir)
    with pytest.raises(SystemExit) as error:
        cli.main(["doctor", "--control-dir", str(control_dir), "--profile", str(config)])
    assert error.value.code == 2


def test_native_plan_uses_one_root_and_ros_venv(control_dir):
    from senior_control.vehicle_deploy import render_service

    config = configure(control_dir)
    profile = cli.load_profile(config, control_dir=control_dir)
    unit = render_service(profile, "wheeltec", container_root=None)
    assert "/opt/robot/.ros-venv/bin/python" in unit
    assert "senior_control.agent.main" in unit
    assert f"EnvironmentFile={control_dir}/agent.env" in unit
    assert f"ReadWritePaths={control_dir}" in unit
    assert "ProtectHome=read-only" in unit
    assert "/etc/senior-control" not in unit
    assert "/var/lib/senior-control" not in unit
    assert "docker exec" not in unit


def test_docker_plan_passes_host_env_file_and_stops_only_agent(control_dir):
    from senior_control.vehicle_deploy import environment, render_service

    config = configure(control_dir, container="ros2")
    profile = cli.load_profile(config, control_dir=control_dir)
    unit = render_service(profile, "wheeltec", container_root=Path("/state/senior-control"))
    assert f"--env-file {control_dir}/agent.env" in unit
    assert "/opt/robot/.ros-venv/bin/python" in unit
    assert "senior_control.docker_start_helper run" in unit
    assert "senior_control.docker_start_helper stop" in unit
    assert "docker stop" not in unit
    assert "docker kill" not in unit
    env = environment(profile, Path("/state/senior-control"))
    assert "SENIOR_CREDENTIALS_FILE=/state/senior-control/device.json\n" in env
    assert "SENIOR_DATABASE_PATH=/state/senior-control/agent.db\n" in env
    assert "SENIOR_AGENT_SOCKET=/state/senior-control/agent.sock\n" in env


def test_container_mapping_uses_deepest_bind_and_refuses_unshared_root(control_dir):
    from senior_control.vehicle_deploy import container_directory

    mounts = [
        {"Type": "bind", "Source": str(control_dir.parent), "Destination": "/shared", "RW": True},
        {"Type": "bind", "Source": str(control_dir), "Destination": "/state", "RW": True},
    ]
    assert container_directory(control_dir, mounts) == Path("/state")
    with pytest.raises(ValueError, match="bind"):
        container_directory(Path("/not/shared"), mounts)
    mounts[1]["RW"] = False
    with pytest.raises(ValueError, match="writable"):
        container_directory(control_dir, mounts)


def test_dry_run_prints_plan_without_writing_or_starting(control_dir, monkeypatch, capsys):
    configure(control_dir)
    monkeypatch.setattr(cli.subprocess, "run", lambda *a, **k: pytest.fail("mutated system"))
    assert cli.main(["deploy", "--control-dir", str(control_dir), "--dry-run"]) == 0
    text = capsys.readouterr().out
    assert "/opt/robot/.ros-venv/bin/python" in text
    assert str(control_dir / "agent.env") in text
    assert sorted(p.name for p in control_dir.iterdir()) == ["config.json"]


def test_invalid_container_name_is_rejected_before_external_commands(control_dir, monkeypatch):
    configure(control_dir)
    monkeypatch.setattr(cli.subprocess, "run", lambda *a, **k: pytest.fail("ran external command"))
    assert cli.main(["deploy", "--control-dir", str(control_dir), "--container", "bad/name"]) != 0


def test_native_deploy_writes_only_into_control_root_and_links_unit(control_dir, monkeypatch):
    import os
    from types import SimpleNamespace

    project = control_dir / "project"
    python = project / ".ros-venv/bin/python"
    python.parent.mkdir(parents=True)
    python.write_text("#!/bin/sh\nexit 0\n")
    python.chmod(0o700)
    configure(control_dir, project_dir=str(project))
    credential = control_dir / "device.json"
    credential.write_text('{"robot_id":"unit-a","device_token":"private-token"}')
    credential.chmod(0o600)
    unit_root = control_dir / "systemd"
    unit_root.mkdir()
    calls = []

    def systemctl(argv, **kwargs):
        calls.append(argv)
        return SimpleNamespace(returncode=0)

    monkeypatch.setattr(cli, "UNIT_ROOT", unit_root)
    monkeypatch.setattr(cli.sys, "platform", "linux")
    monkeypatch.setattr(cli.os, "geteuid", lambda: 0)
    monkeypatch.setattr(cli.subprocess, "run", systemctl)
    assert cli.main(["deploy", "--control-dir", str(control_dir)]) == 0
    services = list(control_dir.glob("*.service"))
    assert len(services) == 1
    assert calls == [["systemctl", "link", str(services[0])], ["systemctl", "daemon-reload"]]
    assert f"User={os.getuid()}" in services[0].read_text()
    assert (control_dir / "agent.env").stat().st_mode & 0o777 == 0o600
    assert "private-token" not in (control_dir / "agent.env").read_text()
    assert cli.main(["deploy", "--control-dir", str(control_dir)]) == 1
    assert credential.read_text().endswith('"private-token"}')


def test_docker_dry_run_maps_runtime_paths_without_starting_container(
    control_dir, monkeypatch, capsys
):
    from types import SimpleNamespace

    configure(control_dir)
    calls = []

    def docker(argv, **kwargs):
        calls.append(argv)
        assert argv == ["docker", "inspect", "ros2"]
        return SimpleNamespace(
            stdout=json.dumps(
                [
                    {
                        "Mounts": [
                            {
                                "Type": "bind",
                                "Source": str(control_dir),
                                "Destination": "/runtime",
                                "RW": True,
                            }
                        ]
                    }
                ]
            ),
            returncode=0,
        )

    monkeypatch.setattr(cli.subprocess, "run", docker)
    assert cli.main(["deploy", "--control-dir", str(control_dir), "--container", "--dry-run"]) == 0
    assert len(calls) == 1
    assert "--control-dir /runtime" in capsys.readouterr().out
    assert sorted(p.name for p in control_dir.iterdir()) == ["config.json"]


def test_simulation_worker_reads_unified_configuration(control_dir, monkeypatch):
    from senior_control import simulation_cli

    setup = control_dir / "setup.bash"
    setup.write_text("")
    config = configure(control_dir, kind="simulation", ros_setup=str(setup), ros_domain=88)
    credential = control_dir / "device.json"
    credential.write_text('{"robot_id":"unit-a","device_token":"secret"}')
    credential.chmod(0o600)
    seen = []

    async def supervise(profile, credentials, *, headless):
        seen.append(profile)
        return 0

    monkeypatch.setattr(simulation_cli, "supervise", supervise)
    assert simulation_cli.main(config) == 0
    assert seen[0].socket == control_dir / "agent.sock"
    assert seen[0].database == control_dir / "agent.db"
