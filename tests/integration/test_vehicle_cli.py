import json
from pathlib import Path

from senior_control import cli


def test_pair_preserves_existing_credentials(tmp_path: Path, monkeypatch, capsys) -> None:
    credential = tmp_path / "credential.json"
    credential.write_text("existing secret")
    profile = tmp_path / "profile.json"
    profile.write_text(json.dumps({"kind": "simulation", "credentials": str(credential)}))
    monkeypatch.setattr(cli, "getpass", lambda _: (_ for _ in ()).throw(AssertionError()))

    assert (
        cli.main(
            [
                "pair",
                "--profile",
                str(profile),
                "--cloud-url",
                "https://example.test",
                "--name",
                "test",
            ]
        )
        == 1
    )
    assert credential.read_text() == "existing secret"
    assert "existing secret" not in capsys.readouterr().err


def test_pair_prompts_and_uses_tls(tmp_path: Path, monkeypatch) -> None:
    credential = tmp_path / "credential.json"
    profile = tmp_path / "profile.json"
    profile.write_text(json.dumps({"kind": "real", "credentials": str(credential)}))
    monkeypatch.setattr(cli, "getpass", lambda _: "one-time code")

    async def pair(url: str, code: str, name: str, path: Path) -> str:
        assert (url, code, name, path) == (
            "wss://example.test",
            "one-time code",
            "test",
            credential,
        )
        return "robot-a"

    monkeypatch.setattr(cli, "pair_device", pair)
    assert (
        cli.main(
            [
                "pair",
                "--profile",
                str(profile),
                "--cloud-url",
                "https://example.test",
                "--name",
                "test",
            ]
        )
        == 0
    )


def test_doctor_reports_missing_build_without_writing(tmp_path: Path, capsys) -> None:
    credential = tmp_path / "credential.json"
    credential.write_text('{"robot_id":"robot-a","device_token":"secret"}')
    credential.chmod(0o600)
    profile = tmp_path / "profile.json"
    profile.write_text(
        json.dumps(
            {
                "kind": "simulation",
                "credentials": str(credential),
                "ros_setup": str(tmp_path / "install" / "setup.bash"),
                "database": str(tmp_path / "robot.db"),
                "socket": str(tmp_path / "robot.sock"),
            }
        )
    )
    assert cli.main(["doctor", "--profile", str(profile)]) == 1
    output = capsys.readouterr().out
    assert "ROS build" in output
    assert "secret" not in output
    assert sorted(p.name for p in tmp_path.iterdir()) == ["credential.json", "profile.json"]


def test_deploy_dry_run_leaves_files_and_services_untouched(tmp_path: Path, monkeypatch) -> None:
    profile = tmp_path / "unit-a.json"
    profile.write_text(
        json.dumps(
            {
                "kind": "real",
                "credentials": "/var/lib/senior-control/unit-a/credentials.json",
                "database": "/var/lib/senior-control/unit-a/state.db",
                "socket": "/run/senior-control/unit-a/adapter.sock",
                "cloud_url": "wss://example.test/ws/agent",
            }
        )
    )

    def unexpected(*args, **kwargs):
        raise AssertionError("dry run invoked a system mutation")

    monkeypatch.setattr(cli.subprocess, "run", unexpected)
    assert cli.main(["deploy", "--profile", str(profile), "--dry-run"]) == 0
    assert list(tmp_path.iterdir()) == [profile]


async def test_simulation_waits_for_matching_adapter(tmp_path, monkeypatch):
    from senior_control import simulation_cli

    events = []

    async def spawn(*args, **kwargs):
        events.append("ros" if "simulation_motion.launch.py" in args else "agent")
        return object()

    async def verify(*args):
        events.append("identity")
        raise ValueError("foreign adapter")

    async def cleanup(process, timeout):
        events.append("cleanup")

    monkeypatch.setattr(simulation_cli.asyncio, "create_subprocess_exec", spawn)
    monkeypatch.setattr(simulation_cli, "_wait_for_adapter", verify)
    monkeypatch.setattr(simulation_cli, "_stop", cleanup)
    profile, credentials = _simulation_profile(tmp_path)
    import pytest

    with pytest.raises(ValueError, match="foreign adapter"):
        await simulation_cli.supervise(profile, credentials)
    assert events == ["ros", "identity", "cleanup"]


def _simulation_profile(tmp_path):
    from senior_control.agent.pairing import Credentials

    profile = cli.VehicleProfile(
        kind="simulation",
        credentials=tmp_path / "credentials.json",
        ros_setup=tmp_path / "setup.bash",
        database=tmp_path / "state.db",
        socket=tmp_path / "adapter.sock",
        estop_state=tmp_path / "estop.json",
        cloud_url="wss://example.test/ws/agent",
        ros_domain=73,
    )
    return profile, Credentials(robot_id="robot-a", device_token="secret")


def test_restart_preserves_estop_state(tmp_path: Path):
    from senior_control.simulation_cli import _initialize_estop

    path = tmp_path / "estop.json"
    _initialize_estop(path, "robot-a")
    state = json.loads(path.read_text())
    state.update(latched=True, revision=3)
    path.write_text(json.dumps(state))
    _initialize_estop(path, "robot-a")
    assert json.loads(path.read_text()) == state


async def test_failed_start_reaps_only_owned_processes(tmp_path, monkeypatch):
    from senior_control import simulation_cli

    owned_ros = object()
    stopped = []

    async def spawn(*args, **kwargs):
        if "simulation_motion.launch.py" in args:
            return owned_ros
        raise OSError("agent failed to spawn")

    async def verify(*args):
        pass

    async def cleanup(process, timeout):
        stopped.append(process)

    monkeypatch.setattr(simulation_cli.asyncio, "create_subprocess_exec", spawn)
    monkeypatch.setattr(simulation_cli, "_wait_for_adapter", verify)
    monkeypatch.setattr(simulation_cli, "_stop", cleanup)
    profile, credentials = _simulation_profile(tmp_path)
    import pytest

    with pytest.raises(OSError, match="agent failed"):
        await simulation_cli.supervise(profile, credentials)
    assert stopped == [owned_ros]
