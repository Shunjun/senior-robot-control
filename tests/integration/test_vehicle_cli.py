import json
from pathlib import Path

import pytest

from senior_control import cli


@pytest.fixture
def default_profile(tmp_path, monkeypatch):
    monkeypatch.setenv("HOME", str(tmp_path))
    profile = tmp_path / "senior-sim" / "simulation.json"
    profile.parent.mkdir()
    profile.write_text(json.dumps({"kind": "simulation", "credentials": "/test/credentials.json"}))
    return profile


def test_doctor_defaults_to_simulation_profile(default_profile, monkeypatch):
    monkeypatch.setattr(cli, "_doctor", lambda profile: 0 if profile.kind == "simulation" else 1)
    assert cli.main(["doctor"]) == 0


def test_explicit_profile_overrides_simulation_default(default_profile, tmp_path, monkeypatch):
    profile = tmp_path / "real.json"
    profile.write_text(json.dumps({"kind": "real", "credentials": "/test/credentials.json"}))
    monkeypatch.setattr(cli, "_doctor", lambda profile: 0 if profile.kind == "real" else 1)
    assert cli.main(["doctor", "--profile", str(profile)]) == 0


def test_real_profile_requires_explicit_selection(default_profile, monkeypatch, capsys):
    default_profile.write_text(
        json.dumps({"kind": "real", "credentials": "/test/credentials.json"})
    )
    monkeypatch.setattr(cli, "_doctor", lambda _: pytest.fail("real profile was loaded by default"))
    assert cli.main(["doctor"]) == 1
    assert "--profile" in capsys.readouterr().err


@pytest.mark.parametrize("command", ["start", "stop", "status"])
def test_lifecycle_uses_default_simulation_profile(default_profile, monkeypatch, command):
    from senior_control import simulation_cli

    def lifecycle(action, path, profile):
        assert (action, path, profile.kind) == (command, default_profile, "simulation")
        return 0

    monkeypatch.setattr(simulation_cli, "lifecycle", lifecycle)
    assert cli.main([command]) == 0


def test_deploy_requires_explicit_real_profile(default_profile, monkeypatch):
    monkeypatch.setattr(cli, "_install_once", lambda *_: pytest.fail("installed a service"))
    with pytest.raises(SystemExit) as error:
        cli.main(["deploy"])
    assert error.value.code == 2
    assert cli.main(["deploy", "--profile", str(default_profile)]) == 1


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


async def test_opt_in_video_shares_one_private_slot_and_cleans_after_failed_start(
    tmp_path,
    monkeypatch,
):
    import pytest

    from senior_control import simulation_cli

    slots = []

    async def spawn(*args, **kwargs):
        slot = Path(kwargs["env"]["SENIOR_SIMULATION_VIDEO_FRAME"])
        slots.append(slot)
        assert slot.parent.stat().st_mode & 0o777 == 0o700
        if "simulation_motion.launch.py" in args:
            assert f"video_frame_path:={slot}" in args
            assert "video_enabled:=true" in args
            return object()
        raise OSError("agent failed")

    async def verify(*args):
        pass

    async def cleanup(process, timeout):
        pass

    monkeypatch.setattr(simulation_cli.asyncio, "create_subprocess_exec", spawn)
    monkeypatch.setattr(simulation_cli, "_wait_for_adapter", verify)
    monkeypatch.setattr(simulation_cli, "_stop", cleanup)
    profile, credentials = _simulation_profile(tmp_path)
    with pytest.raises(OSError, match="agent failed"):
        await simulation_cli.supervise(profile.model_copy(update={"video": True}), credentials)
    assert len(slots) == 2 and slots[0] == slots[1]
    assert not slots[0].parent.exists()


@pytest.mark.parametrize("headless", ["true", "false"])
def test_start_accepts_explicit_headless(default_profile, monkeypatch, headless):
    from senior_control import simulation_cli

    def lifecycle(action, path, profile, *, headless=True):
        assert (action, path) == ("start", default_profile)
        assert headless is (requested == "true")
        return 0

    requested = headless
    monkeypatch.setattr(simulation_cli, "lifecycle", lifecycle)
    assert cli.main(["start", "--headless", requested]) == 0


def test_start_rejects_invalid_headless(default_profile):
    with pytest.raises(SystemExit) as error:
        cli.main(["start", "--headless", "maybe"])
    assert error.value.code == 2


@pytest.mark.parametrize("headless", [True, False])
def test_simulation_service_receives_runtime_mode_and_desktop(tmp_path, monkeypatch, headless):
    from types import SimpleNamespace

    from senior_control import simulation_cli

    calls = []
    monkeypatch.setattr(simulation_cli.sys, "platform", "linux")
    monkeypatch.setenv("DISPLAY", ":7")
    monkeypatch.setenv("XAUTHORITY", "/tmp/desktop auth")
    monkeypatch.setenv("XDG_RUNTIME_DIR", "/run/user/1000")

    def run(argv, **kwargs):
        calls.append(argv)
        return SimpleNamespace(returncode=1 if "is-active" in argv else 0)

    monkeypatch.setattr(simulation_cli.subprocess, "run", run)
    profile, _ = _simulation_profile(tmp_path)
    assert (
        simulation_cli.lifecycle("start", tmp_path / "profile.json", profile, headless=headless)
        == 0
    )
    assert len(calls) == 2
    service = calls[1]
    assert service[-2:] == ["--headless", str(headless).lower()]
    if headless:
        assert not any(arg.startswith("--setenv=") for arg in service)
    else:
        assert "--setenv=DISPLAY=:7" in service
        assert "--setenv=XAUTHORITY=/tmp/desktop auth" in service
        assert "--setenv=XDG_RUNTIME_DIR=/run/user/1000" in service
        assert service.index("--setenv=DISPLAY=:7") < service.index("--")


def test_windowed_start_requires_display_without_starting_service(tmp_path, monkeypatch, capsys):
    from types import SimpleNamespace

    from senior_control import simulation_cli

    calls = []
    monkeypatch.setattr(simulation_cli.sys, "platform", "linux")
    monkeypatch.delenv("DISPLAY", raising=False)

    def run(argv, **kwargs):
        calls.append(argv)
        return SimpleNamespace(returncode=1)

    monkeypatch.setattr(simulation_cli.subprocess, "run", run)
    profile, _ = _simulation_profile(tmp_path)
    assert (
        simulation_cli.lifecycle("start", tmp_path / "profile.json", profile, headless=False) == 1
    )
    assert len(calls) == 1 and "is-active" in calls[0]
    assert "DISPLAY" in capsys.readouterr().err


def test_windowed_start_does_not_duplicate_running_simulation(tmp_path, monkeypatch):
    from types import SimpleNamespace

    from senior_control import simulation_cli

    calls = []
    monkeypatch.setattr(simulation_cli.sys, "platform", "linux")
    monkeypatch.delenv("DISPLAY", raising=False)

    def run(argv, **kwargs):
        calls.append(argv)
        return SimpleNamespace(returncode=0)

    monkeypatch.setattr(simulation_cli.subprocess, "run", run)
    profile, _ = _simulation_profile(tmp_path)
    assert (
        simulation_cli.lifecycle("start", tmp_path / "profile.json", profile, headless=False) == 0
    )
    assert len(calls) == 1 and "is-active" in calls[0]


@pytest.mark.parametrize("headless", [True, False])
async def test_simulation_launch_uses_requested_window_mode(tmp_path, monkeypatch, headless):
    from senior_control import simulation_cli

    calls = []
    monkeypatch.setenv("DISPLAY", ":7")

    async def spawn(*args, **kwargs):
        calls.append((args, kwargs))
        return object()

    async def verify(*args):
        raise ValueError("stop after inspecting launch")

    async def cleanup(*args):
        pass

    monkeypatch.setattr(simulation_cli.asyncio, "create_subprocess_exec", spawn)
    monkeypatch.setattr(simulation_cli, "_wait_for_adapter", verify)
    monkeypatch.setattr(simulation_cli, "_stop", cleanup)
    profile, credentials = _simulation_profile(tmp_path)
    with pytest.raises(ValueError, match="stop after inspecting launch"):
        await simulation_cli.supervise(profile, credentials, headless=headless)
    assert len(calls) == 1
    argv, options = calls[0]
    assert f"headless:={str(headless).lower()}" in argv
    assert ("xvfb-run" in argv) is headless
    assert options["env"]["DISPLAY"] == ":7"
