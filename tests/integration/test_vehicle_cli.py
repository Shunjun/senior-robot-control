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


@pytest.mark.parametrize("explicit", [False, True])
def test_requalify_motion_selects_simulation_profile(default_profile, monkeypatch, explicit):
    from senior_control import simulation_cli

    calls = []

    def requalify(path, profile):
        calls.append((path, profile.kind))
        return 0

    monkeypatch.setattr(simulation_cli, "requalify_motion", requalify)
    argv = ["requalify-motion"]
    if explicit:
        argv += ["--profile", str(default_profile)]
    assert cli.main(argv) == 0
    assert calls == [(default_profile, "simulation")]


@pytest.fixture
def requalification_runtime(tmp_path, monkeypatch):
    from types import SimpleNamespace

    from senior_control import simulation_cli

    profile, _ = _simulation_profile(tmp_path)
    calls = []
    probes = []
    validations = []

    def validate(candidate):
        validations.append(candidate)

    async def probe(candidate, *, require_domain=False):
        assert require_domain is True
        probes.append(candidate)
        return True

    def run(argv, **kwargs):
        calls.append((argv, kwargs))
        return SimpleNamespace(returncode=0, stdout=b"", stderr=b"")

    monkeypatch.setattr(simulation_cli.sys, "platform", "linux")
    monkeypatch.setattr(simulation_cli, "validate_simulation_profile", validate)
    monkeypatch.setattr(simulation_cli, "_probe", probe)
    monkeypatch.setattr(simulation_cli.subprocess, "run", run)
    return profile, calls, probes, validations


def test_requalify_motion_only_arms_existing_service(requalification_runtime, tmp_path, capsys):
    from senior_control import simulation_cli

    profile, calls, probes, validations = requalification_runtime
    path = tmp_path / "simulation.json"
    assert simulation_cli.requalify_motion(path, profile) == 0
    assert validations == [profile] and probes == [profile]
    assert len(calls) == 2
    assert calls[0][0] == [
        "systemctl",
        "--user",
        "is-active",
        "--quiet",
        simulation_cli._unit(path),
    ]
    argv, options = calls[1]
    assert argv[:2] == ["/bin/bash", "-c"]
    assert argv[4:8] == [str(profile.ros_setup), "73", "/usr/bin/python3", "-c"]
    assert 'export ROS_DOMAIN_ID="$1" ROS_LOCALHOST_ONLY=1' in argv[2]
    assert "/motion_fusion/requalify_progress" in argv[-1]
    assert "std_srvs.srv import Trigger" in argv[-1]
    assert "timeout_sec=5.0" in argv[-1] and "timeout_sec=10.0" in argv[-1]
    assert options["env"]["ROS_DOMAIN_ID"] == "73"
    assert options["env"]["ROS_LOCALHOST_ONLY"] == "1"
    assert options["timeout"] == 25 and options["capture_output"] is True
    output = capsys.readouterr()
    assert "accepted (armed), not yet qualified" in output.out
    assert "requested/applied zero" in output.out and "two contiguous stationary" in output.out
    assert output.err == ""


def test_requalify_motion_rejects_real_profile(requalification_runtime, tmp_path, capsys):
    from senior_control import simulation_cli

    profile, calls, probes, validations = requalification_runtime
    assert (
        simulation_cli.requalify_motion(
            tmp_path / "real.json", profile.model_copy(update={"kind": "real"})
        )
        == 1
    )
    assert calls == [] and probes == [] and validations == []
    assert "only for simulation" in capsys.readouterr().err


def test_requalify_motion_validates_before_service_access(tmp_path, monkeypatch, capsys):
    from senior_control import simulation_cli

    profile, _ = _simulation_profile(tmp_path)
    monkeypatch.setattr(simulation_cli.sys, "platform", "linux")
    monkeypatch.setattr(
        simulation_cli.subprocess, "run", lambda *a, **k: pytest.fail("ran service")
    )
    assert simulation_cli.requalify_motion(tmp_path / "simulation.json", profile) == 1
    assert "Check the simulation profile" in capsys.readouterr().err


@pytest.mark.parametrize("missing", ["service", "adapter", "identity_error", "ipc_timeout"])
def test_requalify_motion_requires_live_service_and_adapter(
    requalification_runtime,
    tmp_path,
    monkeypatch,
    capsys,
    missing,
):
    from types import SimpleNamespace

    from senior_control import simulation_cli

    profile, calls, probes, _ = requalification_runtime
    if missing == "service":

        def run(argv, **kwargs):
            calls.append((argv, kwargs))
            return SimpleNamespace(returncode=3)

        monkeypatch.setattr(simulation_cli.subprocess, "run", run)
    else:

        async def probe(candidate, *, require_domain=False):
            probes.append(candidate)
            if missing == "identity_error":
                raise simulation_cli.IpcError("secret must not appear")
            if missing == "ipc_timeout":
                raise TimeoutError("secret must not appear")
            return False

        monkeypatch.setattr(simulation_cli, "_probe", probe)
    assert simulation_cli.requalify_motion(tmp_path / "simulation.json", profile) == 1
    assert len(calls) == 1
    assert (len(probes) == 0) is (missing == "service")
    output = capsys.readouterr()
    assert output.out == "" and "secret" not in output.err


@pytest.mark.parametrize("failure", ["rejected", "timeout", "unavailable"])
def test_requalify_motion_request_failure_is_bounded_and_private(
    requalification_runtime,
    tmp_path,
    monkeypatch,
    capsys,
    failure,
):
    import subprocess
    from types import SimpleNamespace

    from senior_control import simulation_cli

    profile, calls, _, _ = requalification_runtime

    def run(argv, **kwargs):
        calls.append((argv, kwargs))
        if "systemctl" in argv:
            return SimpleNamespace(returncode=0)
        if failure == "timeout":
            raise subprocess.TimeoutExpired(argv, kwargs["timeout"], output=b"secret")
        if failure == "unavailable":
            raise OSError("secret")
        return SimpleNamespace(returncode=1, stdout=b"secret", stderr=b"secret")

    monkeypatch.setattr(simulation_cli.subprocess, "run", run)
    assert simulation_cli.requalify_motion(tmp_path / "simulation.json", profile) == 1
    assert len(calls) == 2
    output = capsys.readouterr()
    assert output.out == "" and "secret" not in output.err
    assert "failed" in output.err


@pytest.mark.parametrize(
    ("reason", "explanation"),
    [
        ("progress_history_not_broken", "no requalification was armed"),
        ("motion_anomaly_latched", "cannot clear it"),
        ("stationary_profile_unqualified", "calibration is unavailable"),
        ("clock_unqualified", "restore valid clock"),
    ],
)
def test_requalify_motion_reports_allowlisted_rejections(
    requalification_runtime,
    tmp_path,
    monkeypatch,
    capsys,
    reason,
    explanation,
):
    from types import SimpleNamespace

    from senior_control import simulation_cli

    profile, _, _, _ = requalification_runtime

    def run(argv, **kwargs):
        if "systemctl" in argv:
            return SimpleNamespace(returncode=0)
        return SimpleNamespace(returncode=1, stdout=f"{reason}\n".encode(), stderr=b"secret")

    monkeypatch.setattr(simulation_cli.subprocess, "run", run)
    assert simulation_cli.requalify_motion(tmp_path / "simulation.json", profile) == 1
    output = capsys.readouterr()
    assert output.out == ""
    assert reason in output.err and explanation in output.err
    assert "secret" not in output.err and "accepted" not in output.err


@pytest.mark.parametrize(
    "untrusted",
    [b"secret\nclock_unqualified", b"motion anomaly remains latched", b"\xffsecret"],
)
def test_requalify_motion_filters_non_allowlisted_child_output(
    requalification_runtime,
    tmp_path,
    monkeypatch,
    capsys,
    untrusted,
):
    from types import SimpleNamespace

    from senior_control import simulation_cli

    profile, _, _, _ = requalification_runtime

    def run(argv, **kwargs):
        if "systemctl" in argv:
            return SimpleNamespace(returncode=0)
        return SimpleNamespace(returncode=1, stdout=untrusted, stderr=b"secret traceback")

    monkeypatch.setattr(simulation_cli.subprocess, "run", run)
    assert simulation_cli.requalify_motion(tmp_path / "simulation.json", profile) == 1
    output = capsys.readouterr()
    assert output.out == ""
    assert "failed or was rejected" in output.err
    assert "secret" not in output.err and "traceback" not in output.err


@pytest.mark.parametrize(
    ("message", "identifier"),
    [
        ("progress history is not broken", "progress_history_not_broken"),
        ("motion anomaly remains latched", "motion_anomaly_latched"),
        ("stationary profile unqualified", "stationary_profile_unqualified"),
        ("clock unqualified", "clock_unqualified"),
        ("unknown secret reason", ""),
    ],
)
def test_requalification_helper_only_emits_approved_identifiers(
    monkeypatch,
    capsys,
    message,
    identifier,
):
    import sys
    from types import SimpleNamespace

    from senior_control import simulation_cli

    cleanup = []
    future = SimpleNamespace(
        done=lambda: True,
        cancelled=lambda: False,
        exception=lambda: None,
        result=lambda: SimpleNamespace(success=False, message=message),
    )
    client = SimpleNamespace(wait_for_service=lambda **_: True, call_async=lambda _: future)
    node = SimpleNamespace(
        create_client=lambda *_: client,
        destroy_node=lambda: cleanup.append("node"),
    )
    monkeypatch.setitem(
        sys.modules,
        "rclpy",
        SimpleNamespace(
            init=lambda **_: None,
            create_node=lambda _: node,
            spin_until_future_complete=lambda *_, **__: None,
            shutdown=lambda: cleanup.append("shutdown"),
        ),
    )
    monkeypatch.setitem(sys.modules, "std_srvs", SimpleNamespace())
    monkeypatch.setitem(
        sys.modules,
        "std_srvs.srv",
        SimpleNamespace(
            Trigger=SimpleNamespace(Request=lambda: object()),
        ),
    )
    with pytest.raises(SystemExit) as error:
        exec(simulation_cli._REQUALIFY_MOTION, {})
    assert error.value.code == 1
    output = capsys.readouterr()
    assert output.out.strip() == identifier and output.err == ""
    assert cleanup == ["node", "shutdown"]


@pytest.mark.parametrize(
    ("reported_domain", "configured_domain", "accepted"),
    [
        (73, 73, True),
        (74, 73, False),
        (None, 73, False),
        (True, 1, False),
        (73.0, 73, False),
        ("73", 73, False),
    ],
)
def test_requalification_verifies_live_adapter_domain_before_service(
    tmp_path,
    monkeypatch,
    capsys,
    reported_domain,
    configured_domain,
    accepted,
):
    from types import SimpleNamespace

    from senior_control import simulation_cli

    profile, credentials = _simulation_profile(tmp_path)
    profile = profile.model_copy(update={"ros_domain": configured_domain})
    calls = []
    closed = []

    class Client:
        def __init__(self, path):
            assert path == profile.socket

        async def request(self, operation, **kwargs):
            assert operation == "identity" and kwargs == {"robot_id": credentials.robot_id}
            reply = {"ok": True}
            if reported_domain is not None:
                reply["ros_domain"] = reported_domain
            return reply

        async def close(self):
            closed.append(True)

    def run(argv, **kwargs):
        calls.append(argv)
        return SimpleNamespace(returncode=0, stdout=b"", stderr=b"")

    monkeypatch.setattr(simulation_cli.sys, "platform", "linux")
    monkeypatch.setattr(simulation_cli, "validate_simulation_profile", lambda _: None)
    monkeypatch.setattr(simulation_cli, "read_credentials", lambda _: credentials)
    monkeypatch.setattr(simulation_cli, "IpcClient", Client)
    monkeypatch.setattr(simulation_cli.subprocess, "run", run)
    assert simulation_cli.requalify_motion(tmp_path / "simulation.json", profile) == (
        0 if accepted else 1
    )
    assert len(calls) == (2 if accepted else 1)
    assert closed == [True]
    output = capsys.readouterr()
    if not accepted:
        assert output.out == "" and "configured ROS domain" in output.err


async def test_status_probe_preserves_legacy_adapter_identity(tmp_path, monkeypatch):
    from senior_control import simulation_cli

    profile, credentials = _simulation_profile(tmp_path)

    class Client:
        def __init__(self, path):
            pass

        async def request(self, *args, **kwargs):
            return {"ok": True}

        async def close(self):
            pass

    monkeypatch.setattr(simulation_cli, "read_credentials", lambda _: credentials)
    monkeypatch.setattr(simulation_cli, "IpcClient", Client)
    assert await simulation_cli._probe(profile) is True


@pytest.fixture
def short_socket_root(monkeypatch):
    import errno
    import stat
    from tempfile import TemporaryDirectory
    from types import SimpleNamespace

    from senior_control import simulation_cli

    original_lstat = Path.lstat
    original_socket = simulation_cli.socket.socket

    # The local test environment prohibits AF_UNIX bind. Preserve real inode races while
    # modeling socket metadata and connect results, without starting a listener.
    def socket_lstat(path, *args, **kwargs):
        info = original_lstat(path, *args, **kwargs)
        if path.suffix == ".sock" and stat.S_ISREG(info.st_mode):
            return SimpleNamespace(
                st_mode=stat.S_IFSOCK | 0o600,
                st_uid=info.st_uid,
                st_dev=info.st_dev,
                st_ino=info.st_ino,
            )
        return info

    class Probe:
        def __enter__(self):
            return self

        def __exit__(self, *args):
            pass

        def setblocking(self, value):
            assert value is False

        def connect_ex(self, address):
            return errno.ECONNREFUSED

    monkeypatch.setattr(Path, "lstat", socket_lstat)
    monkeypatch.setattr(
        simulation_cli.socket,
        "socket",
        lambda *args, **kwargs: (
            original_socket(*args, **kwargs) if "fileno" in kwargs or len(args) > 3 else Probe()
        ),
    )
    with TemporaryDirectory(prefix="sim-socket-", dir="/tmp") as directory:
        yield Path(directory)


def _stale_test_socket(path):
    path.touch()


def test_stale_socket_is_removed_and_missing_path_is_allowed(short_socket_root):
    from senior_control import simulation_cli

    path = short_socket_root / "adapter.sock"
    simulation_cli._remove_stale_socket(path)
    _stale_test_socket(path)
    simulation_cli._remove_stale_socket(path)
    assert not path.exists()


def test_live_socket_is_preserved(short_socket_root, monkeypatch):
    from senior_control import simulation_cli

    path = short_socket_root / "adapter.sock"
    _stale_test_socket(path)
    probe = simulation_cli.socket.socket()
    monkeypatch.setattr(probe, "connect_ex", lambda _: 0)
    monkeypatch.setattr(simulation_cli.socket, "socket", lambda *args: probe)
    identity = path.lstat()
    with pytest.raises(ValueError, match="live or cannot"):
        simulation_cli._remove_stale_socket(path)
    assert path.lstat().st_ino == identity.st_ino


@pytest.mark.parametrize("kind", ["regular", "symlink", "foreign_owner"])
def test_stale_cleanup_rejects_unowned_or_non_socket_paths(
    short_socket_root,
    monkeypatch,
    kind,
):
    from senior_control import simulation_cli

    path = short_socket_root / "adapter.sock"
    if kind == "regular":
        path = short_socket_root / "adapter.txt"
        path.write_text("preserve me")
    elif kind == "symlink":
        target = short_socket_root / "target.sock"
        _stale_test_socket(target)
        path.symlink_to(target)
    else:
        _stale_test_socket(path)
        actual_uid = path.lstat().st_uid
        monkeypatch.setattr(simulation_cli.os, "getuid", lambda: actual_uid + 1)
    original = path.lstat()
    with pytest.raises(ValueError, match="not an owned socket"):
        simulation_cli._remove_stale_socket(path)
    assert path.lstat().st_ino == original.st_ino


@pytest.mark.parametrize("outcome", ["permission", "timeout", "pending", "again", "missing"])
def test_stale_cleanup_preserves_uncertain_socket(short_socket_root, monkeypatch, outcome):
    import errno

    from senior_control import simulation_cli

    path = short_socket_root / "adapter.sock"
    _stale_test_socket(path)
    original = path.lstat()
    errors = {
        "permission": errno.EACCES,
        "timeout": errno.ETIMEDOUT,
        "pending": errno.EINPROGRESS,
        "again": errno.EAGAIN,
        "missing": errno.ENOENT,
    }

    class Probe:
        def __enter__(self):
            return self

        def __exit__(self, *args):
            pass

        def setblocking(self, value):
            assert value is False

        def connect_ex(self, address):
            assert address == str(path)
            return errors[outcome]

    monkeypatch.setattr(simulation_cli.socket, "socket", lambda *args: Probe())
    with pytest.raises(ValueError, match="cannot be safely checked"):
        simulation_cli._remove_stale_socket(path)
    assert path.lstat().st_ino == original.st_ino


def test_stale_cleanup_preserves_socket_replaced_during_probe(short_socket_root, monkeypatch):
    import errno

    from senior_control import simulation_cli

    path = short_socket_root / "adapter.sock"
    replacement = short_socket_root / "replacement.sock"
    _stale_test_socket(path)
    _stale_test_socket(replacement)
    replacement_identity = replacement.lstat()

    class Probe:
        def __enter__(self):
            return self

        def __exit__(self, *args):
            pass

        def setblocking(self, value):
            pass

        def connect_ex(self, address):
            replacement.replace(path)
            return errno.ECONNREFUSED

    monkeypatch.setattr(simulation_cli.socket, "socket", lambda *args: Probe())
    with pytest.raises(ValueError, match="changed during stale check"):
        simulation_cli._remove_stale_socket(path)
    assert path.lstat().st_ino == replacement_identity.st_ino


@pytest.mark.parametrize("lock_failure", [False, True])
def test_main_only_cleans_stale_socket_after_both_locks(
    short_socket_root,
    monkeypatch,
    lock_failure,
):
    import os

    from senior_control import simulation_cli

    profile, credentials = _simulation_profile(short_socket_root)
    path = short_socket_root / "simulation.json"
    path.write_text(profile.model_dump_json())
    _stale_test_socket(profile.socket)
    events = []
    original_open = os.open
    original_cleanup = simulation_cli._remove_stale_socket

    def open_lock(lock_path, flags, mode=0o777, *, dir_fd=None):
        if str(lock_path).startswith("/tmp/senior-simulator-ros-"):
            lock_path = short_socket_root / "domain.lock"
        return original_open(lock_path, flags, mode, dir_fd=dir_fd)

    def lock(fd, operation):
        events.append("lock")
        if lock_failure and len(events) == 2:
            raise BlockingIOError("test lock is busy")

    def cleanup(socket_path):
        assert events == ["lock", "lock"]
        events.append("cleanup")
        original_cleanup(socket_path)

    async def supervise(*args, **kwargs):
        events.append("supervise")
        assert not profile.socket.exists()
        return 0

    monkeypatch.setattr(simulation_cli, "validate_simulation_profile", lambda _: None)
    monkeypatch.setattr(simulation_cli, "read_credentials", lambda _: credentials)
    monkeypatch.setattr(simulation_cli, "_initialize_estop", lambda *_: None)
    monkeypatch.setattr(simulation_cli.os, "open", open_lock)
    monkeypatch.setattr(simulation_cli.fcntl, "flock", lock)
    monkeypatch.setattr(simulation_cli, "_remove_stale_socket", cleanup)
    monkeypatch.setattr(simulation_cli, "supervise", supervise)
    assert simulation_cli.main(path) == (1 if lock_failure else 0)
    assert events == (
        ["lock", "lock"] if lock_failure else ["lock", "lock", "cleanup", "supervise"]
    )
    assert profile.socket.exists() is lock_failure
