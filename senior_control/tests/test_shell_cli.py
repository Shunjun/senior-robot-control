"""Operator boundary checks; never start systemd, Docker or application services."""

import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "bin/senior-robot.sh"


def run_script(*args, env=None):
    return subprocess.run(
        ["/bin/bash", str(SCRIPT), *args],
        capture_output=True,
        text=True,
        env=env,
        timeout=5,
    )


def runtime(tmp_path, monkeypatch):
    trace = tmp_path / "trace"
    python = tmp_path / "runtime python"
    python.write_text("""#!/bin/bash
printf '%s\\n' "$@" >> "$TRACE"
if [[ "$3" == stop && "${STOP_FAIL:-}" == yes ]]; then exit 7; fi
if [[ "$3" == log-route ]]; then printf '{"unit":"senior-sim-test.service","user":true}\\n'; fi
""")
    python.chmod(0o755)
    monkeypatch.setenv("SENIOR_ROBOT_PYTHON", str(python))
    monkeypatch.setenv("TRACE", str(trace))
    return trace


def test_help_and_syntax_work_without_python_or_external_commands():
    env = {"PATH": "/missing", "HOME": "/unused"}
    help_result = run_script("--help", env=env)
    assert help_result.returncode == 0
    assert "docker" in help_result.stdout and "restart" in help_result.stdout
    error = run_script("start", "--headless", "invalid", env=env)
    assert error.returncode == 2
    assert "headless" in error.stderr
    assert "Python" not in error.stderr
    assert run_script("docker", "--help", env=env).returncode == 0


def test_independent_path_install_passes_normalized_arguments_without_evaluation(
    tmp_path, monkeypatch
):
    trace = runtime(tmp_path, monkeypatch)
    installed = tmp_path / "senior-robot"
    shutil.copy2(SCRIPT, installed)
    selected = str(tmp_path / "config with spaces;$(false).json")
    result = subprocess.run(
        ["/bin/bash", str(installed), "start", "--profile", selected, "--headless", "false"],
        capture_output=True,
        text=True,
        timeout=5,
    )
    assert result.returncode == 0, result.stderr
    assert trace.read_text().splitlines() == [
        "-m",
        "senior_control.operator_actions",
        "start",
        "profile",
        selected,
        "false",
        "false",
        "",
        "",
    ]


def test_restart_stops_before_start_and_preserves_stop_failure(tmp_path, monkeypatch):
    trace = runtime(tmp_path, monkeypatch)
    assert run_script("restart").returncode == 0
    calls = trace.read_text().splitlines()
    assert calls[2] == "stop" and calls[11] == "start"
    trace.unlink()
    monkeypatch.setenv("STOP_FAIL", "yes")
    assert run_script("restart").returncode == 7
    assert "start" not in trace.read_text().splitlines()


def test_logs_use_project_user_journal_and_preserve_tool_exit(tmp_path, monkeypatch):
    runtime(tmp_path, monkeypatch)
    journal = tmp_path / "journalctl"
    journal.write_text('#!/bin/bash\nprintf "%s\\n" "$@" > "$JOURNAL_TRACE"\nexit 6\n')
    journal.chmod(0o755)
    monkeypatch.setenv("JOURNAL_TRACE", str(tmp_path / "journal"))
    monkeypatch.setenv("PATH", str(tmp_path) + os.pathsep + os.environ["PATH"])
    result = run_script("logs", "--follow", "--lines", "25")
    assert result.returncode == 6, result.stderr
    assert (tmp_path / "journal").read_text().splitlines() == [
        "--user",
        "--unit=senior-sim-test.service",
        "--no-pager",
        "--lines=25",
        "--follow",
    ]


def docker_fixture(tmp_path, monkeypatch, request, *, running):
    trace = runtime(tmp_path, monkeypatch)
    import tempfile

    directory = tempfile.TemporaryDirectory(prefix="shell-cli-")
    request.addfinalizer(directory.cleanup)
    control = Path(directory.name).resolve()
    config = {
        "kind": "real",
        "project_dir": "/container/project",
        "cloud_url": "wss://example.test/ws/agent",
    }
    (control / "config.json").write_text(json.dumps(config))
    inspection = [
        {
            "State": {"Running": running},
            "Mounts": [
                {"Type": "bind", "Source": str(control), "Destination": str(control), "RW": True},
            ],
        }
    ]
    monkeypatch.setenv("DOCKER_INSPECTION", json.dumps(inspection))
    monkeypatch.setenv("DOCKER_TRACE", str(tmp_path / "docker-trace"))
    docker = tmp_path / "docker"
    docker_source = tmp_path / "docker.py"
    docker_source.write_text("""
import os, sys, json
with open(os.environ["DOCKER_TRACE"], "a") as output:
    output.write(json.dumps(sys.argv[1:]) + "\\n")
if sys.argv[1:] == ["inspect", "ros2"]:
    print(os.environ["DOCKER_INSPECTION"])
else:
    if sys.argv[1:3] == ["exec", "--user"]:
        assert sys.argv[3] == str(os.getuid()) + ":" + str(os.getgid())
        del sys.argv[2:4]
    assert sys.argv[1:5] == ["exec", "-i", "ros2", "/container/project/.venv/bin/python"]
    assert sys.argv[5:] == ["-m", "senior_control.operator_actions", "docker-request"]
    from senior_control.operator_actions import main
    sys.argv = ["operator_actions", "docker-request"]
    raise SystemExit(main())
""")
    import shlex

    docker.write_text(
        "#!/bin/bash\nexec "
        + shlex.quote(sys.executable)
        + " "
        + shlex.quote(str(docker_source))
        + ' "$@"\n'
    )
    docker.chmod(0o755)
    timeout = tmp_path / "timeout"
    timeout.write_text('#!/bin/bash\nshift\nexec "$@"\n')
    timeout.chmod(0o755)
    systemctl = tmp_path / "systemctl"
    systemctl.write_text('#!/bin/bash\nprintf "%s\\n" "$@" >> "$FORBIDDEN_SYSTEMCTL"\nexit 9\n')
    systemctl.chmod(0o755)
    monkeypatch.setenv("FORBIDDEN_SYSTEMCTL", str(tmp_path / "host-systemctl"))
    monkeypatch.setenv("PATH", str(tmp_path) + os.pathsep + os.environ["PATH"])
    # This is a subprocess substitute, not Docker or a running application.
    monkeypatch.setenv(
        "PYTHONPATH",
        os.pathsep.join(
            str(ROOT / path)
            for path in (
                "src",
                "senior_control/src",
                "contracts/src",
                "ros2/src/senior_robot",
                "ros2/src/senior_motion_control",
                "ros2/src/senior_perception",
                "ros2/src/senior_motion_estimation",
                "ros2/src/senior_localization",
                "ros2/src/senior_mapping",
                "ros2/src/senior_navigation",
                "ros2/src/senior_sensor_ingress",
            )
        ),
    )
    return control, trace, tmp_path / "docker-trace"


def test_stopped_docker_preview_and_start_fail_before_exec_or_host_python(
    tmp_path, monkeypatch, request
):
    control, trace, docker_trace = docker_fixture(tmp_path, monkeypatch, request, running=False)
    for args in (("deploy", "--dry-run"), ("start",)):
        result = run_script("docker", *args, "--container", "ros2", "--control-dir", str(control))
        assert result.returncode == 1
        assert "ros2 is stopped" in result.stderr
    assert not trace.exists()
    assert not (tmp_path / "host-systemctl").exists()
    assert [json.loads(line) for line in docker_trace.read_text().splitlines()] == [
        ["inspect", "ros2"]
    ] * 2
    assert sorted(file.name for file in control.iterdir()) == ["config.json"]


def test_running_docker_preview_uses_container_venv_and_original_renderer(
    tmp_path, monkeypatch, request
):
    from senior_control.cli import load_profile
    from senior_control.vehicle_deploy import render_service

    control, trace, docker_trace = docker_fixture(tmp_path, monkeypatch, request, running=True)
    result = run_script("docker", "deploy", "--dry-run", "--control-dir", str(control))
    assert result.returncode == 0, result.stderr
    expected = render_service(
        load_profile(control / "config.json", control_dir=control).model_copy(
            update={"container": "ros2"}
        ),
        str(control.stat().st_uid),
        container_root=control,
        docker_executable=str(tmp_path / "docker"),
        start_container=False,
    )
    assert expected in result.stdout
    assert not (tmp_path / "host-systemctl").exists()
    assert "ExecStartPre=" not in result.stdout
    assert ".venv/bin/python" in result.stdout
    assert not trace.exists()
    assert sorted(file.name for file in control.iterdir()) == ["config.json"]
    calls = [json.loads(line) for line in docker_trace.read_text().splitlines()]
    assert calls[0] == ["inspect", "ros2"]
    assert all(call[0] == "exec" for call in calls[1:])


def test_pair_secret_is_hidden_and_sent_only_on_stdin(tmp_path, monkeypatch):
    import pty
    import select
    import signal
    import termios
    import time

    trace = runtime(tmp_path, monkeypatch)
    python = Path(os.environ["SENIOR_ROBOT_PYTHON"])
    python.write_text("""#!/bin/bash
printf '%s\\n' "$@" >> "$TRACE"
if [[ "$3" == pair ]]; then
    IFS= read -r code
    [[ "$code" == synthetic-one-time-code ]] || exit 8
fi
""")
    pid, terminal = pty.fork()
    if pid == 0:
        os.execve(
            "/bin/bash",
            [
                "bash",
                "-x",
                str(SCRIPT),
                "pair",
                "--cloud-url",
                "https://example.test",
                "--name",
                "test",
            ],
            dict(os.environ),
        )
    output = b""
    sent = False
    status = None
    deadline = time.monotonic() + 5
    try:
        while time.monotonic() < deadline:
            if select.select([terminal], [], [], 0.1)[0]:
                try:
                    output += os.read(terminal, 65536)
                except OSError:
                    break
            if (
                b"Pairing code: " in output
                and not sent
                and not termios.tcgetattr(terminal)[3] & termios.ECHO
            ):
                os.write(terminal, b"synthetic-one-time-code\n")
                sent = True
            ended, result = os.waitpid(pid, os.WNOHANG)
            if ended:
                status = result
                break
        if status is None:
            ended, result = os.waitpid(pid, os.WNOHANG)
            if ended:
                status = result
        assert sent and status == 0, output.decode(errors="replace")
        assert termios.tcgetattr(terminal)[3] & termios.ECHO
        assert b"synthetic-one-time-code" not in output
        assert "synthetic-one-time-code" not in trace.read_text()
        assert "pair-preflight" in trace.read_text() and "pair\n" in trace.read_text()
    finally:
        if status is None:
            os.kill(pid, signal.SIGKILL)
            os.waitpid(pid, 0)
        os.close(terminal)


def test_docker_pair_preserves_operator_identity_and_existing_credentials(
    tmp_path, monkeypatch, request
):
    control, trace, docker_trace = docker_fixture(tmp_path, monkeypatch, request, running=True)
    config = json.loads((control / "config.json").read_text())
    config.pop("cloud_url")  # Original pairing takes its TLS endpoint from --cloud-url.
    (control / "config.json").write_text(json.dumps(config))
    credential = control / "device.json"
    credential.write_bytes(b"existing-private-token")
    credential.chmod(0o600)
    result = run_script(
        "docker",
        "pair",
        "--control-dir",
        str(control),
        "--cloud-url",
        "https://example.test",
        "--name",
        "test",
    )
    assert result.returncode == 1
    assert "Pairing code:" not in result.stderr
    assert "existing-private-token" not in result.stdout + result.stderr
    assert credential.read_bytes() == b"existing-private-token"
    assert not trace.exists() and not (tmp_path / "host-systemctl").exists()
    calls = [json.loads(line) for line in docker_trace.read_text().splitlines()]
    assert calls[0] == ["inspect", "ros2"]
    assert len(calls) == 2 and calls[1][:4] == [
        "exec",
        "--user",
        f"{os.getuid()}:{os.getgid()}",
        "-i",
    ]
    assert sorted(file.name for file in control.iterdir()) == ["config.json", "device.json"]
