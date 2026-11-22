"""Exercise the container wrapper with real child processes and local sockets."""

import asyncio
import json
import os
import signal
import subprocess
import sys
import tempfile
from pathlib import Path

import pytest

from vehicle_core.ipc import IpcClient


@pytest.fixture
def service_root():
    with tempfile.TemporaryDirectory(prefix="senior-service-") as directory:
        yield Path(directory).resolve()


async def wait_for_socket(root, task):
    for _ in range(200):
        if task.done():
            await task
            pytest.fail("service exited before readiness")
        if (root / "service.sock").exists():
            return
        await asyncio.sleep(0.01)
    pytest.fail("service did not become ready")


async def test_control_stop_reaps_owned_child_and_preserves_data(service_root):
    from senior_robot.service_runtime import supervise

    marker = service_root / "agent.db"
    marker.write_text("existing database")
    child = service_root / "child.py"
    child.write_text("import time\ntime.sleep(60)\n")
    task = asyncio.create_task(
        supervise(service_root, [sys.executable, str(child)], dict(os.environ))
    )
    try:
        await wait_for_socket(service_root, task)
        client = IpcClient(service_root / "service.sock")
        try:
            assert (await client.request("status"))["running"] is True
            assert (await client.request("stop"))["ok"] is True
        finally:
            await client.close()
        assert await asyncio.wait_for(task, 5) == 0
        assert not (service_root / "service.sock").exists()
        assert marker.read_text() == "existing database"
    finally:
        if not task.done():
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)


async def test_child_failure_propagates_and_socket_is_removed(service_root):
    from senior_robot.service_runtime import supervise

    result = await supervise(
        service_root, [sys.executable, "-c", "raise SystemExit(78)"], dict(os.environ)
    )
    assert result == 78
    assert not (service_root / "service.sock").exists()


async def test_second_owner_cannot_replace_running_control_socket(service_root):
    from senior_robot.service_runtime import supervise

    task = asyncio.create_task(
        supervise(
            service_root, [sys.executable, "-c", "import time; time.sleep(60)"], dict(os.environ)
        )
    )
    try:
        await wait_for_socket(service_root, task)
        with pytest.raises(BlockingIOError):
            await supervise(
                service_root, [sys.executable, "-c", "raise SystemExit(0)"], dict(os.environ)
            )
        client = IpcClient(service_root / "service.sock")
        try:
            assert (await client.request("stop"))["ok"] is True
        finally:
            await client.close()
        assert await asyncio.wait_for(task, 5) == 0
    finally:
        if not task.done():
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)


def real_profile(root, *, domain=None):
    from vehicle_manager.profile import load_profile

    setup = root / "setup.bash"
    setup.write_text("")
    (root / "config.json").write_text(
        json.dumps(
            {
                "kind": "real",
                "ros_setup": str(setup),
                "ros_domain": domain,
                "vendor_ros_setup": str(setup),
                "motion_profile": str(Path("config/hardware-motion.json").resolve()),
                "safety_parameters": str(Path("config/hardware-safety.yaml").resolve()),
            }
        )
    )
    credentials = root / "device.json"
    credentials.write_text('{"robot_id":"robot-a","device_token":"private-token"}')
    credentials.chmod(0o600)
    return load_profile(root / "config.json", control_dir=root)


def test_adapter_command_sources_runtime_overlay_without_motion_launch(service_root):
    from senior_robot.platform_runtime import _adapter_command

    profile = real_profile(service_root)
    command = _adapter_command(profile, "robot-a")
    assert 'source "$1"' in command[2]
    assert str(profile.ros_setup) in command
    assert "from senior_robot.agent_adapter import main; main()" in command
    assert "ipc_replace_existing_socket:=false" in command
    assert "robot_id:=robot-a" in command
    assert not (service_root / "estop-state.json").exists()
    assert "launch" not in command


def test_motion_launch_uses_project_python_for_env_shebang(service_root):
    from senior_robot.platform_runtime import _motion_command

    profile = real_profile(service_root)
    system_bin = service_root / "system-bin"
    system_bin.mkdir()
    system_python = system_bin / "python3"
    system_python.write_text("#!/bin/sh\nexit 73\n")
    system_python.chmod(0o700)
    profile.ros_setup.write_text(f'export PATH="{system_bin}:$PATH"\n')

    ros2cli = service_root / "ros2cli"
    ros2cli.mkdir()
    (ros2cli / "__init__.py").touch()
    (ros2cli / "cli.py").write_text(
        "import os, subprocess\n"
        "def main():\n"
        "    return subprocess.call([os.environ['SENIOR_TEST_NODE']])\n"
    )
    node = service_root / "safety_node"
    node.write_text("#!/usr/bin/env python3\nimport sys\nprint(sys.executable)\n")
    node.chmod(0o700)

    result = subprocess.run(
        _motion_command(profile, "robot-a"),
        env={**os.environ, "PYTHONPATH": str(service_root), "SENIOR_TEST_NODE": str(node)},
        capture_output=True,
        text=True,
        timeout=10,
    )
    assert result.returncode == 0, result.stderr
    assert Path(result.stdout.strip()).parent == Path(sys.executable).parent


async def test_active_or_unrecognized_socket_is_never_replaced(service_root):
    import socket

    from vehicle_core.ipc import _prepare_socket, _record

    path = service_root / "agent.sock"
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as listener:
        listener.bind(str(path))
        listener.listen()
        _record(path)
        identity = path.stat().st_ino
        with pytest.raises(ValueError, match="active"):
            await _prepare_socket(path)
        assert path.stat().st_ino == identity
    path.with_suffix(".owner.json").unlink()
    with pytest.raises(ValueError, match="unrecognized"):
        await _prepare_socket(path)
    assert path.stat().st_ino == identity
    _record(path)
    await _prepare_socket(path)
    assert not path.exists()


@pytest.mark.parametrize(
    ("reply", "error"),
    [({"ok": True, "ros_domain": 18}, "domain mismatch"), ({"ok": False}, "identity mismatch")],
)
async def test_identity_or_domain_mismatch_prevents_agent_readiness(
    service_root, monkeypatch, reply, error
):
    from types import SimpleNamespace

    import senior_robot.service_runtime as helper

    class Client:
        def __init__(self, *args, **kwargs):
            pass

        async def request(self, kind, **kwargs):
            return reply

        async def close(self):
            pass

    monkeypatch.setattr(helper, "IpcClient", Client)
    with pytest.raises(ValueError, match=error):
        await helper._wait_for_adapter(
            service_root / "agent.sock",
            "robot-a",
            17,
            SimpleNamespace(returncode=None),
            asyncio.Event(),
        )


def test_runtime_profile_maps_explicit_host_paths_without_host_ros(service_root):
    from vehicle_manager.service import _runtime_profile

    (service_root / "config.json").write_text(
        json.dumps(
            {
                "kind": "real",
                "project_dir": "/container/project",
                "control_dir": "/host/control",
                "credentials": "/host/control/device.json",
                "database": "/host/control/agent.db",
            }
        )
    )
    profile = _runtime_profile(service_root)
    assert profile.credentials == service_root / "device.json"
    assert profile.database == service_root / "agent.db"
    assert profile.ros_setup == Path("/container/project/ros2/install/setup.bash")
    assert profile.ros_domain is None


async def test_matching_robot_id_does_not_establish_process_ownership(service_root):
    import socket

    from senior_robot.service_runtime import _verify_adapter_peer

    if not hasattr(socket, "SO_PEERCRED"):
        pytest.skip("Linux Unix peer credentials required")
    path = service_root / "agent.sock"
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as listener:
        listener.bind(str(path))
        listener.listen()
        with pytest.raises(ValueError, match="another process"):
            await _verify_adapter_peer(path, os.getpid() + 1)
        assert path.exists()
        assert not path.with_suffix(".owner.json").exists()


async def test_camera_leaf_recovers_without_restarting_helper_owners(service_root, monkeypatch):
    """Real helper/children; only ROS sourcing and readiness are substituted."""
    import senior_robot.service_runtime as helper

    profile = real_profile(service_root)
    monkeypatch.setenv("PYTHONPATH", str(Path("src").resolve()))
    sleeper = """
import json, os, sys, time
from pathlib import Path
Path(sys.argv[1]).write_text(json.dumps([os.getpid(), os.environ['SENIOR_STARTUP_LEARNING_CLAIM']]))
time.sleep(60)
"""

    def child(name):
        return [sys.executable, "-c", sleeper, str(service_root / name)]

    camera = """
import json, os, resource, signal, sys, time
from pathlib import Path
resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
root = Path(sys.argv[1])
record = [os.getpid(), os.environ['SENIOR_STARTUP_LEARNING_CLAIM'], time.monotonic()]
with (root / 'camera-attempts').open('a') as file:
    file.write(json.dumps(record) + '\\n')
if not os.environ.get('SENIOR_LAUNCH_RECOVERED'):
    os.kill(os.getpid(), signal.SIGSEGV)
"""
    launcher = """
import json, os, shlex, subprocess, sys
from pathlib import Path
root = Path(sys.argv[1])
with (root / 'launch-attempts').open('a') as file:
    file.write(str(os.getpid()) + '\\n')
options = dict(argument.split(':=', 1) for argument in sys.argv[2:])
prefix = shlex.split(options['launch-prefix'])
raise SystemExit(subprocess.call([*prefix, sys.executable, '-c', LEAF_CODE, str(root)]))
""".replace("LEAF_CODE", repr(camera))
    launch_arguments = []

    def launch_command(profile, robot_id, package, filename, *arguments):
        assert (package, filename) == ("turn_on_wheeltec_robot", "wheeltec_camera.launch.py")
        launch_arguments.extend(arguments)
        return [sys.executable, "-c", launcher, str(service_root), *arguments]

    async def video_environment(profile, environment):
        return environment

    async def ready(*args):
        # No ROS adapter or Linux peer credential claim: readiness is outside
        # this camera-owner contract. All four child lifetimes remain real.
        for _ in range(200):
            if (service_root / "adapter").exists():
                return None
            await asyncio.sleep(0.01)
        pytest.fail("adapter fixture did not start")

    monkeypatch.setattr(helper, "_motion_command", lambda *_: child("motion"))
    monkeypatch.setattr(helper, "_adapter_command", lambda *_: child("adapter"))
    monkeypatch.setattr(helper, "_ros_launch_command", launch_command)
    monkeypatch.setattr(helper, "_video_environment", video_environment)
    monkeypatch.setattr(helper, "_wait_for_adapter", ready)
    task = asyncio.create_task(
        helper.supervise(service_root, child("agent"), dict(os.environ), profile=profile)
    )
    try:
        await wait_for_socket(service_root, task)
        for _ in range(500):
            assert not task.done()
            path = service_root / "camera-attempts"
            attempts = (
                [json.loads(line) for line in path.read_text().splitlines()]
                if path.exists()
                else []
            )
            if len(attempts) >= 2:
                break
            await asyncio.sleep(0.02)
        else:
            pytest.fail("abnormal camera leaf did not recover")
        assert len(attempts) == 2
        assert attempts[0][0] != attempts[1][0]
        assert attempts[1][2] - attempts[0][2] >= 2
        owners = [
            json.loads((service_root / name).read_text()) for name in ("agent", "motion", "adapter")
        ]
        assert all(owner[1] == attempts[0][1] == attempts[1][1] for owner in owners)
        for pid, _ in owners:
            os.kill(pid, 0)
        assert any(argument.startswith("launch-prefix:=") for argument in launch_arguments)
        assert (
            "launch-prefix-filter:=^(?:astra_camera_node|usb_cam_node_exe|republish)$"
            in launch_arguments
        )
        await asyncio.sleep(2.1)
        assert len(path.read_text().splitlines()) == 2
        assert len((service_root / "launch-attempts").read_text().splitlines()) == 1
        assert not task.done()
        client = IpcClient(service_root / "service.sock")
        try:
            assert (await client.request("stop"))["ok"] is True
        finally:
            await client.close()
        assert await asyncio.wait_for(task, 8) == 0
        for pid, _ in owners:
            with pytest.raises(ProcessLookupError):
                os.kill(pid, signal.SIGCONT)
    finally:
        if not task.done():
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)
