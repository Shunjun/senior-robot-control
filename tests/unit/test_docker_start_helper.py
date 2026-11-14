"""Exercise the container wrapper with real child processes and local sockets."""

import asyncio
import json
import os
import sys
import tempfile
from pathlib import Path

import pytest

from senior_control.runtime.ipc import IpcClient


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
    from senior_control.docker_start_helper import supervise

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
    from senior_control.docker_start_helper import supervise

    result = await supervise(
        service_root, [sys.executable, "-c", "raise SystemExit(78)"], dict(os.environ)
    )
    assert result == 78
    assert not (service_root / "service.sock").exists()


async def test_second_owner_cannot_replace_running_control_socket(service_root):
    from senior_control.docker_start_helper import supervise

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
    from senior_control.cli import load_profile

    setup = root / "setup.bash"
    setup.write_text("")
    (root / "config.json").write_text(
        json.dumps({"kind": "real", "ros_setup": str(setup), "ros_domain": domain})
    )
    credentials = root / "device.json"
    credentials.write_text('{"robot_id":"robot-a","device_token":"private-token"}')
    credentials.chmod(0o600)
    return load_profile(root / "config.json", control_dir=root)


def test_adapter_command_sources_runtime_overlay_without_motion_launch(service_root):
    from senior_control.docker_start_helper import _adapter_command

    profile = real_profile(service_root)
    command = _adapter_command(profile, "robot-a")
    assert 'source "$1"' in command[2]
    assert str(profile.ros_setup) in command
    assert "from senior_robot.agent_adapter import main; main()" in command
    assert "ipc_replace_existing_socket:=false" in command
    assert f"safety_state_path:={service_root / 'estop-state.json'}" in command
    assert "robot_id:=robot-a" in command
    assert not (service_root / "estop-state.json").exists()
    assert "launch" not in command


async def test_active_or_unrecognized_socket_is_never_replaced(service_root):
    import socket

    from senior_control.docker_start_helper import _prepare_socket, _record

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


@pytest.mark.parametrize(("configured_domain", "expected_domain"), [(17, "17"), (None, "42")])
async def test_verified_adapter_precedes_agent_and_agent_stops_first(
    service_root, monkeypatch, configured_domain, expected_domain
):
    from senior_control import docker_start_helper as helper

    profile = real_profile(service_root, domain=configured_domain)
    events = []
    environments = []

    class Child:
        def __init__(self, name):
            self.name = name
            self.returncode = None
            self.done = asyncio.Event()

        async def wait(self):
            await self.done.wait()
            return self.returncode

        def terminate(self):
            events.append(f"stop-{self.name}")
            self.returncode = 0
            self.done.set()

    adapter = Child("adapter")
    agent = Child("agent")

    async def spawn(*args, env):
        environments.append(env)
        if len(environments) == 1:
            events.append("start-adapter")
            return adapter
        events.append("start-agent")
        # Trigger controlled shutdown after both children exist.
        asyncio.get_running_loop().call_soon(stopping.set)
        return agent

    stopping = None

    async def ready(path, robot_id, domain, child, stop):
        nonlocal stopping
        assert robot_id == "robot-a" and domain == configured_domain and child is adapter
        stopping = stop
        events.append("verified")
        return None

    monkeypatch.setattr(helper.asyncio, "create_subprocess_exec", spawn)
    monkeypatch.setattr(helper, "_wait_for_adapter", ready)
    assert (
        await helper.supervise(service_root, ["agent"], {"ROS_DOMAIN_ID": "42"}, profile=profile)
        == 0
    )
    assert events == ["start-adapter", "verified", "start-agent", "stop-agent", "stop-adapter"]
    assert all(env["ROS_DOMAIN_ID"] == expected_domain for env in environments)
    assert not (service_root / "estop-state.json").exists()


@pytest.mark.parametrize(
    ("reply", "error"),
    [({"ok": True, "ros_domain": 18}, "domain mismatch"), ({"ok": False}, "identity mismatch")],
)
async def test_identity_or_domain_mismatch_prevents_agent_readiness(
    service_root, monkeypatch, reply, error
):
    from types import SimpleNamespace

    from senior_control import docker_start_helper as helper

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
    from senior_control.docker_start_helper import _runtime_profile

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


async def test_handshake_failure_reaps_adapter_without_starting_agent(service_root, monkeypatch):
    from senior_control import docker_start_helper as helper

    profile = real_profile(service_root)
    events = []

    class Adapter:
        returncode = None

        def terminate(self):
            events.append("terminate-adapter")
            self.returncode = 0

        async def wait(self):
            return self.returncode

    async def spawn(*args, **kwargs):
        events.append("start-adapter")
        assert events == ["start-adapter"]
        return Adapter()

    async def not_ready(*args):
        raise ValueError("ROS adapter robot identity mismatch")

    monkeypatch.setattr(helper.asyncio, "create_subprocess_exec", spawn)
    monkeypatch.setattr(helper, "_wait_for_adapter", not_ready)
    with pytest.raises(ValueError, match="identity mismatch"):
        await helper.supervise(service_root, ["agent"], {}, profile=profile)
    assert events == ["start-adapter", "terminate-adapter"]
    assert not (service_root / "service.sock").exists()
    assert not (service_root / "estop-state.json").exists()


async def test_matching_robot_id_does_not_establish_process_ownership(service_root):
    import socket

    from senior_control.docker_start_helper import _verify_adapter_peer

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
