"""Own the real-vehicle ROS adapter and Agent, in Docker or a native service.

Only the adapter is started: vendor drivers and motion stacks remain externally managed.
The private stop endpoint also works through docker exec without stopping the container.
"""

import argparse
import asyncio
import fcntl
import json
import os
import signal
import socket
import stat
import struct
import sys
from pathlib import Path
from typing import TYPE_CHECKING, Any

from senior_control.agent.pairing import read_credentials
from senior_control.runtime.ipc import IpcClient, IpcError, IpcServer
from senior_control.vehicle_deploy import private_directory

if TYPE_CHECKING:
    from senior_control.cli import VehicleProfile


def _identity(path: Path) -> list[int]:
    info = path.lstat()
    if not stat.S_ISSOCK(info.st_mode) or info.st_uid != os.geteuid():
        raise ValueError(f"foreign IPC path: {path.name}")
    return [info.st_dev, info.st_ino, info.st_uid]


def _record(path: Path, expected: list[int] | None = None) -> None:
    identity = _identity(path)
    if expected is not None and identity != expected:
        raise ValueError("IPC socket changed before ownership recording")
    marker = path.with_suffix(".owner.json")
    temporary = marker.with_suffix(".tmp")
    # The enclosing private directory and instance lock protect this ownership record.
    fd = os.open(temporary, os.O_CREAT | os.O_TRUNC | os.O_WRONLY | os.O_NOFOLLOW, 0o600)
    with os.fdopen(fd, "w") as file:
        json.dump(identity, file)
    os.replace(temporary, marker)


async def _prepare_socket(path: Path) -> None:
    """Called under the instance lock; never replace a live or unrecognized socket."""
    if not os.path.lexists(path):
        return
    identity = _identity(path)
    probe = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    probe.setblocking(False)
    try:
        await asyncio.wait_for(asyncio.get_running_loop().sock_connect(probe, str(path)), 1)
    except ConnectionRefusedError:
        marker = path.with_suffix(".owner.json")
        try:
            owned = json.loads(marker.read_text())
        except (OSError, ValueError):
            owned = None
        if owned != identity or _identity(path) != identity:
            raise ValueError(f"unrecognized stale IPC socket: {path.name}") from None
        path.unlink()
    else:
        raise ValueError(f"active IPC socket: {path.name}; stop its owner first")
    finally:
        probe.close()


def _runtime_profile(root: Path) -> "VehicleProfile":
    from senior_control.cli import VehicleProfile

    values = json.loads((root / "config.json").read_bytes())
    if not isinstance(values, dict):
        raise ValueError("profile must be an object")
    # //Compatible: a bind-mounted config may contain the host's explicit data paths.
    configured_root = Path(values.get("control_dir") or root)
    if "control_dir" not in values and values.get("credentials"):
        configured_root = Path(values["credentials"]).parent
    for name, filename in (
        ("credentials", "device.json"),
        ("database", "agent.db"),
        ("socket", "agent.sock"),
        ("estop_state", "estop-state.json"),
    ):
        if name in values and Path(values[name]) != configured_root / filename:
            raise ValueError("data paths must stay in the control directory")
        values[name] = root / filename
    values["control_dir"] = root
    if values.get("project_dir") and not values.get("ros_setup"):
        values["ros_setup"] = Path(values["project_dir"]) / "ros2/install/setup.bash"
    return VehicleProfile.model_validate(values)


def _adapter_command(profile: "VehicleProfile", robot_id: str) -> list[str]:
    setup = profile.ros_setup
    if setup is None or not setup.is_absolute() or not setup.is_file():
        raise ValueError("ROS workspace setup is missing; build it and configure ros_setup")
    if profile.ros_domain is not None and not 0 <= profile.ros_domain <= 232:
        raise ValueError("ros_domain must be between 0 and 232")
    assert profile.estop_state is not None
    return [
        "/bin/bash",
        "-c",
        'source "$1" || exit; shift; '
        'if [ -n "${SENIOR_EXPECTED_ROS_DOMAIN:-}" ]; then '
        'export ROS_DOMAIN_ID="$SENIOR_EXPECTED_ROS_DOMAIN"; fi; exec "$@"',
        "senior-vehicle-ros",
        str(setup),
        sys.executable,
        "-c",
        "from senior_robot.agent_adapter import main; main()",
        "--ros-args",
        "-p",
        f"robot_id:={robot_id}",
        "-p",
        f"safety_state_path:={profile.estop_state}",
        "-p",
        "ipc_replace_existing_socket:=false",
    ]


async def _verify_adapter_peer(path: Path, pid: int) -> list[int]:
    """A matching robot ID alone cannot establish ownership of a racing listener."""
    identity = _identity(path)
    if not hasattr(socket, "SO_PEERCRED"):
        raise ValueError("vehicle adapter supervision requires Linux Unix peer credentials")
    probe = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    probe.setblocking(False)
    try:
        await asyncio.wait_for(asyncio.get_running_loop().sock_connect(probe, str(path)), 0.5)
        peer_pid, peer_uid, _ = struct.unpack(
            "3i", probe.getsockopt(socket.SOL_SOCKET, socket.SO_PEERCRED, struct.calcsize("3i"))
        )
        if peer_pid != pid or peer_uid != os.geteuid() or _identity(path) != identity:
            raise ValueError("ROS adapter socket belongs to another process")
    finally:
        probe.close()
    return identity


async def _wait_for_adapter(
    path: Path,
    robot_id: str,
    domain: int | None,
    adapter: asyncio.subprocess.Process,
    stopping: asyncio.Event,
) -> list[int] | None:
    deadline = asyncio.get_running_loop().time() + 35
    while not stopping.is_set():
        if adapter.returncode is not None:
            raise ValueError(
                "ROS adapter exited; check sourced ROS workspace and Python dependencies"
            )
        client = IpcClient(path, timeout=0.5)
        try:
            reply = await client.request("identity", robot_id=robot_id)
        except (IpcError, OSError):
            if asyncio.get_running_loop().time() >= deadline:
                raise ValueError(
                    "ROS adapter readiness timed out; check ROS workspace/dependencies"
                ) from None
            await asyncio.sleep(0.1)
        else:
            if reply.get("ok") is not True:
                raise ValueError("ROS adapter robot identity mismatch")
            if domain is not None and (
                type(reply.get("ros_domain")) is not int or reply["ros_domain"] != domain
            ):
                raise ValueError("ROS adapter domain mismatch")
            identity = await _verify_adapter_peer(path, adapter.pid)
            _record(path, identity)
            if _identity(path) != identity:
                raise ValueError("ROS adapter socket changed during readiness")
            return identity
        finally:
            await client.close()
    return None


async def _terminate(process: asyncio.subprocess.Process | None, timeout: float) -> None:
    if process is None or process.returncode is not None:
        return
    try:
        process.terminate()
    except ProcessLookupError:
        pass
    try:
        await asyncio.wait_for(process.wait(), timeout)
    except asyncio.TimeoutError:
        try:
            process.kill()
        except ProcessLookupError:
            pass
        await process.wait()


async def supervise(
    root: Path,
    command: list[str],
    environment: dict[str, str],
    *,
    profile: "VehicleProfile | None" = None,
) -> int:
    private_directory(root)
    fd = os.open(root / ".agent-service.lock", os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
    with os.fdopen(fd, "wb") as lock:
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        stopping = asyncio.Event()
        process: asyncio.subprocess.Process | None = None
        adapter: asyncio.subprocess.Process | None = None
        adapter_identity: list[int] | None = None
        adapter_command = None
        robot_id = ""
        # //Compatible: direct supervisor callers may still manage a single arbitrary child.
        if profile is not None:
            robot_id = read_credentials(root / "device.json").robot_id
            adapter_command = _adapter_command(profile, robot_id)
        environment = dict(environment)
        environment.update(ROS_HOME=str(root / "ros-home"), ROS_LOG_DIR=str(root / "ros-log"))
        environment.pop("SENIOR_EXPECTED_ROS_DOMAIN", None)
        if profile is not None and profile.ros_domain is not None:
            environment["SENIOR_EXPECTED_ROS_DOMAIN"] = str(profile.ros_domain)
            environment["ROS_DOMAIN_ID"] = str(profile.ros_domain)

        async def handle(request: dict[str, Any]) -> dict[str, Any]:
            if request["type"] == "status":
                return {"ok": True, "running": process is not None and process.returncode is None}
            if request["type"] == "stop":
                stopping.set()
                return {"ok": True}
            return {"ok": False, "error": "unsupported"}

        server = IpcServer(root / "service.sock", handle, replace_existing=False)
        loop = asyncio.get_running_loop()
        signals = (signal.SIGINT, signal.SIGTERM)
        waiters: list[asyncio.Task[Any]] = []
        try:
            await _prepare_socket(root / "service.sock")
            if adapter_command is not None:
                await _prepare_socket(root / "agent.sock")
            await server.start()
            _record(root / "service.sock")
            for number in signals:
                loop.add_signal_handler(number, stopping.set)
            if adapter_command is not None:
                adapter = await asyncio.create_subprocess_exec(*adapter_command, env=environment)
                assert profile is not None
                adapter_identity = await _wait_for_adapter(
                    root / "agent.sock", robot_id, profile.ros_domain, adapter, stopping
                )
            if stopping.is_set():
                return 0
            process = await asyncio.create_subprocess_exec(*command, env=environment)
            exited = asyncio.create_task(process.wait())
            requested = asyncio.create_task(stopping.wait())
            waiters = [exited, requested]
            if adapter is not None:
                waiters.append(asyncio.create_task(adapter.wait()))
            await asyncio.wait(waiters, return_when=asyncio.FIRST_COMPLETED)
            if stopping.is_set():
                return 0
            if adapter is not None and adapter.returncode is not None:
                return 1
            return await exited
        finally:
            try:
                await _terminate(process, 20)
            finally:
                await _terminate(adapter, 5)
            for task in waiters:
                if not task.done():
                    task.cancel()
            await asyncio.gather(*waiters, return_exceptions=True)
            try:
                if adapter_identity is not None:
                    path = root / "agent.sock"
                    if os.path.lexists(path) and _identity(path) == adapter_identity:
                        path.unlink()
            finally:
                await server.close()
                for number in signals:
                    loop.remove_signal_handler(number)


async def stop(root: Path) -> int:
    private_directory(root)
    client = IpcClient(root / "service.sock", timeout=3)
    try:
        reply = await client.request("stop")
        if reply.get("ok") is not True:
            return 1
    finally:
        await client.close()
    # Wait for the owner to reap its child and remove the endpoint before systemd restarts it.
    for _ in range(280):
        if not (root / "service.sock").exists():
            return 0
        await asyncio.sleep(0.1)
    return 1


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("run", "stop"))
    parser.add_argument("--control-dir", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        if args.action == "stop":
            return asyncio.run(stop(args.control_dir))
        profile = _runtime_profile(args.control_dir)
        if profile.kind != "real":
            raise ValueError("real vehicle profile required")
        env = dict(os.environ)
        expected = {
            "SENIOR_CREDENTIALS_FILE": "device.json",
            "SENIOR_DATABASE_PATH": "agent.db",
            "SENIOR_AGENT_SOCKET": "agent.sock",
        }
        for key, name in expected.items():
            if env.get(key) != str(args.control_dir / name):
                raise ValueError("Agent paths must use the selected directory")
        return asyncio.run(
            supervise(
                args.control_dir,
                [sys.executable, "-m", "senior_control.agent.main"],
                env,
                profile=profile,
            )
        )
    except (OSError, ValueError, IpcError) as exc:
        if isinstance(exc, ValueError) and type(exc) is ValueError:
            print(str(exc), file=sys.stderr)
        print(
            f"Vehicle service failed ({type(exc).__name__}). "
            "Check the private directory, configuration and service owner.",
            file=sys.stderr,
        )
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
