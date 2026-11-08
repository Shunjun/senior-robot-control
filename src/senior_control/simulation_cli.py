"""Own a single simulator and Agent within a transient user service."""

import asyncio
import fcntl
import hashlib
import json
import os
import signal
import socket
import stat
import subprocess
import sys
from pathlib import Path
from urllib.parse import urlsplit

from senior_control.agent.pairing import Credentials, read_credentials
from senior_control.cli import VehicleProfile
from senior_control.runtime.ipc import MAX_PATH, IpcClient, IpcError
from senior_control.safety.arbiter import PersistentEstop


def _unit(profile: Path) -> str:
    key = hashlib.sha256(os.fsencode(profile.resolve())).hexdigest()[:24]
    return f"senior-sim-{key}.service"


def lifecycle(command: str, profile: Path, config: VehicleProfile) -> int:
    if sys.platform != "linux":
        raise ValueError("simulation requires Linux")
    unit = _unit(profile)
    if command == "start":
        active = subprocess.run(
            ["systemctl", "--user", "is-active", "--quiet", unit], check=False, timeout=10
        )
        if active.returncode == 0:
            print(f"{unit} is already running.")
            return 0
        argv = [
            "systemd-run",
            "--user",
            "--collect",
            f"--unit={unit}",
            "--property=Type=notify",
            "--property=KillMode=mixed",
            "--property=TimeoutStartSec=45",
            "--property=TimeoutStopSec=30",
            "--property=UMask=0077",
            "--",
            sys.executable,
            "-m",
            "senior_control.simulation_cli",
            str(profile.resolve()),
        ]
    elif command == "status":
        return service_status(unit, config, user=True)
    else:
        argv = ["systemctl", "--user", command, unit]
    return subprocess.run(argv, check=False, timeout=50).returncode


def service_status(unit: str, profile: VehicleProfile, *, user: bool) -> int:
    scope = ["--user"] if user else []
    result = subprocess.run(
        ["systemctl", *scope, "show", "-p", "ActiveState", "-p", "InvocationID", unit],
        check=False,
        capture_output=True,
        text=True,
        timeout=10,
    )
    properties = dict(line.split("=", 1) for line in result.stdout.splitlines() if "=" in line)
    active = properties.get("ActiveState", "unknown")
    print(f"Process: {active}")
    print(f"ROS IPC: {'verified' if asyncio.run(_probe(profile)) else 'unavailable'}")
    invocation = properties.get("InvocationID")
    cloud = "unverified"
    if active == "active" and invocation and invocation != "0" * 32:
        try:
            event = subprocess.run(
                [
                    "journalctl",
                    *scope,
                    "--no-pager",
                    "-n",
                    "1",
                    "-o",
                    "cat",
                    '--grep="event": "connection_(opened|closed)"',
                    f"_SYSTEMD_INVOCATION_ID={invocation}",
                ],
                check=False,
                capture_output=True,
                text=True,
                timeout=10,
            )
            message = json.loads(event.stdout)
            cloud = {
                "connection_opened": "last event connected",
                "connection_closed": "last event disconnected",
            }.get(message.get("event"), "unverified")
        except (OSError, ValueError, AttributeError, subprocess.TimeoutExpired):
            pass
    print(f"Cloud: {cloud}")
    return 0 if active == "active" else 1


async def _probe(profile: VehicleProfile) -> bool:
    if profile.socket is None:
        return False
    try:
        credentials = read_credentials(profile.credentials)
        client = IpcClient(profile.socket)
        try:
            reply = await client.request("identity", robot_id=credentials.robot_id)
            return reply.get("ok") is True
        finally:
            await client.close()
    except (OSError, ValueError, IpcError):
        return False


def _validate(profile: VehicleProfile) -> None:
    root = profile.credentials.parent
    info = root.stat()
    paths = (profile.credentials, profile.database, profile.socket, profile.estop_state)
    if (
        profile.kind != "simulation"
        or not root.is_absolute()
        or root.resolve() != root
        or info.st_uid != os.getuid()
        or info.st_mode & 0o077
        or len(set(paths)) != len(paths)
        or any(path is None or path.parent != root or path.is_symlink() for path in paths)
    ):
        raise ValueError("simulation requires distinct files in a private profile directory")
    if profile.socket is None or len(os.fsencode(profile.socket)) > MAX_PATH:
        raise ValueError("invalid simulator IPC path")
    if profile.ros_setup is None or not (
        profile.ros_setup.is_absolute() and profile.ros_setup.is_file()
    ):
        raise ValueError("ROS build setup is unavailable")
    url = urlsplit(profile.cloud_url or "")
    if url.scheme != "wss" or not url.hostname or url.username or url.password:
        raise ValueError("cloud URL must use authenticated TLS")
    if profile.ros_domain is None or not 1 <= profile.ros_domain <= 232:
        raise ValueError("simulation requires an isolated ROS domain")
    if not stat.S_ISREG(profile.credentials.lstat().st_mode):
        raise ValueError("credentials must be a regular file")


def _initialize_estop(path: Path, robot_id: str) -> None:
    if os.path.lexists(path):
        return
    try:
        PersistentEstop.initialize(path, robot_id)
    except FileExistsError:
        # The concurrent initializer owns the file and its latch.
        pass


async def _wait_for_adapter(
    path: Path, robot_id: str, ros: asyncio.subprocess.Process, stopping: asyncio.Event
) -> None:
    deadline = asyncio.get_running_loop().time() + 35
    while not stopping.is_set() and ros.returncode is None:
        client = IpcClient(path)
        try:
            answer = await client.request("identity", robot_id=robot_id)
        except (IpcError, OSError):
            if asyncio.get_running_loop().time() >= deadline:
                raise ValueError("simulator adapter did not become ready") from None
            await asyncio.sleep(0.1)
        else:
            if answer.get("ok") is not True:
                raise ValueError("foreign simulator adapter")
            return
        finally:
            await client.close()
    raise ValueError("simulator stopped during startup")


def _notify(message: str) -> None:
    address = os.environ.get("NOTIFY_SOCKET")
    if address is None:
        return
    if address.startswith("@"):
        address = "\0" + address[1:]
    with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as notify:
        notify.sendto(message.encode(), address)


async def _stop(process: asyncio.subprocess.Process, timeout: float) -> None:
    if process.returncode is not None:
        return
    try:
        process.send_signal(signal.SIGINT)
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


async def supervise(profile: VehicleProfile, credentials: Credentials) -> int:
    assert profile.socket is not None
    root = profile.credentials.parent
    env = dict(os.environ)
    env.pop("NOTIFY_SOCKET", None)
    env.update(
        ROS_DOMAIN_ID=str(profile.ros_domain),
        IGN_PARTITION=_unit(root),
        ROS_HOME=str(root / "ros-home"),
        ROS_LOG_DIR=str(root / "ros-home" / "log"),
        LIBGL_ALWAYS_SOFTWARE="1",
        SENIOR_CREDENTIALS_FILE=str(profile.credentials),
        SENIOR_CLOUD_URL=str(profile.cloud_url),
        SENIOR_AGENT_SOCKET=str(profile.socket),
        SENIOR_DATABASE_PATH=str(profile.database),
    )
    stop = asyncio.Event()
    loop = asyncio.get_running_loop()
    for signum in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(signum, stop.set)
    ros = None
    agent = None
    try:
        ros = await asyncio.create_subprocess_exec(
            "/bin/bash",
            "-c",
            'source "$1" || exit; shift; exec "$@"',
            "senior-ros",
            str(profile.ros_setup),
            "xvfb-run",
            "-a",
            "ros2",
            "launch",
            "senior_robot",
            "simulation_motion.launch.py",
            "headless:=true",
            f"agent_socket:={profile.socket}",
            f"robot_id:={credentials.robot_id}",
            f"safety_state_path:={profile.estop_state}",
            env=env,
        )
        await _wait_for_adapter(profile.socket, credentials.robot_id, ros, stop)
        if stop.is_set():
            return 0
        agent = await asyncio.create_subprocess_exec(
            sys.executable,
            "-m",
            "senior_control.agent.main",
            env=env,
        )
        _notify("READY=1\nSTATUS=ROS adapter verified. Agent started, cloud status unverified.")
        waiters = [asyncio.create_task(wait) for wait in (stop.wait(), ros.wait(), agent.wait())]
        try:
            await asyncio.wait(waiters, return_when=asyncio.FIRST_COMPLETED)
            return 0 if stop.is_set() else 1
        finally:
            for waiter in waiters:
                waiter.cancel()
            await asyncio.gather(*waiters, return_exceptions=True)
    finally:
        try:
            if agent is not None:
                await _stop(agent, 12)
        finally:
            try:
                if ros is not None:
                    await _stop(ros, 10)
            finally:
                for signum in (signal.SIGINT, signal.SIGTERM):
                    loop.remove_signal_handler(signum)


def main(path: Path) -> int:
    try:
        profile = VehicleProfile.model_validate_json(path.read_bytes())
        _validate(profile)
        credentials = read_credentials(profile.credentials)
        assert profile.estop_state is not None and profile.socket is not None
        lock_path = profile.credentials.parent / ".senior-simulator.lock"
        fd = os.open(lock_path, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
        with os.fdopen(fd, "wb") as lock:
            fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            if os.path.lexists(profile.socket):
                raise ValueError("socket path is already occupied")
            _initialize_estop(profile.estop_state, credentials.robot_id)
            return asyncio.run(supervise(profile, credentials))
    except (OSError, ValueError, IpcError):
        print("Simulation startup failed. Check the profile, build and adapter.", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main(Path(sys.argv[1])))
