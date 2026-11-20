"""Own a single simulator and Agent within a transient user service."""

import asyncio
import errno
import fcntl
import hashlib
import json
import os
import signal
import socket
import stat
import subprocess
import sys
from collections.abc import Callable, Coroutine
from pathlib import Path
from tempfile import TemporaryDirectory
from typing import Any
from urllib.parse import urlsplit

from vehicle_manager.agent.pairing import Credentials
from vehicle_manager.platform_ports import SimulationPorts
from vehicle_manager.profile import VehicleProfile, load_profile


def _unit(profile: Path) -> str:
    key = hashlib.sha256(os.fsencode(profile.resolve())).hexdigest()[:24]
    return f"senior-sim-{key}.service"


class DisplayUnavailableError(ValueError):
    """Windowed simulation has no desktop display in the invoking session."""


_DESKTOP_ENVIRONMENT = (
    "DISPLAY",
    "XAUTHORITY",
    "WAYLAND_DISPLAY",
    "XDG_RUNTIME_DIR",
    "XDG_SESSION_TYPE",
    "QT_QPA_PLATFORM",
    "DBUS_SESSION_BUS_ADDRESS",
)


def _desktop_environment() -> list[str]:
    if not os.environ.get("DISPLAY", "").strip():
        raise DisplayUnavailableError(
            "Windowed simulation requires an available desktop DISPLAY. "
            "Run start from a graphical session or use --headless true."
        )
    return [
        f"--setenv={name}={os.environ[name]}" for name in _DESKTOP_ENVIRONMENT if name in os.environ
    ]


def lifecycle(
    command: str,
    profile: Path,
    config: VehicleProfile,
    *,
    headless: bool = True,
    status: Callable[..., int],
) -> int:
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
        try:
            desktop = [] if headless else _desktop_environment()
        except DisplayUnavailableError as error:
            print(str(error), file=sys.stderr)
            return 1
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
            *desktop,
            "--",
            sys.executable,
            "-m",
            "senior_control.simulation_cli",
            str(profile.resolve()),
            "--headless",
            str(headless).lower(),
        ]
    elif command == "status":
        return status(unit, config, user=True)
    else:
        argv = ["systemctl", "--user", command, unit]
    return subprocess.run(argv, check=False, timeout=50).returncode


def service_status(
    unit: str,
    profile: VehicleProfile,
    *,
    user: bool,
    probe: Callable[..., Coroutine[Any, Any, bool]],
) -> int:
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
    print(f"ROS IPC: {'verified' if asyncio.run(probe(profile)) else 'unavailable'}")
    if (
        profile.kind == "real"
        and profile.motion_profile is None
        and profile.safety_parameters is None
    ):
        print("Motion: unavailable (hardware motion configuration not selected)")
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


async def _probe(
    profile: VehicleProfile, *, require_domain: bool = False, ports: SimulationPorts
) -> bool:
    if profile.socket is None:
        return False
    try:
        credentials = ports.read_credentials(profile.credentials)
        client = ports.client(profile.socket)
        try:
            reply = await client.request("identity", robot_id=credentials.robot_id)
            if reply.get("ok") is not True:
                return False
            # //Compatible: legacy status needs no domain; recovery verifies the live domain.
            if not require_domain:
                return True
            domain = reply.get("ros_domain")
            return type(domain) is int and domain == profile.ros_domain
        finally:
            await client.close()
    except (OSError, ValueError, ports.error):
        return False


def validate_simulation_profile(profile: VehicleProfile, *, max_path: int) -> None:
    root = profile.credentials.parent
    info = root.stat()
    paths = (profile.credentials, profile.database, profile.socket)
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
    if profile.socket is None or len(os.fsencode(profile.socket)) > max_path:
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


async def _wait_for_adapter(
    path: Path,
    robot_id: str,
    ros: asyncio.subprocess.Process,
    stopping: asyncio.Event,
    *,
    ports: SimulationPorts,
) -> None:
    deadline = asyncio.get_running_loop().time() + 35
    while not stopping.is_set() and ros.returncode is None:
        client = ports.client(path)
        try:
            answer = await client.request("identity", robot_id=robot_id)
        except (ports.error, OSError):
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


async def supervise(
    profile: VehicleProfile,
    credentials: Credentials,
    *,
    headless: bool = True,
    ports: SimulationPorts,
) -> int:
    assert profile.socket is not None
    if not headless:
        _desktop_environment()
    root = profile.credentials.parent
    env = ports.start_environment(dict(os.environ), root)
    learning_service = ports.learning(Path(env["SENIOR_STARTUP_LEARNING_CLAIM"]))
    env.pop("NOTIFY_SOCKET", None)
    env.pop("SENIOR_SIMULATION_VIDEO_FRAME", None)
    env.pop("SENIOR_VERIFIED_ADAPTER_RUN", None)
    env.pop("SENIOR_EXPECTED_ROS_DOMAIN", None)
    camera = TemporaryDirectory(prefix=".camera-", dir=root) if profile.video else None
    video_args: list[str] = []
    if camera is not None:
        frame = Path(camera.name) / "rgb.frame"
        env["SENIOR_SIMULATION_VIDEO_FRAME"] = str(frame)
        video_args = ["video_enabled:=true", f"video_frame_path:={frame}"]
    env.update(
        ROS_DOMAIN_ID=str(profile.ros_domain),
        ROS_LOCALHOST_ONLY="1",
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
    events = ports.events()
    ros = None
    agent = None
    startup = None
    waiters: dict[asyncio.Task[Any], str] = {}
    socket_path = profile.socket

    async def start_local() -> None:
        nonlocal ros
        try:
            events.report("ros", "starting")
            ros = await asyncio.create_subprocess_exec(
                "/bin/bash",
                "-c",
                'source "$1" || exit; shift; exec "$@"',
                "senior-ros",
                str(profile.ros_setup),
                *(["xvfb-run", "-a"] if headless else []),
                "ros2",
                "launch",
                "senior_robot",
                "simulation_motion.launch.py",
                f"headless:={str(headless).lower()}",
                "navigation_enabled:=true",
                *video_args,
                f"agent_socket:={profile.socket}",
                f"robot_id:={credentials.robot_id}",
                env=events.child_environment(env),
                start_new_session=True,
                pass_fds=(events.writer,),
            )
            assert socket_path is not None
            await ports.wait_for_adapter(socket_path, credentials.robot_id, ros, stop)
        except (OSError, ValueError, ports.error) as exc:
            events.report("ros", "unavailable", str(exc))
            if ros is not None:
                await ports.terminate_motion_stack(ros)
                ros = None
            return
        if stop.is_set():
            return
        assert ros is not None
        waiters[asyncio.create_task(ros.wait())] = "ros"
        await asyncio.sleep(0)
        assert agent is not None
        if ros.returncode is None and agent.returncode is None and not stop.is_set():
            ports.cleanup_learning(
                root, env["SENIOR_STARTUP_LEARNING_CLAIM"], peer_lock_name=".agent-service.lock"
            )
        events.report("ros", "ready", "adapter verified; cloud status independent")

    try:
        await learning_service.start()
        agent = await asyncio.create_subprocess_exec(
            sys.executable,
            "-m",
            "senior_control.agent.main",
            env=events.agent_environment(env),
            pass_fds=(events.reader,),
        )
        events.close_reader()
        if agent.returncode is not None:
            return await agent.wait()
        ports.notify("READY=1\nSTATUS=Agent started, cloud and ROS status unverified.")
        startup = asyncio.create_task(start_local())
        waiters = {
            asyncio.create_task(stop.wait()): "stop",
            asyncio.create_task(agent.wait()): "agent",
            startup: "startup",
        }
        while True:
            done, _ = await asyncio.wait(waiters, return_when=asyncio.FIRST_COMPLETED)
            if stop.is_set():
                return 0
            if agent.returncode is not None:
                return await agent.wait()
            for waiter in done:
                name = waiters.pop(waiter)
                result = waiter.result()
                if name == "agent":
                    return int(result)
                if name == "ros":
                    events.report("ros", "exited", returncode=result)
                    await ports.terminate_motion_stack(ros)
                    ros = None
    finally:
        if startup is not None and not startup.done():
            startup.cancel()
            await asyncio.gather(startup, return_exceptions=True)
        for waiter in waiters:
            if not waiter.done():
                waiter.cancel()
        await asyncio.gather(*waiters, return_exceptions=True)
        try:
            if agent is not None:
                await ports.stop(agent, 12)
        finally:
            try:
                if ros is not None:
                    await ports.terminate_motion_stack(ros)
            finally:
                events.close()
                for signum in (signal.SIGINT, signal.SIGTERM):
                    loop.remove_signal_handler(signum)
                if camera is not None:
                    camera.cleanup()
                await learning_service.close()


def _remove_stale_socket(path: Path) -> None:
    """Remove an owned, refused socket only while both simulation locks are held."""
    try:
        original = path.lstat()
    except FileNotFoundError:
        return
    if not stat.S_ISSOCK(original.st_mode) or original.st_uid != os.getuid():
        raise ValueError("simulator IPC path is not an owned socket")
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as probe:
        probe.setblocking(False)
        result = probe.connect_ex(os.fspath(path))
    if result != errno.ECONNREFUSED:
        raise ValueError("simulator IPC socket is live or cannot be safely checked")
    current = path.lstat()
    if (
        not stat.S_ISSOCK(current.st_mode)
        or current.st_uid != os.getuid()
        or (current.st_dev, current.st_ino) != (original.st_dev, original.st_ino)
    ):
        raise ValueError("simulator IPC socket changed during stale check")
    path.unlink()


def main(path: Path, *, headless: bool = True, ports: SimulationPorts) -> int:
    try:
        profile = load_profile(path)
        ports.validate_profile(profile)
        credentials = ports.read_credentials(profile.credentials)
        assert profile.socket is not None
        from contextlib import ExitStack

        lock_paths = (
            Path(f"/tmp/senior-simulator-ros-{profile.ros_domain}.lock"),
            profile.credentials.parent / ".senior-simulator.lock",
        )
        with ExitStack() as stack:
            for lock_path in lock_paths:
                fd = os.open(lock_path, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
                lock = stack.enter_context(os.fdopen(fd, "wb"))
                fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            ports.remove_stale_socket(profile.socket)
            return asyncio.run(ports.supervise(profile, credentials, headless=headless))
    except DisplayUnavailableError as error:
        print(str(error), file=sys.stderr)
        return 1
    except (OSError, ValueError, ports.error):
        print("Simulation startup failed. Check the profile, build and adapter.", file=sys.stderr)
        return 1
