"""Own a single simulator and Agent within a transient user service."""

import argparse
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
from pathlib import Path
from tempfile import TemporaryDirectory
from urllib.parse import urlsplit

from senior_control.agent.pairing import Credentials, read_credentials
from senior_control.cli import VehicleProfile
from senior_control.runtime.ipc import MAX_PATH, IpcClient, IpcError
from senior_control.safety.arbiter import PersistentEstop


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


def lifecycle(command: str, profile: Path, config: VehicleProfile, *, headless: bool = True) -> int:
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
        return service_status(unit, config, user=True)
    else:
        argv = ["systemctl", "--user", command, unit]
    return subprocess.run(argv, check=False, timeout=50).returncode


_REQUALIFICATION_REJECTIONS = {
    b"progress_history_not_broken": "Progress history is not broken; no requalification was armed. "
    "Check live safety state for other blockers.",
    b"motion_anomaly_latched": "A motion anomaly remains latched; requalification cannot clear it.",
    b"stationary_profile_unqualified": "Stationary calibration is unavailable or unqualified; "
    "check the simulation profile.",
    b"clock_unqualified": "The simulation clock is unqualified; restore valid clock observations.",
}


# The helper only requests the existing coordinator check; it publishes no motion commands.
_REQUALIFY_MOTION = """
import sys
import rclpy
from std_srvs.srv import Trigger

rclpy.init(args=[])
node = rclpy.create_node("senior_sim_requalify_motion")
try:
    client = node.create_client(Trigger, "/motion_fusion/requalify_progress")
    accepted = False
    if client.wait_for_service(timeout_sec=5.0):
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
        if future.done() and not future.cancelled() and future.exception() is None:
            response = future.result()
            accepted = response is not None and response.success is True
            if response is not None and not accepted:
                reason = {
                    "progress history is not broken": "progress_history_not_broken",
                    "motion anomaly remains latched": "motion_anomaly_latched",
                    "stationary profile unqualified": "stationary_profile_unqualified",
                    "clock unqualified": "clock_unqualified",
                }.get(response.message)
                if reason is not None:
                    print(reason)
    sys.exit(0 if accepted else 1)
finally:
    node.destroy_node()
    rclpy.shutdown()
"""


def requalify_motion(profile_path: Path, profile: VehicleProfile) -> int:
    """Arm the simulation's existing stationary requalification, without clearing faults."""
    if profile.kind != "simulation":
        print("Motion requalification is available only for simulation profiles.", file=sys.stderr)
        return 1
    try:
        if sys.platform != "linux":
            raise ValueError("simulation requires Linux")
        validate_simulation_profile(profile)
        active = subprocess.run(
            ["systemctl", "--user", "is-active", "--quiet", _unit(profile_path)],
            check=False,
            timeout=10,
        )
        if active.returncode != 0:
            print("Motion requalification requires a running simulation service.", file=sys.stderr)
            return 1

        async def verified_adapter() -> bool:
            return await asyncio.wait_for(_probe(profile, require_domain=True), timeout=5)

        if not asyncio.run(verified_adapter()):
            print(
                "Motion requalification requires a verified live ROS adapter in the configured "
                "ROS domain. Rebuild/restart older adapters or restore the running profile domain.",
                file=sys.stderr,
            )
            return 1
        env = dict(os.environ)
        env.pop("NOTIFY_SOCKET", None)
        env.update(
            ROS_DOMAIN_ID=str(profile.ros_domain),
            ROS_LOCALHOST_ONLY="1",
            ROS_HOME=str(profile.credentials.parent / "ros-home"),
            ROS_LOG_DIR=str(profile.credentials.parent / "ros-home" / "log"),
        )
        result = subprocess.run(
            [
                "/bin/bash",
                "-c",
                'source "$1" || exit; shift; export ROS_DOMAIN_ID="$1" ROS_LOCALHOST_ONLY=1; '
                'shift; exec "$@"',
                "senior-requalify-motion",
                str(profile.ros_setup),
                str(profile.ros_domain),
                "/usr/bin/python3",
                "-c",
                _REQUALIFY_MOTION,
            ],
            env=env,
            check=False,
            capture_output=True,
            timeout=25,
        )
        if result.returncode != 0:
            reason = result.stdout.strip()
            if result.returncode == 1 and reason in _REQUALIFICATION_REJECTIONS:
                print(
                    f"Motion requalification rejected ({reason.decode('ascii')}): "
                    f"{_REQUALIFICATION_REJECTIONS[reason]}",
                    file=sys.stderr,
                )
                return 1
            print(
                "Motion requalification request failed or was rejected. "
                "Check the running simulation and motion fusion service.",
                file=sys.stderr,
            )
            return 1
    except (OSError, ValueError, IpcError, asyncio.TimeoutError, subprocess.SubprocessError):
        print(
            "Motion requalification request failed. "
            "Check the simulation profile, live adapter and ROS service; retry after recovery.",
            file=sys.stderr,
        )
        return 1
    print(
        "Motion requalification request accepted (armed), not yet qualified. "
        "Release movement controls and keep the robot stationary: fresh requested/applied zero "
        "commands and two contiguous stationary observation intervals are required. "
        "Check live safety state before driving."
    )
    return 0


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


async def _probe(profile: VehicleProfile, *, require_domain: bool = False) -> bool:
    if profile.socket is None:
        return False
    try:
        credentials = read_credentials(profile.credentials)
        client = IpcClient(profile.socket)
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
    except (OSError, ValueError, IpcError):
        return False


def validate_simulation_profile(profile: VehicleProfile) -> None:
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


async def supervise(
    profile: VehicleProfile, credentials: Credentials, *, headless: bool = True
) -> int:
    assert profile.socket is not None
    if not headless:
        _desktop_environment()
    root = profile.credentials.parent
    env = dict(os.environ)
    env.pop("NOTIFY_SOCKET", None)
    env.pop("SENIOR_SIMULATION_VIDEO_FRAME", None)
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
    ros = None
    agent = None
    try:
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
            "qualified_reference:=true",
            *video_args,
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
                if camera is not None:
                    camera.cleanup()


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


def main(path: Path, *, headless: bool = True) -> int:
    try:
        profile = VehicleProfile.model_validate_json(path.read_bytes())
        validate_simulation_profile(profile)
        credentials = read_credentials(profile.credentials)
        assert profile.estop_state is not None and profile.socket is not None
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
            _remove_stale_socket(profile.socket)
            _initialize_estop(profile.estop_state, credentials.robot_id)
            return asyncio.run(supervise(profile, credentials, headless=headless))
    except DisplayUnavailableError as error:
        print(str(error), file=sys.stderr)
        return 1
    except (OSError, ValueError, IpcError):
        print("Simulation startup failed. Check the profile, build and adapter.", file=sys.stderr)
        return 1


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("profile", type=Path)
    parser.add_argument("--headless", choices=("true", "false"), default="true")
    arguments = parser.parse_args()
    raise SystemExit(main(arguments.profile, headless=arguments.headless == "true"))
