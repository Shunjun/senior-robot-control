"""Own the real-vehicle ROS adapter and Agent, in Docker or a native service.

The shared sensor/fusion/safety stack and adapter are supervised together.
WHEELTEC chassis/IMU/lidar drivers share the motion launch lifecycle.
Map/localization remain externally managed.
The private stop endpoint also works through docker exec without stopping the container.
"""

import argparse
import asyncio
import fcntl
import json
import logging
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


def _record(path: Path, expected: list[int] | None = None, *, run_id: str | None = None) -> None:
    identity = _identity(path)
    if expected is not None and identity != expected:
        raise ValueError("IPC socket changed before ownership recording")
    marker = path.with_suffix(".owner.json")
    temporary = marker.with_suffix(".tmp")
    # The enclosing private directory and instance lock protect this ownership record.
    fd = os.open(temporary, os.O_CREAT | os.O_TRUNC | os.O_WRONLY | os.O_NOFOLLOW, 0o600)
    with os.fdopen(fd, "w") as file:
        json.dump(identity if run_id is None else {"identity": identity, "run_id": run_id}, file)
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
            # //Compatible: pre-isolation ownership records contain only the identity list.
            if isinstance(owned, dict):
                owned = owned.get("identity")
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
    ):
        if name in values and Path(values[name]) != configured_root / filename:
            raise ValueError("data paths must stay in the control directory")
        values[name] = root / filename
    values["control_dir"] = root
    if values.get("project_dir") and not values.get("ros_setup"):
        values["ros_setup"] = Path(values["project_dir"]) / "ros2/install/setup.bash"
    for name in (
        "motion_profile",
        "safety_parameters",
        "navigation_reference",
        "navigation_parameters",
    ):
        if values.get(name):
            selected = Path(values[name])
            if not selected.is_absolute():
                values[name] = root / selected
            elif selected.is_relative_to(configured_root):
                values[name] = root / selected.relative_to(configured_root)
    return VehicleProfile.model_validate(values)


def _adapter_command(profile: "VehicleProfile", robot_id: str) -> list[str]:
    from senior_control.real_runtime import validate_real_runtime

    hardware = validate_real_runtime(profile.motion_profile, profile.safety_parameters)
    setup = profile.ros_setup
    if setup is None or not setup.is_absolute() or not setup.is_file():
        raise ValueError("ROS workspace setup is missing; build it and configure ros_setup")
    if profile.ros_domain is not None and not 0 <= profile.ros_domain <= 232:
        raise ValueError("ros_domain must be between 0 and 232")
    vendor = profile.vendor_ros_setup
    if not vendor.is_absolute() or not vendor.is_file():
        raise ValueError("vendor_ros_setup: WHEELTEC install/setup.bash is missing")
    from senior_control.real_runtime import validate_navigation

    validate_navigation(profile.navigation_reference, profile.navigation_parameters)
    return [
        "/bin/bash",
        "-c",
        'source "$1" || exit; source "$2" || exit; shift 2; '
        'if [ -n "${SENIOR_EXPECTED_ROS_DOMAIN:-}" ]; then '
        'export ROS_DOMAIN_ID="$SENIOR_EXPECTED_ROS_DOMAIN"; fi; exec "$@"',
        "senior-vehicle-ros",
        str(vendor),
        str(setup),
        sys.executable,
        "-c",
        "from senior_robot.agent_adapter import main; main()",
        "--ros-args",
        "--params-file",
        str(profile.safety_parameters),
        "-p",
        "use_sim_time:=false",
        "-p",
        "native_writer_receipts:=true",
        "-p",
        f"robot_id:={robot_id}",
        "-p",
        "ipc_replace_existing_socket:=false",
        *(
            [
                "-p",
                f"hardware_navigation_reference:={profile.navigation_reference}",
                "-p",
                f"navigation_controller_params:={profile.navigation_parameters}",
            ]
            if profile.navigation_reference is not None
            else []
        ),
        "-p",
        f"navigation_odometry_topic:={hardware.wheel_topic}",
    ]


async def _video_environment(
    profile: "VehicleProfile", environment: dict[str, str]
) -> dict[str, str]:
    """Prepare only the Agent's video imports; failure preserves its control environment."""
    original = dict(environment)
    original.pop("SENIOR_ROS_VIDEO_TOPIC", None)
    process = None
    try:
        if profile.ros_setup is None:
            raise ValueError("ROS workspace setup is missing")
        keys = (
            "AMENT_PREFIX_PATH",
            "COLCON_PREFIX_PATH",
            "CMAKE_PREFIX_PATH",
            "PYTHONPATH",
            "LD_LIBRARY_PATH",
            "PATH",
            "ROS_DISTRO",
            "ROS_VERSION",
            "ROS_PYTHON_VERSION",
            "RMW_IMPLEMENTATION",
            "ROS_DOMAIN_ID",
            "ROS_LOCALHOST_ONLY",
        )
        probe = (
            "import json, os; import rclpy; from sensor_msgs.msg import Image; "
            f"print(json.dumps({{key: os.environ[key] for key in {keys!r} if key in os.environ}}))"
        )
        process = await asyncio.create_subprocess_exec(
            "/bin/bash",
            "-c",
            'source "$1" || exit; source "$2" || exit; '
            'if [ -n "${SENIOR_EXPECTED_ROS_DOMAIN:-}" ]; then '
            'export ROS_DOMAIN_ID="$SENIOR_EXPECTED_ROS_DOMAIN"; fi; exec "$3" -c "$4"',
            "senior-video-environment",
            str(profile.vendor_ros_setup),
            str(profile.ros_setup),
            sys.executable,
            probe,
            env=original,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE,
            start_new_session=True,
        )
        stdout, _ = await asyncio.wait_for(process.communicate(), 5.0)
        if process.returncode != 0:
            raise ValueError("ROS overlay or camera Python imports failed")
        values = json.loads(stdout.decode().splitlines()[-1])
        if not isinstance(values, dict) or any(
            key not in keys or not isinstance(value, str) for key, value in values.items()
        ):
            raise ValueError("invalid ROS environment")
        return {**original, **values, "SENIOR_ROS_VIDEO_TOPIC": "/camera/color/image_raw"}
    except (OSError, ValueError, IndexError, asyncio.TimeoutError) as exc:
        logging.getLogger(__name__).warning(
            "ROS video unavailable; starting Agent with original control environment: %s", exc
        )
        return original
    finally:
        if process is not None and process.returncode is None:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            await process.communicate()


def _motion_command(profile: "VehicleProfile", robot_id: str) -> list[str]:
    # Use the same sourced overlay/domain as the adapter; no second authority
    # owner or actuator publisher is introduced by the hardware launch.
    return _ros_launch_command(
        profile,
        robot_id,
        "senior_robot",
        "safety_motion.launch.py",
        f"safety_parameters:={profile.safety_parameters}",
        f"hardware_motion_profile:={profile.motion_profile}",
        f"robot_id:={robot_id}",
        *(
            [f"navigation_parameters:={profile.navigation_parameters}"]
            if profile.navigation_parameters is not None
            else []
        ),
    )


def _ros_launch_command(
    profile: "VehicleProfile", robot_id: str, package: str, filename: str, *arguments: str
) -> list[str]:
    base = _adapter_command(profile, robot_id)
    return base[:6] + [
        # Keep launch-file imports in the same Python environment as the adapter.
        # The ros2 console script can have a system-Python shebang even when
        # senior_control is installed only in the service's ROS virtualenv.
        sys.executable,
        "-c",
        "from ros2cli.cli import main; raise SystemExit(main())",
        "launch",
        package,
        filename,
        *arguments,
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
        client = IpcClient(path, timeout_ms=500)
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


async def _terminate_motion_stack(process: asyncio.subprocess.Process | None) -> None:
    if process is None:
        return
    # ros2 launch owns multiple required children. Signal the owned process
    # group so timeout escalation cannot leave an orphan actuator publisher.
    try:
        os.killpg(process.pid, signal.SIGINT)
    except ProcessLookupError:
        return
    try:
        await asyncio.wait_for(process.wait(), 10)
    except asyncio.TimeoutError:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        await process.wait()
    # A launcher can exit before its children. Its process group is still ours;
    # reap/stop survivors instead of leaving camera or actuator children behind.
    deadline = asyncio.get_running_loop().time() + 2
    while True:
        try:
            os.killpg(process.pid, 0)
        except ProcessLookupError:
            return
        if asyncio.get_running_loop().time() >= deadline:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            return
        await asyncio.sleep(0.05)


async def supervise(
    root: Path,
    command: list[str],
    environment: dict[str, str],
    *,
    profile: "VehicleProfile | None" = None,
) -> int:
    from senior_control.runtime.lifecycle_events import LifecycleEvents

    private_directory(root)
    fd = os.open(root / ".agent-service.lock", os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
    with os.fdopen(fd, "wb") as lock:
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        stopping = asyncio.Event()
        process: asyncio.subprocess.Process | None = None
        adapter: asyncio.subprocess.Process | None = None
        motion_stack: asyncio.subprocess.Process | None = None
        camera_stack: asyncio.subprocess.Process | None = None
        adapter_identity: list[int] | None = None
        robot_id = read_credentials(root / "device.json").robot_id if profile is not None else ""
        from senior_control.motion.startup_learning import start_environment
        from senior_control.runtime.learning_service import LearningService

        environment = start_environment(environment, root)
        learning_service = LearningService(Path(environment["SENIOR_STARTUP_LEARNING_CLAIM"]))
        if profile is not None:
            for key in tuple(environment):
                if key.startswith("SENIOR_SIMULATION_") or key.startswith("SENIOR_MOTION_CORE_"):
                    environment.pop(key)
            environment["ROS_LOCALHOST_ONLY"] = "0"
            environment["SENIOR_VERIFIED_ADAPTER_RUN"] = Path(
                environment["SENIOR_STARTUP_LEARNING_CLAIM"]
            ).name
        environment.update(ROS_HOME=str(root / "ros-home"), ROS_LOG_DIR=str(root / "ros-log"))
        environment.pop("SENIOR_EXPECTED_ROS_DOMAIN", None)
        if profile is not None and profile.ros_domain is not None:
            environment["SENIOR_EXPECTED_ROS_DOMAIN"] = str(profile.ros_domain)
            environment["ROS_DOMAIN_ID"] = str(profile.ros_domain)
        events = LifecycleEvents() if profile is not None else None

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
        waiters: dict[asyncio.Task[Any], str] = {}
        startup: asyncio.Task[None] | None = None

        def watch(child: asyncio.subprocess.Process, name: str) -> None:
            waiters[asyncio.create_task(child.wait())] = name

        async def start_local() -> None:
            nonlocal adapter, motion_stack, camera_stack, adapter_identity
            assert profile is not None and events is not None
            child_env = events.child_environment(environment)
            try:
                events.report("motion", "starting")
                await _prepare_socket(root / "agent.sock")
                motion_stack = await asyncio.create_subprocess_exec(
                    *_motion_command(profile, robot_id),
                    env=child_env,
                    start_new_session=True,
                    pass_fds=(events.writer,),
                )
                adapter = await asyncio.create_subprocess_exec(
                    *_adapter_command(profile, robot_id),
                    env=environment,
                    start_new_session=True,
                )
                adapter_identity = await _wait_for_adapter(
                    root / "agent.sock", robot_id, profile.ros_domain, adapter, stopping
                )
                if adapter_identity is not None:
                    _record(
                        root / "agent.sock",
                        adapter_identity,
                        run_id=environment["SENIOR_VERIFIED_ADAPTER_RUN"],
                    )
            except (OSError, ValueError, IpcError) as exc:
                events.report("motion", "unavailable", str(exc))
                await _terminate_motion_stack(adapter)
                await _terminate_motion_stack(motion_stack)
                adapter = motion_stack = None
                return
            if stopping.is_set():
                return
            assert adapter is not None and motion_stack is not None
            watch(motion_stack, "motion")
            watch(adapter, "adapter")
            await asyncio.sleep(0)
            if (
                adapter_identity is not None
                and process is not None
                and all(child.returncode is None for child in (process, adapter, motion_stack))
                and not stopping.is_set()
            ):
                from senior_control.runtime.learning_cleanup import cleanup_old_learning_files

                cleanup_old_learning_files(
                    root,
                    environment["SENIOR_STARTUP_LEARNING_CLAIM"],
                    peer_lock_name=".senior-simulator.lock",
                )
            events.report("motion", "ready", "adapter verified; cloud status independent")
            try:
                camera_stack = await asyncio.create_subprocess_exec(
                    *_ros_launch_command(
                        profile, robot_id, "turn_on_wheeltec_robot", "wheeltec_camera.launch.py"
                    ),
                    env=child_env,
                    start_new_session=True,
                    pass_fds=(events.writer,),
                )
                watch(camera_stack, "camera")
            except (OSError, ValueError) as exc:
                events.report("camera", "unavailable", str(exc))

        try:
            await learning_service.start()
            await _prepare_socket(root / "service.sock")
            await server.start()
            _record(root / "service.sock")
            for number in signals:
                loop.add_signal_handler(number, stopping.set)
            # //Compatible: profile=None still propagates a single arbitrary child's exit.
            agent_env = events.agent_environment(environment) if events else environment
            if profile is not None:
                agent_env = await _video_environment(profile, agent_env)
            process = await asyncio.create_subprocess_exec(
                *command, env=agent_env, pass_fds=(events.reader,) if events else ()
            )
            if events:
                events.close_reader()
            if process.returncode is not None:
                return await process.wait()
            watch(process, "agent")
            waiters[asyncio.create_task(stopping.wait())] = "stop"
            if profile is not None:
                startup = asyncio.create_task(start_local())
                waiters[startup] = "startup"
            while True:
                done, _ = await asyncio.wait(waiters, return_when=asyncio.FIRST_COMPLETED)
                if stopping.is_set():
                    return 0
                if process.returncode is not None:
                    return await process.wait()
                for task in done:
                    name = waiters.pop(task)
                    result = task.result()
                    if name == "agent":
                        return int(result)
                    if name == "startup":
                        continue
                    assert events is not None
                    events.report(name, "exited", returncode=result)
                    if name == "motion":
                        await _terminate_motion_stack(motion_stack)
                        motion_stack = None
                    elif name == "adapter":
                        await _terminate_motion_stack(adapter)
                        adapter = None
                    elif name == "camera":
                        await _terminate_motion_stack(camera_stack)
                        camera_stack = None
        finally:
            if startup is not None and not startup.done():
                startup.cancel()
                await asyncio.gather(startup, return_exceptions=True)
            try:
                await _terminate(process, 20)
            finally:
                try:
                    await _terminate_motion_stack(adapter)
                finally:
                    try:
                        await _terminate_motion_stack(motion_stack)
                    finally:
                        await _terminate_motion_stack(camera_stack)
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
                if events:
                    events.close()
                await server.close()
                await learning_service.close()
                for number in signals:
                    loop.remove_signal_handler(number)


async def stop(root: Path) -> int:
    private_directory(root)
    client = IpcClient(root / "service.sock", timeout_ms=3000)
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
    preflight_complete = False
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
        # Validate immutable configuration before opening service/ROS sockets.
        _adapter_command(profile, read_credentials(args.control_dir / "device.json").robot_id)
        preflight_complete = True
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
        # systemd already declares RestartPreventExitStatus=78. Configuration
        # errors require an operator correction; retrying cannot repair them.
        return 78 if args.action == "run" and not preflight_complete else 1


if __name__ == "__main__":
    raise SystemExit(main())
