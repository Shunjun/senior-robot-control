"""Own the real-vehicle ROS adapter and Agent, in Docker or a native service.

The shared sensor/fusion/safety stack and adapter are supervised together.
WHEELTEC chassis/IMU/lidar drivers share the motion launch lifecycle.
Map/localization remain externally managed.
The private stop endpoint also works through docker exec without stopping the container."""

import argparse
import asyncio
import fcntl
import json
import os
import signal
import sys
from pathlib import Path
from typing import Any

from vehicle_manager.deployment import private_directory
from vehicle_manager.platform_ports import ServicePorts
from vehicle_manager.profile import VehicleProfile


def _runtime_profile(root: Path) -> "VehicleProfile":
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


async def _wait_for_adapter(
    path: Path,
    robot_id: str,
    domain: int | None,
    adapter: asyncio.subprocess.Process,
    stopping: asyncio.Event,
    *,
    ports: ServicePorts,
) -> list[int] | None:
    deadline = asyncio.get_running_loop().time() + 35
    while not stopping.is_set():
        if adapter.returncode is not None:
            raise ValueError(
                "ROS adapter exited; check sourced ROS workspace and Python dependencies"
            )
        client = ports.client(path, timeout_ms=500)
        try:
            reply = await client.request("identity", robot_id=robot_id)
        except (ports.error, OSError):
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
            identity = await ports.verify_adapter_peer(path, adapter.pid)
            ports.record(path, identity)
            if ports.identity(path) != identity:
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


async def _terminate_motion_stack(
    process: asyncio.subprocess.Process | None, *, group_grace: float = 2, budget: float = 12
) -> None:
    if process is None:
        return
    loop = asyncio.get_running_loop()
    deadline = loop.time() + budget
    # ros2 launch owns multiple required children. Signal the owned process
    # group so timeout escalation cannot leave an orphan actuator publisher.
    try:
        os.killpg(process.pid, signal.SIGINT)
    except ProcessLookupError:
        return
    try:
        await asyncio.wait_for(process.wait(), min(10, budget))
    except asyncio.TimeoutError:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        await process.wait()
    # A launcher can exit before its children. Its process group is still ours;
    # reap/stop survivors instead of leaving camera or actuator children behind.
    deadline = min(deadline, loop.time() + group_grace)
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
    ports: ServicePorts,
) -> int:
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
        robot_id = (
            ports.read_credentials(root / "device.json").robot_id if profile is not None else ""
        )
        environment = ports.start_environment(environment, root)
        learning_service = ports.learning(Path(environment["SENIOR_STARTUP_LEARNING_CLAIM"]))
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
        events = ports.events() if profile is not None else None

        async def handle(request: dict[str, Any]) -> dict[str, Any]:
            if request["type"] == "status":
                return {"ok": True, "running": process is not None and process.returncode is None}
            if request["type"] == "stop":
                stopping.set()
                return {"ok": True}
            return {"ok": False, "error": "unsupported"}

        server = ports.server(root / "service.sock", handle, replace_existing=False)
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
                await ports.prepare_socket(root / "agent.sock")
                motion_stack = await asyncio.create_subprocess_exec(
                    *ports.motion_command(profile, robot_id),
                    env=child_env,
                    start_new_session=True,
                    pass_fds=(events.writer,),
                )
                adapter = await asyncio.create_subprocess_exec(
                    *ports.adapter_command(profile, robot_id),
                    env=environment,
                    start_new_session=True,
                )
                adapter_identity = await ports.wait_for_adapter(
                    root / "agent.sock", robot_id, profile.ros_domain, adapter, stopping
                )
                if adapter_identity is not None:
                    ports.record(
                        root / "agent.sock",
                        adapter_identity,
                        run_id=environment["SENIOR_VERIFIED_ADAPTER_RUN"],
                    )
            except (OSError, ValueError, ports.error) as exc:
                events.report("motion", "unavailable", str(exc))
                await ports.terminate_motion_stack(adapter)
                await ports.terminate_motion_stack(motion_stack)
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
                ports.cleanup_learning(
                    root,
                    environment["SENIOR_STARTUP_LEARNING_CLAIM"],
                    peer_lock_name=".senior-simulator.lock",
                )
            events.report("motion", "ready", "adapter verified; cloud status independent")
            try:
                camera_stack = await asyncio.create_subprocess_exec(
                    *ports.camera_command(profile, robot_id),
                    env=child_env,
                    start_new_session=True,
                    pass_fds=(events.writer,),
                )
                watch(camera_stack, "camera")
            except (OSError, ValueError) as exc:
                events.report("camera", "unavailable", str(exc))

        try:
            await learning_service.start()
            await ports.prepare_socket(root / "service.sock")
            await server.start()
            ports.record(root / "service.sock")
            for number in signals:
                loop.add_signal_handler(number, stopping.set)
            # //Compatible: profile=None still propagates a single arbitrary child's exit.
            agent_env = events.agent_environment(environment) if events else environment
            if profile is not None:
                agent_env = await ports.video_environment(profile, agent_env)
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
                        await ports.terminate_motion_stack(motion_stack)
                        motion_stack = None
                    elif name == "adapter":
                        await ports.terminate_motion_stack(adapter)
                        adapter = None
                    elif name == "camera":
                        # Let leaf prefixes finish their bounded five-second
                        # cleanup, including scheduling margin, before KILL.
                        await ports.terminate_motion_stack(camera_stack, group_grace=6, budget=10)
                        camera_stack = None
        finally:
            if startup is not None and not startup.done():
                startup.cancel()
                await asyncio.gather(startup, return_exceptions=True)
            try:
                await ports.terminate(process, 20)
            finally:
                try:
                    await ports.terminate_motion_stack(adapter)
                finally:
                    try:
                        await ports.terminate_motion_stack(motion_stack)
                    finally:
                        await ports.terminate_motion_stack(camera_stack, group_grace=6, budget=10)
            for task in waiters:
                if not task.done():
                    task.cancel()
            await asyncio.gather(*waiters, return_exceptions=True)
            try:
                if adapter_identity is not None:
                    path = root / "agent.sock"
                    if os.path.lexists(path) and ports.identity(path) == adapter_identity:
                        path.unlink()
            finally:
                if events:
                    events.close()
                await server.close()
                await learning_service.close()
                for number in signals:
                    loop.remove_signal_handler(number)


async def stop(root: Path, *, ports: ServicePorts) -> int:
    private_directory(root)
    client = ports.client(root / "service.sock", timeout_ms=3000)
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


def main(argv: list[str] | None = None, *, ports: ServicePorts) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("run", "stop"))
    parser.add_argument("--control-dir", type=Path, required=True)
    args = parser.parse_args(argv)
    preflight_complete = False
    try:
        if args.action == "stop":
            return asyncio.run(ports.stop(args.control_dir))
        profile = ports.runtime_profile(args.control_dir)
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
        ports.adapter_command(
            profile, ports.read_credentials(args.control_dir / "device.json").robot_id
        )
        preflight_complete = True
        return asyncio.run(
            ports.supervise(
                args.control_dir,
                [sys.executable, "-m", "senior_control.agent.main"],
                env,
                profile=profile,
            )
        )
    except (OSError, ValueError, ports.error) as exc:
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
