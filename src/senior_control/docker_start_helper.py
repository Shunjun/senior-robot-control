"""//Compatible: installed real service entry; ROS/device effects remain original owners."""

import asyncio
import json
import logging
import os
import signal
import socket
import struct
import sys
from pathlib import Path
from typing import TYPE_CHECKING, Literal

from vehicle_manager import service as _canonical
from vehicle_manager.platform_ports import Events, Learning, ServicePorts
from vehicle_manager.service import _runtime_profile as _runtime_profile
from vehicle_manager.service import _terminate as _terminate
from vehicle_manager.service import _terminate_motion_stack as _terminate_motion_stack

from senior_control.agent.pairing import read_credentials
from senior_control.runtime.ipc import IpcClient, IpcError, IpcServer

# //Compatible: shared socket ownership has one implementation at the existing IPC boundary.
from senior_control.runtime.ipc import _identity as _identity
from senior_control.runtime.ipc import _prepare_socket as _prepare_socket
from senior_control.runtime.ipc import _record as _record

if TYPE_CHECKING:
    from senior_control.cli import VehicleProfile


def _adapter_command(profile: "VehicleProfile", robot_id: str) -> list[str]:
    from senior_robot.real_runtime import validate_real_runtime

    hardware = validate_real_runtime(profile.motion_profile, profile.safety_parameters)
    setup = profile.ros_setup
    if setup is None or not setup.is_absolute() or not setup.is_file():
        raise ValueError("ROS workspace setup is missing; build it and configure ros_setup")
    if profile.ros_domain is not None and not 0 <= profile.ros_domain <= 232:
        raise ValueError("ros_domain must be between 0 and 232")
    vendor = profile.vendor_ros_setup
    if not vendor.is_absolute() or not vendor.is_file():
        raise ValueError("vendor_ros_setup: WHEELTEC install/setup.bash is missing")
    from senior_robot.real_runtime import validate_navigation

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


def _camera_command(profile: "VehicleProfile", robot_id: str) -> list[str]:
    from senior_control.runtime.launch_recovery import restart_prefix

    prefix = restart_prefix()
    # This is a separate launch process: the inherited prefix cannot reach
    # motion/safety. Installed Humble matches the executable basename, not the
    # ROS node name. Keep the vendor XML/OpaqueFunction and original argv.
    return _ros_launch_command(
        profile,
        robot_id,
        "turn_on_wheeltec_robot",
        "wheeltec_camera.launch.py",
        f"launch-prefix:={prefix}",
        "launch-prefix-filter:=^(?:astra_camera_node|usb_cam_node_exe|republish)$",
    )


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


def _start_environment(environment: dict[str, str], root: Path) -> dict[str, str]:
    from senior_perception.startup_learning import start_environment

    return start_environment(environment, root)


def _learning(path: Path) -> Learning:
    from senior_perception.learning_service import LearningService

    return LearningService(path)


def _cleanup_learning(
    root: Path,
    current_claim: str,
    *,
    peer_lock_name: Literal[".agent-service.lock", ".senior-simulator.lock"],
) -> None:
    from senior_perception.learning_cleanup import cleanup_old_learning_files

    return cleanup_old_learning_files(root, current_claim, peer_lock_name=peer_lock_name)


def _events() -> Events:
    from senior_control.runtime.lifecycle_events import LifecycleEvents

    return LifecycleEvents()


def _ports() -> ServicePorts:
    return ServicePorts(
        client=IpcClient,
        server=IpcServer,
        error=IpcError,
        read_credentials=read_credentials,
        identity=_identity,
        record=_record,
        prepare_socket=_prepare_socket,
        verify_adapter_peer=_verify_adapter_peer,
        runtime_profile=_runtime_profile,
        adapter_command=_adapter_command,
        motion_command=_motion_command,
        camera_command=_camera_command,
        video_environment=_video_environment,
        wait_for_adapter=_wait_for_adapter,
        terminate=_terminate,
        terminate_motion_stack=_terminate_motion_stack,
        start_environment=_start_environment,
        learning=_learning,
        cleanup_learning=_cleanup_learning,
        events=_events,
        supervise=supervise,
        stop=stop,
    )


async def _wait_for_adapter(
    path: Path,
    robot_id: str,
    domain: int | None,
    adapter: asyncio.subprocess.Process,
    stopping: asyncio.Event,
) -> list[int] | None:
    return await _canonical._wait_for_adapter(
        path, robot_id, domain, adapter, stopping, ports=_ports()
    )


async def supervise(
    root: Path,
    command: list[str],
    environment: dict[str, str],
    *,
    profile: "VehicleProfile | None" = None,
) -> int:
    return await _canonical.supervise(root, command, environment, profile=profile, ports=_ports())


async def stop(root: Path) -> int:
    return await _canonical.stop(root, ports=_ports())


def main(argv: list[str] | None = None) -> int:
    return _canonical.main(argv, ports=_ports())


if __name__ == "__main__":
    raise SystemExit(main())
