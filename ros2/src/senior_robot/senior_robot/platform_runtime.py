"""Real ROS process commands and optional video environment probing.

No ROS imports run at module import. The operator supplies the original profile
and recovery-prefix function; IPC ownership and service lifetimes stay outside.
"""

import asyncio
import json
import logging
import os
import signal
import sys
from collections.abc import Callable
from pathlib import Path
from typing import Protocol


class RuntimeProfile(Protocol):
    """Read-only fields actually consumed by the real execution provider."""

    @property
    def motion_profile(self) -> Path | None: ...

    @property
    def safety_parameters(self) -> Path | None: ...

    @property
    def ros_setup(self) -> Path | None: ...

    @property
    def ros_domain(self) -> int | None: ...

    @property
    def vendor_ros_setup(self) -> Path: ...

    @property
    def navigation_reference(self) -> Path | None: ...

    @property
    def navigation_parameters(self) -> Path | None: ...


def _adapter_command(profile: RuntimeProfile, robot_id: str) -> list[str]:
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
        # //Compatible: env-shebang ROS nodes must use the launch virtualenv after setup.
        'export PATH="${1%/*}${PATH:+:$PATH}"; '
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
    profile: RuntimeProfile, environment: dict[str, str]
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
        logging.getLogger("senior_robot.service_runtime").warning(
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


def _motion_command(profile: RuntimeProfile, robot_id: str) -> list[str]:
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
    profile: RuntimeProfile, robot_id: str, package: str, filename: str, *arguments: str
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


def _camera_command(
    profile: RuntimeProfile,
    robot_id: str,
    *,
    restart_prefix: Callable[[], str],
    launch_command: Callable[..., list[str]] = _ros_launch_command,
) -> list[str]:
    prefix = restart_prefix()
    # This is a separate launch process: the inherited prefix cannot reach
    # motion/safety. Installed Humble matches the executable basename, not the
    # ROS node name. Keep the vendor XML/OpaqueFunction and original argv.
    return launch_command(
        profile,
        robot_id,
        "turn_on_wheeltec_robot",
        "wheeltec_camera.launch.py",
        f"launch-prefix:={prefix}",
        "launch-prefix-filter:=^(?:astra_camera_node|usb_cam_node_exe|republish)$",
    )
