"""//Compatible: installed real service entry; ROS/device effects remain original owners."""

import asyncio
import os
import socket
import struct
from pathlib import Path
from typing import TYPE_CHECKING, Literal

from senior_robot import platform_runtime as _platform
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
    """//Compatible: original signature over the sole execution provider."""
    return _platform._adapter_command(profile, robot_id)


async def _video_environment(
    profile: "VehicleProfile", environment: dict[str, str]
) -> dict[str, str]:
    """//Compatible: preserve the original optional-video degradation path."""
    return await _platform._video_environment(profile, environment)


def _motion_command(profile: "VehicleProfile", robot_id: str) -> list[str]:
    """//Compatible: the original safety launch, constructed once by its owner."""
    return _platform._motion_command(profile, robot_id)


def _ros_launch_command(
    profile: "VehicleProfile", robot_id: str, package: str, filename: str, *arguments: str
) -> list[str]:
    """//Compatible: keep the old launch hook used by the installed service."""
    return _platform._ros_launch_command(profile, robot_id, package, filename, *arguments)


def _camera_command(profile: "VehicleProfile", robot_id: str) -> list[str]:
    """//Compatible: inject this invocation's real recovery and launch effects."""
    from senior_control.runtime.launch_recovery import restart_prefix

    return _platform._camera_command(
        profile, robot_id, restart_prefix=restart_prefix, launch_command=_ros_launch_command
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
