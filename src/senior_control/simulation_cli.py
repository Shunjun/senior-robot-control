"""//Compatible: original simulator module entry with real IPC/learning/lifecycle bindings."""

import argparse
import asyncio
from pathlib import Path

from vehicle_manager import simulation as _canonical
from vehicle_manager.platform_ports import SimulationPorts
from vehicle_manager.profile import VehicleProfile as VehicleProfile
from vehicle_manager.profile import load_profile as load_profile
from vehicle_manager.simulation import _DESKTOP_ENVIRONMENT as _DESKTOP_ENVIRONMENT
from vehicle_manager.simulation import DisplayUnavailableError as DisplayUnavailableError
from vehicle_manager.simulation import _desktop_environment as _desktop_environment
from vehicle_manager.simulation import _notify as _notify
from vehicle_manager.simulation import _remove_stale_socket as _remove_stale_socket
from vehicle_manager.simulation import _stop as _stop
from vehicle_manager.simulation import _unit as _unit

from senior_control.agent.pairing import Credentials, read_credentials
from senior_control.runtime.ipc import MAX_PATH, IpcClient, IpcError


def _ports() -> SimulationPorts:
    from senior_control import docker_start_helper as helper

    return SimulationPorts(
        client=IpcClient,
        error=IpcError,
        max_path=MAX_PATH,
        read_credentials=read_credentials,
        start_environment=helper._start_environment,
        learning=helper._learning,
        cleanup_learning=helper._cleanup_learning,
        events=helper._events,
        terminate_motion_stack=helper._terminate_motion_stack,
        wait_for_adapter=_wait_for_adapter,
        stop=_stop,
        notify=_notify,
        validate_profile=validate_simulation_profile,
        remove_stale_socket=_remove_stale_socket,
        supervise=supervise,
    )


def lifecycle(command: str, profile: Path, config: VehicleProfile, *, headless: bool = True) -> int:
    return _canonical.lifecycle(command, profile, config, headless=headless, status=service_status)


def service_status(unit: str, profile: VehicleProfile, *, user: bool) -> int:
    return _canonical.service_status(unit, profile, user=user, probe=_probe)


async def _probe(profile: VehicleProfile, *, require_domain: bool = False) -> bool:
    return await _canonical._probe(profile, require_domain=require_domain, ports=_ports())


def validate_simulation_profile(profile: VehicleProfile) -> None:
    return _canonical.validate_simulation_profile(profile, max_path=MAX_PATH)


async def _wait_for_adapter(
    path: Path, robot_id: str, ros: asyncio.subprocess.Process, stopping: asyncio.Event
) -> None:
    return await _canonical._wait_for_adapter(path, robot_id, ros, stopping, ports=_ports())


async def supervise(
    profile: VehicleProfile, credentials: Credentials, *, headless: bool = True
) -> int:
    return await _canonical.supervise(profile, credentials, headless=headless, ports=_ports())


def main(path: Path, *, headless: bool = True) -> int:
    return _canonical.main(path, headless=headless, ports=_ports())


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=_canonical.__doc__)
    parser.add_argument("profile", type=Path)
    parser.add_argument("--headless", choices=("true", "false"), default="true")
    arguments = parser.parse_args()
    raise SystemExit(main(arguments.profile, headless=arguments.headless == "true"))
