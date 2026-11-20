"""//Compatible: original deployment API with existing runtime preflight and CLI roots."""

from pathlib import Path

from vehicle_manager import deployment as _canonical
from vehicle_manager.deployment import _argument as _argument
from vehicle_manager.deployment import _command as _command
from vehicle_manager.deployment import _path_text as _path_text
from vehicle_manager.deployment import _project as _project
from vehicle_manager.deployment import _runtime_root as _runtime_root
from vehicle_manager.deployment import container_directory as container_directory
from vehicle_manager.deployment import environment as environment
from vehicle_manager.deployment import inspect_container as inspect_container
from vehicle_manager.deployment import private_directory as private_directory
from vehicle_manager.deployment import render_service as render_service
from vehicle_manager.platform_ports import DeploymentPorts
from vehicle_manager.profile import VehicleProfile


def _validate_runtime(motion_profile: Path | None, safety_parameters: Path | None) -> object:
    from senior_control.real_runtime import validate_real_runtime

    return validate_real_runtime(motion_profile, safety_parameters)


def _validate_navigation(reference: Path | None, parameters: Path | None) -> None:
    from senior_control.real_runtime import validate_navigation

    return validate_navigation(reference, parameters)


def _ports() -> DeploymentPorts:
    from senior_control.cli import UNIT_ROOT, _install_once
    from senior_control.runtime.ipc import MAX_PATH

    return DeploymentPorts(
        UNIT_ROOT, _install_once, MAX_PATH, _validate_runtime, _validate_navigation
    )


def _validate_paths(profile: VehicleProfile, runtime_root: Path) -> None:
    return _canonical._validate_paths(profile, runtime_root, max_path=_ports().max_path)


def deploy(profile: VehicleProfile, unit: str, *, dry_run: bool) -> int:
    return _canonical.deploy(profile, unit, dry_run=dry_run, ports=_ports())


def doctor(profile: VehicleProfile) -> int:
    return _canonical.doctor(profile, ports=_ports())
