"""//Compatible: existing senior-robot entry and original installed platform effects."""

from getpass import getpass
from pathlib import Path

from vehicle_manager import cli as _canonical
from vehicle_manager.agent.pairing import pair_device
from vehicle_manager.cli import ENV_ROOT as ENV_ROOT
from vehicle_manager.cli import SERVICE_ROOT as SERVICE_ROOT
from vehicle_manager.cli import UNIT_ROOT as UNIT_ROOT
from vehicle_manager.cli import _install_once as _install_once
from vehicle_manager.cli import _unit as _unit
from vehicle_manager.platform_ports import OperatorPorts
from vehicle_manager.profile import VehicleProfile as VehicleProfile
from vehicle_manager.profile import load_profile as load_profile


# //Compatible: retain original lazy platform imports, including pair/help without native runtime.
def _platform_private_directory(path: Path) -> None:
    from vehicle_manager.deployment import private_directory

    return private_directory(path)


def _platform_doctor(profile: VehicleProfile) -> int:
    from vehicle_manager.platform_deployment import doctor

    return doctor(profile)


def _platform_deploy(profile: VehicleProfile, unit: str, *, dry_run: bool) -> int:
    from vehicle_manager.platform_deployment import deploy

    return deploy(profile, unit, dry_run=dry_run)


def _platform_validate(profile: VehicleProfile) -> None:
    from vehicle_manager.simulation_runtime import validate_simulation_profile

    return validate_simulation_profile(profile)


def _platform_lifecycle(
    command: str, profile: Path, config: VehicleProfile, *, headless: bool = True
) -> int:
    from vehicle_manager.simulation_runtime import lifecycle

    # Preserve the legacy call shape when headless has its original default.
    if headless:
        return lifecycle(command, profile, config)
    return lifecycle(command, profile, config, headless=False)


def _platform_status(unit: str, profile: VehicleProfile, *, user: bool) -> int:
    from vehicle_manager.simulation_runtime import service_status

    return service_status(unit, profile, user=user)


def _ports() -> OperatorPorts:
    return OperatorPorts(
        private_directory=_platform_private_directory,
        vehicle_doctor=_platform_doctor,
        vehicle_deploy=_platform_deploy,
        simulation_validate=_platform_validate,
        lifecycle=_platform_lifecycle,
        service_status=_platform_status,
        doctor=_doctor,
        deploy=_deploy,
        pair=_pair,
        pair_device=pair_device,
        getpass=getpass,
        install_once=_install_once,
        service_root=SERVICE_ROOT,
        env_root=ENV_ROOT,
        unit_root=UNIT_ROOT,
    )


def _pair(profile: VehicleProfile, cloud_url: str, name: str) -> None:
    return _canonical._pair(profile, cloud_url, name, ports=_ports())


def _doctor(profile: VehicleProfile) -> int:
    return _canonical._doctor(profile, ports=_ports())


def _deploy(profile: VehicleProfile, profile_path: Path, *, dry_run: bool) -> int:
    return _canonical._deploy(profile, profile_path, dry_run=dry_run, ports=_ports())


def main(argv: list[str] | None = None) -> int:
    return _canonical.main(argv, ports=_ports())


if __name__ == "__main__":
    raise SystemExit(main())
