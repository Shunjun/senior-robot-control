"""Installed fixed actions bound to the original production platform owners.

//Compatible: existing SDK provides the actual CLI/runtime bindings during the
migration; this is not an independently deployed manager factory.
"""

import sys

from vehicle_manager.operator_actions import docker_action
from vehicle_manager.operator_actions import main as execute
from vehicle_manager.platform_ports import DockerOperatorPorts
from vehicle_manager.profile import VehicleProfile

from senior_control.cli import _ports


def main() -> int:
    if sys.argv[1:] == ["docker-request"]:
        from senior_control.docker_start_helper import _adapter_command, _runtime_profile
        from senior_control.runtime.ipc import MAX_PATH
        from senior_control.simulation_cli import _probe

        def validate_runtime(profile: VehicleProfile) -> None:
            _adapter_command(profile, "doctor")

        return docker_action(
            ports=_ports(),
            runtime_ports=DockerOperatorPorts(
                runtime_profile=_runtime_profile,
                validate_runtime=validate_runtime,
                probe=_probe,
                max_path=MAX_PATH,
            ),
        )
    return execute(sys.argv[1:], ports=_ports())


if __name__ == "__main__":
    raise SystemExit(main())
