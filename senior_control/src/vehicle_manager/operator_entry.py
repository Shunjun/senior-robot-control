"""Installed fixed actions bound to the original production platform owners.

//Compatible: existing SDK provides the actual CLI/runtime bindings during the
migration; this is not an independently deployed manager factory.
"""

import sys

from vehicle_manager.operator_actions import docker_action
from vehicle_manager.operator_actions import main as execute
from vehicle_manager.operator_cli import _ports
from vehicle_manager.platform_ports import DockerOperatorPorts
from vehicle_manager.profile import VehicleProfile


def main() -> int:
    if sys.argv[1:] == ["docker-request"]:
        from senior_robot.service_runtime import _adapter_command

        from vehicle_core.ipc import MAX_PATH
        from vehicle_manager.service import _runtime_profile
        from vehicle_manager.simulation_runtime import _probe

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
