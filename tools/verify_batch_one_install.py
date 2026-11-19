"""Verify real installed wheels from outside the checkout; no service is started.

Run with a clean interpreter after installing contracts, control and legacy manager
wheels. Native checks require a built ROS prefix in AMENT_PREFIX_PATH.
"""

import importlib
import importlib.metadata
import sys
from pathlib import Path


def main() -> None:
    from senior_contracts.configuration import DrivingSettings
    from senior_contracts.errors import DomainError
    from senior_contracts.navigation import RoutePlan, RouteProgress
    from senior_contracts.values import Velocity
    from senior_contracts.wire import serialize_envelope, validate_envelope
    from senior_motion_control.final_command_frame import LAYOUT
    from senior_motion_control.motion_output import MotionOutput
    from senior_motion_control.native import native_executable

    from senior_control.errors import DomainError as OldDomainError
    from senior_control.models import Velocity as OldVelocity
    from senior_control.navigation import RoutePlan as OldRoutePlan
    from senior_control.navigation import RouteProgress as OldRouteProgress
    from senior_control.protocol import serialize_envelope as old_serialize
    from senior_control.protocol import validate_envelope as old_validate
    from senior_control.settings.models import DrivingSettings as OldDrivingSettings

    assert (OldVelocity, OldDomainError, OldRoutePlan, OldRouteProgress, OldDrivingSettings) == (
        Velocity,
        DomainError,
        RoutePlan,
        RouteProgress,
        DrivingSettings,
    )
    assert old_serialize is serialize_envelope and old_validate is validate_envelope
    assert LAYOUT.size > 40 and callable(MotionOutput.publish)
    assert "rclpy" not in sys.modules, "installed imports must not load rclpy"
    for name in ("senior-contracts", "senior-motion-control", "senior-control"):
        distribution = importlib.metadata.distribution(name)
        assert distribution.version == "0.1.0"
        print(f"installed: {name} {distribution.version}")
    contracts = importlib.import_module("senior_contracts")
    contracts_root = Path(contracts.__file__).parent
    for path in contracts_root.glob("*.py"):
        import ast

        tree = ast.parse(path.read_text())
        for node in ast.walk(tree):
            if isinstance(node, ast.ImportFrom):
                assert not (node.module or "").startswith(("senior_control", "senior_robot", "rcl"))
    print("same-object compatibility and ROS-independent installed imports: PASS")
    try:
        for name in ("motion_output_guard", "final_command_guard"):
            executable = native_executable(name)
            assert executable.is_file(), executable
            print(f"native installed: {executable}")
    except ModuleNotFoundError as exc:
        if exc.name not in {"ament_index_python", "ament_index_python.packages"}:
            raise
        print("native installation: UNVERIFIED (ament_index_python / ROS prefix absent)")


if __name__ == "__main__":
    main()
