"""Verify real installed wheels from outside the checkout; no service is started.

Run with a clean interpreter after installing contracts, control and legacy manager
wheels. Native checks require a built ROS prefix in AMENT_PREFIX_PATH.
"""

import importlib
import importlib.metadata
import sys
from pathlib import Path


def main() -> None:
    from senior_contracts.errors import ErrorCode
    from senior_contracts.logs import LogQuery as QueryDTO
    from senior_contracts.logs import LogRequest
    from senior_contracts.navigation import RouteProgress
    from senior_contracts.wire import serialize_envelope, validate_envelope
    from senior_localization.geo import GeoAlignment
    from senior_localization.heading import HeadingObserver
    from senior_localization.navigation_reference import NavigationReferenceConfig
    from senior_localization.relocalization import Relocalizer
    from senior_mapping import CellEvidence, MapReferenceStore
    from senior_mapping.operations import MapOperations, OperationRecord
    from senior_mapping.schema import OPERATION_JOURNAL_STATEMENTS
    from senior_mapping.slam_adapter import SlamAdapter
    from senior_motion_control.motion_output import MotionOutput
    from senior_motion_control.native import native_executable
    from senior_perception.raw_safety_observation import RawSafetyObservation
    from senior_perception.scan_access import SensorAccess

    from senior_control.errors import DomainError
    from senior_control.errors import ErrorCode as LocalErrorCode
    from senior_control.hardware_navigation import (
        NavigationReferenceConfig as LegacyReferenceConfig,
    )
    from senior_control.localization import GeoAlignment as LegacyGeoAlignment
    from senior_control.localization.heading import HeadingObserver as LegacyHeadingObserver
    from senior_control.localization.relocalization import Relocalizer as LegacyRelocalizer
    from senior_control.mapping import CellEvidence as LegacyCellEvidence
    from senior_control.mapping import MapReferenceStore as LegacyMapReferenceStore
    from senior_control.mapping.operations import MapOperations as LegacyMapOperations
    from senior_control.mapping.operations import OperationRecord as LegacyOperationRecord
    from senior_control.models import Velocity
    from senior_control.navigation import RoutePlan
    from senior_control.navigation import RouteProgress as LocalRouteProgress
    from senior_control.protocol import MotionRuntimeState
    from senior_control.protocol import serialize_envelope as old_serialize
    from senior_control.protocol import validate_envelope as old_validate
    from senior_control.settings.models import DrivingSettings
    from senior_control.storage.schema import DOMAIN_MIGRATIONS
    from senior_control.vehicle_logs.models import LogQuery
    from senior_control.vehicle_logs.wire import LogRequest as LocalLogRequest

    assert LegacyCellEvidence is CellEvidence
    assert LegacyMapReferenceStore is MapReferenceStore
    assert LegacyOperationRecord is OperationRecord
    assert LegacyMapOperations is not MapOperations
    assert issubclass(LegacyMapOperations, MapOperations)
    assert LegacyMapOperations.import_bundle is MapOperations.import_bundle
    assert LegacyMapOperations.reconcile is MapOperations.reconcile
    assert DOMAIN_MIGRATIONS[0].statements is OPERATION_JOURNAL_STATEMENTS

    assert LegacyGeoAlignment is GeoAlignment
    assert LegacyHeadingObserver is HeadingObserver
    assert LegacyRelocalizer is Relocalizer
    assert LegacyReferenceConfig is NavigationReferenceConfig
    for package in ("senior_localization", "senior_mapping"):
        module = importlib.import_module(package)
        assert (Path(module.__file__).parent / "py.typed").is_file()
    assert SlamAdapter.__module__ == "senior_mapping.slam_adapter"
    assert callable(SlamAdapter.load)

    assert LocalRouteProgress is RouteProgress and LocalErrorCode is ErrorCode
    assert LocalLogRequest is LogRequest
    assert old_serialize is serialize_envelope and old_validate is validate_envelope
    for local in (DomainError, Velocity, RoutePlan, MotionRuntimeState, DrivingSettings):
        assert local.__module__.startswith("senior_control."), local
    assert issubclass(LogQuery, QueryDTO)
    assert hasattr(LogQuery, "matches") and not hasattr(QueryDTO, "matches")
    assert RawSafetyObservation.__module__ == "senior_perception.raw_safety_observation"
    assert SensorAccess.__module__ == "senior_perception.scan_access"
    assert callable(MotionOutput.publish)
    assert "rclpy" not in sys.modules, "installed imports must not load rclpy"
    for name in (
        "senior-localization",
        "senior-mapping",
        "senior-contracts",
        "senior-motion-control",
        "senior-perception",
        "senior-control",
    ):
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
    wire = importlib.import_module("senior_contracts.wire")
    for name in ("BodyReferenceState", "MotionRuntimeState", "RuntimeObservation"):
        assert not hasattr(wire, name), name
    assert not (contracts_root / "values.py").exists()
    assert not (contracts_root / "configuration.py").exists()
    print("shared-only contracts, local owners and installed compatibility: PASS")
    try:
        for name in ("motion_output_guard", "final_command_guard"):
            executable = native_executable(name)
            assert executable.is_file(), executable
            print(f"native installed: {executable}")
    except ModuleNotFoundError as exc:
        if exc.name not in {"ament_index_python", "ament_index_python.packages"}:
            raise
        print("native installation: UNVERIFIED (ament_index_python / ROS prefix absent)")

    try:
        from ament_index_python.packages import get_package_prefix
    except ModuleNotFoundError as exc:
        if exc.name not in {"ament_index_python", "ament_index_python.packages"}:
            raise
        print("SLAM installation/types: UNVERIFIED (ament_index_python / ROS prefix absent)")
    else:
        runtime = Path(get_package_prefix("senior_mapping")) / "lib" / "senior_mapping"
        interfaces = Path(get_package_prefix("senior_slam_bridge"))
        for name in ("slam_graph_node", "acquire_scan_receipt", "candidate_preflight"):
            executable = runtime / name
            assert executable.is_file() and executable.stat().st_mode & 0o111, executable
            assert not (interfaces / "lib" / "senior_slam_bridge" / name).exists()
            print(f"SLAM installed: {executable}")
        message = importlib.import_module("senior_slam_bridge.msg")
        services = importlib.import_module("senior_slam_bridge.srv")
        assert message.Keyframe.__module__.startswith("senior_slam_bridge.msg.")
        for name in (
            "CaptureGraph",
            "ReadGraph",
            "ReadGraphCommit",
            "ReadLoadedGraph",
            "MatchStationaryScan",
        ):
            assert getattr(services, name).__module__.startswith("senior_slam_bridge.srv.")
        print("SLAM runtime prefix and original interface package: PASS (not a DDS probe)")


if __name__ == "__main__":
    main()
