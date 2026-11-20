"""Verify real installed wheels from outside the checkout; no service is started.

Run with a clean interpreter after installing contracts, control and legacy manager
wheels. Native checks require a built ROS prefix in AMENT_PREFIX_PATH.
"""

import importlib
import importlib.metadata
import sys
from pathlib import Path


def verify_lazy_shared_imports() -> None:
    """Fresh processes check the actual installed import graph and public identities."""
    import subprocess

    scripts = (
        """
import json
import sys
from pathlib import Path
from pydantic import ValidationError
from senior_contracts.errors import ErrorCode
from vehicle_core import errors, runtime_state, values
for module in (errors, runtime_state, values):
    assert 'site-packages' in str(Path(module.__file__).resolve()), module.__file__
for name in ('senior_control', 'vehicle_manager', 'cloud', 'rclpy', 'launch',
             'senior_robot', 'senior_motion_control', 'senior_perception'):
    assert name not in sys.modules, name
assert errors.ErrorCode is ErrorCode
assert errors.DomainError(ErrorCode.UNAVAILABLE, 'unavailable').code is ErrorCode.UNAVAILABLE
assert values.OperationResult('operation', 'running').status is values.OperationStatus.RUNNING
packet = dict(
    schema=1, robot_id='robot', boot_id='boot', sequence=1, sampled_steady=10.0,
    mode='idle', reason='', stationary=True, healthy=True, evidence_stamp_steady=None,
    evidence_deadline_steady=None, estop_revision=0, fault_revision=0,
    confirmed_estop_status='clear', allowed_actions=['acquire'], task_id=None,
    output_generation=0, body_reference=None, wheel_linear_mps=None,
)
state = runtime_state.MotionRuntimeState.model_validate(packet)
assert state.schema_version == 1
assert state.model_dump() == json.loads(state.model_dump_json()) == packet
assert 'schema' in state.model_json_schema()['properties']
assert 'schema_version' not in state.model_dump()
for change in (dict(schema=2), dict(sequence=-1), dict(sampled_steady=float('inf')),
               dict(evidence_stamp_steady=9.0), dict(extra_field=1)):
    try:
        runtime_state.MotionRuntimeState.model_validate({**packet, **change})
    except ValidationError:
        pass
    else:
        raise AssertionError(change)
try:
    runtime_state.BodyReferenceState(
        average={'vx': 0.0}, source='wheel', computed_steady=9.0, valid_until_steady=10.0
    )
except ValidationError:
    pass
else:
    raise AssertionError('incomplete body reference accepted')
from senior_control import errors as old_errors, models as old_values, protocol as old_state
for name in ('_finite', 'Velocity', 'BodyTwist', 'Pose2D', 'MapRef', 'SensorHealth',
             'OperationStatus', 'OperationResult'):
    assert getattr(old_values, name) is getattr(values, name), name
for name in ('ErrorCode', 'DomainError'):
    assert getattr(old_errors, name) is getattr(errors, name), name
for name in ('BodyReferenceState', 'RuntimeObservation', 'MotionRuntimeState'):
    assert getattr(old_state, name) is getattr(runtime_state, name), name
assert old_state.MotionRuntimeState.model_validate_json(state.model_dump_json()) == state
for name in ('vehicle_manager', 'cloud', 'rclpy', 'launch', 'senior_control.agent',
             'senior_control.config', 'senior_control.runtime.diagnostics'):
    assert name not in sys.modules, name
""",
        """
import sys
from pathlib import Path
from vehicle_core import ipc
assert 'site-packages' in str(Path(ipc.__file__).resolve()), ipc.__file__
assert ipc.IpcServer.__module__ == ipc.IpcClient.__module__ == 'vehicle_core.ipc'
assert ipc.logger.name == 'senior_control.runtime.ipc'
assert (ipc.PROTOCOL_VERSION, ipc.MAX_FRAME, ipc.MAX_PATH) == (1, 8192, 100)
assert ipc._HEADER.pack(5) == bytes((0, 0, 0, 5))
for name in ('senior_control', 'vehicle_manager', 'cloud', 'rclpy', 'launch',
             'senior_robot', 'senior_motion_control', 'senior_perception'):
    assert name not in sys.modules, name
from vehicle_core import diagnostics
assert ipc.PendingCounter is diagnostics.PendingCounter
assert ipc.PendingCounter('install.ipc').diagnostics is diagnostics.process_diagnostics()
from senior_control.runtime import ipc as old
for name in ('logger', 'PROTOCOL_VERSION', 'MAX_FRAME', 'MAX_PATH', '_HEADER',
             'Message', 'Handler', 'IpcError', '_check_path', '_encode', '_finite_float',
             '_read', 'IpcServer', 'IpcClient', '_identity', '_record', '_prepare_socket'):
    assert getattr(old, name) is getattr(ipc, name), name
for name in ('vehicle_manager', 'cloud', 'rclpy', 'launch', 'senior_control.agent',
             'senior_control.config', 'senior_control.sensors.policy'):
    assert name not in sys.modules, name
""",
        """
import sys
from pathlib import Path
from unittest.mock import patch
from vehicle_core import diagnostics
assert 'site-packages' in str(Path(diagnostics.__file__).resolve()), diagnostics.__file__
assert diagnostics.RuntimeDiagnostics.__module__ == 'vehicle_core.diagnostics'
for name in ('senior_control', 'vehicle_manager', 'cloud', 'rclpy', 'launch',
             'senior_motion_control', 'senior_perception'):
    assert name not in sys.modules, name
with patch('logging.getLogger') as logger:
    diagnostics.RuntimeDiagnostics()._emit(20, {})
    logger.assert_called_once_with('senior_control.runtime.diagnostics')
from senior_control.runtime import diagnostics as old
for name in ('_LIMIT', 'QueueSnapshot', 'QueueWarningPolicy', '_Stats', '_QueueState',
             'RuntimeDiagnostics', 'DiagnosticsSettings', 'diagnostics_from_environment',
             'configure_diagnostics', 'process_diagnostics', 'PendingCounter', 'T',
             'DiagnosticDeque', 'DiagnosticAsyncQueue', 'K', 'V', 'DiagnosticPending',
             'DiagnosticLock'):
    assert getattr(old, name) is getattr(diagnostics, name), name
settings = diagnostics.DiagnosticsSettings(period_s=3.0)
owner = diagnostics.configure_diagnostics(settings)
assert old.configure_diagnostics(settings) is owner
assert old.process_diagnostics() is diagnostics.process_diagnostics() is owner
assert diagnostics.process_diagnostics.cache_info().maxsize == 1
assert owner.period == 3.0
for name in ('vehicle_manager', 'cloud', 'rclpy', 'launch', 'senior_control.agent',
             'senior_control.config', 'senior_control.sensors.policy'):
    assert name not in sys.modules, name
""",
        """
import sys
from pathlib import Path
from importlib.resources import files
from vehicle_core import sample_time, sensor_policy
for module in (sample_time, sensor_policy):
    assert 'site-packages' in str(Path(module.__file__).resolve()), module.__file__
assert sample_time.SampleTime.__module__ == 'vehicle_core.sample_time'
assert sensor_policy.SensorTiming.__module__ == 'vehicle_core.sensor_policy'
resource = files('vehicle_core').joinpath('sensor_defaults.json')
assert resource.read_bytes() == (
    b'{"soft_age_ms": 300, "hard_age_ms": 1000, "diagnostics_period_s": 5.0}' + bytes((10,))
)
assert not files('senior_control.sensors').joinpath('defaults.json').is_file()
for name in ('vehicle_manager', 'cloud', 'rclpy', 'launch',
             'senior_control.runtime.diagnostics', 'senior_control.sensors.policy'):
    assert name not in sys.modules, name
from senior_control.sensors import sample_time as old_time, policy as old_policy
for name in ('_INT64_MAX', '_UINT64_MAX', '_coordinate', 'SampleTime', 'map_sample',
             'StampInterval', 'interval_ns', 'ClockObservation', 'ClockTracker'):
    assert getattr(old_time, name) is getattr(sample_time, name), name
for name in ('Age', 'SensorTiming', 'SensorClockError', 'default_policy', 'sensor_current'):
    assert getattr(old_policy, name) is getattr(sensor_policy, name), name
assert old_policy.default_policy() is sensor_policy.default_policy()
assert sensor_policy.default_policy.cache_info().maxsize == 1
assert old_policy.ros_sensor_parameters.__module__ == 'senior_control.sensors.policy'
for name in ('vehicle_manager', 'cloud', 'rclpy', 'launch',
             'senior_control.runtime.diagnostics'):
    assert name not in sys.modules, name
""",
        """
import sys
from pathlib import Path
from vehicle_core import lifecycle_events, process_resources
for module in (lifecycle_events, process_resources):
    assert 'site-packages' in str(Path(module.__file__).resolve()), module.__file__
assert lifecycle_events.logger.name == 'senior_control.runtime.lifecycle_events'
assert lifecycle_events.READ_FD == 'SENIOR_RUNTIME_EVENTS_READ_FD'
assert lifecycle_events.WRITE_FD == 'SENIOR_RUNTIME_EVENTS_WRITE_FD'
assert lifecycle_events.MAX_EVENT_BYTES == 2048
assert lifecycle_events.LifecycleEvents.__module__ == 'vehicle_core.lifecycle_events'
assert process_resources.ProcessWorker.__module__ == 'vehicle_core.process_resources'
for name in ('senior_control', 'vehicle_manager', 'cloud', 'rclpy', 'launch',
             'senior_motion_control', 'senior_perception'):
    assert name not in sys.modules, name
from senior_control.runtime import lifecycle_events as old_events
from senior_control.motion import process_resources as old_resources
for name in ('logger', 'READ_FD', 'WRITE_FD', 'MAX_EVENT_BYTES', 'emit',
             'LifecycleEvents', 'report_ros_exit', 'observe'):
    assert getattr(old_events, name) is getattr(lifecycle_events, name), name
for name in ('SnapshotT', 'ResultT', 'WorkerReply', 'LatestSnapshot', 'child_lifetime',
             'stop_process', '_refinement_loop', '_ProcessFuture', 'ProcessWorker'):
    assert getattr(old_resources, name) is getattr(process_resources, name), name
for name in ('vehicle_manager', 'cloud', 'rclpy', 'launch',
             'senior_control.agent', 'senior_control.runtime.launch_recovery',
             'senior_control.runtime.diagnostics', 'senior_control.sensors.policy'):
    assert name not in sys.modules, name
""",
        """
import sys
from vehicle_core.clock import Clock, SystemClock
from vehicle_core.image_frame import ImageFrame
assert Clock.__module__ == SystemClock.__module__ == 'vehicle_core.clock'
assert ImageFrame.__module__ == 'vehicle_core.image_frame'
assert isinstance(SystemClock().monotonic(), float)
assert ImageFrame(0.0, 1, 1, 'rgb8', b'abc').data == b'abc'
for name in ('senior_control', 'vehicle_manager', 'cloud', 'rclpy'):
    assert name not in sys.modules, name
from senior_control.clock import Clock as OldClock, SystemClock as OldSystemClock
from senior_control.image_frame import ImageFrame as OldImageFrame
assert OldClock is Clock and OldSystemClock is SystemClock
assert OldImageFrame is ImageFrame
for name in ('senior_control.agent', 'senior_control.vision',
             'senior_control.runtime.diagnostics', 'senior_control.sensors.policy',
             'vehicle_manager', 'cloud', 'rclpy'):
    assert name not in sys.modules, name
""",
        """
import sys
from vehicle_core.image_frame import ImageFrame
from senior_control.image_frame import ImageFrame as OldImageFrame
assert OldImageFrame is ImageFrame
assert ImageFrame.__module__ == 'vehicle_core.image_frame'
from senior_control.vision import rgb1
assert rgb1.ImageFrame is ImageFrame
for name in ('depth', 'detector', 'tracking', 'synchronizer'):
    assert 'senior_control.vision.' + name not in sys.modules, name
assert 'senior_control.runtime.diagnostics' not in sys.modules
assert 'senior_control.sensors.policy' not in sys.modules
""",
        """
import sys
from senior_control.agent.manual_control import ManualIngress, ManualTarget, MotionRuntimeReceiver
assert all(callable(c) for c in (ManualIngress, ManualTarget, MotionRuntimeReceiver))
assert 'senior_control.agent.main' not in sys.modules
assert 'vehicle_manager.agent.frame_projection' not in sys.modules
assert 'vehicle_manager.agent.state_ports' not in sys.modules
""",
        """
from importlib import import_module
from senior_control import agent, vision
from senior_control.agent import main, snapshots
from senior_control.image_frame import ImageFrame
from senior_control.vision.synchronizer import ImageFrame as OldFrame
assert agent.Agent is main.Agent
assert agent.RobotSnapshot is snapshots.RobotSnapshot
assert vision.ImageFrame is OldFrame is ImageFrame
owners = {
    'vehicle_core.image_frame': ('ImageFrame',),
    'senior_control.vision.depth': (
        'CalibrationError', 'CameraCalibration', 'CameraExtrinsic', 'ColumnStatus',
        'GeometryConfig', 'GeometryObservation', 'GeometryPoint', 'project_depth'),
    'senior_control.vision.detector': (
        'Detection', 'DetectionBatch', 'Detector', 'DetectorDisabled', 'DisabledDetector',
        'InferenceWorker', 'load_detector'),
    'senior_control.vision.synchronizer': (
        'AlignedRgbdFrame', 'DepthFrame', 'FrameSyncError', 'RgbdSynchronizer', 'align'),
    'senior_control.vision.tracking': ('DynamicMask', 'Tracker', 'TrackingConfig', 'track'),
}
assert set(vision.__all__) == {name for names in owners.values() for name in names}
for owner, names in owners.items():
    module = import_module(owner)
    for name in names:
        assert getattr(vision, name) is getattr(module, name), name
""",
    )
    for script in scripts:
        subprocess.run([sys.executable, "-c", script], cwd="/tmp", check=True, timeout=15)
    print("installed vehicle_core leaves, legacy identities and lazy execution/media imports: PASS")


def verify_navigation() -> None:
    """Verify the real SDK aliases, then ROS prefix paths when ROS is installed."""
    from senior_navigation import navigation_adapter, route
    from senior_robot import navigation_adapter as old_adapter

    from senior_control import navigation as old_route
    from senior_control.errors import DomainError, ErrorCode
    from senior_control.models import MapRef, Pose2D

    for name in ("RoutePlan", "QualifiedRoute", "RouteRejected", "RouteProgress", "RouteStatus"):
        assert getattr(old_route, name) is getattr(route, name)
    for name in ("NavigationAdapter", "RouteService", "MapGoal", "NavigationRejected", "main"):
        assert getattr(old_adapter, name) is getattr(navigation_adapter, name)
    assert route.MapRef is MapRef and route.Pose2D is Pose2D
    assert issubclass(route.RouteRejected, DomainError)
    assert route.RouteRejected().code is ErrorCode.UNAVAILABLE
    assert route.RoutePlan.__module__ == "vehicle_core.route_plan"
    assert navigation_adapter.RouteService.__module__ == "senior_navigation.navigation_adapter"
    print("installed navigation canonical route and historical SDK identities: PASS")
    try:
        from ament_index_python.packages import get_package_prefix, get_package_share_directory
    except ModuleNotFoundError as exc:
        if exc.name not in {"ament_index_python", "ament_index_python.packages"}:
            raise
        print("navigation ROS executable/share installation: UNVERIFIED (ament/ROS prefix absent)")
        return
    from senior_navigation.dedicated_controller import DedicatedController
    from senior_robot.dedicated_controller import DedicatedController as OldController

    assert OldController is DedicatedController
    nav = Path(get_package_share_directory("senior_navigation"))
    robot = Path(get_package_share_directory("senior_robot"))
    for package in ("senior_navigation", "senior_robot"):
        executable = Path(get_package_prefix(package)) / "lib" / package / "navigation_adapter"
        assert executable.is_file() and executable.stat().st_mode & 0o111, executable
    for asset in (
        "config/nav2.yaml",
        "config/lattice_diff.json",
        "behavior_trees/navigate.xml",
        "behavior_trees/navigate_through_poses.xml",
    ):
        assert (nav / asset).read_bytes() == (robot / asset).read_bytes(), asset
    for share in (nav, robot):
        assert (share / "launch/navigation.launch.py").is_file()
    print("installed navigation new/old ROS executable and byte-identical share assets: PASS")


def verify_platform_runtime() -> None:
    """Locate the installed real provider without management or ROS imports."""
    from senior_robot import platform_runtime

    for name in (
        "_adapter_command",
        "_ros_launch_command",
        "_motion_command",
        "_camera_command",
        "_video_environment",
    ):
        assert callable(getattr(platform_runtime, name))
    assert not any(
        name == "senior_control" or name.startswith(("senior_control.", "vehicle_manager", "rclpy"))
        for name in sys.modules
    )
    print(
        "installed execution platform provider without Root/manager/ROS:", platform_runtime.__file__
    )


def verify_manager_core_distribution() -> None:
    """The installed manager depends on the core wheel, never its legacy business."""
    import re

    import vehicle_core

    manager = importlib.metadata.distribution("vehicle-manager")
    core = importlib.metadata.distribution("senior-control")
    assert "senior-control==0.1.0" in (manager.requires or [])
    requirements = core.requires or []
    names = {re.split(r"[<>=!~;\[ ]", item, maxsplit=1)[0].lower() for item in requirements}
    assert names == {
        "senior-contracts",
        "pydantic",
        "pydantic-settings",
        "pyyaml",
        "aiosqlite",
        "httpx",
        "websockets",
        "aiortc",
        "shapely",
        "numpy",
        "fastapi",
    }, names

    def extra_requirements(distribution, extra):
        return {
            item.split(";", 1)[0].strip()
            for item in distribution.requires or []
            if re.search(rf"extra == ['\"]{extra}['\"]", item)
        }

    for extra in ("agent", "video"):
        assert extra_requirements(core, extra) == extra_requirements(manager, extra), extra
    location = Path(vehicle_core.__file__).resolve()
    assert "site-packages" in str(location), location
    assert location == Path(core.locate_file("vehicle_core/__init__.py")).resolve()
    assert Path("vehicle_core/__init__.py") in (core.files or [])
    # //Compatible: the legacy namespace is shipped in the same core wheel.
    assert importlib.util.find_spec("senior_control") is not None
    assert not any(
        name == "senior_control" or name.startswith("senior_control.") for name in sys.modules
    )
    for name in ("cloud", "rclpy", "senior_mapping", "senior_robot"):
        assert importlib.util.find_spec(name) is None, name
    installed = {item.metadata["Name"].lower() for item in importlib.metadata.distributions()}
    assert not installed.intersection(
        {
            "vehicle-cloud",
            "senior-localization",
            "senior-mapping",
            "senior-motion-control",
            "senior-motion-estimation",
            "senior-navigation",
            "senior-perception",
            "senior-robot",
            "rclpy",
        }
    ), installed
    print("installed manager -> core metadata, wheel ownership and no legacy/ROS load: PASS")


def verify_manager_config(*, compatible: bool = False) -> None:
    """Check the real installed manager leaves and optional original SDK binding."""
    import inspect
    from dataclasses import FrozenInstanceError, fields
    from importlib.util import find_spec

    from pydantic import ValidationError
    from vehicle_manager.agent.config import AgentConfig
    from vehicle_manager.agent.pose_reply import PoseReply

    from vehicle_core.diagnostics import DiagnosticsSettings

    for value in (AgentConfig, PoseReply):
        assert "site-packages" in inspect.getfile(value), inspect.getfile(value)
    if not compatible:
        verify_manager_core_distribution()
        for package in ("cloud", "rclpy", "senior_robot"):
            assert find_spec(package) is None, package
    environment = {
        "SENIOR_ROBOT_ID": "robot",
        "SENIOR_CLOUD_URL": "wss://cloud.test/ws/agent",
        "SENIOR_DEVICE_TOKEN": "private-config-token",
        "SENIOR_DATABASE_PATH": "/tmp/config.db",
        "SENIOR_AGENT_SOCKET": "/tmp/config.sock",
        "SENIOR_UNRELATED": "ignored",
    }
    config = AgentConfig.from_environment(environment)
    assert type(config) is AgentConfig and config.database_path == Path("/tmp/config.db")
    assert "private-config-token" not in repr(config)
    assert type(config.diagnostics) is DiagnosticsSettings
    assert not hasattr(AgentConfig, "_environment_fields")
    assert not next(field for field in fields(config) if field.name == "device_token").repr
    try:
        config.robot_id = "changed"
    except FrozenInstanceError:
        pass
    else:
        raise AssertionError("configuration is mutable")
    for change in (
        {"SENIOR_CLOUD_URL": "ws://cloud.test"},
        {"SENIOR_CLOUD_URL": "wss://user:password@cloud.test"},
        {"SENIOR_DATABASE_PATH": "relative.db"},
        {"SENIOR_WORKSPACE_MAP_BUNDLE": "/tmp/map.zip"},
    ):
        try:
            AgentConfig.from_environment({**environment, **change})
        except ValueError:
            pass
        else:
            raise AssertionError(change)
    pose = dict(
        v=1,
        ok=True,
        sampled_at=10.0,
        healthy=True,
        map_id="map",
        reference_revision=1,
        anchor_id="anchor",
        x=0.0,
        y=0.0,
        yaw=0.0,
        anchor_x=0.0,
        anchor_y=0.0,
        anchor_yaw=0.0,
    )
    assert PoseReply.model_validate(pose).map_generation is None
    for change in (
        {"healthy": False},
        {"sampled_at": float("inf")},
        {"reference_revision": "1"},
        {"extra": 1},
    ):
        try:
            PoseReply.model_validate({**pose, **change})
        except ValidationError:
            pass
        else:
            raise AssertionError(change)
    names = (
        "robot_id",
        "cloud_url",
        "device_token",
        "database_path",
        "socket_path",
        "workspace_map_bundle",
        "workspace_map_sha256",
        "simulation_video_frame",
        "ros_video_topic",
        "simulation_settings",
        "diagnostics",
    )
    assert tuple(inspect.signature(AgentConfig).parameters) == names
    diagnostic = next(field for field in fields(AgentConfig) if field.name == "diagnostics")
    assert diagnostic.default_factory is DiagnosticsSettings
    old = AgentConfig(
        "robot",
        environment["SENIOR_CLOUD_URL"],
        environment["SENIOR_DEVICE_TOKEN"],
        Path("/tmp/config.db"),
        Path("/tmp/config.sock"),
    )
    assert type(old.diagnostics) is DiagnosticsSettings
    parsed = AgentConfig.from_environment({**environment, "SENIOR_DIAGNOSTICS__QUEUE__COUNT": "7"})
    assert type(parsed) is AgentConfig and parsed.diagnostics.queue.count == 7
    assert "private-config-token" not in repr(parsed)
    missing = {key: value for key, value in environment.items() if key != "SENIOR_ROBOT_ID"}
    try:
        AgentConfig.from_environment({**missing, "SENIOR_DIAGNOSTICS__QUEUE__COUNT": "0"})
    except ValueError as exc:
        assert str(exc) == "missing environment variable SENIOR_ROBOT_ID"
        assert isinstance(exc.__cause__, KeyError)
    else:
        raise AssertionError("missing required environment did not fail first")
    try:
        AgentConfig.from_environment(
            {
                **environment,
                "SENIOR_CLOUD_URL": "ws://cloud.test",
                "SENIOR_DIAGNOSTICS__QUEUE__COUNT": "0",
            }
        )
    except ValidationError:
        pass
    else:
        raise AssertionError("diagnostic validation did not precede constructor validation")
    print("installed config signature, default diagnostics and validation priority: PASS")
    if compatible:
        from senior_control.agent.composition import AgentConfig as OldConfig
        from senior_control.agent.ports import PoseReply as OldPoseReply

        assert OldConfig is AgentConfig
        assert OldConfig.__post_init__ is AgentConfig.__post_init__
        assert OldConfig.from_environment.__func__ is AgentConfig.from_environment.__func__
        assert OldPoseReply is PoseReply
    else:
        for name in ("senior_control", "cloud", "rclpy", "senior_robot"):
            assert name not in sys.modules, name
        print("installed manager config/PoseReply without legacy SDK or ROS: PASS")


def verify_route_execution(*, compatible: bool = False) -> None:
    """Assemble actual manager execution types and check shared route validation offline."""
    import asyncio
    import inspect
    import tempfile
    from unittest.mock import AsyncMock

    from pydantic import ValidationError
    from vehicle_manager.agent.composition import build_task_runtime
    from vehicle_manager.agent.ipc_navigation import IpcNavigation as GenericNavigation
    from vehicle_manager.agent.local_adapters import IpcNavigation
    from vehicle_manager.storage.bindings import Database
    from vehicle_manager.storage.core_schema import migrate_core
    from vehicle_manager.storage.migrations import Migration, migrate
    from vehicle_manager.tasks import execution_bindings
    from vehicle_manager.tasks.executor import TaskExecutor as GenericExecutor
    from vehicle_manager.tasks.navigation_port import NavigationEffects as GenericEffects
    from vehicle_manager.tasks.ports import TaskStorePorts
    from vehicle_manager.tasks.store import TASK_STATEMENTS, TaskStore

    from vehicle_core.errors import DomainError, ErrorCode
    from vehicle_core.ipc import IpcError
    from vehicle_core.route_plan import RoutePlan, RouteRejected
    from vehicle_core.values import MapRef, Pose2D

    for value in (RoutePlan, RouteRejected, IpcNavigation, execution_bindings.TaskExecutor):
        assert "site-packages" in inspect.getfile(value), value
    assert RoutePlan.__module__ == RouteRejected.__module__ == "vehicle_core.route_plan"
    assert Path("vehicle_core/route_plan.py") in (
        importlib.metadata.distribution("senior-control").files or []
    )
    assert RouteRejected().code is ErrorCode.UNAVAILABLE
    assert str(RouteRejected()) == "qualified route was not admitted"
    packet = {
        "execution_id": "execution",
        "map_id": "yard",
        "reference_revision": 1,
        "task_epoch": 1,
        "effect_id": "plan",
        "map_generation": "generation",
        "action_ids": ["point"],
        "poses": [[1.0, 2.0, 0.0]],
    }
    plan = RoutePlan.model_validate(packet)
    assert plan.map_ref == MapRef("yard", 1) and plan.task_id is None
    assert RoutePlan.model_validate_json(plan.model_dump_json()) == plan
    for change in (
        {"execution_id": ""},
        {"effect_id": "x" * 129},
        {"reference_revision": -1},
        {"task_epoch": 0},
        {"poses": []},
        {"poses": [[1.0, 2.0]]},
        {"poses": [[float("nan"), 2.0, 0.0]]},
        {"poses": [["1", 2.0, 0.0]]},
        {"action_ids": []},
        {"action_ids": ["same", "same"], "poses": [[0.0] * 3] * 2},
        {"unknown": 1},
    ):
        try:
            RoutePlan.model_validate({**packet, **change})
        except ValidationError:
            pass
        else:
            raise AssertionError(change)
    for bound, generic, methods in (
        (
            IpcNavigation,
            GenericNavigation,
            ("_call", "task_poses", "task_effect", "task_effect_status"),
        ),
        (
            execution_bindings.TaskExecutor,
            GenericExecutor,
            ("control", "current_context", "manual_handoff_matches"),
        ),
        (execution_bindings.NavigationEffects, GenericEffects, ("execute", "query", "resolve")),
    ):
        assert bound is not generic and issubclass(bound, generic)
        for method in methods:
            assert getattr(bound, method) is getattr(generic, method)

    async def verify(directory):
        async with Database(directory / "execution.db") as database:

            async def prepare_schema():
                await migrate(database, [Migration(1, TASK_STATEMENTS)])
                await migrate_core(database)

            store = TaskStore(
                database,
                robot_id="robot",
                ports=TaskStorePorts(
                    prepare_schema=prepare_schema,
                    map_reference=MapRef,
                    error=DomainError,
                ),
            )
            await store.prepare()
            connection, lock = database.connection, database.write_lock
            client = AsyncMock()
            navigation = IpcNavigation(client, tasks=store)
            assert navigation._client is client and navigation._tasks is store
            assert navigation._values.route_plan is RoutePlan
            assert navigation._values.route_rejected is RouteRejected
            assert navigation._values.pose is Pose2D
            assert (
                navigation._values.error is DomainError and navigation._values.ipc_error is IpcError
            )
            runtime, executor = build_task_runtime(
                database,
                store,
                navigation,
                boot_id="installed-execution",
                executor_factory=execution_bindings.TaskExecutor,
                navigation_effects=execution_bindings.NavigationEffects,
            )
            assert type(executor) is execution_bindings.TaskExecutor
            assert runtime._executor is executor and runtime._store is store
            assert executor._database is database and executor._store is store
            assert (
                executor.errors.domain is DomainError
                and executor.errors.route_rejected is RouteRejected
            )
            assert (
                executor.errors.qualification_unavailable
                is execution_bindings.QualificationUnavailable
            )
            assert executor.errors.code is execution_bindings._error_code
            assert executor.errors.code(RouteRejected()) is ErrorCode.UNAVAILABLE
            effects = executor.lifecycle.port
            assert type(effects) is execution_bindings.NavigationEffects
            assert effects.navigation is navigation and effects._rejection_type is RouteRejected
            assert effects.timeout == 0.3
            assert effects.store.database is database and effects.store.tasks is store
            await effects.store.prepare()
            assert database.connection is connection and database.write_lock is lock
            client.request.return_value = {
                "ok": False,
                "error": "rejected",
                "execution_id": "execution",
            }
            try:
                await navigation._call("navigation.task_start", "execution")
            except RouteRejected as exc:
                assert type(exc) is RouteRejected
            else:
                raise AssertionError("route refusal did not retain its original type")
            assert (
                inspect.signature(execution_bindings.NavigationEffects)
                .parameters["timeout_ms"]
                .default
                == 300.0
            )

    with tempfile.TemporaryDirectory(prefix="installed-execution-", dir="/tmp") as directory:
        asyncio.run(verify(Path(directory)))
    if compatible:
        from senior_navigation import navigation_adapter, route
        from senior_robot import agent_adapter
        from senior_robot import navigation_adapter as old_adapter

        from senior_control import navigation as old_route
        from senior_control.agent import composition, ports
        from senior_control.tasks import executor as old_executor
        from senior_control.tasks import navigation_port as old_effects
        from senior_control.tasks.store import TaskStore as LegacyStore
        from senior_control.waypoints.store import WaypointStore

        for owner in (old_route, route):
            assert owner.RoutePlan is RoutePlan and owner.RouteRejected is RouteRejected
        assert route.QualifiedRoute.__module__ == "senior_navigation.route"
        assert (
            navigation_adapter.RoutePlan
            is agent_adapter.RoutePlan
            is old_adapter.RoutePlan
            is RoutePlan
        )
        assert ports.IpcNavigation is composition.IpcNavigation is IpcNavigation
        assert (
            old_executor.TaskExecutor is composition.TaskExecutor is execution_bindings.TaskExecutor
        )
        assert old_executor.QualificationUnavailable is execution_bindings.QualificationUnavailable
        assert old_executor._error_code is execution_bindings._error_code
        assert (
            old_effects.NavigationEffects
            is composition.NavigationEffects
            is execution_bindings.NavigationEffects
        )
        database = Database(Path("/tmp/installed-execution-not-opened"))
        tasks, points = (
            LegacyStore(database, robot_id="robot"),
            WaypointStore(database, robot_id="robot"),
        )
        navigation = IpcNavigation(AsyncMock(), points, tasks=tasks)
        assert navigation._waypoints is points and navigation._tasks is tasks
        runtime, executor = composition.build_task_runtime(
            database, tasks, navigation, boot_id="installed"
        )
        assert type(executor) is execution_bindings.TaskExecutor and runtime._executor is executor
        assert type(executor.lifecycle.port) is execution_bindings.NavigationEffects
    else:
        assert not any(
            name == "senior_control"
            or name.startswith(
                (
                    "senior_control.",
                    "cloud",
                    "rclpy",
                    "launch",
                    "nav2",
                    "senior_mapping",
                    "senior_navigation",
                    "senior_robot",
                )
            )
            for name in sys.modules
        )
    print(
        "installed core route schema, concrete execution and real manager-only assembly: PASS",
        compatible,
    )


def verify_production_bindings(*, compatible: bool = False) -> None:
    """Use the installed production constructors without importing the legacy composition."""
    import asyncio
    import inspect
    import tempfile
    from typing import get_args
    from unittest.mock import AsyncMock

    from vehicle_manager.agent import (
        journal,
        logs_wire,
        production_bindings,
        settings_wire,
        snapshots,
        websocket,
    )
    from vehicle_manager.agent.local_adapters import IpcState
    from vehicle_manager.settings.models import DrivingSettings, LoggingSettings, SettingsSnapshot
    from vehicle_manager.storage import bindings
    from vehicle_manager.storage.database import Database as Engine
    from vehicle_manager.storage.home_database import HomeDatabase, HomeExecutionCoordination
    from vehicle_manager.vehicle_logs.service import VehicleLogService as LogOwner
    from vehicle_manager.vehicle_logs.store import VehicleLogStore

    from vehicle_core.diagnostics import (
        DiagnosticAsyncQueue,
        DiagnosticLock,
        PendingCounter,
        process_diagnostics,
    )
    from vehicle_core.errors import DomainError
    from vehicle_core.values import OperationResult, OperationStatus

    for bound, base, methods in (
        (bindings.Database, HomeDatabase, ("__aenter__", "__aexit__", "home_execution")),
        (bindings.HomeExecutionCoordination, HomeExecutionCoordination, ()),
        (
            production_bindings.AgentJournal,
            journal.AgentJournal,
            ("prepare", "get", "begin", "finish"),
        ),
        (production_bindings.AgentLogConnection, logs_wire.AgentLogConnection, ("serve", "run")),
        (production_bindings.SettingsRuntime, settings_wire.SettingsRuntime, ("refresh", "check")),
        (
            production_bindings.WebSocketLink,
            websocket.WebSocketLink,
            ("recv", "send_heartbeat", "close"),
        ),
        (production_bindings.WebSocketConnector, websocket.WebSocketConnector, ("__call__",)),
        (production_bindings.VehicleLogService, LogOwner, ("start", "enqueue", "query", "close")),
    ):
        assert "site-packages" in inspect.getfile(bound), bound
        assert bound is not base and issubclass(bound, base)
        for name in methods:
            assert getattr(bound, name) is getattr(base, name)
    assert bindings.Database.__aenter__ is Engine.__aenter__
    assert get_args(bindings.HomeExecutionCoordination.__orig_bases__[0]) == (OperationResult,)
    assert tuple(inspect.signature(bindings.Database).parameters) == ("path", "busy_timeout_ms")
    assert inspect.signature(bindings.Database).parameters["busy_timeout_ms"].default == 1000
    assert not inspect.signature(bindings.HomeExecutionCoordination).parameters
    assert production_bindings._OPEN == (
        OperationStatus.ACCEPTED,
        OperationStatus.DISPATCHING,
        OperationStatus.RUNNING,
    )

    async def verify(directory):
        database = bindings.Database(directory / "bound.db")
        write_lock = database.write_lock
        assert type(write_lock) is DiagnosticLock
        assert write_lock._diagnostic.diagnostics is process_diagnostics()
        assert database._home_coordination_factory is bindings.HomeExecutionCoordination
        coordination = database.home_execution("robot")
        assert type(coordination) is bindings.HomeExecutionCoordination
        assert database.home_execution("robot") is coordination
        assert type(coordination.lock) is DiagnosticLock
        assert coordination.lock._diagnostic.diagnostics is process_diagnostics()
        assert database.home_target_lock("robot") is database.home_target_lock("robot")
        async with database:
            connection = database.connection
            bound = production_bindings.AgentJournal(database, robot_id="robot")
            assert bound._database is database and database.write_lock is write_lock
            assert bound._ports.entry is production_bindings.JournalEntry
            assert (
                bound._ports.status is OperationStatus
                and bound._ports.active is production_bindings._OPEN
            )
            await bound.prepare()
            assert await bound.begin("installed-binding", "fingerprint") is None
            entry = await bound.get("installed-binding")
            assert type(entry) is production_bindings.JournalEntry
            assert entry.status is OperationStatus.RUNNING and not entry.closed
            await bound.finish("installed-binding", OperationStatus.SUCCEEDED, revision=1)
            entry = await bound.get("installed-binding")
            assert entry.closed and entry.revision == 1
            assert database.connection is connection and database.write_lock is write_lock
        assert not database.is_open
        snapshot = SettingsSnapshot(
            driving=DrivingSettings(
                low_linear_mps=0.5,
                high_linear_mps=1,
                low_angular_rps=1,
                high_angular_rps=1,
                reverse_linear_mps=0.3,
            ),
            logging=LoggingSettings(),
        )
        grants = logs_wire.CloudLogGrants()
        store = VehicleLogStore(directory / "logs")
        service = production_bindings.VehicleLogService(store, "robot", grants, lambda: snapshot)
        assert service._store is store and service._authorizer is grants
        assert service._diagnostics.lock is DiagnosticLock
        assert service._diagnostics.counter is PendingCounter
        assert service._diagnostics.outbox is DiagnosticAsyncQueue
        assert type(service._queue_diagnostic) is PendingCounter
        assert service._queue_diagnostic.diagnostics is process_diagnostics()
        assert type(service._publication_lock) is DiagnosticLock
        assert service._publication_lock._diagnostic.diagnostics is process_diagnostics()
        assert service._writer is None and service._cleaner is None
        logs = production_bindings.AgentLogConnection(service, grants)
        assert logs.service is service and logs.grants is grants
        assert logs._lock_factory is DiagnosticLock and logs._counter_factory is PendingCounter
        state = IpcState(AsyncMock())
        tasks = AsyncMock()
        runtime = production_bindings.SettingsRuntime(state, tasks)
        assert runtime.state is state and runtime.tasks is tasks and runtime._error is DomainError
        link = production_bindings.WebSocketLink(AsyncMock(), "robot", 1)
        assert link._ports.snapshot is snapshots.snapshot_envelope
        assert link._ports.receive_diagnostic is production_bindings._receive_diagnostic
        assert type(link._send_lock) is DiagnosticLock
        assert link._send_lock._diagnostic.diagnostics is process_diagnostics()
        await link.close()
        connector = production_bindings.WebSocketConnector("wss://cloud.invalid", "token", "robot")
        assert connector._link_factory is production_bindings.WebSocketLink

    with tempfile.TemporaryDirectory(prefix="installed-bindings-", dir="/tmp") as directory:
        asyncio.run(verify(Path(directory)))
    if compatible:
        from senior_control.agent import composition
        from senior_control.agent import journal as old_journal
        from senior_control.agent import logs_wire as old_logs
        from senior_control.agent import settings_wire as old_settings
        from senior_control.agent import websocket as old_websocket
        from senior_control.storage import database as old_database
        from senior_control.vehicle_logs import service as old_service

        for owner, names in (
            (old_journal, ("JournalEntry", "AgentJournal", "_OPEN")),
            (old_logs, ("AgentLogConnection",)),
            (old_settings, ("SettingsRuntime",)),
            (old_websocket, ("WebSocketLink", "WebSocketConnector", "_receive_diagnostic")),
            (old_service, ("VehicleLogService",)),
        ):
            for name in names:
                assert getattr(owner, name) is getattr(production_bindings, name)
        assert old_journal._transaction is journal._transaction
        assert old_journal.AGENT_JOURNAL_STATEMENTS is journal.AGENT_JOURNAL_STATEMENTS
        assert old_database.Database is bindings.Database
        assert old_database.HomeExecutionCoordination is bindings.HomeExecutionCoordination
        assert composition.Database is bindings.Database
        for name in (
            "AgentJournal",
            "AgentLogConnection",
            "SettingsRuntime",
            "WebSocketConnector",
            "VehicleLogService",
        ):
            assert getattr(composition, name) is getattr(production_bindings, name)
    else:
        assert not any(
            name == "senior_control"
            or name.startswith(
                (
                    "senior_control.",
                    "cloud",
                    "rclpy",
                    "launch",
                    "nav2",
                    "senior_mapping",
                    "senior_navigation",
                    "senior_robot",
                    "httpx",
                    "websockets",
                    "aiortc",
                )
            )
            for name in sys.modules
        )
    print(
        "installed concrete production bindings, real SQLite/diagnostics and lazy network: PASS",
        compatible,
    )


def verify_local_ipc(*, compatible: bool = False) -> None:
    """Exercise installed concrete bindings with local replies, without a runtime service."""
    import asyncio
    import inspect
    from unittest.mock import AsyncMock

    from pydantic import ValidationError
    from senior_contracts.wire import SimpleEnvelope
    from vehicle_manager.agent import ipc_motion, ipc_navigation, ipc_state, local_adapters
    from vehicle_manager.agent.local_state import RuntimeState, StateReply
    from vehicle_manager.agent.pose_reply import PoseReply
    from vehicle_manager.tasks.models import ExecutionContext
    from vehicle_manager.waypoints.home_capture import HomeCapture

    from vehicle_core.errors import DomainError, ErrorCode
    from vehicle_core.ipc import IpcError
    from vehicle_core.values import MapRef, Pose2D

    for name, base, methods in (
        ("IpcMotion", ipc_motion.IpcMotion, ("_ok", "manual", "learning", "close")),
        ("TaskAwareMotion", ipc_motion.TaskAwareMotion, ("manual", "send_intent", "bind_tasks")),
        ("IpcMapPort", ipc_navigation.IpcMapPort, ("_call", "export", "import_", "clear")),
        ("IpcState", ipc_state.IpcState, ("read",)),
        ("IpcHomePort", HomeCapture, ("set_current", "home_revision")),
    ):
        bound = getattr(local_adapters, name)
        assert bound is not base and issubclass(bound, base)
        assert "site-packages" in inspect.getfile(bound), bound
        for method in methods:
            assert getattr(bound, method) is getattr(base, method)
    assert local_adapters.TaskAwareMotion.__mro__ == (
        local_adapters.TaskAwareMotion,
        ipc_motion.TaskAwareMotion,
        local_adapters.IpcMotion,
        ipc_motion.IpcMotion,
        object,
    )
    assert local_adapters.TaskAwareMotion.__init__ is local_adapters.IpcMotion.__init__
    assert local_adapters.PoseReply is PoseReply
    assert issubclass(local_adapters.MotionRejected, IpcError)
    packet = {
        "v": 1,
        "ok": True,
        "sampled_at": 10.0,
        "localized": True,
        "safety_clear": True,
        "manual_driving": False,
        "map_id": "yard",
        "reference_revision": 1,
        "execution_status": "idle",
        "task_evidence_ready": True,
        "navigation_ready": True,
    }

    async def verify() -> None:
        client = AsyncMock()
        client.request.return_value = packet
        state = local_adapters.IpcState(client, clock=lambda: 10.5)
        assert state._parse_state == StateReply.model_validate
        assert state._state_factory is RuntimeState and state._map_reference is MapRef
        value = await state.read()
        assert type(value) is RuntimeState and value.map_ref == MapRef("yard", 1)
        assert value.valid_until == 10.5 and value.scan_points == ()
        context = value.for_tasks(True)
        assert type(context) is ExecutionContext and context.connected
        assert context.admission_ready and context.valid_until == 10.5
        for now, change, cause in (
            (10.500001, {}, ValueError),
            (9.9, {}, ValueError),
            (10.0, {"map_id": None}, ValueError),
            (10.0, {"localized": "true"}, ValidationError),
            (10.0, {"unknown": 1}, ValidationError),
        ):
            client.request.return_value = {**packet, **change}
            try:
                await local_adapters.IpcState(client, clock=lambda now=now: now).read()
            except DomainError as exc:
                assert exc.code is ErrorCode.UNAVAILABLE and isinstance(exc.__cause__, cause)
            else:
                raise AssertionError((now, change))
        failure = IpcError("installed local transport failure")
        client.request.side_effect = failure
        for operation in (
            local_adapters.IpcState(client).read(),
            local_adapters.IpcMapPort(client).clear("clear", MapRef("yard", 1)),
        ):
            try:
                await operation
            except DomainError as exc:
                assert exc.code is ErrorCode.UNAVAILABLE and exc.__cause__ is failure
            else:
                raise AssertionError("transport cause lost")
        client.request.side_effect = None
        home = local_adapters.IpcHomePort(client, AsyncMock(), robot_id="robot")
        assert home._client is client and home._state._client is client
        assert type(home._state) is local_adapters.IpcState
        assert home._parse_pose == PoseReply.model_validate
        assert home._transport_errors == (IpcError, OSError, ValueError, ValidationError)
        assert home._values.pose is Pose2D and home._values.map_reference is MapRef
        assert home._values.error is DomainError
        intent, disable, safety = AsyncMock(), AsyncMock(), AsyncMock()
        motion = local_adapters.TaskAwareMotion(
            client,
            intent_client=intent,
            disable_client=disable,
            safety_client=safety,
        )
        assert motion._errors.rejected is local_adapters.MotionRejected
        assert motion._errors.domain is DomainError and motion._errors.ipc is IpcError
        stop = SimpleEnvelope(
            v=1,
            type="drive.stop",
            robot_id="robot",
            session_id="session",
            connection_epoch=1,
            seq=1,
            expires_at=5.0,
            request_id="stop",
            payload={},
        )
        safety.request.return_value = {"ok": True}
        client.request.reset_mock()
        await motion.stop(stop)
        client.request.assert_not_awaited()
        safety.request.assert_awaited_once()
        for reply, error, code in (
            ({"ok": False, "error": "unsupported"}, DomainError, ErrorCode.UNAVAILABLE),
            (
                {"ok": False, "accepted": False, "error": "conflict"},
                local_adapters.MotionRejected,
                ErrorCode.CONFLICT,
            ),
            (
                {"ok": False, "accepted": False, "error": "unknown-code"},
                local_adapters.MotionRejected,
                None,
            ),
            ({"ok": False}, IpcError, None),
        ):
            client.request.return_value = reply
            try:
                await motion._ok("manual")
            except error as exc:
                assert getattr(exc, "code", None) is code
            else:
                raise AssertionError(reply)
        release = SimpleEnvelope.model_validate({**stop.model_dump(), "type": "control.release"})
        safety.request.return_value = {"ok": False, "accepted": False, "error": "conflict"}
        try:
            await motion.manual(release)
        except DomainError as exc:
            assert exc.code is ErrorCode.CONFLICT
            assert type(exc.__cause__) is local_adapters.MotionRejected
            assert exc.__cause__.code is ErrorCode.CONFLICT
        else:
            raise AssertionError("explicit rejection lost its cause")
        safety.request.side_effect = failure
        try:
            await motion.manual(release)
        except DomainError as exc:
            assert exc.code is ErrorCode.OPERATION_UNKNOWN and exc.__cause__ is failure
        else:
            raise AssertionError("transport failure lost its unknown outcome")
        await motion.close()
        for lane in (client, intent, disable, safety):
            lane.close.assert_awaited_once()

    asyncio.run(verify())
    if compatible:
        from vehicle_manager.agent.connection import HeartbeatFrame, Link, MotionPort
        from vehicle_manager.agent.request_ports import Requester

        from senior_control.agent import (
            composition,
            connection,
            direct_navigation,
            dispatcher,
            main,
            ports,
            settings_wire,
            websocket,
            workspace,
        )

        for name in ("IpcState", "IpcHomePort", "IpcMapPort", "TaskAwareMotion", "IpcMotion"):
            assert getattr(ports, name) is getattr(local_adapters, name)
        assert connection.IpcMotion is local_adapters.IpcMotion
        assert connection.MotionRejected is local_adapters.MotionRejected
        assert connection.HeartbeatFrame is HeartbeatFrame
        assert connection.Link is Link and connection.MotionPort is MotionPort
        assert ports.RuntimeState is RuntimeState and ports.StateReply is StateReply
        assert ports.PoseReply is PoseReply
        for name in ("IpcState", "IpcHomePort", "IpcMapPort", "TaskAwareMotion"):
            assert getattr(composition, name) is getattr(local_adapters, name)
        assert composition.IpcNavigation is ports.IpcNavigation
        assert ports.IpcNavigation is not ipc_navigation.IpcNavigation
        assert ports.IpcNavigation._call is ipc_navigation.IpcNavigation._call
        assert main.MotionRejected is local_adapters.MotionRejected
        assert main.Link is Link and main.MotionPort is MotionPort
        assert direct_navigation.IpcState is dispatcher.IpcState is settings_wire.IpcState
        assert settings_wire.IpcState is local_adapters.IpcState
        assert websocket.HeartbeatFrame is HeartbeatFrame and workspace.Requester is Requester
    else:
        assert not any(
            name == "senior_control"
            or name.startswith(
                (
                    "senior_control.",
                    "cloud",
                    "rclpy",
                    "launch",
                    "nav2",
                    "senior_mapping",
                    "senior_navigation",
                    "senior_robot",
                )
            )
            for name in sys.modules
        )
    print(
        "installed concrete local IPC bindings, 0.5s parser/error dispatch and MRO: PASS",
        compatible,
    )


def verify_manager_agent(*, network: bool = False) -> None:
    verify_manager_config()
    from vehicle_manager import agent
    from vehicle_manager.agent.composition import build_task_runtime, run_agent
    from vehicle_manager.agent.ipc_motion import IpcMotion, TaskAwareMotion
    from vehicle_manager.agent.ipc_navigation import IpcMapPort, IpcNavigation
    from vehicle_manager.agent.ipc_state import IpcState

    assert all(
        callable(value)
        for value in (
            build_task_runtime,
            IpcMotion,
            TaskAwareMotion,
            IpcMapPort,
            IpcNavigation,
            IpcState,
        )
    )
    from vehicle_manager.agent.connection import HeartbeatFrame, Link, MotionPort
    from vehicle_manager.agent.main import Agent

    assert agent.Agent is Agent and agent.HeartbeatFrame is HeartbeatFrame
    assert agent.run_agent is run_agent
    assert all(callable(value) for value in (run_agent, Link, MotionPort))
    from vehicle_manager.agent.dispatcher import Dispatcher
    from vehicle_manager.agent.frame_projection import vehicle_frame
    from vehicle_manager.agent.journal import AgentJournal
    from vehicle_manager.agent.logs_wire import AgentLogConnection
    from vehicle_manager.agent.settings_wire import SettingsRuntime
    from vehicle_manager.agent.snapshot_state import SnapshotState
    from vehicle_manager.agent.snapshots import snapshot_envelope
    from vehicle_manager.agent.workspace import WorkspaceReader

    assert all(
        callable(value)
        for value in (
            SnapshotState,
            WorkspaceReader,
            Dispatcher,
            snapshot_envelope,
            vehicle_frame,
            SettingsRuntime,
            AgentLogConnection,
        )
    )
    from vehicle_manager.agent.pairing import Credentials
    from vehicle_manager.agent.supervisor import AuthRejected, Supervisor
    from vehicle_manager.agent.websocket import ProtocolRejected, WebSocketConnector, WebSocketLink

    assert agent.AgentJournal is AgentJournal and agent.Credentials is Credentials
    assert agent.AuthRejected is AuthRejected and agent.Supervisor is Supervisor
    assert agent.WebSocketLink is WebSocketLink and agent.WebSocketConnector is WebSocketConnector
    assert issubclass(ProtocolRejected, ConnectionError)
    assert not any(name in sys.modules for name in ("aiortc", "av", "rclpy"))
    assert not any(
        name == "senior_control" or name.startswith(("senior_control.", "rclpy", "senior_mapping"))
        for name in sys.modules
    )
    verify_local_ipc()
    verify_production_bindings()
    verify_route_execution()
    if network:
        import httpx
        import websockets
        from websockets.asyncio.client import connect

        assert callable(connect) and callable(httpx.AsyncClient)
        print("installed manager agent network extra:", httpx.__version__, websockets.__version__)
    print("installed manager agent foundations without legacy business/ROS: PASS")


def verify_manager_operator() -> None:
    import json
    import tempfile

    from vehicle_manager import cli, deployment, observability, service, simulation
    from vehicle_manager.agent import application

    assert callable(application.main) and callable(observability.configure_logging)
    from vehicle_manager.profile import VehicleProfile, load_profile

    assert all(
        callable(value)
        for value in (
            cli.main,
            deployment.render_service,
            service.supervise,
            simulation.supervise,
            load_profile,
            VehicleProfile,
        )
    )
    with tempfile.TemporaryDirectory(prefix="installed-operator-", dir="/tmp") as directory:
        root = Path(directory).resolve()
        profile_path = root / "config.json"
        profile_path.write_text(
            json.dumps(
                {
                    "kind": "real",
                    "project_dir": "/opt/robot",
                    "cloud_url": "wss://example.test/ws/agent",
                }
            )
        )
        profile = load_profile(profile_path)
        assert profile.credentials == root / "device.json" and profile.socket == root / "agent.sock"
        rendered = deployment.render_service(profile, "1000", container_root=None)
        assert "KillMode=mixed" in rendered and "RestartPreventExitStatus=78" in rendered
        assert "/opt/robot/.venv/bin/python" in rendered
        assert "senior_control.docker_start_helper" in rendered
        assert deployment.environment(profile, root).count("SENIOR_") == 4
    assert not any(
        name == "senior_control" or name.startswith(("senior_control.", "rclpy"))
        for name in sys.modules
    )
    verify_manager_media()
    print("installed manager real profile/rendering without legacy business/ROS: PASS")


def verify_legacy_operator() -> None:
    import contextlib
    import io
    import json
    import subprocess
    import tempfile

    from vehicle_manager import cli as canonical_cli
    from vehicle_manager import deployment, observability, service, simulation
    from vehicle_manager.profile import VehicleProfile, load_profile

    from senior_control import cli, simulation_cli, vehicle_deploy
    from senior_control import docker_start_helper as helper
    from senior_control import observability as old_observability
    from senior_control.runtime import ipc

    for name in ("JsonFormatter", "Metrics", "configure_logging", "OTHER"):
        assert getattr(old_observability, name) is getattr(observability, name)
    assert cli.VehicleProfile is VehicleProfile and cli.load_profile is load_profile
    assert vehicle_deploy.render_service is deployment.render_service
    from senior_robot import platform_runtime

    assert helper._platform is platform_runtime
    ports = helper._ports()
    assert ports.adapter_command is helper._adapter_command
    assert ports.motion_command is helper._motion_command
    assert ports.camera_command is helper._camera_command
    assert ports.video_environment is helper._video_environment
    assert helper._runtime_profile is service._runtime_profile
    assert helper._terminate_motion_stack is service._terminate_motion_stack
    assert simulation_cli.DisplayUnavailableError is simulation.DisplayUnavailableError
    assert helper._identity is ipc._identity and helper._record is ipc._record
    assert helper._prepare_socket is ipc._prepare_socket
    assert (ipc.PROTOCOL_VERSION, ipc.MAX_FRAME, ipc.MAX_PATH) == (1, 8192, 100)
    assert cli._unit is canonical_cli._unit and cli._install_once is canonical_cli._install_once
    with tempfile.TemporaryDirectory(prefix="installed-cli-", dir="/tmp") as directory:
        root = Path(directory).resolve()
        (root / "config.json").write_text(
            json.dumps(
                {
                    "kind": "real",
                    "project_dir": "/opt/robot",
                    "cloud_url": "wss://example.test/ws/agent",
                }
            )
        )
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            assert cli.main(["deploy", "--control-dir", str(root), "--dry-run"]) == 0
        assert "Runtime data directory:" in output.getvalue()
        assert sorted(path.name for path in root.iterdir()) == ["config.json"]
    for module in (
        "senior_control.cli",
        "senior_control.docker_start_helper",
        "senior_control.simulation_cli",
        "senior_control.agent.main",
    ):
        result = subprocess.run(
            [sys.executable, "-m", module, "--help"],
            cwd="/tmp",
            capture_output=True,
            text=True,
            timeout=15,
            check=True,
        )
        assert "usage:" in result.stdout
    console = Path(sys.executable).with_name("senior-robot")
    assert console.read_bytes().startswith(b"#!/bin/bash\n")
    assert not any(
        entry.name == "senior-robot"
        for entry in importlib.metadata.distribution("senior-control").entry_points
    ), "install the single Shell source separately; do not shadow a console-script"
    import os

    without_python = dict(os.environ, PATH="/nonexistent")
    independent = subprocess.run(
        [str(console), "--help"],
        cwd="/tmp",
        env=without_python,
        capture_output=True,
        text=True,
        timeout=15,
        check=True,
    )
    assert "restart" in independent.stdout and "docker" in independent.stdout
    syntax = subprocess.run(
        [str(console), "start", "--headless", "invalid"],
        cwd="/tmp",
        env=without_python,
        capture_output=True,
        text=True,
        timeout=15,
    )
    assert syntax.returncode == 2
    from vehicle_manager import operator_actions as actions_owner

    from senior_control import operator_actions as installed_actions

    assert installed_actions.execute is actions_owner.main
    assert installed_actions.docker_action is actions_owner.docker_action
    fixed = subprocess.run(
        [sys.executable, "-m", "senior_control.operator_actions"],
        cwd="/tmp",
        capture_output=True,
        text=True,
        timeout=15,
    )
    assert fixed.returncode == 2 and "Invalid normalized operator request" in fixed.stderr
    result = subprocess.run(
        [str(console), "--help"], cwd="/tmp", capture_output=True, text=True, timeout=15, check=True
    )
    assert "--help" in result.stdout
    import os

    environment = {
        name: value for name, value in os.environ.items() if not name.startswith("SENIOR_")
    }
    environment.update(
        SENIOR_ROBOT_ID="robot-a",
        SENIOR_CLOUD_URL="ws://cloud.test/ws/agent",
        SENIOR_DEVICE_TOKEN="private-token",
        SENIOR_DATABASE_PATH="/tmp/unused-agent.db",
        SENIOR_AGENT_SOCKET="/tmp/unused-agent.sock",
    )
    rejected = subprocess.run(
        [sys.executable, "-m", "senior_control.agent.main"],
        cwd="/tmp",
        env=environment,
        capture_output=True,
        text=True,
        timeout=15,
    )
    assert rejected.returncode == 78
    assert "invalid agent configuration: cloud URL must use authenticated TLS" in rejected.stderr
    assert "private-token" not in rejected.stdout + rejected.stderr
    print(
        "installed legacy aliases, unchanged dry-run/CLI help and original Agent error exit: PASS"
    )


def verify_manager_media(*, backend: bool = False) -> None:
    """Import real media code without a legacy/ROS dependency or implicit listener."""
    from vehicle_manager.media.latest_frame import LatestFrameFile
    from vehicle_manager.media.signaling import VideoSignaling
    from vehicle_manager.media.stream import AiortcPeerFactory, VideoSession
    from vehicle_manager.media.turn import TurnCredentialIssuer

    assert all(
        callable(value)
        for value in (
            LatestFrameFile,
            VideoSignaling,
            AiortcPeerFactory,
            VideoSession,
            TurnCredentialIssuer,
        )
    )
    assert not any(
        name.startswith(("senior_control", "rclpy", "senior_robot")) for name in sys.modules
    )
    assert "aiortc" not in sys.modules and "av" not in sys.modules
    if backend:
        from vehicle_manager.media.stream import _load_aiortc

        aiortc = _load_aiortc()
        import av

        print("installed isolated media backend:", aiortc.__version__, av.__version__)
    print("installed manager media without legacy business/ROS and lazy backend: PASS")


def verify_legacy_media() -> None:
    from senior_robot.ros_video import RosCameraSource as Camera
    from vehicle_manager.media import stream, turn
    from vehicle_manager.media.latest_frame import LatestFrameFile as Reader
    from vehicle_manager.media.signaling import VideoSignaling as Signaling

    from senior_control.runtime.diagnostics import DiagnosticAsyncQueue
    from senior_control.vision import rgb1
    from senior_control.vision import stream as old_stream
    from senior_control.vision import turn as old_turn
    from senior_control.vision.ros_video import RosCameraSource
    from senior_control.vision.signaling import VideoSignaling
    from senior_control.vision.simulation_video import LatestFrameFile, publish_image
    from senior_control.vision.synchronizer import ImageFrame

    for module, old in ((stream, old_stream), (turn, old_turn)):
        for name in (
            name
            for name in vars(module)
            if not name.startswith("_") and getattr(module, name).__class__.__name__ == "type"
        ):
            if getattr(module, name).__module__ == module.__name__:
                assert getattr(old, name) is getattr(module, name), name
    assert old_stream.ImageFrame is ImageFrame
    assert old_stream.negotiate_codec is stream.negotiate_codec
    assert old_turn.issue_turn_credentials is turn.issue_turn_credentials
    assert old_turn.verify_turn_credentials is turn.verify_turn_credentials
    assert old_stream.logger.name == "senior_control.vision.stream"
    assert publish_image is rgb1.publish_image
    assert issubclass(LatestFrameFile, Reader) and LatestFrameFile.next_frame is Reader.next_frame
    reader = LatestFrameFile(Path("/tmp/unused.rgb"))
    assert reader._read_frame is rgb1._read_frame
    assert reader._poll_seconds == rgb1._POLL_SECONDS
    assert issubclass(RosCameraSource, Camera) and RosCameraSource._run is Camera._run
    camera = RosCameraSource("/camera/color/image_raw")
    assert camera._frame_factory is ImageFrame and camera._thread is None
    import asyncio
    import tempfile

    with tempfile.TemporaryDirectory(prefix="installed-media-") as directory:
        path = Path(directory) / "camera.rgb"
        assert publish_image(path, 1, 1, 3, "rgb8", b"abc")
        frame = asyncio.run(LatestFrameFile(path).next_frame(0.1))
        assert type(frame) is ImageFrame and frame.data == b"abc"
    assert VideoSignaling is not Signaling and issubclass(VideoSignaling, Signaling)
    assert VideoSignaling.submit is Signaling.submit
    assert VideoSignaling.close is Signaling.close
    import inspect

    assert (
        inspect.signature(VideoSignaling).parameters["make_session"].kind
        is inspect.Parameter.KEYWORD_ONLY
    )
    # Actual production binding; no fake diagnostic queue, camera thread or peer.
    assert VideoSignaling.__init__.__globals__["DiagnosticAsyncQueue"] is DiagnosticAsyncQueue
    print("installed original media aliases, shared RGB1/frame and execution camera binding: PASS")


def verify_real_preflight() -> None:
    """The existing hardware preflight requires the original SDK safety extra."""
    from senior_robot import real_runtime as preflight

    from senior_control import real_runtime as old_preflight

    for name in (
        "real_parameter_defaults",
        "load_real_parameters",
        "validate_real_runtime",
        "validate_navigation",
        "REQUIRED_PARAMETERS",
        "RETIRED_PARAMETERS",
    ):
        assert getattr(old_preflight, name) is getattr(preflight, name)
    assert "rclpy" not in sys.modules

    print("installed ROS-free real preflight and original public identities: PASS")


def verify_home_database(*, compatible: bool = False) -> None:
    """Use installed SQL and the same injected per-database Home owner."""
    import asyncio
    import inspect
    import tempfile
    from importlib.util import find_spec
    from unittest.mock import patch

    import aiosqlite
    from vehicle_manager.storage.database import Database as Engine
    from vehicle_manager.storage.domain_schema import domain_migrations
    from vehicle_manager.storage.home_database import HomeDatabase, HomeExecutionCoordination
    from vehicle_manager.storage.migrations import Migration, migrate
    from vehicle_manager.tasks.store import TASK_STATEMENTS
    from vehicle_manager.waypoints.store import WAYPOINT_STATEMENTS

    assert "site-packages" in inspect.getfile(HomeDatabase)
    assert HomeDatabase.__aenter__ is Engine.__aenter__
    if not compatible:
        verify_manager_core_distribution()
        for package in ("senior_mapping", "cloud", "rclpy"):
            assert find_spec(package) is None, package
    locks, coordinations = [], []

    def lock_factory(name):
        lock = asyncio.Lock()
        locks.append((name, lock))
        return lock

    def coordination_factory():
        coordination = HomeExecutionCoordination(lock_factory=lock_factory)
        coordinations.append(coordination)
        return coordination

    def selected(path):
        return HomeDatabase(
            path, lock_factory=lock_factory, coordination_factory=coordination_factory
        )

    map_sql = ("CREATE TABLE install_map_owner (value INTEGER)",)
    migrations = domain_migrations(map_sql)
    assert tuple(m.version for m in migrations) == (1, 2, 3)
    assert all(type(m) is Migration for m in migrations)
    assert migrations[0].statements is map_sql
    assert migrations[1].statements is WAYPOINT_STATEMENTS
    assert migrations[2].statements is TASK_STATEMENTS
    if compatible:
        from senior_mapping.schema import OPERATION_JOURNAL_STATEMENTS

        from senior_control.storage import schema
        from senior_control.storage.database import Database as OldDatabase
        from senior_control.storage.database import HomeExecutionCoordination as OldCoordination
        from vehicle_core.diagnostics import DiagnosticLock, process_diagnostics

        assert tuple(inspect.signature(OldDatabase).parameters) == ("path", "busy_timeout_ms")
        assert inspect.signature(OldDatabase).parameters["busy_timeout_ms"].default == 1000
        assert not inspect.signature(OldCoordination).parameters
        assert OldCoordination is not HomeExecutionCoordination
        assert issubclass(OldCoordination, HomeExecutionCoordination)
        assert OldDatabase.home_execution is HomeDatabase.home_execution
        assert OldDatabase.home_target_lock is HomeDatabase.home_target_lock
        assert schema.Migration is Migration
        assert schema.OPERATION_JOURNAL_STATEMENTS is OPERATION_JOURNAL_STATEMENTS
        assert schema.TASK_STATEMENTS is TASK_STATEMENTS
        assert schema.WAYPOINT_STATEMENTS is WAYPOINT_STATEMENTS
        migrations = schema.DOMAIN_MIGRATIONS
        assert tuple(m.version for m in migrations) == (1, 2, 3)
        assert migrations[0].statements is OPERATION_JOURNAL_STATEMENTS
        assert migrations[1].statements is WAYPOINT_STATEMENTS
        assert migrations[2].statements is TASK_STATEMENTS
        selected = OldDatabase

    async def verify(directory):
        database = selected(directory / "home.db")
        write_lock = database.write_lock
        target = database.home_target_lock("robot")
        assert database.home_target_lock("robot") is target
        shared = database.home_execution("robot")
        assert database.home_execution("robot") is shared
        assert shared.connected and shared.disconnect_generation == 0
        assert shared.cancel_owner is None and not shared.cancel_required
        assert database.home_execution("other") is not shared
        other = selected(directory / "other.db")
        assert other.home_execution("robot") is not shared
        assert other.home_target_lock("robot") is not target
        if compatible:
            assert type(shared) is OldCoordination
            for lock in (write_lock, target, shared.lock):
                assert type(lock) is DiagnosticLock
                assert lock._diagnostic.diagnostics is process_diagnostics()
        else:
            assert [name for name, _ in locks].count("storage.home_target_wait") == 3
            assert len(coordinations) == 4  # setdefault still eagerly constructs on cache hits.
        connection_paths = []
        connect = aiosqlite.connect

        def tracked_connect(path, *args, **kwargs):
            connection_paths.append(str(path))
            return connect(path, *args, **kwargs)

        with patch.object(aiosqlite, "connect", tracked_connect):
            async with database as opened:
                assert opened is database
                connection = database.connection
                await migrate(database, list(migrations))
                assert database.connection is connection and database.write_lock is write_lock
                assert database.home_execution("robot") is shared
                async with connection.execute("PRAGMA user_version") as cursor:
                    assert await cursor.fetchone() == (3,)
                async with connection.execute("SELECT name FROM sqlite_master") as cursor:
                    tables = {row[0] for row in await cursor.fetchall()}
                assert {"waypoint", "task_definition"} <= tables
        assert connection_paths.count(str(database.path)) == 1
        assert not database.is_open

    with tempfile.TemporaryDirectory() as directory:
        asyncio.run(verify(Path(directory)))
    print("installed Home database owner/factory timing and domain SQLite schema: PASS", compatible)


def verify_manager_storage() -> None:
    """Exercise the installed SQLite owner with the original migration cases."""
    import asyncio
    import sqlite3
    import tempfile

    from vehicle_manager.storage import CORE_MIGRATIONS, Database, Migration, migrate, migrate_core
    from vehicle_manager.storage.agent_schema import AGENT_JOURNAL_STATEMENTS

    assert len(AGENT_JOURNAL_STATEMENTS) == 2

    async def verify(directory: Path) -> None:
        database = Database(directory / "vehicle.db", lock_factory=lambda _: asyncio.Lock())
        migrations = [
            Migration(1, ("CREATE TABLE item (value TEXT)", "INSERT INTO item VALUES ('before')")),
            Migration(2, ("INSERT INTO item VALUES ('after')",)),
        ]
        async with database as opened:
            assert opened is database
            await migrate(database, migrations[:1])
            await migrate(database, migrations)
            try:
                await migrate(
                    database,
                    [
                        *migrations,
                        Migration(3, ("INSERT INTO item VALUES ('failed')", "INVALID SQL")),
                    ],
                )
            except sqlite3.OperationalError:
                pass
            else:
                raise AssertionError("invalid migration must roll back")
            await migrate_core(database)
            async with database.connection.execute("PRAGMA user_version") as cursor:
                assert await cursor.fetchone() == (2,)
            async with database.connection.execute("SELECT value FROM item") as cursor:
                assert await cursor.fetchall() == [("before",), ("after",)]
            async with database.connection.execute("SELECT version FROM core_schema_version") as c:
                assert await c.fetchone() == (len(CORE_MIGRATIONS),)
        assert not database.is_open
        with sqlite3.connect(str(database.path) + ".backup") as backup:
            # The failed v3 attempt backs up the committed v2 image before BEGIN.
            assert backup.execute("PRAGMA user_version").fetchone() == (2,)
            assert backup.execute("SELECT value FROM item").fetchall() == [("before",), ("after",)]
        async with database:
            await migrate(database, migrations)
            await migrate_core(database)
        assert not database.is_open

    with tempfile.TemporaryDirectory() as directory:
        asyncio.run(verify(Path(directory)))
    print("installed manager SQLite migrations, rollback, backup, core schema and reopen: PASS")


def verify_manager_tasks() -> None:
    """Verify the installed pure task interfaces without starting any owner."""
    from vehicle_manager import tasks
    from vehicle_manager.tasks import lifecycle
    from vehicle_manager.tasks.models import ExecutionContext, TaskDefinition
    from vehicle_manager.tasks.runner import TaskRunner
    from vehicle_manager.tasks.schedule import Schedule

    assert tasks.Schedule is Schedule
    assert tasks.lifecycle is lifecycle
    assert Schedule.__module__ == "vehicle_manager.tasks.schedule"
    assert lifecycle.Lifecycle.__module__ == "vehicle_manager.tasks.lifecycle"
    assert callable(lifecycle.command) and callable(lifecycle.acknowledge)
    assert tasks.TaskDefinition is TaskDefinition
    assert tasks.ExecutionContext is ExecutionContext
    assert tasks.TaskRunner is TaskRunner
    assert TaskDefinition.__module__ == "vehicle_manager.tasks.models"
    assert TaskRunner.__module__ == "vehicle_manager.tasks.runner"
    print("installed pure task schedule/model/runner/lifecycle interfaces: PASS")


def verify_manager_task_store() -> None:
    from vehicle_manager import tasks
    from vehicle_manager.tasks.store import TaskStore

    assert tasks.TaskStore is TaskStore
    assert TaskStore.__module__ == "vehicle_manager.tasks.store"
    print("installed manager TaskStore interface without legacy business/ROS owners: PASS")


def verify_manager_lifecycle() -> None:
    from vehicle_manager import tasks
    from vehicle_manager.tasks.lifecycle_service import LifecycleService
    from vehicle_manager.tasks.lifecycle_store import LifecycleStore

    assert tasks.LifecycleStore is LifecycleStore
    assert tasks.LifecycleService is LifecycleService
    assert LifecycleStore.__module__ == "vehicle_manager.tasks.lifecycle_store"
    assert LifecycleService.__module__ == "vehicle_manager.tasks.lifecycle_service"
    print("installed manager lifecycle interfaces without legacy business/ROS owners: PASS")


def verify_manager_execution() -> None:
    """Import the canonical drivers without any local exception or ROS implementation."""
    from vehicle_manager import tasks
    from vehicle_manager.tasks.executor import NavigationFailure, ProcessCrash, TaskExecutor
    from vehicle_manager.tasks.navigation_port import NavigationEffects, TaskNavigation
    from vehicle_manager.tasks.runtime import TaskRuntime

    for name, value in (
        ("TaskExecutor", TaskExecutor),
        ("TaskRuntime", TaskRuntime),
        ("NavigationEffects", NavigationEffects),
        ("TaskNavigation", TaskNavigation),
        ("NavigationFailure", NavigationFailure),
        ("ProcessCrash", ProcessCrash),
    ):
        assert getattr(tasks, name) is value
    assert issubclass(ProcessCrash, BaseException) and not issubclass(ProcessCrash, Exception)
    assert not any(
        name == "senior_control" or name.startswith(("senior_control.", "rclpy", "senior_mapping"))
        for name in sys.modules
    )
    print("installed canonical task drivers without legacy business/ROS: PASS")


def verify_manager_waypoints() -> None:
    from vehicle_manager import waypoints
    from vehicle_manager.waypoints.direct_navigation import DirectNavigation
    from vehicle_manager.waypoints.home_capture import HomeCapture
    from vehicle_manager.waypoints.home_service import HomeService
    from vehicle_manager.waypoints.ports import WaypointStorePorts
    from vehicle_manager.waypoints.store import WAYPOINT_STATEMENTS, WaypointStore

    assert waypoints.HomeService is HomeService
    assert waypoints.HomeCapture is HomeCapture
    assert waypoints.DirectNavigation is DirectNavigation
    assert waypoints.WaypointStore is WaypointStore
    assert waypoints.WaypointStorePorts is WaypointStorePorts
    assert len(WAYPOINT_STATEMENTS) == 2
    assert not any(
        name == "senior_control" or name.startswith(("senior_control.", "rclpy", "senior_mapping"))
        for name in sys.modules
    )
    print("installed canonical waypoint storage without legacy business/ROS: PASS")


def verify_legacy_task_store() -> None:
    """Reuse the original real SQLite create/reopen/conflict path in the installed SDK."""
    import asyncio
    import tempfile
    from datetime import datetime, timezone

    from pydantic import ValidationError
    from senior_mapping.operations import HomeLink
    from senior_mapping.store import MapReferenceStore
    from vehicle_manager.agent.journal import AgentJournal as ManagerJournal
    from vehicle_manager.agent.pairing import Credentials as ManagerCredentials
    from vehicle_manager.agent.pairing import pair_device as manager_pair_device
    from vehicle_manager.agent.supervisor import AuthRejected as ManagerAuthRejected
    from vehicle_manager.agent.supervisor import Supervisor as ManagerSupervisor
    from vehicle_manager.agent.supervisor import logger as manager_supervisor_logger
    from vehicle_manager.agent.websocket import WebSocketConnector as ManagerConnector
    from vehicle_manager.agent.websocket import WebSocketLink as ManagerLink
    from vehicle_manager.tasks.executor import NavigationFailure as ManagerNavigationFailure
    from vehicle_manager.tasks.executor import ProcessCrash as ManagerProcessCrash
    from vehicle_manager.tasks.executor import TaskExecutor as ManagerExecutor
    from vehicle_manager.tasks.navigation_port import NavigationEffects as ManagerEffects
    from vehicle_manager.tasks.runtime import TaskRuntime as ManagerRuntime
    from vehicle_manager.waypoints.direct_navigation import (
        DirectNavigation as ManagerDirectNavigation,
    )
    from vehicle_manager.waypoints.home_capture import HomeCapture
    from vehicle_manager.waypoints.home_service import HomeService as ManagerHomeService
    from vehicle_manager.waypoints.store import WAYPOINT_STATEMENTS as ManagerWaypointStatements
    from vehicle_manager.waypoints.store import WaypointStore as ManagerWaypointStore

    from senior_control.agent.composition import build_task_runtime
    from senior_control.agent.direct_navigation import DirectNavigation
    from senior_control.agent.dispatcher import LifecycleNavigationOperations, _TaskRouteReceiver
    from senior_control.agent.journal import _OPEN, AgentJournal, JournalEntry
    from senior_control.agent.pairing import Credentials, pair_device
    from senior_control.agent.ports import IpcHomePort, IpcNavigation, IpcState, PoseReply
    from senior_control.agent.websocket import WebSocketConnector, WebSocketLink
    from senior_control.agent.workspace import WorkspaceReader
    from senior_control.errors import DomainError, ErrorCode
    from senior_control.models import MapRef, OperationStatus, Pose2D
    from senior_control.navigation import RouteRejected
    from senior_control.runtime.ipc import IpcClient, IpcError
    from senior_control.runtime.supervisor import AuthRejected, Supervisor
    from senior_control.storage.database import Database
    from senior_control.storage.schema import DOMAIN_MIGRATIONS
    from senior_control.tasks import Schedule, TaskDefinition
    from senior_control.tasks.executor import (
        NavigationFailure,
        ProcessCrash,
        QualificationUnavailable,
        TaskExecutor,
    )
    from senior_control.tasks.lifecycle import Command, Context
    from senior_control.tasks.lifecycle_service import LifecycleService
    from senior_control.tasks.lifecycle_store import LifecycleStore
    from senior_control.tasks.navigation_port import NavigationEffects
    from senior_control.tasks.runtime import TaskRuntime
    from senior_control.tasks.store import TaskStore
    from senior_control.waypoints.home import HomeGoal, HomeResult
    from senior_control.waypoints.home import HomeService as LegacyHomeService
    from senior_control.waypoints.store import (
        WAYPOINT_STATEMENTS,
        StoredWaypoint,
        Waypoint,
        WaypointStore,
    )

    assert WaypointStore is not ManagerWaypointStore
    assert issubclass(WaypointStore, ManagerWaypointStore)
    assert Waypoint.__module__ == StoredWaypoint.__module__ == "senior_control.waypoints.store"
    assert DOMAIN_MIGRATIONS[1].statements is WAYPOINT_STATEMENTS is ManagerWaypointStatements
    for method in ("home", "waypoint", "route_poses", "save_home", "save_waypoint", "sync_poses"):
        assert getattr(WaypointStore, method) is getattr(ManagerWaypointStore, method)

    from vehicle_manager.agent import dispatcher as manager_dispatcher
    from vehicle_manager.agent import snapshots as manager_snapshots
    from vehicle_manager.agent.logs_wire import AgentLogConnection as ManagerLogConnection
    from vehicle_manager.agent.settings_wire import SettingsRuntime as ManagerSettingsRuntime
    from vehicle_manager.agent.snapshot_state import SnapshotState as ManagerSnapshotState
    from vehicle_manager.agent.workspace import WorkspaceReader as ManagerWorkspace

    from senior_control.agent.composition import SnapshotState
    from senior_control.agent.dispatcher import (
        Dispatcher,
        OperationNotCommitted,
        TaskOperationResult,
    )
    from senior_control.agent.logs_wire import AgentLogConnection
    from senior_control.agent.settings_wire import SettingsRuntime
    from senior_control.agent.snapshots import HomeSummary, LiveState, RobotSnapshot, TaskSummary

    for old, canonical in (
        (SnapshotState, ManagerSnapshotState),
        (WorkspaceReader, ManagerWorkspace),
        (Dispatcher, manager_dispatcher.Dispatcher),
        (LifecycleNavigationOperations, manager_dispatcher.LifecycleNavigationOperations),
        (SettingsRuntime, ManagerSettingsRuntime),
        (AgentLogConnection, ManagerLogConnection),
    ):
        assert old is not canonical and issubclass(old, canonical)
    assert SnapshotState.collect is ManagerSnapshotState.collect
    assert WorkspaceReader.read is ManagerWorkspace.read
    assert Dispatcher.run is manager_dispatcher.Dispatcher.run
    assert manager_dispatcher.logger.name == "senior_control.agent"
    for name, value in (
        ("RobotSnapshot", RobotSnapshot),
        ("LiveState", LiveState),
        ("HomeSummary", HomeSummary),
        ("TaskSummary", TaskSummary),
    ):
        assert getattr(manager_snapshots, name) is value

    from vehicle_manager.agent.composition import pause_failed_tasks as manager_pause_tasks
    from vehicle_manager.agent.connection import HeartbeatFrame as ManagerHeartbeat
    from vehicle_manager.agent.connection import Link as ManagerLinkProtocol
    from vehicle_manager.agent.connection import MotionPort as ManagerMotionProtocol
    from vehicle_manager.agent.main import Agent as ManagerAgent

    from senior_control.agent.composition import pause_failed_tasks
    from senior_control.agent.connection import HeartbeatFrame, Link, MotionPort, MotionRejected
    from senior_control.agent.dispatcher import StoreTaskPort
    from senior_control.agent.main import Agent as LegacyAgent
    from senior_control.agent.main import ManualOperationResult
    from senior_control.agent.ports import IpcMapPort, TaskAwareMotion
    from senior_control.clock import SystemClock
    from senior_control.models import OperationResult
    from senior_control.runtime.diagnostics import DiagnosticPending, PendingCounter

    assert HeartbeatFrame is ManagerHeartbeat and Link is ManagerLinkProtocol
    assert MotionPort is ManagerMotionProtocol and pause_failed_tasks is manager_pause_tasks
    assert LegacyAgent is not ManagerAgent and issubclass(LegacyAgent, ManagerAgent)
    for name in ("run", "_session", "dispatch", "_collect", "request_shutdown", "_release"):
        assert getattr(LegacyAgent, name) is getattr(ManagerAgent, name)
    assert issubclass(MotionRejected, IpcError)
    from vehicle_manager.agent.ipc_motion import IpcMotion as ManagerMotion
    from vehicle_manager.agent.ipc_motion import TaskAwareMotion as ManagerTaskMotion
    from vehicle_manager.agent.ipc_navigation import IpcMapPort as ManagerMaps
    from vehicle_manager.agent.ipc_navigation import IpcNavigation as ManagerNavigation
    from vehicle_manager.agent.ipc_state import IpcState as ManagerState

    from senior_control.agent.connection import IpcMotion as LegacyMotion
    from senior_control.agent.ports import RuntimeState, StateReply
    from senior_control.navigation import RoutePlan

    for legacy, canonical, methods in (
        (LegacyMotion, ManagerMotion, ("_ok", "manual", "learning", "close")),
        (TaskAwareMotion, ManagerTaskMotion, ("manual", "send_intent", "bind_tasks")),
        (IpcMapPort, ManagerMaps, ("_call", "export", "import_", "clear")),
        (IpcNavigation, ManagerNavigation, ("_call", "task_effect", "task_effect_status")),
        (IpcState, ManagerState, ("read",)),
    ):
        assert legacy is not canonical and issubclass(legacy, canonical)
        for method in methods:
            assert getattr(legacy, method) is getattr(canonical, method)
    assert issubclass(TaskAwareMotion, LegacyMotion)
    assert ManualOperationResult.__bases__ == (OperationResult,)

    import subprocess

    help_result = subprocess.run(
        [sys.executable, "-m", "senior_control.agent.main", "--help"],
        cwd="/tmp",
        capture_output=True,
        text=True,
        timeout=15,
        check=True,
    )
    assert "--credentials-file" in help_result.stdout and "--pair" in help_result.stdout
    print("installed original Agent constructor and legacy CLI help: PASS")

    assert Credentials is ManagerCredentials and pair_device is manager_pair_device
    assert Supervisor is ManagerSupervisor and AuthRejected is ManagerAuthRejected
    assert manager_supervisor_logger.name == "senior_control.runtime.supervisor"
    assert AgentJournal is not ManagerJournal and issubclass(AgentJournal, ManagerJournal)
    for name in ("prepare", "get", "begin", "finish", "deleted_task_ids"):
        assert getattr(AgentJournal, name) is getattr(ManagerJournal, name)
    assert WebSocketLink is not ManagerLink and issubclass(WebSocketLink, ManagerLink)
    assert WebSocketLink.recv is ManagerLink.recv
    assert WebSocketConnector is not ManagerConnector
    connector = WebSocketConnector("wss://cloud.invalid/ws/agent", "unused", "robot")
    assert connector._link_factory is WebSocketLink

    async def verify(directory: Path) -> None:
        reference = MapRef("yard", 1)
        created = datetime(2026, 1, 1, tzinfo=timezone.utc)
        definition = TaskDefinition(
            "task",
            "robot",
            reference,
            ("point",),
            Schedule.interval(created, 60),
            "Asia/Shanghai",
            True,
            0,
            created,
        )
        path = directory / "tasks.db"
        async with Database(path) as database:
            store = TaskStore(database, robot_id="robot")
            assert store._database is database
            assert store._ports.map_reference is MapRef and store._ports.error is DomainError
            assert store._codec._map_reference is MapRef and store._codec._error is DomainError
            await store.prepare()
            journal = AgentJournal(database, robot_id="robot")
            assert journal._database is database
            assert journal._ports.entry is JournalEntry
            assert journal._ports.status is OperationStatus
            assert journal._ports.active is _OPEN
            await journal.prepare()
            assert await journal.begin("installed-edit", "fingerprint") is None
            entry = await journal.get("installed-edit")
            assert type(entry) is JournalEntry and entry.status is OperationStatus.RUNNING
            assert not entry.closed
            await journal.finish("installed-edit", OperationStatus.SUCCEEDED, revision=1)
            assert (await journal.get("installed-edit")).closed
            saved = await store.create(definition)
            assert type(saved.map_ref) is MapRef and saved.map_ref == reference
            assert await store.get("task") == saved
            try:
                await store.update(saved, 0)
            except DomainError as exc:
                assert type(exc) is DomainError and exc.code is ErrorCode.CONFLICT
            else:
                raise AssertionError("stale revision must preserve the original DomainError")
            durable = LifecycleStore(store, boot_id="installed-boot")
            assert durable.tasks is store and durable.database is database
            client = IpcClient(directory / "unused.sock", robot_id="robot")
            points = WaypointStore(database, robot_id="robot")
            assert points._database is database and points._ports.map_reference is MapRef
            assert points._ports.pose is Pose2D and points._ports.error is DomainError
            assert points._ports.waypoint is Waypoint and points._ports.stored is StoredWaypoint
            await points.prepare()
            point_pose = Pose2D(1.0, 2.0, 0.0)
            point = await points.save_waypoint(
                waypoint_id="point",
                pose=point_pose,
                map_ref=reference,
                anchor_id="anchor",
                anchor_pose=point_pose,
                source_robot_id="robot",
                status="confirmed",
                latitude=None,
                longitude=None,
            )
            assert (
                type(point) is Waypoint and point.pose is point_pose and point.map_ref is reference
            )
            read = await points.waypoint("point")
            assert type(read) is StoredWaypoint and type(read.waypoint) is Waypoint
            assert type(read.waypoint.pose) is Pose2D and type(read.waypoint.map_ref) is MapRef
            assert read.waypoint == point
            navigation = IpcNavigation(client, points, tasks=store)
            assert await navigation.task_poses(saved) == (point_pose,)
            effects = NavigationEffects(durable, navigation)
            assert NavigationEffects is not ManagerEffects
            assert issubclass(NavigationEffects, ManagerEffects)
            assert NavigationEffects.execute is ManagerEffects.execute
            assert effects._rejection_type is RouteRejected
            runtime, executor = build_task_runtime(database, store, navigation, boot_id="installed")
            assert type(executor) is TaskExecutor and isinstance(executor, ManagerExecutor)
            assert TaskExecutor is not ManagerExecutor
            assert TaskExecutor.control is ManagerExecutor.control
            assert TaskRuntime is ManagerRuntime and type(runtime) is ManagerRuntime
            assert (
                NavigationFailure is ManagerNavigationFailure
                and ProcessCrash is ManagerProcessCrash
            )
            assert not issubclass(ProcessCrash, Exception)
            assert runtime._store is store and executor._store is store
            assert executor._database is database
            assert runtime._executor is executor
            assert runtime._generation() == executor._generation() == 0
            assert executor.lifecycle.store.tasks is store
            assert executor.lifecycle.store.database is database
            assert isinstance(executor.lifecycle.port, ManagerEffects)
            assert executor.lifecycle.port.navigation is navigation
            capture = IpcHomePort(client, points, robot_id="robot")
            assert IpcHomePort is not HomeCapture and isinstance(capture, HomeCapture)
            assert IpcHomePort.set_current is HomeCapture.set_current
            assert IpcHomePort.home_revision is HomeCapture.home_revision
            assert capture._client is client and capture._store is points
            assert type(capture._state) is IpcState and capture._state._client is client
            assert capture._parse_pose == PoseReply.model_validate
            assert capture._transport_errors == (IpcError, OSError, ValueError, ValidationError)
            assert capture._values.map_reference is MapRef
            assert capture._values.pose is Pose2D and capture._values.error is DomainError
            state = IpcState(client)
            assert state._parse_state == StateReply.model_validate
            assert state._state_factory is RuntimeState and state._map_reference is MapRef
            assert state._error is DomainError and state._ipc_error is IpcError
            assert navigation._values.route_plan is RoutePlan
            assert navigation._values.route_rejected is RouteRejected
            assert navigation._values.pose is Pose2D
            assert navigation._values.error is DomainError
            assert navigation._values.ipc_error is IpcError
            assert navigation._waypoints is points and navigation._tasks is store
            assert state is not capture._state and state._client is capture._state._client
            workspace = WorkspaceReader("robot", store, points, client)
            actions = LifecycleNavigationOperations(executor, state, workspace, points)
            assert workspace._ports.map_reference is MapRef and workspace._ports.pose is Pose2D
            assert workspace._ports.error is DomainError and workspace._ports.ipc_error is IpcError
            assert actions._values.result.__module__ == "senior_control.models"
            assert actions._values.not_committed is OperationNotCommitted
            assert actions._values.task_result is TaskOperationResult
            assert actions._values.direct_navigation is DirectNavigation
            snapshots = SnapshotState(state, store, points, workspace, navigation=actions)
            assert snapshots._state is state and snapshots._tasks is store
            assert snapshots._waypoints is points and snapshots._navigation is actions
            assert snapshots._ports.error is DomainError

            motion = TaskAwareMotion(
                client,
                intent_client=IpcClient(directory / "unused.sock", robot_id="robot"),
                disable_client=IpcClient(directory / "unused.sock", robot_id="robot"),
                safety_client=IpcClient(directory / "unused.sock", robot_id="robot"),
            )
            assert motion._errors.domain is DomainError and motion._errors.ipc is IpcError
            assert motion._errors.rejected is MotionRejected
            installed_agent = LegacyAgent(
                "robot",
                connector=connector,
                journal=journal,
                tasks=StoreTaskPort(store, robot_id="robot"),
                home=capture,
                maps=IpcMapPort(client),
                state=snapshots,
                motion=motion,
                navigation=actions,
            )
            assert installed_agent._journal is journal and journal._database is database
            assert installed_agent._state is snapshots and installed_agent._motion is motion
            assert installed_agent._navigation is actions
            assert installed_agent._ports.error is DomainError
            assert installed_agent._ports.result is OperationResult
            assert installed_agent._ports.manual_result is ManualOperationResult
            assert installed_agent._ports.manual_result_type is ManualOperationResult
            assert installed_agent._ports.motion_rejected is MotionRejected
            assert installed_agent._ports.ipc_error is IpcError
            assert installed_agent._ports.counter is PendingCounter
            assert installed_agent._ports.safety_pending is DiagnosticPending
            assert installed_agent._ports.clock is SystemClock
            assert motion._manual_clock == installed_agent._clock.monotonic
            assert (
                len(
                    {
                        motion._client,
                        motion._intent_client,
                        motion._disable_client,
                        motion._safety_client,
                    }
                )
                == 4
            )

            receiver = _TaskRouteReceiver(executor, "installed-fingerprint", state)
            target = DirectNavigation(state, workspace, points, receiver)
            assert DirectNavigation is not ManagerDirectNavigation
            assert isinstance(target, ManagerDirectNavigation)
            assert DirectNavigation.__call__ is ManagerDirectNavigation.__call__
            assert target._state is actions.state is state
            assert target._workspace is actions.workspace is workspace
            assert target._waypoints is actions.waypoints is points
            assert target._navigation is receiver and receiver.executor is executor
            assert target._values.map_reference is MapRef and target._values.pose is Pose2D
            assert target._values.error is DomainError
            reference_owner = MapReferenceStore(reference, "odom", 0.5)
            legacy_home = LegacyHomeService(database, reference_owner, robot_id="robot")
            peer = LegacyHomeService(database, reference_owner, robot_id="robot")
            assert LegacyHomeService is not ManagerHomeService
            assert isinstance(legacy_home, ManagerHomeService)
            for name in ("prepare", "initialize", "return_home", "note_terminal", "_write_guard"):
                assert getattr(LegacyHomeService, name) is getattr(ManagerHomeService, name)
            assert legacy_home._database is database and legacy_home._store._database is database
            assert (
                legacy_home._home_execution
                is peer._home_execution
                is database.home_execution("robot")
            )
            assert legacy_home._return_lock is peer._return_lock is legacy_home._home_execution.lock
            assert (
                legacy_home._home_target_lock
                is peer._home_target_lock
                is database.home_target_lock("robot")
            )
            assert legacy_home._reference is reference_owner
            assert legacy_home._ports.reference is reference_owner
            assert legacy_home._ports.place == reference_owner.place
            assert legacy_home._ports.apply_anchor_update == reference_owner.apply_anchor_update
            assert legacy_home._ports.pose is Pose2D and legacy_home._ports.error is DomainError
            assert legacy_home._ports.result is HomeResult and legacy_home._ports.goal is HomeGoal
            assert legacy_home._ports.link is HomeLink
            assert legacy_home._ports.succeeded_status is OperationStatus.SUCCEEDED
            assert HomeResult.__module__ == HomeGoal.__module__ == "senior_control.waypoints.home"
            await legacy_home.prepare()
            assert await legacy_home._read_guard() is None
            assert executor.errors.domain is DomainError
            assert executor.errors.route_rejected is RouteRejected
            assert executor.errors.qualification_unavailable is QualificationUnavailable
            unavailable = executor.errors.qualification_unavailable()
            assert type(unavailable) is QualificationUnavailable and isinstance(
                unavailable, DomainError
            )
            assert executor.errors.code(unavailable) is ErrorCode.UNAVAILABLE
            assert str(unavailable) == "qualified task navigation is unavailable"
            assert issubclass(RouteRejected, DomainError)
            context = Context(True, True, "pose", "map")
            service = LifecycleService(durable, effects, context=lambda: context)
            pose = Pose2D(1.0, 2.0, 0.0)
            transition = await service.submit(
                Command("request", "submit", "execution", ("point",)),
                saved,
                created,
                execution_poses=(pose,),
            )
            assert transition.receipt is not None and transition.receipt.accepted
            assert await store.execution_route("execution") == ((pose.x, pose.y, pose.yaw),)
            assert len(await durable.effects("execution")) == 1
            assert client._writer is None
        async with Database(path) as database:
            store = TaskStore(database, robot_id="robot")
            await store.prepare()
            assert await store.get("task") == saved
            reopened = WaypointStore(database, robot_id="robot")
            assert (await reopened.waypoint("point")).waypoint == point

    with tempfile.TemporaryDirectory() as directory:
        asyncio.run(verify(Path(directory)))
    print(
        "installed original task/waypoint DB, constructors, reads/reopen/CAS/error identity: PASS"
    )


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
    from vehicle_manager.settings.models import (
        DrivingSettings,
        SettingsSnapshot,
        vehicle_settings_profile,
    )
    from vehicle_manager.settings.service import RuntimeGate, SettingsService
    from vehicle_manager.settings.store import SettingsStore
    from vehicle_manager.storage import CORE_MIGRATIONS, Migration, migrate, migrate_core
    from vehicle_manager.storage.agent_schema import AGENT_JOURNAL_STATEMENTS
    from vehicle_manager.storage.database import Database as ManagerDatabase
    from vehicle_manager.tasks.lifecycle_service import LifecycleService as ManagerLifecycleService
    from vehicle_manager.tasks.lifecycle_store import (
        SCHEMA as ManagerLifecycleSchema,
    )
    from vehicle_manager.tasks.lifecycle_store import (
        LifecycleStore as ManagerLifecycleStore,
    )
    from vehicle_manager.tasks.lifecycle_store import (
        RequestConflict as ManagerRequestConflict,
    )
    from vehicle_manager.tasks.lifecycle_store import (
        StaleEffect as ManagerStaleEffect,
    )
    from vehicle_manager.tasks.lifecycle_store import (
        VersionConflict as ManagerVersionConflict,
    )
    from vehicle_manager.tasks.store import TASK_STATEMENTS
    from vehicle_manager.tasks.store import TaskStore as ManagerTaskStore
    from vehicle_manager.vehicle_logs.models import LogQuery
    from vehicle_manager.vehicle_logs.service import LogSubscription
    from vehicle_manager.vehicle_logs.service import VehicleLogService as ManagerLogService
    from vehicle_manager.vehicle_logs.store import VehicleLogStore

    from senior_control.agent.journal import AGENT_JOURNAL_STATEMENTS as LegacyJournalStatements
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
    from senior_control.settings.models import DrivingSettings as LegacyDrivingSettings
    from senior_control.settings.service import RuntimeGate as LegacyRuntimeGate
    from senior_control.settings.service import SettingsService as LegacySettingsService
    from senior_control.settings.store import SettingsStore as LegacySettingsStore
    from senior_control.storage.core_schema import CORE_MIGRATIONS as LegacyCoreMigrations
    from senior_control.storage.core_schema import migrate_core as old_migrate_core
    from senior_control.storage.database import Database as LegacyDatabase
    from senior_control.storage.migrations import Migration as LegacyMigration
    from senior_control.storage.migrations import migrate as old_migrate
    from senior_control.storage.schema import DOMAIN_MIGRATIONS
    from senior_control.tasks.lifecycle_service import LifecycleService as LegacyLifecycleService
    from senior_control.tasks.lifecycle_store import (
        SCHEMA as LegacyLifecycleSchema,
    )
    from senior_control.tasks.lifecycle_store import (
        LifecycleStore as LegacyLifecycleStore,
    )
    from senior_control.tasks.lifecycle_store import (
        RequestConflict as LegacyRequestConflict,
    )
    from senior_control.tasks.lifecycle_store import (
        StaleEffect as LegacyStaleEffect,
    )
    from senior_control.tasks.lifecycle_store import (
        VersionConflict as LegacyVersionConflict,
    )
    from senior_control.tasks.store import TaskStore as LegacyTaskStore
    from senior_control.vehicle_logs.models import LogQuery as LegacyLogQuery
    from senior_control.vehicle_logs.service import LogSubscription as LegacyLogSubscription
    from senior_control.vehicle_logs.service import VehicleLogService as LegacyLogService
    from senior_control.vehicle_logs.store import VehicleLogStore as LegacyLogStore
    from senior_control.vehicle_logs.wire import LogRequest as LocalLogRequest

    assert LegacyCellEvidence is CellEvidence
    assert LegacyMapReferenceStore is MapReferenceStore
    assert LegacyOperationRecord is OperationRecord
    assert LegacyMapOperations is not MapOperations
    assert issubclass(LegacyMapOperations, MapOperations)
    assert LegacyMapOperations.import_bundle is MapOperations.import_bundle
    assert LegacyMapOperations.reconcile is MapOperations.reconcile
    assert DOMAIN_MIGRATIONS[0].statements is OPERATION_JOURNAL_STATEMENTS
    assert LegacyJournalStatements is AGENT_JOURNAL_STATEMENTS
    assert DOMAIN_MIGRATIONS[2].statements is TASK_STATEMENTS
    assert LegacyTaskStore is not ManagerTaskStore
    assert issubclass(LegacyTaskStore, ManagerTaskStore)
    for name in ("prepare", "create", "get", "claim", "claim_in_transaction", "update"):
        assert getattr(LegacyTaskStore, name) is getattr(ManagerTaskStore, name)
    assert LegacyLifecycleStore is ManagerLifecycleStore
    assert LegacyLifecycleService is ManagerLifecycleService
    assert LegacyRequestConflict is ManagerRequestConflict
    assert LegacyStaleEffect is ManagerStaleEffect
    assert LegacyVersionConflict is ManagerVersionConflict
    assert LegacyLifecycleSchema is ManagerLifecycleSchema

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
    for local in (DomainError, Velocity, MotionRuntimeState):
        assert local.__module__.startswith("senior_control."), local
    assert RoutePlan.__module__ == "vehicle_core.route_plan"
    assert DrivingSettings.__module__ == "vehicle_manager.settings.models"
    assert LegacyDrivingSettings is DrivingSettings
    assert LegacySettingsService is SettingsService
    assert LegacyRuntimeGate is RuntimeGate
    assert LegacySettingsStore is SettingsStore
    manager = importlib.import_module("vehicle_manager")
    assert (Path(manager.__file__).parent / "py.typed").is_file()
    assert all(
        not any(
            name in dependency.lower()
            for name in (
                "senior-control",
                "senior-robot",
                "rclpy",
                "vehicle-cloud",
                "senior-motion",
                "senior-perception",
                "senior-localization",
                "senior-mapping",
            )
        )
        for dependency in importlib.metadata.requires("vehicle-manager") or []
    )
    # Import isolation must be checked before any other installed SDK code runs.
    import subprocess

    subprocess.run(
        [
            sys.executable,
            "-c",
            (
                "import sys; from senior_control.settings.models import DrivingSettings; "
                "from vehicle_manager.settings.models import DrivingSettings as New; "
                "assert DrivingSettings is New; "
                "assert 'vehicle_manager.settings.service' not in sys.modules; "
                "assert 'vehicle_manager.settings.store' not in sys.modules; "
                "assert 'senior_control.storage.database' not in sys.modules; "
                "from senior_control import settings as old; "
                "from vehicle_manager import settings as new; "
                "assert old.RuntimeGate is new.RuntimeGate; "
                "assert old.SettingsService is new.SettingsService"
            ),
        ],
        check=True,
    )
    subprocess.run(
        [
            sys.executable,
            "-c",
            (
                "import sys; import senior_control.tasks as old; "
                "import vehicle_manager.tasks as new; "
                "assert old.Schedule is new.Schedule; "
                "from senior_control.tasks import lifecycle as before; "
                "from vehicle_manager.tasks import lifecycle as after; "
                "names = ('Action', 'Phase', 'StopReason', 'EventKind', 'TERMINAL', "
                "'STOP_PRIORITY', 'Context', 'Command', 'Receipt', 'Effect', 'Lifecycle', "
                "'Event', 'Transition', 'query', 'command', 'acknowledge', 'event'); "
                "assert all(getattr(before, name) is getattr(after, name) for name in names); "
                "assert not any(name in sys.modules for name in "
                "('senior_control.tasks.models', 'senior_control.tasks.runner', "
                "'senior_control.tasks.store', 'senior_control.tasks.executor', "
                "'senior_control.storage.database', 'senior_mapping', 'rclpy')); "
                "from vehicle_manager.tasks import models as model, TaskRunner as NewRunner; "
                "names = ('ExecutionContext', 'ExecutionRecord', 'ExecutionStatus', "
                "'IntentAction', "
                "'SkipRecord', 'TaskDefinition', 'TaskIntent'); "
                "assert all(getattr(old, name) is getattr(model, name) for name in names); "
                "assert old.TaskRunner is NewRunner; "
                "assert not any(name in sys.modules for name in "
                "('senior_control.tasks.models', 'senior_control.tasks.runner', "
                "'senior_control.tasks.store', 'senior_control.tasks.executor', "
                "'senior_mapping')); "
                "from senior_control.tasks import models as legacy, runner as legacy_runner; "
                "assert all(getattr(legacy, name) is getattr(model, name) for name in names); "
                "from vehicle_manager.tasks import runner; "
                "assert legacy_runner._ACTIVE is runner._ACTIVE; "
                "assert legacy_runner._scan is runner._scan; "
                "from senior_control.tasks import TaskStore, TaskExecutor, TaskRunner; "
                "from senior_control.tasks.store import TaskStore as Store; "
                "from senior_control.tasks.executor import TaskExecutor as Executor; "
                "from senior_control.tasks.runner import TaskRunner as Runner; "
                "assert TaskStore is Store and TaskExecutor is Executor and TaskRunner is Runner"
            ),
        ],
        check=True,
    )
    verify_manager_tasks()
    verify_manager_task_store()
    verify_manager_lifecycle()
    verify_legacy_task_store()
    verify_lazy_shared_imports()
    verify_home_database(compatible=True)
    verify_manager_config(compatible=True)
    verify_legacy_operator()
    verify_navigation()
    verify_legacy_media()
    subprocess.run(
        [
            sys.executable,
            "-c",
            "import sys; from senior_control.waypoints import Waypoint; "
            "from senior_control.waypoints.store import Waypoint as Original; "
            "assert Waypoint is Original; "
            "assert 'senior_control.waypoints.home' not in sys.modules; "
            "from senior_control.waypoints import HomeService; "
            "from senior_control.waypoints.home import HomeService as Owner; "
            "assert HomeService is Owner",
        ],
        check=True,
    )
    # The local caller still supplies its real MapRef; the manager never wraps it.
    from datetime import datetime, timezone

    from vehicle_manager.tasks import ExecutionContext, Schedule, TaskDefinition, TaskRunner

    from senior_control.models import MapRef

    reference = MapRef("yard", 1)
    created = datetime(2026, 1, 1, tzinfo=timezone.utc)
    definition = TaskDefinition(
        "task",
        "robot",
        reference,
        ("point",),
        Schedule.interval(created, 60),
        "Asia/Shanghai",
        True,
        0,
        created,
    )
    context = ExecutionContext(True, True, True, False, reference)
    runner = TaskRunner((definition,))
    assert definition.map_ref is reference and context.active_map is reference
    assert runner._tasks[0] is definition
    assert definition.map_ref == MapRef("yard", 1)
    assert LegacyLogQuery is LogQuery and LegacyLogStore is VehicleLogStore
    assert LegacyLogSubscription is LogSubscription
    assert LegacyLogService is not ManagerLogService
    assert issubclass(LegacyLogService, ManagerLogService)
    for name in ("enqueue", "start", "query", "follow", "download", "close"):
        assert getattr(LegacyLogService, name) is getattr(ManagerLogService, name)
    # Production constructor binds the same diagnostic classes and process owner.
    from senior_control.agent.logs_wire import CloudLogGrants
    from senior_control.runtime.diagnostics import (
        DiagnosticAsyncQueue,
        DiagnosticLock,
        PendingCounter,
        process_diagnostics,
    )

    assert LegacyDatabase is not ManagerDatabase
    assert issubclass(LegacyDatabase, ManagerDatabase)
    for name in ("__aenter__", "_open", "__aexit__"):
        assert getattr(LegacyDatabase, name) is getattr(ManagerDatabase, name)
    assert LegacyMigration is Migration and old_migrate is migrate
    assert LegacyCoreMigrations is CORE_MIGRATIONS and old_migrate_core is migrate_core
    database = LegacyDatabase(Path("/tmp/installed-vehicle-owner-not-opened"))
    assert isinstance(database.write_lock, DiagnosticLock)
    assert database.write_lock._diagnostic.diagnostics is process_diagnostics()
    assert database.home_target_lock("robot") is database.home_target_lock("robot")
    assert database.home_execution("robot") is database.home_execution("robot")
    assert not hasattr(ManagerDatabase, "home_execution")
    verify_manager_storage()

    profile = vehicle_settings_profile()
    policy = SettingsSnapshot(driving=profile.driving_defaults, logging=profile.logging_defaults)
    service = LegacyLogService(
        VehicleLogStore(Path("/tmp/installed-log-owner-not-opened")),
        "robot",
        CloudLogGrants(),
        lambda: policy,
    )
    assert service._diagnostics.counter is PendingCounter
    assert service._diagnostics.lock is DiagnosticLock
    assert service._diagnostics.outbox is DiagnosticAsyncQueue
    assert isinstance(service._queue_diagnostic, PendingCounter)
    assert service._queue_diagnostic.diagnostics is process_diagnostics()
    assert isinstance(service._publication_lock, DiagnosticLock)
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
        "vehicle-manager",
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
    if sys.argv[1:] == ["--lazy-shared-only"]:
        verify_lazy_shared_imports()
    elif sys.argv[1:] == ["--manager-route-only"]:
        verify_manager_core_distribution()
        verify_route_execution()
    elif sys.argv[1:] == ["--manager-route-compatible"]:
        verify_route_execution(compatible=True)
    elif sys.argv[1:] == ["--manager-production-only"]:
        verify_manager_core_distribution()
        verify_manager_config()
        verify_production_bindings()
    elif sys.argv[1:] == ["--manager-production-compatible"]:
        verify_manager_config(compatible=True)
        verify_production_bindings(compatible=True)
    elif sys.argv[1:] == ["--manager-local-ipc-only"]:
        verify_manager_core_distribution()
        verify_local_ipc()
    elif sys.argv[1:] == ["--manager-local-ipc-compatible"]:
        verify_local_ipc(compatible=True)
    elif sys.argv[1:] == ["--home-database-compatible"]:
        verify_home_database(compatible=True)
    elif sys.argv[1:] == ["--manager-config-only"]:
        verify_manager_config()
    elif sys.argv[1:] == ["--manager-config-compatible"]:
        verify_manager_config(compatible=True)
    elif sys.argv[1:] == ["--navigation-only"]:
        verify_navigation()
    elif sys.argv[1:] == ["--platform-runtime-only"]:
        verify_platform_runtime()
    elif sys.argv[1:] == ["--manager-media-only"]:
        verify_manager_media()
    elif sys.argv[1:] == ["--manager-video-only"]:
        verify_manager_media(backend=True)
    elif sys.argv[1:] == ["--real-preflight-only"]:
        verify_real_preflight()
    elif sys.argv[1:] == ["--manager-agent-only"]:
        import importlib.util

        verify_manager_core_distribution()
        for package in ("cloud", "rclpy", "senior_mapping", "senior_robot"):
            assert importlib.util.find_spec(package) is None, package
        verify_manager_agent(network=True)
        verify_manager_operator()
    elif sys.argv[1:] in (["--manager-storage-only"], ["--manager-tasks-only"]):
        import importlib.util

        verify_manager_core_distribution()
        for package in ("cloud", "rclpy", "senior_mapping", "senior_robot"):
            assert importlib.util.find_spec(package) is None, package
        if sys.argv[1:] == ["--manager-storage-only"]:
            verify_manager_storage()
        else:
            verify_manager_tasks()
            assert "aiosqlite" not in sys.modules and "pydantic" not in sys.modules
            verify_home_database()
            verify_manager_task_store()
            verify_manager_lifecycle()
            verify_manager_execution()
            verify_manager_waypoints()
            verify_manager_agent()
            verify_manager_operator()
    else:
        main()
