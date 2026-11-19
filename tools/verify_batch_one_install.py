"""Verify real installed wheels from outside the checkout; no service is started.

Run with a clean interpreter after installing contracts, control and legacy manager
wheels. Native checks require a built ROS prefix in AMENT_PREFIX_PATH.
"""

import importlib
import importlib.metadata
import sys
from pathlib import Path


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
    print("installed manager TaskStore interface without Root/ROS owners: PASS")


def verify_manager_lifecycle() -> None:
    from vehicle_manager import tasks
    from vehicle_manager.tasks.lifecycle_service import LifecycleService
    from vehicle_manager.tasks.lifecycle_store import LifecycleStore

    assert tasks.LifecycleStore is LifecycleStore
    assert tasks.LifecycleService is LifecycleService
    assert LifecycleStore.__module__ == "vehicle_manager.tasks.lifecycle_store"
    assert LifecycleService.__module__ == "vehicle_manager.tasks.lifecycle_service"
    print("installed manager lifecycle interfaces without Root/ROS owners: PASS")


def verify_legacy_task_store() -> None:
    """Reuse the original real SQLite create/reopen/conflict path in the installed SDK."""
    import asyncio
    import tempfile
    from datetime import datetime, timezone

    from senior_control.agent.ports import IpcNavigation
    from senior_control.errors import DomainError, ErrorCode
    from senior_control.models import MapRef, Pose2D
    from senior_control.runtime.ipc import IpcClient
    from senior_control.storage.database import Database
    from senior_control.tasks import Schedule, TaskDefinition
    from senior_control.tasks.lifecycle import Command, Context
    from senior_control.tasks.lifecycle_service import LifecycleService
    from senior_control.tasks.lifecycle_store import LifecycleStore
    from senior_control.tasks.navigation_port import NavigationEffects
    from senior_control.tasks.store import TaskStore

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
            effects = NavigationEffects(durable, IpcNavigation(client, tasks=store))
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

    with tempfile.TemporaryDirectory() as directory:
        asyncio.run(verify(Path(directory)))
    print("installed original TaskStore DB/constructors, create/reopen/CAS/error identity: PASS")


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
    for local in (DomainError, Velocity, RoutePlan, MotionRuntimeState):
        assert local.__module__.startswith("senior_control."), local
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
    if sys.argv[1:] in (["--manager-storage-only"], ["--manager-tasks-only"]):
        import importlib.util

        for package in ("senior_control", "cloud", "rclpy", "senior_mapping", "senior_robot"):
            assert importlib.util.find_spec(package) is None, package
        if sys.argv[1:] == ["--manager-storage-only"]:
            verify_manager_storage()
        else:
            verify_manager_tasks()
            assert "aiosqlite" not in sys.modules and "pydantic" not in sys.modules
            verify_manager_task_store()
            verify_manager_lifecycle()
    else:
        main()
