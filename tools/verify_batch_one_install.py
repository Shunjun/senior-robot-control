"""Verify installed canonical packages outside the checkout without starting services.

The retired senior_control import namespace must be absent. The distribution
name senior-control remains stable and now ships only vehicle_core.
"""

import importlib
import importlib.metadata
import importlib.util
import sys
from pathlib import Path


def verify_production_entry(*, compatible: bool = False) -> None:
    """Run the installed production assembly against local IPC and in-memory WS peers."""
    import asyncio
    import os
    import subprocess
    import tempfile
    import time
    from unittest.mock import patch

    from vehicle_manager.agent import production
    from vehicle_manager.agent.__main__ import main
    from vehicle_manager.agent.config import AgentConfig

    from vehicle_core.ipc import IpcClient, IpcServer

    assert production.run_agent.__module__ == "vehicle_manager.agent.production"
    assert callable(main)
    assert "site-packages" in str(Path(production.__file__).resolve())
    forbidden = (
        "senior_control",
        "senior_mapping",
        "senior_robot",
        "senior_navigation",
        "cloud",
        "rclpy",
        "launch",
        "aiortc",
        "av",
        "httpx",
        "websockets",
    )

    def isolated() -> None:
        assert not any(
            name == prefix or name.startswith(prefix + ".")
            for name in sys.modules
            for prefix in forbidden
        )

    isolated()
    if not compatible:
        from vehicle_core.errors import DomainError, ErrorCode

        try:
            production.load_exported_grid(Path("/tmp/not-opened-display.zip"), "0" * 64, "robot")
        except DomainError as exc:
            assert exc.code is ErrorCode.UNAVAILABLE
            assert isinstance(exc.__cause__, ModuleNotFoundError)
            assert exc.__cause__.name == "senior_mapping"
        else:
            raise AssertionError("missing optional mapping feature was not reported")
    environment = {
        key: value
        for key, value in os.environ.items()
        if not key.startswith("SENIOR_") and key != "PYTHONPATH"
    }
    for arguments, code, expected in ((["--help"], 0, "usage:"), ([], 78, "SENIOR_ROBOT_ID")):
        result = subprocess.run(
            [sys.executable, "-m", "vehicle_manager.agent", *arguments],
            cwd="/tmp",
            env=environment,
            capture_output=True,
            text=True,
            timeout=15,
        )
        assert result.returncode == code, result.stderr
        assert expected in result.stdout + result.stderr
    invalid = dict(
        environment,
        SENIOR_ROBOT_ID="robot",
        SENIOR_DEVICE_TOKEN="private-token",
        SENIOR_CLOUD_URL="ws://remote.test/ws/agent",
        SENIOR_DATABASE_PATH="/tmp/never-open-entry.db",
        SENIOR_AGENT_SOCKET="/tmp/never-connect-entry.sock",
    )
    rejected = subprocess.run(
        [sys.executable, "-m", "vehicle_manager.agent"],
        cwd="/tmp",
        env=invalid,
        capture_output=True,
        text=True,
        timeout=15,
    )
    assert rejected.returncode == 78 and "authenticated TLS" in rejected.stderr
    assert "private-token" not in rejected.stdout + rejected.stderr

    class Link:
        epoch = 1

        def __init__(self):
            self.snapshots = []
            self.closed = False
            self.heartbeats = 0

        async def recv(self):
            await asyncio.Future()

        async def send_snapshot(self, snapshot):
            self.snapshots.append(snapshot)

        async def send_heartbeat(self):
            self.heartbeats += 1

        async def close(self):
            self.closed = True

    async def lifecycle(directory: Path) -> None:
        link = Link()
        agents, clients, closed, requests = [], [], [], []
        database_exit = production.Database.__aexit__

        async def peer(message):
            kind = message["type"]
            requests.append(kind)
            if kind == "state.get":
                return dict(
                    ok=True,
                    sampled_at=time.monotonic(),
                    localized=False,
                    safety_clear=True,
                    manual_driving=False,
                    map_id=None,
                    reference_revision=None,
                    execution_status="idle",
                )
            assert kind in {"identity", "disable_motion", "exit"}, kind
            return {"ok": True}

        server = IpcServer(directory / "agent.sock", peer)
        await server.start()

        async def connect():
            return link

        async def logs(_self, _url, _token):
            # Only the remote log transport is replaced, never the log/SQLite owner.
            await asyncio.Future()

        def client(*args, **kwargs):
            value = IpcClient(*args, **kwargs)
            clients.append(value)
            return value

        async def database_close(database, *args):
            await database_exit(database, *args)
            closed.append(not database.is_open)

        config = AgentConfig(
            "robot",
            "wss://cloud.invalid/ws/agent",
            "test-token",
            directory / "agent.db",
            directory / "agent.sock",
        )
        with (
            patch.object(production, "IpcClient", client),
            patch.object(production, "WebSocketConnector", lambda *args: connect),
            patch.object(production.AgentLogConnection, "run", logs),
            patch.object(production.Database, "__aexit__", database_close),
            patch.object(
                production.Agent, "install_signal_handlers", lambda self: agents.append(self)
            ),
        ):
            runner = asyncio.create_task(production.run_agent(config))
            try:

                async def reporting():
                    while len(link.snapshots) < 2:
                        if runner.done():
                            await runner
                            raise AssertionError("entry exited before reporting")
                        await asyncio.sleep(0.01)

                await asyncio.wait_for(reporting(), 5)
                assert len(clients) == len({id(value) for value in clients}) == 9
                # Control, navigation and long-running mapping retain independent deadlines.
                assert sorted(value._timeout for value in clients) == [1.0] * 7 + [3.0, 30.0]
                agent = agents[0]
                database = agent._journal._database
                tasks = agent._dispatcher._tasks._store
                assert tasks._database is agent._state._waypoints._database is database
                assert agent._state._tasks is tasks
                assert agent._motion.tasks._database.connection is database.connection
                assert agent._motion.tasks._database.write_lock is database.write_lock
                assert agent.connected and "state.get" in requests
                assert not runner.done() and not link.closed
                assert link.snapshots[-1].vehicle_ui["observation"]["status"] != "ipc_unavailable"
                isolated()
            finally:
                if agents:
                    agents[0].request_shutdown()
                try:
                    assert await asyncio.wait_for(runner, 5) == 0
                finally:
                    await server.close()
        assert closed == [True] and link.closed
        assert all(value._writer is None for value in clients)
        assert "disable_motion" in requests and "exit" in requests
        assert not (directory / "agent.sock").exists()

    with tempfile.TemporaryDirectory(prefix="entry-", dir="/tmp") as directory:
        asyncio.run(lifecycle(Path(directory)))
    isolated()
    print(
        "installed canonical Agent: CLI exits, real SQLite/9 IPC clients/"
        "state reporting/cleanup, isolation: PASS"
    )


def verify_navigation() -> None:
    """Verify canonical navigation contracts and the functional ROS installation."""
    from senior_navigation import navigation_adapter, route

    from vehicle_core.errors import DomainError, ErrorCode
    from vehicle_core.values import MapRef, Pose2D

    assert route.MapRef is MapRef and route.Pose2D is Pose2D
    assert issubclass(route.RouteRejected, DomainError)
    assert route.RouteRejected().code is ErrorCode.UNAVAILABLE
    assert route.RoutePlan.__module__ == "vehicle_core.route_plan"
    assert navigation_adapter.RouteService.__module__ == "senior_navigation.navigation_adapter"
    print("installed navigation canonical route contracts: PASS")
    try:
        from ament_index_python.packages import get_package_prefix, get_package_share_directory
    except ModuleNotFoundError as exc:
        if exc.name not in {"ament_index_python", "ament_index_python.packages"}:
            raise
        print("navigation ROS executable/share installation: UNVERIFIED (ament/ROS prefix absent)")
        return
    from senior_navigation.dedicated_controller import DedicatedController

    assert DedicatedController.__module__ == "senior_navigation.dedicated_controller"
    nav = Path(get_package_share_directory("senior_navigation"))
    executable = (
        Path(get_package_prefix("senior_navigation")) / "lib/senior_navigation/navigation_adapter"
    )
    assert executable.is_file() and executable.stat().st_mode & 0o111, executable
    for asset in (
        "config/nav2.yaml",
        "config/lattice_diff.json",
        "behavior_trees/navigate.xml",
        "behavior_trees/navigate_through_poses.xml",
        "launch/navigation.launch.py",
    ):
        assert (nav / asset).is_file(), asset
    print("installed navigation canonical executable and assets: PASS")


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
        from vehicle_manager.agent.pose_reply import PoseReply as OldPoseReply
        from vehicle_manager.agent.production import AgentConfig as OldConfig

        assert OldConfig is AgentConfig
        assert OldConfig.__post_init__ is AgentConfig.__post_init__
        assert OldConfig.from_environment.__func__ is AgentConfig.from_environment.__func__
        assert OldPoseReply is PoseReply
    else:
        for name in ("senior_control", "cloud", "rclpy", "senior_robot"):
            assert name not in sys.modules, name
        print("installed manager config/PoseReply without legacy SDK or ROS: PASS")


def verify_workspace_domain(*, compatible: bool = False) -> None:
    """Use real canonical stores and explicit workspace effects without a running map process."""
    import asyncio
    import hashlib
    import inspect
    import tempfile
    from dataclasses import replace
    from datetime import datetime, timezone
    from types import SimpleNamespace
    from unittest.mock import AsyncMock

    from pydantic import ValidationError
    from senior_contracts.wire import MapRefPayload
    from vehicle_manager.agent import workspace_bindings
    from vehicle_manager.agent.workspace import WorkspaceReader as GenericWorkspace
    from vehicle_manager.storage import domain_bindings
    from vehicle_manager.storage.bindings import Database
    from vehicle_manager.storage.core_schema import CORE_MIGRATIONS
    from vehicle_manager.tasks.models import TaskDefinition
    from vehicle_manager.tasks.schedule import Schedule
    from vehicle_manager.tasks.store import TASK_STATEMENTS
    from vehicle_manager.tasks.store import TaskStore as GenericTasks
    from vehicle_manager.waypoints.store import WAYPOINT_STATEMENTS
    from vehicle_manager.waypoints.store import WaypointStore as GenericPoints

    from vehicle_core import workspace_limits
    from vehicle_core.errors import DomainError, ErrorCode
    from vehicle_core.ipc import IpcError
    from vehicle_core.map_journal_schema import OPERATION_JOURNAL_STATEMENTS
    from vehicle_core.values import MapRef, Pose2D

    for value, base, methods in (
        (domain_bindings.TaskStore, GenericTasks, ("prepare", "create", "get")),
        (domain_bindings.WaypointStore, GenericPoints, ("prepare", "save_waypoint", "save_home")),
        (workspace_bindings.WorkspaceReader, GenericWorkspace, ("read", "_map", "active_map")),
    ):
        assert "site-packages" in inspect.getfile(value), value
        assert value is not base and issubclass(value, base)
        for name in methods:
            assert getattr(value, name) is getattr(base, name)
    migrations = domain_bindings.DOMAIN_MIGRATIONS
    assert tuple(m.version for m in migrations) == (1, 2, 3, 4)
    assert migrations[0].statements is OPERATION_JOURNAL_STATEMENTS
    assert (
        migrations[1].statements is WAYPOINT_STATEMENTS
        and migrations[2].statements is TASK_STATEMENTS
    )
    assert (
        workspace_limits.TILE_SIZE,
        workspace_limits.MAX_GRID_SIDE,
        workspace_limits.MAX_GRID_CELLS,
    ) == (32, 65_536, 16_777_216)

    async def verify(directory):
        async with Database(directory / "domain.db") as database:
            tasks = domain_bindings.TaskStore(database, robot_id="robot")
            points = domain_bindings.WaypointStore(database, robot_id="robot")
            connection, lock = database.connection, database.write_lock
            await tasks.prepare()
            await points.prepare()
            assert tasks._database is points._database is database
            assert database.connection is connection and database.write_lock is lock
            assert tasks._ports.map_reference is MapRef and points._ports.pose is Pose2D
            assert points._ports.waypoint is domain_bindings.Waypoint
            assert points._ports.stored is domain_bindings.StoredWaypoint
            for query, expected in (
                ("PRAGMA user_version", 4),
                ("SELECT version FROM core_schema_version WHERE id=1", len(CORE_MIGRATIONS)),
            ):
                async with connection.execute(query) as cursor:
                    assert (await cursor.fetchone())[0] == expected
            ref, pose = MapRef("yard", 1), Pose2D(0.0, 0.0, 0.0)
            point = await points.save_waypoint(
                waypoint_id="point",
                pose=pose,
                map_ref=ref,
                anchor_id="origin",
                anchor_pose=pose,
                source_robot_id="robot",
                status="confirmed",
                latitude=None,
                longitude=None,
            )
            assert type(point) is domain_bindings.Waypoint and point.pose is pose
            assert type(await points.waypoint("point")) is domain_bindings.StoredWaypoint
            created = datetime(2026, 1, 1, tzinfo=timezone.utc)
            definition = TaskDefinition(
                "task",
                "robot",
                ref,
                ("point",),
                Schedule.interval(created, 60, "UTC"),
                "UTC",
                True,
                0,
                created,
            )
            saved = await tasks.create(definition)
            assert await tasks.get("task") == saved
            async with connection.execute(
                "SELECT map_id FROM map_task_link WHERE task_id='task'"
            ) as cursor:
                assert await cursor.fetchone() == ("yard",)
            client = AsyncMock()
            client.request.return_value = {"ok": False, "error": "unavailable"}
            reader = workspace_bindings.WorkspaceReader("robot", tasks, points, client)
            assert reader._tasks is tasks and reader._waypoints is points and reader._maps is client
            assert await reader.active_map() is None
            exported = SimpleNamespace(display_only=True, map_ref=ref)
            for grid, effect, text in (
                (exported, None, "explicit read effect"),
                (exported, False, "explicit read effect"),
                (SimpleNamespace(display_only=False), None, "must be read-only"),
            ):
                try:
                    workspace_bindings.WorkspaceReader(
                        "robot", tasks, points, client, exported_grid=grid, exported_read=effect
                    )
                except ValueError as exc:
                    assert text in str(exc)
                else:
                    raise AssertionError("export silently accepted without its real effect")
            if compatible:
                from senior_mapping import archive as archive_module
                from senior_mapping.evidence import OccupancySnapshot
                from senior_mapping.references import GridExtent, SlamSnapshot
                from senior_mapping.workspace import WorkspaceMap
                from senior_mapping.workspace_export import load_exported_grid

                snapshot = SlamSnapshot(
                    ref,
                    "wheel_odom",
                    0.1,
                    Path("pose_graph"),
                    OccupancySnapshot(ref, ()),
                    (),
                    (),
                    (),
                    GridExtent(1, 1, 0.0, 0.0),
                    (),
                    (),
                    False,
                )
                layers = directory / "layers"
                snapshot.write(layers / "occupancy")
                archive_module._write_grid(
                    snapshot, layers / "grid", archive_module.MAX_BUNDLE_BYTES
                )
                (layers / "waypoints.json").write_bytes(archive_module._dumps({"waypoints": []}))
                files = archive_module._file_bytes(layers, archive_module.MAX_BUNDLE_BYTES)
                # Opaque, unexecuted graph bytes: this only checks the read-only display loader.
                files["pose_graph/graph.posegraph"] = b"local archive fixture"
                manifest = archive_module.BundleManifest(
                    1,
                    tuple(
                        sorted(
                            (name, hashlib.sha256(data).hexdigest()) for name, data in files.items()
                        )
                    ),
                    archive_module.BACKEND_VERSION,
                    ref,
                    archive_module.STABLE_LAYER,
                    (),
                    archive_module.Georeference("unreferenced"),
                    (),
                    archive_module.ROS_VERSION,
                    archive_module.SLAM_VERSION,
                    archive_module.COORDINATE_CONVENTION,
                    None,
                    None,
                    "robot",
                    False,
                    (),
                )
                files["manifest.json"] = archive_module._dumps(
                    archive_module._manifest_json(manifest)
                )
                bundle = directory / "display.zip"
                archive_module._write_zip(bundle, files)
                digest = hashlib.sha256(bundle.read_bytes()).hexdigest()
                grid = load_exported_grid(bundle, digest, "robot")
                assert grid.display_only and not grid.waypoint_writes
                effects = []

                def read_export(value, kind, fields):
                    effects.append(kind)
                    return WorkspaceMap(lambda: value).read(kind, fields)

                reader = workspace_bindings.WorkspaceReader(
                    "robot", tasks, points, client, exported_grid=grid, exported_read=read_export
                )
                payload = MapRefPayload(
                    map_id=ref.map_id, reference_revision=ref.reference_revision
                )
                display = await reader._manifest(payload, allow_export=True)
                assert (
                    display.display_only
                    and not display.navigation_ready
                    and not display.waypoint_writes
                )
                assert effects == ["map.manifest"]
                live = replace(grid, grid_id="live", display_only=False)
                client.request.return_value = {
                    "v": 1,
                    **WorkspaceMap(lambda: live).read("map.manifest", payload.model_dump()),
                }
                assert (await reader._manifest(payload, allow_export=True)).grid_id == "live"
                assert effects == ["map.manifest"]
                for response, code in (
                    ({"ok": False, "error": "conflict"}, ErrorCode.CONFLICT),
                    ({"ok": False, "error": "validation"}, ErrorCode.VALIDATION),
                    ({"ok": True}, ErrorCode.UNAVAILABLE),
                ):
                    client.request.return_value = response
                    try:
                        await reader._manifest(payload, allow_export=True)
                    except DomainError as exc:
                        assert exc.code is code
                        if response.get("ok") is True:
                            assert isinstance(exc.__cause__, ValidationError)
                    else:
                        raise AssertionError("map failure was hidden by the export")
                    assert effects == ["map.manifest"]
                client.request.return_value = {"ok": False, "error": "unavailable"}
                try:
                    await reader.verify_free_target(ref, digest, pose)
                except DomainError as exc:
                    assert exc.code is ErrorCode.UNAVAILABLE
                else:
                    raise AssertionError("display export granted live target verification")
                assert effects == ["map.manifest"]
                client.request.side_effect = IpcError("local IPC unavailable")
                assert (await reader._manifest(payload, allow_export=True)).grid_id == digest
                assert effects == ["map.manifest", "map.manifest"]
                from vehicle_manager.agent import production

                canonical_grid = production.load_exported_grid(bundle, digest, "robot")
                assert canonical_grid == grid
                canonical = production._workspace_reader(
                    "robot", tasks, points, client, exported_grid=canonical_grid
                )
                assert type(canonical) is workspace_bindings.WorkspaceReader
                assert (await canonical._manifest(payload, allow_export=True)).grid_id == digest
                from vehicle_manager.agent.exported_workspace import (
                    WorkspaceReader as LegacyWorkspace,
                )

                legacy = LegacyWorkspace("robot", tasks, points, client, exported_grid=grid)
                assert (await legacy._manifest(payload, allow_export=True)).grid_id == digest
                try:
                    load_exported_grid(bundle, "0" * 64, "robot")
                except DomainError as exc:
                    assert exc.code is ErrorCode.VALIDATION
                else:
                    raise AssertionError("bad archive checksum accepted")

    with tempfile.TemporaryDirectory(prefix="installed-workspace-domain-", dir="/tmp") as directory:
        asyncio.run(verify(Path(directory)))
    if compatible:
        import vehicle_manager.agent.exported_workspace as legacy_workspace
        import vehicle_manager.agent.production as composition
        import vehicle_manager.storage.domain_bindings as old_points
        import vehicle_manager.storage.domain_bindings as schema
        import vehicle_manager.tasks.store as old_tasks
        from senior_mapping import schema as mapping_schema
        from senior_mapping import workspace as mapping_workspace

        assert schema.DOMAIN_MIGRATIONS is migrations
        assert mapping_schema.OPERATION_JOURNAL_STATEMENTS is OPERATION_JOURNAL_STATEMENTS
        for name in ("TILE_SIZE", "MAX_GRID_SIDE", "MAX_GRID_CELLS"):
            assert getattr(mapping_workspace, name) is getattr(workspace_limits, name)
        for name in ("_MapReply", "_Origin", "_Manifest", "_Tile", "_WaypointCreate"):
            assert getattr(legacy_workspace, name) is getattr(workspace_bindings, name)
        assert old_tasks.TaskStore is composition.TaskStore is domain_bindings.TaskStore
        for name in ("Waypoint", "StoredWaypoint", "WaypointStore", "adopted_home_blocks_foreign"):
            assert getattr(old_points, name) is getattr(domain_bindings, name)
        assert composition.WaypointStore is domain_bindings.WaypointStore
        assert composition.WorkspaceReader is legacy_workspace.WorkspaceReader
        assert legacy_workspace.WorkspaceReader is not workspace_bindings.WorkspaceReader
        assert legacy_workspace.WorkspaceReader.read is GenericWorkspace.read
    else:
        assert not any(
            name == "senior_control"
            or name.startswith(
                (
                    "senior_control.",
                    "senior_mapping",
                    "senior_navigation",
                    "senior_robot",
                    "cloud",
                    "rclpy",
                    "nav2",
                )
            )
            for name in sys.modules
        )
    print(
        "installed domain stores/schema and explicit workspace IPC/export effects: PASS", compatible
    )


def verify_snapshot_frames(*, compatible: bool = False) -> None:
    """Read the canonical non-simulation codec and bind display-only production snapshots."""
    import asyncio
    import inspect
    import subprocess
    import tempfile
    import time
    from types import SimpleNamespace
    from unittest.mock import AsyncMock, patch

    from vehicle_manager.agent.local_state import RuntimeState
    from vehicle_manager.agent.snapshot_bindings import SnapshotState, vehicle_frame
    from vehicle_manager.agent.snapshot_state import SnapshotState as GenericSnapshots
    from vehicle_manager.media.frame_bindings import LatestFrameFile
    from vehicle_manager.media.latest_frame import LatestFrameFile as GenericFrames

    from vehicle_core import rgb1
    from vehicle_core.errors import DomainError, ErrorCode
    from vehicle_core.image_frame import ImageFrame
    from vehicle_core.motion_state import MotionMode, MotionState
    from vehicle_core.runtime_state import MotionRuntimeState, RuntimeObservation

    scripts = [
        """
import sys
from vehicle_core.motion_state import MotionMode, MotionState
from vehicle_core import rgb1
assert MotionMode.__module__ == MotionState.__module__ == 'vehicle_core.motion_state'
assert 'vehicle_manager' not in sys.modules and 'senior_control' not in sys.modules
assert rgb1._HEADER.format == '!4sBHHQ' and rgb1._MAGIC == b'RGB1'
assert rgb1._MAX_AGE_NS == 1_000_000_000 and rgb1._POLL_SECONDS == 0.025
"""
    ]
    if compatible:
        scripts.append("""
import sys
from senior_motion_control.manual_control import ManualIngress, MotionRuntimeReceiver
assert callable(ManualIngress) and callable(MotionRuntimeReceiver)
for name in ('vehicle_manager.agent.frame_projection', 'vehicle_manager.agent.state_ports',
             'vehicle_manager.agent.snapshot_bindings', 'vehicle_manager.agent.entry'):
    assert name not in sys.modules, name
""")
    for script in scripts:
        subprocess.run([sys.executable, "-c", script], cwd="/tmp", check=True)
    for value in (MotionMode, MotionState, SnapshotState, LatestFrameFile):
        assert "site-packages" in inspect.getfile(value), value
    assert SnapshotState.__module__ == "vehicle_manager.agent.snapshot_bindings"
    assert LatestFrameFile.__module__ == "vehicle_manager.media.frame_bindings"
    assert rgb1.publish_image.__module__ == rgb1._read_frame.__module__ == "vehicle_core.rgb1"
    for name in ("vehicle_core/motion_state.py", "vehicle_core/rgb1.py"):
        assert Path(name) in (importlib.metadata.distribution("senior-control").files or [])
    for name in (
        "vehicle_manager/agent/snapshot_bindings.py",
        "vehicle_manager/media/frame_bindings.py",
    ):
        assert Path(name) in (importlib.metadata.distribution("vehicle-manager").files or [])
    assert SnapshotState is not GenericSnapshots and issubclass(SnapshotState, GenericSnapshots)
    assert SnapshotState.collect is GenericSnapshots.collect
    assert LatestFrameFile is not GenericFrames and issubclass(LatestFrameFile, GenericFrames)
    assert LatestFrameFile.next_frame is GenericFrames.next_frame
    mode = MotionState(MotionMode.IDLE, "standby", None, 0, 0, True, ("acquire",))
    assert mode.mode is MotionMode.IDLE
    sample = MotionRuntimeState.model_validate(
        {
            "schema": 1,
            "robot_id": "robot",
            "boot_id": "boot",
            "sequence": 1,
            "sampled_steady": 10.0,
            "mode": "idle",
            "reason": "standby",
            "stationary": True,
            "healthy": True,
            "evidence_stamp_steady": 10.0,
            "evidence_deadline_steady": 10.05,
            "estop_revision": 0,
            "fault_revision": 0,
            "confirmed_estop_status": "clear",
            "allowed_actions": ["acquire"],
            "task_id": None,
            "output_generation": 1,
            "body_reference": None,
            "wheel_linear_mps": None,
        }
    )
    before = sample.model_dump()
    arguments = dict(
        sequence=1,
        connected=True,
        task=None,
        manual=None,
        manual_enabled=True,
        tasks_enabled=False,
        runtime_enabled=True,
        safety_revisions=(0, 0),
        now=10.1,
    )
    projected = vehicle_frame(sample, **arguments)
    assert projected["mode"] == "idle" and projected["allowed_actions"] == ["acquire"]
    assert projected["observation"]["status"] == "current"
    assert sample.model_dump() == before and sample.evidence_deadline_steady == 10.05

    async def verify(directory):
        live = RuntimeState(
            False,
            False,
            False,
            None,
            "idle",
            motion_runtime=sample,
            motion_runtime_version=1,
            runtime_observation=RuntimeObservation(status="current", last_sampled_steady=10.0),
        )
        state = SimpleNamespace(read=AsyncMock(return_value=live))
        tasks = SimpleNamespace(snapshot_tasks=AsyncMock(return_value=((), frozenset())))
        points = SimpleNamespace(home=AsyncMock(return_value=None))
        snapshots = SnapshotState(state, tasks, points, manual_enabled=True, clock=lambda: 10.1)
        assert snapshots._ports.frame is vehicle_frame
        assert (
            snapshots._ports.observation is RuntimeObservation
            and snapshots._ports.error is DomainError
        )
        assert (
            snapshots._state is state
            and snapshots._tasks is tasks
            and snapshots._waypoints is points
        )
        first = await snapshots.collect()
        assert first.vehicle_ui["mode"] == "idle"
        state.read.side_effect = DomainError(ErrorCode.UNAVAILABLE, "lost")
        second = await snapshots.collect()
        assert second.vehicle_ui["observation"]["status"] == "ipc_unavailable"
        assert second.vehicle_ui["sequence"] > first.vehicle_ui["sequence"]
        assert sample.model_dump() == before
        path = directory / "frame.rgb"
        assert rgb1.publish_image(path, 1, 1, 3, "rgb8", b"abc")
        reader = LatestFrameFile(path)
        assert reader._read_frame is rgb1._read_frame and reader._poll_seconds == rgb1._POLL_SECONDS
        frame = await reader.next_frame(0.1)
        assert type(frame) is ImageFrame and frame.data == b"abc"
        assert frame.encoding == "rgb8" and frame.width == frame.height == 1
        assert rgb1._read_frame(path, reader._last_capture) is None

    with tempfile.TemporaryDirectory(prefix="installed-snapshot-frame-", dir="/tmp") as directory:
        asyncio.run(verify(Path(directory)))
    if compatible:
        import senior_motion_control.contracts as contracts
        import senior_motion_control.manual_control as manual_control
        import vehicle_manager.agent.production as composition

        import vehicle_core.rgb1 as old_frame_exports
        import vehicle_core.rgb1 as old_rgb1

        assert contracts.MotionMode is MotionMode and contracts.MotionState is MotionState
        assert manual_control.MotionMode is MotionMode and manual_control.MotionState is MotionState
        assert composition.SnapshotState is SnapshotState
        assert composition.LatestFrameFile is old_frame_exports.LatestFrameFile is LatestFrameFile
        for name in (
            "_HEADER",
            "_MAGIC",
            "_ENCODINGS",
            "_MAX_WIDTH",
            "_MAX_HEIGHT",
            "_MAX_AGE_NS",
            "_POLL_SECONDS",
            "_size",
            "_read_frame",
            "publish_image",
        ):
            assert getattr(old_rgb1, name) is getattr(rgb1, name)
            assert getattr(old_frame_exports, name) is getattr(rgb1, name)
        assert inspect.signature(manual_control.vehicle_frame) == inspect.signature(vehicle_frame)
        assert manual_control.vehicle_frame(sample, **arguments) == projected
        signature = inspect.signature(SnapshotState)
        assert signature.parameters["clock"].default is time.monotonic
        assert signature.parameters["manual_enabled"].default is False
        from vehicle_manager.agent import production

        captured = []
        marker = object()

        async def capture(config, *, ports):
            captured.append(ports.snapshot_state)
            return 0

        config = composition.AgentConfig(
            "robot",
            "wss://cloud.invalid",
            "token",
            Path("/tmp/snapshot.db"),
            Path("/tmp/snapshot.sock"),
        )
        with (
            patch.object(production, "SnapshotState", marker),
            patch.object(production, "_run_agent", capture),
        ):
            assert asyncio.run(composition.run_agent(config)) == 0
        assert captured == [marker]
        assert composition._media_binding.__globals__["LatestFrameFile"] is LatestFrameFile
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
                    "av",
                )
            )
            for name in sys.modules
        )
    print(
        "installed shared motion/RGB1, production snapshot/frame and lazy compatibility: PASS",
        compatible,
    )


def verify_request_runtime(*, compatible: bool = False) -> None:
    """Construct the actual request/runtime bindings and commit a local edit without a service."""
    import asyncio
    import inspect
    import tempfile
    import time
    from unittest.mock import AsyncMock, Mock

    from senior_contracts.wire import HEARTBEAT_INTERVAL_MS, OperationEnvelope, OperationPayload
    from vehicle_manager.agent import dispatcher, main, request_bindings, runtime_bindings
    from vehicle_manager.agent.production_bindings import AgentJournal
    from vehicle_manager.media.signaling import VideoSignaling as GenericSignaling
    from vehicle_manager.observability import Metrics
    from vehicle_manager.storage.bindings import Database
    from vehicle_manager.waypoints.direct_navigation import DirectNavigation as GenericDirect

    from vehicle_core.clock import SystemClock
    from vehicle_core.diagnostics import (
        DiagnosticAsyncQueue,
        DiagnosticPending,
        PendingCounter,
        process_diagnostics,
    )
    from vehicle_core.errors import DomainError, ErrorCode
    from vehicle_core.values import MapRef, OperationResult, OperationStatus

    for bound, generic, methods in (
        (request_bindings.Dispatcher, dispatcher.Dispatcher, ("run", "_plan", "_execute")),
        (request_bindings.StoreTaskPort, dispatcher.StoreTaskPort, ("create", "update", "delete")),
        (
            request_bindings.LifecycleNavigationOperations,
            dispatcher.LifecycleNavigationOperations,
            ("query", "accept", "execution_command", "__call__"),
        ),
        (
            request_bindings._TaskRouteReceiver,
            dispatcher._TaskRouteReceiver,
            ("start_poses", "cancel_current"),
        ),
        (request_bindings.DirectNavigation, GenericDirect, ("__call__",)),
        (runtime_bindings.Agent, main.Agent, ("run", "dispatch", "_session", "_release")),
        (runtime_bindings.VideoSignaling, GenericSignaling, ("submit", "close", "retire")),
    ):
        assert "site-packages" in inspect.getfile(bound), bound
        assert bound is not generic and issubclass(bound, generic)
        for name in methods:
            assert getattr(bound, name) is getattr(generic, name)
    assert runtime_bindings.logger is main.logger is dispatcher.logger
    assert runtime_bindings.logger.name == "vehicle_manager.agent"
    assert request_bindings.TaskOperationResult.__bases__ == (OperationResult,)
    assert runtime_bindings.ManualOperationResult.__bases__ == (OperationResult,)
    assert issubclass(request_bindings.OperationNotCommitted, DomainError)
    assert request_bindings._map_ref({"map_id": "yard", "reference_revision": 1}) == MapRef(
        "yard", 1
    )
    values = request_bindings._values()
    assert values.result is OperationResult and values.status is OperationStatus
    assert values.task_result is request_bindings.TaskOperationResult
    assert values.direct_navigation is request_bindings.DirectNavigation
    assert values.not_committed is request_bindings.OperationNotCommitted
    assert values.counter is PendingCounter

    async def verify(directory):
        async with Database(directory / "requests.db") as database:
            journal = AgentJournal(database, robot_id="robot")
            await journal.prepare()
            tasks, home, maps = AsyncMock(), AsyncMock(), AsyncMock()
            tasks.create.return_value = 1
            admissions = []
            metrics = Metrics()
            instance = request_bindings.Dispatcher(
                "robot",
                journal=journal,
                tasks=tasks,
                home=home,
                maps=maps,
                deadlines=request_bindings.Deadlines(),
                metrics=metrics,
                map_root=directory,
                admit=lambda envelope: admissions.append(envelope.type),
            )
            assert instance._journal is journal and journal._database is database
            assert instance._metrics is metrics and instance._tasks is tasks
            store = Mock()
            task_port = request_bindings.StoreTaskPort(store, robot_id="robot")
            assert task_port._store is store and task_port._values.result is OperationResult
            state, workspace, points, navigation = Mock(), Mock(), Mock(), Mock()
            direct = request_bindings.DirectNavigation(state, workspace, points, navigation)
            assert direct._state is state and direct._workspace is workspace
            assert direct._waypoints is points and direct._navigation is navigation
            executor = Mock()
            actions = request_bindings.LifecycleNavigationOperations(
                executor, state, workspace, points
            )
            assert actions.executor is executor and actions.state is state
            assert actions.workspace is workspace and actions.waypoints is points
            receiver = request_bindings._TaskRouteReceiver(executor, "fingerprint", state)
            assert receiver.executor is executor and receiver.state is state
            assert receiver.fingerprint == "fingerprint"
            envelope = OperationEnvelope(
                v=1,
                robot_id="robot",
                session_id="session",
                connection_epoch=1,
                request_id="request",
                seq=1,
                expires_at=time.time() + 30,
                type="task.create",
                payload=OperationPayload(operation_id="installed-request", body={}),
            )
            result = await instance.run(envelope)
            assert type(result) is OperationResult and result.status is OperationStatus.SUCCEEDED
            assert result.operation_id == "installed-request" and admissions == ["task.create"]
            assert (await journal.get("installed-request")).closed
            assert (await journal.get("installed-request")).revision == 1
            replay = await instance.run(envelope)
            assert replay == result
            tasks.create.assert_awaited_once()
            changed = envelope.model_copy(
                update={
                    "payload": OperationPayload(
                        operation_id="installed-request",
                        body={"changed": True},
                    )
                }
            )
            try:
                await instance.run(changed)
            except DomainError as exc:
                assert exc.code is ErrorCode.CONFLICT
            else:
                raise AssertionError("different request content was admitted")
            motion = Mock()
            agent = runtime_bindings.Agent(
                "robot",
                connector=AsyncMock(),
                journal=journal,
                tasks=tasks,
                home=home,
                maps=maps,
                state=Mock(),
                motion=motion,
                metrics=metrics,
            )
            assert type(agent._dispatcher) is request_bindings.Dispatcher
            assert agent._dispatcher._journal is journal and agent.metrics is metrics
            assert agent._ports.error is DomainError and agent._ports.status is OperationStatus
            assert agent._ports.result is OperationResult and agent._ports.metrics is Metrics
            assert agent._ports.manual_result is runtime_bindings.ManualOperationResult
            assert agent._ports.manual_result_type is runtime_bindings.ManualOperationResult
            assert agent._ports.task_result is request_bindings.TaskOperationResult
            assert agent._ports.not_committed is request_bindings.OperationNotCommitted
            assert agent._ports.dispatcher is request_bindings.Dispatcher
            assert (
                agent._ports.lifecycle_navigation is request_bindings.LifecycleNavigationOperations
            )
            assert (
                agent._ports.counter is PendingCounter
                and agent._ports.safety_pending is DiagnosticPending
            )
            assert agent._ports.clock is SystemClock
            assert agent._ports.report_queues is runtime_bindings._report_queues
            assert agent._ports.video_signaling is runtime_bindings.VideoSignaling
            assert agent._intent_queue.diagnostics is process_diagnostics()
            assert agent._heartbeat_ms == HEARTBEAT_INTERVAL_MS and agent._retry_delay_ms == 1000
            assert agent._command_ttl_seconds == 0.3 and agent._snapshot_seconds == 2.0
            assert agent._state_poll_seconds == 0.25 and isinstance(agent._clock, SystemClock)
            assert not agent._inflight and agent._supervisor is None and not agent.connected
            motion.bind_clock.assert_called_once_with(agent._clock.monotonic)
            video = runtime_bindings.VideoSignaling(
                "robot",
                1,
                send=AsyncMock(),
                make_session=lambda: None,
            )
            assert video._queue_factory is DiagnosticAsyncQueue
            assert type(video._candidates) is DiagnosticAsyncQueue and video._task is None
            await video.close()
            home.set_current.assert_not_awaited()
            maps.export.assert_not_awaited()

    with tempfile.TemporaryDirectory(prefix="installed-requests-", dir="/tmp") as directory:
        asyncio.run(verify(Path(directory)))
    if compatible:
        import vehicle_manager.agent.entry as old_main
        import vehicle_manager.agent.production as composition
        import vehicle_manager.agent.request_bindings as direct_navigation
        import vehicle_manager.agent.runtime_bindings as old_signaling
        import vehicle_manager.agent.store_adapters as old_dispatcher
        from vehicle_manager.media.signaling import logger as signaling_logger
        from vehicle_manager.media.stream import VideoSession

        for name in (
            "MapPort",
            "OperationNotCommitted",
            "TaskOperationResult",
            "_values",
            "Dispatcher",
            "StoreTaskPort",
            "LifecycleNavigationOperations",
            "_map_ref",
            "_TaskRouteReceiver",
        ):
            assert getattr(old_dispatcher, name) is getattr(request_bindings, name)
        assert direct_navigation.DirectNavigation is request_bindings.DirectNavigation
        assert direct_navigation._Navigation is request_bindings._Navigation
        for name in ("Agent", "ManualOperationResult", "_report_queues"):
            assert getattr(old_main, name) is getattr(runtime_bindings, name)
        assert old_signaling.VideoSignaling is runtime_bindings.VideoSignaling
        assert (
            old_signaling.logger is signaling_logger and old_signaling.VideoSession is VideoSession
        )
        assert composition.Agent is runtime_bindings.Agent
        assert composition.StoreTaskPort is request_bindings.StoreTaskPort
        assert (
            composition.LifecycleNavigationOperations
            is request_bindings.LifecycleNavigationOperations
        )
        assert old_dispatcher.StoreHomePort is not dispatcher.StoreHomePort
        assert old_dispatcher.StoreHomePort.set_current is dispatcher.StoreHomePort.set_current
        assert (
            old_dispatcher.StoreHomePort.__init__.__globals__["_values"] is request_bindings._values
        )
        assert old_dispatcher.StoreMapPort is not dispatcher.StoreMapPort
        assert old_dispatcher.StoreMapPort.export is dispatcher.StoreMapPort.export
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
                    "av",
                )
            )
            for name in sys.modules
        )
    print(
        "installed bound requests/runtime/video, SQLite commit and independent constructor: PASS",
        compatible,
    )


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
        import vehicle_manager.agent.local_adapters as ports
        import vehicle_manager.agent.production as composition
        import vehicle_manager.tasks.execution_bindings as old_effects
        import vehicle_manager.tasks.executor as old_executor
        from senior_navigation import navigation_adapter, route
        from senior_robot import agent_adapter
        from vehicle_manager.storage.domain_bindings import TaskStore as LegacyStore
        from vehicle_manager.storage.domain_bindings import WaypointStore

        assert route.RoutePlan is RoutePlan and route.RouteRejected is RouteRejected
        assert route.QualifiedRoute.__module__ == "senior_navigation.route"
        assert navigation_adapter.RoutePlan is agent_adapter.RoutePlan is RoutePlan
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
        import vehicle_manager.agent.local_adapters as old_settings
        import vehicle_manager.agent.logs_wire as old_logs
        import vehicle_manager.agent.production as composition
        import vehicle_manager.agent.production_bindings as old_journal
        import vehicle_manager.agent.production_bindings as old_websocket
        import vehicle_manager.storage.bindings as old_database

        import vehicle_core.diagnostics as old_service

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
        import vehicle_manager.agent.connection as connection
        import vehicle_manager.agent.entry as main
        import vehicle_manager.agent.exported_workspace as workspace
        import vehicle_manager.agent.local_adapters as ports
        import vehicle_manager.agent.local_adapters as settings_wire
        import vehicle_manager.agent.production as composition
        import vehicle_manager.agent.production_bindings as websocket
        import vehicle_manager.agent.request_bindings as direct_navigation
        import vehicle_manager.agent.store_adapters as dispatcher
        from vehicle_manager.agent.connection import HeartbeatFrame, Link, MotionPort
        from vehicle_manager.agent.request_ports import Requester

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
    verify_request_runtime()
    verify_snapshot_frames()
    verify_workspace_domain()
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
        assert "senior_robot.service_runtime" in rendered
        assert deployment.environment(profile, root).count("SENIOR_") == 4
    assert not any(
        name == "senior_control" or name.startswith(("senior_control.", "rclpy"))
        for name in sys.modules
    )
    verify_manager_media()
    print("installed manager real profile/rendering without legacy business/ROS: PASS")


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


def verify_real_preflight() -> None:
    """The existing hardware preflight requires the original SDK safety extra."""
    from senior_robot import real_runtime as preflight

    defaults = preflight.real_parameter_defaults()
    assert defaults["scan_receipt"]["scan_topic"] == "/scan"
    assert defaults["scan_receipt"]["use_sim_time"] is False
    # Exercise the actual read-only validation port without starting ROS.
    preflight.validate_navigation(None, None)
    assert "rclpy" not in sys.modules
    print("installed ROS-free real preflight policies and validation: PASS")


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
        import vehicle_manager.storage.domain_bindings as schema
        from senior_mapping.schema import OPERATION_JOURNAL_STATEMENTS
        from vehicle_manager.storage.bindings import Database as OldDatabase
        from vehicle_manager.storage.bindings import HomeExecutionCoordination as OldCoordination

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


def verify_manager_core_distribution() -> None:
    import vehicle_core

    manager = importlib.metadata.distribution("vehicle-manager")
    core = importlib.metadata.distribution("senior-control")
    assert "senior-control==0.1.0" in (manager.requires or [])
    assert "site-packages" in str(Path(vehicle_core.__file__).resolve())
    assert Path("vehicle_core/__init__.py") in (core.files or [])
    assert not any(str(item).startswith("senior_control/") for item in core.files or [])
    assert importlib.util.find_spec("senior_control") is None
    print("installed shared core ownership, manager dependency and retired namespace: PASS")


def verify_lazy_shared_imports() -> None:
    import subprocess

    code = """
import importlib.util
import sys
from vehicle_core.clock import SystemClock
from vehicle_core.image_frame import ImageFrame
from vehicle_core.ipc import IpcClient
from vehicle_core.process_resources import ProcessWorker
from vehicle_core.sensor_policy import default_policy
assert default_policy().hard_age_ms > 0
assert importlib.util.find_spec('senior_control') is None
for name in ('rclpy', 'torch', 'ultralytics', 'cv2', 'aiortc', 'av', 'vehicle_manager'):
    assert name not in sys.modules, name
"""
    subprocess.run([sys.executable, "-c", code], cwd="/tmp", check=True, timeout=15)
    print("installed core resources and lazy imports: PASS")


def main() -> None:
    import subprocess

    modes = {
        "--real-preflight-only": ("verify_real_preflight",),
        "--manager-entry-only": ("verify_production_entry",),
        "--manager-agent-only": ("verify_manager_agent", "verify_production_entry"),
        "--manager-tasks-only": (
            "verify_manager_storage",
            "verify_manager_tasks",
            "verify_manager_task_store",
            "verify_manager_lifecycle",
            "verify_manager_execution",
            "verify_manager_waypoints",
        ),
        "--lazy-shared-only": ("verify_lazy_shared_imports",),
        "": (
            "verify_lazy_shared_imports",
            "verify_manager_storage",
            "verify_navigation",
            "verify_platform_runtime",
        ),
    }
    argument = sys.argv[1] if len(sys.argv) == 2 else "" if len(sys.argv) == 1 else None
    if argument not in modes:
        raise SystemExit("unsupported installed-package check")
    verify_manager_core_distribution()
    # Isolation checks must run before unrelated checks import optional libraries.
    for name in modes[argument]:
        code = (
            "import runpy; checks=runpy.run_path("
            + repr(str(Path(__file__).resolve()))
            + "); checks["
            + repr(name)
            + "]()"
        )
        subprocess.run([sys.executable, "-c", code], cwd="/tmp", check=True, timeout=60)
    if not argument:
        print("Native ROS/DDS/SLAM and vehicle deployment: UNVERIFIED (not run)")


if __name__ == "__main__":
    main()
