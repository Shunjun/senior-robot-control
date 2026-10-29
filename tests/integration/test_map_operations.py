"""Durable map import, export, and clear survive disconnects and crashes."""

import asyncio
import hashlib
import json
import zipfile
from pathlib import Path

import pytest

from senior_control.errors import DomainError, ErrorCode
from senior_control.mapping.archive import BundleManifest, MapArchive
from senior_control.mapping.evidence import CellEvidence
from senior_control.mapping.operations import MapOperations, ProcessCrash
from senior_control.mapping.references import (
    AnchoredWaypoint,
    AnchorPose,
    AnchorSnapshot,
    GridExtent,
    SlamSnapshot,
)
from senior_control.mapping.store import MapReferenceStore
from senior_control.models import MapRef, OperationStatus, Pose2D
from senior_control.storage.database import Database

CELL = 0.5
_HINT = Pose2D(1.0, 2.0, 0.25)


def cell_of(x: float, y: float) -> tuple[int, int]:
    return (int(x // CELL), int(y // CELL))


class Graph:
    def __init__(self, map_ref: MapRef, hint: Pose2D | None = _HINT) -> None:
        self._map_ref = map_ref
        self._hint = hint

    def map_ref(self) -> MapRef:
        return self._map_ref

    def last_pose_hint(self) -> Pose2D | None:
        return self._hint

    def painted_cells(self) -> tuple[tuple[int, int], ...]:
        return ((4, 1),)

    async def write(self, directory: Path) -> tuple[str, ...]:
        directory.mkdir(parents=True, exist_ok=True)
        (directory / "graph.posegraph").write_text(self._map_ref.map_id, encoding="utf-8")
        (directory / "graph.data").write_text("scans", encoding="utf-8")
        return ("graph.posegraph", "graph.data")

    async def read(self, directory: Path) -> None:
        del directory

    async def reset(self) -> None:
        return None


class Gate:
    def __init__(self) -> None:
        self.driving = True
        self.blocked = False
        self.calls: list[str] = []

    async def stop_and_block_navigation(self) -> None:
        self.calls.append("stop_and_block_navigation")
        self.driving = False
        self.blocked = True


class Barrier:
    def __init__(self) -> None:
        self.entered = asyncio.Event()
        self.hold = asyncio.Event()

    async def checkpoint(self) -> None:
        self.entered.set()
        await self.hold.wait()


class Loader:
    def __init__(self, fail: bool = False) -> None:
        self.fail = fail
        self.hints: list[Pose2D | None] = []

    async def load(self, directory: Path, hint: Pose2D | None) -> None:
        del directory
        self.hints.append(hint)
        if self.fail:
            raise RuntimeError("slam rejected the graph")


def filled_store(map_id: str, revision: int, point: tuple[float, float]) -> MapReferenceStore:
    map_ref = MapRef(map_id, revision)
    store = MapReferenceStore(map_ref, "wheel_odom", CELL)
    origin = Pose2D(0.0, 0.0, 0.0)
    store.place(
        CellEvidence(cell_of(*point), "kf-1", 3, 0, 1.0, 3.0, "geometric_confirmation"),
        world=point,
        anchor_pose=origin,
    )
    store.place_waypoint(
        AnchoredWaypoint("home", Pose2D(point[0], point[1], 0.1), "kf-1", False),
        anchor_pose=origin,
    )
    store.apply_anchor_update(
        AnchorSnapshot(
            map_ref,
            "wheel_odom",
            (AnchorPose("kf-1", origin, True),),
            GridExtent(8, 8, 0.0, 0.0),
        )
    )
    return store


def write_config(directory: Path) -> dict[str, bytes]:
    directory.mkdir(parents=True, exist_ok=True)
    files = {
        "credentials.json": b'{"token":"real-secret"}',
        "calibration.json": b'{"lidar":"vehicle"}',
        "footprint.json": b'{"length_m":0.6}',
        "deployment.json": b'{"profile":"hardware"}',
    }
    for name, payload in files.items():
        (directory / name).write_bytes(payload)
    return files


def archive_for(store: MapReferenceStore, robot_id: str) -> MapArchive:
    return MapArchive(
        Graph(store.map_ref),
        source_robot_id=robot_id,
        home_waypoint_id="home",
        sensor_summary=(("lidar", "source-description-only"),),
    )


def operations_for(
    database: Database,
    store: MapReferenceStore,
    root: Path,
    *,
    robot_id: str,
    loader: Loader | None = None,
    checkpoint: Barrier | None = None,
    fault_after: str | None = None,
    gate: Gate | None = None,
) -> MapOperations:
    return MapOperations(
        database,
        store,
        archive_for(store, robot_id),
        gate or Gate(),
        robot_id=robot_id,
        map_root=root / "maps",
        config_dir=root / "config",
        loader=loader,
        checkpoint=checkpoint,
        fault_after=fault_after,
    )


def corrupt_hashes(source: Path, destination: Path) -> None:
    with zipfile.ZipFile(source) as bundle:
        files = {name: bundle.read(name) for name in bundle.namelist()}
    files["occupancy/occupancy.json"] = files["occupancy/occupancy.json"] + b" "
    with zipfile.ZipFile(destination, "w") as bundle:
        for name, payload in files.items():
            bundle.writestr(name, payload)


def rewrite_waypoint_map(source: Path, destination: Path, map_id: str) -> None:
    with zipfile.ZipFile(source) as bundle:
        files = {name: bundle.read(name) for name in bundle.namelist()}
    manifest = json.loads(files["manifest.json"])
    waypoints = json.loads(files["waypoints.json"])
    for item in waypoints["waypoints"]:
        item["map_id"] = map_id
    files["waypoints.json"] = json.dumps(waypoints).encode()
    manifest["file_hashes"] = {
        name: hashlib.sha256(payload).hexdigest()
        for name, payload in files.items()
        if name != "manifest.json"
    }
    files["manifest.json"] = json.dumps(manifest).encode()
    with zipfile.ZipFile(destination, "w") as bundle:
        for name, payload in files.items():
            bundle.writestr(name, payload)


@pytest.mark.asyncio
async def test_corrupt_bundle_preserves_active_map(tmp_path: Path) -> None:
    store = filled_store("yard", 3, (1.25, 0.25))
    original = store.map_ref
    cells = store.occupancy().cells
    gate = Gate()
    good = tmp_path / "good.zip"
    bad = tmp_path / "bad.zip"
    async with Database(tmp_path / "robot.db") as database:
        ops = operations_for(database, store, tmp_path, robot_id="robot-a", gate=gate)
        await ops.prepare()
        await ops.export_bundle(original, good, operation_id="export-good")
        corrupt_hashes(good, bad)
        with pytest.raises(DomainError) as caught:
            await ops.import_bundle(bad, "robot-a", operation_id="import-bad")

    assert caught.value.code is ErrorCode.VALIDATION
    assert ops.store.map_ref == original
    assert ops.store.occupancy().cells == cells
    assert gate.calls == []
    assert gate.driving is True


@pytest.mark.asyncio
async def test_failed_load_restores_map_and_stays_stopped(tmp_path: Path) -> None:
    store = filled_store("yard", 3, (1.25, 0.25))
    original = store.map_ref
    cells = store.occupancy().cells
    gate = Gate()
    loader = Loader(fail=True)
    bundle = tmp_path / "yard.zip"
    async with Database(tmp_path / "robot.db") as database:
        ops = operations_for(
            database, store, tmp_path, robot_id="robot-a", loader=loader, gate=gate
        )
        await ops.prepare()
        await ops.export_bundle(original, bundle, operation_id="export-load")
        with pytest.raises(RuntimeError, match="slam rejected"):
            await ops.import_bundle(bundle, "robot-a", operation_id="import-load")
        record = await ops.operation_record("import-load")

    assert record.result.status is OperationStatus.FAILED
    assert record.relocalized is False
    assert record.rollback_confirmed is True
    assert ops.store.map_ref == original
    assert ops.store.occupancy().cells == cells
    assert gate.calls == ["stop_and_block_navigation"]
    assert gate.driving is False
    assert gate.blocked is True
    active = SlamSnapshot.read(tmp_path / "maps" / "active")
    assert active.map_ref == original


@pytest.mark.asyncio
async def test_clear_invalidates_bound_tasks(tmp_path: Path) -> None:
    store = filled_store("yard", 3, (1.25, 0.25))
    old = store.map_ref
    cells = store.occupancy().cells
    definition = '{"schedule":"0 8 * * *","waypoint_ids":["home"]}'
    gate = Gate()
    async with Database(tmp_path / "robot.db") as database:
        ops = operations_for(database, store, tmp_path, robot_id="robot-a", gate=gate)
        await ops.prepare()
        await ops.bind_task("patrol", definition, old)
        result = await ops.clear_map(old, operation_id="clear-1")
        links = await ops.task_links()
        assert result.status is OperationStatus.SUCCEEDED
        assert result.operation_id == "clear-1"
        assert ops.store.map_ref.map_id != old.map_id
        assert ops.store.map_ref.reference_revision == 0
        assert ops.store.is_empty()
        assert links[0].task_id == "patrol"
        assert links[0].definition == definition
        assert links[0].map_ref == old
        assert links[0].invalidated is True
        history = SlamSnapshot.read(tmp_path / "maps" / "history" / old.map_id / "3")
        active = SlamSnapshot.read(tmp_path / "maps" / "active")

    assert history.map_ref == old
    assert history.occupancy.cells == cells
    assert active.map_ref == ops.store.map_ref
    assert active.occupancy.cells == ()
    assert gate.calls == ["stop_and_block_navigation"]
    assert gate.driving is False


@pytest.mark.asyncio
async def test_cross_robot_home_requires_confirmation(tmp_path: Path) -> None:
    source = filled_store("yard", 3, (1.25, 0.25))
    local = filled_store("local", 1, (2.25, 0.25))
    local_cells = local.occupancy().cells
    bundle = tmp_path / "from-a.zip"
    config = write_config(tmp_path / "b" / "config")
    loader = Loader()
    gate = Gate()
    async with (
        Database(tmp_path / "a.db") as database_a,
        Database(tmp_path / "b.db") as database_b,
    ):
        origin = operations_for(database_a, source, tmp_path / "a", robot_id="robot-a")
        target = operations_for(
            database_b, local, tmp_path / "b", robot_id="robot-b", loader=loader, gate=gate
        )
        await origin.prepare()
        await target.prepare()
        exported = await origin.export_bundle(source.map_ref, bundle, operation_id="export-a")
        result = await target.import_bundle(bundle, "robot-b", operation_id="import-b")
        home = await target.home_for("robot-b")
        record = await target.operation_record("import-b")
        assert await target.task_links() == ()

    assert isinstance(exported, BundleManifest)
    assert exported.tasks_included is False
    assert result.status is OperationStatus.SUCCEEDED
    assert home is not None
    assert home.source_robot_id == "robot-a"
    assert home.status == "pending_confirmation"
    assert home.adopted is False
    assert home.waypoint_id == "home"
    assert home.map_ref == source.map_ref
    assert record.relocalized is False
    assert loader.hints == [Pose2D(1.0, 2.0, 0.25)]
    assert target.store.map_ref == source.map_ref
    assert gate.driving is False
    assert gate.blocked is True
    history = SlamSnapshot.read(tmp_path / "b" / "maps" / "history" / "local" / "1")
    assert history.occupancy.cells == local_cells
    for name, payload in config.items():
        assert (tmp_path / "b" / "config" / name).read_bytes() == payload


@pytest.mark.asyncio
async def test_same_robot_restore_checks_map_identity(tmp_path: Path) -> None:
    store = filled_store("yard", 3, (1.25, 0.25))
    original = store.map_ref
    cells = store.occupancy().cells
    bundle = tmp_path / "same.zip"
    async with Database(tmp_path / "robot.db") as database:
        ops = operations_for(database, store, tmp_path, robot_id="robot-a", loader=Loader())
        await ops.prepare()
        await ops.export_bundle(original, bundle, operation_id="export-same")
        tampered = tmp_path / "tampered.zip"
        rewrite_waypoint_map(bundle, tampered, "other-map")
        with pytest.raises(DomainError) as caught:
            await ops.import_bundle(tampered, "robot-a", operation_id="import-same")
        home = await ops.home_for("robot-a")

    assert caught.value.code is ErrorCode.VALIDATION
    assert ops.store.map_ref == original
    assert ops.store.occupancy().cells == cells
    assert home is None or home.adopted is False


@pytest.mark.asyncio
async def test_unknown_operation_after_disconnect_queryable(tmp_path: Path) -> None:
    store = filled_store("yard", 3, (1.25, 0.25))
    async with Database(tmp_path / "robot.db") as database:
        ops = operations_for(database, store, tmp_path, robot_id="robot-a")
        await ops.prepare()
        done = await ops.clear_map(store.map_ref, operation_id="keep-me")
        assert done.operation_id == "keep-me"
    fresh = filled_store("yard", 3, (1.25, 0.25))
    async with Database(tmp_path / "robot.db") as reopened:
        again = operations_for(reopened, fresh, tmp_path, robot_id="robot-a")
        await again.prepare()
        found = await again.query("keep-me")
        with pytest.raises(DomainError) as caught:
            await again.query("never-issued")

    assert found.operation_id == "keep-me"
    assert found.status is OperationStatus.SUCCEEDED
    assert caught.value.code is ErrorCode.OPERATION_UNKNOWN


@pytest.mark.asyncio
async def test_same_operation_different_payload_conflicts(tmp_path: Path) -> None:
    store = filled_store("yard", 3, (1.25, 0.25))
    gate = Gate()
    async with Database(tmp_path / "robot.db") as database:
        ops = operations_for(database, store, tmp_path, robot_id="robot-a", gate=gate)
        await ops.prepare()
        first = await ops.clear_map(store.map_ref, operation_id="once")
        cleared = ops.store.map_ref
        with pytest.raises(DomainError) as caught:
            await ops.clear_map(MapRef("yard", 9), operation_id="once")
        record = await ops.operation_record("once")

    assert caught.value.code is ErrorCode.CONFLICT
    assert first.status is OperationStatus.SUCCEEDED
    assert record.result.status is OperationStatus.SUCCEEDED
    assert ops.store.map_ref == cleared
    assert gate.calls == ["stop_and_block_navigation"]


@pytest.mark.asyncio
async def test_cancelled_io_outcome_unknown(tmp_path: Path) -> None:
    store = filled_store("yard", 3, (1.25, 0.25))
    original = store.map_ref
    barrier = Barrier()
    gate = Gate()
    async with Database(tmp_path / "robot.db") as database:
        ops = operations_for(
            database,
            store,
            tmp_path,
            robot_id="robot-a",
            checkpoint=barrier,
            gate=gate,
        )
        await ops.prepare()
        await ops.bind_task("patrol", "keep", original)
        task = asyncio.create_task(ops.clear_map(original, operation_id="cancel-1"))
        await asyncio.wait_for(barrier.entered.wait(), timeout=1)
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task
        record = await ops.operation_record("cancel-1")
        links = await ops.task_links()
        async with database.connection.execute(
            "SELECT status FROM map_operation WHERE operation_id = ?",
            ("cancel-1",),
        ) as cursor:
            stored = await cursor.fetchone()

    assert stored is not None
    assert stored[0] == "needs_review"
    assert record.result.status is OperationStatus.NEEDS_REVIEW
    assert record.rollback_confirmed is False
    assert record.relocalized is False
    assert ops.store.map_ref == original
    assert links[0].invalidated is False
    assert gate.driving is False


@pytest.mark.asyncio
async def test_crash_between_commit_and_dispatch_needs_review(tmp_path: Path) -> None:
    store = filled_store("yard", 3, (1.25, 0.25))
    original = store.map_ref
    cells = store.occupancy().cells
    gate = Gate()
    async with Database(tmp_path / "robot.db") as database:
        ops = operations_for(
            database,
            store,
            tmp_path,
            robot_id="robot-a",
            fault_after="accepted",
            gate=gate,
        )
        await ops.prepare()
        await ops.bind_task("patrol", "keep", original)
        with pytest.raises(ProcessCrash):
            await ops.clear_map(original, operation_id="crash-1")
    async with Database(tmp_path / "robot.db") as reopened:
        again = operations_for(reopened, store, tmp_path, robot_id="robot-a")
        await again.prepare()
        record = await again.operation_record("crash-1")
        phases = await again.phases("crash-1")
        links = await again.task_links()

    assert record.result.status is OperationStatus.NEEDS_REVIEW
    assert record.rollback_confirmed is False
    assert "accepted" in phases
    assert "dispatching" not in phases
    assert again.store.map_ref == original
    assert again.store.occupancy().cells == cells
    assert links[0].invalidated is False
    assert links[0].definition == "keep"
    assert gate.calls == []


class Session(Graph):
    def __init__(self, map_ref: MapRef) -> None:
        super().__init__(map_ref)
        self.token = "live"
        self.resets = 0
        self.reads: list[Path] = []
        self.rollback_during_restore: bool | None = None

    async def write(self, directory: Path) -> tuple[str, ...]:
        directory.mkdir(parents=True, exist_ok=True)
        (directory / "graph.posegraph").write_text(self.token, encoding="utf-8")
        (directory / "graph.data").write_text("scans", encoding="utf-8")
        return ("graph.posegraph", "graph.data")

    async def read(self, directory: Path) -> None:
        self.reads.append(directory)
        pose = directory / "graph.posegraph"
        if pose.is_file():
            self.token = pose.read_text(encoding="utf-8")

    async def reset(self) -> None:
        self.resets += 1
        self.token = "reset"


class HoldingGraph(Graph):
    def __init__(self, map_ref: MapRef) -> None:
        super().__init__(map_ref)
        self.started = asyncio.Event()
        self.release = asyncio.Event()

    async def write(self, directory: Path) -> tuple[str, ...]:
        self.started.set()
        await self.release.wait()
        return await super().write(directory)


def session_operations(
    database: Database,
    store: MapReferenceStore,
    root: Path,
    session: Graph,
    *,
    robot_id: str,
    loader: Loader | None = None,
    gate: Gate | None = None,
) -> MapOperations:
    return MapOperations(
        database,
        store,
        MapArchive(
            session,
            source_robot_id=robot_id,
            home_waypoint_id="home",
            sensor_summary=(("lidar", "source-description-only"),),
        ),
        gate or Gate(),
        robot_id=robot_id,
        map_root=root / "maps",
        config_dir=root / "config",
        loader=loader,
        checkpoint=None,
        fault_after=None,
    )


@pytest.mark.asyncio
async def test_import_keeps_pose_graph_and_stage_log(tmp_path: Path) -> None:
    source = filled_store("yard", 3, (1.25, 0.25))
    local = filled_store("local", 1, (2.25, 0.25))
    session = Session(source.map_ref)
    bundle = tmp_path / "yard.zip"
    async with (
        Database(tmp_path / "a.db") as database_a,
        Database(tmp_path / "b.db") as database_b,
    ):
        origin = session_operations(database_a, source, tmp_path / "a", session, robot_id="robot-a")
        target_session = Session(local.map_ref)
        target = session_operations(
            database_b,
            local,
            tmp_path / "b",
            target_session,
            robot_id="robot-b",
            loader=Loader(),
        )
        await origin.prepare()
        await target.prepare()
        await origin.export_bundle(source.map_ref, bundle, operation_id="export-graph")
        await target.import_bundle(bundle, "robot-b", operation_id="import-graph")
        phases = await target.phases("import-graph")

    history = tmp_path / "b" / "maps" / "history" / "local" / "1" / "pose_graph" / "graph.posegraph"
    active = tmp_path / "b" / "maps" / "active" / "pose_graph" / "graph.posegraph"
    assert history.is_file()
    assert history.read_text(encoding="utf-8") == "live"
    assert active.is_file()
    assert "history_durable" in phases
    assert "running" in phases
    assert "active_durable" in phases
    assert phases.index("history_durable") < phases.index("running")
    assert phases.index("running") < phases.index("active_durable")


@pytest.mark.asyncio
async def test_failed_load_restores_session_before_rollback_flag(tmp_path: Path) -> None:
    store = filled_store("yard", 3, (1.25, 0.25))
    session = Session(store.map_ref)
    gate = Gate()

    class DirtyLoader(Loader):
        async def load(self, directory: Path, hint: Pose2D | None) -> None:
            del directory, hint
            session.token = "dirty"
            raise RuntimeError("slam rejected the graph")

    bundle = tmp_path / "yard.zip"
    async with Database(tmp_path / "robot.db") as database:
        ops = session_operations(
            database, store, tmp_path, session, robot_id="robot-a", loader=DirtyLoader(), gate=gate
        )
        await ops.prepare()
        await ops.export_bundle(store.map_ref, bundle, operation_id="export-fail")

        async def watch_restore(directory: Path) -> None:
            del directory
            current = await ops.operation_record("import-fail")
            session.rollback_during_restore = current.rollback_confirmed

        session.read = watch_restore  # type: ignore[method-assign]
        original_read = Session.read

        async def reading(directory: Path) -> None:
            await watch_restore(directory)
            await original_read(session, directory)

        session.read = reading  # type: ignore[method-assign]
        with pytest.raises(RuntimeError, match="slam rejected"):
            await ops.import_bundle(bundle, "robot-a", operation_id="import-fail")
        record = await ops.operation_record("import-fail")

    assert session.reads
    assert session.token == "live"
    assert session.rollback_during_restore is False
    assert record.rollback_confirmed is True
    assert record.result.status is OperationStatus.FAILED
    assert gate.driving is False
    assert gate.blocked is True
    assert ops.store.map_ref.map_id == "yard"


@pytest.mark.asyncio
async def test_clear_resets_session_and_unadopts_home(tmp_path: Path) -> None:
    store = filled_store("yard", 3, (1.25, 0.25))
    session = Session(store.map_ref)
    bundle = tmp_path / "yard.zip"
    async with Database(tmp_path / "robot.db") as database:
        ops = session_operations(
            database, store, tmp_path, session, robot_id="robot-a", loader=Loader()
        )
        await ops.prepare()
        await ops.export_bundle(store.map_ref, bundle, operation_id="export-home")
        await ops.import_bundle(bundle, "robot-a", operation_id="import-home")
        adopted = await ops.home_for("robot-a")
        assert adopted is not None and adopted.adopted is True
        await ops.clear_map(store.map_ref, operation_id="clear-home")
        home = await ops.home_for("robot-a")
        phases = await ops.phases("clear-home")

    assert session.resets == 1
    history = tmp_path / "maps" / "history" / "yard" / "3" / "pose_graph" / "graph.posegraph"
    active = tmp_path / "maps" / "active" / "pose_graph" / "graph.posegraph"
    assert history.is_file()
    assert active.is_file()
    assert active.read_text(encoding="utf-8") == "reset"
    assert home is not None
    assert home.adopted is False
    assert home.waypoint_id == "home"
    assert home.map_ref == MapRef("yard", 3)
    assert home.source_robot_id == "robot-a"
    assert "running" in phases


@pytest.mark.asyncio
async def test_export_is_queryable_while_running(tmp_path: Path) -> None:
    store = filled_store("yard", 3, (1.25, 0.25))
    graph = HoldingGraph(store.map_ref)
    destination = tmp_path / "yard.zip"
    async with Database(tmp_path / "robot.db") as database:
        ops = session_operations(database, store, tmp_path, graph, robot_id="robot-a")
        await ops.prepare()
        task = asyncio.create_task(
            ops.export_bundle(store.map_ref, destination, operation_id="export-live")
        )
        await asyncio.wait_for(graph.started.wait(), timeout=1)
        found = await asyncio.wait_for(ops.query("export-live"), timeout=1)
        assert found.status is not OperationStatus.NEEDS_REVIEW
        assert found.status in {
            OperationStatus.ACCEPTED,
            OperationStatus.DISPATCHING,
            OperationStatus.RUNNING,
        }
        graph.release.set()
        manifest = await task
        done = await ops.query("export-live")

    assert isinstance(manifest, BundleManifest)
    assert done.status is OperationStatus.SUCCEEDED
    assert done.operation_id == "export-live"


@pytest.mark.asyncio
async def test_clear_mismatch_fails_before_stop(tmp_path: Path) -> None:
    store = filled_store("yard", 3, (1.25, 0.25))
    gate = Gate()
    async with Database(tmp_path / "robot.db") as database:
        ops = operations_for(database, store, tmp_path, robot_id="robot-a", gate=gate)
        await ops.prepare()
        with pytest.raises(DomainError) as caught:
            await ops.clear_map(MapRef("yard", 9), operation_id="bad-clear")
        record = await ops.operation_record("bad-clear")
        phases = await ops.phases("bad-clear")

    assert caught.value.code is ErrorCode.CONFLICT
    assert gate.calls == []
    assert gate.driving is True
    assert record.result.status is OperationStatus.FAILED
    assert "dispatching" not in phases
    assert ops.store.map_ref == MapRef("yard", 3)


@pytest.mark.asyncio
async def test_map_lock_blocks_clear_until_export_fsync(tmp_path: Path) -> None:
    store = filled_store("yard", 3, (1.25, 0.25))
    graph = HoldingGraph(store.map_ref)
    gate = Gate()
    destination = tmp_path / "yard.zip"
    async with Database(tmp_path / "robot.db") as database:
        ops = session_operations(database, store, tmp_path, graph, robot_id="robot-a", gate=gate)
        await ops.prepare()
        export_task = asyncio.create_task(
            ops.export_bundle(store.map_ref, destination, operation_id="export-lock")
        )
        await asyncio.wait_for(graph.started.wait(), timeout=1)
        clear_task = asyncio.create_task(ops.clear_map(store.map_ref, operation_id="clear-lock"))
        await asyncio.sleep(0.05)
        assert gate.calls == []
        assert clear_task.done() is False
        graph.release.set()
        await export_task
        await clear_task

    assert gate.calls == ["stop_and_block_navigation"]
