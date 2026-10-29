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
    AnchorPose,
    AnchorSnapshot,
    AnchoredWaypoint,
    GridExtent,
    SlamSnapshot,
)
from senior_control.mapping.store import MapReferenceStore
from senior_control.models import MapRef, OperationStatus, Pose2D
from senior_control.storage.database import Database

CELL = 0.5


def cell_of(x: float, y: float) -> tuple[int, int]:
    return (int(x // CELL), int(y // CELL))


class Graph:
    def __init__(self, map_ref: MapRef, hint: Pose2D | None = Pose2D(1.0, 2.0, 0.25)) -> None:
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
