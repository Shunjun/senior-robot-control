"""Durable map import, export, and clear survive disconnects and crashes."""

import asyncio
import hashlib
import json
import tempfile
import zipfile
from collections.abc import Callable
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


def synthetic_capture_proof(snapshot: SlamSnapshot) -> str:
    """Fake provenance for tests; real backends must prove graph-anchor identity."""
    with tempfile.TemporaryDirectory() as raw:
        snapshot.write(Path(raw))
        return json.dumps(json.loads((Path(raw) / "occupancy.json").read_text()), sort_keys=True)


class Graph:
    def __init__(self, map_ref: MapRef, hint: Pose2D | None = _HINT) -> None:
        self._map_ref = map_ref
        self._hint = hint
        self.current_graph = map_ref.map_id

    def map_ref(self) -> MapRef:
        return self._map_ref

    def last_pose_hint(self) -> Pose2D | None:
        return self._hint

    def painted_cells(self) -> tuple[tuple[int, int], ...]:
        return ((4, 1),)

    async def write(self, directory: Path) -> tuple[str, ...]:
        directory.mkdir(parents=True, exist_ok=True)
        (directory / "graph.posegraph").write_text(self.current_graph, encoding="utf-8")
        (directory / "graph.data").write_text("scans", encoding="utf-8")
        return ("graph.posegraph", "graph.data")

    async def capture_consistent(
        self, directory: Path, snapshot: Callable[[], SlamSnapshot]
    ) -> SlamSnapshot:
        # Synthetic single-loop backend: no graph or anchor update runs after write returns.
        await self.write(directory)
        captured = snapshot()
        (directory / "capture-proof.json").write_text(synthetic_capture_proof(captured))
        return captured

    async def verify_consistent(self, directory: Path, snapshot: SlamSnapshot) -> bool:
        proof = directory / "capture-proof.json"
        return proof.is_file() and json.loads(proof.read_text()) == json.loads(
            synthetic_capture_proof(snapshot)
        )

    async def read(self, directory: Path) -> None:
        self.current_graph = (directory / "graph.posegraph").read_text()

    async def reset(self) -> None:
        self.current_graph = "reset"

    async def matches(self, directory: Path, map_ref: MapRef) -> bool:
        expected = (directory / "graph.posegraph").read_text()
        identity_matches = (
            map_ref.map_id.startswith("unmapped-")
            if expected == "reset"
            else expected == map_ref.map_id
        )
        return self.current_graph == expected and identity_matches


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


def seed_active_for_test(root: Path, store: MapReferenceStore, graph: Graph) -> None:
    maps = root / "maps"
    if maps.exists():
        return
    active = maps / "active"
    snapshot = store.snapshot(Path("pose_graph"), ())
    snapshot.write(active)
    graph_dir = active / "pose_graph"
    graph_dir.mkdir()
    token = getattr(graph, "token", graph.current_graph)
    identity = getattr(graph, "graph_identity", "scans")
    (graph_dir / "graph.posegraph").write_text(token, encoding="utf-8")
    (graph_dir / "graph.data").write_text(identity, encoding="utf-8")
    (graph_dir / "capture-proof.json").write_text(synthetic_capture_proof(snapshot))


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
    archive = archive_for(store, robot_id)
    seed_active_for_test(root, store, archive._pose_graph)  # type: ignore[arg-type]
    return MapOperations(
        database,
        store,
        archive,
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
    assert gate.calls == ["stop_and_block_navigation"]
    assert gate.driving is False


@pytest.mark.asyncio
async def test_failed_load_is_unknown_and_stays_stopped(tmp_path: Path) -> None:
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

    assert record.result.status is OperationStatus.NEEDS_REVIEW
    assert record.relocalized is False
    assert record.rollback_confirmed is False
    assert ops.store.map_ref == original
    assert ops.store.occupancy().cells == cells
    assert ops.store.review_required
    assert gate.calls == ["stop_and_block_navigation", "stop_and_block_navigation"]
    assert gate.driving is False
    assert gate.blocked is True
    assert SlamSnapshot.read(tmp_path / "maps" / "active").map_ref == original


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
    assert gate.calls == ["stop_and_block_navigation", "stop_and_block_navigation"]
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
    assert gate.calls == ["stop_and_block_navigation", "stop_and_block_navigation"]


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
    assert gate.calls == ["stop_and_block_navigation"]


class Session(Graph):
    def __init__(self, map_ref: MapRef) -> None:
        super().__init__(map_ref)
        self.token = "live"
        self.graph_identity = map_ref.map_id
        self.resets = 0
        self.reads: list[Path] = []
        self.rollback_during_restore: bool | None = None

    async def write(self, directory: Path) -> tuple[str, ...]:
        directory.mkdir(parents=True, exist_ok=True)
        (directory / "graph.posegraph").write_text(self.token, encoding="utf-8")
        (directory / "graph.data").write_text(self.graph_identity, encoding="utf-8")
        return ("graph.posegraph", "graph.data")

    async def read(self, directory: Path) -> None:
        self.reads.append(directory)
        pose = directory / "graph.posegraph"
        if pose.is_file():
            self.token = pose.read_text(encoding="utf-8")
            data = (directory / "graph.data").read_text(encoding="utf-8")
            self.graph_identity = self.token if data == "scans" else data

    async def reset(self) -> None:
        self.resets += 1
        self.token = "reset"
        self.graph_identity = "reset"

    async def matches(self, directory: Path, map_ref: MapRef) -> bool:
        expected = (directory / "graph.posegraph").read_text()
        data = (directory / "graph.data").read_text()
        identity = expected if data == "scans" else data
        identity_matches = (
            map_ref.map_id.startswith("unmapped-")
            if identity == "reset"
            else identity == map_ref.map_id
        )
        return self.token == expected and self.graph_identity == identity and identity_matches


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
    seed_active_for_test(root, store, session)
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
async def test_failed_load_does_not_claim_unconfirmed_rollback(tmp_path: Path) -> None:
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
    assert session.token == "dirty"
    assert session.rollback_during_restore is False
    assert record.rollback_confirmed is False
    assert record.result.status is OperationStatus.NEEDS_REVIEW
    assert store.review_required
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
    assert gate.calls == ["stop_and_block_navigation"]
    assert gate.driving is False
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
        assert gate.calls == ["stop_and_block_navigation"]
        assert clear_task.done() is False
        graph.release.set()
        await export_task
        await clear_task

    assert gate.calls == ["stop_and_block_navigation", "stop_and_block_navigation"]


class BrokenPublish(MapOperations):
    async def _publish_active(self, snapshot: SlamSnapshot, pose_graph: Path | None) -> None:
        del snapshot, pose_graph
        raise RuntimeError("active publish failed")


def _operations(
    kind: type[MapOperations],
    database: Database,
    store: MapReferenceStore,
    root: Path,
    *,
    robot_id: str,
    loader: Loader | None = None,
    session: Graph | None = None,
) -> MapOperations:
    graph = session or Graph(store.map_ref)
    seed_active_for_test(root, store, graph)
    return kind(
        database,
        store,
        MapArchive(
            graph,
            source_robot_id=robot_id,
            home_waypoint_id="home",
            sensor_summary=(("lidar", "source-description-only"),),
        ),
        Gate(),
        robot_id=robot_id,
        map_root=root / "maps",
        config_dir=root / "config",
        loader=loader,
    )


@pytest.mark.asyncio
async def test_export_conflict_after_running_is_failed(tmp_path: Path) -> None:
    store = filled_store("yard", 3, (1.25, 0.25))
    destination = tmp_path / "yard.zip"
    async with Database(tmp_path / "robot.db") as database:
        ops = operations_for(database, store, tmp_path, robot_id="robot-a")
        await ops.prepare()
        await ops.export_bundle(store.map_ref, destination, operation_id="export-ok")
        with pytest.raises(DomainError) as caught:
            await ops.export_bundle(store.map_ref, destination, operation_id="export-dup")
        replay = await ops.export_bundle(store.map_ref, destination, operation_id="export-dup")
        record = await ops.operation_record("export-dup")
        queried = await ops.query("export-dup")

    assert caught.value.code is ErrorCode.CONFLICT
    assert isinstance(replay, BundleManifest)
    assert record.result.status is OperationStatus.FAILED
    assert queried.status is OperationStatus.FAILED
    assert record.result.status is not OperationStatus.NEEDS_REVIEW


@pytest.mark.asyncio
async def test_export_cancel_after_running_needs_review(tmp_path: Path) -> None:
    store = filled_store("yard", 3, (1.25, 0.25))
    graph = HoldingGraph(store.map_ref)
    destination = tmp_path / "yard.zip"
    async with Database(tmp_path / "robot.db") as database:
        ops = session_operations(database, store, tmp_path, graph, robot_id="robot-a")
        await ops.prepare()
        task = asyncio.create_task(
            ops.export_bundle(store.map_ref, destination, operation_id="export-cancel")
        )
        await asyncio.wait_for(graph.started.wait(), timeout=1)
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task
        record = await ops.operation_record("export-cancel")
        queried = await ops.query("export-cancel")

    assert record.result.status is OperationStatus.NEEDS_REVIEW
    assert queried.status is OperationStatus.NEEDS_REVIEW
    assert record.rollback_confirmed is False


@pytest.mark.asyncio
async def test_import_replace_after_identity_change_needs_review(tmp_path: Path) -> None:
    store = filled_store("local", 1, (2.25, 0.25))
    source = filled_store("yard", 3, (1.25, 0.25))
    bundle = tmp_path / "yard.zip"
    session = Session(store.map_ref)
    calls = {"n": 0}
    real_replace = store.replace

    def replace_then_fail(snapshot: SlamSnapshot) -> None:
        calls["n"] += 1
        real_replace(snapshot)
        if calls["n"] == 1:
            raise RuntimeError("replace failed after identity")

    async with Database(tmp_path / "robot.db") as database:
        ops = session_operations(
            database, store, tmp_path, session, robot_id="robot-a", loader=Loader()
        )
        await ops.prepare()
        store.replace = replace_then_fail  # type: ignore[method-assign]
        async with Database(tmp_path / "source.db") as source_db:
            origin = operations_for(source_db, source, tmp_path / "source", robot_id="robot-a")
            await origin.prepare()
            await origin.export_bundle(source.map_ref, bundle, operation_id="export-replace")
        with pytest.raises(RuntimeError, match="replace failed after identity"):
            await ops.import_bundle(bundle, "robot-a", operation_id="import-replace")
        record = await ops.operation_record("import-replace")
        queried = await ops.query("import-replace")

    assert store.map_ref == source.map_ref
    assert store.review_required
    assert session.token == "yard"
    assert record.result.status is OperationStatus.NEEDS_REVIEW
    assert queried.status is OperationStatus.NEEDS_REVIEW
    assert record.result.status is not OperationStatus.SUCCEEDED


@pytest.mark.asyncio
async def test_import_publish_failure_after_running_needs_review(tmp_path: Path) -> None:
    store = filled_store("local", 1, (2.25, 0.25))
    source = filled_store("yard", 3, (1.25, 0.25))
    bundle = tmp_path / "yard.zip"
    session = Session(store.map_ref)
    async with Database(tmp_path / "robot.db") as database:
        ops = _operations(
            BrokenPublish,
            database,
            store,
            tmp_path,
            robot_id="robot-a",
            loader=Loader(),
            session=session,
        )
        await ops.prepare()
        async with Database(tmp_path / "source.db") as source_db:
            origin = operations_for(source_db, source, tmp_path / "source", robot_id="robot-a")
            await origin.prepare()
            await origin.export_bundle(source.map_ref, bundle, operation_id="export-publish")
        with pytest.raises(RuntimeError, match="active publish failed"):
            await ops.import_bundle(bundle, "robot-a", operation_id="import-publish")
        record = await ops.operation_record("import-publish")
        queried = await ops.query("import-publish")

    assert store.map_ref == source.map_ref
    assert store.review_required
    assert session.token == "yard"
    assert record.result.status is OperationStatus.NEEDS_REVIEW
    assert queried.status is OperationStatus.NEEDS_REVIEW


@pytest.mark.asyncio
async def test_import_cancel_during_load_needs_review(tmp_path: Path) -> None:
    store = filled_store("local", 1, (2.25, 0.25))
    source = filled_store("yard", 3, (1.25, 0.25))
    bundle = tmp_path / "yard.zip"

    class HoldLoader(Loader):
        def __init__(self) -> None:
            super().__init__()
            self.started = asyncio.Event()
            self.release = asyncio.Event()

        async def load(self, directory: Path, hint: Pose2D | None) -> None:
            del directory, hint
            self.started.set()
            await self.release.wait()

    loader = HoldLoader()
    async with Database(tmp_path / "robot.db") as database:
        ops = session_operations(
            database,
            store,
            tmp_path,
            Session(store.map_ref),
            robot_id="robot-a",
            loader=loader,
        )
        await ops.prepare()
        async with Database(tmp_path / "source.db") as source_db:
            origin = operations_for(source_db, source, tmp_path / "source", robot_id="robot-a")
            await origin.prepare()
            await origin.export_bundle(source.map_ref, bundle, operation_id="export-load-cancel")
        task = asyncio.create_task(
            ops.import_bundle(bundle, "robot-a", operation_id="import-load-cancel")
        )
        await asyncio.wait_for(loader.started.wait(), timeout=1)
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task
        record = await ops.operation_record("import-load-cancel")
        queried = await ops.query("import-load-cancel")

    assert record.result.status is OperationStatus.NEEDS_REVIEW
    assert queried.status is OperationStatus.NEEDS_REVIEW
    assert record.rollback_confirmed is False


@pytest.mark.asyncio
async def test_clear_reset_failure_is_unknown(tmp_path: Path) -> None:
    store = filled_store("yard", 3, (1.25, 0.25))
    original = store.map_ref

    class BoomReset(Session):
        async def reset(self) -> None:
            raise RuntimeError("reset failed")

    async with Database(tmp_path / "robot.db") as database:
        ops = session_operations(
            database, store, tmp_path, BoomReset(store.map_ref), robot_id="robot-a"
        )
        await ops.prepare()
        with pytest.raises(RuntimeError, match="reset failed"):
            await ops.clear_map(original, operation_id="clear-reset")
        record = await ops.operation_record("clear-reset")
        queried = await ops.query("clear-reset")

    assert store.map_ref == original
    assert store.review_required
    assert record.result.status is OperationStatus.NEEDS_REVIEW
    assert queried.status is OperationStatus.NEEDS_REVIEW


@pytest.mark.asyncio
async def test_clear_publish_failure_after_identity_change_needs_review(tmp_path: Path) -> None:
    store = filled_store("yard", 3, (1.25, 0.25))
    original = store.map_ref
    session = Session(store.map_ref)
    async with Database(tmp_path / "robot.db") as database:
        ops = _operations(
            BrokenPublish,
            database,
            store,
            tmp_path,
            robot_id="robot-a",
            session=session,
        )
        await ops.prepare()
        with pytest.raises(RuntimeError, match="active publish failed"):
            await ops.clear_map(original, operation_id="clear-publish")
        record = await ops.operation_record("clear-publish")
        queried = await ops.query("clear-publish")

    assert store.map_ref != original
    assert store.review_required
    assert session.token == "reset"
    assert record.result.status is OperationStatus.NEEDS_REVIEW
    assert queried.status is OperationStatus.NEEDS_REVIEW
    assert record.result.status is not OperationStatus.SUCCEEDED


@pytest.mark.asyncio
async def test_clear_cancel_after_running_needs_review(tmp_path: Path) -> None:
    store = filled_store("yard", 3, (1.25, 0.25))

    class HoldReset(Session):
        def __init__(self, map_ref: MapRef) -> None:
            super().__init__(map_ref)
            self.started = asyncio.Event()
            self.release = asyncio.Event()

        async def reset(self) -> None:
            self.started.set()
            await self.release.wait()
            await super().reset()

    session = HoldReset(store.map_ref)
    async with Database(tmp_path / "robot.db") as database:
        ops = session_operations(database, store, tmp_path, session, robot_id="robot-a")
        await ops.prepare()
        task = asyncio.create_task(ops.clear_map(store.map_ref, operation_id="clear-cancel"))
        await asyncio.wait_for(session.started.wait(), timeout=1)
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task
        record = await ops.operation_record("clear-cancel")
        queried = await ops.query("clear-cancel")

    assert record.result.status is OperationStatus.NEEDS_REVIEW
    assert queried.status is OperationStatus.NEEDS_REVIEW
    assert record.rollback_confirmed is False


async def test_import_operation_id_binds_complete_bundle_content(tmp_path: Path) -> None:
    first_source = filled_store("same", 1, (1.0, 1.0))
    second_source = filled_store("same", 1, (3.0, 3.0))
    local = filled_store("local", 1, (4.0, 4.0))
    first_path, second_path = tmp_path / "first.zip", tmp_path / "second.zip"
    await archive_for(first_source, "origin").export_bundle(
        first_source, first_source.map_ref, first_path
    )
    await archive_for(second_source, "origin").export_bundle(
        second_source, second_source.map_ref, second_path
    )
    async with Database(tmp_path / "robot.db") as database:
        ops = operations_for(database, local, tmp_path, robot_id="target")
        await ops.prepare()
        first = await ops.import_bundle(first_path, "target", operation_id="same-id")
        retry = await ops.import_bundle(first_path, "target", operation_id="same-id")
        with pytest.raises(DomainError) as error:
            await ops.import_bundle(second_path, "target", operation_id="same-id")
        assert error.value.code is ErrorCode.CONFLICT
        assert retry == first
        assert ops.store.occupancy().cells == first_source.occupancy().cells


@pytest.mark.parametrize("absolute", [False, True])
async def test_opaque_map_id_cannot_escape_history_on_clear_and_reimport(
    tmp_path: Path, absolute: bool
) -> None:
    escaped = tmp_path / "outside" if absolute else tmp_path / "owned" / "outside"
    marker = escaped / "1" / "keep"
    marker.parent.mkdir(parents=True)
    marker.write_text("unchanged")
    identity = str(escaped) if absolute else "../../outside"
    source = filled_store(identity, 1, (1.0, 1.0))
    local = filled_store("local", 1, (2.0, 2.0))
    bundle = tmp_path / "opaque.zip"
    await archive_for(source, "source").export_bundle(source, source.map_ref, bundle)
    async with Database(tmp_path / "robot.db") as database:
        ops = operations_for(database, local, tmp_path / "owned", robot_id="target")
        await ops.prepare()
        await ops.import_bundle(bundle, "target", operation_id="first")
        await ops.clear_map(source.map_ref, operation_id="clear")
        await ops.import_bundle(bundle, "target", operation_id="again")
    assert marker.read_text() == "unchanged"
    assert list(marker.parent.iterdir()) == [marker]


@pytest.mark.parametrize("part", ["history", "history/local", ".staging", "active"])
async def test_map_symlink_parent_cannot_modify_outside_tree(tmp_path: Path, part: str) -> None:
    outside = tmp_path / "outside"
    outside.mkdir()
    marker = outside / "keep"
    marker.write_text("unchanged")
    root = tmp_path / "owned"
    maps = root / "maps"
    maps.mkdir(parents=True)
    (maps / part).parent.mkdir(parents=True, exist_ok=True)
    (maps / part).symlink_to(outside, target_is_directory=True)
    store = filled_store("local", 1, (2.0, 2.0))
    async with Database(tmp_path / "robot.db") as database:
        ops = operations_for(database, store, root, robot_id="target")
        with pytest.raises(DomainError):
            await ops.prepare()
            await ops.clear_map(store.map_ref, operation_id="clear")
    assert marker.read_text() == "unchanged"
    assert list(outside.iterdir()) == [marker]


@pytest.mark.parametrize("crash_after_install", [False, True])
async def test_prepare_recovers_active_generation_after_rename_crash(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, crash_after_install: bool
) -> None:
    import os

    from senior_control.mapping.references import durable_move_tree

    old = filled_store("old", 1, (1.0, 1.0))
    new = filled_store("new", 2, (4.0, 4.0))
    root = tmp_path / "maps"
    active = root / "active"
    staging = root / ".staging" / "incoming"
    active.mkdir(parents=True)
    (active / "pose_graph").mkdir()
    (active / "pose_graph" / "graph.posegraph").write_text("old graph")
    old.snapshot(Path("pose_graph"), ()).write(active)
    staging.mkdir(parents=True)
    (staging / "pose_graph").mkdir()
    (staging / "pose_graph" / "graph.posegraph").write_text("new graph")
    new.snapshot(Path("pose_graph"), ()).write(staging)
    real_replace = os.replace

    def crash_between_renames(source: object, destination: object) -> None:
        if Path(source) == staging and Path(destination) == active:
            if crash_after_install:
                real_replace(source, destination)
            raise ProcessCrash("rename gap")
        real_replace(source, destination)

    with monkeypatch.context() as patch:
        patch.setattr(os, "replace", crash_between_renames)
        with pytest.raises(ProcessCrash):
            durable_move_tree(staging, active)
    async with Database(tmp_path / "robot.db") as database:
        ops = operations_for(database, old, tmp_path, robot_id="target")
        await ops.prepare()
    recovered = SlamSnapshot.read(active)
    expected = new if crash_after_install else old
    assert recovered.map_ref == expected.map_ref
    assert recovered.occupancy.cells == expected.occupancy().cells
    assert (active / "pose_graph" / "graph.posegraph").read_text() == (
        "new graph" if crash_after_install else "old graph"
    )


@pytest.mark.parametrize("action", ["clear", "import"])
async def test_cancelled_graph_switch_waits_for_fence_before_recovery(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, action: str
) -> None:
    import senior_control.mapping.operations as operations_module

    class AfterEffect(Session):
        def __init__(self, map_ref: MapRef) -> None:
            super().__init__(map_ref)
            self.entered = asyncio.Event()
            self.release = asyncio.Event()
            self.finished = asyncio.Event()

        async def reset(self) -> None:
            self.token = "reset"
            self.entered.set()
            await self.release.wait()
            self.finished.set()

        async def read(self, directory: Path) -> None:
            payload = (directory / "graph.posegraph").read_text()
            self.token = payload
            if payload != "live":
                self.entered.set()
                await self.release.wait()
                self.finished.set()

        async def quiesce(self) -> None:
            await self.finished.wait()

        async def matches(self, directory: Path, map_ref: MapRef) -> bool:
            return (
                map_ref == self._map_ref
                and self.token == (directory / "graph.posegraph").read_text()
            )

    monkeypatch.setattr(operations_module, "_GRAPH_WAIT_SECONDS", 0.02, raising=False)
    local = filled_store("local", 1, (2.0, 2.0))
    source = filled_store("remote", 2, (1.0, 1.0))
    bundle = tmp_path / "source.zip"
    await archive_for(source, "source").export_bundle(source, source.map_ref, bundle)
    graph = AfterEffect(local.map_ref)
    path = tmp_path / "robot.db"
    async with Database(path) as database:
        gate = Gate()
        ops = session_operations(database, local, tmp_path, graph, robot_id="r", gate=gate)
        await ops.prepare()
        if action == "clear":
            operation = ops.clear_map(local.map_ref, operation_id="switch")
        else:
            operation = ops.import_bundle(bundle, "r", operation_id="switch")
        running = asyncio.create_task(operation)
        await asyncio.wait_for(graph.entered.wait(), 1)
        running.cancel()
        with pytest.raises(asyncio.CancelledError):
            await asyncio.wait_for(running, 1)
        assert gate.blocked
        assert local.review_required
        assert (await ops.query("switch")).status is OperationStatus.NEEDS_REVIEW
        assert graph.token != "live"
        with pytest.raises(DomainError):
            await ops.clear_map(local.map_ref, operation_id="blocked")
    local = filled_store("local", 1, (2.0, 2.0))
    async with Database(path) as database:
        restarted_gate = Gate()
        reopened = session_operations(
            database, local, tmp_path, graph, robot_id="r", gate=restarted_gate
        )
        await reopened.prepare()
        assert restarted_gate.blocked
        with pytest.raises(DomainError):
            await reopened.reconcile()
        assert graph.token != "live"
        graph.release.set()
        await asyncio.wait_for(graph.finished.wait(), 1)
        await reopened.reconcile()
        assert graph.token == "live"
        assert local.map_ref == MapRef("local", 1)
        assert not local.review_required
        active = SlamSnapshot.read(tmp_path / "maps" / "active")
        assert active.map_ref == local.map_ref
        assert active.occupancy.cells == local.occupancy().cells
        assert (
            tmp_path / "maps" / "active" / "pose_graph" / "graph.posegraph"
        ).read_text() == "live"


@pytest.mark.parametrize("action", ["clear", "import"])
async def test_switch_without_backend_identity_confirmation_never_reports_success(
    tmp_path: Path, action: str
) -> None:
    local = filled_store("local", 1, (2.0, 2.0))
    source = filled_store("source", 2, (1.0, 1.0))
    bundle = tmp_path / "source.zip"
    await archive_for(source, "source").export_bundle(source, source.map_ref, bundle)

    graph = Session(local.map_ref)
    gate = Gate()
    async with Database(tmp_path / "robot.db") as database:
        ops = session_operations(database, local, tmp_path, graph, robot_id="r", gate=gate)
        await ops.prepare()
        graph.matches = None  # type: ignore[method-assign,assignment]
        with pytest.raises(DomainError) as error:
            if action == "clear":
                await ops.clear_map(local.map_ref, operation_id="unconfirmed")
            else:
                await ops.import_bundle(bundle, "r", operation_id="unconfirmed")
        assert error.value.code is ErrorCode.UNAVAILABLE
        assert (await ops.query("unconfirmed")).status is OperationStatus.NEEDS_REVIEW
        assert local.review_required
        assert gate.blocked
        if action == "clear":
            retry = await ops.clear_map(local.map_ref, operation_id="unconfirmed")
        else:
            retry = await ops.import_bundle(bundle, "r", operation_id="unconfirmed")
        assert retry.status is OperationStatus.NEEDS_REVIEW


async def test_prepare_requires_live_backend_match_for_existing_active_map(tmp_path: Path) -> None:
    store = filled_store("yard", 3, (1.0, 1.0))
    active = tmp_path / "maps" / "active"
    store.snapshot(Path("pose_graph"), ()).write(active)
    (active / "pose_graph").mkdir()
    (active / "pose_graph" / "graph.posegraph").write_text("yard")
    (active / "pose_graph" / "graph.data").write_text("scans")
    snapshot = store.snapshot(Path("pose_graph"), ())
    (active / "pose_graph" / "capture-proof.json").write_text(synthetic_capture_proof(snapshot))
    graph = Graph(store.map_ref)
    graph.current_graph = "different"
    gate = Gate()
    async with Database(tmp_path / "robot.db") as database:
        ops = session_operations(database, store, tmp_path, graph, robot_id="r", gate=gate)
        await ops.prepare()
        assert gate.blocked
        assert store.review_required
        with pytest.raises(DomainError):
            await ops.clear_map(store.map_ref, operation_id="must-not-clear")


async def test_failed_backend_identity_confirmation_remains_blocked_after_restart(
    tmp_path: Path,
) -> None:
    class Mismatch(Session):
        reject = False

        async def matches(self, directory: Path, map_ref: MapRef) -> bool:
            return False if self.reject else await super().matches(directory, map_ref)

        async def quiesce(self) -> None:
            return None

    store = filled_store("yard", 3, (1.0, 1.0))
    graph = Mismatch(store.map_ref)
    async with Database(tmp_path / "robot.db") as database:
        ops = session_operations(database, store, tmp_path, graph, robot_id="r")
        await ops.prepare()
        graph.reject = True
        with pytest.raises(DomainError) as error:
            await ops.clear_map(store.map_ref, operation_id="mismatch")
        assert error.value.code is ErrorCode.OPERATION_UNKNOWN
        gate = Gate()
        restarted = session_operations(database, store, tmp_path, graph, robot_id="r", gate=gate)
        await restarted.prepare()
        with pytest.raises(DomainError) as error:
            await restarted.reconcile()
        assert error.value.code is ErrorCode.OPERATION_UNKNOWN
        assert store.review_required
        assert gate.blocked
        assert (await restarted.query("mismatch")).status is OperationStatus.NEEDS_REVIEW


@pytest.mark.parametrize("action", ["clear", "import"])
async def test_guard_delete_failure_does_not_commit_success(tmp_path: Path, action: str) -> None:
    import sqlite3

    local = filled_store("local", 1, (2.0, 2.0))
    source = filled_store("remote", 2, (1.0, 1.0))
    bundle = tmp_path / "source.zip"
    await archive_for(source, "source").export_bundle(source, source.map_ref, bundle)
    async with Database(tmp_path / "robot.db") as database:
        ops = operations_for(database, local, tmp_path, robot_id="r")
        await ops.prepare()
        await database.connection.execute(
            "CREATE TRIGGER prevent_graph_guard_delete BEFORE DELETE ON map_graph_guard "
            "BEGIN SELECT RAISE(ABORT, 'guard deletion failed'); END"
        )
        await database.connection.commit()
        with pytest.raises(sqlite3.IntegrityError):
            if action == "clear":
                await ops.clear_map(local.map_ref, operation_id="commit-fail")
            else:
                await ops.import_bundle(bundle, "r", operation_id="commit-fail")
        record = await ops.operation_record("commit-fail")
        assert record.result.status is OperationStatus.NEEDS_REVIEW
        assert record.rollback_confirmed is False
        assert local.review_required


async def test_startup_without_active_requires_verified_backend(tmp_path: Path) -> None:
    class Unverified(Session):
        matches = None  # type: ignore[assignment]

    store = filled_store("yard", 3, (1.0, 1.0))
    gate = Gate()
    async with Database(tmp_path / "robot.db") as database:
        ops = MapOperations(
            database,
            store,
            MapArchive(Unverified(store.map_ref), source_robot_id="r"),
            gate,
            robot_id="r",
            map_root=tmp_path / "maps",
            config_dir=tmp_path / "config",
        )
        await ops.prepare()
        assert gate.blocked
        assert store.review_required
        with pytest.raises(DomainError):
            await ops.clear_map(store.map_ref, operation_id="blocked-at-startup")
    assert not (tmp_path / "maps" / "active").exists()


async def test_verified_bootstrap_can_reconcile_initial_missing_snapshot(tmp_path: Path) -> None:
    class BootstrapGraph(Session):
        async def quiesce(self) -> None:
            return None

    store = filled_store("yard", 3, (1.0, 1.0))
    snapshot = store.snapshot(Path("pose_graph"), ())
    graph = BootstrapGraph(store.map_ref)
    gate = Gate()
    async with Database(tmp_path / "robot.db") as database:
        ops = MapOperations(
            database,
            store,
            MapArchive(graph, source_robot_id="r"),
            gate,
            robot_id="r",
            map_root=tmp_path / "maps",
            config_dir=tmp_path / "config",
        )
        await ops.prepare()
        with pytest.raises(DomainError) as error:
            await ops.reconcile()
        assert error.value.code is ErrorCode.UNAVAILABLE
        assert gate.blocked and store.review_required
        active = tmp_path / "maps" / "active"
        snapshot.write(active)
        await graph.capture_consistent(active / "pose_graph", lambda: snapshot)
        await ops.reconcile()
        assert not store.review_required
        assert store.map_ref == snapshot.map_ref
        assert gate.blocked


async def test_startup_blocks_motion_before_waiting_for_database(tmp_path: Path) -> None:
    store = filled_store("yard", 3, (1.0, 1.0))
    gate = Gate()
    async with Database(tmp_path / "robot.db") as database:
        ops = MapOperations(
            database,
            store,
            MapArchive(Graph(store.map_ref), source_robot_id="r"),
            gate,
            robot_id="r",
            map_root=tmp_path / "maps",
            config_dir=tmp_path / "config",
        )
        await database.write_lock.acquire()
        prepare = asyncio.create_task(ops.prepare())
        try:
            for _ in range(20):
                if gate.blocked:
                    break
                await asyncio.sleep(0)
            assert gate.blocked
            assert store.review_required
        finally:
            database.write_lock.release()
            await prepare


async def test_backup_rejects_graph_and_occupancy_change_during_capture(tmp_path: Path) -> None:
    class ChangingGraph(Session):
        def __init__(self, map_ref: MapRef) -> None:
            super().__init__(map_ref)
            self.entered = asyncio.Event()
            self.release = asyncio.Event()

        async def write(self, directory: Path) -> tuple[str, ...]:
            self.entered.set()
            await self.release.wait()
            return await super().write(directory)

        async def capture_consistent(self, directory: Path, capture):
            await self.write(directory)
            return capture()

        async def reset(self) -> None:
            self.resets += 1
            self.token = "reset"
            raise RuntimeError("changed graph")

        async def quiesce(self) -> None:
            return None

    store = filled_store("yard", 3, (2.0, 2.0))
    graph = ChangingGraph(store.map_ref)
    async with Database(tmp_path / "robot.db") as database:
        ops = session_operations(database, store, tmp_path, graph, robot_id="r")
        await ops.prepare()
        task = asyncio.create_task(ops.clear_map(store.map_ref, operation_id="race"))
        await asyncio.wait_for(graph.entered.wait(), 1)
        graph.token = "live-generation-2"
        store.apply_anchor_update(
            AnchorSnapshot(
                store.map_ref,
                "wheel_odom",
                (AnchorPose("kf-1", Pose2D(1, 0, 0), True),),
                GridExtent(8, 8, 0, 0),
            )
        )
        graph.release.set()
        try:
            await task
        except (DomainError, RuntimeError):
            pass
        assert graph.resets == 0
        assert store.review_required
        assert await ops._graph_guard() is not None
        assert (await ops.query("race")).status is OperationStatus.NEEDS_REVIEW


async def test_backup_without_consistent_capture_stays_unavailable(tmp_path: Path) -> None:
    class NoCapture(Session):
        capture_consistent = None

    store = filled_store("yard", 3, (2.0, 2.0))
    graph = NoCapture(store.map_ref)
    gate = Gate()
    async with Database(tmp_path / "robot.db") as database:
        ops = session_operations(database, store, tmp_path, graph, robot_id="r", gate=gate)
        await ops.prepare()
        with pytest.raises(DomainError) as error:
            await ops.clear_map(store.map_ref, operation_id="unsupported-capture")
        assert error.value.code is ErrorCode.UNAVAILABLE
        assert graph.resets == 0
        assert store.review_required and gate.blocked
        assert await ops._graph_guard() is not None
        assert (await ops.query("unsupported-capture")).status is OperationStatus.NEEDS_REVIEW


async def test_failed_reconcile_guard_deletion_keeps_reference_in_review(tmp_path: Path) -> None:
    import sqlite3

    from senior_control.waypoints import HomeService, LocalizedPose

    class Nav:
        def __init__(self) -> None:
            self.goals: list[object] = []

        async def navigate_qualified(self, goal: object) -> str:
            self.goals.append(goal)
            return "goal"

        async def cancel(self, execution_id: str):
            from senior_control.models import OperationResult

            return OperationResult(execution_id, OperationStatus.SUCCEEDED)

    class Recoverable(Session):
        async def quiesce(self) -> None:
            return None

    store = filled_store("yard", 3, (2.0, 2.0))
    graph = Recoverable(store.map_ref)
    gate = Gate()
    async with Database(tmp_path / "robot.db") as database:
        ops = session_operations(database, store, tmp_path, graph, robot_id="r", gate=gate)
        await ops.prepare()
        navigation = Nav()
        home = HomeService(database, store, robot_id="r", navigation=navigation)
        await home.prepare()
        await home.set_current(
            LocalizedPose(
                Pose2D(2, 2, 0),
                store.map_ref,
                "kf-1",
                Pose2D(0, 0, 0),
                True,
                map_generation=store.map_generation,
            )
        )
        await ops._set_graph_guard("unknown", tmp_path / "maps" / "active")
        await database.connection.execute(
            "CREATE TRIGGER reject_guard_deletion BEFORE DELETE ON map_graph_guard "
            "BEGIN SELECT RAISE(ABORT, 'cannot delete'); END"
        )
        await database.connection.commit()
        with pytest.raises(sqlite3.IntegrityError):
            await ops.reconcile()
        assert await ops._graph_guard() is not None
        assert store.review_required
        assert gate.blocked
        await home.note_localization(
            LocalizedPose(
                Pose2D(2, 2, 0),
                store.map_ref,
                "kf-1",
                Pose2D(0, 0, 0),
                True,
                map_generation=store.map_generation,
            )
        )
        with pytest.raises(DomainError):
            await home.return_home(store.map_ref)
        assert navigation.goals == []


async def test_same_reference_import_invalidates_old_home_localization(tmp_path: Path) -> None:
    from senior_control.waypoints import HomeService, LocalizedPose, StartupReference

    class Nav:
        def __init__(self) -> None:
            self.goals: list[object] = []

        async def navigate_qualified(self, goal: object) -> str:
            self.goals.append(goal)
            return "goal"

        async def cancel(self, execution_id: str):
            from senior_control.models import OperationResult

            return OperationResult(execution_id, OperationStatus.SUCCEEDED)

    old = filled_store("yard", 3, (2.0, 2.0))
    source = filled_store("yard", 3, (8.0, 8.0))
    bundle = tmp_path / "same-ref.zip"
    await archive_for(source, "r").export_bundle(source, source.map_ref, bundle)
    graph, gate, nav = Session(old.map_ref), Gate(), Nav()
    async with Database(tmp_path / "robot.db") as database:
        ops = session_operations(database, old, tmp_path, graph, robot_id="r", gate=gate)
        await ops.prepare()
        home = HomeService(database, old, robot_id="r", navigation=nav)
        await home.prepare()
        pose = LocalizedPose(Pose2D(2, 2, 0), old.map_ref, "kf-1", Pose2D(0, 0, 0), True)
        object.__setattr__(pose, "map_generation", old.map_generation)
        await home.initialize(StartupReference(False, True, True), pose)
        await ops.import_bundle(bundle, "r", operation_id="same-ref")
        with pytest.raises(DomainError) as error:
            await home.return_home(old.map_ref)
        assert error.value.code is ErrorCode.LOCALIZATION_UNAVAILABLE
        assert nav.goals == []
        fresh = LocalizedPose(
            Pose2D(8, 8, 0),
            old.map_ref,
            "kf-1",
            Pose2D(0, 0, 0),
            True,
            map_generation=old.map_generation,
        )
        await home.note_localization(fresh)
        assert await home.return_home(old.map_ref) == "goal"
        assert len(nav.goals) == 1
        assert nav.goals[0].map_generation == old.map_generation


async def test_startup_rejects_legacy_snapshot_without_same_generation_proof(
    tmp_path: Path,
) -> None:
    store = filled_store("yard", 3, (2.0, 2.0))
    graph = Session(store.map_ref)
    active = tmp_path / "maps" / "active"
    store.snapshot(Path("pose_graph"), ()).write(active)
    await graph.write(active / "pose_graph")
    gate = Gate()
    async with Database(tmp_path / "robot.db") as database:
        ops = MapOperations(
            database,
            store,
            MapArchive(graph, source_robot_id="r"),
            gate,
            robot_id="r",
            map_root=tmp_path / "maps",
            config_dir=tmp_path / "config",
        )
        await ops.prepare()
        assert gate.blocked
        assert store.review_required
        assert await ops._graph_guard() is not None


async def test_valid_checksum_bundle_with_unproven_snapshot_stays_in_review(tmp_path: Path) -> None:
    source = filled_store("yard", 3, (1.0, 1.0))
    local = filled_store("local", 1, (2.0, 2.0))
    original = tmp_path / "original.zip"
    bad = tmp_path / "mixed.zip"
    await archive_for(source, "r").export_bundle(source, source.map_ref, original)
    with zipfile.ZipFile(original) as bundle:
        files = {info.filename: bundle.read(info) for info in bundle.infolist()}
    occupancy = json.loads(files["occupancy/occupancy.json"])
    occupancy["bindings"][0]["hit_count"] += 1
    files["occupancy/occupancy.json"] = json.dumps(occupancy).encode()
    manifest = json.loads(files["manifest.json"])
    manifest["file_hashes"]["occupancy/occupancy.json"] = hashlib.sha256(
        files["occupancy/occupancy.json"]
    ).hexdigest()
    files["manifest.json"] = json.dumps(manifest).encode()
    with zipfile.ZipFile(bad, "w") as bundle:
        for name, payload in files.items():
            bundle.writestr(name, payload)
    graph = Session(local.map_ref)
    async with Database(tmp_path / "robot.db") as database:
        ops = session_operations(database, local, tmp_path, graph, robot_id="r")
        await ops.prepare()
        with pytest.raises(DomainError) as error:
            await ops.import_bundle(bad, "r", operation_id="mixed")
        assert error.value.code is ErrorCode.OPERATION_UNKNOWN
        assert graph.reads == []
        assert local.review_required
        assert (await ops.query("mixed")).status is OperationStatus.NEEDS_REVIEW


async def test_reconcile_rejects_unproven_history_before_reading_graph(tmp_path: Path) -> None:
    class Recoverable(Session):
        async def quiesce(self) -> None:
            return None

    store = filled_store("yard", 3, (2.0, 2.0))
    graph = Recoverable(store.map_ref)
    async with Database(tmp_path / "robot.db") as database:
        ops = session_operations(database, store, tmp_path, graph, robot_id="r")
        await ops.prepare()
        active = tmp_path / "maps" / "active"
        await ops._set_graph_guard("unknown", active)
        payload = json.loads((active / "occupancy.json").read_text())
        payload["bindings"][0]["hit_count"] += 1
        (active / "occupancy.json").write_text(json.dumps(payload))
        with pytest.raises(DomainError) as error:
            await ops.reconcile()
        assert error.value.code is ErrorCode.OPERATION_UNKNOWN
        assert graph.reads == []
        assert store.review_required
        assert await ops._graph_guard() is not None


async def test_startup_without_authoritative_snapshot_verifier_remains_guarded(
    tmp_path: Path,
) -> None:
    class NoVerifier(Session):
        verify_consistent = None  # type: ignore[assignment]

    store = filled_store("yard", 3, (2.0, 2.0))
    graph = NoVerifier(store.map_ref)
    gate = Gate()
    async with Database(tmp_path / "robot.db") as database:
        ops = session_operations(database, store, tmp_path, graph, robot_id="r", gate=gate)
        await ops.prepare()
        assert store.review_required and gate.blocked
        assert await ops._graph_guard() is not None


@pytest.mark.parametrize("foreign_home", [False, True])
async def test_same_ref_import_cannot_return_to_unverified_old_home(
    tmp_path: Path, foreign_home: bool
) -> None:
    from senior_control.waypoints import HomeService, LocalizedPose

    class Nav:
        def __init__(self) -> None:
            self.goals: list[object] = []

        async def navigate_qualified(self, goal: object) -> str:
            self.goals.append(goal)
            return "goal"

        async def cancel(self, execution_id: str):
            from senior_control.models import OperationResult

            return OperationResult(execution_id, OperationStatus.SUCCEEDED)

    local = filled_store("yard", 3, (2.0, 2.0))
    if foreign_home:
        source = filled_store("yard", 3, (8.0, 8.0))
    else:
        source = MapReferenceStore(local.map_ref, "wheel_odom", CELL)
    bundle = tmp_path / "no-local-home.zip"
    await archive_for(source, "foreign").export_bundle(source, source.map_ref, bundle)
    navigation = Nav()
    async with Database(tmp_path / "robot.db") as database:
        ops = session_operations(database, local, tmp_path, Session(local.map_ref), robot_id="r")
        await ops.prepare()
        home = HomeService(database, local, robot_id="r", navigation=navigation)
        await home.prepare()
        pose = LocalizedPose(
            Pose2D(2, 2, 0),
            local.map_ref,
            "kf-1",
            Pose2D(0, 0, 0),
            True,
            map_generation=local.map_generation,
        )
        saved = await home.set_current(pose)
        await ops.import_bundle(bundle, "r", operation_id="import")
        assert (await home.home()).id == saved.id
        await home.note_localization(
            LocalizedPose(
                Pose2D(8, 8, 0),
                local.map_ref,
                "kf-1",
                Pose2D(0, 0, 0),
                True,
                map_generation=local.map_generation,
            )
        )
        with pytest.raises(DomainError) as error:
            await home.return_home(local.map_ref)
        assert error.value.code is ErrorCode.CONFLICT
        assert navigation.goals == []
        restarted = HomeService(database, local, robot_id="r", navigation=navigation)
        await restarted.prepare()
        from senior_control.waypoints import StartupReference

        fresh = LocalizedPose(
            Pose2D(8, 8, 0),
            local.map_ref,
            "kf-1",
            Pose2D(0, 0, 0),
            True,
            map_generation=local.map_generation,
        )
        assert (await restarted.initialize(StartupReference(False, True, True), fresh)).initialized
        with pytest.raises(DomainError):
            await restarted.return_home(local.map_ref)
        assert navigation.goals == []


async def test_retained_home_remains_usable_when_current_binding_is_verified(
    tmp_path: Path,
) -> None:
    from senior_control.waypoints import HomeService, LocalizedPose

    class Nav:
        def __init__(self) -> None:
            self.goals: list[object] = []

        async def navigate_qualified(self, goal: object) -> str:
            self.goals.append(goal)
            return "goal"

        async def cancel(self, execution_id: str):
            from senior_control.models import OperationResult

            return OperationResult(execution_id, OperationStatus.SUCCEEDED)

    local = filled_store("yard", 3, (2.0, 2.0))
    navigation = Nav()
    async with Database(tmp_path / "robot.db") as database:
        ops = session_operations(database, local, tmp_path, Session(local.map_ref), robot_id="r")
        await ops.prepare()
        home = HomeService(database, local, robot_id="r", navigation=navigation)
        await home.prepare()
        pose = LocalizedPose(
            Pose2D(2, 2, 0),
            local.map_ref,
            "kf-1",
            Pose2D(0, 0, 0),
            True,
            map_generation=local.map_generation,
        )
        saved = await home.set_current(pose)
        source = MapReferenceStore(local.map_ref, "wheel_odom", CELL)
        source.replace(local.snapshot(Path("pose_graph"), ()))
        bundle = tmp_path / "without-home-kind.zip"
        await MapArchive(Graph(source.map_ref), source_robot_id="r").export_bundle(
            source, source.map_ref, bundle
        )
        await ops.import_bundle(bundle, "r", operation_id="import")
        assert (await home.home()).id == saved.id
        await home.note_localization(
            LocalizedPose(
                Pose2D(2, 2, 0),
                local.map_ref,
                "kf-1",
                Pose2D(0, 0, 0),
                True,
                map_generation=local.map_generation,
            )
        )
        assert await home.return_home(local.map_ref) == "goal"
        assert len(navigation.goals) == 1
