"""Anchored map references stay stable when the grid grows and move only with their anchor."""

import asyncio
import importlib.util
import math
from dataclasses import fields, replace
from pathlib import Path

import pytest

from senior_control.errors import DomainError, ErrorCode
from senior_control.mapping.evidence import CellEvidence
from senior_control.mapping.references import (
    AnchoredWaypoint,
    AnchorPose,
    AnchorSnapshot,
    CellBinding,
    GraphNodeMarker,
    GridExtent,
    SlamSnapshot,
    anchor_from_graph_marker,
)
from senior_control.mapping.store import MapReferenceStore
from senior_control.models import MapRef, OperationStatus, Pose2D

ROOT = Path(__file__).resolve().parents[2]
CELL = 0.5
MAP = MapRef("home", 1)
BOX = (1.25, 0.25)
WALL = (2.25, 0.25)


def cell_of(x: float, y: float) -> tuple[int, int]:
    return (math.floor(x / CELL), math.floor(y / CELL))


def evidence(
    point: tuple[float, float],
    anchor: str,
    *,
    hits: int = 3,
    frees: int = 0,
    stamp: float = 1.0,
    belief: float = 3.0,
    source: str = "geometric_confirmation",
) -> CellEvidence:
    return CellEvidence(cell_of(*point), anchor, hits, frees, stamp, belief, source)  # type: ignore[arg-type]


def assert_no_velocity(value: object) -> None:
    assert {item.name for item in fields(value)}.isdisjoint({"vx", "wz", "velocity"})


def slam_adapter_module():
    path = ROOT / "ros2" / "src" / "senior_robot" / "senior_robot" / "slam_adapter.py"
    spec = importlib.util.spec_from_file_location("slam_adapter_under_test", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class MemoryPoseGraph:
    def __init__(self, cells: tuple[tuple[int, int], ...], owner: str = "wheel_odom") -> None:
        self._cells = cells
        self._owner = owner
        self.serialized: list[str] = []
        self.deserialized: list[tuple[str, int, Pose2D | None]] = []

    def odom_owner(self) -> str:
        return self._owner

    def painted_cells(self) -> tuple[tuple[int, int], ...]:
        return self._cells

    def grid(self) -> GridExtent:
        return GridExtent(8, 8, 0.0, 0.0)

    async def corrected_anchors(self) -> tuple[AnchorPose, ...]:
        return ()

    async def serialize_pose_graph(self, prefix: str) -> int:
        path = Path(prefix + ".posegraph")
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(",".join(f"{ix}:{iy}" for ix, iy in self._cells), encoding="utf-8")
        Path(prefix + ".data").write_text("scans", encoding="utf-8")
        self.serialized.append(prefix)
        return 0

    async def deserialize_pose_graph(
        self, prefix: str, match_type: int, initial_pose: Pose2D | None
    ) -> object:
        self.deserialized.append((prefix, match_type, initial_pose))
        return object()


def test_grid_growth_keeps_reference() -> None:
    store = MapReferenceStore(MAP, "wheel_odom", CELL)
    wall = evidence(WALL, "kf-wall")
    store.place(wall, world=WALL, anchor_pose=Pose2D(0.0, 0.0, 0.0))
    grown = AnchorSnapshot(
        MAP,
        "wheel_odom",
        (AnchorPose("kf-wall", Pose2D(0.0, 0.0, 0.0), True),),
        GridExtent(200, 200, -10.0, -10.0),
    )
    change = store.apply_anchor_update(grown)
    assert_no_velocity(change)
    assert change.map_ref == MAP
    assert change.map_ref.reference_revision == 1
    assert change.review_required is False
    assert change.odom_owner == "wheel_odom"
    assert store.map_ref == MAP
    assert store.grid == GridExtent(200, 200, -10.0, -10.0)
    assert change.cells == (wall,)

    wider = AnchorSnapshot(
        MapRef("home", 9),
        "wheel_odom",
        (AnchorPose("kf-wall", Pose2D(0.0, 0.0, 0.0), True),),
        GridExtent(400, 360, -20.0, -15.0),
    )
    again = store.apply_anchor_update(wider)
    assert again.map_ref == MAP
    assert store.map_ref == MAP
    assert again.cells == (wall,)
    assert store.grid == GridExtent(400, 360, -20.0, -15.0)


def test_loop_reprojects_box_and_waypoint() -> None:
    store = MapReferenceStore(MAP, "wheel_odom", CELL)
    anchor_a = Pose2D(0.0, 0.0, 0.0)
    anchor_b = Pose2D(10.0, 0.0, 0.0)
    box = evidence((2.0, 0.0), "kf-a")
    other = evidence((11.0, 0.0), "kf-b", hits=4, belief=4.0)
    store.place(box, world=(2.0, 0.0), anchor_pose=anchor_a)
    store.place(other, world=(11.0, 0.0), anchor_pose=anchor_b)
    dock = AnchoredWaypoint("dock", Pose2D(2.0, 0.0, 0.2), "kf-a", False)
    store.place_waypoint(dock, anchor_pose=anchor_a)

    update = AnchorSnapshot(
        MAP,
        "wheel_odom",
        (
            AnchorPose("kf-a", Pose2D(0.0, 1.0, math.pi / 2), True),
            AnchorPose("kf-b", Pose2D(12.0, 0.0, 0.0), True),
        ),
        GridExtent(80, 80, -5.0, -5.0),
    )
    change = store.apply_anchor_update(update)
    assert change.map_ref == MAP
    assert change.review_required is False
    assert_no_velocity(change)
    moved_box = next(item for item in change.cells if item.anchor_id == "kf-a")
    moved_other = next(item for item in change.cells if item.anchor_id == "kf-b")
    assert moved_box.cell == (0, 6)
    assert moved_box.hit_count == 3
    assert moved_box.belief == 3.0
    assert moved_box.change_source == "geometric_confirmation"
    assert moved_other.cell == (26, 0)
    assert moved_other.hit_count == 4
    moved = next(item for item in change.waypoints if item.waypoint_id == "dock")
    assert moved.review_required is False
    assert moved.anchor_id == "kf-a"
    assert moved.pose.x == pytest.approx(0.0, abs=1e-9)
    assert moved.pose.y == pytest.approx(3.0, abs=1e-9)
    assert moved.pose.yaw == pytest.approx(0.2 + math.pi / 2, abs=1e-9)
    assert store.map_ref == MAP

    again = store.apply_anchor_update(update)
    assert next(item for item in again.cells if item.anchor_id == "kf-a").cell == (0, 6)
    assert next(item for item in again.cells if item.anchor_id == "kf-b").cell == (26, 0)
    held = next(item for item in again.waypoints if item.waypoint_id == "dock")
    assert held.pose.x == pytest.approx(0.0, abs=1e-9)
    assert held.pose.y == pytest.approx(3.0, abs=1e-9)


def test_unavailable_anchor_marks_review_required() -> None:
    edge = anchor_from_graph_marker(GraphNodeMarker("slam_toolbox_edges", 0, 0.0, 0.0, None))
    assert edge is None
    parsed = anchor_from_graph_marker(GraphNodeMarker("slam_toolbox", 7, 5.0, 0.0, None))
    assert parsed is not None
    assert parsed.anchor_id == "7"
    assert parsed.available is False
    assert parsed.pose is None

    store = MapReferenceStore(MAP, "wheel_odom", CELL)
    origin = Pose2D(0.0, 0.0, 0.0)
    box = evidence((2.0, 0.0), "kf-a")
    other = evidence((10.0, 0.0), "kf-b")
    store.place(box, world=(2.0, 0.0), anchor_pose=origin)
    store.place(other, world=(10.0, 0.0), anchor_pose=Pose2D(10.0, 0.0, 0.0))
    store.place_waypoint(
        AnchoredWaypoint("dock", Pose2D(2.0, 0.0, 0.3), "kf-a", False),
        anchor_pose=origin,
    )
    change = store.apply_anchor_update(
        AnchorSnapshot(
            MAP,
            "wheel_odom",
            (
                AnchorPose("kf-a", Pose2D(9.0, 9.0, 0.4), False),
                AnchorPose("kf-b", Pose2D(11.0, 0.0, 0.0), True),
            ),
            GridExtent(10, 10, 0.0, 0.0),
        )
    )
    assert change.map_ref == MAP
    assert change.review_required is True
    assert store.review_required is True
    held = next(item for item in change.cells if item.anchor_id == "kf-a")
    assert held.cell == (4, 0)
    moved = next(item for item in change.cells if item.anchor_id == "kf-b")
    assert moved.cell == (22, 0)
    dock = next(item for item in change.waypoints if item.waypoint_id == "dock")
    assert dock.pose == Pose2D(2.0, 0.0, 0.3)
    assert dock.review_required is True

    missing = store.apply_anchor_update(
        AnchorSnapshot(
            MAP,
            "wheel_odom",
            (AnchorPose("kf-b", Pose2D(11.0, 0.0, 0.0), True),),
            GridExtent(10, 10, 0.0, 0.0),
        )
    )
    assert missing.review_required is True
    assert next(item for item in missing.cells if item.anchor_id == "kf-a").cell == (4, 0)
    assert next(item for item in missing.waypoints if item.waypoint_id == "dock").review_required

    hinted = MapReferenceStore(MAP, "wheel_odom", CELL)
    hinted_box = evidence((2.0, 0.0), "7")
    hinted.place(hinted_box, world=(2.0, 0.0), anchor_pose=origin)
    hinted_change = hinted.apply_anchor_update(
        AnchorSnapshot(MAP, "wheel_odom", (parsed,), GridExtent(4, 4, 0.0, 0.0))
    )
    assert hinted_change.review_required is True
    assert hinted_change.cells == (hinted_box,)
    assert hinted.map_ref == MAP


async def test_reload_preserves_removed_box(tmp_path: Path) -> None:
    module = slam_adapter_module()
    store = MapReferenceStore(MAP, "wheel_odom", CELL)
    wall = evidence(WALL, "kf-wall", hits=5, belief=4.0)
    box = evidence(BOX, "kf-box")
    origin = Pose2D(0.0, 0.0, 0.0)
    store.place(wall, world=WALL, anchor_pose=origin)
    store.place(box, world=BOX, anchor_pose=origin)
    store.remove(CellEvidence(box.cell, "free-2", 3, 3, 31.5, 0.0, "observed_free_space"))
    assert store.occupancy().cells == (wall,)

    backend = MemoryPoseGraph((box.cell, wall.cell))
    adapter = module.SlamAdapter(backend, store)
    destination = tmp_path / "session"
    snapshot = await adapter.serialize(destination)
    assert isinstance(snapshot, SlamSnapshot)
    assert_no_velocity(snapshot)
    assert snapshot.map_ref == MAP
    assert snapshot.odom_owner == "wheel_odom"
    assert box.cell in snapshot.pose_graph_cells
    assert wall.cell in snapshot.pose_graph_cells
    assert {item.cell for item in snapshot.occupancy.cells} == {wall.cell}
    assert snapshot.removed[0].cell == box.cell
    assert snapshot.removed[0].anchor_id == "kf-box"
    assert snapshot.removed[0].change_source == "observed_free_space"
    payload = Path(backend.serialized[0] + ".posegraph").read_text(encoding="utf-8")
    assert f"{box.cell[0]}:{box.cell[1]}" in payload
    assert backend.deserialized == []

    reread = SlamSnapshot.read(destination)
    assert reread.pose_graph_cells == snapshot.pose_graph_cells
    assert {item.cell for item in reread.occupancy.cells} == {wall.cell}
    assert reread.removed[0].anchor_id == "kf-box"

    fresh_backend = MemoryPoseGraph(reread.pose_graph_cells)
    fresh = MapReferenceStore(MAP, "wheel_odom", CELL)
    loaded = module.SlamAdapter(fresh_backend, fresh)
    result = await loaded.load(reread, None)
    assert result.status == OperationStatus.SUCCEEDED
    assert result.operation_id.strip()
    assert fresh_backend.deserialized == [
        (str(reread.pose_graph_prefix), module.SlamAdapter.MATCH_START_AT_FIRST_NODE, None)
    ]
    assert {item.cell for item in fresh.occupancy().cells} == {wall.cell}
    assert fresh.removals()[0].cell == box.cell
    assert fresh.removals()[0].anchor_id == "kf-box"
    assert fresh.map_ref == MAP

    posed = MapReferenceStore(MAP, "wheel_odom", CELL)
    pose_backend = MemoryPoseGraph(reread.pose_graph_cells)
    pose_result = await module.SlamAdapter(pose_backend, posed).load(reread, Pose2D(0.4, -0.2, 0.1))
    assert pose_result.status == OperationStatus.SUCCEEDED
    assert pose_backend.deserialized[-1][1] == module.SlamAdapter.MATCH_START_AT_GIVEN_POSE
    assert pose_backend.deserialized[-1][2] == Pose2D(0.4, -0.2, 0.1)
    assert box.cell not in {item.cell for item in posed.occupancy().cells}

    shift = fresh.apply_anchor_update(
        AnchorSnapshot(
            MAP,
            "wheel_odom",
            (
                AnchorPose("kf-wall", Pose2D(0.0, 1.0, 0.0), True),
                AnchorPose("kf-box", Pose2D(0.0, 1.0, 0.0), True),
            ),
            GridExtent(16, 16, -1.0, -1.0),
        )
    )
    assert shift.map_ref == MAP
    assert shift.review_required is False
    assert {item.cell for item in shift.cells} == {(4, 2)}
    painted = fresh.project_pose_graph((box.cell, (4, 2), (2, 2)))
    assert {item.cell for item in painted.cells} == {(4, 2)}
    fresh.clear_costmap()
    assert {item.cell for item in fresh.occupancy().cells} == {(4, 2)}
    assert fresh.removals()[0].cell == (2, 2)
    assert fresh.removals()[0].anchor_id == "kf-box"


async def test_null_deserialize_does_not_restore_occupancy(tmp_path: Path) -> None:
    module = slam_adapter_module()
    source = MapReferenceStore(MAP, "wheel_odom", CELL)
    wall = evidence(WALL, "kf-wall", hits=5, belief=4.0)
    source.place(wall, world=WALL, anchor_pose=Pose2D(0.0, 0.0, 0.0))
    snapshot = await module.SlamAdapter(MemoryPoseGraph(()), source).serialize(tmp_path / "layer")

    class FinishedFuture:
        def __init__(self, response: object, error: BaseException | None = None) -> None:
            self._response = response
            self._error = error

        def done(self) -> bool:
            return True

        def exception(self) -> BaseException | None:
            return self._error

        def result(self) -> object:
            if self._error is not None:
                raise self._error
            return self._response

    class FutureBackend(MemoryPoseGraph):
        def __init__(self, future: FinishedFuture) -> None:
            super().__init__(())
            self._future = future

        async def deserialize_pose_graph(
            self, prefix: str, match_type: int, initial_pose: Pose2D | None
        ) -> object | None:
            self.deserialized.append((prefix, match_type, initial_pose))
            return module.finished_deserialize_response(self._future)

    async def load_with(future: FinishedFuture) -> tuple[object, MapReferenceStore]:
        store = MapReferenceStore(MAP, "wheel_odom", CELL)
        result = await module.SlamAdapter(FutureBackend(future), store).load(snapshot, None)
        return result, store

    null_result, null_store = await load_with(FinishedFuture(None))
    assert null_result.status == OperationStatus.FAILED
    assert null_store.occupancy().cells == ()
    assert null_store.removals() == ()

    broken_result, broken_store = await load_with(
        FinishedFuture(None, RuntimeError("deserialize failed"))
    )
    assert broken_result.status == OperationStatus.FAILED
    assert broken_store.occupancy().cells == ()
    assert broken_store.removals() == ()

    restored_result, restored_store = await load_with(FinishedFuture(object()))
    assert restored_result.status == OperationStatus.SUCCEEDED
    assert {item.cell for item in restored_store.occupancy().cells} == {wall.cell}


async def test_single_map_odom_owner() -> None:
    store = MapReferenceStore(MAP, "wheel_odom", CELL)
    grid = GridExtent(4, 4, 0.0, 0.0)
    change = store.apply_anchor_update(AnchorSnapshot(MAP, "wheel_odom", (), grid))
    assert change.odom_owner == "wheel_odom"
    assert change.map_ref == MAP
    assert store.odom_owner == "wheel_odom"

    with pytest.raises(DomainError) as caught:
        store.apply_anchor_update(AnchorSnapshot(MAP, "ground_truth_odom", (), grid))
    assert caught.value.code == ErrorCode.CONFLICT
    assert store.odom_owner == "wheel_odom"
    assert store.map_ref == MAP

    with pytest.raises(DomainError) as blank:
        store.apply_anchor_update(AnchorSnapshot(MAP, " ", (), grid))
    assert blank.value.code == ErrorCode.CONFLICT
    assert store.map_ref == MAP

    with pytest.raises(DomainError) as other_map:
        store.apply_anchor_update(AnchorSnapshot(MapRef("other", 1), "wheel_odom", (), grid))
    assert other_map.value.code == ErrorCode.CONFLICT
    assert store.map_ref == MAP
    assert store.odom_owner == "wheel_odom"

    module = slam_adapter_module()

    class ForeignOdom:
        def odom_owner(self) -> str:
            return "visual_odom"

        def grid(self) -> GridExtent:
            return grid

        async def corrected_anchors(self) -> tuple[AnchorPose, ...]:
            return ()

    adapter = module.SlamAdapter(ForeignOdom(), store)
    with pytest.raises(DomainError) as foreign:
        await adapter.anchors()
    assert foreign.value.code == ErrorCode.CONFLICT
    assert store.odom_owner == "wheel_odom"
    assert store.map_ref == MAP

    slam_yaml = (ROOT / "ros2" / "src" / "senior_robot" / "config" / "slam.yaml").read_text(
        encoding="utf-8"
    )
    launch = (ROOT / "ros2" / "src" / "senior_robot" / "launch" / "mapping.launch.py").read_text(
        encoding="utf-8"
    )
    assert slam_yaml.count("odom_frame:") == 1
    assert "odom_frame: odom" in slam_yaml
    assert "ground_truth" not in slam_yaml
    assert "cmd_vel" not in launch
    assert "async_slam_toolbox_node" in launch
    assert "gazebo" not in launch.lower()


def test_nonce_ignores_obstacle_and_new_anchor_but_tracks_old_anchor() -> None:
    store = MapReferenceStore(MAP, "wheel_odom", CELL)
    origin = Pose2D(0.0, 0.0, 0.0)
    store.place(evidence(BOX, "kf-a"), world=BOX, anchor_pose=origin)
    store.apply_anchor_update(
        AnchorSnapshot(
            MAP, "wheel_odom", (AnchorPose("kf-a", origin, True),), GridExtent(8, 8, 0, 0)
        )
    )
    generation = store.map_generation
    store.place(evidence(WALL, "kf-a"), world=WALL, anchor_pose=origin)
    store.remove(evidence(WALL, "kf-a"))
    store.apply_anchor_update(
        AnchorSnapshot(
            MAP,
            "wheel_odom",
            (AnchorPose("kf-a", origin, True), AnchorPose("kf-new", Pose2D(3, 0, 0), True)),
            GridExtent(12, 12, 0, 0),
        )
    )
    assert store.map_generation == generation
    store.apply_anchor_update(
        AnchorSnapshot(
            MAP,
            "wheel_odom",
            (
                AnchorPose("kf-a", Pose2D(1, 0, 0), True),
                AnchorPose("kf-new", Pose2D(3, 0, 0), True),
            ),
            GridExtent(12, 12, 0, 0),
        )
    )
    assert store.map_generation != generation


@pytest.mark.parametrize(
    "mutation", ["cell", "removal", "waypoint", "anchors", "restore", "replace", "clear"]
)
def test_reference_capture_freeze_fences_mutations(mutation: str) -> None:
    store = MapReferenceStore(MAP, "wheel_odom", CELL)
    pose = Pose2D(0, 0, 0)
    hit = evidence(BOX, "kf")
    before = store.snapshot(Path("graph"), ())
    actions = {
        "cell": lambda: store.place(hit, world=BOX, anchor_pose=pose),
        "removal": lambda: store.remove(hit),
        "waypoint": lambda: store.place_waypoint(
            AnchoredWaypoint("home", pose, "kf", False), anchor_pose=pose
        ),
        "anchors": lambda: store.apply_anchor_update(
            AnchorSnapshot(MAP, "wheel_odom", (), GridExtent(5, 5, 0, 0))
        ),
        "restore": lambda: store.restore(before),
        "replace": lambda: store.replace(before),
        "clear": lambda: store.clear(MAP, odom_owner="wheel_odom", cell_size_m=CELL),
    }
    with pytest.raises(DomainError) as interrupted:
        with store.freeze_for_capture():
            with pytest.raises(DomainError) as error:
                actions[mutation]()
            assert error.value.code is ErrorCode.CONFLICT
            assert store.snapshot(Path("graph"), ()) == before
    assert interrupted.value.code is ErrorCode.OPERATION_UNKNOWN
    actions[mutation]()


async def test_reference_capture_freeze_releases_on_cancellation() -> None:
    store = MapReferenceStore(MAP, "wheel_odom", CELL)
    entered = asyncio.Event()

    async def capture() -> None:
        with store.freeze_for_capture():
            entered.set()
            await asyncio.Future()

    task = asyncio.create_task(capture())
    await asyncio.wait_for(entered.wait(), timeout=1)
    with pytest.raises(DomainError) as error:
        with store.freeze_for_capture():
            pytest.fail("a second capture cannot share the freeze")
    assert error.value.code is ErrorCode.CONFLICT
    task.cancel()
    with pytest.raises(asyncio.CancelledError):
        await task
    with store.freeze_for_capture():
        assert store.is_empty()


def test_reference_capture_freeze_allows_guard_and_rejects_preempted_capture() -> None:
    store = MapReferenceStore(MAP, "wheel_odom", CELL)
    with pytest.raises(DomainError) as error:
        with store.freeze_for_capture():
            store.guard_reference()
            assert store.review_required
            store.release_reference()
    assert error.value.code is ErrorCode.OPERATION_UNKNOWN
    with store.freeze_for_capture():
        assert not store.review_required


def test_occupancy_progress_commits_real_layers_and_fences_on_gap() -> None:
    store = MapReferenceStore(MAP, "wheel_odom", CELL)
    binding = store.bind_ingest(MAP, backend_boot="graph-a", source_epoch="scanner-a")
    assert store.ingest_progress is None
    pose = Pose2D(0, 0, 0)
    anchors = AnchorSnapshot(
        MAP, "wheel_odom", (AnchorPose("kf1", pose, True),), GridExtent(8, 8, 0, 0)
    )
    cell = CellBinding(evidence(BOX, "kf1"), *BOX, pose, False)
    progress = store.commit_ingest(binding, 1, "kf1", anchors, (cell,))
    assert (progress.binding, progress.sequence, progress.keyframe_id) == (binding, 1, "kf1")
    assert store.occupancy().cells == (cell.evidence,)
    with pytest.raises(DomainError):
        store.commit_ingest(binding, 1, "kf1", anchors, (cell,))
    assert store.ingest_progress == progress
    with pytest.raises(DomainError):
        store.commit_ingest(binding, 3, "kf1", anchors, ())
    assert store.ingest_progress is None
    with pytest.raises(DomainError):
        store.commit_ingest(binding, 2, "kf1", anchors, ())
    assert store.occupancy().cells == (cell.evidence,)


def test_occupancy_commit_failure_leaves_corrected_anchors_and_cells_unchanged() -> None:
    store = MapReferenceStore(MAP, "wheel_odom", CELL)
    binding = store.bind_ingest(MAP, backend_boot="graph-a", source_epoch="scanner-a")
    pose = Pose2D(0, 0, 0)
    anchors = AnchorSnapshot(
        MAP, "wheel_odom", (AnchorPose("kf1", pose, True),), GridExtent(8, 8, 0, 0)
    )
    cell = CellBinding(evidence(BOX, "kf1"), *BOX, pose, False)
    store.commit_ingest(binding, 1, "kf1", anchors, (cell,))
    before = store.snapshot(Path("graph"), ())
    corrected_pose = Pose2D(0.1, 0, 0)
    correction = AnchorSnapshot(
        MAP,
        "wheel_odom",
        (AnchorPose("kf1", corrected_pose, True), AnchorPose("kf2", pose, True)),
        GridExtent(9, 8, 0, 0),
    )
    good = CellBinding(evidence(WALL, "kf2"), *WALL, pose, False)
    invalid = CellBinding(evidence(BOX, "kf2"), *BOX, corrected_pose, False)
    with pytest.raises(DomainError):
        store.commit_ingest(binding, 2, "kf2", correction, (good, invalid))
    assert store.snapshot(Path("graph"), ()) == before
    assert store.ingest_progress is None


def test_occupancy_progress_requires_current_binding_after_clear_and_bypass() -> None:
    store = MapReferenceStore(MAP, "wheel_odom", CELL)
    old = store.bind_ingest(MAP, backend_boot="graph-a", source_epoch="scanner-a")
    pose = Pose2D(0, 0, 0)
    anchors = AnchorSnapshot(
        MAP, "wheel_odom", (AnchorPose("kf1", pose, True),), GridExtent(8, 8, 0, 0)
    )
    store.commit_ingest(old, 1, "kf1", anchors, ())
    store.place(evidence(BOX, "kf1"), world=BOX, anchor_pose=pose)
    assert store.ingest_progress is None
    with pytest.raises(DomainError):
        store.bind_ingest(MAP, backend_boot="graph-a", source_epoch="scanner-a")
    store.clear(MAP, odom_owner="wheel_odom", cell_size_m=CELL)
    binding = store.bind_ingest(MAP, backend_boot="graph-b", source_epoch="scanner-b")
    with pytest.raises(DomainError):
        store.commit_ingest(old, 1, "kf1", anchors, ())
    progress = store.commit_ingest(binding, 1, "kf1", anchors, ())
    assert store.ingest_progress == progress
    store.guard_reference()
    assert store.ingest_progress is None


def test_occupancy_progress_commits_corrected_removal_and_revokes_during_freeze() -> None:
    store = MapReferenceStore(MAP, "wheel_odom", CELL)
    binding = store.bind_ingest(MAP, backend_boot="graph-a", source_epoch="scanner-a")
    pose = Pose2D(0, 0, 0)
    grid = GridExtent(8, 8, 0, 0)
    first = AnchorSnapshot(MAP, "wheel_odom", (AnchorPose("kf1", pose, True),), grid)
    hit = CellBinding(evidence(BOX, "kf1"), *BOX, pose, False)
    store.commit_ingest(binding, 1, "kf1", first, (hit,))

    corrected_pose = Pose2D(1.0, 0, 0)
    correction = AnchorSnapshot(
        MAP,
        "wheel_odom",
        (AnchorPose("kf1", corrected_pose, True), AnchorPose("kf2", pose, True)),
        grid,
    )
    tombstone = CellBinding(evidence(WALL, "kf1"), *WALL, corrected_pose, True)
    progress = store.commit_ingest(binding, 2, "kf2", correction, (tombstone,))
    assert store.ingest_progress == progress
    assert store.occupancy().cells == ()
    assert store.removals() == (tombstone.evidence,)

    with pytest.raises(DomainError) as interrupted:
        with store.freeze_for_capture():
            store.revoke_ingest(binding)
            assert store.ingest_progress is None
    assert interrupted.value.code is ErrorCode.OPERATION_UNKNOWN


def test_occupancy_commit_rejects_oversized_batch_before_touching_store() -> None:
    store = MapReferenceStore(MAP, "wheel_odom", CELL)
    binding = store.bind_ingest(MAP, backend_boot="graph-a", source_epoch="scanner-a")
    pose = Pose2D(0, 0, 0)
    anchors = AnchorSnapshot(
        MAP, "wheel_odom", (AnchorPose("kf1", pose, True),), GridExtent(8, 8, 0, 0)
    )
    hit = CellBinding(evidence(BOX, "kf1"), *BOX, pose, False)
    before = store.snapshot(Path("graph"), ())
    with pytest.raises(DomainError, match="exceeds capture limits"):
        store.commit_ingest(binding, 1, "kf1", anchors, (hit,) * 8193)
    assert store.snapshot(Path("graph"), ()) == before
    assert store.ingest_progress is None


def test_first_occupancy_commit_cannot_adopt_a_preexisting_graph() -> None:
    store = MapReferenceStore(MAP, "wheel_odom", CELL)
    binding = store.bind_ingest(MAP, backend_boot="graph-a", source_epoch="scanner-a")
    pose = Pose2D(0, 0, 0)
    anchors = AnchorSnapshot(
        MAP,
        "wheel_odom",
        (AnchorPose("skipped", pose, True), AnchorPose("latest", pose, True)),
        GridExtent(8, 8, 0, 0),
    )
    with pytest.raises(DomainError):
        store.commit_ingest(binding, 1, "latest", anchors, ())
    assert store.ingest_progress is None


@pytest.mark.parametrize("invalid", ["origin", "dimension", "belief", "stamp", "count", "anchor"])
def test_malformed_ingest_metadata_cannot_advance_or_change_the_layer(invalid: str) -> None:
    store = MapReferenceStore(MAP, "wheel_odom", CELL)
    binding = store.bind_ingest(MAP, backend_boot="graph-a", source_epoch="scanner-a")
    pose = Pose2D(0, 0, 0)
    grid = GridExtent(8, 8, 0, 0)
    first = AnchorSnapshot(MAP, "wheel_odom", (AnchorPose("kf1", pose, True),), grid)
    original = CellBinding(evidence(BOX, "kf1"), *BOX, pose, False)
    store.commit_ingest(binding, 1, "kf1", first, (original,))
    before = store.snapshot(Path("graph"), ())
    if invalid == "origin":
        grid = replace(grid, origin_x=math.inf)
    elif invalid == "dimension":
        grid = replace(grid, width=-1)
    corrected = Pose2D(0.1, 0, 0)
    update = AnchorSnapshot(
        MAP,
        "wheel_odom",
        (
            AnchorPose("kf1", corrected, True),
            AnchorPose(" kf2 " if invalid == "anchor" else "kf2", pose, True),
        ),
        grid,
    )
    new_evidence = evidence(WALL, "kf2")
    if invalid == "belief":
        new_evidence = replace(new_evidence, belief=math.nan)
    elif invalid == "stamp":
        new_evidence = replace(new_evidence, last_valid_time=math.inf)
    elif invalid == "count":
        new_evidence = replace(new_evidence, hit_count=-1)
    change = CellBinding(new_evidence, *WALL, pose, False)

    with pytest.raises(DomainError) as failure:
        store.commit_ingest(binding, 2, "kf2", update, (change,))
    assert failure.value.code is ErrorCode.VALIDATION
    assert store.snapshot(Path("graph"), ()) == before
    assert store.ingest_progress is None
