"""Map persistence acceptance assertions that do not require ROS."""

from __future__ import annotations

import math
from pathlib import Path

from conftest import SimulationProfile

from senior_control.localization.geo import AlignedPose, GeoAlignment, QualifiedFix
from senior_control.localization.relocalization import PoseHint, Relocalizer
from senior_control.mapping import (
    CellEvidence,
    MappingObservation,
    OccupancyMaintainer,
    OccupancyThresholds,
    RangeRay,
    SceneHit,
)
from senior_control.mapping.archive import Georeference, MapArchive
from senior_control.mapping.operations import MapOperations
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

MAP = MapRef("acceptance-map", 1)
POSE = Pose2D(0.25, 0.25, 0.0)
BOX = (1.25, 0.25)


class FixedClock:
    def monotonic(self) -> float:
        return 100.0


class Graph:
    def __init__(self, map_ref: MapRef) -> None:
        self._map_ref = map_ref

    def map_ref(self) -> MapRef:
        return self._map_ref

    def last_pose_hint(self) -> Pose2D:
        return Pose2D(1.0, 2.0, 0.25)

    def painted_cells(self) -> tuple[tuple[int, int], ...]:
        return ((2, 0),)

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
        self.blocked = False

    async def stop_and_block_navigation(self) -> None:
        self.blocked = True


def referenced() -> Georeference:
    return Georeference(
        "referenced",
        datum="WGS84",
        height_datum="ellipsoidal",
        translation_m=(10.0, -3.0, 0.0),
        rotation=(0.0, 0.0, 0.0, 1.0),
        heading_convention="enu_math_yaw",
        quality="survey",
        source="acceptance",
    )


def filled_store(map_ref: MapRef, point: tuple[float, float]) -> MapReferenceStore:
    store = MapReferenceStore(map_ref, "wheel_odom", 0.5)
    anchor_pose = Pose2D(0.0, 0.0, 0.0)
    store.place(
        CellEvidence(
            (int(point[0] // 0.5), int(point[1] // 0.5)),
            "kf-1",
            3,
            0,
            1.0,
            3.0,
            "geometric_confirmation",
        ),
        world=point,
        anchor_pose=anchor_pose,
    )
    store.place_waypoint(
        AnchoredWaypoint("home", Pose2D(point[0], point[1], 0.1), "kf-1", False),
        anchor_pose=anchor_pose,
    )
    store.apply_anchor_update(
        AnchorSnapshot(
            map_ref,
            "wheel_odom",
            (AnchorPose("kf-1", anchor_pose, True),),
            GridExtent(8, 8, 0.0, 0.0),
        )
    )
    return store


def operations(
    database: Database,
    store: MapReferenceStore,
    root: Path,
    *,
    robot_id: str,
    georeference: Georeference | None = None,
) -> tuple[MapOperations, Gate]:
    gate = Gate()
    return (
        MapOperations(
            database,
            store,
            MapArchive(
                Graph(store.map_ref),
                source_robot_id=robot_id,
                georeference=georeference,
                home_waypoint_id="home",
            ),
            gate,
            robot_id=robot_id,
            map_root=root / "maps",
            config_dir=root / "config",
        ),
        gate,
    )


def thresholds() -> OccupancyThresholds:
    return OccupancyThresholds(
        cell_size_m=0.5,
        confirm_hits=3,
        confirm_span_s=0.4,
        clear_free=3,
        clear_span_s=0.4,
        distance_tolerance_m=0.1,
        hit_weight=1.0,
        free_weight=1.0,
        occupied_belief=2.5,
        free_belief=0.5,
        belief_min=0.0,
        belief_max=6.0,
        max_age_s=0.5,
        min_quality=0.5,
    )


def observation(
    stamp: float,
    *,
    hits: tuple[SceneHit, ...] = (),
    ray: RangeRay | None = None,
    dynamic: bool = False,
) -> MappingObservation:
    rays = () if ray is None else (ray,)
    return MappingObservation(
        MAP,
        f"kf-{stamp}",
        POSE,
        stamp,
        rays,
        hits,
        (dynamic,) * len(rays),
        (False,) * len(rays),
        1.0,
        True,
        False,
        0.0,
    )


def test_box_added_then_removed_survives_reload(
    tmp_path: Path,
    simulation_profile: SimulationProfile,
) -> None:
    maintainer = OccupancyMaintainer(thresholds())
    box_ray = RangeRay(0.0, math.dist((POSE.x, POSE.y), BOX), True, False, None)
    for stamp in (0.0, 0.5, 1.0):
        maintainer.update(observation(stamp, hits=(SceneHit(*BOX, "static"),), ray=box_ray))
    added = maintainer.snapshot().cells
    assert len(added) == 1

    store = MapReferenceStore(MAP, "wheel_odom", 0.5)
    store.place(added[0], world=BOX, anchor_pose=POSE)
    free_ray = RangeRay(0.0, 2.5, True, False, None)
    removed: tuple[CellEvidence, ...] = ()
    for stamp in (2.0, 2.5, 3.0):
        removed = maintainer.update(observation(stamp, ray=free_ray)).removed
    assert len(removed) == 1
    store.remove(removed[0])

    destination = tmp_path / "snapshot"
    store.snapshot(Path("graph"), ()).write(destination)
    loaded = SlamSnapshot.read(destination)

    assert loaded.occupancy.cells == ()
    assert len(loaded.removed) == 1
    assert loaded.removed[0].cell == removed[0].cell
    assert loaded.removed[0].change_source == "observed_free_space"
    assert loaded.removed[0].anchor_id == added[0].anchor_id
    assert loaded.map_ref == MAP
    assert simulation_profile.database.parent == tmp_path


def test_person_not_persisted() -> None:
    maintainer = OccupancyMaintainer(thresholds())
    person = (0.75, 0.25)
    ray = RangeRay(0.0, 0.5, True, False, None)
    for stamp in (0.0, 0.5, 1.0, 1.5):
        delta = maintainer.update(
            observation(stamp, hits=(SceneHit(*person, "person"),), ray=ray, dynamic=True)
        )
        assert delta.added == ()
    assert maintainer.snapshot().cells == ()


async def test_map_import_with_and_without_gps() -> None:
    calls: list[Pose2D | AlignedPose | None] = []

    def match(seed: Pose2D | AlignedPose | None) -> tuple[Pose2D, bool]:
        calls.append(seed)
        return Pose2D(3.0, 4.0, 0.2), True

    without_gps = Relocalizer(
        map_ref=MAP,
        georeference=Georeference("unreferenced"),
        alignment=None,
        scan_match=match,
        bundle_has_gps=False,
    )
    manual = await without_gps.begin(MAP, PoseHint(Pose2D(2.9, 4.1, 0.2), "manual"))
    assert manual.status is OperationStatus.SUCCEEDED
    assert calls[-1] == Pose2D(2.9, 4.1, 0.2)

    georeference = referenced()
    alignment = GeoAlignment(
        georeference,
        origin_latitude_deg=31.0,
        origin_longitude_deg=121.0,
        origin_altitude_m=4.0,
        uncertainty_m=(0.4, 0.5, 0.05),
        clock=FixedClock(),
        max_age_s=0.5,
        max_horizontal_std_m=3.0,
    )
    with_gps = Relocalizer(
        map_ref=MAP,
        georeference=georeference,
        alignment=alignment,
        scan_match=match,
        bundle_has_gps=True,
    )
    gps = QualifiedFix(31.0, 121.0, 4.0, 100.0, 0.3, 0.5, "fixed", 2, True, None)
    imported = await with_gps.begin(MAP, PoseHint(None, "gps", gps))

    assert imported.status is OperationStatus.SUCCEEDED
    assert isinstance(calls[-1], AlignedPose)
    assert with_gps.ready is True
    assert with_gps.pose == Pose2D(3.0, 4.0, 0.2)


async def test_cross_robot_home_import_stays_pending_confirmation(tmp_path: Path) -> None:
    source_ref = MapRef("source-yard", 3)
    source = filled_store(source_ref, (1.25, 0.25))
    local = filled_store(MapRef("local-yard", 1), (2.25, 0.25))
    bundle = tmp_path / "source.zip"

    async with (
        Database(tmp_path / "source.db") as source_db,
        Database(tmp_path / "target.db") as target_db,
    ):
        exporter, _ = operations(
            source_db,
            source,
            tmp_path / "source",
            robot_id="robot-a",
            georeference=referenced(),
        )
        importer, gate = operations(
            target_db,
            local,
            tmp_path / "target",
            robot_id="robot-b",
        )
        await exporter.prepare()
        await importer.prepare()
        await exporter.export_bundle(source_ref, bundle, operation_id="export-cross-home")
        result = await importer.import_bundle(
            bundle,
            "robot-b",
            operation_id="import-cross-home",
        )
        home = await importer.home_for("robot-b")

    assert result.status is OperationStatus.SUCCEEDED
    assert home is not None
    assert home.source_robot_id == "robot-a"
    assert home.map_ref == source_ref
    assert home.status == "pending_confirmation"
    assert home.adopted is False
    assert gate.blocked is True
