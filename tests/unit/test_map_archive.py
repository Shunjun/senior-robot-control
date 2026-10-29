"""Bundle format rejects hostile archives and mixed map revisions."""

import json
import stat
import zipfile
from pathlib import Path

import pytest

from senior_control.errors import DomainError, ErrorCode
from senior_control.mapping.archive import MapArchive
from senior_control.mapping.evidence import CellEvidence
from senior_control.mapping.references import AnchoredWaypoint, AnchorPose, AnchorSnapshot, GridExtent
from senior_control.mapping.store import MapReferenceStore
from senior_control.models import MapRef, Pose2D

CELL = 0.5
MAP = MapRef("yard", 3)


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
        return ((1, 1),)

    async def write(self, directory: Path) -> tuple[str, ...]:
        directory.mkdir(parents=True, exist_ok=True)
        (directory / "graph.posegraph").write_text("nodes", encoding="utf-8")
        (directory / "graph.data").write_text("scans", encoding="utf-8")
        return ("graph.posegraph", "graph.data")


def filled_store(map_ref: MapRef = MAP) -> MapReferenceStore:
    store = MapReferenceStore(map_ref, "wheel_odom", CELL)
    point = (1.25, 0.25)
    wall = CellEvidence(
        cell_of(*point), "kf-1", 3, 0, 1.0, 3.0, "geometric_confirmation"
    )
    origin = Pose2D(0.0, 0.0, 0.0)
    store.place(wall, world=point, anchor_pose=origin)
    store.place_waypoint(
        AnchoredWaypoint("home", Pose2D(1.0, 0.2, 0.1), "kf-1", False),
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


def archive_for(store: MapReferenceStore, graph: Graph | None = None) -> MapArchive:
    return MapArchive(
        graph or Graph(store.map_ref),
        source_robot_id="robot-a",
        home_waypoint_id="home",
        sensor_summary=(("lidar", "source-description-only"),),
    )


def hostile_zip(path: Path) -> None:
    with zipfile.ZipFile(path, "w") as bundle:
        escaped = zipfile.ZipInfo("../escaped.txt")
        bundle.writestr(escaped, "owned")
        nested = zipfile.ZipInfo("occupancy/../../escaped.txt")
        bundle.writestr(nested, "owned-too")
        link = zipfile.ZipInfo("alias.txt")
        link.create_system = 3
        link.external_attr = (stat.S_IFLNK | 0o777) << 16
        bundle.writestr(link, "credentials.json")


@pytest.mark.asyncio
async def test_path_traversal_and_symlink_rejected(tmp_path: Path) -> None:
    bundle = tmp_path / "hostile.zip"
    hostile_zip(bundle)
    unpack = tmp_path / "unpack"
    unpack.mkdir()
    archive = archive_for(filled_store())

    with pytest.raises(DomainError) as caught:
        await archive.validate_and_extract(bundle, unpack)

    assert caught.value.code is ErrorCode.VALIDATION
    assert not (tmp_path / "escaped.txt").exists()
    assert list(unpack.rglob("*")) == []
    assert not (unpack / "alias.txt").is_symlink()


@pytest.mark.asyncio
async def test_mixed_revision_export_rejected(tmp_path: Path) -> None:
    store = filled_store(MAP)
    destination = tmp_path / "mixed.zip"
    archive = archive_for(store, Graph(MapRef(MAP.map_id, MAP.reference_revision + 1)))

    with pytest.raises(DomainError) as caught:
        await archive.export_bundle(store, MAP, destination)

    assert caught.value.code is ErrorCode.CONFLICT
    assert not destination.exists()


@pytest.mark.asyncio
async def test_export_without_georeference_marks_unreferenced(tmp_path: Path) -> None:
    store = filled_store()
    destination = tmp_path / "yard.zip"
    manifest = await archive_for(store).export_bundle(store, store.map_ref, destination)

    assert manifest.georeference.status == "unreferenced"
    assert manifest.georeference.datum is None
    assert manifest.startup_pose is None
    assert manifest.last_pose_hint == Pose2D(1.0, 2.0, 0.25)
    assert manifest.tasks_included is False
    assert manifest.map_ref == store.map_ref
    assert manifest.stable_layer == "occupancy/occupancy.json"
    assert any(anchor.anchor_id == "kf-1" for anchor in manifest.anchors)
    home = next(item for item in manifest.waypoints if item.waypoint_id == "home")
    assert home.kind == "home"
    assert home.map_ref == store.map_ref
    assert home.source_robot_id == "robot-a"
    with zipfile.ZipFile(destination) as bundle:
        names = set(bundle.namelist())
        occupancy = bundle.read("occupancy/occupancy.json")
        body = json.loads(bundle.read("manifest.json"))
    assert "occupancy/occupancy.json" in names
    assert "pose_graph/graph.posegraph" in names
    assert "pose_graph/graph.data" in names
    assert b"pose_graph" in occupancy
    assert body["georeference"] == {"status": "unreferenced"}
    assert body["startup_pose"] is None
    assert body["tasks_included"] is False
    assert body["file_hashes"]["pose_graph/graph.posegraph"]
    assert "credentials" not in json.dumps(body)
