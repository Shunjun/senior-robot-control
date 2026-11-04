"""Bundle format rejects hostile archives and mixed map revisions."""

import asyncio
import hashlib
import json
import os
import stat
import tempfile
import zipfile
from collections.abc import Callable
from pathlib import Path

import pytest

from senior_control.errors import DomainError, ErrorCode
from senior_control.mapping.archive import MapArchive
from senior_control.mapping.evidence import CellEvidence
from senior_control.mapping.references import (
    AnchoredWaypoint,
    AnchorPose,
    AnchorSnapshot,
    GridExtent,
    SlamSnapshot,
)
from senior_control.mapping.store import MapReferenceStore
from senior_control.models import MapRef, Pose2D

CELL = 0.5
_HINT = Pose2D(1.0, 2.0, 0.25)
MAP = MapRef("yard", 3)


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


def filled_store(map_ref: MapRef = MAP) -> MapReferenceStore:
    store = MapReferenceStore(map_ref, "wheel_odom", CELL)
    point = (1.25, 0.25)
    wall = CellEvidence(cell_of(*point), "kf-1", 3, 0, 1.0, 3.0, "geometric_confirmation")
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


def _rewrite(source: Path, destination: Path, mutate) -> None:
    with zipfile.ZipFile(source) as bundle:
        files = {name: bundle.read(name) for name in bundle.namelist()}
    mutate(files)
    with zipfile.ZipFile(destination, "w") as bundle:
        for name, payload in files.items():
            bundle.writestr(name, payload)


@pytest.mark.asyncio
async def test_bundle_without_pose_graph_is_rejected(tmp_path: Path) -> None:
    store = filled_store()
    source = tmp_path / "yard.zip"
    await archive_for(store).export_bundle(store, store.map_ref, source)
    stripped = tmp_path / "stripped.zip"

    def drop_pose_graph(files: dict[str, bytes]) -> None:
        for name in list(files):
            if name.startswith("pose_graph/"):
                del files[name]
        manifest = json.loads(files["manifest.json"])
        manifest["file_hashes"] = {
            name: hashlib.sha256(payload).hexdigest()
            for name, payload in files.items()
            if name != "manifest.json"
        }
        files["manifest.json"] = json.dumps(manifest).encode()

    _rewrite(source, stripped, drop_pose_graph)
    with pytest.raises(DomainError) as caught:
        await archive_for(store).validate_and_extract(stripped, tmp_path / "unpack")

    assert caught.value.code is ErrorCode.VALIDATION


@pytest.mark.asyncio
async def test_export_includes_grid_map_yaml(tmp_path: Path) -> None:
    store = filled_store()
    destination = tmp_path / "yard.zip"
    await archive_for(store).export_bundle(store, store.map_ref, destination)

    with zipfile.ZipFile(destination) as bundle:
        yaml_text = bundle.read("grid/map.yaml").decode()
        image = bundle.read("grid/map.pgm")
    assert "image:" in yaml_text
    assert "resolution:" in yaml_text
    assert "origin:" in yaml_text
    assert image.startswith(b"P5") or image.startswith(b"P2")


@pytest.mark.asyncio
async def test_behavior_checksum_covers_source_robot(tmp_path: Path) -> None:
    store = filled_store()
    source = tmp_path / "yard.zip"
    await archive_for(store).export_bundle(store, store.map_ref, source)
    tampered = tmp_path / "tampered.zip"

    def retitle(files: dict[str, bytes]) -> None:
        manifest = json.loads(files["manifest.json"])
        manifest["source_robot_id"] = "robot-b"
        files["manifest.json"] = json.dumps(manifest).encode()

    _rewrite(source, tampered, retitle)
    with pytest.raises(DomainError) as caught:
        await archive_for(store).validate_and_extract(tampered, tmp_path / "unpack")

    assert caught.value.code is ErrorCode.VALIDATION


@pytest.mark.asyncio
async def test_waypoint_source_must_match_manifest(tmp_path: Path) -> None:
    store = filled_store()
    source = tmp_path / "yard.zip"
    await archive_for(store).export_bundle(store, store.map_ref, source)
    tampered = tmp_path / "source.zip"

    def retitle_waypoints(files: dict[str, bytes]) -> None:
        waypoints = json.loads(files["waypoints.json"])
        for item in waypoints["waypoints"]:
            item["source_robot_id"] = "robot-b"
        files["waypoints.json"] = json.dumps(waypoints).encode()
        manifest = json.loads(files["manifest.json"])
        manifest["file_hashes"] = {
            name: hashlib.sha256(payload).hexdigest()
            for name, payload in files.items()
            if name != "manifest.json"
        }
        files["manifest.json"] = json.dumps(manifest).encode()

    _rewrite(source, tampered, retitle_waypoints)
    with pytest.raises(DomainError) as caught:
        await archive_for(store).validate_and_extract(tampered, tmp_path / "unpack")

    assert caught.value.code is ErrorCode.VALIDATION


def test_exported_grid_uses_one_frame_with_the_yaml_origin(tmp_path: Path) -> None:
    """Negative cell indexes stay inside the grid, and the image row is map_server order."""
    from senior_control.mapping.archive import _write_grid

    store = MapReferenceStore(MAP, "wheel_odom", CELL)
    origin = Pose2D(0.0, 0.0, 0.0)
    occupied = ((4, 0), (-5, -18))
    for cell in occupied:
        world = (cell[0] * CELL + 0.1, cell[1] * CELL + 0.1)
        store.place(
            CellEvidence(cell, "kf-1", 3, 0, 1.0, 3.0, "geometric_confirmation"),
            world=world,
            anchor_pose=origin,
        )
    store.apply_anchor_update(
        AnchorSnapshot(
            MAP,
            "wheel_odom",
            (AnchorPose("kf-1", origin, True),),
            GridExtent(200, 200, -10.0, -10.0),
        )
    )
    _write_grid(store.snapshot(Path("pose_graph"), ()), tmp_path)
    payload = (tmp_path / "map.pgm").read_bytes()
    header, _, pixels = payload.partition(b"\n255\n")
    assert header == b"P5\n200 200"
    origin_index = int(-10.0 // CELL)
    width = height = 200
    for cell in occupied:
        px = cell[0] - origin_index
        py = cell[1] - origin_index
        row = height - 1 - py
        assert pixels[row * width + px] == 0
    yaml_text = (tmp_path / "map.yaml").read_text(encoding="utf-8")
    assert "origin: [-10.0, -10.0, 0.0]" in yaml_text


def test_zero_grid_is_not_promoted_to_a_pixel(tmp_path: Path) -> None:
    from senior_control.mapping.archive import _write_grid
    from senior_control.mapping.evidence import OccupancySnapshot
    from senior_control.mapping.references import SlamSnapshot

    snapshot = SlamSnapshot(
        MAP,
        "wheel_odom",
        CELL,
        Path("pose_graph"),
        OccupancySnapshot(MAP, ()),
        (),
        (),
        (),
        GridExtent(0, 0, 0.0, 0.0),
        (),
        (),
        False,
    )
    _write_grid(snapshot, tmp_path)
    payload = (tmp_path / "map.pgm").read_bytes()
    assert payload.startswith(b"P5\n0 0\n")
    assert payload.partition(b"\n255\n")[2] == b""


@pytest.mark.asyncio
async def test_waypoints_file_must_match_the_hashed_manifest(tmp_path: Path) -> None:
    store = filled_store()
    source = tmp_path / "yard.zip"
    await archive_for(store).export_bundle(store, store.map_ref, source)
    tampered = tmp_path / "moved.zip"

    def move_waypoint(files: dict[str, bytes]) -> None:
        waypoints = json.loads(files["waypoints.json"])
        waypoints["waypoints"][0]["x"] = 99
        files["waypoints.json"] = json.dumps(waypoints).encode()
        manifest = json.loads(files["manifest.json"])
        manifest["file_hashes"] = {
            name: hashlib.sha256(payload).hexdigest()
            for name, payload in files.items()
            if name != "manifest.json"
        }
        files["manifest.json"] = json.dumps(manifest).encode()

    _rewrite(source, tampered, move_waypoint)
    with pytest.raises(DomainError) as caught:
        await archive_for(store).validate_and_extract(tampered, tmp_path / "unpack")

    assert caught.value.code is ErrorCode.VALIDATION


def test_replace_and_clear_change_identity_together() -> None:
    source = MapReferenceStore(MapRef("yard", 4), "lidar_odom", 0.25)
    point = (0.3, 0.1)
    cell = (int(point[0] // 0.25), int(point[1] // 0.25))
    source.place(
        CellEvidence(cell, "kf-9", 2, 0, 1.0, 2.0, "geometric_confirmation"),
        world=point,
        anchor_pose=Pose2D(0.0, 0.0, 0.0),
    )
    snapshot = source.snapshot(Path("pose_graph"), ())
    target = filled_store()

    target.replace(snapshot)

    assert target.map_ref == MapRef("yard", 4)
    assert target.odom_owner == "lidar_odom"
    assert target.cell_size_m == 0.25
    assert target.occupancy().cells == source.occupancy().cells
    assert target.is_empty() is False
    target.clear(MapRef("cleared", 0), odom_owner="wheel_odom", cell_size_m=CELL)
    assert target.map_ref == MapRef("cleared", 0)
    assert target.odom_owner == "wheel_odom"
    assert target.cell_size_m == CELL
    assert target.is_empty()
    empty = MapReferenceStore(snapshot.map_ref, snapshot.odom_owner, snapshot.cell_size_m)
    empty.restore(snapshot)
    assert empty.occupancy().cells == source.occupancy().cells


@pytest.mark.asyncio
async def test_export_fsyncs_bundle_file_and_parent(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    synced: set[tuple[int, int]] = set()
    real_fsync = os.fsync

    def spy(fd: int) -> None:
        info = os.fstat(fd)
        synced.add((info.st_dev, info.st_ino))
        real_fsync(fd)

    monkeypatch.setattr(os, "fsync", spy)
    destination = tmp_path / "yard.zip"
    await archive_for(filled_store()).export_bundle(filled_store(), MAP, destination)

    for path in (destination, destination.parent):
        info = path.stat()
        assert (info.st_dev, info.st_ino) in synced


async def test_oversized_member_is_rejected_before_decompression(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    path = tmp_path / "large.zip"
    with zipfile.ZipFile(path, "w", compression=zipfile.ZIP_DEFLATED) as out:
        out.writestr("huge", b"x" * 100_000)
    archive = MapArchive(Graph(MAP), source_robot_id="r", max_uncompressed_bytes=1024)
    opened = False
    original = zipfile.ZipFile.open

    def observe(self: zipfile.ZipFile, *args, **kwargs):
        nonlocal opened
        opened = True
        return original(self, *args, **kwargs)

    monkeypatch.setattr(zipfile.ZipFile, "open", observe)
    with pytest.raises(DomainError):
        await archive.validate_and_extract(path, tmp_path / "unpacked")
    assert not opened
    assert not (tmp_path / "unpacked").exists()


def test_streamed_size_is_bounded_even_when_metadata_understates_it() -> None:
    import io

    class DishonestArchive:
        def infolist(self) -> list[zipfile.ZipInfo]:
            info = zipfile.ZipInfo("member")
            info.file_size = 1
            return [info]

        def open(self, info: zipfile.ZipInfo) -> io.BytesIO:
            return io.BytesIO(b"x" * 1025)

    archive = MapArchive(Graph(MAP), source_robot_id="r", max_uncompressed_bytes=1024)
    with pytest.raises(DomainError):
        archive._read_members(DishonestArchive())  # type: ignore[arg-type]


def test_excessive_member_count_is_rejected_before_opening() -> None:
    class TooMany:
        def infolist(self) -> list[zipfile.ZipInfo]:
            return [zipfile.ZipInfo(str(index)) for index in range(1025)]

        def open(self, info: zipfile.ZipInfo) -> None:
            raise AssertionError("members must not be opened")

    archive = MapArchive(Graph(MAP), source_robot_id="r")
    with pytest.raises(DomainError):
        archive._read_members(TooMany())  # type: ignore[arg-type]


async def test_archive_rejects_normalized_alias_before_materialization(tmp_path: Path) -> None:
    source = filled_store()
    base = tmp_path / "base.zip"
    await archive_for(source).export_bundle(source, source.map_ref, base)
    with zipfile.ZipFile(base) as original:
        files = {info.filename: original.read(info) for info in original.infolist()}
    manifest = json.loads(files["manifest.json"])
    files["./manifest.json"] = b"shadow"
    manifest["file_hashes"]["./manifest.json"] = hashlib.sha256(b"shadow").hexdigest()
    files["manifest.json"] = json.dumps(manifest).encode()
    hostile = tmp_path / "alias.zip"
    with zipfile.ZipFile(hostile, "w") as bundle:
        for name, payload in files.items():
            bundle.writestr(name, payload)
    destination = tmp_path / "extract"
    with pytest.raises(DomainError) as error:
        await archive_for(source).validate_and_extract(hostile, destination)
    assert error.value.code is ErrorCode.VALIDATION
    assert not destination.exists()


async def test_export_rejects_snapshot_changed_during_graph_capture(tmp_path: Path) -> None:
    import asyncio

    class ChangingGraph(Graph):
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

    store = filled_store()
    graph = ChangingGraph(store.map_ref)
    path = tmp_path / "mixed.zip"
    export = asyncio.create_task(
        archive_for(store, graph).export_bundle(store, store.map_ref, path)
    )
    await asyncio.wait_for(graph.entered.wait(), 1)
    with pytest.raises(DomainError) as late_write:
        store.apply_anchor_update(
            AnchorSnapshot(
                store.map_ref,
                "wheel_odom",
                (AnchorPose("kf-1", Pose2D(1, 0, 0), True),),
                GridExtent(8, 8, 0, 0),
            )
        )
    assert late_write.value.code is ErrorCode.CONFLICT
    graph.release.set()
    with pytest.raises(DomainError):
        await export
    assert not path.exists()


async def test_archive_rejects_file_and_directory_with_same_normalized_name(tmp_path: Path) -> None:
    path = tmp_path / "collision.zip"
    with zipfile.ZipFile(path, "w") as bundle:
        bundle.writestr("manifest.json", "{}")
        bundle.writestr("manifest.json/", b"")
    with pytest.raises(DomainError) as error:
        await archive_for(filled_store()).validate_and_extract(path, tmp_path / "extract")
    assert error.value.code is ErrorCode.VALIDATION
    assert "duplicated" in str(error.value)


async def test_export_without_consistent_capture_is_unavailable(tmp_path: Path) -> None:
    class UnverifiedGraph(Graph):
        capture_consistent = None  # type: ignore[assignment]

    store = filled_store()
    destination = tmp_path / "unsupported.zip"
    with pytest.raises(DomainError) as error:
        await archive_for(store, UnverifiedGraph(store.map_ref)).export_bundle(
            store, store.map_ref, destination
        )
    assert error.value.code is ErrorCode.UNAVAILABLE
    assert not destination.exists()


async def test_directory_member_cannot_hide_payload_from_bundle_fingerprint(tmp_path: Path) -> None:
    source = filled_store()
    original = tmp_path / "original.zip"
    await archive_for(source).export_bundle(source, source.map_ref, original)
    with zipfile.ZipFile(original) as archive:
        files = {info.filename: archive.read(info) for info in archive.infolist()}
    files["unused/"] = b"hidden-data"
    path = tmp_path / "hidden.zip"
    with zipfile.ZipFile(path, "w") as archive:
        for name, data in files.items():
            archive.writestr(name, data)
    with pytest.raises(DomainError) as error:
        await archive_for(source).validate_and_extract(path, tmp_path / "extract")
    assert error.value.code is ErrorCode.VALIDATION


async def test_export_without_authoritative_snapshot_verifier_is_unavailable(
    tmp_path: Path,
) -> None:
    class NoVerifier(Graph):
        verify_consistent = None  # type: ignore[assignment]

    store = filled_store()
    destination = tmp_path / "unverified.zip"
    with pytest.raises(DomainError) as error:
        await archive_for(store, NoVerifier(store.map_ref)).export_bundle(
            store, store.map_ref, destination
        )
    assert error.value.code is ErrorCode.UNAVAILABLE
    assert not destination.exists()


async def test_export_rejects_graph_replaced_during_verification(tmp_path: Path) -> None:
    class ChangingVerifier(Graph):
        async def verify_consistent(self, directory: Path, snapshot: SlamSnapshot) -> bool:
            verified = await super().verify_consistent(directory, snapshot)
            (directory / "graph.data").write_text("different scans", encoding="utf-8")
            return verified

    store = filled_store()
    destination = tmp_path / "changed.zip"
    with pytest.raises(DomainError) as error:
        await archive_for(store, ChangingVerifier(store.map_ref)).export_bundle(
            store, store.map_ref, destination
        )
    assert error.value.code is ErrorCode.OPERATION_UNKNOWN
    assert not destination.exists()


async def test_export_preserves_destination_created_during_verification(tmp_path: Path) -> None:
    destination = tmp_path / "competing.zip"

    class CompetingVerifier(Graph):
        async def verify_consistent(self, directory: Path, snapshot: SlamSnapshot) -> bool:
            destination.write_bytes(b"existing export")
            return await super().verify_consistent(directory, snapshot)

    store = filled_store()
    with pytest.raises(DomainError) as error:
        await archive_for(store, CompetingVerifier(store.map_ref)).export_bundle(
            store, store.map_ref, destination
        )
    assert error.value.code is ErrorCode.CONFLICT
    assert destination.read_bytes() == b"existing export"
    assert list(tmp_path.iterdir()) == [destination]


async def test_export_rejects_symlink_substituted_between_stat_and_open(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    original_lstat = Path.lstat
    external = tmp_path / "external.data"
    external.write_bytes(b"scans")

    def replace_after_stat(path: Path):
        entry = original_lstat(path)
        if path.name == "graph.data" and stat.S_ISREG(entry.st_mode):
            path.unlink()
            path.symlink_to(external)
        return entry

    monkeypatch.setattr(Path, "lstat", replace_after_stat)
    store = filled_store()
    destination = tmp_path / "unsafe.zip"
    with pytest.raises(DomainError) as error:
        await archive_for(store).export_bundle(store, store.map_ref, destination)
    assert error.value.code is ErrorCode.OPERATION_UNKNOWN
    assert not destination.exists()


async def test_export_uses_checked_bytes_without_reopening_graph_after_verification(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    original_write = SlamSnapshot.write

    def change_disk_after_verification(snapshot: SlamSnapshot, output: Path) -> None:
        if output.name == "occupancy":
            graph_directory = graph.capture_directory
            assert graph_directory is not None
            (graph_directory / "graph.data").write_bytes(b"other scans")
        original_write(snapshot, output)

    class RecordedGraph(Graph):
        capture_directory: Path | None = None

        async def verify_consistent(self, directory: Path, snapshot: SlamSnapshot) -> bool:
            self.capture_directory = directory
            return await super().verify_consistent(directory, snapshot)

    store = filled_store()
    graph = RecordedGraph(store.map_ref)
    monkeypatch.setattr(SlamSnapshot, "write", change_disk_after_verification)
    destination = tmp_path / "verified.zip"
    await archive_for(store, graph).export_bundle(store, store.map_ref, destination)
    with zipfile.ZipFile(destination) as archive:
        assert archive.read("pose_graph/graph.data") == b"scans"


@pytest.mark.parametrize("cancel_caller", [False, True])
async def test_capture_freeze_is_released_before_uncooperative_backend_finishes(
    tmp_path: Path, cancel_caller: bool
) -> None:
    store = filled_store()
    entered = asyncio.Event()
    cancelled = asyncio.Event()
    release = asyncio.Event()
    finished = asyncio.Event()
    late_codes: list[ErrorCode] = []

    class StalledGraph(Graph):
        async def capture_consistent(
            self, directory: Path, snapshot: Callable[[], SlamSnapshot]
        ) -> SlamSnapshot:
            entered.set()
            try:
                await asyncio.Future()
            except asyncio.CancelledError:
                cancelled.set()
                await release.wait()
                try:
                    return snapshot()
                except DomainError as error:
                    late_codes.append(error.code)
                    raise
                finally:
                    finished.set()

    archive = MapArchive(
        StalledGraph(store.map_ref),
        source_robot_id="robot-a",
        capture_timeout_seconds=1.0 if cancel_caller else 0.05,
    )
    destination = tmp_path / "stalled.zip"
    task = asyncio.create_task(archive.export_bundle(store, store.map_ref, destination))
    try:
        await asyncio.wait_for(entered.wait(), timeout=1)
        if cancel_caller:
            task.cancel()
            with pytest.raises(asyncio.CancelledError):
                await asyncio.wait_for(task, timeout=1)
        else:
            with pytest.raises(DomainError) as timeout:
                await asyncio.wait_for(task, timeout=1)
            assert timeout.value.code is ErrorCode.OPERATION_UNKNOWN
        await asyncio.wait_for(cancelled.wait(), timeout=1)
        assert not finished.is_set()

        assert store.occupancy().cells
        store.remove(store.occupancy().cells[0])
        assert not store.occupancy().cells
        with pytest.raises(DomainError) as busy:
            await archive.export_bundle(store, store.map_ref, destination)
        assert busy.value.code is ErrorCode.CONFLICT
        assert not destination.exists()
    finally:
        release.set()
        if cancelled.is_set():
            await asyncio.wait_for(finished.wait(), timeout=1)
    assert late_codes == [ErrorCode.OPERATION_UNKNOWN]
