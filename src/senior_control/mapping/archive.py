"""Portable SLAM map bundle. Tasks, secrets, and vehicle config stay out."""

import hashlib
import json
import stat
import tempfile
import zipfile
from dataclasses import dataclass
from pathlib import Path, PurePosixPath
from typing import Protocol

from senior_control.errors import DomainError, ErrorCode
from senior_control.mapping.references import AnchorPose, SlamSnapshot, durable_replace
from senior_control.mapping.store import MapReferenceStore
from senior_control.models import MapRef, Pose2D

BUNDLE_FORMAT_VERSION = 1
BACKEND_VERSION = "slam_toolbox 2.6.10"
ROS_VERSION = "humble"
SLAM_VERSION = "2.6.10"
COORDINATE_CONVENTION = "right_handed_xy_yaw"
STABLE_LAYER = "occupancy/occupancy.json"


@dataclass(frozen=True)
class Georeference:
    status: str
    datum: str | None = None
    height_datum: str | None = None
    translation_m: tuple[float, float, float] | None = None
    rotation: tuple[float, float, float, float] | None = None
    heading_convention: str | None = None
    quality: str | None = None
    source: str | None = None


@dataclass(frozen=True)
class BundleWaypoint:
    waypoint_id: str
    kind: str
    pose: Pose2D
    map_ref: MapRef
    anchor_id: str
    source_robot_id: str


@dataclass(frozen=True)
class BundleManifest:
    format_version: int
    file_hashes: tuple[tuple[str, str], ...]
    backend_version: str
    map_ref: MapRef
    stable_layer: str
    anchors: tuple[AnchorPose, ...]
    georeference: Georeference
    waypoints: tuple[BundleWaypoint, ...]
    ros_version: str
    slam_version: str
    coordinate_convention: str
    last_pose_hint: Pose2D | None
    startup_pose: Pose2D | None
    source_robot_id: str
    tasks_included: bool
    sensor_summary: tuple[tuple[str, str], ...]


class PoseGraphSource(Protocol):
    def map_ref(self) -> MapRef: ...

    def last_pose_hint(self) -> Pose2D | None: ...

    def painted_cells(self) -> tuple[tuple[int, int], ...]: ...

    async def write(self, directory: Path) -> tuple[str, ...]: ...

    async def read(self, directory: Path) -> None: ...

    async def reset(self) -> None: ...


class MapArchive:
    def __init__(
        self,
        pose_graph: PoseGraphSource,
        *,
        source_robot_id: str,
        georeference: Georeference | None = None,
        home_waypoint_id: str | None = None,
        sensor_summary: tuple[tuple[str, str], ...] = (),
        max_uncompressed_bytes: int = 67_108_864,
    ) -> None:
        self._pose_graph = pose_graph
        self._source_robot_id = source_robot_id
        self._georeference = georeference
        self._home_waypoint_id = home_waypoint_id
        self._sensor_summary = sensor_summary
        self._max_uncompressed_bytes = max_uncompressed_bytes

    async def export_bundle(
        self, store: MapReferenceStore, map_ref: MapRef, destination: Path
    ) -> BundleManifest:
        graph_ref = self._pose_graph.map_ref()
        if graph_ref != map_ref or store.map_ref != map_ref:
            raise DomainError(ErrorCode.CONFLICT, "pose graph and occupancy revisions differ")
        if destination.exists():
            raise DomainError(ErrorCode.CONFLICT, "bundle destination already exists")

        snapshot = store.snapshot(Path("pose_graph"), self._pose_graph.painted_cells())
        waypoints = tuple(
            BundleWaypoint(
                item.waypoint.waypoint_id,
                "home" if item.waypoint.waypoint_id == self._home_waypoint_id else "waypoint",
                item.waypoint.pose,
                map_ref,
                item.waypoint.anchor_id,
                self._source_robot_id,
            )
            for item in snapshot.waypoint_bindings
        )
        georeference = self._georeference or Georeference("unreferenced")
        with tempfile.TemporaryDirectory() as raw:
            staging = Path(raw)
            await self._pose_graph.write(staging / "pose_graph")
            snapshot.write(staging / "occupancy")
            waypoint_bytes = _dumps({"waypoints": [_waypoint_json(item) for item in waypoints]})
            (staging / "waypoints.json").write_bytes(waypoint_bytes)
            _write_grid(snapshot, staging / "grid")
            payloads = _file_bytes(staging)
            digests = {
                name: hashlib.sha256(payload).hexdigest() for name, payload in payloads.items()
            }
            manifest = BundleManifest(
                BUNDLE_FORMAT_VERSION,
                tuple(sorted(digests.items())),
                BACKEND_VERSION,
                map_ref,
                STABLE_LAYER,
                snapshot.anchors,
                georeference,
                waypoints,
                ROS_VERSION,
                SLAM_VERSION,
                COORDINATE_CONVENTION,
                self._pose_graph.last_pose_hint(),
                None,
                self._source_robot_id,
                False,
                self._sensor_summary,
            )
            files = dict(payloads)
            files["manifest.json"] = _dumps(_manifest_json(manifest))
        _write_zip(destination, files)
        return manifest

    async def write_pose_graph(self, directory: Path) -> tuple[str, ...]:
        return await self._pose_graph.write(directory)

    async def read_pose_graph(self, directory: Path) -> None:
        await self._pose_graph.read(directory)

    async def reset_pose_graph(self) -> None:
        await self._pose_graph.reset()

    async def validate_and_extract(self, path: Path, directory: Path) -> BundleManifest:
        try:
            archive = zipfile.ZipFile(path)
        except zipfile.BadZipFile as exc:
            raise DomainError(ErrorCode.VALIDATION, "bundle is not a zip archive") from exc
        with archive:
            files = self._read_members(archive)
        manifest_bytes = files.get("manifest.json")
        if manifest_bytes is None:
            raise DomainError(ErrorCode.VALIDATION, "bundle manifest is missing")
        try:
            manifest_body = _object(json.loads(manifest_bytes))
        except json.JSONDecodeError as exc:
            raise DomainError(ErrorCode.VALIDATION, "bundle manifest is not json") from exc
        declared = _hash_map(manifest_body.get("file_hashes"))
        actual = {
            name: hashlib.sha256(payload).hexdigest()
            for name, payload in files.items()
            if name != "manifest.json"
        }
        if declared != actual:
            raise DomainError(ErrorCode.VALIDATION, "bundle file hash mismatch")
        _check_content_hash(manifest_body)
        if files.get(STABLE_LAYER) is None:
            raise DomainError(ErrorCode.VALIDATION, "stable occupancy layer is missing")
        if not any(name.startswith("pose_graph/") and not name.endswith("/") for name in files):
            raise DomainError(ErrorCode.VALIDATION, "pose graph is missing")
        if "grid/map.yaml" not in files or "grid/map.pgm" not in files:
            raise DomainError(ErrorCode.VALIDATION, "grid map is missing")
        waypoint_bytes = files.get("waypoints.json")
        if waypoint_bytes is None:
            raise DomainError(ErrorCode.VALIDATION, "waypoints are missing")
        try:
            waypoint_body = _object(json.loads(waypoint_bytes))
        except json.JSONDecodeError as exc:
            raise DomainError(ErrorCode.VALIDATION, "waypoints are not json") from exc
        manifest = _manifest_from_json(manifest_body, _waypoint_list(waypoint_body))
        if manifest.tasks_included:
            raise DomainError(ErrorCode.VALIDATION, "tasks are not part of a map bundle")
        for waypoint in manifest.waypoints:
            if waypoint.map_ref != manifest.map_ref:
                raise DomainError(
                    ErrorCode.VALIDATION, "waypoint map does not match the bundle map"
                )
            if waypoint.source_robot_id != manifest.source_robot_id:
                raise DomainError(
                    ErrorCode.VALIDATION, "waypoint source does not match the bundle source"
                )
        _materialize(directory, files)
        return manifest

    def _read_members(self, archive: zipfile.ZipFile) -> dict[str, bytes]:
        files: dict[str, bytes] = {}
        total = 0
        for info in archive.infolist():
            name = info.filename
            _reject_member(name, info)
            if name.endswith("/"):
                continue
            if name in files:
                raise DomainError(ErrorCode.VALIDATION, "bundle path is duplicated")
            payload = archive.read(info)
            total += len(payload)
            if total > self._max_uncompressed_bytes:
                raise DomainError(ErrorCode.VALIDATION, "bundle exceeds the size limit")
            files[name] = payload
        if total > self._max_uncompressed_bytes:
            raise DomainError(ErrorCode.VALIDATION, "bundle exceeds the size limit")
        return files


def _reject_member(name: str, info: zipfile.ZipInfo) -> None:
    mode = (info.external_attr >> 16) & 0xFFFF
    if not name or "\x00" in name or "\\" in name or stat.S_ISLNK(mode):
        raise DomainError(ErrorCode.VALIDATION, "bundle member is not a regular file")
    relative = PurePosixPath(name)
    if relative.is_absolute() or ".." in relative.parts:
        raise DomainError(ErrorCode.VALIDATION, "bundle path escapes the archive")


def _file_bytes(root: Path) -> dict[str, bytes]:
    files: dict[str, bytes] = {}
    for path in sorted(root.rglob("*")):
        if not path.is_file():
            continue
        files[path.relative_to(root).as_posix()] = path.read_bytes()
    return files


def _write_zip(destination: Path, files: dict[str, bytes]) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    partial = destination.with_name(destination.name + ".partial")
    try:
        with zipfile.ZipFile(partial, "w") as bundle:
            for name in sorted(files):
                bundle.writestr(name, files[name])
        durable_replace(partial, destination)
    except BaseException:
        partial.unlink(missing_ok=True)
        raise


def _materialize(directory: Path, files: dict[str, bytes]) -> None:
    root = directory.resolve()
    for name, payload in files.items():
        target = directory.joinpath(*PurePosixPath(name).parts)
        resolved = target.resolve()
        if resolved != root and root not in resolved.parents:
            raise DomainError(ErrorCode.VALIDATION, "bundle path escapes the archive")
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(payload)


def _dumps(body: object) -> bytes:
    return json.dumps(body).encode("utf-8")


def _waypoint_json(waypoint: BundleWaypoint) -> dict[str, object]:
    return {
        "waypoint_id": waypoint.waypoint_id,
        "kind": waypoint.kind,
        "x": waypoint.pose.x,
        "y": waypoint.pose.y,
        "yaw": waypoint.pose.yaw,
        "map_id": waypoint.map_ref.map_id,
        "reference_revision": waypoint.map_ref.reference_revision,
        "anchor_id": waypoint.anchor_id,
        "source_robot_id": waypoint.source_robot_id,
    }


def _geo_json(geo: Georeference) -> dict[str, object]:
    if geo.status == "unreferenced":
        return {"status": "unreferenced"}
    body: dict[str, object] = {"status": geo.status}
    if geo.datum is not None:
        body["datum"] = geo.datum
    if geo.height_datum is not None:
        body["height_datum"] = geo.height_datum
    if geo.translation_m is not None:
        body["translation_m"] = list(geo.translation_m)
    if geo.rotation is not None:
        body["rotation"] = list(geo.rotation)
    if geo.heading_convention is not None:
        body["heading_convention"] = geo.heading_convention
    if geo.quality is not None:
        body["quality"] = geo.quality
    if geo.source is not None:
        body["source"] = geo.source
    return body


def _anchor_json(anchor: AnchorPose) -> dict[str, object]:
    pose = anchor.pose
    return {
        "anchor_id": anchor.anchor_id,
        "available": anchor.available and pose is not None,
        "x": None if pose is None else pose.x,
        "y": None if pose is None else pose.y,
        "yaw": None if pose is None else pose.yaw,
    }


def _pose_json(pose: Pose2D | None) -> dict[str, float] | None:
    if pose is None:
        return None
    return {"x": pose.x, "y": pose.y, "yaw": pose.yaw}


def _manifest_json(manifest: BundleManifest) -> dict[str, object]:
    body: dict[str, object] = {
        "format_version": manifest.format_version,
        "backend_version": manifest.backend_version,
        "map_ref": {
            "map_id": manifest.map_ref.map_id,
            "reference_revision": manifest.map_ref.reference_revision,
        },
        "stable_layer": manifest.stable_layer,
        "anchors": [_anchor_json(anchor) for anchor in manifest.anchors],
        "georeference": _geo_json(manifest.georeference),
        "waypoints": [_waypoint_json(item) for item in manifest.waypoints],
        "ros_version": manifest.ros_version,
        "slam_version": manifest.slam_version,
        "coordinate_convention": manifest.coordinate_convention,
        "last_pose_hint": _pose_json(manifest.last_pose_hint),
        "startup_pose": _pose_json(manifest.startup_pose),
        "source_robot_id": manifest.source_robot_id,
        "tasks_included": manifest.tasks_included,
        "sensor_summary": [list(item) for item in manifest.sensor_summary],
    }
    body["content_hash"] = hashlib.sha256(_canonical(body)).hexdigest()
    body["file_hashes"] = dict(manifest.file_hashes)
    return body


def _check_content_hash(body: dict[str, object]) -> None:
    declared = body.get("content_hash")
    if not isinstance(declared, str) or not declared:
        raise DomainError(ErrorCode.VALIDATION, "manifest behavior checksum is missing")
    covered = {
        key: value for key, value in body.items() if key not in {"content_hash", "file_hashes"}
    }
    actual = hashlib.sha256(_canonical(covered)).hexdigest()
    if actual != declared:
        raise DomainError(ErrorCode.VALIDATION, "manifest behavior checksum mismatch")


def _canonical(body: dict[str, object]) -> bytes:
    return json.dumps(body, sort_keys=True, separators=(",", ":")).encode("utf-8")


def _write_grid(snapshot: SlamSnapshot, directory: Path) -> None:
    width = snapshot.grid.width if snapshot.grid.width > 0 else 1
    height = snapshot.grid.height if snapshot.grid.height > 0 else 1
    occupied = {evidence.cell for evidence in snapshot.occupancy.cells}
    pixels = bytearray()
    for y in range(height):
        for x in range(width):
            pixels.append(0 if (x, y) in occupied else 205)
    directory.mkdir(parents=True, exist_ok=True)
    (directory / "map.pgm").write_bytes(f"P5\n{width} {height}\n255\n".encode() + bytes(pixels))
    origin = f"[{snapshot.grid.origin_x}, {snapshot.grid.origin_y}, 0.0]"
    yaml_text = (
        "image: map.pgm\n"
        f"resolution: {snapshot.cell_size_m}\n"
        f"origin: {origin}\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.196\n"
    )
    (directory / "map.yaml").write_text(yaml_text, encoding="utf-8")


def _manifest_from_json(
    body: dict[str, object], waypoints: tuple[BundleWaypoint, ...]
) -> BundleManifest:
    if _whole(body.get("format_version")) != BUNDLE_FORMAT_VERSION:
        raise DomainError(ErrorCode.VALIDATION, "unsupported bundle format")
    if body.get("tasks_included") is not False:
        raise DomainError(ErrorCode.VALIDATION, "tasks are not part of a map bundle")
    stable_layer = _text(body.get("stable_layer"))
    if stable_layer != STABLE_LAYER:
        raise DomainError(ErrorCode.VALIDATION, "unsupported stable layer")
    startup = body.get("startup_pose")
    return BundleManifest(
        BUNDLE_FORMAT_VERSION,
        tuple(sorted(_hash_map(body.get("file_hashes")).items())),
        _text(body.get("backend_version")),
        _map_ref(_object(body.get("map_ref"))),
        stable_layer,
        tuple(_anchor(item) for item in _list(body.get("anchors"))),
        _georeference(body.get("georeference")),
        waypoints,
        _text(body.get("ros_version")),
        _text(body.get("slam_version")),
        _text(body.get("coordinate_convention")),
        None if body.get("last_pose_hint") is None else _pose(_object(body.get("last_pose_hint"))),
        None if startup is None else _pose(_object(startup)),
        _text(body.get("source_robot_id")),
        False,
        _sensor_summary(body.get("sensor_summary")),
    )


def _waypoint_list(body: dict[str, object]) -> tuple[BundleWaypoint, ...]:
    return tuple(_waypoint(item) for item in _list(body.get("waypoints")))


def _waypoint(value: object) -> BundleWaypoint:
    body = _object(value)
    return BundleWaypoint(
        _text(body.get("waypoint_id")),
        _text(body.get("kind")),
        Pose2D(_real(body.get("x")), _real(body.get("y")), _real(body.get("yaw"))),
        MapRef(_text(body.get("map_id")), _whole(body.get("reference_revision"))),
        _text(body.get("anchor_id")),
        _text(body.get("source_robot_id")),
    )


def _georeference(value: object) -> Georeference:
    body = _object(value)
    status = _text(body.get("status"))
    if status == "unreferenced":
        return Georeference("unreferenced")
    return Georeference(
        status,
        None if body.get("datum") is None else _text(body.get("datum")),
        None if body.get("height_datum") is None else _text(body.get("height_datum")),
        None if body.get("translation_m") is None else _triple(body.get("translation_m")),
        None if body.get("rotation") is None else _quad(body.get("rotation")),
        None if body.get("heading_convention") is None else _text(body.get("heading_convention")),
        None if body.get("quality") is None else _text(body.get("quality")),
        None if body.get("source") is None else _text(body.get("source")),
    )


def _anchor(value: object) -> AnchorPose:
    body = _object(value)
    available = body.get("available") is True
    if body.get("x") is None or body.get("y") is None or body.get("yaw") is None:
        return AnchorPose(_text(body.get("anchor_id")), None, False)
    return AnchorPose(
        _text(body.get("anchor_id")),
        Pose2D(_real(body.get("x")), _real(body.get("y")), _real(body.get("yaw"))),
        available,
    )


def _hash_map(value: object) -> dict[str, str]:
    body = _object(value)
    hashes: dict[str, str] = {}
    for name, digest in body.items():
        if not isinstance(digest, str) or not digest:
            raise DomainError(ErrorCode.VALIDATION, "file hash is missing")
        hashes[name] = digest
    return hashes


def _sensor_summary(value: object) -> tuple[tuple[str, str], ...]:
    rows: list[tuple[str, str]] = []
    for item in _list(value):
        pair = _list(item)
        if len(pair) != 2:
            raise DomainError(ErrorCode.VALIDATION, "sensor summary entry is invalid")
        rows.append((_text(pair[0]), _text(pair[1])))
    return tuple(rows)


def _map_ref(body: dict[str, object]) -> MapRef:
    return MapRef(_text(body.get("map_id")), _whole(body.get("reference_revision")))


def _pose(body: dict[str, object]) -> Pose2D:
    return Pose2D(_real(body.get("x")), _real(body.get("y")), _real(body.get("yaw")))


def _triple(value: object) -> tuple[float, float, float]:
    items = _list(value)
    if len(items) != 3:
        raise DomainError(ErrorCode.VALIDATION, "expected three georeference values")
    return (_real(items[0]), _real(items[1]), _real(items[2]))


def _quad(value: object) -> tuple[float, float, float, float]:
    items = _list(value)
    if len(items) != 4:
        raise DomainError(ErrorCode.VALIDATION, "expected four georeference values")
    return (_real(items[0]), _real(items[1]), _real(items[2]), _real(items[3]))


def _object(value: object) -> dict[str, object]:
    if not isinstance(value, dict):
        raise DomainError(ErrorCode.VALIDATION, "expected object")
    body: dict[str, object] = {}
    for key, item in value.items():
        if not isinstance(key, str):
            raise DomainError(ErrorCode.VALIDATION, "expected string key")
        body[key] = item
    return body


def _list(value: object) -> tuple[object, ...]:
    if not isinstance(value, list):
        raise DomainError(ErrorCode.VALIDATION, "expected list")
    return tuple(value)


def _text(value: object) -> str:
    if not isinstance(value, str) or not value.strip():
        raise DomainError(ErrorCode.VALIDATION, "expected text")
    return value


def _whole(value: object) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise DomainError(ErrorCode.VALIDATION, "expected integer")
    return value


def _real(value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise DomainError(ErrorCode.VALIDATION, "expected number")
    return float(value)
