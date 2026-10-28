"""Map identity, pose-graph anchors, and the saved localization/occupancy split."""

import json
import os
from dataclasses import dataclass
from pathlib import Path

from senior_control.mapping.evidence import CellEvidence, OccupancySnapshot
from senior_control.models import MapRef, Pose2D

FORMAT_VERSION = 1
# Humble 2.6.10 graph_visualization publishes node spheres in this namespace.
# Edge lines use slam_toolbox_edges and are not anchors.
NODE_MARKER_NAMESPACE = "slam_toolbox"


@dataclass(frozen=True)
class GridExtent:
    width: int
    height: int
    origin_x: float
    origin_y: float


@dataclass(frozen=True)
class AnchorPose:
    anchor_id: str
    pose: Pose2D | None
    available: bool


@dataclass(frozen=True)
class AnchorSnapshot:
    map_ref: MapRef
    odom_owner: str
    anchors: tuple[AnchorPose, ...]
    grid: GridExtent


@dataclass(frozen=True)
class AnchoredWaypoint:
    waypoint_id: str
    pose: Pose2D
    anchor_id: str
    review_required: bool


@dataclass(frozen=True)
class GraphNodeMarker:
    """One sphere from slam_toolbox 2.6.10 graph_visualization, without ROS types.

    That message carries corrected x/y and an identity quaternion. ``yaw`` is
    None when the marker did not include a corrected heading.
    """

    namespace: str
    marker_id: int
    x: float
    y: float
    yaw: float | None


@dataclass(frozen=True)
class CellBinding:
    evidence: CellEvidence
    world_x: float
    world_y: float
    anchor_pose: Pose2D
    removed: bool


@dataclass(frozen=True)
class WaypointBinding:
    waypoint: AnchoredWaypoint
    anchor_pose: Pose2D


@dataclass(frozen=True)
class ReferenceChange:
    map_ref: MapRef
    cells: tuple[CellEvidence, ...]
    waypoints: tuple[AnchoredWaypoint, ...]
    review_required: bool
    odom_owner: str


@dataclass(frozen=True)
class SlamSnapshot:
    """Pose graph for localization, plus the long-term occupancy layer.

    ``pose_graph_cells`` are cells the localization raster would paint. They
    are not copied into ``occupancy``. ``removed`` stays removed across reload.
    """

    map_ref: MapRef
    odom_owner: str
    cell_size_m: float
    pose_graph_prefix: Path
    occupancy: OccupancySnapshot
    removed: tuple[CellEvidence, ...]
    pose_graph_cells: tuple[tuple[int, int], ...]
    anchors: tuple[AnchorPose, ...]
    grid: GridExtent
    bindings: tuple[CellBinding, ...]
    waypoint_bindings: tuple[WaypointBinding, ...]
    review_required: bool

    def write(self, destination: Path) -> None:
        destination.mkdir(parents=True, exist_ok=True)
        payload = {
            "format_version": FORMAT_VERSION,
            "map_id": self.map_ref.map_id,
            "reference_revision": self.map_ref.reference_revision,
            "odom_owner": self.odom_owner,
            "cell_size_m": self.cell_size_m,
            "pose_graph_prefix": str(self.pose_graph_prefix),
            "review_required": self.review_required,
            "grid": _grid_payload(self.grid),
            "pose_graph_cells": [list(cell) for cell in self.pose_graph_cells],
            "anchors": [_anchor_payload(anchor) for anchor in self.anchors],
            "bindings": [_binding_payload(binding) for binding in self.bindings],
            "waypoint_bindings": [_waypoint_payload(binding) for binding in self.waypoint_bindings],
        }
        target = destination / "occupancy.json"
        temporary = destination / "occupancy.json.tmp"
        temporary.write_text(json.dumps(payload), encoding="utf-8")
        os.replace(temporary, target)

    @classmethod
    def read(cls, destination: Path) -> "SlamSnapshot":
        raw: object = json.loads((destination / "occupancy.json").read_text(encoding="utf-8"))
        body = _mapping(raw)
        if _whole(body.get("format_version")) != FORMAT_VERSION:
            raise ValueError("unsupported occupancy snapshot")
        bindings = tuple(_binding(_item) for _item in _sequence(body.get("bindings")))
        live = tuple(item.evidence for item in bindings if not item.removed)
        removed = tuple(item.evidence for item in bindings if item.removed)
        map_ref = MapRef(_text(body.get("map_id")), _whole(body.get("reference_revision")))
        return cls(
            map_ref,
            _text(body.get("odom_owner")),
            _real(body.get("cell_size_m")),
            Path(_text(body.get("pose_graph_prefix"))),
            OccupancySnapshot(map_ref, live),
            removed,
            tuple(_cell(item) for item in _sequence(body.get("pose_graph_cells"))),
            tuple(_anchor(item) for item in _sequence(body.get("anchors"))),
            _grid(_mapping(body.get("grid"))),
            bindings,
            tuple(_waypoint(item) for item in _sequence(body.get("waypoint_bindings"))),
            _flag(body.get("review_required")),
        )


def anchor_from_graph_marker(marker: GraphNodeMarker) -> AnchorPose | None:
    """Turn one Humble graph marker into an anchor, or ignore an edge marker.

    2.6.10 node markers do not carry ``GetCorrectedPose`` heading. A missing
    yaw stays unavailable so the store will not invent a rotation.
    """

    if marker.namespace != NODE_MARKER_NAMESPACE:
        return None
    anchor_id = str(marker.marker_id)
    if marker.yaw is None:
        return AnchorPose(anchor_id, None, False)
    return AnchorPose(anchor_id, Pose2D(marker.x, marker.y, marker.yaw), True)


def _grid_payload(grid: GridExtent) -> dict[str, float | int]:
    return {
        "width": grid.width,
        "height": grid.height,
        "origin_x": grid.origin_x,
        "origin_y": grid.origin_y,
    }


def _anchor_payload(anchor: AnchorPose) -> dict[str, object]:
    pose = anchor.pose
    return {
        "anchor_id": anchor.anchor_id,
        "available": anchor.available and pose is not None,
        "x": None if pose is None else pose.x,
        "y": None if pose is None else pose.y,
        "yaw": None if pose is None else pose.yaw,
    }


def _binding_payload(binding: CellBinding) -> dict[str, object]:
    evidence = binding.evidence
    return {
        "cell": [evidence.cell[0], evidence.cell[1]],
        "anchor_id": evidence.anchor_id,
        "hit_count": evidence.hit_count,
        "free_count": evidence.free_count,
        "last_valid_time": evidence.last_valid_time,
        "belief": evidence.belief,
        "change_source": evidence.change_source,
        "world_x": binding.world_x,
        "world_y": binding.world_y,
        "anchor_x": binding.anchor_pose.x,
        "anchor_y": binding.anchor_pose.y,
        "anchor_yaw": binding.anchor_pose.yaw,
        "removed": binding.removed,
    }


def _waypoint_payload(binding: WaypointBinding) -> dict[str, object]:
    waypoint = binding.waypoint
    return {
        "waypoint_id": waypoint.waypoint_id,
        "x": waypoint.pose.x,
        "y": waypoint.pose.y,
        "yaw": waypoint.pose.yaw,
        "anchor_id": waypoint.anchor_id,
        "review_required": waypoint.review_required,
        "anchor_x": binding.anchor_pose.x,
        "anchor_y": binding.anchor_pose.y,
        "anchor_yaw": binding.anchor_pose.yaw,
    }


def _mapping(value: object) -> dict[str, object]:
    if not isinstance(value, dict):
        raise ValueError("expected object")
    body: dict[str, object] = {}
    for key, item in value.items():
        if not isinstance(key, str):
            raise ValueError("expected string key")
        body[key] = item
    return body


def _sequence(value: object) -> tuple[object, ...]:
    if not isinstance(value, list):
        raise ValueError("expected list")
    return tuple(value)


def _item(value: object) -> dict[str, object]:
    return _mapping(value)


def _text(value: object) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError("expected text")
    return value


def _whole(value: object) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError("expected integer")
    return value


def _real(value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("expected number")
    return float(value)


def _flag(value: object) -> bool:
    if not isinstance(value, bool):
        raise ValueError("expected flag")
    return value


def _cell(value: object) -> tuple[int, int]:
    pair = _sequence(value)
    if len(pair) != 2:
        raise ValueError("expected cell")
    return (_whole(pair[0]), _whole(pair[1]))


def _grid(body: dict[str, object]) -> GridExtent:
    return GridExtent(
        _whole(body.get("width")),
        _whole(body.get("height")),
        _real(body.get("origin_x")),
        _real(body.get("origin_y")),
    )


def _anchor(value: object) -> AnchorPose:
    body = _item(value)
    available = _flag(body.get("available"))
    pose: Pose2D | None
    if body.get("x") is None or body.get("y") is None or body.get("yaw") is None:
        pose = None
        available = False
    else:
        pose = Pose2D(_real(body.get("x")), _real(body.get("y")), _real(body.get("yaw")))
    return AnchorPose(_text(body.get("anchor_id")), pose, available)


def _binding(value: object) -> CellBinding:
    body = _item(value)
    source = body.get("change_source")
    if source not in ("geometric_confirmation", "observed_free_space"):
        raise ValueError("expected change source")
    evidence = CellEvidence(
        _cell(body.get("cell")),
        _text(body.get("anchor_id")),
        _whole(body.get("hit_count")),
        _whole(body.get("free_count")),
        _real(body.get("last_valid_time")),
        _real(body.get("belief")),
        source,
    )
    return CellBinding(
        evidence,
        _real(body.get("world_x")),
        _real(body.get("world_y")),
        Pose2D(
            _real(body.get("anchor_x")),
            _real(body.get("anchor_y")),
            _real(body.get("anchor_yaw")),
        ),
        _flag(body.get("removed")),
    )


def _waypoint(value: object) -> WaypointBinding:
    body = _item(value)
    waypoint = AnchoredWaypoint(
        _text(body.get("waypoint_id")),
        Pose2D(_real(body.get("x")), _real(body.get("y")), _real(body.get("yaw"))),
        _text(body.get("anchor_id")),
        _flag(body.get("review_required")),
    )
    pose = Pose2D(
        _real(body.get("anchor_x")),
        _real(body.get("anchor_y")),
        _real(body.get("anchor_yaw")),
    )
    return WaypointBinding(waypoint, pose)
