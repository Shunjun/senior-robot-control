"""Derive the isolated baseline occupancy grid from its pinned SDF."""

import hashlib
import math
import xml.etree.ElementTree as ET
from pathlib import Path

from senior_control.mapping.workspace import WorkspaceGrid
from senior_control.models import MapRef, Pose2D

_BASELINE_HASH = "528523ece69d7eaf1eeb2d9fd2930a024ac56a151f62ffa939bd63c338db1c44"
_RESOLUTION = 0.05
_SIDE = 160
_ORIGIN = Pose2D(-4.0, -4.0, 0.0)


def baseline_grid(path: Path, generation: str) -> WorkspaceGrid:
    """Rasterize the collision box in the pinned SDF, in Gazebo world coordinates."""
    with path.open("rb") as stream:
        data = stream.read(65_537)
    digest = hashlib.sha256(data).hexdigest()
    if digest != _BASELINE_HASH:
        raise ValueError("unsupported Gazebo baseline world")
    world = ET.fromstring(data)
    model = world.find(".//model[@name='test_obstacle']")
    assert model is not None
    x, y, *_ = map(float, (model.findtext("pose") or "").split())
    width, height, _ = map(
        float, (model.findtext("link/collision/geometry/box/size") or "").split()
    )
    cells = bytearray(_SIDE**2)
    for row in range(_SIDE):
        lo_y = _ORIGIN.y + row * _RESOLUTION
        if lo_y + _RESOLUTION <= y - height / 2 or lo_y >= y + height / 2:
            continue
        for column in range(_SIDE):
            lo_x = _ORIGIN.x + column * _RESOLUTION
            if lo_x + _RESOLUTION > x - width / 2 and lo_x < x + width / 2:
                cells[row * _SIDE + column] = 100
    return WorkspaceGrid(
        MapRef(f"sim-baseline-{digest[:24]}", 1),
        generation,
        _SIDE,
        _SIDE,
        _RESOLUTION,
        _ORIGIN,
        bytes(cells),
        waypoint_writes=True,
    )


def map_to_odom(truth: Pose2D, wheel: Pose2D) -> Pose2D:
    yaw = math.remainder(truth.yaw - wheel.yaw, 2 * math.pi)
    c, s = math.cos(yaw), math.sin(yaw)
    return Pose2D(truth.x - c * wheel.x + s * wheel.y, truth.y - s * wheel.x - c * wheel.y, yaw)
