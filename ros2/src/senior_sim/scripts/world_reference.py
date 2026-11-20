#!/usr/bin/env python3
"""Derive the isolated baseline occupancy grid from its pinned SDF."""

import hashlib
import math
import xml.etree.ElementTree as ET
from collections.abc import Sequence
from pathlib import Path

from senior_mapping.workspace import WorkspaceGrid

from vehicle_core.values import MapRef, Pose2D

_BASELINE_HASH = "528523ece69d7eaf1eeb2d9fd2930a024ac56a151f62ffa939bd63c338db1c44"
_QUALIFICATION_HASH = "dc9869da0f8328cfac25cae53e18e7cb5ccb56b317f3da840f07e0f572569759"
_RESOLUTION = 0.05
_SIDE = 160
_ORIGIN = Pose2D(-4.0, -4.0, 0.0)


def baseline_grid(path: Path, generation: str) -> WorkspaceGrid:
    """Rasterize the collision box in the pinned SDF, in Gazebo world coordinates."""
    with path.open("rb") as stream:
        data = stream.read(65_537)
    digest = hashlib.sha256(data).hexdigest()
    if digest not in {_BASELINE_HASH, _QUALIFICATION_HASH}:
        raise ValueError("unsupported Gazebo baseline world")
    world = ET.fromstring(data)
    cells = bytearray(_SIDE**2)
    for model in world.findall(".//model"):
        collision = model.findtext("link/collision/geometry/box/size")
        if collision is None:
            continue
        x, y, _, _, _, yaw = map(float, (model.findtext("pose") or "").split())
        width, height, _ = map(float, collision.split())
        # Cover every occupied cell conservatively for the rotated feature.
        c, s = abs(math.cos(yaw)), abs(math.sin(yaw))
        extent_x, extent_y = (c * width + s * height) / 2, (s * width + c * height) / 2
        for row in range(_SIDE):
            lo_y = _ORIGIN.y + row * _RESOLUTION
            if lo_y + _RESOLUTION <= y - extent_y or lo_y >= y + extent_y:
                continue
            for column in range(_SIDE):
                lo_x = _ORIGIN.x + column * _RESOLUTION
                if lo_x + _RESOLUTION > x - extent_x and lo_x < x + extent_x:
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


def project_scan(
    robot: Pose2D,
    sensor: Pose2D,
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
) -> list[dict[str, float]]:
    """Display-only hit points, sampled to keep state IPC well below 8 KiB."""
    points = []
    c, s = math.cos(robot.yaw), math.sin(robot.yaw)
    x = robot.x + c * sensor.x - s * sensor.y
    y = robot.y + s * sensor.x + c * sensor.y
    stride = max(1, math.ceil(len(ranges) / 64))
    for index in range(0, len(ranges), stride):
        distance = ranges[index]
        if not math.isfinite(distance) or not range_min <= distance < range_max:
            continue
        angle = robot.yaw + sensor.yaw + angle_min + index * angle_increment
        points.append(
            {
                "x": round(x + distance * math.cos(angle), 3),
                "y": round(y + distance * math.sin(angle), 3),
            }
        )
    return points


def generate_reference(world: Path, output: Path) -> Path:
    """Write pinned map assets for stock map_server/AMCL and the common facade."""
    import json
    from dataclasses import asdict

    output.mkdir(mode=0o700, parents=True, exist_ok=True)
    grid = baseline_grid(world, "environment-map-v1")
    values = asdict(grid)
    values["cells"] = list(grid.cells)
    grid_path = (output / "grid.json").resolve()
    grid_path.write_text(json.dumps(values) + "\n")
    # PGM origin is bottom left; image rows run from top to bottom.
    pixels = bytes(
        0 if grid.cells[row * grid.width + col] == 100 else 254
        for row in reversed(range(grid.height))
        for col in range(grid.width)
    )
    (output / "map.pgm").write_bytes(f"P5\n{grid.width} {grid.height}\n255\n".encode() + pixels)
    (output / "map.yaml").write_text(
        f"image: map.pgm\nresolution: {grid.resolution}\n"
        f"origin: [{grid.origin.x}, {grid.origin.y}, {grid.origin.yaw}]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n"
    )
    template = Path(__file__).resolve().parents[1] / "config/navigation_reference.json"
    if not template.is_file():
        from ament_index_python.packages import get_package_share_directory

        template = (
            Path(get_package_share_directory("senior_sim")) / "config/navigation_reference.json"
        )
    reference = json.loads(template.read_text())
    reference.update(
        grid_path=str(grid_path), grid_sha256=hashlib.sha256(grid_path.read_bytes()).hexdigest()
    )
    path = output / "navigation_reference.json"
    path.write_text(json.dumps(reference, indent=2) + "\n")
    return path


def main() -> None:
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    generate_reference(args.world, args.output)


if __name__ == "__main__":
    main()
