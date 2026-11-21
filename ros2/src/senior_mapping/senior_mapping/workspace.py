"""Bounded map reads from an authoritative, immutable occupancy grid."""

import math
from collections.abc import Callable, Mapping
from dataclasses import asdict, dataclass
from typing import Any

from senior_mapping.evidence import OccupancySnapshot
from senior_mapping.references import GridExtent
from vehicle_core.values import MapRef, Pose2D

# //Compatible: all map consumers share the same original limit values.
from vehicle_core.workspace_limits import MAX_GRID_CELLS as MAX_GRID_CELLS
from vehicle_core.workspace_limits import MAX_GRID_SIDE as MAX_GRID_SIDE
from vehicle_core.workspace_limits import TILE_SIZE as TILE_SIZE


@dataclass(frozen=True)
class WorkspaceGrid:
    """One map raster cut. Its id changes whenever its reference, geometry or cells change."""

    map_ref: MapRef
    grid_id: str
    width: int
    height: int
    resolution: float
    origin: Pose2D
    # Dense archives use bytes (255 encodes unknown) to bound memory use.
    cells: tuple[int, ...] | bytes
    display_only: bool = False
    waypoint_writes: bool = False

    def __post_init__(self) -> None:
        size = self.width * self.height
        if (
            not self.grid_id.strip()
            or len(self.grid_id) > 128
            or not 0 < self.width <= MAX_GRID_SIDE
            or not 0 < self.height <= MAX_GRID_SIDE
            or size > MAX_GRID_CELLS
            or len(self.cells) != size
            or not math.isfinite(self.resolution)
            or self.resolution <= 0
            or (
                any(cell > 100 and cell != 255 for cell in self.cells)
                if isinstance(self.cells, bytes)
                else any(type(cell) is not int or not -1 <= cell <= 100 for cell in self.cells)
            )
        ):
            raise ValueError("invalid occupancy grid")


def render_stable_grid(
    map_ref: MapRef,
    grid_id: str,
    extent: GridExtent,
    cell_size_m: float,
    occupancy: OccupancySnapshot,
) -> WorkspaceGrid:
    """Render confirmed occupancy only; absence never proves free space.

    Geometry is bounded before allocating the dense raster. This is a display
    projection, not a localization result or a navigation permission.
    """
    width, height = extent.width, extent.height
    if (
        type(width) is not int
        or type(height) is not int
        or not 0 < width <= MAX_GRID_SIDE
        or not 0 < height <= MAX_GRID_SIDE
        or width * height > MAX_GRID_CELLS
        or not math.isfinite(cell_size_m)
        or cell_size_m <= 0
    ):
        raise ValueError("invalid exported grid geometry")
    origin = Pose2D(extent.origin_x, extent.origin_y, 0.0)
    x0, y0 = math.floor(origin.x / cell_size_m), math.floor(origin.y / cell_size_m)
    pixels = bytearray(b"\xff") * (width * height)
    for evidence in occupancy.cells:
        x, y = evidence.cell[0] - x0, evidence.cell[1] - y0
        if 0 <= x < width and 0 <= y < height:
            pixels[y * width + x] = 100
    return WorkspaceGrid(
        map_ref, grid_id, width, height, cell_size_m, origin, bytes(pixels), display_only=True
    )


class WorkspaceMap:
    """Serve a trusted map cut. Never infer a reference from an untagged ROS topic."""

    def __init__(self, active_grid: Callable[[], WorkspaceGrid | None]) -> None:
        self._active_grid = active_grid

    def read(self, kind: str, fields: Mapping[str, Any]) -> dict[str, Any]:
        grid = self._active_grid()
        if grid is None:
            return {"ok": False, "error": "unavailable"}
        if kind == "map.active":
            return {"ok": True, "map_ref": asdict(grid.map_ref), "grid_id": grid.grid_id}
        if (
            fields.get("map_id") != grid.map_ref.map_id
            or type(fields.get("reference_revision")) is not int
            or fields["reference_revision"] != grid.map_ref.reference_revision
        ):
            return {"ok": False, "error": "conflict"}
        identity = {"ok": True, "map_ref": asdict(grid.map_ref), "grid_id": grid.grid_id}
        if kind == "map.manifest":
            return {
                **identity,
                "width": grid.width,
                "height": grid.height,
                "resolution": grid.resolution,
                "origin": asdict(grid.origin),
                "tile_size": TILE_SIZE,
                **({"display_only": True} if grid.display_only else {}),
                **({"waypoint_writes": True} if grid.waypoint_writes else {}),
            }
        if kind != "map.tile":
            return {"ok": False, "error": "validation"}
        if fields.get("grid_id") != grid.grid_id:
            return {"ok": False, "error": "conflict"}
        tx, ty = fields.get("tile_x"), fields.get("tile_y")
        if (
            type(tx) is not int
            or type(ty) is not int
            or not 0 <= tx < (grid.width + TILE_SIZE - 1) // TILE_SIZE
            or not 0 <= ty < (grid.height + TILE_SIZE - 1) // TILE_SIZE
        ):
            return {"ok": False, "error": "validation"}
        x, y = tx * TILE_SIZE, ty * TILE_SIZE
        width, height = min(TILE_SIZE, grid.width - x), min(TILE_SIZE, grid.height - y)
        cells = [
            -1 if cell == 255 else cell
            for row in range(y, y + height)
            for cell in grid.cells[row * grid.width + x : row * grid.width + x + width]
        ]
        return {
            **identity,
            "tile_x": tx,
            "tile_y": ty,
            "width": width,
            "height": height,
            "cells": cells,
        }
