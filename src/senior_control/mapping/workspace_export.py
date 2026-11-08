"""Load an explicitly pinned local archive as a read-only workspace raster."""

import hashlib
import io
import math
from pathlib import Path

from senior_control.errors import DomainError, ErrorCode
from senior_control.mapping.archive import MAX_BUNDLE_BYTES, STABLE_LAYER, MapArchive
from senior_control.mapping.references import SlamSnapshot
from senior_control.mapping.workspace import MAX_GRID_CELLS, MAX_GRID_SIDE, WorkspaceGrid
from senior_control.models import Pose2D


def load_exported_grid(path: Path, digest: str, robot_id: str) -> WorkspaceGrid:
    with path.open("rb") as stream:
        archive = stream.read(MAX_BUNDLE_BYTES + 1)
    if len(archive) > MAX_BUNDLE_BYTES or hashlib.sha256(archive).hexdigest() != digest:
        raise DomainError(ErrorCode.VALIDATION, "workspace bundle hash or size is invalid")
    try:
        manifest, files = MapArchive.inspect_bundle(io.BytesIO(archive))
    except (ValueError, OverflowError, RecursionError) as exc:
        raise DomainError(ErrorCode.VALIDATION, "workspace bundle is invalid") from exc
    if manifest.source_robot_id != robot_id:
        raise DomainError(ErrorCode.CONFLICT, "workspace bundle belongs to another vehicle")
    try:
        snapshot = SlamSnapshot.from_json(files[STABLE_LAYER])
        if snapshot.map_ref != manifest.map_ref or snapshot.anchors != manifest.anchors:
            raise ValueError("occupancy identity differs from its manifest")
        extent, cell = snapshot.grid, snapshot.cell_size_m
        width, height = extent.width, extent.height
        if (
            not 0 < width <= MAX_GRID_SIDE
            or not 0 < height <= MAX_GRID_SIDE
            or width * height > MAX_GRID_CELLS
            or not math.isfinite(cell)
            or cell <= 0
        ):
            raise ValueError("invalid exported grid geometry")
        origin = Pose2D(extent.origin_x, extent.origin_y, 0.0)
        x0, y0 = math.floor(origin.x / cell), math.floor(origin.y / cell)
        pixels = bytearray(b"\xff") * (width * height)
        for evidence in snapshot.occupancy.cells:
            x, y = evidence.cell[0] - x0, evidence.cell[1] - y0
            if 0 <= x < width and 0 <= y < height:
                pixels[y * width + x] = 100
        return WorkspaceGrid(
            manifest.map_ref, digest, width, height, cell, origin, bytes(pixels), display_only=True
        )
    except (ValueError, OverflowError, RecursionError) as exc:
        raise DomainError(ErrorCode.VALIDATION, "workspace occupancy is invalid") from exc
