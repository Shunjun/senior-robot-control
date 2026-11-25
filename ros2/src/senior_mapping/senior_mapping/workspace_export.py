"""Load an explicitly pinned local archive as a read-only workspace raster."""

import hashlib
import io
from pathlib import Path

from senior_mapping.archive import MAX_BUNDLE_BYTES, STABLE_LAYER, MapArchive
from senior_mapping.references import load_snapshot
from senior_mapping.workspace import WorkspaceGrid, render_stable_grid
from vehicle_core.errors import DomainError, ErrorCode


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
        snapshot = load_snapshot(files[STABLE_LAYER])
        if snapshot.map_ref != manifest.map_ref or snapshot.anchors != manifest.anchors:
            raise ValueError("occupancy identity differs from its manifest")
        if snapshot.fixed is not None:
            return snapshot.fixed.grid
        return render_stable_grid(
            manifest.map_ref, digest, snapshot.grid, snapshot.cell_size_m, snapshot.occupancy
        )
    except (ValueError, OverflowError, RecursionError) as exc:
        raise DomainError(ErrorCode.VALIDATION, "workspace occupancy is invalid") from exc
