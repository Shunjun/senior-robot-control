"""Fresh in-process map session with a conservative, read-only raster view.

This is not a ROS source verifier, transport, or an old-map activation path.
Only the upstream owner that has already verified its source may bind it here.
Production transport must provide that verification and an opaque session token;
backend boot/source strings alone never confer map or localization authority.
All methods run synchronously on the owning loop, without concurrent writers.
"""

import uuid
from dataclasses import replace

from senior_mapping.references import AnchorSnapshot, CellBinding
from senior_mapping.store import (
    MapReferenceStore,
    OccupancyIngestBinding,
    OccupancyIngestProgress,
)
from senior_mapping.workspace import WorkspaceGrid, render_stable_grid
from vehicle_core.errors import DomainError, ErrorCode
from vehicle_core.values import MapRef


class ActiveMap:
    """Own one fresh store; a terminal session must be replaced, never rebound.

    A caller can expose ``WorkspaceMap(session.grid)`` without gaining store
    mutation access. The view contains stable occupied cells and unknown only;
    it grants no navigation, waypoint writes, persistence, or trusted pose.
    """

    def __init__(self, map_ref: MapRef, odom_owner: str, cell_size_m: float) -> None:
        self._store = MapReferenceStore(map_ref, odom_owner, cell_size_m)
        self._session_id = uuid.uuid4().hex
        self._source: object | None = None
        self._binding: OccupancyIngestBinding | None = None
        self._closed = False
        self._grid: WorkspaceGrid | None = None

    @property
    def map_generation(self) -> str:
        """Reference/localization nonce, independent of raster data IDs."""
        return self._store.map_generation

    def bind_verified_source(self, source: object, *, backend_boot: str, source_epoch: str) -> None:
        """Accept a verifier-owned object by identity, not by serialized fields.

        Calling this method is the in-process trust boundary, not proof that
        verification happened. Do not call it on receipt of an arbitrary /map,
        graph boot, or source name. There is no production verifier in this slice.
        """
        if self._closed or self._binding is not None:
            raise DomainError(ErrorCode.CONFLICT, "active map source cannot be rebound")
        if source is None or isinstance(source, (str, bytes, int, float, bool)):
            raise DomainError(ErrorCode.VALIDATION, "active map source requires an opaque token")
        binding = self._store.bind_ingest(
            self._store.map_ref, backend_boot=backend_boot, source_epoch=source_epoch
        )
        self._source, self._binding = source, binding

    def commit(
        self,
        source: object,
        sequence: int,
        keyframe_id: str,
        anchors: AnchorSnapshot,
        changes: tuple[CellBinding, ...],
    ) -> OccupancyIngestProgress:
        """Commit an already qualified observation, then publish one complete view."""
        binding = self._require_source(source)
        try:
            progress = self._store.commit_ingest(binding, sequence, keyframe_id, anchors, changes)
        except Exception:
            # Duplicates leave the original progress and published cut intact.
            # Gaps or malformed batches fence the underlying store permanently.
            if self._store.ingest_progress is None:
                self._close(fenced=True)
            raise
        try:
            candidate = render_stable_grid(
                self._store.map_ref,
                f"{self._session_id}:{sequence}",
                self._store.grid,
                self._store.cell_size_m,
                self._store.occupancy(),
            )
        except (ValueError, OverflowError) as exc:
            self._close()
            raise DomainError(ErrorCode.VALIDATION, "active map raster is invalid") from exc
        current = self._grid
        if current is None or replace(candidate, grid_id=current.grid_id) != current:
            self._grid = candidate
        return progress

    def grid(self) -> WorkspaceGrid | None:
        """Narrow provider for WorkspaceMap; revoked views immediately disappear."""
        if self._closed or self._store.review_required or self._store.ingest_progress is None:
            return None
        return self._grid

    def revoke(self, source: object) -> None:
        """Retire this view before the owning caller switches to another session."""
        self._require_source(source)
        self._close()

    def _require_source(self, source: object) -> OccupancyIngestBinding:
        if self._closed or self._binding is None or source is not self._source:
            raise DomainError(ErrorCode.CONFLICT, "active map source is stale or unbound")
        return self._binding

    def _close(self, *, fenced: bool = False) -> None:
        if self._binding is not None and not fenced:
            self._store.revoke_ingest(self._binding)
        self._closed = True
        self._source = self._binding = self._grid = None
