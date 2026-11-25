"""Verified in-process map session with a conservative, read-only raster view.

This does not verify ROS sources, load a graph, or localize a vehicle.
Only the upstream owner that has already verified its source may bind it here.
Production transport must provide that verification and an opaque session token;
backend boot/source strings alone never confer map or localization authority.
All methods run synchronously on the owning loop, without concurrent writers.
"""

from __future__ import annotations

import time
import uuid
from collections.abc import Callable
from dataclasses import replace
from typing import TYPE_CHECKING

from senior_mapping.evidence import (
    FixedDelta,
    FixedWatermark,
    ProfileProvenance,
    SourceIdentity,
    VerifiedBasis,
)
from senior_mapping.references import (
    AnchorSnapshot,
    CellBinding,
    FixedSnapshot,
    LegacyBase,
    SlamSnapshot,
    WaypointBinding,
)
from senior_mapping.shared_scan import QualifiedScan, ScanKey
from senior_mapping.store import (
    MapReferenceStore,
    OccupancyIngestBinding,
    OccupancyIngestProgress,
)
from senior_mapping.workspace import WorkspaceGrid, render_stable_grid
from vehicle_core.errors import DomainError, ErrorCode
from vehicle_core.values import MapRef

if TYPE_CHECKING:
    from senior_mapping.catalog import CatalogEntry
    from senior_mapping.checkpoints import MapCheckpoints


class ActiveMap:
    """Own one store; a terminal session must be replaced, never rebound.

    A caller can expose ``WorkspaceMap(session.grid)`` without gaining store
    mutation access. The view contains stable occupied cells and unknown only;
    it grants no navigation, waypoint writes, or trusted pose. Checkpoint
    delegation retains the archive and catalog guards.
    """

    def __init__(self, map_ref: MapRef, odom_owner: str, cell_size_m: float) -> None:
        self._store = MapReferenceStore(map_ref, odom_owner, cell_size_m)
        self._session_id = uuid.uuid4().hex
        self._source: object | None = None
        self._binding: OccupancyIngestBinding | None = None
        self._closed = False
        self._grid: WorkspaceGrid | None = None
        self._restored_cut = False

    @classmethod
    def resume_verified(
        cls, snapshot: SlamSnapshot, source: object, *, backend_boot: str, source_epoch: str
    ) -> ActiveMap:
        """Use only after the caller verified the loaded graph, source and localization.

        This reconstructs an in-process writer and a display view, not that
        external proof. In particular, a serialized boot/source name is no proof.
        """
        if source is None or isinstance(source, (str, bytes, int, float, bool)):
            raise DomainError(ErrorCode.VALIDATION, "active map source requires an opaque token")
        store, binding = MapReferenceStore.resume_verified(
            snapshot, backend_boot=backend_boot, source_epoch=source_epoch
        )
        active = cls(snapshot.map_ref, snapshot.odom_owner, snapshot.cell_size_m)
        active._store, active._source, active._binding = store, source, binding
        active._grid = render_stable_grid(
            snapshot.map_ref,
            f"{active._session_id}:restored",
            snapshot.grid,
            snapshot.cell_size_m,
            snapshot.occupancy,
        )
        active._restored_cut = True
        return active

    @property
    def change_sequence(self) -> int:
        """Dirty checkpoint watermark, including canonical waypoint binding edits."""
        return self._store.change_sequence

    @property
    def observation_sequence(self) -> int:
        progress = self._store.ingest_progress
        return 0 if progress is None else progress.sequence

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
        if self._store._fixed_cut is not None:
            self._grid = self._store.fixed_snapshot(binding).grid
            return progress
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

    def update_waypoints(
        self,
        source: object,
        *,
        expected_sequence: int,
        changes: tuple[WaypointBinding, ...],
    ) -> None:
        """Canonical waypoint owner updates a binding on its exact current cut."""
        binding = self._require_source(source)
        self._store.commit_waypoints(binding, expected_sequence=expected_sequence, changes=changes)

    def waypoint_bindings(self, source: object) -> tuple[WaypointBinding, ...]:
        self._require_source(source)
        return self._store.waypoint_bindings()

    async def save_checkpoint(
        self,
        source: object,
        repository: MapCheckpoints,
        *,
        home_id: str,
        expected: str | None,
        session_token: str | None = None,
    ) -> CatalogEntry:
        """Production owner serializes this with observation intake/retirement.

        Ordered observations must wait upstream while the archive's graph/store
        capture fence is held. They must never be dropped to make a save succeed.
        """
        self._require_source(source)
        return await repository.save(
            self._store, home_id=home_id, expected=expected, session_token=session_token
        )

    def grid(self) -> WorkspaceGrid | None:
        """Narrow provider for WorkspaceMap; revoked views immediately disappear."""
        if (
            self._closed
            or self._store.review_required
            or self._store.ingest_progress is None
            and not self._restored_cut
        ):
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
        self._restored_cut = False
        self._source = self._binding = self._grid = None

    def bind_fixed_context(
        self,
        source: object,
        *,
        source_identity: SourceIdentity,
        host_boot_id: str,
        acceptance_floor: ScanKey | None,
        acceptance_floor_steady_ns: int,
        accepting: Callable[[], bool],
        clock: Callable[[], int] = time.monotonic_ns,
        profiles: tuple[ProfileProvenance, ...] = (),
        legacy_base: LegacyBase | None = None,
    ) -> FixedSnapshot:
        """Explicit verifier boundary; production selection waits for T4."""
        cut = self._store.bind_fixed_context(
            self._require_source(source),
            source_identity=source_identity,
            host_boot_id=host_boot_id,
            acceptance_floor=acceptance_floor,
            acceptance_floor_steady_ns=acceptance_floor_steady_ns,
            accepting=accepting,
            clock=clock,
            profiles=profiles,
            legacy_base=legacy_base,
        )
        self._grid = cut.grid
        return cut

    def allocate_support_id(self, source: object) -> str:
        return self._store.allocate_support_id(self._require_source(source))

    def accept_scan(self, source: object, scan: QualifiedScan, basis: VerifiedBasis) -> None:
        self._store.accept_scan(self._require_source(source), scan, basis)

    def retire_fixed(self, source: object) -> None:
        self._store.retire_fixed(self._require_source(source))

    def fixed_snapshot(self, source: object) -> FixedSnapshot:
        return self._store.fixed_snapshot(self._require_source(source))

    def apply_evidence(
        self,
        source: object,
        *,
        expected_graph_sequence: int,
        expected_anchor_digest: str,
        delta: FixedDelta,
    ) -> FixedSnapshot:
        cut = self._store.commit_evidence(
            self._require_source(source),
            expected_graph_sequence=expected_graph_sequence,
            expected_anchor_digest=expected_anchor_digest,
            delta=delta,
        )
        self._grid = cut.grid
        return cut

    def ensure_observed_extent(
        self, source: object, *, expected: FixedWatermark, bounds: tuple[float, float, float, float]
    ) -> FixedSnapshot:
        cut = self._store.ensure_observed_extent(
            self._require_source(source), expected=expected, bounds=bounds
        )
        self._grid = cut.grid
        return cut
