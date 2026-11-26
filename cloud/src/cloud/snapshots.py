"""Cloud-owned cache of shared snapshot payloads; never a vehicle state owner."""

import time
from collections.abc import Callable
from dataclasses import dataclass

from senior_contracts.wire import SnapshotEnvelope, SnapshotPayload


def snapshot_envelope(
    snapshot: SnapshotPayload,
    *,
    robot_id: str,
    session_id: str,
    connection_epoch: int,
    request_id: str,
    seq: int,
    expires_at: float,
    stale: bool = False,
    last_synced_at: float | None = None,
) -> SnapshotEnvelope:
    """Project the cloud copy using the original visibility and omission rules."""
    payload = snapshot.model_copy(deep=True)
    payload.stale = stale
    payload.last_synced_at = last_synced_at
    if stale:
        payload.route_progress = None
    if stale or not payload.connected or not payload.localized or payload.map_generation is None:
        payload.robot_pose = None
        payload.scan_points = []
        if payload.scan_layer is not None:
            payload.scan_layer = payload.scan_layer.model_copy(
                update=dict(valid=False, remaining_s=0.0, preview_count=0)
            )
    if "navigation_ready" in snapshot.model_fields_set:
        payload.navigation_ready = snapshot.navigation_ready and snapshot.connected and not stale
    return SnapshotEnvelope(
        v=1,
        robot_id=robot_id,
        session_id=session_id,
        connection_epoch=connection_epoch,
        request_id=request_id,
        seq=seq,
        expires_at=expires_at,
        type="snapshot",
        payload=payload,
    )


@dataclass(frozen=True)
class CloudSnapshot:
    snapshot: SnapshotPayload
    last_synced_at: float
    stale: bool


class SnapshotCache:
    """Owned by the relay; offline and elapsed staleness never grant readiness."""

    def __init__(
        self,
        *,
        stale_after: float = 5.0,
        clock: Callable[[], float] = time.time,
        monotonic: Callable[[], float] = time.monotonic,
    ) -> None:
        if stale_after <= 0:
            raise ValueError("stale_after must be positive")
        self._stale_after = stale_after
        self._clock = clock
        self._monotonic = monotonic
        self._current_reference: dict[str, tuple] = {}
        self._scan_scope: dict[str, tuple] = {}
        self._scan_sequence: dict[str, int] = {}
        self._scan_deadline: dict[str, float] = {}
        self._retired_references: dict[str, set[tuple]] = {}
        self._retired_sources: dict[str, set[tuple]] = {}
        self._retired_boots: dict[str, set[str]] = {}
        self._entries: dict[str, tuple[SnapshotPayload, float]] = {}
        self._offline: set[str] = set()

    def store(self, envelope: SnapshotEnvelope) -> CloudSnapshot:
        robot_id = envelope.robot_id
        payload = envelope.payload
        # //Compatible: the former vehicle projection rejected blank map identities.
        references = (payload.map_ref, None if payload.home is None else payload.home.map_ref)
        for reference in references:
            if reference is not None and not reference.map_id.strip():
                raise ValueError("invalid map reference")
        stored = payload.model_copy(deep=True)
        self._accept_scan(robot_id, stored)
        self._entries[robot_id] = (stored, self._clock())
        self._offline.discard(robot_id)
        view = self._view(robot_id)
        assert view is not None
        return view

    def _accept_scan(self, robot_id: str, payload: SnapshotPayload) -> None:
        now = self._monotonic()
        layer = payload.scan_layer
        retired_refs = self._retired_references.setdefault(robot_id, set())
        if payload.map_ref is not None and payload.map_generation is not None:
            observed_ref = (
                payload.map_ref.map_id,
                payload.map_ref.reference_revision,
                payload.map_generation,
            )
            if observed_ref not in retired_refs:
                previous_ref = self._current_reference.get(robot_id)
                if previous_ref is not None and previous_ref != observed_ref:
                    retired_refs.add(previous_ref)
                self._current_reference[robot_id] = observed_ref
        if layer is None:
            # A missing/invalid observation revokes the current local display lease.
            self._scan_deadline[robot_id] = now
            return
        ref = (layer.map_ref.map_id, layer.map_ref.reference_revision, layer.map_generation)
        source = (layer.backend_boot, layer.source_id)
        scope = (ref, source)
        previous = self._scan_scope.get(robot_id)
        retired_sources = self._retired_sources.setdefault(robot_id, set())
        retired_boots = self._retired_boots.setdefault(robot_id, set())
        rejected = (
            ref in retired_refs or source in retired_sources or layer.backend_boot in retired_boots
        )
        if previous == scope and layer.sequence < self._scan_sequence[robot_id]:
            rejected = True
        if payload.map_ref != layer.map_ref or payload.map_generation != layer.map_generation:
            rejected = True
        if rejected:
            payload.scan_points = []
            payload.scan_layer = layer.model_copy(
                update=dict(valid=False, remaining_s=0.0, preview_count=0)
            )
            return
        deadline = now + layer.remaining_s if layer.valid else now
        if previous == scope and layer.sequence == self._scan_sequence[robot_id]:
            deadline = min(deadline, self._scan_deadline[robot_id])
        else:
            if previous is not None:
                if previous[0] != ref:
                    retired_refs.add(previous[0])
                if previous[1] != source:
                    retired_sources.add(previous[1])
                if previous[1][0] != layer.backend_boot:
                    retired_boots.add(previous[1][0])
            self._scan_scope[robot_id] = scope
            self._scan_sequence[robot_id] = layer.sequence
        if not payload.connected or not payload.localized or payload.stale:
            deadline = now
        self._scan_deadline[robot_id] = deadline

    def mark_offline(self, robot_id: str) -> None:
        self._offline.add(robot_id)
        self._scan_deadline[robot_id] = self._monotonic()

    def get(self, robot_id: str) -> CloudSnapshot | None:
        return self._view(robot_id)

    def _view(self, robot_id: str) -> CloudSnapshot | None:
        entry = self._entries.get(robot_id)
        if entry is None:
            return None
        snapshot, synced = entry
        stale = robot_id in self._offline or self._clock() - synced > self._stale_after
        shown = snapshot.model_copy(deep=True)
        shown.connected = snapshot.connected and not stale
        if stale:
            shown.robot_pose = None
            shown.scan_points = []
        layer = shown.scan_layer
        if layer is not None:
            remaining = max(
                0.0,
                min(layer.remaining_s, self._scan_deadline.get(robot_id, 0.0) - self._monotonic()),
            )
            valid = (
                layer.valid and not stale and shown.connected and shown.localized and remaining > 0
            )
            if not valid:
                shown.scan_points = []
            shown.scan_layer = layer.model_copy(
                update=dict(
                    remaining_s=remaining if valid else 0.0,
                    valid=valid,
                    preview_count=len(shown.scan_points),
                )
            )
        if "navigation_ready" in snapshot.model_fields_set:
            shown.navigation_ready = snapshot.navigation_ready and not stale
        return CloudSnapshot(shown, synced, stale)
