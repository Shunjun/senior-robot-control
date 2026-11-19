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

    def __init__(self, *, stale_after: float = 5.0, clock: Callable[[], float] = time.time) -> None:
        if stale_after <= 0:
            raise ValueError("stale_after must be positive")
        self._stale_after = stale_after
        self._clock = clock
        self._entries: dict[str, tuple[SnapshotPayload, float]] = {}
        self._offline: set[str] = set()

    def store(self, envelope: SnapshotEnvelope) -> CloudSnapshot:
        robot_id = envelope.robot_id
        self._entries[robot_id] = (envelope.payload.model_copy(deep=True), self._clock())
        self._offline.discard(robot_id)
        view = self._view(robot_id)
        assert view is not None
        return view

    def mark_offline(self, robot_id: str) -> None:
        self._offline.add(robot_id)

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
        if "navigation_ready" in snapshot.model_fields_set:
            shown.navigation_ready = snapshot.navigation_ready and not stale
        return CloudSnapshot(shown, synced, stale)
