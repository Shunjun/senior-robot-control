"""Live robot state, and the cloud's cache of it. A cached snapshot is never live state."""

import time
from collections.abc import Callable
from dataclasses import dataclass, replace
from typing import Protocol

from senior_control.models import MapRef
from senior_control.protocol import (
    HomePayload,
    MapRefPayload,
    SnapshotEnvelope,
    SnapshotPayload,
)
from senior_control.waypoints import Waypoint


@dataclass(frozen=True)
class LiveState:
    """What the robot's own stores and sensors say right now."""

    localized: bool
    map_ref: MapRef | None
    home: Waypoint | None
    task_revision: int
    execution_status: str
    runnable_task_ids: tuple[str, ...] = ()


class StatePort(Protocol):
    async def collect(self) -> LiveState: ...


@dataclass(frozen=True)
class HomeSummary:
    waypoint_id: str
    map_ref: MapRef
    status: str


@dataclass(frozen=True)
class RobotSnapshot:
    robot_id: str
    connected: bool
    localized: bool
    map_ref: MapRef | None
    home: HomeSummary | None
    task_revision: int
    execution_status: str
    collected_at: float
    runnable_task_ids: tuple[str, ...] = ()


def _map_payload(map_ref: MapRef | None) -> MapRefPayload | None:
    if map_ref is None:
        return None
    return MapRefPayload(map_id=map_ref.map_id, reference_revision=map_ref.reference_revision)


def snapshot_envelope(
    snapshot: RobotSnapshot,
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
    """The uplink frame a ``Link`` sends for a snapshot.

    ``stale`` and ``last_synced_at`` belong to the cloud's cached copy; an agent leaves them out.
    """
    home = snapshot.home
    return SnapshotEnvelope(
        v=1,
        robot_id=robot_id,
        session_id=session_id,
        connection_epoch=connection_epoch,
        request_id=request_id,
        seq=seq,
        expires_at=expires_at,
        type="snapshot",
        payload=SnapshotPayload(
            connected=snapshot.connected,
            localized=snapshot.localized,
            map_ref=_map_payload(snapshot.map_ref),
            home=None
            if home is None
            else HomePayload(
                waypoint_id=home.waypoint_id,
                map_ref=MapRefPayload(
                    map_id=home.map_ref.map_id,
                    reference_revision=home.map_ref.reference_revision,
                ),
                status=home.status,
            ),
            task_revision=snapshot.task_revision,
            runnable_task_ids=list(snapshot.runnable_task_ids),
            execution_status=snapshot.execution_status,
            collected_at=snapshot.collected_at,
            stale=stale,
            last_synced_at=last_synced_at,
        ),
    )


def snapshot_from_envelope(envelope: SnapshotEnvelope) -> RobotSnapshot:
    payload = envelope.payload
    home = payload.home
    return RobotSnapshot(
        robot_id=envelope.robot_id,
        connected=payload.connected,
        localized=payload.localized,
        map_ref=None
        if payload.map_ref is None
        else MapRef(payload.map_ref.map_id, payload.map_ref.reference_revision),
        home=None
        if home is None
        else HomeSummary(
            home.waypoint_id,
            MapRef(home.map_ref.map_id, home.map_ref.reference_revision),
            home.status,
        ),
        task_revision=payload.task_revision,
        execution_status=payload.execution_status,
        collected_at=payload.collected_at,
        runnable_task_ids=tuple(payload.runnable_task_ids),
    )


@dataclass(frozen=True)
class CloudSnapshot:
    snapshot: RobotSnapshot
    last_synced_at: float
    stale: bool


class SnapshotCache:
    """Read-only cloud copy. It goes stale when the robot is offline or has been quiet."""

    def __init__(self, *, stale_after: float = 5.0, clock: Callable[[], float] = time.time) -> None:
        if stale_after <= 0:
            raise ValueError("stale_after must be positive")
        self._stale_after = stale_after
        self._clock = clock
        self._entries: dict[str, tuple[RobotSnapshot, float]] = {}
        self._offline: set[str] = set()

    def store(self, snapshot: RobotSnapshot) -> CloudSnapshot:
        self._entries[snapshot.robot_id] = (snapshot, self._clock())
        self._offline.discard(snapshot.robot_id)
        view = self._view(snapshot.robot_id)
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
        # The cached copy never claims the robot is connected.
        shown = replace(snapshot, connected=snapshot.connected and not stale)
        return CloudSnapshot(shown, synced, stale)
