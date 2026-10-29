"""Live robot state, and the cloud's cache of it. A cached snapshot is never live state."""

import time
from collections.abc import Callable
from dataclasses import dataclass, replace
from typing import Any, Protocol

from senior_control.models import MapRef
from senior_control.waypoints import Waypoint


@dataclass(frozen=True)
class LiveState:
    """What the robot's own stores and sensors say right now."""

    localized: bool
    map_ref: MapRef | None
    home: Waypoint | None
    task_revision: int
    execution_status: str


class StatePort(Protocol):
    async def collect(self) -> LiveState: ...


@dataclass(frozen=True)
class RobotSnapshot:
    robot_id: str
    connected: bool
    localized: bool
    map_ref: MapRef | None
    home: Waypoint | None
    task_revision: int
    execution_status: str
    collected_at: float

    def to_wire(self) -> dict[str, Any]:
        return {
            "robot_id": self.robot_id,
            "connected": self.connected,
            "localized": self.localized,
            "map_ref": None
            if self.map_ref is None
            else {
                "map_id": self.map_ref.map_id,
                "reference_revision": self.map_ref.reference_revision,
            },
            "home": None
            if self.home is None
            else {
                "id": self.home.id,
                "map_id": self.home.map_ref.map_id,
                "status": self.home.status,
            },
            "task_revision": self.task_revision,
            "execution_status": self.execution_status,
            "collected_at": self.collected_at,
        }


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
