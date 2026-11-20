"""//Compatible: original values and store construction over the manager SQL owner."""

import math
from dataclasses import dataclass

import aiosqlite
from vehicle_manager.waypoints.ports import WaypointStorePorts
from vehicle_manager.waypoints.store import WAYPOINT_STATEMENTS as WAYPOINT_STATEMENTS
from vehicle_manager.waypoints.store import WaypointStore as _WaypointStore
from vehicle_manager.waypoints.store import (
    adopted_home_blocks_foreign as _adopted_home_blocks_foreign,
)
from vehicle_manager.waypoints.store import replace_imported_home as replace_imported_home

from senior_control.storage.database import Database
from vehicle_core.errors import DomainError
from vehicle_core.values import MapRef, Pose2D


@dataclass(frozen=True)
class Waypoint:
    id: str
    kind: str
    pose: Pose2D
    map_ref: MapRef
    anchor_id: str
    source_robot_id: str
    status: str
    latitude: float | None = None
    longitude: float | None = None

    def __post_init__(self) -> None:
        if self.kind not in {"home", "waypoint"}:
            raise ValueError("waypoint kind is invalid")
        if not self.id.strip() or not self.anchor_id.strip() or not self.source_robot_id.strip():
            raise ValueError("waypoint identity is required")
        if not self.status.strip():
            raise ValueError("waypoint status is required")
        if (self.latitude is None) != (self.longitude is None):
            raise ValueError("gps display needs both coordinates")
        if self.latitude is not None and self.longitude is not None:
            if not math.isfinite(self.latitude) or not math.isfinite(self.longitude):
                raise ValueError("gps display must be finite")


@dataclass(frozen=True)
class StoredWaypoint:
    waypoint: Waypoint
    anchor_pose: Pose2D
    adopted: bool | None
    link_status: str | None


class WaypointStore(_WaypointStore[Waypoint, StoredWaypoint, Pose2D]):
    """//Compatible: original signature and concrete values; inherited SQL/transactions."""

    def __init__(self, database: Database, *, robot_id: str) -> None:
        async def prepare_schema() -> None:
            from senior_control.storage.migrations import migrate
            from senior_control.storage.schema import DOMAIN_MIGRATIONS

            await migrate(database, list(DOMAIN_MIGRATIONS))

        super().__init__(
            database,
            robot_id=robot_id,
            ports=WaypointStorePorts(
                prepare_schema=prepare_schema,
                map_reference=MapRef,
                pose=Pose2D,
                waypoint=Waypoint,
                stored=StoredWaypoint,
                error=DomainError,
            ),
        )


async def adopted_home_blocks_foreign(connection: aiosqlite.Connection, robot_id: str) -> bool:
    """//Compatible: the same map transaction, with its original validation exception."""
    return await _adopted_home_blocks_foreign(connection, robot_id, error=DomainError)
