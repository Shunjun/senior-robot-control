"""Management-owned schema and Home integration on the original map connection."""

from typing import TYPE_CHECKING

import aiosqlite
from senior_mapping.persistence import MapPersistence

from vehicle_core.values import MapRef, Pose2D

if TYPE_CHECKING:
    from vehicle_manager.storage.bindings import Database


class _WaypointHomeImport:
    async def adopted_home_blocks_foreign(
        self, connection: aiosqlite.Connection, robot_id: str
    ) -> bool:
        from vehicle_manager.storage.domain_bindings import adopted_home_blocks_foreign

        return await adopted_home_blocks_foreign(connection, robot_id)

    async def replace_imported_home(
        self,
        connection: aiosqlite.Connection,
        *,
        robot_id: str,
        waypoint_id: str,
        pose: Pose2D,
        map_ref: MapRef,
        anchor_id: str,
        anchor_pose: Pose2D,
        source_robot_id: str,
        status: str,
    ) -> None:
        from vehicle_manager.waypoints.store import replace_imported_home

        await replace_imported_home(
            connection,
            robot_id=robot_id,
            waypoint_id=waypoint_id,
            pose=pose,
            map_ref=map_ref,
            anchor_id=anchor_id,
            anchor_pose=anchor_pose,
            source_robot_id=source_robot_id,
            status=status,
        )


def map_persistence(database: "Database") -> MapPersistence:
    """Create effects only; schema work still starts after the map's motion stop."""

    async def prepare_schema() -> None:
        from vehicle_manager.storage.core_schema import migrate_core
        from vehicle_manager.storage.domain_bindings import DOMAIN_MIGRATIONS
        from vehicle_manager.storage.migrations import migrate

        await migrate(database, list(DOMAIN_MIGRATIONS))
        await migrate_core(database)

    return MapPersistence(prepare_schema=prepare_schema, home=_WaypointHomeImport())
