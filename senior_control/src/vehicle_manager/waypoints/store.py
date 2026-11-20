"""Original waypoint SQL and transactions with caller-owned value constructors."""

import sqlite3
import uuid
from collections.abc import AsyncIterator, Callable
from contextlib import asynccontextmanager
from datetime import datetime, timezone
from typing import Generic

import aiosqlite
from senior_contracts.errors import ErrorCode
from senior_contracts.navigation import MAX_ROUTE_WAYPOINTS

from vehicle_manager.storage.database import Database
from vehicle_manager.tasks.ports import MapReference, PoseReading
from vehicle_manager.waypoints.codec import _flag, _stored, _text
from vehicle_manager.waypoints.ports import (
    BindingReading,
    PoseT,
    StoredT,
    WaypointStorePorts,
    WaypointT,
)

WAYPOINT_STATEMENTS: tuple[str, ...] = (
    """
    CREATE TABLE waypoint (
        waypoint_id TEXT PRIMARY KEY,
        robot_id TEXT NOT NULL,
        kind TEXT NOT NULL,
        x REAL NOT NULL,
        y REAL NOT NULL,
        yaw REAL NOT NULL,
        map_id TEXT NOT NULL,
        reference_revision INTEGER NOT NULL,
        anchor_id TEXT NOT NULL,
        source_robot_id TEXT NOT NULL,
        status TEXT NOT NULL,
        latitude REAL,
        longitude REAL,
        anchor_x REAL NOT NULL,
        anchor_y REAL NOT NULL,
        anchor_yaw REAL NOT NULL,
        recorded_at TEXT NOT NULL
    )
    """,
    """
    CREATE UNIQUE INDEX waypoint_one_home
    ON waypoint (robot_id)
    WHERE kind = 'home'
    """,
)


class WaypointStore(Generic[WaypointT, StoredT, PoseT]):
    def __init__(
        self,
        database: Database,
        *,
        robot_id: str,
        ports: WaypointStorePorts[WaypointT, StoredT, PoseT],
    ) -> None:
        if not robot_id.strip():
            raise ValueError("robot id is required")
        self._ports = ports
        self._database = database
        self._robot_id = robot_id

    async def prepare(self) -> None:
        await self._ports.prepare_schema()

    async def home(self) -> StoredT | None:
        async with self._database.write_lock:
            connection = self._database.connection
            async with connection.execute(
                """
                SELECT w.waypoint_id, w.kind, w.x, w.y, w.yaw, w.map_id, w.reference_revision,
                       w.anchor_id, w.source_robot_id, w.status, w.latitude, w.longitude,
                       w.anchor_x, w.anchor_y, w.anchor_yaw, l.adopted, l.status
                FROM waypoint AS w
                JOIN map_home_link AS l
                  ON l.robot_id = w.robot_id AND l.waypoint_id = w.waypoint_id
                WHERE w.robot_id = ? AND w.kind = 'home'
                """,
                (self._robot_id,),
            ) as cursor:
                row = await cursor.fetchone()
            await connection.rollback()
        if row is None:
            return None
        return _stored(
            row,
            adopted=_flag(row[15], error=self._ports.error),
            link_status=_text(row[16], error=self._ports.error),
            ports=self._ports,
        )

    async def waypoint(self, waypoint_id: str) -> StoredT | None:
        async with self._database.write_lock:
            connection = self._database.connection
            async with connection.execute(
                """
                SELECT waypoint_id, kind, x, y, yaw, map_id, reference_revision,
                       anchor_id, source_robot_id, status, latitude, longitude,
                       anchor_x, anchor_y, anchor_yaw
                FROM waypoint
                WHERE robot_id = ? AND waypoint_id = ?
                """,
                (self._robot_id, waypoint_id),
            ) as cursor:
                row = await cursor.fetchone()
            await connection.rollback()
        if row is None:
            return None
        return _stored(row, adopted=None, link_status=None, ports=self._ports)

    async def points(self) -> tuple[StoredT, ...]:
        async with self._database.write_lock:
            connection = self._database.connection
            async with connection.execute(
                """
                SELECT waypoint_id, kind, x, y, yaw, map_id, reference_revision,
                       anchor_id, source_robot_id, status, latitude, longitude,
                       anchor_x, anchor_y, anchor_yaw
                FROM waypoint
                WHERE robot_id = ?
                ORDER BY rowid
                """,
                (self._robot_id,),
            ) as cursor:
                rows = await cursor.fetchall()
            await connection.rollback()
        return tuple(
            _stored(row, adopted=None, link_status=None, ports=self._ports) for row in rows
        )

    async def route_poses(
        self, map_ref: MapReference, waypoint_ids: tuple[str, ...]
    ) -> tuple[PoseT, ...]:
        if not 1 <= len(waypoint_ids) <= MAX_ROUTE_WAYPOINTS:
            raise self._ports.error(ErrorCode.VALIDATION, "route waypoint count is invalid")
        placeholders = ", ".join("?" for _ in waypoint_ids)
        async with self._database.write_lock:
            connection = self._database.connection
            async with connection.execute(
                f"""
                SELECT waypoint_id, x, y, yaw FROM waypoint
                WHERE robot_id = ? AND map_id = ? AND reference_revision = ?
                    AND waypoint_id IN ({placeholders}) AND status = 'confirmed'
                    AND (kind = 'waypoint' OR EXISTS (
                        SELECT 1 FROM map_home_link AS link
                        WHERE link.robot_id = waypoint.robot_id
                          AND link.waypoint_id = waypoint.waypoint_id
                          AND link.adopted = 1 AND link.status = 'confirmed'
                    ))
                """,
                (self._robot_id, map_ref.map_id, map_ref.reference_revision, *waypoint_ids),
            ) as cursor:
                rows = await cursor.fetchall()
            await connection.rollback()
        poses = {
            _text(row[0], error=self._ports.error): self._ports.pose(
                float(row[1]), float(row[2]), float(row[3])
            )
            for row in rows
        }
        try:
            return tuple(poses[waypoint_id] for waypoint_id in waypoint_ids)
        except KeyError as exc:
            raise self._ports.error(ErrorCode.CONFLICT, "route waypoint is unavailable") from exc

    async def page(self, map_ref: MapReference, after: str, limit: int) -> tuple[StoredT, ...]:
        async with self._database.write_lock:
            connection = self._database.connection
            async with connection.execute(
                """
                SELECT waypoint_id, kind, x, y, yaw, map_id, reference_revision,
                       anchor_id, source_robot_id, status, latitude, longitude,
                       anchor_x, anchor_y, anchor_yaw
                FROM waypoint
                WHERE robot_id = ? AND map_id = ? AND reference_revision = ?
                    AND waypoint_id > ? AND status = 'confirmed'
                    AND (kind = 'waypoint' OR EXISTS (
                        SELECT 1 FROM map_home_link AS link
                        WHERE link.robot_id = waypoint.robot_id
                          AND link.waypoint_id = waypoint.waypoint_id
                          AND link.adopted = 1 AND link.status = 'confirmed'
                    ))
                ORDER BY waypoint_id LIMIT ?
                """,
                (self._robot_id, map_ref.map_id, map_ref.reference_revision, after, limit),
            ) as cursor:
                rows = await cursor.fetchall()
            await connection.rollback()
        return tuple(
            _stored(row, adopted=None, link_status=None, ports=self._ports) for row in rows
        )

    async def save_home(
        self,
        *,
        waypoint_id: str | None,
        pose: PoseT,
        map_ref: MapReference,
        anchor_id: str,
        anchor_pose: PoseT,
        source_robot_id: str,
        status: str,
        adopted: bool,
        latitude: float | None,
        longitude: float | None,
        expected_revision: int | None = None,
    ) -> WaypointT:
        return await self._save(
            waypoint_id=waypoint_id,
            kind="home",
            pose=pose,
            map_ref=map_ref,
            anchor_id=anchor_id,
            anchor_pose=anchor_pose,
            source_robot_id=source_robot_id,
            status=status,
            adopted=adopted,
            latitude=latitude,
            longitude=longitude,
            expected_revision=expected_revision,
        )

    async def save_waypoint(
        self,
        *,
        waypoint_id: str,
        pose: PoseT,
        map_ref: MapReference,
        anchor_id: str,
        anchor_pose: PoseT,
        source_robot_id: str,
        status: str,
        latitude: float | None,
        longitude: float | None,
        create_only: bool = False,
    ) -> WaypointT:
        return await self._save(
            waypoint_id=waypoint_id,
            kind="waypoint",
            pose=pose,
            map_ref=map_ref,
            anchor_id=anchor_id,
            anchor_pose=anchor_pose,
            source_robot_id=source_robot_id,
            status=status,
            adopted=None,
            latitude=latitude,
            longitude=longitude,
            create_only=create_only,
        )

    async def sync_poses(self, bindings: tuple[BindingReading, ...]) -> None:
        async with self._database.write_lock:
            async with self._transaction() as connection:
                for binding in bindings:
                    await connection.execute(
                        """
                        UPDATE waypoint
                        SET x = ?, y = ?, yaw = ?, anchor_x = ?, anchor_y = ?, anchor_yaw = ?,
                            recorded_at = ?
                        WHERE robot_id = ? AND waypoint_id = ?
                        """,
                        (
                            binding.waypoint.pose.x,
                            binding.waypoint.pose.y,
                            binding.waypoint.pose.yaw,
                            binding.anchor_pose.x,
                            binding.anchor_pose.y,
                            binding.anchor_pose.yaw,
                            _timestamp(),
                            self._robot_id,
                            binding.waypoint.waypoint_id,
                        ),
                    )

    async def _save(
        self,
        *,
        waypoint_id: str | None,
        kind: str,
        pose: PoseT,
        map_ref: MapReference,
        anchor_id: str,
        anchor_pose: PoseT,
        source_robot_id: str,
        status: str,
        adopted: bool | None,
        latitude: float | None,
        longitude: float | None,
        expected_revision: int | None = None,
        create_only: bool = False,
    ) -> WaypointT:
        async with self._database.write_lock:
            async with self._transaction() as connection:
                if kind == "home":
                    await self._bump_home_revision(connection, expected_revision)
                target = await self._target_id(connection, kind, waypoint_id)
                if create_only:
                    async with connection.execute(
                        "SELECT 1 FROM waypoint WHERE waypoint_id = ?", (target,)
                    ) as cursor:
                        if await cursor.fetchone() is not None:
                            raise self._ports.error(
                                ErrorCode.CONFLICT, "waypoint id already exists"
                            )
                await _write_waypoint(
                    connection,
                    waypoint_id=target,
                    robot_id=self._robot_id,
                    kind=kind,
                    pose=pose,
                    map_ref=map_ref,
                    anchor_id=anchor_id,
                    anchor_pose=anchor_pose,
                    source_robot_id=source_robot_id,
                    status=status,
                    latitude=latitude,
                    longitude=longitude,
                )
                if kind == "home":
                    if adopted is None:
                        raise self._ports.error(ErrorCode.VALIDATION, "home adoption is required")
                    await connection.execute(
                        """
                        INSERT INTO map_home_link (
                            robot_id, source_robot_id, map_id, reference_revision,
                            waypoint_id, status, adopted
                        )
                        VALUES (?, ?, ?, ?, ?, ?, ?)
                        ON CONFLICT(robot_id) DO UPDATE SET
                            source_robot_id = excluded.source_robot_id,
                            map_id = excluded.map_id,
                            reference_revision = excluded.reference_revision,
                            waypoint_id = excluded.waypoint_id,
                            status = excluded.status,
                            adopted = excluded.adopted
                        """,
                        (
                            self._robot_id,
                            source_robot_id,
                            map_ref.map_id,
                            map_ref.reference_revision,
                            target,
                            status,
                            int(adopted),
                        ),
                    )
        return self._ports.waypoint(
            target,
            kind,
            pose,
            map_ref,
            anchor_id,
            source_robot_id,
            status,
            latitude,
            longitude,
        )

    async def home_revision(self) -> int:
        async with self._database.write_lock:
            connection = self._database.connection
            try:
                async with connection.execute(
                    "SELECT revision FROM home_edit WHERE robot_id = ?",
                    (self._robot_id,),
                ) as cursor:
                    row = await cursor.fetchone()
            except sqlite3.OperationalError:
                return 0
        if row is None:
            return 0
        return int(row[0])

    async def _bump_home_revision(
        self, connection: aiosqlite.Connection, expected_revision: int | None
    ) -> None:
        await connection.execute(
            """
            CREATE TABLE IF NOT EXISTS home_edit (
                robot_id TEXT PRIMARY KEY,
                revision INTEGER NOT NULL
            )
            """
        )
        async with connection.execute(
            "SELECT revision FROM home_edit WHERE robot_id = ?",
            (self._robot_id,),
        ) as cursor:
            row = await cursor.fetchone()
        current = 0 if row is None else int(row[0])
        if expected_revision is not None and expected_revision != current:
            raise self._ports.error(ErrorCode.CONFLICT, "home revision conflict")
        await connection.execute(
            """
            INSERT INTO home_edit (robot_id, revision)
            VALUES (?, ?)
            ON CONFLICT(robot_id) DO UPDATE SET revision = excluded.revision
            """,
            (self._robot_id, current + 1),
        )

    async def _target_id(
        self, connection: aiosqlite.Connection, kind: str, waypoint_id: str | None
    ) -> str:
        if kind != "home":
            if waypoint_id is None or not waypoint_id.strip():
                raise self._ports.error(ErrorCode.VALIDATION, "waypoint id is required")
            return waypoint_id
        async with connection.execute(
            """
            SELECT waypoint_id
            FROM waypoint
            WHERE robot_id = ? AND kind = 'home'
            """,
            (self._robot_id,),
        ) as cursor:
            row = await cursor.fetchone()
        if row is not None:
            return _text(row[0], error=self._ports.error)
        if waypoint_id is not None and waypoint_id.strip():
            return waypoint_id
        return uuid.uuid4().hex

    @asynccontextmanager
    async def _transaction(self) -> AsyncIterator[aiosqlite.Connection]:
        connection = self._database.connection
        await connection.rollback()
        async with connection.execute("BEGIN IMMEDIATE"):
            pass
        try:
            yield connection
        except BaseException:
            await connection.rollback()
            raise
        else:
            await connection.commit()


async def adopted_home_blocks_foreign(
    connection: aiosqlite.Connection,
    robot_id: str,
    *,
    error: Callable[[ErrorCode, str], Exception],
) -> bool:
    async with connection.execute(
        """
        SELECT status, adopted
        FROM map_home_link
        WHERE robot_id = ?
        """,
        (robot_id,),
    ) as cursor:
        row = await cursor.fetchone()
    if row is None:
        return False
    return _text(row[0], error=error) == "confirmed" and _flag(row[1], error=error)


async def replace_imported_home(
    connection: aiosqlite.Connection,
    *,
    robot_id: str,
    waypoint_id: str,
    pose: PoseReading,
    map_ref: MapReference,
    anchor_id: str,
    anchor_pose: PoseReading,
    source_robot_id: str,
    status: str,
) -> None:
    """Write the imported home pose on a connection that already holds the write transaction."""

    await connection.execute(
        """
        DELETE FROM waypoint
        WHERE robot_id = ? AND kind = 'home' AND waypoint_id != ?
        """,
        (robot_id, waypoint_id),
    )
    await _write_waypoint(
        connection,
        waypoint_id=waypoint_id,
        robot_id=robot_id,
        kind="home",
        pose=pose,
        map_ref=map_ref,
        anchor_id=anchor_id,
        anchor_pose=anchor_pose,
        source_robot_id=source_robot_id,
        status=status,
        latitude=None,
        longitude=None,
    )


async def _write_waypoint(
    connection: aiosqlite.Connection,
    *,
    waypoint_id: str,
    robot_id: str,
    kind: str,
    pose: PoseReading,
    map_ref: MapReference,
    anchor_id: str,
    anchor_pose: PoseReading,
    source_robot_id: str,
    status: str,
    latitude: float | None,
    longitude: float | None,
) -> None:
    await connection.execute(
        """
        INSERT INTO waypoint (
            waypoint_id, robot_id, kind, x, y, yaw, map_id, reference_revision,
            anchor_id, source_robot_id, status, latitude, longitude,
            anchor_x, anchor_y, anchor_yaw, recorded_at
        )
        VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
        ON CONFLICT(waypoint_id) DO UPDATE SET
            robot_id = excluded.robot_id,
            kind = excluded.kind,
            x = excluded.x,
            y = excluded.y,
            yaw = excluded.yaw,
            map_id = excluded.map_id,
            reference_revision = excluded.reference_revision,
            anchor_id = excluded.anchor_id,
            source_robot_id = excluded.source_robot_id,
            status = excluded.status,
            latitude = excluded.latitude,
            longitude = excluded.longitude,
            anchor_x = excluded.anchor_x,
            anchor_y = excluded.anchor_y,
            anchor_yaw = excluded.anchor_yaw,
            recorded_at = excluded.recorded_at
        """,
        (
            waypoint_id,
            robot_id,
            kind,
            pose.x,
            pose.y,
            pose.yaw,
            map_ref.map_id,
            map_ref.reference_revision,
            anchor_id,
            source_robot_id,
            status,
            latitude,
            longitude,
            anchor_pose.x,
            anchor_pose.y,
            anchor_pose.yaw,
            _timestamp(),
        ),
    )


def _timestamp() -> str:
    return datetime.now(timezone.utc).isoformat()
