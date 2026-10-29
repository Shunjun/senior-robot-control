"""Waypoint rows. Version 2 of the domain schema lives with this table."""

import math
import uuid
from collections.abc import AsyncIterator
from contextlib import asynccontextmanager
from dataclasses import dataclass
from datetime import datetime, timezone

import aiosqlite

from senior_control.errors import DomainError, ErrorCode
from senior_control.mapping.references import WaypointBinding
from senior_control.models import MapRef, Pose2D
from senior_control.storage.database import Database

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


class WaypointStore:
    def __init__(self, database: Database, *, robot_id: str) -> None:
        if not robot_id.strip():
            raise ValueError("robot id is required")
        self._database = database
        self._robot_id = robot_id

    async def prepare(self) -> None:
        from senior_control.storage.migrations import migrate
        from senior_control.storage.schema import DOMAIN_MIGRATIONS

        await migrate(self._database, list(DOMAIN_MIGRATIONS))

    async def home(self) -> StoredWaypoint | None:
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
        return _stored(row, adopted=_flag(row[15]), link_status=_text(row[16]))

    async def waypoint(self, waypoint_id: str) -> StoredWaypoint | None:
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
        return _stored(row, adopted=None, link_status=None)

    async def points(self) -> tuple[StoredWaypoint, ...]:
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
        return tuple(_stored(row, adopted=None, link_status=None) for row in rows)

    async def save_home(
        self,
        *,
        waypoint_id: str | None,
        pose: Pose2D,
        map_ref: MapRef,
        anchor_id: str,
        anchor_pose: Pose2D,
        source_robot_id: str,
        status: str,
        adopted: bool,
        latitude: float | None,
        longitude: float | None,
    ) -> Waypoint:
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
        )

    async def save_waypoint(
        self,
        *,
        waypoint_id: str,
        pose: Pose2D,
        map_ref: MapRef,
        anchor_id: str,
        anchor_pose: Pose2D,
        source_robot_id: str,
        status: str,
        latitude: float | None,
        longitude: float | None,
    ) -> Waypoint:
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
        )

    async def sync_poses(self, bindings: tuple[WaypointBinding, ...]) -> None:
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
        pose: Pose2D,
        map_ref: MapRef,
        anchor_id: str,
        anchor_pose: Pose2D,
        source_robot_id: str,
        status: str,
        adopted: bool | None,
        latitude: float | None,
        longitude: float | None,
    ) -> Waypoint:
        async with self._database.write_lock:
            async with self._transaction() as connection:
                target = await self._target_id(connection, kind, waypoint_id)
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
                        raise DomainError(ErrorCode.VALIDATION, "home adoption is required")
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
        return Waypoint(
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

    async def _target_id(
        self, connection: aiosqlite.Connection, kind: str, waypoint_id: str | None
    ) -> str:
        if kind != "home":
            if waypoint_id is None or not waypoint_id.strip():
                raise DomainError(ErrorCode.VALIDATION, "waypoint id is required")
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
            return _text(row[0])
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


async def replace_imported_home(
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
    pose: Pose2D,
    map_ref: MapRef,
    anchor_id: str,
    anchor_pose: Pose2D,
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


def _stored(row: aiosqlite.Row, *, adopted: bool | None, link_status: str | None) -> StoredWaypoint:
    latitude = _optional_real(row[10])
    longitude = _optional_real(row[11])
    waypoint = Waypoint(
        _text(row[0]),
        _text(row[1]),
        Pose2D(_real(row[2]), _real(row[3]), _real(row[4])),
        MapRef(_text(row[5]), _whole(row[6])),
        _text(row[7]),
        _text(row[8]),
        _text(row[9]),
        latitude,
        longitude,
    )
    return StoredWaypoint(
        waypoint,
        Pose2D(_real(row[12]), _real(row[13]), _real(row[14])),
        adopted,
        link_status,
    )


def _timestamp() -> str:
    return datetime.now(timezone.utc).isoformat()


def _text(value: object) -> str:
    if not isinstance(value, str):
        raise DomainError(ErrorCode.VALIDATION, "expected text")
    return value


def _whole(value: object) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise DomainError(ErrorCode.VALIDATION, "expected integer")
    return value


def _real(value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise DomainError(ErrorCode.VALIDATION, "expected number")
    number = float(value)
    if not math.isfinite(number):
        raise DomainError(ErrorCode.VALIDATION, "expected finite number")
    return number


def _optional_real(value: object) -> float | None:
    if value is None:
        return None
    return _real(value)


def _flag(value: object) -> bool:
    if isinstance(value, bool):
        return value
    if value == 0:
        return False
    if value == 1:
        return True
    raise DomainError(ErrorCode.VALIDATION, "expected flag")
