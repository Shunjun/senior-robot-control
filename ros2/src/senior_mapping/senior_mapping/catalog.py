"""Durable map catalog; metadata never constitutes graph or pose authority.

The production session owner must verify/capture graph + raster + the canonical
Home transaction before publishing a checkpoint ID. This store keeps only that
immutable checkpoint's Home identity, never a competing live Home pose.
"""

import re
import time
import uuid
from collections.abc import AsyncIterator
from contextlib import asynccontextmanager
from dataclasses import dataclass
from typing import Any

import aiosqlite

from senior_mapping.persistence import MapDatabase
from vehicle_core.errors import DomainError, ErrorCode
from vehicle_core.values import MapRef

_CHECKPOINT = re.compile(r"[0-9a-f]{32}\Z")


@dataclass(frozen=True)
class CatalogEntry:
    map_ref: MapRef
    name: str
    checkpoint_id: str | None
    home_id: str | None
    version: int
    saved_at: float | None


@dataclass(frozen=True)
class MapSelection:
    map_ref: MapRef | None
    token: str | None
    generation: int
    state: str


class MapCatalog:
    def __init__(self, database: MapDatabase, *, robot_id: str, max_maps: int = 64) -> None:
        if not robot_id.strip() or not 1 <= max_maps <= 1024:
            raise ValueError("catalog requires robot identity and bounded capacity")
        self._database, self._robot_id, self._max_maps = database, robot_id, max_maps
        self._prepared = False

    @asynccontextmanager
    async def _write(self) -> AsyncIterator[aiosqlite.Connection]:
        async with self._database.write_lock:
            connection = self._database.connection
            await connection.execute("BEGIN IMMEDIATE")
            try:
                yield connection
            except BaseException:
                await connection.rollback()
                raise
            else:
                await connection.commit()

    async def prepare(self) -> None:
        if self._prepared:
            return
        async with self._write() as connection:
            await connection.execute("""
                CREATE TABLE IF NOT EXISTS mapping_catalog (
                    robot_id TEXT NOT NULL,
                    map_id TEXT NOT NULL,
                    reference_revision INTEGER NOT NULL,
                    name TEXT NOT NULL,
                    checkpoint_id TEXT,
                    home_id TEXT,
                    version INTEGER NOT NULL DEFAULT 0,
                    saved_at REAL,
                    retired INTEGER NOT NULL DEFAULT 0,
                    PRIMARY KEY (robot_id, map_id)
                )
            """)
            await connection.execute("""
                CREATE TABLE IF NOT EXISTS mapping_selection (
                    robot_id TEXT PRIMARY KEY,
                    map_id TEXT,
                    token TEXT,
                    generation INTEGER NOT NULL DEFAULT 0,
                    state TEXT NOT NULL DEFAULT 'none',
                    FOREIGN KEY (robot_id, map_id) REFERENCES mapping_catalog (robot_id, map_id)
                )
            """)
            await connection.execute(
                "INSERT OR IGNORE INTO mapping_selection (robot_id) VALUES (?)", (self._robot_id,)
            )
            # Restart retains the selected map, but never restores an old pose or
            # lets a result from the previous owner complete this boot's match.
            await connection.execute(
                """
                UPDATE mapping_selection
                SET token = ?, generation = generation + 1, state = 'pending_calibration'
                WHERE robot_id = ? AND map_id IS NOT NULL
            """,
                (uuid.uuid4().hex, self._robot_id),
            )
        self._prepared = True

    async def create(self, operation_id: str, name: str) -> CatalogEntry:
        if not 1 <= len(operation_id) <= 128 or not name.strip() or len(name) > 64:
            raise DomainError(ErrorCode.VALIDATION, "invalid map name or creation operation")
        map_id = (
            "map-"
            + uuid.uuid5(uuid.NAMESPACE_URL, f"senior-map:{self._robot_id}:{operation_id}").hex
        )
        async with self._write() as connection:
            row = await self._row(connection, map_id)
            if row is not None:
                if row[2] != name or row[7]:
                    raise DomainError(ErrorCode.CONFLICT, "map creation was changed or retired")
                return _entry(row)
            async with connection.execute(
                "SELECT count(*) FROM mapping_catalog WHERE robot_id = ? AND retired = 0",
                (self._robot_id,),
            ) as cursor:
                count = (await cursor.fetchone())[0]
            if count >= self._max_maps:
                raise DomainError(ErrorCode.CONFLICT, "map catalog capacity reached")
            await connection.execute(
                """
                INSERT INTO mapping_catalog (robot_id, map_id, reference_revision, name)
                VALUES (?, ?, 1, ?)
            """,
                (self._robot_id, map_id, name),
            )
            return CatalogEntry(MapRef(map_id, 1), name, None, None, 0, None)

    async def list(self) -> tuple[CatalogEntry, ...]:
        async with self._database.write_lock:
            async with self._database.connection.execute(
                """
                SELECT map_id, reference_revision, name, checkpoint_id, home_id, version, saved_at
                FROM mapping_catalog WHERE robot_id = ? AND retired = 0 ORDER BY rowid
            """,
                (self._robot_id,),
            ) as cursor:
                return tuple(_entry(row) for row in await cursor.fetchall())

    async def checkpoint(
        self,
        map_ref: MapRef,
        checkpoint_id: str,
        *,
        home_id: str,
        expected: str | None,
        session_token: str | None = None,
    ) -> None:
        """Publish only after the session owner's complete durable capture.

        Compare-and-swap rejects a late older capture. Ordinary raster/checkpoint
        updates advance content version, never MapRef or activation generation.
        """
        if not _CHECKPOINT.fullmatch(checkpoint_id) or not 1 <= len(home_id) <= 128:
            raise DomainError(ErrorCode.VALIDATION, "invalid complete checkpoint identity")
        async with self._write() as connection:
            row = await self._row(connection, map_ref.map_id)
            if row is None or row[7]:
                raise DomainError(ErrorCode.CONFLICT, "map is absent or retired")
            if row[5] > 0:
                selected = await self._selection(connection)
                if (
                    session_token is None
                    or selected.token != session_token
                    or selected.map_ref != map_ref
                    or selected.state != "ready"
                ):
                    raise DomainError(
                        ErrorCode.CONFLICT, "checkpoint belongs to unlocalized or old session"
                    )
            async with connection.execute(
                """
                UPDATE mapping_catalog SET checkpoint_id = ?, home_id = ?,
                    version = version + 1, saved_at = ?
                WHERE robot_id = ? AND map_id = ? AND reference_revision = ?
                    AND checkpoint_id IS ? AND retired = 0
            """,
                (
                    checkpoint_id,
                    home_id,
                    time.time(),
                    self._robot_id,
                    map_ref.map_id,
                    map_ref.reference_revision,
                    expected,
                ),
            ) as cursor:
                if cursor.rowcount != 1:
                    raise DomainError(ErrorCode.CONFLICT, "map checkpoint changed or retired")

    async def selection(self) -> MapSelection:
        async with self._database.write_lock:
            return await self._selection(self._database.connection)

    async def begin_activation(self, map_ref: MapRef, *, expected_generation: int) -> MapSelection:
        """After the old session's retire/flush barrier, begin an untrusted match."""
        async with self._write() as connection:
            row = await self._row(connection, map_ref.map_id)
            if row is None or row[7] or row[1] != map_ref.reference_revision or row[3] is None:
                raise DomainError(ErrorCode.CONFLICT, "target map has no complete checkpoint")
            async with connection.execute(
                """
                UPDATE mapping_selection SET map_id = ?, token = ?,
                    generation = generation + 1, state = 'matching'
                WHERE robot_id = ? AND generation = ?
            """,
                (map_ref.map_id, uuid.uuid4().hex, self._robot_id, expected_generation),
            ) as cursor:
                if cursor.rowcount != 1:
                    raise DomainError(ErrorCode.CONFLICT, "active map changed")
            return await self._selection(connection)

    async def finish_activation(self, token: str, *, localized: bool) -> MapSelection:
        """Only the verified localization owner completes a match, never a UI hint."""
        async with self._write() as connection:
            async with connection.execute(
                """
                UPDATE mapping_selection SET state = ?
                WHERE robot_id = ? AND token = ? AND state = 'matching'
            """,
                ("ready" if localized else "pending_calibration", self._robot_id, token),
            ) as cursor:
                if cursor.rowcount != 1:
                    raise DomainError(
                        ErrorCode.CONFLICT, "localization result belongs to old session"
                    )
            return await self._selection(connection)

    async def withdraw(self, *, expected_generation: int) -> MapSelection:
        """Session owner calls this after actual backend/navigation retirement."""
        async with self._write() as connection:
            async with connection.execute(
                """
                UPDATE mapping_selection SET map_id = NULL, token = NULL,
                    generation = generation + 1, state = 'none'
                WHERE robot_id = ? AND generation = ?
            """,
                (self._robot_id, expected_generation),
            ) as cursor:
                if cursor.rowcount != 1:
                    raise DomainError(ErrorCode.CONFLICT, "active map changed")
            return await self._selection(connection)

    async def delete_inactive(self, map_ref: MapRef) -> None:
        async with self._write() as connection:
            selected = await self._selection(connection)
            if selected.map_ref == map_ref:
                raise DomainError(ErrorCode.CONFLICT, "active map must be retired before deletion")
            async with connection.execute(
                """
                UPDATE mapping_catalog SET retired = 1
                WHERE robot_id = ? AND map_id = ? AND reference_revision = ?
            """,
                (self._robot_id, map_ref.map_id, map_ref.reference_revision),
            ) as cursor:
                if cursor.rowcount != 1:
                    raise DomainError(ErrorCode.CONFLICT, "map reference changed")

    async def _row(self, connection: aiosqlite.Connection, map_id: str) -> tuple[Any, ...] | None:
        async with connection.execute(
            """
            SELECT map_id, reference_revision, name, checkpoint_id,
                home_id, version, saved_at, retired
            FROM mapping_catalog WHERE robot_id = ? AND map_id = ?
        """,
            (self._robot_id, map_id),
        ) as cursor:
            return await cursor.fetchone()

    async def _selection(self, connection: aiosqlite.Connection) -> MapSelection:
        async with connection.execute(
            """
            SELECT s.map_id, c.reference_revision, s.token, s.generation, s.state
            FROM mapping_selection s LEFT JOIN mapping_catalog c
                ON c.robot_id = s.robot_id AND c.map_id = s.map_id
            WHERE s.robot_id = ?
        """,
            (self._robot_id,),
        ) as cursor:
            row = await cursor.fetchone()
        if row is None:
            raise DomainError(ErrorCode.UNAVAILABLE, "map catalog is not prepared")
        return MapSelection(
            None if row[0] is None else MapRef(row[0], row[1]), row[2], row[3], row[4]
        )


def _entry(row: tuple[Any, ...]) -> CatalogEntry:
    return CatalogEntry(MapRef(row[0], row[1]), row[2], row[3], row[4], row[5], row[6])
