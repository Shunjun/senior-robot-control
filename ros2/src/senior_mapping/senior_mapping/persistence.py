"""Only the existing connection, schema and in-transaction Home effects maps consume."""

from collections.abc import Awaitable, Callable
from contextlib import AbstractAsyncContextManager
from dataclasses import dataclass
from typing import Any, Protocol

import aiosqlite

from senior_control.models import MapRef, Pose2D


class MapDatabase(Protocol):
    @property
    def connection(self) -> aiosqlite.Connection: ...

    @property
    def write_lock(self) -> AbstractAsyncContextManager[Any]: ...


class HomeImport(Protocol):
    """Use the caller's open transaction; never acquire a lock or commit here."""

    async def adopted_home_blocks_foreign(
        self, connection: aiosqlite.Connection, robot_id: str
    ) -> bool: ...

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
    ) -> None: ...


@dataclass(frozen=True)
class MapPersistence:
    prepare_schema: Callable[[], Awaitable[None]]
    home: HomeImport
