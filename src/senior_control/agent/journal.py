"""The robot's own record of every device-owned edit it was asked to make.

The table is created idempotently here instead of as a numbered migration: the schema
version line is shared with the cloud database (which already uses version 4), and the
domain stores refuse a database whose version they do not know.
"""

from collections.abc import AsyncIterator
from contextlib import asynccontextmanager
from dataclasses import dataclass

import aiosqlite

from senior_control.errors import ErrorCode
from senior_control.models import OperationStatus
from senior_control.storage.database import Database

AGENT_JOURNAL_STATEMENTS: tuple[str, ...] = (
    """
    CREATE TABLE IF NOT EXISTS agent_operation (
        operation_id TEXT PRIMARY KEY,
        robot_id TEXT NOT NULL,
        fingerprint TEXT NOT NULL,
        status TEXT NOT NULL,
        error TEXT,
        revision INTEGER
    )
    """,
    """
    CREATE TABLE IF NOT EXISTS agent_deleted_task (
        task_id TEXT PRIMARY KEY,
        robot_id TEXT NOT NULL,
        operation_id TEXT NOT NULL,
        revision INTEGER NOT NULL
    )
    """,
)

_OPEN = (
    OperationStatus.ACCEPTED,
    OperationStatus.DISPATCHING,
    OperationStatus.RUNNING,
)


@dataclass(frozen=True)
class JournalEntry:
    operation_id: str
    fingerprint: str
    status: OperationStatus
    error: ErrorCode | None
    revision: int | None

    @property
    def closed(self) -> bool:
        return self.status not in _OPEN


class AgentJournal:
    """Deduplicates by operation id. The same id with other content is a conflict.

    An entry is written before the port runs and closed after the port's own commit, so a
    crash between the two leaves ``running`` and the next boot turns it into ``needs_review``
    instead of replaying the edit.
    """

    def __init__(self, database: Database, *, robot_id: str) -> None:
        if not robot_id.strip():
            raise ValueError("robot id is required")
        self._database = database
        self._robot_id = robot_id

    async def prepare(self) -> None:
        async with self._database.write_lock:
            async with self._transaction() as connection:
                for statement in AGENT_JOURNAL_STATEMENTS:
                    await connection.execute(statement)
                await connection.execute(
                    f"""
                    UPDATE agent_operation SET status = ?
                    WHERE robot_id = ? AND status IN ({",".join("?" for _ in _OPEN)})
                    """,
                    (
                        OperationStatus.NEEDS_REVIEW.value,
                        self._robot_id,
                        *(status.value for status in _OPEN),
                    ),
                )

    async def get(self, operation_id: str) -> JournalEntry | None:
        async with self._database.write_lock:
            connection = self._database.connection
            await connection.rollback()
            entry = await self._entry(connection, operation_id)
            await connection.rollback()
        return entry

    async def begin(self, operation_id: str, fingerprint: str) -> JournalEntry | None:
        """Record a new operation, or return the entry that already owns the id."""
        async with self._database.write_lock:
            async with self._transaction() as connection:
                existing = await self._entry(connection, operation_id)
                if existing is not None:
                    return existing
                await connection.execute(
                    """
                    INSERT INTO agent_operation (operation_id, robot_id, fingerprint, status)
                    VALUES (?, ?, ?, ?)
                    """,
                    (operation_id, self._robot_id, fingerprint, OperationStatus.RUNNING.value),
                )
        return None

    async def finish(
        self,
        operation_id: str,
        status: OperationStatus,
        *,
        error: ErrorCode | None = None,
        revision: int | None = None,
    ) -> None:
        async with self._database.write_lock:
            async with self._transaction() as connection:
                await connection.execute(
                    """
                    UPDATE agent_operation SET status = ?, error = ?, revision = ?
                    WHERE operation_id = ? AND robot_id = ?
                    """,
                    (
                        status.value,
                        error.value if error is not None else None,
                        revision,
                        operation_id,
                        self._robot_id,
                    ),
                )

    async def mark_deleted(self, task_id: str, operation_id: str, revision: int) -> None:
        """Remember that the controller deleted this task. The task row and its history stay."""
        async with self._database.write_lock:
            async with self._transaction() as connection:
                await connection.execute(
                    """
                    INSERT INTO agent_deleted_task (task_id, robot_id, operation_id, revision)
                    VALUES (?, ?, ?, ?)
                    ON CONFLICT(task_id) DO UPDATE SET
                        operation_id = excluded.operation_id, revision = excluded.revision
                    """,
                    (task_id, self._robot_id, operation_id, revision),
                )

    async def deleted_task_ids(self) -> frozenset[str]:
        async with self._database.write_lock:
            connection = self._database.connection
            await connection.rollback()
            async with connection.execute(
                "SELECT task_id FROM agent_deleted_task WHERE robot_id = ?", (self._robot_id,)
            ) as cursor:
                rows = await cursor.fetchall()
            await connection.rollback()
        return frozenset(str(row[0]) for row in rows)

    async def _entry(
        self, connection: aiosqlite.Connection, operation_id: str
    ) -> JournalEntry | None:
        async with connection.execute(
            """
            SELECT operation_id, fingerprint, status, error, revision
            FROM agent_operation WHERE operation_id = ? AND robot_id = ?
            """,
            (operation_id, self._robot_id),
        ) as cursor:
            row = await cursor.fetchone()
        if row is None:
            return None
        error = row[3]
        revision = row[4]
        return JournalEntry(
            operation_id=str(row[0]),
            fingerprint=str(row[1]),
            status=OperationStatus(str(row[2])),
            error=ErrorCode(str(error)) if error is not None else None,
            revision=int(revision) if revision is not None else None,
        )

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
