"""Version 3 task definitions, claims, and recorded skip intervals."""

import json
import sqlite3
import uuid
from collections.abc import AsyncIterator
from contextlib import asynccontextmanager
from datetime import datetime, timezone

import aiosqlite

from senior_control.errors import DomainError, ErrorCode
from senior_control.models import MapRef
from senior_control.storage.database import Database
from senior_control.tasks.models import (
    ExecutionRecord,
    ExecutionStatus,
    SkipRecord,
    TaskDefinition,
)
from senior_control.tasks.schedule import Schedule

_SCAN_COLUMNS = (
    "SELECT e.execution_id, e.task_id, e.occurrence_utc, e.status, "
    "e.navigation_sent, e.rollback_confirmed FROM task_definition t CROSS JOIN "
)
ACTIVE_SCAN_SQL = (
    _SCAN_COLUMNS + "task_execution e INDEXED BY task_execution_scan_idx "
    "WHERE t.robot_id = ? AND e.task_id = t.task_id AND e.status IN (?, ?, ?) LIMIT ?"
)
WINDOW_SCAN_SQL = (
    _SCAN_COLUMNS + "task_execution e INDEXED BY task_execution_window_idx "
    "WHERE t.robot_id = ? AND e.task_id = t.task_id AND e.occurrence_utc > ? "
    "AND e.occurrence_utc <= ? AND e.status NOT IN (?, ?, ?) LIMIT ?"
)
_SCAN_LIMIT = 4096


TASK_STATEMENTS: tuple[str, ...] = (
    """
    CREATE TABLE task_definition (
        task_id TEXT PRIMARY KEY,
        robot_id TEXT NOT NULL,
        map_id TEXT NOT NULL,
        reference_revision INTEGER NOT NULL,
        waypoint_ids TEXT NOT NULL,
        schedule_kind TEXT NOT NULL,
        every_seconds INTEGER,
        cron_expression TEXT,
        timezone TEXT NOT NULL,
        enabled INTEGER NOT NULL,
        revision INTEGER NOT NULL,
        created_at TEXT NOT NULL
    )
    """,
    """
    CREATE TABLE task_execution (
        execution_id TEXT PRIMARY KEY,
        task_id TEXT NOT NULL,
        occurrence_utc TEXT NOT NULL,
        status TEXT NOT NULL,
        navigation_sent INTEGER NOT NULL,
        rollback_confirmed INTEGER NOT NULL,
        UNIQUE (task_id, occurrence_utc),
        FOREIGN KEY (task_id) REFERENCES task_definition (task_id)
    )
    """,
    """
    CREATE TABLE task_skip (
        skip_id TEXT PRIMARY KEY,
        task_id TEXT NOT NULL,
        skip_from TEXT NOT NULL,
        skip_until TEXT NOT NULL,
        reason TEXT NOT NULL,
        FOREIGN KEY (task_id) REFERENCES task_definition (task_id)
    )
    """,
    """
    CREATE TABLE task_runtime (
        robot_id TEXT PRIMARY KEY,
        boot_id TEXT NOT NULL
    )
    """,
)


class TaskStore:
    def __init__(self, database: Database, *, robot_id: str) -> None:
        if not robot_id.strip():
            raise ValueError("robot id is required")
        self._database = database
        self._robot_id = robot_id
        self._boot_id = uuid.uuid4().hex

    async def prepare(self) -> None:
        from senior_control.storage.core_schema import migrate_core
        from senior_control.storage.migrations import migrate
        from senior_control.storage.schema import DOMAIN_MIGRATIONS

        await migrate(self._database, list(DOMAIN_MIGRATIONS))
        await migrate_core(self._database)
        async with self._database.write_lock:
            async with self._transaction() as connection:
                # //Compatible: indexes are rebuilt idempotently on existing databases.
                await connection.execute(
                    "CREATE INDEX IF NOT EXISTS task_execution_generation_idx "
                    "ON task_execution_generation(generation, execution_id)"
                )
                await connection.execute(
                    "CREATE INDEX IF NOT EXISTS task_execution_scan_idx "
                    "ON task_execution(task_id, status, occurrence_utc)"
                )
                await connection.execute(
                    "CREATE INDEX IF NOT EXISTS task_execution_window_idx "
                    "ON task_execution(task_id, occurrence_utc)"
                )
                async with connection.execute(
                    "SELECT boot_id FROM task_runtime WHERE robot_id = ?",
                    (self._robot_id,),
                ) as cursor:
                    row = await cursor.fetchone()
                if row is None or _text(row[0]) != self._boot_id:
                    await connection.execute(
                        """
                        UPDATE task_execution
                        SET status = ?, rollback_confirmed = 0
                        WHERE status IN (?, ?)
                          AND task_id IN (
                              SELECT task_id FROM task_definition WHERE robot_id = ?
                          )
                        """,
                        (
                            ExecutionStatus.CANCEL_PENDING.value,
                            ExecutionStatus.DISPATCHING.value,
                            ExecutionStatus.RUNNING.value,
                            self._robot_id,
                        ),
                    )
                    await connection.execute(
                        """
                        INSERT INTO task_runtime (robot_id, boot_id)
                        VALUES (?, ?)
                        ON CONFLICT(robot_id) DO UPDATE SET boot_id = excluded.boot_id
                        """,
                        (self._robot_id, self._boot_id),
                    )

    async def scan_cursor(self) -> datetime | None:
        async with self._database.write_lock:
            async with self._database.connection.execute(
                "SELECT last_tick_utc FROM task_scan_cursor WHERE robot_id = ?",
                (self._robot_id,),
            ) as cursor:
                row = await cursor.fetchone()
        return _moment(_text(row[0])) if row is not None else None

    async def save_scan_cursor(self, moment: datetime) -> None:
        async with self._database.write_lock:
            async with self._transaction() as connection:
                await connection.execute(
                    "INSERT INTO task_scan_cursor(robot_id, last_tick_utc) VALUES (?, ?) "
                    "ON CONFLICT(robot_id) DO UPDATE SET last_tick_utc=excluded.last_tick_utc",
                    (self._robot_id, _stamp(moment)),
                )

    async def list_tasks(self) -> tuple[TaskDefinition, ...]:
        async with self._database.write_lock:
            async with self._database.connection.execute(
                "SELECT task_id, robot_id, map_id, reference_revision, waypoint_ids, "
                "schedule_kind, every_seconds, cron_expression, timezone, enabled, revision, "
                "created_at FROM task_definition WHERE robot_id = ? ORDER BY task_id",
                (self._robot_id,),
            ) as cursor:
                rows = await cursor.fetchall()
        return tuple(_task(row) for row in rows)

    async def snapshot_tasks(self) -> tuple[tuple[TaskDefinition, ...], frozenset[str]]:
        """Read task revisions and their deletion markers in one SQLite statement."""
        try:
            async with self._database.write_lock:
                async with self._database.connection.execute(
                    """
                    SELECT t.task_id, t.robot_id, t.map_id, t.reference_revision, t.waypoint_ids,
                           t.schedule_kind, t.every_seconds, t.cron_expression, t.timezone,
                           t.enabled, t.revision, t.created_at, d.task_id
                    FROM task_definition AS t
                    LEFT JOIN agent_deleted_task AS d
                        ON d.task_id = t.task_id AND d.robot_id = t.robot_id
                    WHERE t.robot_id = ?
                    ORDER BY t.task_id
                    """,
                    (self._robot_id,),
                ) as cursor:
                    rows = await cursor.fetchall()
        except sqlite3.OperationalError as exc:
            # //Compatible: standalone TaskStore users may not have an agent journal.
            if str(exc) != "no such table: agent_deleted_task":
                raise
            return await self.list_tasks(), frozenset()
        return tuple(_task(row) for row in rows), frozenset(
            _text(row[12]) for row in rows if row[12] is not None
        )

    async def executions_for_scan(
        self, start: datetime | None, end: datetime
    ) -> tuple[ExecutionRecord, ...]:
        """Bound memory while retaining active executions and deduplication in the scan window."""
        statuses = (
            ExecutionStatus.DISPATCHING.value,
            ExecutionStatus.RUNNING.value,
            ExecutionStatus.CANCEL_PENDING.value,
        )
        async with self._database.write_lock:
            async with self._database.connection.execute(
                ACTIVE_SCAN_SQL, (self._robot_id, *statuses, _SCAN_LIMIT + 1)
            ) as cursor:
                rows = list(await cursor.fetchall())
            if len(rows) > _SCAN_LIMIT:
                raise RuntimeError("task scan execution limit exceeded")
            if start is not None:
                async with self._database.connection.execute(
                    WINDOW_SCAN_SQL,
                    (
                        self._robot_id,
                        _stamp(start),
                        _stamp(end),
                        *statuses,
                        _SCAN_LIMIT + 1 - len(rows),
                    ),
                ) as cursor:
                    rows.extend(await cursor.fetchall())
        if len(rows) > _SCAN_LIMIT:
            raise RuntimeError("task scan execution limit exceeded")
        return tuple(_execution(row) for row in rows)

    async def list_executions(self) -> tuple[ExecutionRecord, ...]:
        async with self._database.write_lock:
            async with self._database.connection.execute(
                "SELECT e.execution_id, e.task_id, e.occurrence_utc, e.status, "
                "e.navigation_sent, e.rollback_confirmed FROM task_execution e "
                "JOIN task_definition t ON t.task_id = e.task_id WHERE t.robot_id = ? "
                "ORDER BY e.occurrence_utc, e.execution_id",
                (self._robot_id,),
            ) as cursor:
                rows = await cursor.fetchall()
        return tuple(_execution(row) for row in rows)

    async def create(self, task: TaskDefinition) -> TaskDefinition:
        self._require_robot(task)
        async with self._database.write_lock:
            async with self._transaction() as connection:
                await self._insert_task(connection, task, revision=1)
                await self._bind_link(connection, task)
                stored = await self._task_in(connection, task.task_id)
        if stored is None:
            raise DomainError(ErrorCode.VALIDATION, "task was not stored")
        return stored

    async def get(self, task_id: str) -> TaskDefinition | None:
        async with self._database.write_lock:
            connection = self._database.connection
            await connection.rollback()
            stored = await self._task_in(connection, task_id)
            await connection.rollback()
        return stored

    async def update(self, task: TaskDefinition, expected_revision: int) -> TaskDefinition:
        self._require_robot(task)
        if isinstance(expected_revision, bool) or not isinstance(expected_revision, int):
            raise DomainError(ErrorCode.VALIDATION, "revision must be an integer")
        async with self._database.write_lock:
            async with self._transaction() as connection:
                current = await self._task_in(connection, task.task_id)
                if current is None:
                    raise DomainError(ErrorCode.CONFLICT, "task does not exist")
                if await self._deleted(connection, task.task_id):
                    raise DomainError(ErrorCode.CONFLICT, "task was deleted")
                if current.revision != expected_revision:
                    raise DomainError(ErrorCode.CONFLICT, "revision conflict")
                if current.created_at != task.created_at:
                    raise DomainError(ErrorCode.VALIDATION, "created time is immutable")
                revision = expected_revision + 1
                await connection.execute(
                    """
                    UPDATE task_definition
                    SET map_id = ?, reference_revision = ?, waypoint_ids = ?,
                        schedule_kind = ?, every_seconds = ?, cron_expression = ?,
                        timezone = ?, enabled = ?, revision = ?
                    WHERE task_id = ? AND robot_id = ? AND revision = ?
                    """,
                    (
                        task.map_ref.map_id,
                        task.map_ref.reference_revision,
                        _points(task),
                        task.schedule.kind,
                        task.schedule.every_seconds,
                        task.schedule.expression,
                        task.timezone,
                        int(task.enabled),
                        revision,
                        task.task_id,
                        self._robot_id,
                        expected_revision,
                    ),
                )
                if task.map_ref != current.map_ref:
                    await connection.execute(
                        """
                        UPDATE map_task_link
                        SET map_id = ?, reference_revision = ?, invalidated = 0
                        WHERE task_id = ?
                        """,
                        (
                            task.map_ref.map_id,
                            task.map_ref.reference_revision,
                            task.task_id,
                        ),
                    )
                stored = await self._task_in(connection, task.task_id)
        if stored is None or stored.revision != revision:
            raise DomainError(ErrorCode.CONFLICT, "revision conflict")
        return stored

    async def delete(self, task_id: str, expected_revision: int, *, operation_id: str) -> int:
        """Disable a task and write the agent tombstone in the same transaction."""
        if isinstance(expected_revision, bool) or not isinstance(expected_revision, int):
            raise DomainError(ErrorCode.VALIDATION, "revision must be an integer")
        revision = expected_revision + 1
        async with self._database.write_lock:
            async with self._transaction() as connection:
                if await self._deleted(connection, task_id):
                    raise DomainError(ErrorCode.CONFLICT, "task was deleted")
                async with connection.execute(
                    """
                    UPDATE task_definition SET enabled = 0, revision = ?
                    WHERE task_id = ? AND robot_id = ? AND revision = ?
                    """,
                    (revision, task_id, self._robot_id, expected_revision),
                ) as cursor:
                    if cursor.rowcount != 1:
                        raise DomainError(
                            ErrorCode.CONFLICT, "task is missing or revision conflicts"
                        )
                await connection.execute(
                    """
                    INSERT INTO agent_deleted_task (task_id, robot_id, operation_id, revision)
                    VALUES (?, ?, ?, ?)
                    """,
                    (task_id, self._robot_id, operation_id, revision),
                )
        return revision

    async def claim(
        self, task: TaskDefinition, occurrence_utc: datetime, *, generation: int = 0
    ) -> ExecutionRecord:
        self._require_robot(task)
        occurrence = _utc(occurrence_utc)
        execution_id = uuid.uuid4().hex
        try:
            async with self._database.write_lock:
                async with self._transaction() as connection:
                    current = await self._task_in(connection, task.task_id)
                    if current is None:
                        raise DomainError(ErrorCode.CONFLICT, "task does not exist")
                    if not current.enabled:
                        raise DomainError(ErrorCode.CONFLICT, "task is disabled")
                    if current != task:
                        raise DomainError(ErrorCode.CONFLICT, "task definition changed")
                    if await self._deleted(connection, task.task_id):
                        raise DomainError(ErrorCode.CONFLICT, "task is deleted")
                    if await self._invalidated(connection, task.task_id):
                        raise DomainError(ErrorCode.CONFLICT, "map link invalidated")
                    if await self._in_flight(connection):
                        raise DomainError(ErrorCode.CONFLICT, "execution already in flight")
                    await connection.execute(
                        """
                        INSERT INTO task_execution (
                            execution_id, task_id, occurrence_utc, status,
                            navigation_sent, rollback_confirmed
                        )
                        VALUES (?, ?, ?, ?, 0, 0)
                        """,
                        (
                            execution_id,
                            task.task_id,
                            _stamp(occurrence),
                            ExecutionStatus.DISPATCHING.value,
                        ),
                    )
                    await connection.execute(
                        "INSERT INTO task_execution_generation(execution_id, generation) "
                        "VALUES (?, ?)",
                        (execution_id, generation),
                    )
                    stored = await self._execution_in(connection, execution_id)
        except sqlite3.IntegrityError as exc:
            raise DomainError(ErrorCode.CONFLICT, "duplicate execution") from exc
        if stored is None:
            raise DomainError(ErrorCode.CONFLICT, "execution was not stored")
        return stored

    async def mark_running(self, execution_id: str) -> ExecutionRecord:
        async with self._database.write_lock:
            async with self._transaction() as connection:
                await connection.execute(
                    "UPDATE task_execution SET status = ?, navigation_sent = 1 "
                    "WHERE execution_id = ? AND status = ? AND task_id IN "
                    "(SELECT task_id FROM task_definition WHERE robot_id = ?)",
                    (
                        ExecutionStatus.RUNNING.value,
                        execution_id,
                        ExecutionStatus.DISPATCHING.value,
                        self._robot_id,
                    ),
                )
                stored = await self._execution_in(connection, execution_id)
        if stored is None or stored.status not in {
            ExecutionStatus.RUNNING,
            ExecutionStatus.CANCEL_PENDING,
        }:
            raise DomainError(ErrorCode.CONFLICT, "execution cannot be marked running")
        return stored

    async def finish(self, execution_id: str, *, succeeded: bool) -> None:
        status = ExecutionStatus.SUCCEEDED if succeeded else ExecutionStatus.FAILED
        async with self._database.write_lock:
            async with self._transaction() as connection:
                await connection.execute(
                    "UPDATE task_execution SET status = ? "
                    "WHERE execution_id = ? AND status = ? AND navigation_sent = 1 "
                    "AND task_id IN (SELECT task_id FROM task_definition WHERE robot_id = ?)",
                    (status.value, execution_id, ExecutionStatus.RUNNING.value, self._robot_id),
                )

    async def mark_active_cancel_pending(self, generation: int) -> None:
        async with self._database.write_lock:
            async with self._transaction() as connection:
                await connection.execute(
                    "UPDATE task_execution SET status = ? WHERE status IN (?, ?) AND task_id IN "
                    "(SELECT task_id FROM task_definition WHERE robot_id = ?) AND execution_id IN "
                    "(SELECT execution_id FROM task_execution_generation WHERE generation = ?)",
                    (
                        ExecutionStatus.CANCEL_PENDING.value,
                        ExecutionStatus.RUNNING.value,
                        ExecutionStatus.DISPATCHING.value,
                        self._robot_id,
                        generation,
                    ),
                )

    async def mark_cancel_pending(self, execution_id: str) -> ExecutionRecord:
        async with self._database.write_lock:
            async with self._transaction() as connection:
                current = await self._execution_in(connection, execution_id)
                if current is None or current.status not in {
                    ExecutionStatus.DISPATCHING,
                    ExecutionStatus.RUNNING,
                    ExecutionStatus.CANCEL_PENDING,
                }:
                    raise DomainError(ErrorCode.CONFLICT, "execution is not in flight")
                await connection.execute(
                    "UPDATE task_execution SET status = ? WHERE execution_id = ?",
                    (ExecutionStatus.CANCEL_PENDING.value, execution_id),
                )
                stored = await self._execution_in(connection, execution_id)
        assert stored is not None
        return stored

    async def pause(self, execution_id: str) -> ExecutionRecord:
        if not execution_id.strip():
            raise DomainError(ErrorCode.VALIDATION, "execution id is required")
        async with self._database.write_lock:
            async with self._transaction() as connection:
                current = await self._execution_in(connection, execution_id)
                if current is None:
                    raise DomainError(ErrorCode.CONFLICT, "execution does not exist")
                if current.status is not ExecutionStatus.PAUSED:
                    if current.status not in {
                        ExecutionStatus.DISPATCHING,
                        ExecutionStatus.RUNNING,
                        ExecutionStatus.CANCEL_PENDING,
                    }:
                        raise DomainError(ErrorCode.CONFLICT, "execution is not in flight")
                    await connection.execute(
                        """
                        UPDATE task_execution
                        SET status = ?
                        WHERE execution_id = ? AND status IN (?, ?, ?)
                        """,
                        (
                            ExecutionStatus.PAUSED.value,
                            execution_id,
                            ExecutionStatus.DISPATCHING.value,
                            ExecutionStatus.RUNNING.value,
                            ExecutionStatus.CANCEL_PENDING.value,
                        ),
                    )
                stored = await self._execution_in(connection, execution_id)
        if stored is None or stored.status is not ExecutionStatus.PAUSED:
            raise DomainError(ErrorCode.CONFLICT, "execution was not paused")
        return stored

    async def skip_occurrence(
        self, task: TaskDefinition, occurrence_utc: datetime
    ) -> ExecutionRecord:
        self._require_robot(task)
        occurrence = _utc(occurrence_utc)
        execution_id = uuid.uuid4().hex
        try:
            async with self._database.write_lock:
                async with self._transaction() as connection:
                    current = await self._task_in(connection, task.task_id)
                    if current is None:
                        raise DomainError(ErrorCode.CONFLICT, "task does not exist")
                    await connection.execute(
                        """
                        INSERT INTO task_execution (
                            execution_id, task_id, occurrence_utc, status,
                            navigation_sent, rollback_confirmed
                        )
                        VALUES (?, ?, ?, ?, 0, 0)
                        """,
                        (
                            execution_id,
                            task.task_id,
                            _stamp(occurrence),
                            ExecutionStatus.SKIPPED.value,
                        ),
                    )
                    stored = await self._execution_in(connection, execution_id)
        except sqlite3.IntegrityError as exc:
            raise DomainError(ErrorCode.CONFLICT, "duplicate execution") from exc
        if stored is None:
            raise DomainError(ErrorCode.CONFLICT, "skip was not stored")
        return stored

    async def mark_needs_review(self, execution_id: str) -> ExecutionRecord:
        return await self._mark(
            execution_id,
            ExecutionStatus.NEEDS_REVIEW,
            navigation_sent=False,
            rollback_confirmed=False,
        )

    async def execution_for(self, task_id: str, occurrence_utc: datetime) -> ExecutionRecord | None:
        async with self._database.write_lock:
            connection = self._database.connection
            await connection.rollback()
            async with connection.execute(
                """
                SELECT e.execution_id, e.task_id, e.occurrence_utc, e.status,
                       e.navigation_sent, e.rollback_confirmed
                FROM task_execution AS e
                JOIN task_definition AS t ON t.task_id = e.task_id
                WHERE e.task_id = ? AND e.occurrence_utc = ? AND t.robot_id = ?
                """,
                (task_id, _stamp(_utc(occurrence_utc)), self._robot_id),
            ) as cursor:
                row = await cursor.fetchone()
            await connection.rollback()
        if row is None:
            return None
        return _execution(row)

    async def execution_by_id(self, execution_id: str) -> ExecutionRecord | None:
        async with self._database.write_lock:
            connection = self._database.connection
            await connection.rollback()
            stored = await self._execution_in(connection, execution_id)
            await connection.rollback()
        return stored

    async def record_skip(
        self,
        task_id: str,
        skip_from: datetime,
        skip_until: datetime,
        reason: str,
        # //Compatible: existing callers omit the id and receive a fresh record.
        *,
        skip_id: str | None = None,
    ) -> SkipRecord:
        if not reason.strip():
            raise DomainError(ErrorCode.VALIDATION, "skip reason is required")
        skip_id = uuid.uuid4().hex if skip_id is None else skip_id
        if not skip_id.strip():
            raise DomainError(ErrorCode.VALIDATION, "skip identity is required")
        async with self._database.write_lock:
            async with self._transaction() as connection:
                current = await self._task_in(connection, task_id)
                if current is None:
                    raise DomainError(ErrorCode.CONFLICT, "task does not exist")
                await connection.execute(
                    """
                    INSERT INTO task_skip (skip_id, task_id, skip_from, skip_until, reason)
                    VALUES (?, ?, ?, ?, ?)
                    ON CONFLICT(skip_id) DO UPDATE SET
                        skip_from = MIN(task_skip.skip_from, excluded.skip_from),
                        skip_until = MAX(task_skip.skip_until, excluded.skip_until)
                    WHERE task_skip.task_id = excluded.task_id
                      AND task_skip.reason = excluded.reason
                    """,
                    (skip_id, task_id, _stamp(_utc(skip_from)), _stamp(_utc(skip_until)), reason),
                )
                stored = await self._skip_in(connection, skip_id)
                if stored is not None and (stored.task_id != task_id or stored.reason != reason):
                    raise DomainError(
                        ErrorCode.CONFLICT, "skip identity belongs to another interval"
                    )
        if stored is None:
            raise DomainError(ErrorCode.VALIDATION, "skip was not stored")
        return stored

    async def skips(self) -> tuple[SkipRecord, ...]:
        async with self._database.write_lock:
            connection = self._database.connection
            await connection.rollback()
            async with connection.execute(
                """
                SELECT s.skip_id, s.task_id, s.skip_from, s.skip_until, s.reason
                FROM task_skip AS s
                JOIN task_definition AS t ON t.task_id = s.task_id
                WHERE t.robot_id = ?
                ORDER BY s.skip_from, s.skip_id
                """,
                (self._robot_id,),
            ) as cursor:
                rows = await cursor.fetchall()
            await connection.rollback()
        return tuple(_skip(row) for row in rows)

    async def _mark(
        self,
        execution_id: str,
        status: ExecutionStatus,
        *,
        navigation_sent: bool,
        rollback_confirmed: bool,
    ) -> ExecutionRecord:
        async with self._database.write_lock:
            async with self._transaction() as connection:
                await connection.execute(
                    """
                    UPDATE task_execution
                    SET status = ?, navigation_sent = ?, rollback_confirmed = ?
                    WHERE execution_id = ?
                      AND task_id IN (
                          SELECT task_id FROM task_definition WHERE robot_id = ?
                      )
                    """,
                    (
                        status.value,
                        int(navigation_sent),
                        int(rollback_confirmed),
                        execution_id,
                        self._robot_id,
                    ),
                )
                stored = await self._execution_in(connection, execution_id)
        if stored is None:
            raise DomainError(ErrorCode.CONFLICT, "execution does not exist")
        return stored

    def _require_robot(self, task: TaskDefinition) -> None:
        if task.robot_id != self._robot_id:
            raise DomainError(ErrorCode.VALIDATION, "robot mismatch")

    async def _insert_task(
        self,
        connection: aiosqlite.Connection,
        task: TaskDefinition,
        *,
        revision: int,
    ) -> None:
        await connection.execute(
            """
            INSERT INTO task_definition (
                task_id, robot_id, map_id, reference_revision, waypoint_ids,
                schedule_kind, every_seconds, cron_expression, timezone,
                enabled, revision, created_at
            )
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            """,
            (
                task.task_id,
                task.robot_id,
                task.map_ref.map_id,
                task.map_ref.reference_revision,
                _points(task),
                task.schedule.kind,
                task.schedule.every_seconds,
                task.schedule.expression,
                task.timezone,
                int(task.enabled),
                revision,
                _stamp(task.created_at),
            ),
        )

    async def _bind_link(self, connection: aiosqlite.Connection, task: TaskDefinition) -> None:
        await connection.execute(
            """
            INSERT INTO map_task_link (
                task_id, definition, map_id, reference_revision, invalidated
            )
            VALUES (?, '', ?, ?, 0)
            ON CONFLICT(task_id) DO UPDATE SET
                map_id = excluded.map_id,
                reference_revision = excluded.reference_revision,
                invalidated = 0
            """,
            (task.task_id, task.map_ref.map_id, task.map_ref.reference_revision),
        )

    async def _in_flight(self, connection: aiosqlite.Connection) -> bool:
        async with connection.execute(
            """
            SELECT 1
            FROM task_execution AS e
            JOIN task_definition AS t ON t.task_id = e.task_id
            WHERE t.robot_id = ? AND e.status IN (?, ?, ?)
            LIMIT 1
            """,
            (
                self._robot_id,
                ExecutionStatus.DISPATCHING.value,
                ExecutionStatus.RUNNING.value,
                ExecutionStatus.CANCEL_PENDING.value,
            ),
        ) as cursor:
            row = await cursor.fetchone()
        return row is not None

    async def _deleted(self, connection: aiosqlite.Connection, task_id: str) -> bool:
        try:
            async with connection.execute(
                "SELECT 1 FROM agent_deleted_task WHERE task_id = ? AND robot_id = ?",
                (task_id, self._robot_id),
            ) as cursor:
                row = await cursor.fetchone()
        except sqlite3.OperationalError as exc:
            # //Compatible: standalone TaskStore users may not have an agent journal.
            if str(exc) != "no such table: agent_deleted_task":
                raise
            return False
        return row is not None

    async def _invalidated(self, connection: aiosqlite.Connection, task_id: str) -> bool:
        async with connection.execute(
            "SELECT invalidated FROM map_task_link WHERE task_id = ?",
            (task_id,),
        ) as cursor:
            row = await cursor.fetchone()
        if row is None:
            return True
        return _flag(row[0])

    async def _task_in(
        self, connection: aiosqlite.Connection, task_id: str
    ) -> TaskDefinition | None:
        async with connection.execute(
            """
            SELECT task_id, robot_id, map_id, reference_revision, waypoint_ids,
                   schedule_kind, every_seconds, cron_expression, timezone,
                   enabled, revision, created_at
            FROM task_definition
            WHERE task_id = ? AND robot_id = ?
            """,
            (task_id, self._robot_id),
        ) as cursor:
            row = await cursor.fetchone()
        if row is None:
            return None
        return _task(row)

    async def _execution_in(
        self, connection: aiosqlite.Connection, execution_id: str
    ) -> ExecutionRecord | None:
        async with connection.execute(
            """
            SELECT e.execution_id, e.task_id, e.occurrence_utc, e.status,
                   e.navigation_sent, e.rollback_confirmed
            FROM task_execution AS e
            JOIN task_definition AS t ON t.task_id = e.task_id
            WHERE e.execution_id = ? AND t.robot_id = ?
            """,
            (execution_id, self._robot_id),
        ) as cursor:
            row = await cursor.fetchone()
        if row is None:
            return None
        return _execution(row)

    async def _skip_in(self, connection: aiosqlite.Connection, skip_id: str) -> SkipRecord | None:
        async with connection.execute(
            """
            SELECT s.skip_id, s.task_id, s.skip_from, s.skip_until, s.reason
            FROM task_skip AS s
            JOIN task_definition AS t ON t.task_id = s.task_id
            WHERE s.skip_id = ? AND t.robot_id = ?
            """,
            (skip_id, self._robot_id),
        ) as cursor:
            row = await cursor.fetchone()
        if row is None:
            return None
        return _skip(row)

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


def _task(row: aiosqlite.Row) -> TaskDefinition:
    created = _moment(_text(row[11]))
    kind = _text(row[5])
    timezone_name = _text(row[8])
    every = _optional_whole(row[6])
    expression = _optional_text(row[7])
    if kind == "interval":
        if every is None:
            raise DomainError(ErrorCode.VALIDATION, "interval is incomplete")
        schedule = Schedule.interval(created, every, timezone_name)
    elif kind == "cron":
        if expression is None:
            raise DomainError(ErrorCode.VALIDATION, "cron is incomplete")
        schedule = Schedule.cron(expression, timezone_name)
    else:
        raise DomainError(ErrorCode.VALIDATION, "unknown schedule")
    return TaskDefinition(
        _text(row[0]),
        _text(row[1]),
        MapRef(_text(row[2]), _whole(row[3])),
        _waypoint_ids(_text(row[4])),
        schedule,
        timezone_name,
        _flag(row[9]),
        _whole(row[10]),
        created,
    )


def _execution(row: aiosqlite.Row) -> ExecutionRecord:
    return ExecutionRecord(
        _text(row[0]),
        _text(row[1]),
        _moment(_text(row[2])),
        ExecutionStatus(_text(row[3])),
        _flag(row[4]),
        _flag(row[5]),
    )


def _skip(row: aiosqlite.Row) -> SkipRecord:
    return SkipRecord(
        _text(row[0]),
        _text(row[1]),
        _moment(_text(row[2])),
        _moment(_text(row[3])),
        _text(row[4]),
    )


def _points(task: TaskDefinition) -> str:
    return json.dumps(list(task.waypoint_ids), separators=(",", ":"))


def _waypoint_ids(raw: str) -> tuple[str, ...]:
    parsed = json.loads(raw)
    if not isinstance(parsed, list) or any(not isinstance(item, str) for item in parsed):
        raise DomainError(ErrorCode.VALIDATION, "waypoints are invalid")
    return tuple(parsed)


def _stamp(moment: datetime) -> str:
    return _utc(moment).isoformat()


def _moment(value: str) -> datetime:
    parsed = datetime.fromisoformat(value)
    if parsed.tzinfo is None:
        raise DomainError(ErrorCode.VALIDATION, "timestamp must be timezone-aware")
    return parsed.astimezone(timezone.utc)


def _utc(moment: datetime) -> datetime:
    if moment.tzinfo is None or moment.utcoffset() is None:
        raise DomainError(ErrorCode.VALIDATION, "timestamp must be timezone-aware")
    return moment.astimezone(timezone.utc)


def _text(value: object) -> str:
    if not isinstance(value, str):
        raise DomainError(ErrorCode.VALIDATION, "expected text")
    return value


def _optional_text(value: object) -> str | None:
    if value is None:
        return None
    return _text(value)


def _whole(value: object) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise DomainError(ErrorCode.VALIDATION, "expected integer")
    return value


def _optional_whole(value: object) -> int | None:
    if value is None:
        return None
    return _whole(value)


def _flag(value: object) -> bool:
    if isinstance(value, bool):
        return value
    if value == 0:
        return False
    if value == 1:
        return True
    raise DomainError(ErrorCode.VALIDATION, "expected flag")
