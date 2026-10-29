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
        from senior_control.storage.migrations import migrate
        from senior_control.storage.schema import DOMAIN_MIGRATIONS

        await migrate(self._database, list(DOMAIN_MIGRATIONS))
        async with self._database.write_lock:
            async with self._transaction() as connection:
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
                            ExecutionStatus.NEEDS_REVIEW.value,
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

    async def claim(self, task: TaskDefinition, occurrence_utc: datetime) -> ExecutionRecord:
        self._require_robot(task)
        occurrence = _utc(occurrence_utc)
        execution_id = uuid.uuid4().hex
        try:
            async with self._database.write_lock:
                async with self._transaction() as connection:
                    current = await self._task_in(connection, task.task_id)
                    if current is None:
                        raise DomainError(ErrorCode.CONFLICT, "task does not exist")
                    if await self._invalidated(connection, task.task_id):
                        raise DomainError(ErrorCode.CONFLICT, "map link invalidated")
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
                    stored = await self._execution_in(connection, execution_id)
        except sqlite3.IntegrityError as exc:
            raise DomainError(ErrorCode.CONFLICT, "duplicate execution") from exc
        if stored is None:
            raise DomainError(ErrorCode.CONFLICT, "execution was not stored")
        return stored

    async def mark_running(self, execution_id: str) -> ExecutionRecord:
        return await self._mark(
            execution_id,
            ExecutionStatus.RUNNING,
            navigation_sent=True,
            rollback_confirmed=False,
        )

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
    ) -> SkipRecord:
        if not reason.strip():
            raise DomainError(ErrorCode.VALIDATION, "skip reason is required")
        skip_id = uuid.uuid4().hex
        async with self._database.write_lock:
            async with self._transaction() as connection:
                current = await self._task_in(connection, task_id)
                if current is None:
                    raise DomainError(ErrorCode.CONFLICT, "task does not exist")
                await connection.execute(
                    """
                    INSERT INTO task_skip (skip_id, task_id, skip_from, skip_until, reason)
                    VALUES (?, ?, ?, ?, ?)
                    """,
                    (skip_id, task_id, _stamp(_utc(skip_from)), _stamp(_utc(skip_until)), reason),
                )
                stored = await self._skip_in(connection, skip_id)
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
