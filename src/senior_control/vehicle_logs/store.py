"""Dedicated log-only SQLite IO; never uses settings/control Database or its lock."""

import asyncio
import sqlite3
from pathlib import Path
from types import TracebackType

import aiosqlite

from senior_control.vehicle_logs.models import (
    DISK_BYTES,
    PAGE_BYTES,
    LogEntry,
    LogGap,
    LogPage,
    LogQuery,
    redact_fields_json,
    redact_text,
)


class VehicleLogStore:
    """Trusted composition injects a private directory, never a request path.

    MEMORY rollback journal avoids unbounded WAL/journal disk growth; SQLite page
    count hard-caps the sole disk file. Logs are best effort, not a safety journal:
    power loss/corruption is an explicit unavailable error, never silent recreation.
    Logical payload uses 1/4 of the cap to leave room for pages/index/fragmentation.
    """

    def __init__(self, directory: Path, *, disk_limit_bytes: int = DISK_BYTES) -> None:
        if type(disk_limit_bytes) is not int or not 65536 <= disk_limit_bytes <= DISK_BYTES:
            raise ValueError("invalid internal log capacity")
        self._path = directory / "vehicle-logs.sqlite3"
        self._disk_limit = disk_limit_bytes
        self._payload_limit = disk_limit_bytes // 4
        self._lock = asyncio.Lock()
        self._connection: aiosqlite.Connection

    async def __aenter__(self) -> "VehicleLogStore":
        self._path.parent.mkdir(parents=True, exist_ok=True)
        if self._path.is_symlink() or (
            self._path.exists() and self._path.stat().st_size > self._disk_limit
        ):
            raise ValueError("log storage unavailable: unsafe/oversize file")
        fresh = not self._path.exists()
        try:
            connection = await aiosqlite.connect(self._path)
        except sqlite3.OperationalError as exc:
            # Failed opens allocate no usable connection; aiosqlite stops its worker.
            # Keep this storage boundary consistent with post-open validation errors.
            raise ValueError("log storage unavailable: cannot open database") from exc
        self._connection = connection
        try:
            for statement in (
                "PRAGMA page_size=4096",
                "PRAGMA journal_mode=MEMORY",
                "PRAGMA synchronous=FULL",
                "PRAGMA temp_store=MEMORY",
                "PRAGMA cache_size=-1024",
                "PRAGMA busy_timeout=100",
                f"PRAGMA max_page_count={self._disk_limit // 4096}",
            ):
                await connection.execute(statement)
            async with connection.execute("PRAGMA page_size") as cursor:
                page_size = await cursor.fetchone()
            if page_size is None or page_size[0] != 4096:
                raise ValueError("log storage unavailable: unsupported page size")
            if fresh:
                await self._initialize()
            await self._validate_existing()
        except BaseException as exc:
            await connection.rollback()
            await connection.close()
            if isinstance(exc, (sqlite3.Error, ValueError)):
                raise ValueError("log storage unavailable: invalid existing state") from exc
            raise
        return self

    async def _initialize(self) -> None:
        connection = self._connection
        await connection.execute("BEGIN IMMEDIATE")
        await connection.execute(
            "CREATE TABLE log_meta "
            "(id INTEGER PRIMARY KEY CHECK(id=1), version INTEGER NOT NULL, "
            "high_water INTEGER NOT NULL, floor INTEGER NOT NULL, payload_bytes INTEGER NOT NULL)"
        )
        await connection.execute(
            "CREATE TABLE log_entry "
            "(sequence INTEGER PRIMARY KEY, received_utc REAL NOT NULL, payload TEXT NOT NULL, "
            "wire_bytes INTEGER NOT NULL)"
        )
        await connection.execute("INSERT INTO log_meta VALUES (1,1,0,0,0)")
        await connection.commit()

    async def _validate_existing(self) -> None:
        """One startup-only scan, bounded by the log file's physical size cap.

        Never reconstruct metadata from records. A valid retained stream is exactly
        floor+1..high_water, or empty with floor==high_water after complete pruning.
        """
        connection = self._connection
        for table, columns in (
            ("log_meta", ("id", "version", "high_water", "floor", "payload_bytes")),
            ("log_entry", ("sequence", "received_utc", "payload", "wire_bytes")),
        ):
            async with connection.execute(f"PRAGMA table_info({table})") as cursor:
                schema = list(await cursor.fetchall())
            if tuple(row[1] for row in schema) != columns or tuple(row[5] for row in schema) != (
                1,
            ) + (0,) * (len(columns) - 1):
                raise ValueError("incomplete log schema")
        async with connection.execute(
            "SELECT id,version,high_water,floor,payload_bytes FROM log_meta LIMIT 2"
        ) as cursor:
            metadata = list(await cursor.fetchall())
        if len(metadata) != 1 or any(type(value) is not int for value in metadata[0]):
            raise ValueError("missing or invalid log metadata")
        identifier, version, high, floor, total = metadata[0]
        if (
            identifier != 1
            or version != 1
            or not 0 <= floor <= high <= 2**63 - 1
            or not 0 <= total <= self._payload_limit
        ):
            raise ValueError("invalid log metadata counters")
        async with connection.execute(
            "SELECT COUNT(*),MIN(sequence),MAX(sequence),COALESCE(SUM(wire_bytes),0),"
            "COALESCE(SUM(CASE WHEN typeof(wire_bytes) != 'integer' OR wire_bytes <= 0 "
            "OR wire_bytes != length(CAST(payload AS BLOB)) THEN 1 ELSE 0 END),0) "
            "FROM log_entry"
        ) as cursor:
            records = await cursor.fetchone()
        assert records is not None
        count, first, last, payload_bytes, invalid = records
        if (
            count != high - floor
            or payload_bytes != total
            or invalid
            or (count > 0 and (first != floor + 1 or last != high))
        ):
            raise ValueError("log metadata disagrees with retained records")

    async def __aexit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        tb: TracebackType | None,
    ) -> None:
        await self._connection.close()

    async def _meta(self) -> tuple[int, int, int]:
        async with self._connection.execute(
            "SELECT high_water, floor, payload_bytes FROM log_meta WHERE id=1"
        ) as cursor:
            row = await cursor.fetchone()
        if row is None:
            raise ValueError("log storage unavailable: missing metadata")
        return int(row[0]), int(row[1]), int(row[2])

    async def append(self, entries: tuple[LogEntry, ...]) -> tuple[LogEntry, ...]:
        if not entries or len(entries) > 64:
            raise ValueError("invalid log write batch")
        async with self._lock:
            connection = self._connection
            await connection.execute("BEGIN IMMEDIATE")
            try:
                high, floor, total = await self._meta()
                committed: list[LogEntry] = []
                for item in entries:
                    high += 1
                    entry = item.model_copy(update={"sequence": high})
                    payload = entry.model_dump_json()
                    size = len(payload.encode("utf-8"))
                    if size > self._payload_limit:
                        raise ValueError("record exceeds internal log capacity")
                    while total + size > self._payload_limit:
                        async with connection.execute(
                            "SELECT sequence,wire_bytes FROM log_entry ORDER BY sequence LIMIT 1"
                        ) as cursor:
                            oldest = await cursor.fetchone()
                        if oldest is None:
                            raise ValueError("log storage unavailable: byte accounting")
                        floor = int(oldest[0])
                        total -= int(oldest[1])
                        await connection.execute("DELETE FROM log_entry WHERE sequence=?", (floor,))
                    await connection.execute(
                        "INSERT INTO log_entry VALUES (?,?,?,?)",
                        (
                            high,
                            entry.received_utc,
                            payload,
                            size,
                        ),
                    )
                    total += size
                    committed.append(entry)
                await connection.execute(
                    "UPDATE log_meta SET high_water=?,floor=?,payload_bytes=? WHERE id=1",
                    (high, floor, total),
                )
                await connection.commit()
            except BaseException:
                await connection.rollback()
                raise
        return tuple(committed)

    async def prune(self, cutoff_utc: float) -> None:
        async with self._lock:
            connection = self._connection
            await connection.execute("BEGIN IMMEDIATE")
            try:
                async with connection.execute(
                    "SELECT MAX(sequence) FROM log_entry WHERE received_utc < ?", (cutoff_utc,)
                ) as cursor:
                    row = await cursor.fetchone()
                through = row[0] if row is not None else None
                if through is not None:
                    async with connection.execute(
                        "SELECT COALESCE(SUM(wire_bytes),0) FROM log_entry WHERE sequence<=?",
                        (through,),
                    ) as cursor:
                        sizes = await cursor.fetchone()
                    assert sizes is not None
                    await connection.execute("DELETE FROM log_entry WHERE sequence<=?", (through,))
                    await connection.execute(
                        "UPDATE log_meta SET floor=MAX(floor,?),payload_bytes=payload_bytes-? "
                        "WHERE id=1",
                        (through, sizes[0]),
                    )
                await connection.commit()
            except BaseException:
                await connection.rollback()
                raise

    async def query(self, query: LogQuery) -> LogPage:
        # Scan at most 200 physical rows, even when filters match nothing. An empty
        # page may have has_more=True; its cursor still makes finite progress.
        async with self._lock:
            high, floor, _ = await self._meta()
            if query.after > high:
                raise ValueError("cursor ahead of this vehicle stream")
            after = max(query.after, floor)
            gaps = (
                ()
                if query.after >= floor
                else (LogGap(reason="evicted", after=query.after, through=floor, durable=True),)
            )
            async with self._connection.execute(
                "SELECT sequence,payload FROM log_entry WHERE sequence>? "
                "ORDER BY sequence LIMIT 200",
                (after,),
            ) as cursor:
                rows = await cursor.fetchall()
            entries: list[LogEntry] = []
            size, scanned = 8192, 0  # conservative envelope/gap budget
            for sequence, payload in rows:
                # Defence in depth: exports and reads redact legacy/tampered strings too.
                entry = LogEntry.model_validate_json(str(payload))
                entry = entry.model_copy(
                    update={
                        "summary": redact_text(entry.summary),
                        "detail": redact_text(entry.detail),
                        "fields_json": redact_fields_json(entry.fields_json),
                        "module": redact_text(entry.module),
                        "request_id": redact_text(entry.request_id) if entry.request_id else None,
                        "task_id": redact_text(entry.task_id) if entry.task_id else None,
                    }
                )
                if entry.sequence != sequence:
                    raise ValueError("log storage unavailable: sequence corruption")
                encoded_size = len(entry.model_dump_json().encode("utf-8")) + 1
                matches = query.matches(entry)
                if matches and (len(entries) >= query.limit or size + encoded_size > PAGE_BYTES):
                    break
                after = int(sequence)
                scanned += 1
                if matches:
                    entries.append(entry)
                    size += encoded_size
            return LogPage(
                entries=tuple(entries),
                gaps=gaps,
                next_cursor=after,
                high_water=high,
                has_more=after < high,
                scanned=scanned,
            )
