"""Authenticated, bounded logging core. No control/settings IO or debug timer."""

import asyncio
import queue
import sqlite3
import threading
import time
from collections.abc import AsyncIterator, Callable
from contextlib import asynccontextmanager, suppress
from typing import Literal, Protocol

from senior_control.settings.models import SettingsSnapshot
from senior_control.vehicle_logs.models import (
    DOWNLOAD_BYTES,
    LEVEL_WEIGHT,
    PAGE_BYTES,
    QUEUE_LIMIT,
    LogDownload,
    LogEntry,
    LogGap,
    LogInput,
    LogPage,
    LogQuery,
    LossReason,
    prepare_entry,
)
from senior_control.vehicle_logs.store import VehicleLogStore

Action = Literal["query", "download", "follow"]


class LogAuthorizer(Protocol):
    async def require_vehicle(self, credential: str, vehicle_id: str, action: Action) -> None:
        """Trusted port verifies the credential's current identity/vehicle grant.

        Authentication comes from composition's actual session verifier. Caller
        booleans or a caller-provided principal ID are never access decisions.
        Raise PermissionError if the session is expired, revoked or unauthorized.
        """
        ...


class LogSubscription:
    """Each consumer has at most four pages; credentials are never in payloads."""

    def __init__(
        self,
        service: "VehicleLogService",
        credential: str,
        query: LogQuery,
    ) -> None:
        self._service = service
        self._credential = credential
        self.query = query
        self._live_after = query.after
        self._outbox: asyncio.Queue[LogPage | None] = asyncio.Queue(maxsize=4)
        self._closed = False

    @property
    def outbox_depth(self) -> int:
        return self._outbox.qsize()

    async def next_page(self) -> LogPage:
        if self._closed:
            raise RuntimeError("log subscription closed")
        try:
            await self._service._authorize(self._credential, self._service.vehicle_id, "follow")
            page = await self._outbox.get()
            if page is None:
                raise RuntimeError("log subscription closed")
            # Recheck a credential revoked while waiting for a new page.
            await self._service._authorize(self._credential, self._service.vehicle_id, "follow")
            return page
        except PermissionError:
            self._close()
            raise

    def _offer(self, page: LogPage) -> None:
        if self._closed:
            return
        if self._outbox.full():
            earliest = page.next_cursor
            while not self._outbox.empty():
                discarded = self._outbox.get_nowait()
                if discarded is not None:
                    for gap in discarded.gaps:
                        if gap.after is not None:
                            earliest = min(earliest, gap.after)
                    earliest = min(
                        earliest,
                        discarded.entries[0].sequence - 1
                        if discarded.entries
                        else discarded.next_cursor,
                    )
            page = page.model_copy(
                update={
                    "gaps": page.gaps
                    + (
                        LogGap(
                            reason="slow_subscriber",
                            after=earliest,
                            through=(
                                page.entries[0].sequence - 1 if page.entries else page.next_cursor
                            ),
                        ),
                    ),
                }
            )
        self._outbox.put_nowait(page)

    async def close(self) -> None:
        self._close()

    def _close(self) -> None:
        self._closed = True
        self._service._subscriptions.discard(self)
        while not self._outbox.empty():
            self._outbox.get_nowait()
        self._outbox.put_nowait(None)


class VehicleLogService:
    """enqueue is synchronous, bounded and thread-safe, with no filesystem access.

    The sole writer and retention worker use a separate log connection. Persisted
    gap records coalesce lost input by closed reason. While disk is broken, queries
    expose undurable losses separately and never claim a complete log history.
    """

    def __init__(
        self,
        store: VehicleLogStore,
        vehicle_id: str,
        authorizer: LogAuthorizer,
        settings: Callable[[], SettingsSnapshot],
        *,
        utc_now: Callable[[], float] = time.time,
    ) -> None:
        if not vehicle_id or len(vehicle_id) > 128:
            raise ValueError("invalid vehicle ID")
        self.vehicle_id = vehicle_id
        self._store = store
        self._authorizer = authorizer
        self._settings = settings
        self._utc_now = utc_now
        self._queue: queue.Queue[LogEntry] = queue.Queue(maxsize=QUEUE_LIMIT)
        self._mutex = threading.Lock()
        self._losses: dict[LossReason, int] = {}
        self._signalled = False
        self._accepting = False
        self._wake = asyncio.Event()
        self._idle = asyncio.Event()
        self._loop: asyncio.AbstractEventLoop
        self._writer: asyncio.Task[None] | None = None
        self._cleaner: asyncio.Task[None] | None = None
        self._publication_lock = asyncio.Lock()
        self._inflight: dict[LossReason, int] = {}
        self._subscriptions: set[LogSubscription] = set()
        self._queries = 0
        self._write_error: str | None = None
        self._cleanup_error: str | None = None

    @property
    def storage_error(self) -> str | None:
        return self._write_error or self._cleanup_error

    @property
    def queue_depth(self) -> int:
        return self._queue.qsize()

    async def start(self) -> None:
        if self._writer is not None:
            raise RuntimeError("log service already started")
        self._loop = asyncio.get_running_loop()
        initial = await self._store.query(LogQuery(limit=1))
        if initial.high_water:
            # A crash may lose an unflushed queue. Do not invent an exact loss count.
            self._losses["restart_unknown"] = 0
        self._accepting = True
        self._writer = asyncio.create_task(self._write_loop())
        self._cleaner = asyncio.create_task(self._cleanup_loop())
        self._signal()

    def _signal(self) -> None:
        self._idle.clear()
        self._wake.set()

    def _schedule(self) -> None:
        # Caller holds _mutex. At most one cross-thread callback is outstanding,
        # even under a continuous producer flood while the event loop is busy.
        if not self._signalled:
            self._signalled = True
            self._loop.call_soon_threadsafe(self._signal)

    def _lose(self, reason: LossReason, count: int = 1) -> None:
        with self._mutex:
            self._losses[reason] = min(2**63 - 1, self._losses.get(reason, 0) + count)
            self._schedule()

    def enqueue(self, value: LogInput) -> bool:
        if not self._accepting:
            return False
        try:
            policy = self._settings()
            if value.level not in LEVEL_WEIGHT:
                raise ValueError("unknown log level")
            if LEVEL_WEIGHT[value.level] < LEVEL_WEIGHT[policy.effective_log_level]:
                return False  # intentional policy filtering, not lost data
            with self._mutex:
                if self._queue.full():
                    self._losses["queue_full"] = min(
                        2**63 - 1,
                        self._losses.get("queue_full", 0) + 1,
                    )
                    self._schedule()
                    return False
            entry = prepare_entry(value, self._utc_now())
        except OverflowError:
            self._lose("oversize")
            return False
        except (ValueError, TypeError):
            self._lose("invalid")
            return False
        with self._mutex:
            # Recheck while holding the producer lock so close can fence enqueue.
            if not self._accepting:
                return False
            try:
                self._queue.put_nowait(entry)
            except queue.Full:
                self._losses["queue_full"] = min(
                    2**63 - 1,
                    self._losses.get("queue_full", 0) + 1,
                )
                self._schedule()
                return False
            self._schedule()
        return True

    def _batch(self) -> tuple[tuple[LogEntry, ...], dict[LossReason, int]]:
        with self._mutex:
            self._signalled = False
            losses, self._losses = self._losses, {}
            self._inflight = losses.copy()
        now = self._utc_now()
        entries = [
            LogEntry(
                kind="gap",
                level="WARNING",
                summary="log input lost",
                source_utc=now,
                received_utc=now,
                loss_reason=reason,
                loss_count=None if reason == "restart_unknown" else count,
            )
            for reason, count in losses.items()
        ]
        while len(entries) < 64:
            try:
                entries.append(self._queue.get_nowait())
            except queue.Empty:
                break
        return tuple(entries), losses

    async def _write_loop(self) -> None:
        while True:
            await self._wake.wait()
            self._wake.clear()
            entries, losses = self._batch()
            if not entries:
                with self._mutex:
                    if not self._losses and self._queue.empty():
                        self._idle.set()
                    else:
                        self._wake.set()
                continue
            try:
                async with self._publication_lock:
                    committed = await self._store.append(entries)
                    await self._publish(committed)
                self._write_error = None
            except (sqlite3.Error, OSError, ValueError):
                # Never include exception text: it may itself contain credentials.
                self._write_error = "log_storage_unavailable"
                with self._mutex:
                    self._inflight = {}
                    for reason, count in losses.items():
                        self._losses[reason] = min(
                            2**63 - 1,
                            self._losses.get(reason, 0) + count,
                        )
                    self._losses["storage_error"] = min(
                        2**63 - 1,
                        self._losses.get("storage_error", 0)
                        + sum(entry.kind == "event" for entry in entries),
                    )
                    self._schedule()
                await asyncio.sleep(0.05)
            finally:
                with self._mutex:
                    self._inflight = {}
            self._wake.set()

    @staticmethod
    def _live_pages(entries: tuple[LogEntry, ...], high: int, limit: int) -> list[LogPage]:
        pages: list[LogPage] = []
        current: list[LogEntry] = []
        # _offer may add a coalesced gap. Include its largest possible envelope in
        # byte accounting now, so a full page stays bounded even after overflow.
        reserve = LogGap(reason="slow_subscriber", after=2**63 - 1, through=2**63 - 1)
        for entry in entries:
            candidate = LogPage(
                entries=tuple(current) + (entry,),
                next_cursor=high,
                high_water=high,
                has_more=False,
                gaps=(reserve,),
            )
            if current and (
                len(current) >= limit
                or len(candidate.model_dump_json().encode("utf-8")) > PAGE_BYTES
            ):
                pages.append(
                    LogPage(
                        entries=tuple(current),
                        next_cursor=current[-1].sequence,
                        high_water=high,
                        has_more=True,
                    )
                )
                current = []
            current.append(entry)
        if current:
            # Any trailing filtered rows are intentionally skipped, not missing.
            pages.append(
                LogPage(entries=tuple(current), next_cursor=high, high_water=high, has_more=False)
            )
        return pages

    async def _publish(self, entries: tuple[LogEntry, ...]) -> None:
        high = entries[-1].sequence
        for subscription in tuple(self._subscriptions):
            matches = tuple(
                entry
                for entry in entries
                if entry.sequence > subscription._live_after and subscription.query.matches(entry)
            )
            for page in self._live_pages(matches, high, subscription.query.limit):
                subscription._offer(page)
                # Give a waiting consumer and control work a turn between genuinely
                # necessary pages; never wait for an outbox slot or consumer ack.
                await asyncio.sleep(0)
            subscription._live_after = high

    async def flush(self, *, timeout: float = 5.0) -> None:
        if self._writer is None:
            raise RuntimeError("log service not started")
        self._signal()
        await asyncio.wait_for(self._idle.wait(), timeout)
        if self._writer.done():
            self._writer.result()

    async def cleanup(self) -> None:
        retention = self._settings().logging.retention_seconds
        try:
            await self._store.prune(self._utc_now() - retention)
        except (sqlite3.Error, OSError, ValueError):
            self._cleanup_error = "log_cleanup_unavailable"
            raise
        self._cleanup_error = None

    async def _cleanup_loop(self) -> None:
        while True:
            await asyncio.sleep(30)
            try:
                await self.cleanup()
            except (sqlite3.Error, OSError, ValueError):
                # cleanup() exposes a durable-policy warning to authorized readers;
                # keep this independent worker alive to retry on its next interval.
                continue

    async def _authorize(self, credential: str, vehicle_id: str, action: Action) -> None:
        if vehicle_id != self.vehicle_id:
            raise PermissionError("vehicle access denied")
        await self._authorizer.require_vehicle(credential, vehicle_id, action)

    @asynccontextmanager
    async def _query_slot(self) -> AsyncIterator[None]:
        if self._queries >= 2:
            raise RuntimeError("log query resource quota exceeded")
        self._queries += 1
        try:
            yield
        finally:
            self._queries -= 1

    def _pending_gaps(self) -> tuple[LogGap, ...]:
        with self._mutex:
            combined = self._losses.copy()
            for reason, count in self._inflight.items():
                combined[reason] = combined.get(reason, 0) + count
            gaps = tuple(
                LogGap(
                    reason=reason,
                    count=None if reason == "restart_unknown" else count,
                    durable=False,
                )
                for reason, count in combined.items()
            )
        gaps += tuple(
            LogGap(reason=reason)
            for reason in (
                self._write_error,
                self._cleanup_error,
            )
            if reason is not None
        )
        return gaps

    async def query(self, credential: str, vehicle_id: str, query: LogQuery) -> LogPage:
        await self._authorize(credential, vehicle_id, "query")
        async with self._query_slot():
            page = await self._store.query(query)
            await self._authorize(credential, vehicle_id, "query")
            return page.model_copy(update={"gaps": page.gaps + self._pending_gaps()})

    async def download(
        self,
        credential: str,
        vehicle_id: str,
        query: LogQuery,
    ) -> LogDownload:
        await self._authorize(credential, vehicle_id, "download")
        async with self._query_slot():
            data = bytearray()
            gaps: dict[str, LogGap] = {}
            cursor, has_more = query.after, True
            # At most 3200 scanned rows plus 2MiB; sparse filters cannot run forever.
            for _ in range(16):
                await self._authorize(credential, vehicle_id, "download")
                page = await self._store.query(query.model_copy(update={"after": cursor}))
                await self._authorize(credential, vehicle_id, "download")
                for gap in page.gaps:
                    gaps[gap.reason] = gap
                for entry in page.entries:
                    encoded = entry.model_dump_json().encode("utf-8") + b"\n"
                    if len(data) + len(encoded) > DOWNLOAD_BYTES - 8192:
                        for pending in self._pending_gaps():
                            gaps[pending.reason] = pending
                        return LogDownload(bytes(data), cursor, True, tuple(gaps.values()))
                    data.extend(encoded)
                    cursor = entry.sequence
                cursor, has_more = page.next_cursor, page.has_more
                if not has_more:
                    break
            for gap in self._pending_gaps():
                gaps[gap.reason] = gap
            return LogDownload(bytes(data), cursor, has_more, tuple(gaps.values()))

    async def follow(
        self,
        credential: str,
        vehicle_id: str,
        query: LogQuery,
    ) -> LogSubscription:
        await self._authorize(credential, vehicle_id, "follow")
        if len(self._subscriptions) >= 32:
            raise RuntimeError("log subscription quota exceeded")
        async with self._query_slot():
            async with self._publication_lock:
                initial = await self._store.query(query)
                await self._authorize(credential, vehicle_id, "follow")
                if len(self._subscriptions) >= 32:
                    raise RuntimeError("log subscription quota exceeded")
                subscription = LogSubscription(self, credential, query)
                subscription._live_after = initial.high_water
                gaps = initial.gaps + self._pending_gaps()
                if initial.has_more:
                    gaps += (
                        LogGap(
                            reason="follow_backfill",
                            after=initial.next_cursor,
                            through=initial.high_water,
                            durable=True,
                        ),
                    )
                initial = initial.model_copy(update={"gaps": gaps})
                self._subscriptions.add(subscription)
                if initial.entries or initial.gaps:
                    subscription._offer(initial)
                return subscription

    async def close(self, *, timeout: float = 5.0) -> None:
        with self._mutex:
            self._accepting = False
        for subscription in tuple(self._subscriptions):
            subscription._close()
        if self._cleaner is not None:
            self._cleaner.cancel()
            with suppress(asyncio.CancelledError):
                await self._cleaner
            self._cleaner = None
        try:
            await self.flush(timeout=timeout)
        finally:
            if self._writer is not None:
                self._writer.cancel()
                with suppress(asyncio.CancelledError):
                    await self._writer
                self._writer = None
