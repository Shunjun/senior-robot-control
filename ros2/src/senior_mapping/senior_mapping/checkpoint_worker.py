"""Bounded deferred persistence, independent of ordered observation ingestion."""

import asyncio
import math
import time
from collections.abc import Awaitable, Callable


class CheckpointWorker:
    """One fixed-session capture at a time, with a single coalesced dirty watermark.

    ``save`` must persist a consistent cut through the graph/store transaction
    boundary. It runs on the mapping owner, never a motion callback. CPU work and
    disk I/O belong in that adapter's bounded executor. A dirty watermark is not
    permission to drop, reorder or merge raw observations/graph commits.

    Close/flush are lifecycle barriers. The caller must await close before
    activating another session; cancellation cannot prove durable completion.
    """

    def __init__(
        self,
        save: Callable[[int], Awaitable[None]],
        *,
        idle: Callable[[], bool],
        max_delay_s: float,
        retry_s: float = 1.0,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        if not all(math.isfinite(v) and v > 0 for v in (max_delay_s, retry_s)):
            raise ValueError("checkpoint delays must be finite and positive")
        self._save, self._idle, self._clock = save, idle, clock
        self._max_delay, self._retry = max_delay_s, retry_s
        self._dirty = self.saved_sequence = 0
        self._since: float | None = None
        self._forced = 0
        self._retry_at = 0.0
        self._wake = asyncio.Event()
        self._settled = asyncio.Condition()
        self._closing = self._closed = self._running = False
        self.last_error: Exception | None = None
        self._failed_sequence = 0

    def changed(self, sequence: int) -> None:
        if self._closing or self._closed:
            raise RuntimeError("checkpoint session is closing")
        if type(sequence) is not int or sequence <= self._dirty:
            raise ValueError("checkpoint watermark must advance")
        if self._dirty == self.saved_sequence:
            self._since = self._clock()
        self._dirty = sequence
        self._wake.set()

    async def flush(self) -> None:
        target = self._dirty
        if self._closed and self.saved_sequence < target:
            raise RuntimeError("checkpoint worker stopped before flush")
        self._forced = max(self._forced, target)
        self._wake.set()
        async with self._settled:
            await self._settled.wait_for(
                lambda: (
                    self.saved_sequence >= target or self._failed_sequence >= target or self._closed
                )
            )
            if self.saved_sequence < target:
                raise RuntimeError("checkpoint flush was not durable") from self.last_error

    async def close(self) -> None:
        self._closing = True
        try:
            await self.flush()
        finally:
            self._closed = True
            self._wake.set()

    async def run(self) -> None:
        if self._running or self._closed:
            raise RuntimeError("checkpoint worker cannot be restarted")
        self._running = True
        try:
            while not self._closed:
                self._wake.clear()
                now = self._clock()
                if self._dirty > self.saved_sequence and now >= self._retry_at:
                    due = self._since is not None and now - self._since >= self._max_delay
                    if due or self._forced > self.saved_sequence or self._idle():
                        target = self._dirty
                        try:
                            await self._save(target)
                        except Exception as exc:
                            self.last_error = exc
                            self._failed_sequence = target
                            self._retry_at = self._clock() + self._retry
                        else:
                            self.saved_sequence = target
                            self.last_error = None
                            self._failed_sequence = 0
                            # Updates accepted during capture retain their oldest
                            # dirty time; continuous traffic cannot postpone forever.
                            if self._dirty == target:
                                self._since = None
                        async with self._settled:
                            self._settled.notify_all()
                        continue
                timeout = 0.2
                if self._retry_at > now:
                    timeout = min(timeout, self._retry_at - now)
                elif self._since is not None:
                    timeout = min(timeout, max(0.001, self._since + self._max_delay - now))
                try:
                    await asyncio.wait_for(self._wake.wait(), timeout)
                except asyncio.TimeoutError:
                    pass
        finally:
            self._closed = True
            async with self._settled:
                self._settled.notify_all()
