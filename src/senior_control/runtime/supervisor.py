"""Agent supervisor: heartbeat, latest-only drive forwarding, background watch, uplink retry.

Nothing here imports ROS. Every ROS-facing call is injected. The supervisor never
publishes `/cmd_vel`; drive values leave only through `drive_sink`, and the safety chain
behind that sink owns the topic.
"""

import asyncio
import logging
import math
from collections.abc import Awaitable, Callable, Mapping
from types import TracebackType
from typing import Any

logger = logging.getLogger(__name__)


class AuthRejected(Exception):
    """The cloud refused the device credential; retrying cannot help."""


class Supervisor:
    def __init__(
        self,
        *,
        send_heartbeat: Callable[[], Awaitable[None]],
        call_ros: Callable[[str], Awaitable[Any]],
        drive_sink: Callable[[float, float], Awaitable[None]],
        publish_cmd_vel: Callable[[float, float], Awaitable[None]],
        connect: Callable[[], Awaitable[bool | None]] | None = None,
        background: Mapping[str, Callable[[], Awaitable[None]]] | None = None,
        heartbeat_seconds: float = 0.2,
        retry_delay: float = 1.0,
        max_retry_delay: float | None = None,
        shutdown_timeout: float = 1.0,
        on_fault: Callable[[str], None] | None = None,
    ) -> None:
        if (
            heartbeat_seconds <= 0
            or retry_delay < 0
            or not math.isfinite(shutdown_timeout)
            or shutdown_timeout <= 0
        ):
            raise ValueError("heartbeat and retry delay must be positive")
        self._send_heartbeat = send_heartbeat
        self._call_ros = call_ros
        self._drive_sink = drive_sink
        # Accepted so the owner can prove the topic stays untouched; deliberately never called.
        self._publish_cmd_vel = publish_cmd_vel
        self._connect = connect
        self._background = dict(background or {})
        self._heartbeat_seconds = heartbeat_seconds
        self._shutdown_timeout = shutdown_timeout
        self._on_fault = on_fault
        self._retry_delay = retry_delay
        self._max_retry_delay = (
            max_retry_delay if max_retry_delay is not None else max(retry_delay * 30, retry_delay)
        )
        self._pending: tuple[float, float] | None = None
        self._wake = asyncio.Event()
        self._tasks: list[asyncio.Task[None]] = []
        self._running = False
        self.state = "idle"
        self.revoked_reason: str | None = None

    @property
    def pending(self) -> int:
        return 0 if self._pending is None else 1

    @property
    def motion_allowed(self) -> bool:
        return self._running and self.revoked_reason is None

    async def __aenter__(self) -> "Supervisor":
        if self._running or self._tasks:
            raise RuntimeError("supervisor already started")
        self._running = True
        self.state = "running"
        self._spawn(self._heartbeat_loop())
        self._spawn(self._drain_loop())
        for name, factory in self._background.items():
            self._spawn(self._watch(name, factory))
        if self._connect is not None:
            self._spawn(self._uplink_loop(self._connect))
        return self

    async def __aexit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        tb: TracebackType | None,
    ) -> None:
        self._running = False
        self._pending = None
        self._wake.set()
        tasks = list(self._tasks)
        for task in tasks:
            task.cancel()
        done, pending = await asyncio.wait(tasks, timeout=self._shutdown_timeout)
        await asyncio.gather(*done, return_exceptions=True)
        self._tasks = list(pending)
        if pending:
            self.state = "shutdown_failed"
            for task in pending:
                task.cancel()
            raise RuntimeError("supervisor tasks did not stop before shutdown deadline")
        self.state = "stopped"

    def _spawn(self, coroutine: Awaitable[None]) -> None:
        self._tasks.append(asyncio.ensure_future(coroutine))

    async def request_ros(self, name: str) -> Any:
        """Run the injected ROS call in the caller's task; the heartbeat has its own task."""
        return await self._call_ros(name)

    def submit_drive(self, vx: float, wz: float) -> bool:
        """Keep one pending velocity. A newer value, a stop included, replaces it."""
        if not self._running or self.revoked_reason is not None:
            return False
        self._pending = (vx, wz)
        self._wake.set()
        return True

    def revoke(self, reason: str) -> None:
        """Stop accepting and draining motion now. Safe to call from a signal handler."""
        self._revoke(reason)

    def _revoke(self, reason: str) -> None:
        if self.revoked_reason is None:
            self.revoked_reason = reason
            logger.error("motion revoked: %s", reason)
        self._pending = None

    def _fault(self, reason: str) -> None:
        self._revoke(reason)
        if self._on_fault is not None:
            self._on_fault(reason)

    async def _heartbeat_loop(self) -> None:
        loop = asyncio.get_running_loop()
        interval = self._heartbeat_seconds
        deadline = loop.time()
        while self._running:
            try:
                await asyncio.wait_for(self._send_heartbeat(), timeout=interval * 5)
            except Exception:
                logger.warning("heartbeat failed", exc_info=True)
            # //Compatible: Python 3.10 wait_for can swallow cancellation at completion.
            if not self._running:
                return
            deadline += interval
            now = loop.time()
            if deadline < now:
                deadline = now
            await asyncio.sleep(deadline - now)

    async def _drain_loop(self) -> None:
        while self._running:
            await self._wake.wait()
            if not self._running:
                return
            while self._running and self._pending is not None and self.revoked_reason is None:
                vx, wz = self._pending
                self._pending = None
                try:
                    await self._drive_sink(vx, wz)
                except Exception:
                    logger.exception("drive sink failed")
                    self._fault("drive sink failed")
            self._wake.clear()

    async def _watch(self, name: str, factory: Callable[[], Awaitable[None]]) -> None:
        try:
            await factory()
        except Exception:
            logger.exception("background task %s failed", name)
            if self._running:
                self._fault(f"background task {name} failed")
        else:
            if self._running:
                self._fault(f"background task {name} ended")

    async def _uplink_loop(self, connect: Callable[[], Awaitable[bool | None]]) -> None:
        delay = self._retry_delay
        while self._running:
            self.state = "connecting"
            try:
                healthy = await connect()
            except AuthRejected:
                self.state = "auth_failed"
                self._revoke("device credential rejected")
                return
            except Exception:
                logger.warning("uplink failed", exc_info=True)
                self.state = "reconnecting"
            else:
                self.state = "reconnecting"
                # //Compatible: existing connect callbacks return None on success.
                if healthy is not False:
                    delay = self._retry_delay
            if not self._running:
                return
            await asyncio.sleep(delay)
            delay = min(max(delay * 2, self._retry_delay), self._max_retry_delay)
