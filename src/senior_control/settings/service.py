"""Single settings owner and a concrete runtime/commit exclusion boundary."""

import asyncio
import json
import math
import sqlite3
import time
from collections.abc import AsyncIterator, Awaitable, Callable, Coroutine
from contextlib import AsyncExitStack, asynccontextmanager, suppress
from dataclasses import dataclass
from typing import Any, TypeVar

from senior_control.settings.models import (
    DrivingSettings,
    LoggingSettings,
    SettingsCapabilities,
    SettingsProfile,
    SettingsRequest,
    SettingsResult,
    SettingsSnapshot,
)
from senior_control.settings.store import SettingsStore

T = TypeVar("T")


async def _settle(operation: Coroutine[Any, Any, T]) -> T:
    """Cancellation cannot separate SQLite commit from immutable publication."""
    task = asyncio.create_task(operation)
    cancelled = False
    while not task.done():
        try:
            await asyncio.shield(task)
        except asyncio.CancelledError:
            cancelled = True
    result = task.result()
    if cancelled:
        raise asyncio.CancelledError
    return result


@dataclass(frozen=True)
class RuntimeCheck:
    stationary: bool = False
    executing_motion_task: bool = True
    manual_active: bool = False


class RuntimeGate:
    """Composition MUST hold this same lock for runtime transitions/output admission.

    Production serializes short task commits and the physical manual handoff,
    not task-port reconciliation. A generation fences samples taken outside the
    lock; the scoped provider rechecks generation/expiry inside the transaction.
    Settings holds the lock through durable commit and immutable publication.
    ``update``/``transition`` retain the same fence for isolated runtime callers.
    """

    def __init__(
        self,
        observe: Callable[[], RuntimeCheck] | None = None,
        *,
        refresh: Callable[[], Awaitable[None]] | None = None,
    ) -> None:
        self._observe = observe
        self._refresh = refresh
        self.lock = asyncio.Lock()
        self.generation = 0
        self._state = RuntimeCheck()

    def invalidate(self) -> None:
        """Nonblocking fence for safety/connection changes that must bypass the lock."""
        self.generation += 1

    @asynccontextmanager
    async def admission(self) -> AsyncIterator[None]:
        async with self.lock:
            self.invalidate()
            yield

    @property
    def state(self) -> RuntimeCheck:
        return self._state

    def check(self) -> RuntimeCheck:
        """Read current evidence inside the guard; production injects a fresh T5 check.

        The injected synchronous observer checks T5 evidence expiry at the current
        steady time and the actual T6 executing/manual owners. Cached update() is
        for tests or a composition that already refreshes evidence in this stream.
        """
        if not self.lock.locked():
            raise RuntimeError("runtime check requires the serialization guard")
        return self._observe() if self._observe is not None else self._state

    async def refresh(self) -> None:
        """Refresh trusted IPC/task evidence while excluding all motion admissions."""
        if not self.lock.locked():
            raise RuntimeError("runtime refresh requires the serialization guard")
        if self._refresh is not None:
            await self._refresh()

    async def transition(self, operation: Callable[[], Awaitable[RuntimeCheck]]) -> None:
        """Run admission/reducer/output transition and publish under the commit lock.

        An exception leaves state unconfirmed, so a partially completed transition
        cannot accidentally permit settings. Composition must route EVERY motion
        acceptance/start through here, including task resume and manual intent.
        """
        async with self.admission():
            self._state = RuntimeCheck()
            self._state = await operation()

    async def update(
        self, *, stationary: bool, executing_motion_task: bool, manual_active: bool = False
    ) -> None:
        """Publish observed state; motion admission itself must use transition()."""
        async with self.admission():
            self._state = RuntimeCheck(stationary, executing_motion_task, manual_active)


class SettingsService:
    """Owns publication and debug expiry. No motion events or automatic resume.

    Instantiate exactly once per vehicle database. ``prepare`` recovers expiry before
    publication. ``start_debug_restore`` is the sole timer, started/stopped by composition;
    get/apply also enforce expiry. T8 consumes snapshots and never maintains a timer.
    """

    def __init__(
        self,
        store: SettingsStore,
        profile: SettingsProfile,
        runtime: RuntimeGate,
        *,
        utc_now: Callable[[], float] = time.time,
        steady_now: Callable[[], float] = time.monotonic,
        utc_trusted: Callable[[], bool] = lambda: True,
    ) -> None:
        self._store = store
        self.profile = profile
        self.capabilities = SettingsCapabilities(
            driving_defaults=profile.driving_defaults,
            driving_minimum=profile.driving_minimum,
            driving_maximum=profile.driving_maximum,
            logging_defaults=profile.logging_defaults,
        )
        self.runtime = runtime
        self._utc_now = utc_now
        self._steady_now = steady_now
        self._utc_trusted = utc_trusted
        self._lock = asyncio.Lock()
        self._snapshot: SettingsSnapshot | None = None
        self._ready = False
        self._deadline_steady: float | None = None
        self._timer: asyncio.Task[None] | None = None
        self.debug_restore_error: str | None = None

    async def prepare(self) -> None:
        await _settle(self._prepare())

    async def _prepare(self) -> None:
        async with self._lock:
            if self._snapshot is not None:
                raise RuntimeError("settings already prepared")
            try:
                snapshot = await self._store.prepare(
                    SettingsSnapshot(
                        driving=self.profile.driving_defaults,
                        logging=self.profile.logging_defaults,
                    )
                )
                self.profile.validate_driving(snapshot.driving)
                if snapshot.debug_deadline_utc is not None:
                    remaining = snapshot.debug_deadline_utc - self._utc_now()
                    if not math.isfinite(remaining) or not self._utc_trusted():
                        remaining = 0.0
                    remaining = min(remaining, snapshot.logging.debug_restore_seconds)
                    self._deadline_steady = self._steady_now() + max(0.0, remaining)
                self._snapshot = snapshot
                await self._expire_locked()
                self._ready = True
            except Exception as exc:
                self._snapshot = None
                raise ValueError(
                    "settings unavailable: invalid or unreadable durable state"
                ) from exc

    def _current(self) -> SettingsSnapshot:
        if self._snapshot is None:
            raise ValueError("settings unavailable")
        return self._snapshot

    @property
    def published_snapshot(self) -> SettingsSnapshot:
        """Last committed immutable image; safe to read on a control/log hot path.

        This does no I/O and never waits for a settings transaction. The restore
        owner publishes changes; get() is the explicit refreshed query interface.
        """
        if not self._ready:
            raise ValueError("settings unavailable")
        return self._current()

    async def get(self) -> SettingsSnapshot:
        return await _settle(self._get())

    async def _get(self) -> SettingsSnapshot:
        async with self._lock:
            await self._expire_locked()
            return self._current()

    async def result(self, request_id: str) -> SettingsResult | None:
        async with self._lock:
            self._current()
            async with self._store.transaction() as connection:
                stored = await self._store.lookup(connection, request_id)
            return None if stored is None else stored[1]

    async def apply(self, request: SettingsRequest) -> SettingsResult:
        copied = SettingsRequest(
            request.request_id,
            request.expected_version,
            request.group,
            dict(request.values),
            request.restore_defaults,
        )
        return await _settle(self._apply(copied))

    async def _apply(self, request: SettingsRequest) -> SettingsResult:
        # Copy caller-owned dict before the first await; it cannot mutate an in-flight command.
        values = dict(request.values)
        if (
            not request.request_id.strip()
            or request.group not in ("driving", "logging")
            or type(request.expected_version) is not int
            or request.expected_version < 0
            or type(request.restore_defaults) is not bool
        ):
            raise ValueError("invalid settings request envelope")
        fingerprint = json.dumps(
            [request.group, request.expected_version, request.restore_defaults, values],
            sort_keys=True,
            separators=(",", ":"),
        )
        async with self._lock:
            await self._expire_locked()
            async with AsyncExitStack() as guards:
                if request.group == "driving":
                    await guards.enter_async_context(self.runtime.lock)
                    await self.runtime.refresh()
                current = self._current()
                async with self._store.transaction() as connection:
                    stored = await self._store.lookup(connection, request.request_id)
                    if stored is not None:
                        if stored[0] == fingerprint:
                            return stored[1]
                        return SettingsResult(
                            request_id=request.request_id,
                            group=request.group,
                            reason="request_conflict",
                            snapshot=current,
                            capabilities=self.capabilities,
                        )
                    await self._store.assert_current(connection, current)
                    runtime = self.runtime.check() if request.group == "driving" else None
                    reason = "applied"
                    updated = current
                    if current.version(request.group) != request.expected_version:
                        reason = "version_conflict"
                    elif runtime is not None and (
                        not runtime.stationary
                        or runtime.executing_motion_task
                        or runtime.manual_active
                    ):
                        reason = "stop_required"
                    else:
                        try:
                            updated = self._candidate(request, values, current)
                        except ValueError:
                            reason = "invalid_values"
                    result = SettingsResult(
                        request_id=request.request_id,
                        group=request.group,
                        reason=reason,
                        snapshot=updated,
                        capabilities=self.capabilities,
                    )
                    if reason == "applied":
                        await self._store.write_snapshot(connection, updated)
                    await self._store.write_receipt(connection, fingerprint, result)
                # This assignment is after commit and before either serialization lock releases.
                self._snapshot = updated
                if request.group == "driving":
                    self.runtime.invalidate()
                if updated.debug_deadline_utc != current.debug_deadline_utc:
                    self._deadline_steady = (
                        self._steady_now() + updated.logging.debug_restore_seconds
                        if updated.debug_deadline_utc is not None
                        else None
                    )
                return result

    def _candidate(
        self, request: SettingsRequest, values: dict[str, object], current: SettingsSnapshot
    ) -> SettingsSnapshot:
        if request.group == "driving":
            driving = (
                self.profile.driving_defaults
                if request.restore_defaults
                else (DrivingSettings.model_validate(values))
            )
            self.profile.validate_driving(driving)
            return current.model_copy(
                update={
                    "driving": driving,
                    "driving_version": current.driving_version + 1,
                }
            )
        if not request.restore_defaults and set(values) != set(LoggingSettings.model_fields):
            raise ValueError("complete logging group required")
        logging = (
            self.profile.logging_defaults
            if request.restore_defaults
            else (LoggingSettings.model_validate(values))
        )
        original = current.debug_original_level
        deadline = current.debug_deadline_utc
        if logging.debug_enabled and not current.logging.debug_enabled:
            now = self._utc_now()
            if (
                not self._utc_trusted()
                or not math.isfinite(now)
                or not math.isfinite(self._steady_now())
            ):
                raise ValueError("trusted UTC required to enable debug")
            original = current.logging.level
            deadline = now + logging.debug_restore_seconds
        elif not logging.debug_enabled:
            if current.logging.debug_enabled and not request.restore_defaults:
                logging = logging.model_copy(update={"level": original})
            original, deadline = None, None
        if logging.debug_enabled:
            logging = logging.model_copy(update={"level": original})
        return current.model_copy(
            update={
                "logging": logging,
                "logging_version": current.logging_version + 1,
                "debug_original_level": original,
                "debug_deadline_utc": deadline,
            }
        )

    async def _expire_locked(self) -> None:
        snapshot = self._current()
        deadline = snapshot.debug_deadline_utc
        if deadline is None:
            return
        utc, steady = self._utc_now(), self._steady_now()
        if (
            self._utc_trusted()
            and math.isfinite(utc)
            and math.isfinite(steady)
            and utc < deadline
            and self._deadline_steady is not None
            and steady < self._deadline_steady
        ):
            return
        logging = snapshot.logging.model_copy(
            update={
                "debug_enabled": False,
                "level": snapshot.debug_original_level,
            }
        )
        updated = snapshot.model_copy(
            update={
                "logging": logging,
                "logging_version": snapshot.logging_version + 1,
                "debug_original_level": None,
                "debug_deadline_utc": None,
            }
        )
        async with self._store.transaction() as connection:
            await self._store.assert_current(connection, snapshot)
            await self._store.write_snapshot(connection, updated)
        self._snapshot = updated
        self._deadline_steady = None

    def start_debug_restore(self) -> None:
        """Composition calls after prepare; close before closing the database."""
        self._current()
        if self._timer is not None:
            raise RuntimeError("debug restore owner already running")
        self._timer = asyncio.create_task(self._run_debug_restore())

    async def _run_debug_restore(self) -> None:
        while True:
            try:
                await self.get()
                self.debug_restore_error = None
            except (sqlite3.Error, OSError) as exc:
                # Expired reads still fail rather than publishing a false successful restore.
                # Keep this sole owner alive to retry once the storage fault clears.
                self.debug_restore_error = str(exc)
            await asyncio.sleep(0.25)

    async def close(self) -> None:
        if self._timer is not None:
            self._timer.cancel()
            with suppress(asyncio.CancelledError):
                await self._timer
            self._timer = None
