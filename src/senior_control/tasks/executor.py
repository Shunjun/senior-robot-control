"""Claim an execution, then call an injected navigation port.

The SQLite write lock is released before navigation starts. A crash after the
claim commits leaves the instance unsent; the next process start marks it
needs_review and does not send it again. Cancelling the navigation wait does
not report that the side effect was rolled back.
"""

import asyncio
from collections.abc import Callable
from typing import Protocol

from senior_control.storage.database import Database
from senior_control.tasks.models import (
    ExecutionRecord,
    ExecutionStatus,
    IntentAction,
    SkipRecord,
    TaskDefinition,
    TaskIntent,
)
from senior_control.tasks.store import TaskStore


class ProcessCrash(BaseException):
    """Injected process death. It is not an execution result."""


class NavigationPort(Protocol):
    async def start(self, execution_id: str, task: TaskDefinition) -> None: ...

    async def cancel(self, execution_id: str) -> None: ...


class NavigationFailure(Exception):
    """A navigation RPC failed; persisted state retains any cancellation obligation."""


class TaskExecutor:
    def __init__(
        self,
        database: Database,
        store: TaskStore,
        navigation: NavigationPort | None,
        *,
        fault_after: str | None = None,
        link_up: Callable[[], bool] | None = None,
        generation: Callable[[], int] | None = None,
    ) -> None:
        if store._database is not database:
            raise ValueError("executor store must use the given database")
        self._database = database
        self._store = store
        self._navigation = navigation
        self._fault_after = fault_after
        self._link_up = link_up if link_up is not None else (lambda: True)
        # //Compatible: injected legacy callers without a connection use one fixed generation.
        self._generation = generation if generation is not None else (lambda: 0)

    async def persist_and_dispatch(
        self, intent: TaskIntent, task: TaskDefinition, *, expected_generation: int | None = None
    ) -> ExecutionRecord:
        if intent.action is not IntentAction.DISPATCH or intent.occurrence_utc is None:
            raise ValueError("dispatch intent required")
        if intent.task_id != task.task_id:
            raise ValueError("dispatch task does not match the intent")
        started_generation = self._generation()
        if expected_generation is not None and expected_generation != started_generation:
            return await self._store.skip_occurrence(task, intent.occurrence_utc)
        record = await self._store.claim(task, intent.occurrence_utc, generation=started_generation)
        self._crash_if("committed")
        if (
            self._navigation is None
            or not self._link_up()
            or self._generation() != started_generation
        ):
            return await self._store.mark_needs_review(record.execution_id)
        try:
            await self._navigation.start(record.execution_id, task)
        except asyncio.CancelledError:
            pending = asyncio.create_task(self._store.mark_cancel_pending(record.execution_id))
            await asyncio.shield(pending)
            raise
        except Exception as exc:
            await self._store.mark_cancel_pending(record.execution_id)
            try:
                await self._navigation.cancel(record.execution_id)
            except Exception:
                raise NavigationFailure("start and cancellation are unconfirmed") from exc
            await self._store.mark_needs_review(record.execution_id)
            raise NavigationFailure("start outcome is unknown") from exc
        if not self._link_up() or self._generation() != started_generation:
            await self._store.mark_cancel_pending(record.execution_id)
            try:
                await self._navigation.cancel(record.execution_id)
            except Exception as exc:
                raise NavigationFailure("reconnected action cancellation is unconfirmed") from exc
            return await self._store.mark_needs_review(record.execution_id)
        running = await self._store.mark_running(record.execution_id)
        if (
            running.status is ExecutionStatus.CANCEL_PENDING
            or not self._link_up()
            or self._generation() != started_generation
        ):
            await self._store.mark_cancel_pending(record.execution_id)
            try:
                await self._navigation.cancel(record.execution_id)
            except Exception as exc:
                raise NavigationFailure("reconnected action cancellation is unconfirmed") from exc
            return await self._store.mark_needs_review(record.execution_id)
        return running

    async def apply(
        self, intent: TaskIntent, task: TaskDefinition, *, expected_generation: int | None = None
    ) -> ExecutionRecord | SkipRecord:
        if intent.task_id != task.task_id:
            raise ValueError("intent task does not match")
        if intent.action is IntentAction.DISPATCH:
            return await self.persist_and_dispatch(
                intent, task, expected_generation=expected_generation
            )
        if intent.action is IntentAction.PAUSE:
            if intent.execution_id is None:
                raise ValueError("pause requires an execution id")
            pending = await self._store.mark_cancel_pending(intent.execution_id)
            if self._navigation is None:
                raise NavigationFailure("navigation cancellation is unavailable")
            try:
                await self._navigation.cancel(intent.execution_id)
            except Exception as exc:
                raise NavigationFailure("navigation cancellation is unconfirmed") from exc
            if not pending.navigation_sent:
                return await self._store.mark_needs_review(intent.execution_id)
            return await self._store.pause(intent.execution_id)
        if intent.action is IntentAction.SKIP and intent.reason == "clock_jump":
            if intent.skip_from is None or intent.skip_until is None:
                raise ValueError("clock jump requires an interval")
            if intent.skip_from > intent.skip_until:
                raise ValueError("skip interval is inverted")
            return await self._store.record_skip(
                task.task_id, intent.skip_from, intent.skip_until, intent.reason
            )
        if intent.action is IntentAction.SKIP and intent.occurrence_utc is not None:
            return await self._store.skip_occurrence(task, intent.occurrence_utc)
        raise ValueError("unsupported intent")

    def _crash_if(self, phase: str) -> None:
        if self._fault_after == phase:
            raise ProcessCrash(phase)
