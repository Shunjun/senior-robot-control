"""Claim an execution, then call an injected navigation port.

The SQLite write lock is released before navigation starts. A crash after the
claim commits leaves the instance unsent; the next process start marks it
needs_review and does not send it again. Cancelling the navigation wait does
not report that the side effect was rolled back.
"""

import asyncio
from typing import Protocol

from senior_control.storage.database import Database
from senior_control.tasks.models import (
    ExecutionRecord,
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


class TaskExecutor:
    def __init__(
        self,
        database: Database,
        store: TaskStore,
        navigation: NavigationPort | None,
        *,
        fault_after: str | None = None,
    ) -> None:
        if store._database is not database:
            raise ValueError("executor store must use the given database")
        self._database = database
        self._store = store
        self._navigation = navigation
        self._fault_after = fault_after

    async def persist_and_dispatch(
        self, intent: TaskIntent, task: TaskDefinition
    ) -> ExecutionRecord:
        if intent.action is not IntentAction.DISPATCH or intent.occurrence_utc is None:
            raise ValueError("dispatch intent required")
        if intent.task_id != task.task_id:
            raise ValueError("dispatch task does not match the intent")
        record = await self._store.claim(task, intent.occurrence_utc)
        self._crash_if("committed")
        if self._navigation is None:
            return await self._store.mark_needs_review(record.execution_id)
        try:
            await self._navigation.start(record.execution_id, task)
        except asyncio.CancelledError:
            review = asyncio.create_task(self._store.mark_needs_review(record.execution_id))
            await asyncio.shield(review)
            raise
        except Exception:
            await self._store.mark_needs_review(record.execution_id)
            raise
        return await self._store.mark_running(record.execution_id)

    async def apply(self, intent: TaskIntent, task: TaskDefinition) -> ExecutionRecord | SkipRecord:
        if intent.task_id != task.task_id:
            raise ValueError("intent task does not match")
        if intent.action is IntentAction.DISPATCH:
            return await self.persist_and_dispatch(intent, task)
        if intent.action is IntentAction.PAUSE:
            if intent.execution_id is None:
                raise ValueError("pause requires an execution id")
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
