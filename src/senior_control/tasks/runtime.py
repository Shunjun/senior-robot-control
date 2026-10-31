"""Drive persisted task decisions from the live connection and ROS state."""

from collections.abc import Callable, Sequence
from dataclasses import replace
from datetime import datetime

from senior_control.errors import DomainError, ErrorCode
from senior_control.tasks.executor import NavigationFailure, QualificationUnavailable, TaskExecutor
from senior_control.tasks.models import ExecutionContext, IntentAction, TaskDefinition, TaskIntent
from senior_control.tasks.runner import TaskRunner, _scan
from senior_control.tasks.store import TaskStore


class TaskRuntime:
    def __init__(
        self,
        store: TaskStore,
        executor: TaskExecutor,
        *,
        generation: Callable[[], int] | None = None,
    ) -> None:
        self._store = store
        self._executor = executor
        self._last_wall: datetime | None = None
        self._last_mono: float | None = None
        self._pending_skips: dict[tuple[str, datetime], TaskDefinition] = {}
        # //Compatible: isolated callers without an agent use a fixed generation.
        self._generation = generation if generation is not None else (lambda: 0)
        self._last_generation: int | None = None

    async def tick(self, now: datetime, monotonic: float, context: ExecutionContext) -> None:
        generation = self._generation()
        if self._last_generation is not None and generation != self._last_generation:
            await self._store.mark_active_cancel_pending(self._last_generation)
        self._last_generation = generation
        pending_failure: Exception | None = None
        for (_task_id, occurrence), task in tuple(self._pending_skips.items()):
            try:
                await self._persist_skip(task, occurrence)
            except Exception as exc:
                pending_failure = exc
                break
        tasks = await self._store.list_tasks()
        if self._last_wall is None:
            cursor = await self._store.scan_cursor()
            if cursor is not None and cursor != now:
                start, end = sorted((cursor, now))
                for task in tasks:
                    if task.enabled:
                        await self._store.record_skip(task.task_id, start, end, "restart")
        reference = self._executor.map_reference
        map_generation = (
            reference.map_generation
            if reference is not None and not reference.review_required
            else None
        )
        current = replace(
            context,
            last_tick_utc=self._last_wall,
            last_tick_mono=self._last_mono,
            map_generation=map_generation,
        )
        scan_start, scan_end, _jump = _scan(now, monotonic, current)
        instances = await self._store.executions_for_scan(scan_start, scan_end)
        runner = TaskRunner(tasks, instances)
        by_id = {task.task_id: task for task in tasks}
        intents = runner.tick(now, monotonic, current)
        for index, intent in enumerate(intents):
            if pending_failure is not None and intent.action is not IntentAction.PAUSE:
                self._defer_occurrences((intent,), by_id)
                continue
            try:
                if intent.action is IntentAction.SKIP and intent.occurrence_utc is not None:
                    task = by_id[intent.task_id]
                    self._pending_skips[(task.task_id, intent.occurrence_utc)] = task
                    await self._persist_skip(task, intent.occurrence_utc)
                    continue
                try:
                    await self._executor.apply(
                        intent,
                        by_id[intent.task_id],
                        expected_generation=generation,
                        map_generation=current.map_generation,
                        localization_generation=current.localization_generation,
                        localized=current.localized,
                    )
                except NavigationFailure:
                    # The execution remains unknown; the next tick retries cancellation.
                    continue
                except QualificationUnavailable:
                    # This error occurs before claim and before any navigation call.
                    assert (
                        intent.action is IntentAction.DISPATCH and intent.occurrence_utc is not None
                    )
                    task = by_id[intent.task_id]
                    self._pending_skips[(task.task_id, intent.occurrence_utc)] = task
                    await self._persist_skip(task, intent.occurrence_utc)
            except BaseException:
                self._defer_occurrences(intents[index + 1 :], by_id)
                raise
        if pending_failure is not None:
            raise pending_failure
        await self._store.save_scan_cursor(now)
        self._last_wall = now
        self._last_mono = monotonic

    def _defer_occurrences(
        self, intents: Sequence[TaskIntent], tasks: dict[str, TaskDefinition]
    ) -> None:
        for intent in intents:
            if intent.action in (IntentAction.DISPATCH, IntentAction.SKIP):
                if intent.occurrence_utc is not None:
                    self._pending_skips[(intent.task_id, intent.occurrence_utc)] = tasks[
                        intent.task_id
                    ]

    async def _persist_skip(self, task: TaskDefinition, occurrence: datetime) -> None:
        try:
            await self._store.skip_occurrence(task, occurrence)
        except DomainError as exc:
            if (
                exc.code is not ErrorCode.CONFLICT
                or await self._store.execution_for(task.task_id, occurrence) is None
            ):
                raise
        self._pending_skips.pop((task.task_id, occurrence), None)
