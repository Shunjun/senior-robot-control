"""Drive persisted task decisions from the live connection and ROS state."""

import json
import uuid
from collections.abc import Callable, Sequence
from dataclasses import dataclass, replace
from datetime import datetime, timezone

from senior_control.errors import DomainError, ErrorCode
from senior_control.tasks.executor import NavigationFailure, QualificationUnavailable, TaskExecutor
from senior_control.tasks.lifecycle_store import VersionConflict
from senior_control.tasks.models import ExecutionContext, IntentAction, TaskDefinition, TaskIntent
from senior_control.tasks.runner import TaskRunner, _scan
from senior_control.tasks.store import TaskStore


@dataclass(frozen=True)
class _PendingRange:
    start: datetime
    end: datetime
    skip_id: str


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
        self._pending_ranges: dict[tuple[str, str], _PendingRange] = {}
        # //Compatible: isolated callers without an agent use a fixed generation.
        self._generation = generation if generation is not None else (lambda: 0)

    async def tick(self, now: datetime, monotonic: float, context: ExecutionContext) -> None:
        generation = self._generation()
        self._executor.context = context
        service = self._executor.lifecycle
        if service is not None:
            await service.start()
            bound = set(await service.store.instances(active_only=True))
            for previous in await self._store.executions_for_scan(None, now):
                if previous.execution_id not in bound:
                    definition = await self._store.get(previous.task_id)
                    if definition is not None:
                        await service.store.bind_existing(
                            previous, definition, self._executor.current_context
                        )
                        bound.add(previous.execution_id)
        else:
            bound = set()
        tasks = await self._store.list_tasks()
        by_id = {task.task_id: task for task in tasks}
        reference = self._executor.map_reference
        map_generation = (
            reference.map_generation
            if reference is not None and not reference.review_required
            else context.map_generation
            if reference is None
            else None
        )
        current = replace(
            context,
            last_tick_utc=self._last_wall,
            last_tick_mono=self._last_mono,
            map_generation=map_generation,
        )

        # Safety cancellation precedes every skip audit, including restart and retries.
        active = await self._store.executions_for_scan(None, now)
        safety_context = replace(current, last_tick_utc=None, last_tick_mono=None)
        safety_intents = TaskRunner(tasks, active).tick(now, monotonic, safety_context)
        safety_failure: Exception | None = None
        for intent in safety_intents:
            if intent.action is not IntentAction.PAUSE or intent.execution_id in bound:
                continue
            try:
                await self._executor.apply(intent, by_id[intent.task_id])
            except NavigationFailure:
                # The durable cancellation obligation remains for the next tick.
                continue
            except Exception as exc:
                if safety_failure is None:
                    safety_failure = exc

        canceling = {
            intent.execution_id for intent in safety_intents if intent.action is IntentAction.PAUSE
        }
        for identity in bound:
            state = await service.store.load(identity) if service is not None else None
            if state is not None and state.phase not in {"succeeded", "failed", "canceled"}:
                record = await self._store.execution_by_id(identity)
                if record is not None:
                    try:
                        await self._executor.reconcile(record, current.map_generation)
                    except VersionConflict:
                        # A concurrent user command won; reload its state next tick.
                        continue
        for record in active:
            if record.execution_id not in bound and record.execution_id not in canceling:
                await self._executor.reconcile(record, current.map_generation)

        if self._last_wall is None:
            cursor = await self._store.scan_cursor()
            if cursor is not None and cursor != now:
                start, end = sorted((cursor, now))
                for task in tasks:
                    if task.enabled:
                        self._queue_range(task.task_id, start, end, "restart", origin=cursor)

        scan_start, scan_end, _jump = _scan(now, monotonic, current)
        instances = await self._store.executions_for_scan(scan_start, scan_end)
        # An execution active at tick entry keeps the tick busy even if just cancelled.
        scan_instances = {item.execution_id: item for item in instances}
        scan_instances.update({item.execution_id: item for item in active})
        intents = TaskRunner(tasks, tuple(scan_instances.values())).tick(now, monotonic, current)
        for intent in intents:
            if intent.action is IntentAction.SKIP:
                self._queue_skip(intent, by_id)

        try:
            await self._flush_skips()
        except BaseException:
            self._defer_occurrences(intents, by_id)
            raise
        if safety_failure is not None:
            self._defer_occurrences(intents, by_id)
            raise safety_failure

        for index, intent in enumerate(intents):
            if intent.action is not IntentAction.DISPATCH:
                continue
            try:
                assert intent.occurrence_utc is not None
                if (
                    await self._store.execution_for(intent.task_id, intent.occurrence_utc)
                    is not None
                ):
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
                    self._queue_skip(intent, by_id)
                    await self._flush_skips()
            except BaseException:
                self._defer_occurrences(intents[index:], by_id)
                raise
        await self._store.save_scan_cursor(now)
        self._last_wall = now
        self._last_mono = monotonic
        self._pending_ranges.clear()

    def _queue_skip(self, intent: TaskIntent, tasks: dict[str, TaskDefinition]) -> None:
        if intent.occurrence_utc is not None:
            self._pending_skips[(intent.task_id, intent.occurrence_utc)] = tasks[intent.task_id]
            return
        if intent.skip_from is None or intent.skip_until is None:
            raise ValueError("skip intent is missing its occurrence or interval")
        self._queue_range(intent.task_id, intent.skip_from, intent.skip_until, intent.reason)

    def _queue_range(
        self,
        task_id: str,
        start: datetime,
        end: datetime,
        reason: str,
        *,
        origin: datetime | None = None,
    ) -> None:
        key = (task_id, reason)
        previous = self._pending_ranges.get(key)
        if previous is None:
            if origin is None:
                skip_id = uuid.uuid4().hex
            else:
                identity = json.dumps(
                    [
                        "task-skip-restart",
                        task_id,
                        reason,
                        origin.astimezone(timezone.utc).isoformat(),
                    ]
                )
                skip_id = uuid.uuid5(uuid.NAMESPACE_URL, identity).hex
            self._pending_ranges[key] = _PendingRange(start, end, skip_id)
        else:
            self._pending_ranges[key] = _PendingRange(
                min(start, previous.start), max(end, previous.end), previous.skip_id
            )

    def _defer_occurrences(
        self, intents: Sequence[TaskIntent], tasks: dict[str, TaskDefinition]
    ) -> None:
        for intent in intents:
            if intent.action is IntentAction.DISPATCH and intent.occurrence_utc is not None:
                self._pending_skips[(intent.task_id, intent.occurrence_utc)] = tasks[intent.task_id]

    async def _flush_skips(self) -> None:
        for (task_id, reason), pending in tuple(self._pending_ranges.items()):
            await self._store.record_skip(
                task_id, pending.start, pending.end, reason, skip_id=pending.skip_id
            )
        for (_task_id, occurrence), task in tuple(self._pending_skips.items()):
            await self._persist_skip(task, occurrence)

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
