"""Drive persisted task decisions from the live connection and ROS state."""

from dataclasses import replace
from datetime import datetime

from senior_control.tasks.executor import NavigationFailure, TaskExecutor
from senior_control.tasks.models import ExecutionContext
from senior_control.tasks.runner import TaskRunner, _scan
from senior_control.tasks.store import TaskStore


class TaskRuntime:
    def __init__(self, store: TaskStore, executor: TaskExecutor) -> None:
        self._store = store
        self._executor = executor
        self._last_wall: datetime | None = None
        self._last_mono: float | None = None

    async def tick(self, now: datetime, monotonic: float, context: ExecutionContext) -> None:
        tasks = await self._store.list_tasks()
        if self._last_wall is None:
            cursor = await self._store.scan_cursor()
            if cursor is not None and cursor != now:
                start, end = sorted((cursor, now))
                for task in tasks:
                    if task.enabled:
                        await self._store.record_skip(task.task_id, start, end, "restart")
        current = replace(context, last_tick_utc=self._last_wall, last_tick_mono=self._last_mono)
        scan_start, scan_end, _jump = _scan(now, monotonic, current)
        instances = await self._store.executions_for_scan(scan_start, scan_end)
        runner = TaskRunner(tasks, instances)
        by_id = {task.task_id: task for task in tasks}
        for intent in runner.tick(now, monotonic, current):
            try:
                await self._executor.apply(intent, by_id[intent.task_id])
            except NavigationFailure:
                # The execution is durably marked unknown; the next tick retries cancellation.
                continue
        await self._store.save_scan_cursor(now)
        self._last_wall = now
        self._last_mono = monotonic
