"""Pure schedule decisions. No database writes and no navigation."""

from collections.abc import Sequence
from datetime import datetime, timedelta, timezone

from senior_control.tasks.models import (
    ExecutionContext,
    ExecutionRecord,
    ExecutionStatus,
    IntentAction,
    TaskDefinition,
    TaskIntent,
)

_JUMP_SECONDS = 2.0
_ACTIVE = {ExecutionStatus.RUNNING, ExecutionStatus.DISPATCHING}


class TaskRunner:
    def __init__(
        self,
        tasks: Sequence[TaskDefinition],
        instances: Sequence[ExecutionRecord] = (),
    ) -> None:
        ordered = tuple(sorted(tasks, key=lambda task: task.task_id))
        if len({task.task_id for task in ordered}) != len(ordered):
            raise ValueError("task id must be unique")
        self._tasks = ordered
        self._instances = tuple(instances)

    def tick(
        self,
        now_utc: datetime,
        now_mono: float,
        context: ExecutionContext,
    ) -> list[TaskIntent]:
        moment = _utc(now_utc)
        start, end, jump = _scan(moment, now_mono, context)
        tasks = {task.task_id: task for task in self._tasks}
        intents: list[TaskIntent] = []
        if jump is not None:
            skip_from, skip_until = jump
            for task in self._tasks:
                if task.enabled:
                    intents.append(
                        TaskIntent(
                            task.task_id,
                            IntentAction.SKIP,
                            "clock_jump",
                            skip_from=skip_from,
                            skip_until=skip_until,
                        )
                    )
        active = [
            instance
            for instance in self._instances
            if instance.status in _ACTIVE and instance.task_id in tasks
        ]
        busy = bool(active)
        for instance in active:
            task = tasks[instance.task_id]
            reason = _block_reason(task, context, busy=False)
            if reason is not None:
                intents.append(
                    TaskIntent(
                        task.task_id,
                        IntentAction.PAUSE,
                        reason,
                        occurrence_utc=_utc(instance.occurrence_utc),
                        execution_id=instance.execution_id,
                    )
                )
        claimed = {
            (instance.task_id, _utc(instance.occurrence_utc)) for instance in self._instances
        }
        if start is None:
            return intents
        due: list[tuple[datetime, TaskDefinition, str | None]] = []
        for task in self._tasks:
            if not task.enabled:
                continue
            cursor = start
            while True:
                occurrence = task.schedule.next_after(cursor)
                if occurrence > end:
                    break
                cursor = occurrence
                if (task.task_id, occurrence) in claimed:
                    continue
                due.append((occurrence, task, _block_reason(task, context, busy)))
        if not due:
            return intents
        latest = max(item[0] for item in due)
        chosen = min(
            (item for item in due if item[0] == latest),
            key=lambda item: item[1].task_id,
        )
        chosen_blocked = chosen[2] is not None
        for occurrence, task, reason in sorted(due, key=lambda item: (item[0], item[1].task_id)):
            selected = (
                not chosen_blocked and task.task_id == chosen[1].task_id and occurrence == chosen[0]
            )
            if selected:
                intents.append(
                    TaskIntent(
                        task.task_id, IntentAction.DISPATCH, "due", occurrence_utc=occurrence
                    )
                )
                continue
            intents.append(
                TaskIntent(
                    task.task_id,
                    IntentAction.SKIP,
                    reason if reason is not None else "busy",
                    occurrence_utc=occurrence,
                )
            )
        return intents


def _scan(
    now_utc: datetime,
    now_mono: float,
    context: ExecutionContext,
) -> tuple[datetime | None, datetime, tuple[datetime, datetime] | None]:
    if context.last_tick_utc is None or context.last_tick_mono is None:
        return None, now_utc, None
    last = _utc(context.last_tick_utc)
    wall = (now_utc - last).total_seconds()
    mono = now_mono - context.last_tick_mono
    jumped = wall < -_JUMP_SECONDS or mono < -_JUMP_SECONDS or abs(wall - mono) > _JUMP_SECONDS
    if not jumped:
        return last, now_utc, None
    if now_utc >= last and mono >= 0:
        eligible_start = now_utc - timedelta(seconds=mono)
        if eligible_start < last:
            eligible_start = last
        if eligible_start > last:
            return eligible_start, now_utc, (last, eligible_start)
        return last, now_utc, None
    if now_utc < last:
        return now_utc, now_utc, (now_utc, last)
    return now_utc, now_utc, (last, now_utc)


def _block_reason(task: TaskDefinition, context: ExecutionContext, busy: bool) -> str | None:
    if not context.connected:
        return "offline"
    if context.manual_driving:
        return "manual"
    if not context.safety_clear:
        return "safety"
    if not context.localized:
        return "localization"
    if context.active_map != task.map_ref:
        return "map"
    if task.task_id in context.invalidated_task_ids:
        return "invalidated"
    if busy:
        return "busy"
    return None


def _utc(moment: datetime) -> datetime:
    if moment.tzinfo is None or moment.utcoffset() is None:
        raise ValueError("timestamp must be timezone-aware")
    return moment.astimezone(timezone.utc)
