"""Schedule arithmetic and pure execution decisions."""

from dataclasses import replace
from datetime import datetime, timedelta, timezone

import pytest

from senior_control.models import MapRef
from senior_control.tasks.models import (
    ExecutionContext,
    ExecutionRecord,
    ExecutionStatus,
    IntentAction,
    TaskDefinition,
    TaskIntent,
)
from senior_control.tasks.runner import TaskRunner
from senior_control.tasks.schedule import Schedule

MAP = MapRef("yard", 1)


def context(
    *,
    connected: bool = True,
    localized: bool = True,
    safety_clear: bool = True,
    manual_driving: bool = False,
    active_map: MapRef | None = MAP,
    invalidated_task_ids: frozenset[str] = frozenset(),
    last_tick_utc: datetime | None = None,
    last_tick_mono: float | None = None,
) -> ExecutionContext:
    return ExecutionContext(
        connected,
        localized,
        safety_clear,
        manual_driving,
        active_map,
        invalidated_task_ids,
        last_tick_utc,
        last_tick_mono,
    )


def interval_task(
    created: datetime,
    every_seconds: int = 1800,
    task_id: str = "patrol",
    map_ref: MapRef = MAP,
) -> TaskDefinition:
    return TaskDefinition(
        task_id,
        "robot-a",
        map_ref,
        ("gate",),
        Schedule.interval(created, every_seconds),
        "Asia/Shanghai",
        True,
        1,
        created,
    )


def test_interval_anchored_to_created_at() -> None:
    created = datetime(2026, 10, 27, 10, 17, 42, tzinfo=timezone.utc)
    schedule = Schedule.interval(created, every_seconds=30 * 60)

    first = schedule.next_after(created)

    assert schedule.timezone == "Asia/Shanghai"
    assert first == created + timedelta(minutes=30)
    assert (first.minute, first.second) == (47, 42)
    assert schedule.next_after(created + timedelta(minutes=10)) == first
    assert schedule.next_after(first) == created + timedelta(minutes=60)


def test_cron_dom_dow_or() -> None:
    schedule = Schedule.cron("0 9 15 * 1")
    before_monday = datetime(2026, 10, 26, 0, 59, tzinfo=timezone.utc)
    monday = datetime(2026, 10, 26, 1, 0, tzinfo=timezone.utc)
    before_fifteenth = datetime(2026, 11, 15, 0, 59, tzinfo=timezone.utc)
    fifteenth = datetime(2026, 11, 15, 1, 0, tzinfo=timezone.utc)
    next_monday = datetime(2026, 11, 2, 1, 0, tzinfo=timezone.utc)

    assert schedule.timezone == "Asia/Shanghai"
    assert schedule.next_after(before_monday) == monday
    assert schedule.next_after(before_fifteenth) == fifteenth
    assert schedule.next_after(monday) == next_monday


def test_dst_gap_skips_and_fold_runs_once() -> None:
    gap = Schedule.cron("30 2 * * *", timezone="America/New_York")
    before_gap = datetime(2026, 3, 7, 7, 30, tzinfo=timezone.utc)
    after_gap = datetime(2026, 3, 9, 6, 30, tzinfo=timezone.utc)
    fold = Schedule.cron("30 1 * * *", timezone="America/New_York")
    before_fold = datetime(2026, 11, 1, 5, 0, tzinfo=timezone.utc)
    first_fold = datetime(2026, 11, 1, 5, 30, tzinfo=timezone.utc)
    second_fold = datetime(2026, 11, 1, 6, 30, tzinfo=timezone.utc)
    next_day = datetime(2026, 11, 2, 6, 30, tzinfo=timezone.utc)
    between_folds = datetime(2026, 11, 1, 6, 0, tzinfo=timezone.utc)

    assert gap.next_after(before_gap) == after_gap
    assert fold.next_after(before_fold) == first_fold
    assert fold.next_after(first_fold) == next_day
    assert fold.next_after(first_fold) != second_fold
    assert fold.next_after(between_folds) == next_day


def test_offline_skips_no_catchup() -> None:
    created = datetime(2026, 10, 27, 1, 0, tzinfo=timezone.utc)
    task = interval_task(created)
    due = created + timedelta(minutes=30)
    runner = TaskRunner([task])
    offline = context(
        connected=False,
        active_map=task.map_ref,
        last_tick_utc=due - timedelta(seconds=30),
        last_tick_mono=970.0,
    )

    skipped = runner.tick(due, 1000.0, offline)
    later = runner.tick(
        due + timedelta(minutes=30),
        2800.0,
        context(
            active_map=task.map_ref,
            last_tick_utc=due,
            last_tick_mono=1000.0,
        ),
    )
    running = ExecutionRecord(
        "exec-1",
        task.task_id,
        due,
        ExecutionStatus.RUNNING,
        True,
        False,
    )
    paused = TaskRunner([task], [running]).tick(
        due + timedelta(seconds=10),
        1010.0,
        context(
            connected=False,
            active_map=task.map_ref,
            last_tick_utc=due,
            last_tick_mono=1000.0,
        ),
    )
    reconnected = TaskRunner(
        [task],
        [
            ExecutionRecord(
                "exec-1",
                task.task_id,
                due,
                ExecutionStatus.PAUSED,
                True,
                False,
            )
        ],
    ).tick(
        due + timedelta(seconds=20),
        1020.0,
        context(
            active_map=task.map_ref,
            last_tick_utc=due + timedelta(seconds=10),
            last_tick_mono=1010.0,
        ),
    )

    assert skipped == [TaskIntent(task.task_id, IntentAction.SKIP, "offline", occurrence_utc=due)]
    assert runner.tick(due, 1000.0, offline) == skipped
    assert later == [
        TaskIntent(
            task.task_id,
            IntentAction.DISPATCH,
            "due",
            occurrence_utc=due + timedelta(minutes=30),
        )
    ]
    assert paused == [
        TaskIntent(
            task.task_id,
            IntentAction.PAUSE,
            "offline",
            occurrence_utc=due,
            execution_id="exec-1",
        )
    ]
    assert reconnected == []


def test_manual_takeover_requires_resume() -> None:
    created = datetime(2026, 10, 27, 1, 0, tzinfo=timezone.utc)
    task = interval_task(created)
    due = created + timedelta(minutes=30)
    running = ExecutionRecord(
        "exec-1",
        task.task_id,
        due,
        ExecutionStatus.RUNNING,
        True,
        False,
    )

    takeover = TaskRunner([task], [running]).tick(
        due + timedelta(seconds=5),
        1005.0,
        context(
            manual_driving=True,
            active_map=task.map_ref,
            last_tick_utc=due,
            last_tick_mono=1000.0,
        ),
    )
    held = TaskRunner([task]).tick(
        due,
        1000.0,
        context(
            manual_driving=True,
            active_map=task.map_ref,
            last_tick_utc=due - timedelta(seconds=30),
            last_tick_mono=970.0,
        ),
    )
    released = TaskRunner(
        [task],
        [
            ExecutionRecord(
                "exec-1",
                task.task_id,
                due,
                ExecutionStatus.PAUSED,
                True,
                False,
            )
        ],
    ).tick(
        due + timedelta(seconds=10),
        1010.0,
        context(
            active_map=task.map_ref,
            last_tick_utc=due + timedelta(seconds=5),
            last_tick_mono=1005.0,
        ),
    )

    assert takeover == [
        TaskIntent(
            task.task_id,
            IntentAction.PAUSE,
            "manual",
            occurrence_utc=due,
            execution_id="exec-1",
        )
    ]
    assert held == [TaskIntent(task.task_id, IntentAction.SKIP, "manual", occurrence_utc=due)]
    assert released == []


def test_next_scheduled_occurrence_runs_but_interrupted_instance_does_not_resume() -> None:
    created = datetime(2026, 10, 27, 1, 0, tzinfo=timezone.utc)
    task = interval_task(created)
    interrupted = created + timedelta(minutes=30)
    following = created + timedelta(minutes=60)
    paused = ExecutionRecord(
        "exec-old",
        task.task_id,
        interrupted,
        ExecutionStatus.PAUSED,
        True,
        False,
    )

    intents = TaskRunner([task], [paused]).tick(
        following,
        5000.0,
        context(
            active_map=task.map_ref,
            last_tick_utc=following - timedelta(seconds=20),
            last_tick_mono=4980.0,
        ),
    )

    assert intents == [
        TaskIntent(task.task_id, IntentAction.DISPATCH, "due", occurrence_utc=following)
    ]
    assert all(item.execution_id != "exec-old" for item in intents)


def test_first_scan_does_not_catch_up() -> None:
    created = datetime(2026, 10, 27, 1, 0, tzinfo=timezone.utc)
    task = interval_task(created)

    assert TaskRunner([task]).tick(created + timedelta(minutes=30), 10.0, context()) == []


def test_clock_jump_records_skipped_interval() -> None:
    created = datetime(2026, 1, 1, tzinfo=timezone.utc)
    task = interval_task(created, every_seconds=600)
    last = created + timedelta(minutes=10)
    now = last + timedelta(hours=2, seconds=30)

    intents = TaskRunner([task]).tick(
        now,
        101.0,
        context(active_map=task.map_ref, last_tick_utc=last, last_tick_mono=100.0),
    )

    assert intents == [
        TaskIntent(
            task.task_id,
            IntentAction.SKIP,
            "clock_jump",
            skip_from=last,
            skip_until=now - timedelta(seconds=1),
        )
    ]


def test_backward_clock_jump_skips_without_dispatch() -> None:
    created = datetime(2026, 1, 1, tzinfo=timezone.utc)
    task = interval_task(created, every_seconds=600)
    last = created + timedelta(hours=3)
    now = created + timedelta(hours=1)

    intents = TaskRunner([task]).tick(
        now,
        50.0,
        context(active_map=task.map_ref, last_tick_utc=last, last_tick_mono=100.0),
    )

    assert intents == [
        TaskIntent(
            task.task_id,
            IntentAction.SKIP,
            "clock_jump",
            skip_from=now,
            skip_until=last,
        )
    ]


@pytest.mark.parametrize(
    ("override", "reason"),
    [
        ({"connected": False}, "offline"),
        ({"localized": False}, "localization"),
        ({"safety_clear": False}, "safety"),
        ({"manual_driving": True}, "manual"),
        ({"active_map": MapRef("other", 1)}, "map"),
        ({"invalidated_task_ids": frozenset({"patrol"})}, "invalidated"),
    ],
)
def test_preconditions_skip_without_dispatch(
    override: dict[str, object],
    reason: str,
) -> None:
    created = datetime(2026, 10, 27, 1, 0, tzinfo=timezone.utc)
    task = interval_task(created)
    due = created + timedelta(minutes=30)
    base = context(
        active_map=task.map_ref,
        last_tick_utc=due - timedelta(seconds=30),
        last_tick_mono=970.0,
    )

    intents = TaskRunner([task]).tick(due, 1000.0, replace(base, **override))

    assert intents == [TaskIntent(task.task_id, IntentAction.SKIP, reason, occurrence_utc=due)]


def test_busy_navigation_skips_other_task() -> None:
    created = datetime(2026, 10, 27, 1, 0, tzinfo=timezone.utc)
    first = interval_task(created, task_id="alpha")
    second = interval_task(created, task_id="beta")
    due = created + timedelta(minutes=30)
    running = ExecutionRecord(
        "exec-alpha",
        first.task_id,
        due,
        ExecutionStatus.RUNNING,
        True,
        False,
    )

    intents = TaskRunner([second, first], [running]).tick(
        due,
        1000.0,
        context(
            active_map=first.map_ref,
            last_tick_utc=due - timedelta(seconds=10),
            last_tick_mono=990.0,
        ),
    )

    assert intents == [TaskIntent(second.task_id, IntentAction.SKIP, "busy", occurrence_utc=due)]


def test_one_tick_dispatches_only_the_latest_occurrence() -> None:
    created = datetime(2026, 10, 27, 1, 0, tzinfo=timezone.utc)
    task = interval_task(created, every_seconds=600)
    end = created + timedelta(minutes=30)

    intents = TaskRunner([task]).tick(
        end,
        1800.0,
        context(active_map=task.map_ref, last_tick_utc=created, last_tick_mono=0.0),
    )

    assert intents == [
        TaskIntent(
            task.task_id,
            IntentAction.SKIP,
            "busy",
            occurrence_utc=created + timedelta(minutes=10),
        ),
        TaskIntent(
            task.task_id,
            IntentAction.SKIP,
            "busy",
            occurrence_utc=created + timedelta(minutes=20),
        ),
        TaskIntent(task.task_id, IntentAction.DISPATCH, "due", occurrence_utc=end),
    ]


def test_one_tick_dispatches_at_most_one_navigation() -> None:
    created = datetime(2026, 10, 27, 1, 0, tzinfo=timezone.utc)
    alpha = interval_task(created, task_id="alpha")
    beta = interval_task(created, task_id="beta")
    due = created + timedelta(minutes=30)

    intents = TaskRunner([beta, alpha]).tick(
        due,
        1800.0,
        context(active_map=alpha.map_ref, last_tick_utc=created, last_tick_mono=0.0),
    )

    assert [item for item in intents if item.action is IntentAction.DISPATCH] == [
        TaskIntent("alpha", IntentAction.DISPATCH, "due", occurrence_utc=due)
    ]
    assert any(
        item.task_id == "beta" and item.action is IntentAction.SKIP and item.occurrence_utc == due
        for item in intents
    )


def test_monotonic_lead_does_not_reopen_scanned_time() -> None:
    created = datetime(2026, 1, 1, tzinfo=timezone.utc)
    task = interval_task(created, every_seconds=10)
    last = created + timedelta(seconds=40)
    now = last + timedelta(seconds=10)

    intents = TaskRunner([task]).tick(
        now,
        130.0,
        context(active_map=task.map_ref, last_tick_utc=last, last_tick_mono=100.0),
    )

    assert all(
        item.skip_from <= item.skip_until
        for item in intents
        if item.skip_from is not None and item.skip_until is not None
    )
    assert all(item.occurrence_utc is None or item.occurrence_utc > last for item in intents)
    assert [item for item in intents if item.action is IntentAction.DISPATCH] == [
        TaskIntent(task.task_id, IntentAction.DISPATCH, "due", occurrence_utc=now)
    ]
