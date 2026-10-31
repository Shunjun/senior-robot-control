"""Persisted schedules, claims, and injected navigation."""

import asyncio
import sqlite3
from dataclasses import replace
from datetime import datetime, timedelta, timezone
from pathlib import Path

import pytest

from senior_control.errors import DomainError, ErrorCode
from senior_control.models import MapRef
from senior_control.storage.database import Database
from senior_control.storage.migrations import migrate
from senior_control.storage.schema import DOMAIN_MIGRATIONS
from senior_control.tasks.executor import ProcessCrash, TaskExecutor
from senior_control.tasks.models import (
    ExecutionContext,
    ExecutionStatus,
    IntentAction,
    TaskDefinition,
    TaskIntent,
)
from senior_control.tasks.runner import TaskRunner
from senior_control.tasks.schedule import Schedule
from senior_control.tasks.store import TaskStore

MAP = MapRef("yard", 3)


def interval_task(created: datetime, task_id: str = "patrol") -> TaskDefinition:
    return TaskDefinition(
        task_id,
        "robot-a",
        MAP,
        ("gate", "dock"),
        Schedule.interval(created, 1800),
        "Asia/Shanghai",
        True,
        0,
        created,
    )


def due_intent(task: TaskDefinition, occurrence: datetime) -> TaskIntent:
    return TaskIntent(task.task_id, IntentAction.DISPATCH, "due", occurrence_utc=occurrence)


class HoldNavigation:
    def __init__(self, store: TaskStore) -> None:
        self.store = store
        self.calls: list[str] = []
        self.started = asyncio.Event()
        self.release = asyncio.Event()

    async def start(self, execution_id: str, task: TaskDefinition) -> None:
        del task
        stored = await self.store.execution_by_id(execution_id)
        if (
            stored is None
            or stored.navigation_sent
            or stored.status is not ExecutionStatus.DISPATCHING
        ):
            raise AssertionError("navigation started before the committed execution id existed")
        self.calls.append(execution_id)
        self.started.set()
        await self.release.wait()

    async def cancel(self, execution_id: str) -> None:
        self.calls.append(f"cancel:{execution_id}")


class RecordingNavigation:
    def __init__(self) -> None:
        self.calls: list[str] = []

    async def start(self, execution_id: str, task: TaskDefinition) -> None:
        del task
        self.calls.append(execution_id)

    async def cancel(self, execution_id: str) -> None:
        self.calls.append(f"cancel:{execution_id}")


async def saved_task(database: Database, created: datetime) -> tuple[TaskStore, TaskDefinition]:
    store = TaskStore(database, robot_id="robot-a")
    await store.prepare()
    return store, await store.create(interval_task(created))


async def test_device_reboot_preserves_tasks(tmp_path: Path) -> None:
    created = datetime(2026, 10, 27, 10, 17, 42, tzinfo=timezone.utc)
    path = tmp_path / "robot.db"
    async with Database(path) as database:
        _store, saved = await saved_task(database, created)
    async with Database(path) as database:
        store = TaskStore(database, robot_id="robot-a")
        await store.prepare()
        loaded = await store.get(saved.task_id)

    assert loaded == saved


async def test_version_three_leaves_map_link_unchanged(tmp_path: Path) -> None:
    created = datetime(2026, 10, 27, 1, 0, tzinfo=timezone.utc)
    async with Database(tmp_path / "robot.db") as database:
        await migrate(database, list(DOMAIN_MIGRATIONS))
        async with database.connection.execute("PRAGMA user_version") as cursor:
            version = await cursor.fetchone()
        async with database.connection.execute("PRAGMA table_info(map_task_link)") as cursor:
            columns = [row[1] for row in await cursor.fetchall()]
        store = TaskStore(database, robot_id="robot-a")
        await store.prepare()
        saved = await store.create(interval_task(created))
        async with database.connection.execute(
            "SELECT definition FROM map_task_link WHERE task_id = ?",
            (saved.task_id,),
        ) as cursor:
            link = await cursor.fetchone()

    assert version is not None
    assert version[0] == 3
    assert columns == [
        "task_id",
        "definition",
        "map_id",
        "reference_revision",
        "invalidated",
    ]
    assert link is not None
    assert link[0] == ""


async def test_crash_after_dispatch_never_redispatches_unknown(tmp_path: Path) -> None:
    created = datetime(2026, 10, 27, 1, 0, tzinfo=timezone.utc)
    due = created + timedelta(minutes=30)
    path = tmp_path / "robot.db"
    async with Database(path) as database:
        store, saved = await saved_task(database, created)
        navigation = RecordingNavigation()
        executor = TaskExecutor(database, store, navigation, fault_after="committed")
        with pytest.raises(ProcessCrash):
            await executor.persist_and_dispatch(due_intent(saved, due), saved)
        assert navigation.calls == []

    async with Database(path) as database:
        store = TaskStore(database, robot_id="robot-a")
        await store.prepare()
        saved = await store.get("patrol")
        record = await store.execution_for("patrol", due)
        navigation = RecordingNavigation()
        executor = TaskExecutor(database, store, navigation)
        with pytest.raises(DomainError) as caught:
            assert saved is not None
            await executor.persist_and_dispatch(due_intent(saved, due), saved)

    assert saved is not None
    assert record is not None
    assert record.status is ExecutionStatus.NEEDS_REVIEW
    assert record.navigation_sent is False
    assert record.rollback_confirmed is False
    assert caught.value.code is ErrorCode.CONFLICT
    assert navigation.calls == []
    intents = TaskRunner([saved], [record]).tick(
        due,
        10.0,
        ExecutionContext(
            True,
            True,
            True,
            False,
            saved.map_ref,
            frozenset(),
            due - timedelta(seconds=5),
            5.0,
        ),
    )
    assert intents == []


async def test_claim_conflicts_while_another_execution_is_in_flight(tmp_path: Path) -> None:
    created = datetime(2026, 10, 27, 1, 0, tzinfo=timezone.utc)
    due = created + timedelta(minutes=30)
    later = due + timedelta(minutes=30)
    async with Database(tmp_path / "robot.db") as database:
        store, saved = await saved_task(database, created)
        other = await store.create(interval_task(created, task_id="other"))
        navigation = HoldNavigation(store)
        executor = TaskExecutor(database, store, navigation)
        dispatch = asyncio.create_task(executor.persist_and_dispatch(due_intent(saved, due), saved))
        await asyncio.wait_for(navigation.started.wait(), timeout=1)
        with pytest.raises(DomainError) as caught:
            await store.claim(other, due)
        assert caught.value.code is ErrorCode.CONFLICT
        assert await store.execution_for(other.task_id, due) is None
        navigation.release.set()
        running = await asyncio.wait_for(dispatch, timeout=1)
        with pytest.raises(DomainError) as again:
            await store.claim(saved, later)
        assert again.value.code is ErrorCode.CONFLICT
        assert await store.execution_for(saved.task_id, later) is None

    assert running.status is ExecutionStatus.RUNNING


async def test_cancel_persists_pending_on_a_different_task(tmp_path: Path) -> None:
    created = datetime(2026, 10, 27, 1, 0, tzinfo=timezone.utc)
    due = created + timedelta(minutes=30)
    async with Database(tmp_path / "robot.db") as database:
        store, saved = await saved_task(database, created)
        navigation = HoldNavigation(store)
        executor = TaskExecutor(database, store, navigation)
        writers: list[asyncio.Task[object]] = []
        original = store.mark_cancel_pending

        async def watching(execution_id: str):
            current = asyncio.current_task()
            assert current is not None
            writers.append(current)
            return await original(execution_id)

        store.mark_cancel_pending = watching  # type: ignore[method-assign]
        dispatch = asyncio.create_task(executor.persist_and_dispatch(due_intent(saved, due), saved))
        await asyncio.wait_for(navigation.started.wait(), timeout=1)
        dispatch.cancel()
        with pytest.raises(asyncio.CancelledError):
            await dispatch
        record = await store.execution_for(saved.task_id, due)

    assert writers
    assert writers[0] is not dispatch
    assert record is not None
    assert record.status is ExecutionStatus.CANCEL_PENDING
    assert record.rollback_confirmed is False


async def test_duplicate_claim_conflicts(tmp_path: Path) -> None:
    created = datetime(2026, 10, 27, 1, 0, tzinfo=timezone.utc)
    due = created + timedelta(minutes=30)
    async with Database(tmp_path / "robot.db") as database:
        store, saved = await saved_task(database, created)
        navigation = RecordingNavigation()
        executor = TaskExecutor(database, store, navigation)
        first = await executor.persist_and_dispatch(due_intent(saved, due), saved)
        with pytest.raises(DomainError) as caught:
            await executor.persist_and_dispatch(due_intent(saved, due), saved)

    assert caught.value.code is ErrorCode.CONFLICT
    assert navigation.calls == [first.execution_id]
    assert first.status is ExecutionStatus.RUNNING
    assert first.navigation_sent is True


async def test_dispatch_persists_execution_id_and_releases_the_database(tmp_path: Path) -> None:
    created = datetime(2026, 10, 27, 1, 0, tzinfo=timezone.utc)
    due = created + timedelta(minutes=30)
    async with Database(tmp_path / "robot.db") as database:
        store, saved = await saved_task(database, created)
        other = await store.create(interval_task(created, task_id="other"))
        navigation = HoldNavigation(store)
        executor = TaskExecutor(database, store, navigation)
        dispatch = asyncio.create_task(executor.persist_and_dispatch(due_intent(saved, due), saved))
        await asyncio.wait_for(navigation.started.wait(), timeout=1)
        updated = await asyncio.wait_for(
            store.update(replace(other, enabled=False), expected_revision=other.revision),
            timeout=1,
        )
        navigation.release.set()
        record = await asyncio.wait_for(dispatch, timeout=1)

    assert updated.revision == other.revision + 1
    assert updated.enabled is False
    assert record.execution_id == navigation.calls[0]
    assert record.navigation_sent is True
    assert record.status is ExecutionStatus.RUNNING


async def test_cancel_during_navigation_tracks_pending(tmp_path: Path) -> None:
    created = datetime(2026, 10, 27, 1, 0, tzinfo=timezone.utc)
    due = created + timedelta(minutes=30)
    async with Database(tmp_path / "robot.db") as database:
        store, saved = await saved_task(database, created)
        navigation = HoldNavigation(store)
        executor = TaskExecutor(database, store, navigation)
        dispatch = asyncio.create_task(executor.persist_and_dispatch(due_intent(saved, due), saved))
        await asyncio.wait_for(navigation.started.wait(), timeout=1)
        dispatch.cancel()
        with pytest.raises(asyncio.CancelledError):
            await dispatch
        record = await store.execution_for(saved.task_id, due)

    assert record is not None
    assert record.status is ExecutionStatus.CANCEL_PENDING
    assert record.navigation_sent is False
    assert record.rollback_confirmed is False


async def test_missing_navigation_port_does_not_report_success(tmp_path: Path) -> None:
    created = datetime(2026, 10, 27, 1, 0, tzinfo=timezone.utc)
    due = created + timedelta(minutes=30)
    async with Database(tmp_path / "robot.db") as database:
        store, saved = await saved_task(database, created)
        record = await TaskExecutor(database, store, None).persist_and_dispatch(
            due_intent(saved, due),
            saved,
        )
        stored = await store.execution_for(saved.task_id, due)

    assert record.status is ExecutionStatus.NEEDS_REVIEW
    assert record.navigation_sent is False
    assert record.rollback_confirmed is False
    assert stored == record


async def test_invalidated_link_keeps_definition_and_blocks_dispatch(tmp_path: Path) -> None:
    created = datetime(2026, 10, 27, 1, 0, tzinfo=timezone.utc)
    due = created + timedelta(minutes=30)
    async with Database(tmp_path / "robot.db") as database:
        store, saved = await saved_task(database, created)
        async with database.write_lock:
            await database.connection.execute(
                "UPDATE map_task_link SET invalidated = 1 WHERE task_id = ?",
                (saved.task_id,),
            )
            await database.connection.commit()
        revised = await store.update(
            replace(saved, waypoint_ids=("dock",)),
            expected_revision=saved.revision,
        )
        async with database.connection.execute(
            "SELECT definition, invalidated FROM map_task_link WHERE task_id = ?",
            (saved.task_id,),
        ) as cursor:
            link = await cursor.fetchone()
        navigation = RecordingNavigation()
        with pytest.raises(DomainError) as caught:
            await TaskExecutor(database, store, navigation).persist_and_dispatch(
                due_intent(revised, due),
                revised,
            )
        loaded = await store.get(saved.task_id)

    assert link == ("", 1)
    assert loaded == revised
    assert loaded is not None
    assert loaded.waypoint_ids == ("dock",)
    assert caught.value.code is ErrorCode.CONFLICT
    assert navigation.calls == []
    intents = TaskRunner([loaded]).tick(
        due,
        1000.0,
        ExecutionContext(
            True,
            True,
            True,
            False,
            loaded.map_ref,
            frozenset({loaded.task_id}),
            due - timedelta(seconds=30),
            970.0,
        ),
    )
    assert intents == [
        TaskIntent(loaded.task_id, IntentAction.SKIP, "invalidated", occurrence_utc=due)
    ]


async def test_concurrent_update_conflict_is_bounded(tmp_path: Path) -> None:
    created = datetime(2026, 10, 27, 1, 0, tzinfo=timezone.utc)
    async with Database(tmp_path / "robot.db") as database:
        store, saved = await saved_task(database, created)
        results = await asyncio.wait_for(
            asyncio.gather(
                store.update(replace(saved, enabled=False), expected_revision=saved.revision),
                store.update(
                    replace(saved, waypoint_ids=("dock",)),
                    expected_revision=saved.revision,
                ),
                return_exceptions=True,
            ),
            timeout=1,
        )
        stored = await store.get(saved.task_id)

    successes = [item for item in results if isinstance(item, TaskDefinition)]
    conflicts = [item for item in results if isinstance(item, DomainError)]
    assert len(successes) == 1
    assert len(conflicts) == 1
    assert conflicts[0].code is ErrorCode.CONFLICT
    assert successes[0].revision == saved.revision + 1
    assert stored == successes[0]


async def test_disk_full_has_no_success_ack(tmp_path: Path) -> None:
    created = datetime(2026, 10, 27, 1, 0, tzinfo=timezone.utc)
    async with Database(tmp_path / "robot.db") as database:
        store = TaskStore(database, robot_id="robot-a")
        await store.prepare()
        async with database.connection.execute("PRAGMA page_count") as cursor:
            count = (await cursor.fetchone())[0]
        await database.connection.execute(f"PRAGMA max_page_count = {count}")
        oversized = replace(interval_task(created), waypoint_ids=("x" * 1_048_576,))
        with pytest.raises(sqlite3.OperationalError, match="full"):
            await store.create(oversized)
        assert await store.get("patrol") is None


async def test_pause_persists_and_lets_the_next_occurrence_claim(tmp_path: Path) -> None:
    created = datetime(2026, 10, 27, 1, 0, tzinfo=timezone.utc)
    due = created + timedelta(minutes=30)
    following = due + timedelta(minutes=30)
    async with Database(tmp_path / "robot.db") as database:
        store, saved = await saved_task(database, created)
        executor = TaskExecutor(database, store, RecordingNavigation())
        running = await executor.persist_and_dispatch(due_intent(saved, due), saved)
        await executor.apply(
            TaskIntent(
                saved.task_id,
                IntentAction.PAUSE,
                "manual",
                occurrence_utc=due,
                execution_id=running.execution_id,
            ),
            saved,
        )
        paused = await store.execution_by_id(running.execution_id)
        claimed = await store.claim(saved, following)
        still_paused = await store.execution_by_id(running.execution_id)

    assert paused is not None
    assert paused.status is ExecutionStatus.PAUSED
    assert claimed.status is ExecutionStatus.DISPATCHING
    assert still_paused is not None
    assert still_paused.status is ExecutionStatus.PAUSED


async def test_claim_rejects_a_disabled_or_deleted_task(tmp_path: Path) -> None:
    created = datetime(2026, 10, 27, 1, 0, tzinfo=timezone.utc)
    due = created + timedelta(minutes=30)
    async with Database(tmp_path / "robot.db") as database:
        store, saved = await saved_task(database, created)
        disabled = await store.update(
            replace(saved, enabled=False), expected_revision=saved.revision
        )
        caller = replace(disabled, enabled=True)
        with pytest.raises(DomainError) as caught:
            await store.claim(caller, due)
        assert caught.value.code is ErrorCode.CONFLICT
        await database.connection.execute(
            """
            CREATE TABLE IF NOT EXISTS agent_deleted_task (
                task_id TEXT PRIMARY KEY,
                robot_id TEXT NOT NULL,
                operation_id TEXT NOT NULL,
                revision INTEGER NOT NULL
            )
            """
        )
        enabled = await store.update(
            replace(disabled, enabled=True), expected_revision=disabled.revision
        )
        await database.connection.execute(
            """
            INSERT INTO agent_deleted_task (task_id, robot_id, operation_id, revision)
            VALUES (?, ?, ?, ?)
            """,
            (enabled.task_id, "robot-a", "op-delete", enabled.revision),
        )
        await database.connection.commit()
        with pytest.raises(DomainError) as deleted:
            await store.claim(enabled, due)

    assert deleted.value.code is ErrorCode.CONFLICT


async def test_offline_link_does_not_start_navigation(tmp_path: Path) -> None:
    created = datetime(2026, 10, 27, 1, 0, tzinfo=timezone.utc)
    due = created + timedelta(minutes=30)
    async with Database(tmp_path / "robot.db") as database:
        store, saved = await saved_task(database, created)
        navigation = RecordingNavigation()
        executor = TaskExecutor(database, store, navigation, link_up=lambda: False)
        record = await executor.persist_and_dispatch(due_intent(saved, due), saved)

    assert navigation.calls == []
    assert record.status is ExecutionStatus.NEEDS_REVIEW
    assert record.navigation_sent is False


async def test_pause_cancels_a_sent_goal(tmp_path: Path) -> None:
    created = datetime(2026, 10, 27, 1, 0, tzinfo=timezone.utc)
    due = created + timedelta(minutes=30)
    async with Database(tmp_path / "robot.db") as database:
        store, saved = await saved_task(database, created)
        navigation = RecordingNavigation()
        executor = TaskExecutor(database, store, navigation)
        running = await executor.persist_and_dispatch(due_intent(saved, due), saved)
        await executor.apply(
            TaskIntent(
                saved.task_id,
                IntentAction.PAUSE,
                "manual",
                occurrence_utc=due,
                execution_id=running.execution_id,
            ),
            saved,
        )

    assert f"cancel:{running.execution_id}" in navigation.calls


async def test_skipped_occurrence_occupies_the_claim_key(tmp_path: Path) -> None:
    created = datetime(2026, 10, 27, 1, 0, tzinfo=timezone.utc)
    due = created + timedelta(minutes=30)
    following = due + timedelta(minutes=30)
    async with Database(tmp_path / "robot.db") as database:
        store, saved = await saved_task(database, created)
        executor = TaskExecutor(database, store, RecordingNavigation())
        await executor.apply(
            TaskIntent(saved.task_id, IntentAction.SKIP, "busy", occurrence_utc=due),
            saved,
        )
        skipped = await store.execution_for(saved.task_id, due)
        with pytest.raises(DomainError) as caught:
            await store.claim(saved, due)
        claimed = await store.claim(saved, following)

    assert skipped is not None
    assert skipped.status is ExecutionStatus.SKIPPED
    assert caught.value.code is ErrorCode.CONFLICT
    assert claimed.status is ExecutionStatus.DISPATCHING


async def test_executor_records_clock_jump_interval(tmp_path: Path) -> None:
    created = datetime(2026, 1, 1, tzinfo=timezone.utc)
    skip_from = created + timedelta(minutes=10)
    skip_until = skip_from + timedelta(hours=2)
    async with Database(tmp_path / "robot.db") as database:
        store, saved = await saved_task(database, created)
        executor = TaskExecutor(database, store, None)
        await executor.apply(
            TaskIntent(
                saved.task_id,
                IntentAction.SKIP,
                "clock_jump",
                skip_from=skip_from,
                skip_until=skip_until,
            ),
            saved,
        )
        loaded = await store.skips()

    assert len(loaded) == 1
    assert loaded[0].reason == "clock_jump"
    assert loaded[0].skip_from == skip_from
    assert loaded[0].skip_until == skip_until


async def test_skip_interval_survives_restart(tmp_path: Path) -> None:
    created = datetime(2026, 1, 1, tzinfo=timezone.utc)
    path = tmp_path / "robot.db"
    skip_from = created + timedelta(minutes=10)
    skip_until = skip_from + timedelta(hours=2)
    async with Database(path) as database:
        store, saved = await saved_task(database, created)
        recorded = await store.record_skip(saved.task_id, skip_from, skip_until, "clock_jump")
    async with Database(path) as database:
        store = TaskStore(database, robot_id="robot-a")
        await store.prepare()
        loaded = await store.skips()

    assert loaded == (recorded,)


async def test_store_lists_tasks_and_executions_for_robot(tmp_path: Path) -> None:
    created = datetime(2026, 10, 27, tzinfo=timezone.utc)
    async with Database(tmp_path / "robot.db") as database:
        store, task = await saved_task(database, created)
        record = await store.claim(task, created + timedelta(minutes=30))
        assert await store.list_tasks() == (task,)
        assert await store.list_executions() == (record,)


async def test_runtime_dispatches_due_task_and_cancels_on_disconnect(tmp_path: Path) -> None:
    from senior_control.tasks.runtime import TaskRuntime

    created = datetime(2026, 10, 27, tzinfo=timezone.utc)
    async with Database(tmp_path / "robot.db") as database:
        store, task = await saved_task(database, created)
        navigation = RecordingNavigation()
        executor = TaskExecutor(database, store, navigation)
        runtime = TaskRuntime(store, executor)
        context = ExecutionContext(True, True, True, False, task.map_ref)
        await runtime.tick(created + timedelta(seconds=1799), 10.0, context)
        await runtime.tick(created + timedelta(seconds=1801), 12.0, context)
        assert len(navigation.calls) == 1
        await runtime.tick(
            created + timedelta(seconds=1802), 13.0, replace(context, connected=False)
        )
        assert navigation.calls[1] == f"cancel:{navigation.calls[0]}"


async def test_runtime_restart_records_offline_gap_without_dispatch(tmp_path: Path) -> None:
    from senior_control.tasks.runtime import TaskRuntime

    created = datetime(2026, 10, 27, tzinfo=timezone.utc)
    path = tmp_path / "robot.db"
    async with Database(path) as database:
        store, task = await saved_task(database, created)
        runtime = TaskRuntime(store, TaskExecutor(database, store, RecordingNavigation()))
        context = ExecutionContext(True, True, True, False, task.map_ref)
        await runtime.tick(created, 10.0, context)
    async with Database(path) as database:
        store = TaskStore(database, robot_id="robot-a")
        await store.prepare()
        navigation = RecordingNavigation()
        runtime = TaskRuntime(store, TaskExecutor(database, store, navigation))
        await runtime.tick(created + timedelta(hours=1), 1.0, context)
        assert navigation.calls == []
        assert len(await store.skips()) == 1


async def test_runtime_recovers_after_navigation_start_failure(tmp_path: Path) -> None:
    from senior_control.tasks.runtime import TaskRuntime

    class Flaky(RecordingNavigation):
        async def start(self, execution_id: str, task: TaskDefinition) -> None:
            if not self.calls:
                self.calls.append("failed")
                raise OSError("navigation unavailable")
            await super().start(execution_id, task)

    created = datetime(2026, 10, 27, tzinfo=timezone.utc)
    async with Database(tmp_path / "robot.db") as database:
        store, task = await saved_task(database, created)
        nav = Flaky()
        runtime = TaskRuntime(store, TaskExecutor(database, store, nav))
        context = ExecutionContext(True, True, True, False, task.map_ref)
        await runtime.tick(created + timedelta(seconds=1799), 1.0, context)
        await runtime.tick(created + timedelta(seconds=1800), 2.0, context)
        first = await store.execution_for(task.task_id, created + timedelta(seconds=1800))
        assert first is not None and first.status is ExecutionStatus.NEEDS_REVIEW
        await runtime.tick(created + timedelta(seconds=3599), 1801.0, context)
        await runtime.tick(created + timedelta(seconds=3600), 1802.0, context)
        second = await store.execution_for(task.task_id, created + timedelta(seconds=3600))
        assert second is not None and second.status is ExecutionStatus.RUNNING


async def test_runtime_retries_unknown_cancel_even_after_connection_returns(tmp_path: Path) -> None:
    from senior_control.tasks.runtime import TaskRuntime

    class FlakyCancel(RecordingNavigation):
        async def cancel(self, execution_id: str) -> None:
            self.calls.append(f"cancel:{execution_id}")
            if len([c for c in self.calls if c.startswith("cancel:")]) == 1:
                raise OSError("timeout")

    created = datetime(2026, 10, 27, tzinfo=timezone.utc)
    async with Database(tmp_path / "robot.db") as database:
        store, task = await saved_task(database, created)
        nav = FlakyCancel()
        runtime = TaskRuntime(store, TaskExecutor(database, store, nav))
        live = ExecutionContext(True, True, True, False, task.map_ref)
        await runtime.tick(created + timedelta(seconds=1799), 1.0, live)
        await runtime.tick(created + timedelta(seconds=1800), 2.0, live)
        record = await store.execution_for(task.task_id, created + timedelta(seconds=1800))
        assert record is not None
        await runtime.tick(created + timedelta(seconds=1801), 3.0, replace(live, connected=False))
        pending = await store.execution_by_id(record.execution_id)
        assert pending is not None and pending.status.value == "cancel_pending"
        await runtime.tick(created + timedelta(seconds=1802), 4.0, live)
        paused = await store.execution_by_id(record.execution_id)
        assert paused is not None and paused.status is ExecutionStatus.PAUSED
        assert nav.calls.count(f"cancel:{record.execution_id}") == 2


async def test_scan_loads_only_active_and_window_records(tmp_path: Path) -> None:
    created = datetime(2026, 1, 1, tzinfo=timezone.utc)
    async with Database(tmp_path / "robot.db") as database:
        store, task = await saved_task(database, created)
        rows = [
            (
                f"old-{i}",
                task.task_id,
                (created + timedelta(seconds=i)).isoformat(),
                "skipped",
                0,
                0,
            )
            for i in range(3000)
        ]
        rows += [
            (
                "active",
                task.task_id,
                (created + timedelta(seconds=3001)).isoformat(),
                "running",
                1,
                0,
            ),
            (
                "window",
                task.task_id,
                (created + timedelta(seconds=4001)).isoformat(),
                "skipped",
                0,
                0,
            ),
        ]
        await database.connection.executemany(
            "INSERT INTO task_execution VALUES (?, ?, ?, ?, ?, ?)", rows
        )
        await database.connection.commit()
        selected = await store.executions_for_scan(
            created + timedelta(seconds=4000), created + timedelta(seconds=4002)
        )
        assert {row.execution_id for row in selected} == {"active", "window"}


async def test_pending_cancel_survives_restart_and_blocks_new_claim(tmp_path: Path) -> None:
    created = datetime(2026, 10, 27, tzinfo=timezone.utc)
    path = tmp_path / "robot.db"
    async with Database(path) as database:
        store, task = await saved_task(database, created)
        record = await store.claim(task, created + timedelta(seconds=1800))
        await store.mark_cancel_pending(record.execution_id)
    async with Database(path) as database:
        store = TaskStore(database, robot_id="robot-a")
        await store.prepare()
        pending = await store.execution_by_id(record.execution_id)
        assert pending is not None and pending.status is ExecutionStatus.CANCEL_PENDING
        with pytest.raises(DomainError):
            await store.claim(task, created + timedelta(seconds=3600))


async def test_scan_overflow_fails_closed_with_bounded_result(tmp_path: Path) -> None:
    created = datetime(2026, 10, 27, tzinfo=timezone.utc)
    async with Database(tmp_path / "robot.db") as database:
        store, task = await saved_task(database, created)
        rows = [
            (
                f"r-{i}",
                task.task_id,
                (created + timedelta(seconds=i + 1)).isoformat(),
                "skipped",
                0,
                0,
            )
            for i in range(4097)
        ]
        await database.connection.executemany(
            "INSERT INTO task_execution VALUES (?, ?, ?, ?, ?, ?)", rows
        )
        await database.connection.commit()
        with pytest.raises(RuntimeError, match="limit exceeded"):
            await store.executions_for_scan(created, created + timedelta(seconds=5000))


async def test_failed_start_with_unknown_cancel_remains_pending_until_confirmed(
    tmp_path: Path,
) -> None:
    from senior_control.tasks.runtime import TaskRuntime

    class BrokenStart(RecordingNavigation):
        async def start(self, execution_id: str, task: TaskDefinition) -> None:
            self.calls.append(f"start:{execution_id}")
            raise OSError("start outcome unknown")

        async def cancel(self, execution_id: str) -> None:
            self.calls.append(f"cancel:{execution_id}")
            if self.calls.count(f"cancel:{execution_id}") == 1:
                raise OSError("cancel outcome unknown")

    created = datetime(2026, 10, 27, tzinfo=timezone.utc)
    async with Database(tmp_path / "robot.db") as database:
        store, task = await saved_task(database, created)
        nav = BrokenStart()
        runtime = TaskRuntime(store, TaskExecutor(database, store, nav))
        live = ExecutionContext(True, True, True, False, task.map_ref)
        await runtime.tick(created + timedelta(seconds=1799), 1.0, live)
        await runtime.tick(created + timedelta(seconds=1800), 2.0, live)
        record = await store.execution_for(task.task_id, created + timedelta(seconds=1800))
        assert record is not None and record.status is ExecutionStatus.CANCEL_PENDING
        await runtime.tick(created + timedelta(seconds=1801), 3.0, live)
        record = await store.execution_by_id(record.execution_id)
        assert record is not None and record.status is ExecutionStatus.NEEDS_REVIEW
        assert record.rollback_confirmed is False


async def test_scan_queries_use_indexes_for_existing_database(tmp_path: Path) -> None:
    from senior_control.tasks.store import ACTIVE_SCAN_SQL, WINDOW_SCAN_SQL

    path = tmp_path / "robot.db"
    async with Database(path) as database:
        store = TaskStore(database, robot_id="robot-a")
        await store.prepare()
        await database.connection.execute("DROP INDEX task_execution_scan_idx")
        await database.connection.execute("DROP INDEX task_execution_window_idx")
        await database.connection.commit()
    async with Database(path) as database:
        store = TaskStore(database, robot_id="robot-a")
        await store.prepare()
        queries = (
            (ACTIVE_SCAN_SQL, ("robot-a", "dispatching", "running", "cancel_pending", 4097)),
            (
                WINDOW_SCAN_SQL,
                ("robot-a", "start", "end", "dispatching", "running", "cancel_pending", 4097),
            ),
        )
        for sql, params in queries:
            async with database.connection.execute("EXPLAIN QUERY PLAN " + sql, params) as cursor:
                plan = " ".join(str(row[3]) for row in await cursor.fetchall())
            assert "SEARCH e USING INDEX task_execution_" in plan
            assert "SCAN e" not in plan


async def test_runtime_propagates_database_failure(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    from senior_control.tasks.runtime import TaskRuntime

    async with Database(tmp_path / "robot.db") as database:
        store = TaskStore(database, robot_id="robot-a")
        await store.prepare()
        runtime = TaskRuntime(store, TaskExecutor(database, store, None))

        async def fail() -> tuple[TaskDefinition, ...]:
            raise sqlite3.OperationalError("database unavailable")

        monkeypatch.setattr(store, "list_tasks", fail)
        with pytest.raises(sqlite3.OperationalError, match="database unavailable"):
            await runtime.tick(
                datetime.now(timezone.utc), 0.0, ExecutionContext(False, False, False, False, None)
            )
