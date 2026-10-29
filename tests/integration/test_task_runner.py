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


class RecordingNavigation:
    def __init__(self) -> None:
        self.calls: list[str] = []

    async def start(self, execution_id: str, task: TaskDefinition) -> None:
        del task
        self.calls.append(execution_id)


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


async def test_cancel_during_navigation_needs_review(tmp_path: Path) -> None:
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
    assert record.status is ExecutionStatus.NEEDS_REVIEW
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
