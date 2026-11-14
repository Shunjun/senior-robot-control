"""Durable task ownership on the existing SQLite database."""

import asyncio
import sqlite3
from dataclasses import replace
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import cast

import pytest

from senior_control.errors import DomainError
from senior_control.models import MapRef
from senior_control.storage.database import Database
from senior_control.tasks.lifecycle import Command, Context, Event, EventKind
from senior_control.tasks.lifecycle_store import (
    LifecycleStore,
    RequestConflict,
    StaleEffect,
    VersionConflict,
)
from senior_control.tasks.models import TaskDefinition
from senior_control.tasks.schedule import Schedule
from senior_control.tasks.store import TaskStore

NOW = datetime(2026, 11, 13, tzinfo=timezone.utc)
CONTEXT = Context(True, True, "pose", "map")
SUBMIT = Command("request", "submit", "execution", ("a", "b"))


async def setup_store(
    database: Database, boot: str = "boot"
) -> tuple[LifecycleStore, TaskDefinition]:
    tasks = TaskStore(database, robot_id="robot")
    await tasks.prepare()
    task = await tasks.get("definition")
    if task is None:
        task = await tasks.create(
            TaskDefinition(
                "definition",
                "robot",
                MapRef("map", 1),
                ("a", "b"),
                Schedule.interval(NOW, 60, "UTC"),
                "UTC",
                True,
                0,
                NOW,
            )
        )
    store = LifecycleStore(tasks, boot_id=boot)
    await store.prepare()
    return store, task


async def test_atomic_acceptance_binds_existing_execution_and_request(tmp_path: Path) -> None:
    async with Database(tmp_path / "task.db") as database:
        store, task = await setup_store(database)
        result = await store.submit(SUBMIT, task, NOW, lambda: CONTEXT)
        assert result.state.task_id == "execution" and result.state.phase == "starting"
        assert await store.load("execution") == result.state
        assert await store.query("request") == result.receipt
        assert (await store.effects("execution"))[0].effect == result.effects[0]
        assert await store.tasks.execution_by_id("execution") is not None


async def activate(store: LifecycleStore, task: TaskDefinition) -> None:
    result = await store.submit(SUBMIT, task, NOW, lambda: CONTEXT)
    effect = result.effects[0]
    await store.claim_effect("execution", effect.effect_id)
    await store.acknowledge(
        "execution",
        effect.effect_id,
        effect.epoch,
        True,
        lambda: CONTEXT,
        expected_version=result.state.version,
    )


async def apply_event(store: LifecycleStore, kind: str, action: str | None = None) -> None:

    state = await store.load("execution")
    await store.event(
        "execution",
        Event(cast(EventKind, kind), state.epoch, action),
        lambda: CONTEXT,
        expected_version=state.version,
    )


async def test_replay_conflict_and_journal_remain_authoritative(tmp_path: Path) -> None:

    async with Database(tmp_path / "task.db") as database:
        store, task = await setup_store(database)
        first = await store.submit(SUBMIT, task, NOW, lambda: CONTEXT)
        replay = await store.submit(SUBMIT, task, NOW, lambda: CONTEXT)
        assert replay.receipt == first.receipt and replay.effects == ()
        for cmd in (replace(SUBMIT, actions=("other",)), replace(SUBMIT, task_id="other")):
            with pytest.raises(RequestConflict):
                await store.submit(cmd, task, NOW, lambda: CONTEXT)
        with pytest.raises(RequestConflict):
            await store.submit(
                SUBMIT, replace(task, revision=task.revision + 1), NOW, lambda: CONTEXT
            )
        await database.connection.execute(
            "INSERT INTO agent_operation(operation_id,robot_id,fingerprint,status) "
            "VALUES ('external','robot','other','running')"
        )
        await database.connection.commit()
        with pytest.raises(RequestConflict):
            await store.submit(replace(SUBMIT, request_id="external"), task, NOW, lambda: CONTEXT)
        assert await store.query("missing") is None


@pytest.mark.parametrize(
    "table",
    [
        "task_execution",
        "task_lifecycle",
        "task_lifecycle_effect",
        "agent_operation",
        "task_lifecycle_receipt",
    ],
)
async def test_failure_at_each_acceptance_write_rolls_back_everything(
    tmp_path: Path, table: str
) -> None:

    async with Database(tmp_path / "task.db") as database:
        store, task = await setup_store(database)
        await database.connection.execute(
            f"CREATE TRIGGER abort_accept BEFORE INSERT ON {table} "
            "BEGIN SELECT RAISE(ABORT, 'injected transaction failure'); END"
        )
        await database.connection.commit()
        with pytest.raises(sqlite3.IntegrityError):
            await store.submit(SUBMIT, task, NOW, lambda: CONTEXT)
        assert await store.query("request") is None
        assert await store.tasks.execution_by_id("execution") is None
        for name in (
            "task_lifecycle",
            "task_lifecycle_effect",
            "task_lifecycle_binding",
            "agent_operation",
        ):
            async with database.connection.execute(f"SELECT COUNT(*) FROM {name}") as cursor:
                row = await cursor.fetchone()
            assert row is not None and row[0] == 0


async def test_competing_versions_and_instances_cannot_double_own_motion(tmp_path: Path) -> None:

    async with Database(tmp_path / "task.db") as database:
        store, task = await setup_store(database)
        attempts = await asyncio.gather(
            store.submit(SUBMIT, task, NOW, lambda: CONTEXT),
            store.submit(
                replace(SUBMIT, request_id="second", task_id="second"), task, NOW, lambda: CONTEXT
            ),
            return_exceptions=True,
        )
        assert sum(isinstance(item, DomainError) for item in attempts) == 1
        state = await store.load("execution")
        results = await asyncio.gather(
            store.command(
                Command("pause", "pause", "execution"),
                lambda: CONTEXT,
                expected_version=state.version,
            ),
            store.event(
                "execution", Event("feedback_lost"), lambda: CONTEXT, expected_version=state.version
            ),
            return_exceptions=True,
        )
        assert sum(isinstance(item, VersionConflict) for item in results) == 1


@pytest.mark.parametrize(
    "operation", ["mark_running", "mark_cancel_pending", "pause", "mark_needs_review", "finish"]
)
async def test_legacy_writes_reject_lifecycle_instances(tmp_path: Path, operation: str) -> None:

    async with Database(tmp_path / "task.db") as database:
        store, task = await setup_store(database)
        await activate(store, task)
        before = await store.load("execution")
        with pytest.raises(DomainError):
            if operation == "finish":
                await store.tasks.finish("execution", succeeded=True)
            else:
                await getattr(store.tasks, operation)("execution")
        await store.tasks.mark_active_cancel_pending(0)
        await TaskStore(database, robot_id="robot").prepare()
        assert await store.load("execution") == before
        coarse = await store.tasks.execution_by_id("execution")
        assert coarse is not None and coarse.status.value == "running"


async def test_restart_preserves_progress_receipt_and_requires_new_revoke(tmp_path: Path) -> None:
    path = tmp_path / "task.db"
    async with Database(path) as database:
        store, task = await setup_store(database)
        await activate(store, task)
        await apply_event(store, "action_completed", "a")
        receipt = await store.query("request")
        previous = await store.load("execution")
    async with Database(path) as database:
        store, _task = await setup_store(database, "next-boot")
        assert await store.recover(lambda: CONTEXT) == ("execution",)
        state = await store.load("execution")
        assert state.phase == "revoking" and state.completed == ("a",)
        assert state.epoch > previous.epoch
        assert await store.query("request") == receipt
        effects = await store.effects("execution")
        assert [item.effect.kind for item in effects if item.status == "ready"] == ["revoke"]
        assert await store.recover(lambda: CONTEXT) == ("execution",)
        assert await store.load("execution") == state


async def test_restart_retires_manual_ready_even_for_completed_task(tmp_path: Path) -> None:
    async with Database(tmp_path / "task.db") as database:
        store, task = await setup_store(database)
        await activate(store, task)
        await apply_event(store, "action_completed", "a")
        await apply_event(store, "action_completed", "b")
        state = await store.load("execution")
        stop = await store.command(
            Command("takeover", "takeover", "execution"),
            lambda: CONTEXT,
            expected_version=state.version,
        )
        effect = stop.effects[0]
        await store.claim_effect("execution", effect.effect_id)
        result = await store.acknowledge(
            "execution",
            effect.effect_id,
            effect.epoch,
            True,
            lambda: CONTEXT,
            expected_version=stop.state.version,
        )
        assert result.state.phase == "succeeded" and result.effects[0].kind == "manual_ready"
        other, _ = await setup_store(database, "next-boot")
        await other.recover(lambda: CONTEXT)
        assert all(item.status in {"done", "retired"} for item in await other.effects("execution"))


async def test_cancellation_during_atomic_acceptance_leaves_one_queryable_result(
    tmp_path: Path,
) -> None:

    async with Database(tmp_path / "task.db") as database:
        store, task = await setup_store(database)
        await database.write_lock.acquire()
        pending = asyncio.create_task(store.submit(SUBMIT, task, NOW, lambda: CONTEXT))
        await asyncio.sleep(0)
        await asyncio.sleep(0)
        pending.cancel()
        await asyncio.sleep(0)
        pending.cancel()
        database.write_lock.release()
        with pytest.raises(asyncio.CancelledError):
            await pending
        assert await store.query("request") is not None
        retry = await store.submit(SUBMIT, task, NOW, lambda: CONTEXT)
        assert retry.effects == ()
        assert len(await store.effects("execution")) == 1


async def test_two_database_connections_compete_on_persisted_version(tmp_path: Path) -> None:

    path = tmp_path / "task.db"
    async with Database(path) as first_db, Database(path) as second_db:
        first, task = await setup_store(first_db)
        second, _ = await setup_store(second_db)
        submitted = await first.submit(SUBMIT, task, NOW, lambda: CONTEXT)
        outcomes = await asyncio.gather(
            first.command(
                Command("pause-one", "pause", "execution"),
                lambda: CONTEXT,
                expected_version=submitted.state.version,
            ),
            second.command(
                Command("pause-two", "pause", "execution"),
                lambda: CONTEXT,
                expected_version=submitted.state.version,
            ),
            return_exceptions=True,
        )
        assert sum(isinstance(item, VersionConflict) for item in outcomes) == 1
        assert (await first.load("execution")).phase == "revoking"
        assert len(await first.effects("execution")) == 2


async def test_stale_epoch_and_ack_cannot_change_recovered_state(tmp_path: Path) -> None:

    async with Database(tmp_path / "task.db") as database:
        store, task = await setup_store(database)
        submitted = await store.submit(SUBMIT, task, NOW, lambda: CONTEXT)
        effect = submitted.effects[0]
        next_store, _ = await setup_store(database, "next-boot")
        await next_store.recover(lambda: CONTEXT)
        state = await next_store.load("execution")
        with pytest.raises(StaleEffect):
            await next_store.event(
                "execution",
                Event("action_completed", effect.epoch, "a"),
                lambda: CONTEXT,
                expected_version=state.version,
            )
        with pytest.raises(StaleEffect):
            await next_store.acknowledge(
                "execution",
                effect.effect_id,
                effect.epoch,
                True,
                lambda: CONTEXT,
                expected_version=state.version,
            )
        assert await next_store.load("execution") == state


async def test_different_robots_cannot_reuse_global_operation_id(tmp_path: Path) -> None:

    async with Database(tmp_path / "task.db") as database:
        first, task = await setup_store(database)
        await first.submit(SUBMIT, task, NOW, lambda: CONTEXT)
        other_tasks = TaskStore(database, robot_id="other")
        await other_tasks.prepare()
        other_task = await other_tasks.create(
            replace(task, task_id="other-definition", robot_id="other")
        )
        other = LifecycleStore(other_tasks, boot_id="boot")
        await other.prepare()
        with pytest.raises(RequestConflict):
            await other.submit(
                Command("request", "submit", "other-execution", ("a",)),
                other_task,
                NOW,
                lambda: CONTEXT,
            )
        assert await other.query("request") is None


async def test_multiple_instances_keep_separate_receipts_and_resume_admission(
    tmp_path: Path,
) -> None:
    async with Database(tmp_path / "task.db") as database:
        store, task = await setup_store(database)
        await activate(store, task)
        state = await store.load("execution")
        paused = await store.command(
            Command("pause", "pause", "execution"), lambda: CONTEXT, expected_version=state.version
        )
        revoke = paused.effects[0]
        await store.claim_effect("execution", revoke.effect_id)
        await store.acknowledge(
            "execution",
            revoke.effect_id,
            revoke.epoch,
            True,
            lambda: CONTEXT,
            expected_version=paused.state.version,
        )
        second = replace(SUBMIT, request_id="second", task_id="second-execution")
        await store.submit(second, task, NOW + timedelta(minutes=1), lambda: CONTEXT)
        first_receipt = await store.query("request")
        second_receipt = await store.query("second")
        assert first_receipt is not None and first_receipt.task_id == "execution"
        assert second_receipt is not None and second_receipt.task_id == "second-execution"
        state = await store.load("execution")
        with pytest.raises(VersionConflict):
            await store.command(
                Command("resume", "resume", "execution"),
                lambda: CONTEXT,
                expected_version=state.version,
            )
        assert await store.query("resume") is None
        assert (await store.load("execution")).phase == "paused"


@pytest.mark.parametrize(
    "phase,code", [("before_commit", 23), ("after_commit", 24), ("after_claim", 25)]
)
async def test_abrupt_process_exit_has_only_committed_acceptance(
    tmp_path: Path,
    phase: str,
    code: int,
) -> None:
    import subprocess
    import sys

    path = tmp_path / "task.db"
    async with Database(path) as database:
        await setup_store(database)
    # An actual process exit exercises WAL recovery, not an exception mocked at
    # the repository boundary. No server, navigation process, or ROS is started.
    child = r"""
import asyncio
import os
import sys
from datetime import datetime, timezone
from pathlib import Path
from senior_control.storage.database import Database
from senior_control.tasks.store import TaskStore
from senior_control.tasks.lifecycle import Command, Context
from senior_control.tasks.lifecycle_store import LifecycleStore
async def main():
    async with Database(Path(sys.argv[1])) as database:
        tasks = TaskStore(database, robot_id="robot")
        await tasks.prepare()
        store = LifecycleStore(tasks, boot_id="child-boot")
        await store.prepare()
        task = await tasks.get("definition")
        assert task is not None
        if sys.argv[2] == "before_commit":
            def trace(sql):
                if sql.strip().upper() == "COMMIT":
                    os._exit(23)
            await database.connection.set_trace_callback(trace)
        result = await store.submit(Command("request", "submit", "execution", ("a", "b")),
                                    task, datetime(2026,11,13,tzinfo=timezone.utc),
                                    lambda: Context(True,True,"pose","map"))
        if sys.argv[2] == "after_commit":
            os._exit(24)
        await store.claim_effect("execution", result.effects[0].effect_id)
        os._exit(25)
asyncio.run(main())
"""
    completed = subprocess.run(
        [sys.executable, "-c", child, str(path), phase], capture_output=True, text=True, timeout=15
    )
    assert completed.returncode == code, completed.stderr
    async with Database(path) as database:
        store, task = await setup_store(database, "parent-boot")
        if phase == "before_commit":
            assert await store.query("request") is None
            assert await store.tasks.execution_by_id("execution") is None
        else:
            receipt = await store.query("request")
            assert receipt is not None and receipt.accepted
            before = await store.effects("execution")
            assert before[0].status == ("ready" if phase == "after_commit" else "sending")
            await store.recover(lambda: CONTEXT)
            state = await store.load("execution")
            assert state.phase == "revoking" and state.pending is not None
            assert state.pending.kind == "revoke"
            assert (await store.effects("execution"))[0].status == "retired"
            replay = await store.submit(SUBMIT, task, NOW, lambda: CONTEXT)
            assert replay.receipt == receipt and replay.effects == ()


async def test_active_limits_and_keyset_audit_do_not_scan_history(tmp_path: Path) -> None:
    import json
    from dataclasses import asdict

    async with Database(tmp_path / "task.db") as database:
        store, task = await setup_store(database)
        result = await store.submit(SUBMIT, task, NOW, lambda: CONTEXT)
        effect = result.effects[0]
        rows: list[tuple[str, str, str, str, int | None]] = []
        for i in range(1000):
            old = replace(effect, effect_id=f"old-{i}")
            rows.append((old.effect_id, "execution", json.dumps(asdict(old)), "done", 1))
        for i in range(6):
            active = replace(effect, effect_id=f"active-{i}")
            rows.append(
                (active.effect_id, "execution", json.dumps(asdict(active)), "sending", None)
            )
        await database.connection.executemany(
            "INSERT INTO task_lifecycle_effect VALUES (?,?,?,?,?)",
            rows,
        )
        await database.connection.commit()
        active_rows = await store.active_effects("execution", limit=3)
        assert [item.effect.effect_id for item in active_rows] == [
            effect.effect_id,
            "active-0",
            "active-1",
        ]
        first = await store.effect_page("execution", limit=2)
        assert [item.effect.effect_id for item in first.effects] == [effect.effect_id, "old-0"]
        assert first.next_cursor is not None
        second = await store.effect_page("execution", after=first.next_cursor, limit=2)
        assert [item.effect.effect_id for item in second.effects] == ["old-1", "old-2"]
        async with database.connection.execute(
            "EXPLAIN QUERY PLAN SELECT payload FROM task_lifecycle_effect "
            "INDEXED BY task_lifecycle_effect_active_idx WHERE execution_id=? "
            "AND status IN ('ready','sending') ORDER BY rowid LIMIT ?",
            ("execution", 3),
        ) as cursor:
            plan = await cursor.fetchall()
        assert any("task_lifecycle_effect_active_idx" in str(row) for row in plan)
        with pytest.raises(ValueError):
            await store.active_effects("execution", limit=101)


async def test_store_admission_rejects_context_value_instead_of_provider(tmp_path: Path) -> None:
    from senior_control.tasks.lifecycle_store import ContextProvider

    async with Database(tmp_path / "task.db") as database:
        store, task = await setup_store(database)
        with pytest.raises(TypeError):
            await store.submit(SUBMIT, task, NOW, cast(ContextProvider, CONTEXT))
        assert await store.query("request") is None
        assert await store.tasks.execution_by_id("execution") is None
