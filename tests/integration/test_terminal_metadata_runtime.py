"""Actual Runtime/Service/SQLite continue metadata work after business settlement."""

from dataclasses import replace
from datetime import timedelta

import pytest
from test_task_lifecycle_store import NOW
from test_task_ownership import Navigation, start
from vehicle_manager.agent.production import build_task_runtime
from vehicle_manager.storage.bindings import Database
from vehicle_manager.storage.domain_bindings import TaskStore


async def prepare_terminal(database):
    runtime, executor, nav, live, identity = await start(database)
    nav.progress = nav.progress.model_copy(
        update={
            "completed_action_ids": ["0:a", "1:b"],
            "status": "succeeded",
            "controller_output_exited": True,
        }
    )
    offline = replace(live, connected=False)
    await runtime.tick(NOW + timedelta(seconds=60.5), 1, offline)
    await runtime.tick(NOW + timedelta(seconds=60.6), 1.1, offline)
    state = await executor.lifecycle.store.load(identity)
    assert state.phase == "succeeded" and state.pending is None
    effect = (await executor.lifecycle.store.active_effects(identity))[0].effect
    assert effect.kind == "terminal_settled"
    return (
        runtime,
        executor,
        nav,
        replace(
            offline,
            localized=False,
            safety_clear=False,
            admission_ready=False,
            motion_halt_reason="fault",
        ),
        identity,
        effect,
    )


async def test_runtime_sends_and_retries_terminal_metadata_without_motion_health(tmp_path):
    async with Database(tmp_path / "runtime.db") as database:
        runtime, executor, nav, context, identity, effect = await prepare_terminal(database)
        answer = None
        calls = []
        old_execute, old_query = nav.task_effect, nav.task_effect_status

        async def execute(incoming, definition):
            if incoming.kind != "terminal_settled":
                return await old_execute(incoming, definition)
            assert not database.write_lock.locked()
            assert (await executor.lifecycle.store.load(identity)).phase == "succeeded"
            calls.append(("execute", incoming))
            return answer

        async def query(incoming):
            if incoming.kind != "terminal_settled":
                return await old_query(incoming)
            calls.append(("query", incoming))
            return answer

        nav.task_effect, nav.task_effect_status = execute, query
        assert await executor.lifecycle.store.metadata_executions(limit=1) == (identity,)
        with pytest.raises(ValueError):
            await executor.lifecycle.store.metadata_executions(limit=0)
        plan_count = sum(e.kind == "plan" for e in nav.effects)
        for offset, answer in enumerate((None, False, True)):
            await runtime.tick(
                NOW + timedelta(seconds=60.7 + offset * 0.1), 1.2 + offset * 0.1, context
            )
            assert (await executor.lifecycle.store.load(identity)).phase == "succeeded"
            active = await executor.lifecycle.store.active_effects(identity)
            assert bool(active) is (answer is not True)
            if active:
                assert active[0].status == "sending" and active[0].effect == effect
        assert [kind for kind, _ in calls] == ["execute", "query", "query"]
        assert all(e.effect_id == effect.effect_id and e.dispatch_sequence > 0 for _, e in calls)
        assert len({e.dispatch_sequence for _, e in calls}) == 1
        await runtime.tick(NOW + timedelta(seconds=61.1), 1.6, context)
        assert len(calls) == 3 and sum(e.kind == "plan" for e in nav.effects) == plan_count


async def test_runtime_backend_restart_queries_same_pending_metadata_row(tmp_path):
    path = tmp_path / "restart.db"
    async with Database(path) as database:
        runtime, executor, nav, context, identity, effect = await prepare_terminal(database)

        async def unknown(incoming, _definition):
            assert incoming.kind == "terminal_settled"
            return None

        nav.task_effect = unknown
        await runtime.tick(NOW + timedelta(seconds=60.7), 1.2, context)
        pending = await executor.lifecycle.store.active_effects(identity)
        assert len(pending) == 1 and pending[0].status == "sending"
    async with Database(path) as database:
        nav = Navigation()
        queried = []

        async def no_execution(*_):
            raise AssertionError("restart must not launch route or recreate a cleanup effect")

        async def recovered(incoming):
            assert incoming.kind == "terminal_settled" and incoming.effect_id == effect.effect_id
            queried.append(incoming)
            return True

        nav.task_effect, nav.task_effect_status = no_execution, recovered
        tasks = TaskStore(database, robot_id="robot")
        await tasks.prepare()
        runtime, executor = build_task_runtime(database, tasks, nav, boot_id="new-backend-boot")
        await runtime.tick(NOW + timedelta(seconds=60.8), 1.3, context)
        assert len(queried) == 1
        assert (await executor.lifecycle.store.load(identity)).phase == "succeeded"
        assert not await executor.lifecycle.store.active_effects(identity)
