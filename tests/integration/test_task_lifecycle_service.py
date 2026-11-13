"""Effects run only after SQLite commit, and uncertainty is queried."""

import asyncio
import sqlite3
from dataclasses import replace
from pathlib import Path
from typing import Literal

import pytest
from test_task_lifecycle_store import CONTEXT, NOW, SUBMIT, setup_store

from senior_control.storage.database import Database
from senior_control.tasks.lifecycle import Action, Command, Context, Effect, Event, Lifecycle
from senior_control.tasks.lifecycle_service import LifecycleService


class Port:
    def __init__(self) -> None:
        self.sent: list[Effect] = []
        self.queried: list[Effect] = []
        self.answer: bool | None = True

    async def execute(self, effect: Effect) -> bool | None:
        self.sent.append(effect)
        return self.answer

    async def query(self, effect: Effect) -> bool | None:
        self.queried.append(effect)
        return self.answer


async def test_service_dispatches_committed_plan_and_acks(tmp_path: Path) -> None:
    async with Database(tmp_path / "task.db") as database:
        store, task = await setup_store(database)
        port = Port()
        service = LifecycleService(store, port, context=lambda: CONTEXT)
        await service.start()
        await service.submit(SUBMIT, task, NOW)
        await service.reconcile("execution")
        assert len(port.sent) == 1
        assert (await store.load("execution")).phase == "running"


async def test_port_observes_committed_snapshot_without_database_lock(tmp_path: Path) -> None:

    async with Database(tmp_path / "task.db") as database:
        store, task = await setup_store(database)

        class Inspect(Port):
            async def execute(self, effect: Effect) -> bool | None:
                assert not database.write_lock.locked()
                assert (await asyncio.wait_for(store.load("execution"), 1)).phase == "starting"
                assert await store.query("request") is not None
                return await super().execute(effect)

        service = LifecycleService(store, Inspect(), context=lambda: CONTEXT)
        await service.submit(SUBMIT, task, NOW)
        await service.reconcile("execution")
        assert (await store.load("execution")).phase == "running"


async def test_unknown_send_is_only_queried_and_offline_does_not_cancel(tmp_path: Path) -> None:

    async with Database(tmp_path / "task.db") as database:
        store, task = await setup_store(database)
        port = Port()
        port.answer = None
        current = CONTEXT
        service = LifecycleService(store, port, context=lambda: current)
        accepted = await service.submit(SUBMIT, task, NOW)
        await service.reconcile("execution")
        assert (await store.load("execution")).phase == "starting"
        current = replace(CONTEXT, connected=False)
        port.answer = True
        await service.reconcile("execution")
        await service.reconcile("execution")
        assert len(port.sent) == 1 and len(port.queried) == 1
        assert (await store.load("execution")).phase == "running"
        assert await store.query("request") == accepted.receipt


async def test_send_exception_then_query_never_reexecutes(tmp_path: Path) -> None:
    async with Database(tmp_path / "task.db") as database:
        store, task = await setup_store(database)

        class LostReply(Port):
            async def execute(self, effect: Effect) -> bool | None:
                self.sent.append(effect)
                raise OSError("reply lost after acceptance")

        port = LostReply()
        service = LifecycleService(store, port, context=lambda: CONTEXT)
        await service.submit(SUBMIT, task, NOW)
        await service.reconcile("execution")
        await service.reconcile("execution")
        assert len(port.sent) == 1 and len(port.queried) == 1
        assert (await store.load("execution")).phase == "running"


async def test_cancelled_wait_keeps_committed_sending_obligation_queryable(tmp_path: Path) -> None:

    async with Database(tmp_path / "task.db") as database:
        store, task = await setup_store(database)
        entered = asyncio.Event()

        class Hold(Port):
            async def execute(self, effect: Effect) -> bool | None:
                self.sent.append(effect)
                entered.set()
                await asyncio.Event().wait()
                return True

        port = Hold()
        service = LifecycleService(store, port, context=lambda: CONTEXT)
        await service.submit(SUBMIT, task, NOW)
        dispatch = asyncio.create_task(service.reconcile("execution"))
        await entered.wait()
        dispatch.cancel()
        with pytest.raises(asyncio.CancelledError):
            await dispatch
        assert (await store.effects("execution"))[0].status == "sending"
        await service.reconcile("execution")
        assert len(port.sent) == 1 and len(port.queried) == 1
        assert (await store.load("execution")).phase == "running"


async def test_takeover_does_not_wait_for_pending_plan_io_and_late_ack_is_fenced(
    tmp_path: Path,
) -> None:

    async with Database(tmp_path / "task.db") as database:
        store, task = await setup_store(database)
        entered, release = asyncio.Event(), asyncio.Event()

        class HoldPlan(Port):
            async def execute(self, effect: Effect) -> bool | None:
                self.sent.append(effect)
                if effect.kind == "plan":
                    entered.set()
                    await release.wait()
                return True

        port = HoldPlan()
        service = LifecycleService(store, port, context=lambda: CONTEXT)
        accepted = await service.submit(SUBMIT, task, NOW)
        start = asyncio.create_task(service.reconcile("execution"))
        await entered.wait()
        stop = await asyncio.wait_for(
            service.command(
                Command("takeover", "takeover", "execution"),
                expected_version=accepted.state.version,
            ),
            1,
        )
        assert stop.state.phase == "revoking"
        await service.reconcile("execution")
        assert (await store.load("execution")).phase == "paused"
        release.set()
        await start
        assert (await store.load("execution")).phase == "paused"
        await service.reconcile("execution")
        await service.reconcile("execution")
        assert [effect.kind for effect in port.sent] == ["plan", "revoke", "manual_ready"]


async def test_restart_retires_ready_plan_before_any_dispatch(tmp_path: Path) -> None:
    path = tmp_path / "task.db"
    async with Database(path) as database:
        store, task = await setup_store(database)
        await store.submit(SUBMIT, task, NOW, lambda: CONTEXT)
    async with Database(path) as database:
        store, _ = await setup_store(database, "new-boot")
        port = Port()
        service = LifecycleService(store, port, context=lambda: CONTEXT)
        await service.reconcile("execution")
        assert [effect.kind for effect in port.sent] == ["revoke"]
        assert (await store.load("execution")).phase == "paused"
        await service.reconcile("execution")
        assert len(port.sent) == 1


async def test_ack_transaction_failure_is_reconciled_by_query_without_repeat_send(
    tmp_path: Path,
) -> None:

    async with Database(tmp_path / "task.db") as database:
        store, task = await setup_store(database)
        port = Port()
        service = LifecycleService(store, port, context=lambda: CONTEXT)
        await service.submit(SUBMIT, task, NOW)
        await database.connection.execute(
            "CREATE TRIGGER abort_ack BEFORE UPDATE ON task_lifecycle "
            "BEGIN SELECT RAISE(ABORT, 'ack rollback'); END"
        )
        await database.connection.commit()
        with pytest.raises(sqlite3.IntegrityError):
            await service.reconcile("execution")
        assert (await store.load("execution")).phase == "starting"
        assert (await store.effects("execution"))[0].status == "sending"
        await database.connection.execute("DROP TRIGGER abort_ack")
        await database.connection.commit()
        await service.reconcile("execution")
        assert len(port.sent) == 1 and len(port.queried) == 1
        assert (await store.load("execution")).phase == "running"


@pytest.mark.parametrize("allowed", [True, False])
async def test_ack_samples_context_after_waiting_for_database_transaction(
    tmp_path: Path, allowed: bool
) -> None:

    async with Database(tmp_path / "task.db") as database:
        store, task = await setup_store(database)
        port = Port()
        current = CONTEXT
        service = LifecycleService(store, port, context=lambda: current)
        await service.submit(SUBMIT, task, NOW)
        await service.reconcile("execution")
        state = await store.load("execution")
        lost = await service.event(
            "execution", Event("feedback_lost"), expected_version=state.version
        )
        recovered = await service.event(
            "execution", Event("feedback_recovered"), expected_version=lost.state.version
        )
        effect = lost.effects[0]
        await store.claim_effect("execution", effect.effect_id)
        await database.write_lock.acquire()
        pending = asyncio.create_task(
            service.acknowledge(effect, True, expected_version=recovered.state.version)
        )
        await asyncio.sleep(0)
        await asyncio.sleep(0)
        current = replace(
            CONTEXT, pose_ref="latest-pose", map_ref="latest-map", motion_allowed=allowed
        )
        database.write_lock.release()
        result = await pending
        if allowed:
            assert result.effects[0].pose_ref == "latest-pose"
            assert result.effects[0].map_ref == "latest-map"
        else:
            assert result.state.phase == "paused" and result.effects == ()


async def test_rejected_plan_requires_revoke_and_unconfirmed_revoke_stays_queryable(
    tmp_path: Path,
) -> None:
    async with Database(tmp_path / "task.db") as database:
        store, task = await setup_store(database)
        port = Port()
        port.answer = False
        service = LifecycleService(store, port, context=lambda: CONTEXT)
        await service.submit(SUBMIT, task, NOW)
        await service.reconcile("execution")
        assert (await store.load("execution")).phase == "revoking"
        await service.reconcile("execution")
        assert (await store.load("execution")).phase == "revoking"
        port.answer = True
        await service.reconcile("execution")
        assert (await store.load("execution")).phase == "paused"
        assert [effect.kind for effect in port.sent] == ["plan", "revoke"]
        assert [effect.kind for effect in port.queried] == ["revoke"]


async def test_manual_consumption_persists_and_does_not_reauthorize_on_restart(
    tmp_path: Path,
) -> None:

    path = tmp_path / "task.db"
    async with Database(path) as database:
        store, task = await setup_store(database)
        port = Port()
        service = LifecycleService(store, port, context=lambda: CONTEXT)
        await service.submit(SUBMIT, task, NOW)
        await service.reconcile("execution")
        state = await store.load("execution")
        await service.command(
            Command("takeover", "takeover", "execution"), expected_version=state.version
        )
        await service.reconcile("execution")
        port.answer = None
        await service.reconcile("execution")
        assert [effect.kind for effect in port.sent].count("manual_ready") == 1
        port.answer = True
        await service.reconcile("execution")
        await service.reconcile("execution")
        manual = [
            item for item in await store.effects("execution") if item.effect.kind == "manual_ready"
        ]
        assert len(manual) == 1 and manual[0].status == "done" and manual[0].result is True
        assert [effect.kind for effect in port.queried] == ["manual_ready"]
        assert [effect.kind for effect in port.sent].count("manual_ready") == 1
    async with Database(path) as database:
        store, _ = await setup_store(database, "next-boot")
        port = Port()
        await LifecycleService(store, port, context=lambda: CONTEXT).reconcile("execution")
        assert not port.sent and not port.queried


async def test_failure_priority_and_progress_survive_database_reopen(tmp_path: Path) -> None:

    path = tmp_path / "task.db"
    async with Database(path) as database:
        store, task = await setup_store(database)
        port = Port()
        service = LifecycleService(store, port, context=lambda: CONTEXT)
        await service.submit(SUBMIT, task, NOW)
        await service.reconcile("execution")
        state = await store.load("execution")
        progress = await service.event(
            "execution", Event("action_completed", state.epoch, "a"), expected_version=state.version
        )
        failed = await service.event(
            "execution", Event("failed", state.epoch), expected_version=progress.state.version
        )
        await service.command(
            Command("pause", "pause", "execution"), expected_version=failed.state.version
        )
    async with Database(path) as database:
        store, _ = await setup_store(database, "next-boot")
        port = Port()
        service = LifecycleService(store, port, context=lambda: CONTEXT)
        await service.reconcile("execution")
        state = await store.load("execution")
        assert state.phase == "failed" and state.completed == ("a",)
        assert [effect.kind for effect in port.sent] == ["revoke"]


async def test_failed_sending_claim_never_calls_external_port(tmp_path: Path) -> None:
    async with Database(tmp_path / "task.db") as database:
        store, task = await setup_store(database)
        port = Port()
        service = LifecycleService(store, port, context=lambda: CONTEXT)
        await service.submit(SUBMIT, task, NOW)
        await database.connection.execute(
            "CREATE TRIGGER abort_claim BEFORE UPDATE ON task_lifecycle_effect "
            "BEGIN SELECT RAISE(ABORT, 'claim rollback'); END"
        )
        await database.connection.commit()
        with pytest.raises(sqlite3.IntegrityError):
            await service.reconcile("execution")
        assert port.sent == []
        assert (await store.effects("execution"))[0].status == "ready"
        await database.connection.execute("DROP TRIGGER abort_claim")
        await database.connection.commit()
        await service.reconcile("execution")
        assert len(port.sent) == 1


async def test_revoke_ack_and_manual_outbox_creation_are_one_transaction(tmp_path: Path) -> None:
    async with Database(tmp_path / "task.db") as database:
        store, task = await setup_store(database)
        port = Port()
        service = LifecycleService(store, port, context=lambda: CONTEXT)
        await service.submit(SUBMIT, task, NOW)
        await service.reconcile("execution")
        state = await store.load("execution")
        await service.command(
            Command("takeover", "takeover", "execution"), expected_version=state.version
        )
        await database.connection.execute(
            "CREATE TRIGGER abort_manual BEFORE INSERT ON task_lifecycle_effect "
            "WHEN json_extract(NEW.payload,'$.kind')='manual_ready' "
            "BEGIN SELECT RAISE(ABORT, 'manual outbox rollback'); END"
        )
        await database.connection.commit()
        with pytest.raises(sqlite3.IntegrityError):
            await service.reconcile("execution")
        assert (await store.load("execution")).phase == "revoking"
        assert all(effect.kind != "manual_ready" for effect in port.sent)
        await database.connection.execute("DROP TRIGGER abort_manual")
        await database.connection.commit()
        await service.reconcile("execution")
        assert (await store.load("execution")).phase == "paused"
        await service.reconcile("execution")
        await service.reconcile("execution")
        assert [effect.kind for effect in port.sent] == ["plan", "revoke", "manual_ready"]
        assert [effect.kind for effect in port.queried] == ["revoke"]


async def test_unconsumed_manual_notice_survives_idempotent_takeover_and_rejected_resume(
    tmp_path: Path,
) -> None:
    async with Database(tmp_path / "task.db") as database:
        store, task = await setup_store(database)
        port = Port()
        current = CONTEXT
        service = LifecycleService(store, port, context=lambda: current)
        await service.submit(SUBMIT, task, NOW)
        await service.reconcile("execution")
        state = await store.load("execution")
        await service.command(
            Command("takeover", "takeover", "execution"), expected_version=state.version
        )
        await service.reconcile("execution")
        state = await store.load("execution")
        repeated = await service.command(
            Command("repeat-takeover", "takeover", "execution"), expected_version=state.version
        )
        current = replace(CONTEXT, connected=False)
        rejected = await service.command(
            Command("offline-resume", "resume", "execution"),
            expected_version=repeated.state.version,
        )
        assert rejected.receipt is not None and not rejected.receipt.accepted
        await service.reconcile("execution")
        assert [effect.kind for effect in port.sent] == ["plan", "revoke", "manual_ready"]


@pytest.mark.parametrize("action", ["pause", "cancel"])
async def test_explicit_stop_retires_unconsumed_manual_notice(
    tmp_path: Path, action: Action
) -> None:
    async with Database(tmp_path / "task.db") as database:
        store, task = await setup_store(database)
        port = Port()
        service = LifecycleService(store, port, context=lambda: CONTEXT)
        await service.submit(SUBMIT, task, NOW)
        await service.reconcile("execution")
        state = await store.load("execution")
        await service.command(
            Command("takeover", "takeover", "execution"), expected_version=state.version
        )
        await service.reconcile("execution")
        state = await store.load("execution")
        await service.command(Command("stop", action, "execution"), expected_version=state.version)
        await service.reconcile("execution")
        assert [effect.kind for effect in port.sent] == ["plan", "revoke"]


@pytest.mark.parametrize("changed", ["unavailable", "reference"])
@pytest.mark.parametrize("path", ["submit", "resume", "feedback_recovered", "admission_changed"])
async def test_all_plan_paths_sample_context_after_database_lock(
    tmp_path: Path,
    path: Literal["submit", "resume", "feedback_recovered", "admission_changed"],
    changed: str,
) -> None:
    async with Database(tmp_path / "task.db") as database:
        store, task = await setup_store(database)
        current = CONTEXT
        service = LifecycleService(store, Port(), context=lambda: current)
        await service.start()
        if path != "submit":
            await service.submit(SUBMIT, task, NOW)
            await service.reconcile("execution")
            state = await store.load("execution")
            if path == "resume":
                await service.command(
                    Command("pause", "pause", "execution"), expected_version=state.version
                )
            else:
                await service.event(
                    "execution", Event("feedback_lost"), expected_version=state.version
                )
            await service.reconcile("execution")
            if path == "admission_changed":
                current = replace(CONTEXT, motion_allowed=False)
                state = await store.load("execution")
                await service.event(
                    "execution", Event("feedback_recovered"), expected_version=state.version
                )
            state = await store.load("execution")
        current = CONTEXT
        await database.write_lock.acquire()
        if path == "submit":
            pending = asyncio.create_task(service.submit(SUBMIT, task, NOW))
        elif path == "resume":
            pending = asyncio.create_task(
                service.command(
                    Command("resume", "resume", "execution"), expected_version=state.version
                )
            )
        else:
            pending = asyncio.create_task(
                service.event("execution", Event(path), expected_version=state.version)
            )
        await asyncio.sleep(0)
        await asyncio.sleep(0)
        current = (
            replace(CONTEXT, motion_allowed=False, pose_ref=None)
            if changed == "unavailable"
            else replace(CONTEXT, pose_ref="new-pose", map_ref="new-map")
        )
        database.write_lock.release()
        result = await pending
        if changed == "unavailable":
            assert not result.effects
            assert result.state.phase in {"empty", "paused"}
            if result.receipt:
                assert not result.receipt.accepted
        else:
            assert result.effects[0].kind == "plan"
            assert result.effects[0].pose_ref == "new-pose"
            assert result.effects[0].map_ref == "new-map"


async def test_continuous_cas_conflicts_leave_observable_work_for_next_query(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    async with Database(tmp_path / "task.db") as database:
        store, task = await setup_store(database)
        port = Port()
        service = LifecycleService(store, port, context=lambda: CONTEXT)
        await service.submit(SUBMIT, task, NOW)
        original_load = store.load
        attempts = 0

        async def competing_receipt(execution_id: str) -> Lifecycle:
            nonlocal attempts
            state = await original_load(execution_id)
            attempts += 1
            if attempts > 3:
                raise AssertionError("one pass exceeded its acknowledgement budget")
            # A real committed receipt changes the version while preserving the
            # pending plan. Each ensuing ack must fail the actual SQLite CAS.
            await store.command(
                Command(f"racing-{attempts}", "resume", execution_id),
                lambda: CONTEXT,
                expected_version=state.version,
            )
            return state

        monkeypatch.setattr(store, "load", competing_receipt)
        result = await service.reconcile("execution")
        assert attempts == 3 and result.ack_attempts == 3 and result.needs_continue
        assert result.operations == 1
        monkeypatch.setattr(store, "load", original_load)
        assert (await store.effects("execution"))[0].status == "sending"
        settled = await service.reconcile("execution")
        assert not settled.needs_continue
        assert len(port.sent) == 1 and len(port.queried) == 1
        assert (await store.load("execution")).phase == "running"


async def test_coordinator_does_not_decode_historical_effects(tmp_path: Path) -> None:
    async with Database(tmp_path / "task.db") as database:
        store, task = await setup_store(database)
        port = Port()
        service = LifecycleService(store, port, context=lambda: CONTEXT)
        await service.submit(SUBMIT, task, NOW)
        # Legacy/corrupt audit payloads are intentionally undecodable. They must
        # never enter the coordinator's active-effect read or its decoder.
        await database.connection.executemany(
            "INSERT INTO task_lifecycle_effect VALUES (?, 'execution', ?, ?, NULL)",
            [(f"history-{i}", "audit-only", "done" if i % 2 else "retired") for i in range(5000)],
        )
        await database.connection.commit()
        result = await service.reconcile("execution")
        assert result.operations == 1 and not result.needs_continue
        assert (await store.load("execution")).phase == "running"


async def test_per_pass_external_budget_is_observable_and_unknown_effects_are_not_resent(
    tmp_path: Path,
) -> None:
    import json
    from dataclasses import asdict

    async with Database(tmp_path / "task.db") as database:
        store, task = await setup_store(database)
        port = Port()
        port.answer = None
        service = LifecycleService(store, port, context=lambda: CONTEXT, effect_limit=2)
        result = await service.submit(SUBMIT, task, NOW)
        effect = result.effects[0]
        await database.connection.executemany(
            "INSERT INTO task_lifecycle_effect VALUES (?,?,?,'sending',NULL)",
            [
                (
                    f"unknown-{i}",
                    "execution",
                    json.dumps(asdict(replace(effect, effect_id=f"unknown-{i}"))),
                )
                for i in range(6)
            ],
        )
        await database.connection.commit()
        first = await service.reconcile("execution")
        assert first.operations == 2 and first.ack_attempts == 0 and first.needs_continue
        assert len(port.sent) == 1 and len(port.queried) == 1
        second = await service.reconcile("execution")
        assert second.operations == 2 and second.needs_continue
        assert len(port.sent) == 1 and len(port.queried) == 3


async def test_provider_is_not_sampled_before_version_check(
    tmp_path: Path,
) -> None:
    from senior_control.tasks.lifecycle_store import VersionConflict

    async with Database(tmp_path / "task.db") as database:
        store, task = await setup_store(database)
        calls = 0

        def current() -> Context:
            nonlocal calls
            calls += 1
            assert database.write_lock.locked()
            return CONTEXT

        service = LifecycleService(store, Port(), context=current)
        await service.start()
        assert calls == 0
        accepted = await service.submit(SUBMIT, task, NOW)
        assert calls == 1
        with pytest.raises(VersionConflict):
            await service.command(
                Command("pause", "pause", "execution"), expected_version=accepted.state.version - 1
            )
        assert calls == 1
