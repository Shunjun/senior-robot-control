"""SQLite terminal obligations survive uncertainty; only their own ACK can settle."""

from pathlib import Path

from test_task_lifecycle_store import CONTEXT, NOW, SUBMIT, setup_store

from senior_control.storage.database import Database
from senior_control.tasks.lifecycle import Command
from senior_control.tasks.lifecycle_service import LifecycleService


async def test_identity_obligation_is_committed_and_false_ack_remains_queryable(tmp_path: Path):
    path = tmp_path / "tasks.db"
    async with Database(path) as db:
        store, definition = await setup_store(db)

        class Port:
            answer = None

            def __init__(self):
                self.sent = []
                self.queried = []

            async def execute(self, effect):
                assert not db.write_lock.locked()
                assert (await store.load("execution")).pending == effect
                self.sent.append(effect)
                return self.answer if effect.kind == "terminal_release" else True

            async def query(self, effect):
                self.queried.append(effect)
                return self.answer

        port = Port()
        service = LifecycleService(store, port, context=lambda: CONTEXT)
        await service.submit(SUBMIT, definition, NOW)
        await service.reconcile("execution")
        current = await store.load("execution")
        await service.command(
            Command("cancel", "cancel", "execution"), expected_version=current.version
        )
        await service.reconcile("execution")
        current = await store.load("execution")
        assert current.phase == "revoking" and current.pending.kind == "terminal_release"
        effect = current.pending
        assert effect.terminal_outcome == "canceled"
        await service.reconcile("execution")
        port.answer = False
        await service.reconcile("execution")
        assert (await store.load("execution")).pending == effect
        active = await store.active_effects("execution")
        assert len(active) == 1 and active[0].status == "sending"
        assert [e.kind for e in port.sent] == ["plan", "revoke", "terminal_release"]
    # Same-boot transport reconnect queries the same committed effect, never sends a fresh one.
    async with Database(path) as db:
        store, _ = await setup_store(db)

        class Query:
            async def execute(self, _):
                raise AssertionError("duplicate physical operation")

            async def query(self, incoming):
                assert incoming == effect
                return True

        service = LifecycleService(store, Query(), context=lambda: CONTEXT)
        await service.reconcile("execution")
        current = await store.load("execution")
        assert current.phase == "canceled" and current.pending is None
        before = current
        await service.acknowledge(effect, True, expected_version=current.version)
        assert await store.load("execution") == before


async def test_process_boot_change_retires_old_identity_obligation_and_reacquires_output(
    tmp_path: Path,
):
    path = tmp_path / "tasks.db"
    async with Database(path) as db:
        store, definition = await setup_store(db)

        class Port:
            async def execute(self, effect):
                return None if effect.kind == "terminal_release" else True

            async def query(self, _):
                return None

        service = LifecycleService(store, Port(), context=lambda: CONTEXT)
        await service.submit(SUBMIT, definition, NOW)
        await service.reconcile("execution")
        current = await store.load("execution")
        await service.command(
            Command("cancel", "cancel", "execution"), expected_version=current.version
        )
        await service.reconcile("execution")
        old = (await store.load("execution")).pending
        assert old.kind == "terminal_release"
    async with Database(path) as db:
        store, _ = await setup_store(db, "new-boot")
        await store.recover(lambda: CONTEXT)
        current = await store.load("execution")
        assert current.phase == "revoking" and current.stopped_for == "cancel"
        assert current.epoch > old.epoch and current.pending.kind == "revoke"
        assert any(
            x.effect == old and x.status == "retired" for x in await store.effects("execution")
        )


async def test_settlement_commits_independent_collection_obligation(tmp_path):
    async with Database(tmp_path / "collect.db") as db:
        store, definition = await setup_store(db)
        service = LifecycleService(store, None, context=lambda: CONTEXT)
        await service.submit(SUBMIT, definition, NOW)
        state = await store.load("execution")
        await store.claim_effect("execution", state.pending.effect_id)
        await service.acknowledge(state.pending, True, expected_version=state.version)
        state = await store.load("execution")
        await service.command(
            Command("cancel", "cancel", "execution"), expected_version=state.version
        )
        for _ in range(2):
            state = await store.load("execution")
            await store.claim_effect("execution", state.pending.effect_id)
            await service.acknowledge(state.pending, True, expected_version=state.version)
        state = await store.load("execution")
        assert state.phase == "canceled" and state.pending is None
        active = await store.active_effects("execution")
        assert len(active) == 1 and active[0].effect.kind == "terminal_settled"
        assert active[0].status == "ready"


async def test_persisted_settlement_answers_queries_and_collection_never_skips_unresolved(tmp_path):
    from senior_control.tasks.navigation_port import NavigationEffects

    async with Database(tmp_path / "frontier.db") as db:
        store, definition = await setup_store(db)
        service = LifecycleService(store, None, context=lambda: CONTEXT)
        await service.submit(SUBMIT, definition, NOW)
        state = await store.load("execution")
        original = state.pending
        wire, done = await store.wire_effect(original)
        assert wire.dispatch_sequence > 0 and wire.settled_through < wire.dispatch_sequence
        assert not done
        await store.claim_effect("execution", original.effect_id)
        await service.acknowledge(original, True, expected_version=state.version)

        class NoTransport:
            async def task_effect_status(self, _):
                raise AssertionError("durably answered query must not reach ROS")

        effects = NavigationEffects(store, NoTransport())
        assert await effects.query(original) is True
        state = await store.load("execution")
        await service.command(
            Command("cancel", "cancel", "execution"), expected_version=state.version
        )
        for _ in range(2):
            state = await store.load("execution")
            await store.claim_effect("execution", state.pending.effect_id)
            await service.acknowledge(state.pending, True, expected_version=state.version)
        collection = (await store.active_effects("execution"))[0].effect
        wire, done = await store.wire_effect(collection)
        assert wire.settled_through < wire.dispatch_sequence and not done
        await store.claim_effect("execution", collection.effect_id)
        current = await store.load("execution")
        await service.acknowledge(collection, False, expected_version=current.version)
        assert (await store.active_effects("execution"))[0].status == "sending"
        assert (await store.load("execution")).phase == "canceled"
