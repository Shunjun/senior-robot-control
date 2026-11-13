import asyncio
from dataclasses import replace

import pytest

from senior_control.settings.models import (
    DrivingSettings,
    LoggingSettings,
    SettingsProfile,
    SettingsRequest,
)
from senior_control.settings.service import RuntimeGate, SettingsService
from senior_control.settings.store import SettingsStore
from senior_control.storage.database import Database


def profile():
    defaults = DrivingSettings(
        low_linear_mps=5 / 9,
        high_linear_mps=25 / 18,
        low_angular_rps=1,
        high_angular_rps=1,
        reverse_linear_mps=0.3,
    )
    return SettingsProfile(
        defaults,
        DrivingSettings(
            low_linear_mps=2,
            high_linear_mps=2,
            low_angular_rps=2,
            high_angular_rps=2,
            reverse_linear_mps=1,
        ),
    )


def request(id="r", version=0, group="driving", **changes):
    values = (
        profile().driving_defaults.model_dump()
        if group == "driving"
        else LoggingSettings().model_dump()
    )
    values.update(changes)
    return SettingsRequest(request_id=id, expected_version=version, group=group, values=values)


async def test_persistence_concurrency_idempotency_and_failure(tmp_path):
    path = tmp_path / "settings.db"
    async with Database(path) as db:
        service = SettingsService(SettingsStore(db), profile(), RuntimeGate())
        await service.runtime.update(stationary=True, executing_motion_task=False)
        await service.prepare()
        first, second = await asyncio.gather(service.apply(request()), service.apply(request("s")))
        assert {first.reason, second.reason} == {"applied", "version_conflict"}
        assert await service.apply(request()) == first
        assert (await service.apply(request(high_linear_mps=1))).reason == "request_conflict"
        assert await service.result("r") == first
        before = await service.get()
        await db.connection.execute(
            "CREATE TRIGGER fail_settings BEFORE UPDATE ON vehicle_settings "
            "BEGIN SELECT RAISE(ABORT, 'disk failure'); END"
        )
        await db.connection.commit()
        with pytest.raises(Exception, match="disk failure"):
            await service.apply(request("fail", 1, high_linear_mps=1.5))
        assert await service.get() == before
        assert await service.result("fail") is None
    async with Database(path) as db:
        service = SettingsService(SettingsStore(db), profile(), RuntimeGate())
        await service.runtime.update(stationary=True, executing_motion_task=False)
        await service.prepare()
        assert await service.get() == before
        assert await service.result("r") == first


@pytest.mark.parametrize("field", list(profile().driving_defaults.model_dump()))
@pytest.mark.parametrize("value", [float("nan"), float("inf"), -1, 99, True])
async def test_reject_all_invalid_driving_fields(tmp_path, field, value):
    async with Database(tmp_path / "test.db") as db:
        service = SettingsService(SettingsStore(db), profile(), RuntimeGate())
        await service.runtime.update(stationary=True, executing_motion_task=False)
        await service.prepare()
        assert (await service.apply(request(**{field: value}))).reason == "invalid_values"
        assert (await service.get()).driving_version == 0


async def test_runtime_gate_and_logging(tmp_path):
    async with Database(tmp_path / "test.db") as db:
        gate = RuntimeGate()
        service = SettingsService(SettingsStore(db), profile(), gate)
        await service.runtime.update(stationary=True, executing_motion_task=False)
        await service.prepare()
        await gate.update(stationary=True, executing_motion_task=True)
        assert (await service.apply(request())).reason == "stop_required"
        assert (await service.apply(request("log", group="logging"))).reason == "applied"
        await gate.update(stationary=False, executing_motion_task=False)
        assert (await service.apply(request("moving"))).reason == "stop_required"
        await gate.update(stationary=True, executing_motion_task=False)
        assert (await service.apply(request("paused"))).reason == "applied"
        assert (await service.get()).driving.reverse_limit(high=False) == 0.3
        assert (
            await service.apply(request("order", 1, low_linear_mps=1.6))
        ).reason == "invalid_values"
        assert (
            await service.apply(replace(request("defaults", 1), restore_defaults=True))
        ).reason == "applied"


async def test_corruption_is_unavailable_not_defaults(tmp_path):
    async with Database(tmp_path / "test.db") as db:
        service = SettingsService(SettingsStore(db), profile(), RuntimeGate())
        await service.runtime.update(stationary=True, executing_motion_task=False)
        await service.prepare()
        await db.connection.execute("UPDATE vehicle_settings SET payload = '{}' WHERE id = 1")
        await db.connection.commit()
        broken = SettingsService(SettingsStore(db), profile(), RuntimeGate())
        with pytest.raises(ValueError, match="unavailable"):
            await broken.prepare()
        with pytest.raises(ValueError, match="unavailable"):
            await broken.get()


async def test_debug_deadline_original_level_and_restart(tmp_path):
    path = tmp_path / "test.db"
    now = [1000.0]
    async with Database(path) as db:
        service = SettingsService(
            SettingsStore(db),
            profile(),
            RuntimeGate(),
            utc_now=lambda: now[0],
            steady_now=lambda: now[0],
        )
        await service.runtime.update(stationary=True, executing_motion_task=False)
        await service.prepare()
        await service.apply(request("debug", group="logging", debug_enabled=True, level="WARNING"))
        snapshot = await service.get()
        assert snapshot.debug_deadline_utc == 1300
        assert snapshot.effective_log_level == "DEBUG"
        await service.apply(request("again", 1, "logging", debug_enabled=True, level="ERROR"))
        assert (await service.get()).debug_deadline_utc == 1300
        assert (await service.get()).debug_original_level == "INFO"
    now[0] = 1400
    async with Database(path) as db:
        service = SettingsService(
            SettingsStore(db), profile(), RuntimeGate(), utc_now=lambda: now[0]
        )
        await service.runtime.update(stationary=True, executing_motion_task=False)
        await service.prepare()
        snapshot = await service.get()
        assert not snapshot.logging.debug_enabled
        assert snapshot.effective_log_level == "INFO"
        assert snapshot.logging_version == 3


async def test_commit_excludes_runtime_transition_and_settles_cancellation(tmp_path):
    from senior_control.settings.service import RuntimeCheck

    class PausingStore(SettingsStore):
        async def write_snapshot(self, connection, snapshot):
            entered.set()
            await release.wait()
            await super().write_snapshot(connection, snapshot)

    entered, release, started = asyncio.Event(), asyncio.Event(), asyncio.Event()
    async with Database(tmp_path / "test.db") as db:
        gate = RuntimeGate()
        await gate.update(stationary=True, executing_motion_task=False)
        service = SettingsService(PausingStore(db), profile(), gate)
        await service.prepare()
        applying = asyncio.create_task(service.apply(request()))
        await entered.wait()

        async def admit_motion():
            started.set()
            return RuntimeCheck(stationary=False, executing_motion_task=True)

        transitioning = asyncio.create_task(gate.transition(admit_motion))
        applying.cancel()
        await asyncio.sleep(0)
        assert not started.is_set()
        assert not applying.done()
        release.set()
        with pytest.raises(asyncio.CancelledError):
            await applying
        await transitioning
        assert (await service.get()).driving_version == 1
        assert (await service.result("r")).reason == "applied"
        assert (await service.apply(request("after", 1))).reason == "stop_required"


@pytest.mark.parametrize(
    "field,value",
    [
        ("level", "TRACE"),
        ("retention_seconds", 3599),
        ("retention_seconds", 2592001),
        ("debug_restore_seconds", 59),
        ("debug_restore_seconds", 3601),
        ("debug_enabled", 1),
        ("retention_seconds", float("nan")),
        ("disk_limit", 100),
    ],
)
async def test_logging_validation(tmp_path, field, value):
    async with Database(tmp_path / "test.db") as db:
        service = SettingsService(SettingsStore(db), profile(), RuntimeGate())
        await service.prepare()
        assert (
            await service.apply(request(group="logging", **{field: value}))
        ).reason == "invalid_values"
        assert (await service.get()).logging_version == 0


async def test_default_gate_is_closed_and_manual_must_release(tmp_path):
    async with Database(tmp_path / "test.db") as db:
        service = SettingsService(SettingsStore(db), profile(), RuntimeGate())
        await service.prepare()
        assert (await service.apply(request("unknown"))).reason == "stop_required"
        await service.runtime.update(
            stationary=True, executing_motion_task=False, manual_active=True
        )
        assert (await service.apply(request("manual"))).reason == "stop_required"


async def test_offline_debug_timer_and_untrusted_restart(tmp_path):
    path = tmp_path / "test.db"
    utc, steady = [1000.0], [5.0]
    restored = asyncio.Event()

    class ObservedStore(SettingsStore):
        async def write_snapshot(self, connection, snapshot):
            await super().write_snapshot(connection, snapshot)
            if not snapshot.logging.debug_enabled:
                restored.set()

    async with Database(path) as db:
        service = SettingsService(
            ObservedStore(db),
            profile(),
            RuntimeGate(),
            utc_now=lambda: utc[0],
            steady_now=lambda: steady[0],
        )
        await service.prepare()
        service.start_debug_restore()
        try:
            await service.apply(request("enable", group="logging", debug_enabled=True))
            utc[0], steady[0] = 900, 306
            # No browser get/apply request performs expiry: the service owner runs it.
            await asyncio.wait_for(restored.wait(), 2)
            async with db.connection.execute("SELECT payload FROM vehicle_settings") as cursor:
                row = await cursor.fetchone()
            assert '"debug_enabled":false' in row[0]
            await service.apply(request("again", 2, "logging", debug_enabled=True))
        finally:
            await service.close()
    async with Database(path) as db:
        service = SettingsService(
            SettingsStore(db),
            profile(),
            RuntimeGate(),
            utc_now=lambda: 901,
            utc_trusted=lambda: False,
        )
        await service.prepare()
        assert not (await service.get()).logging.debug_enabled


async def test_idempotent_debug_does_not_extend_and_restore_defaults_is_gated(tmp_path):
    now = [1000.0]
    async with Database(tmp_path / "test.db") as db:
        service = SettingsService(
            SettingsStore(db), profile(), RuntimeGate(), utc_now=lambda: now[0]
        )
        await service.prepare()
        enabling = request("enable", group="logging", debug_enabled=True)
        first = await service.apply(enabling)
        now[0] += 10
        assert await service.apply(enabling) == first
        assert (await service.get()).debug_deadline_utc == 1300
        assert (
            await service.apply(replace(request("restore"), restore_defaults=True))
        ).reason == "stop_required"


async def test_logging_does_not_wait_for_runtime_gate(tmp_path):
    async with Database(tmp_path / "test.db") as db:
        gate = RuntimeGate()
        service = SettingsService(SettingsStore(db), profile(), gate)
        await service.prepare()
        async with gate.lock:
            result = await asyncio.wait_for(service.apply(request(group="logging")), 1)
        assert result.reason == "applied"


async def test_missing_or_future_settings_schema_is_unavailable(tmp_path):
    async with Database(tmp_path / "test.db") as db:
        service = SettingsService(SettingsStore(db), profile(), RuntimeGate())
        await service.prepare()
        await db.connection.execute("UPDATE vehicle_settings SET schema_version = 99")
        await db.connection.commit()
        with pytest.raises(ValueError, match="unavailable"):
            await SettingsService(SettingsStore(db), profile(), RuntimeGate()).prepare()
        await db.connection.execute("DELETE FROM vehicle_settings")
        await db.connection.commit()
        with pytest.raises(ValueError, match="unavailable"):
            await SettingsService(SettingsStore(db), profile(), RuntimeGate()).prepare()


async def test_debug_restore_retries_storage_failure_without_false_publication(tmp_path):
    now = [1000.0]
    failed, restored = asyncio.Event(), asyncio.Event()

    class ObservedStore(SettingsStore):
        async def write_snapshot(self, connection, snapshot):
            try:
                await super().write_snapshot(connection, snapshot)
            except Exception:
                failed.set()
                raise
            if not snapshot.logging.debug_enabled:
                restored.set()

    async with Database(tmp_path / "test.db") as db:
        service = SettingsService(
            ObservedStore(db), profile(), RuntimeGate(), utc_now=lambda: now[0]
        )
        await service.prepare()
        await service.apply(request(group="logging", debug_enabled=True))
        await db.connection.execute(
            "CREATE TRIGGER fail_restore BEFORE UPDATE ON vehicle_settings "
            "BEGIN SELECT RAISE(ABORT, 'disk failure'); END"
        )
        await db.connection.commit()
        now[0] = 1400
        service.start_debug_restore()
        try:
            await asyncio.wait_for(failed.wait(), 2)
            with pytest.raises(Exception, match="disk failure"):
                await service.get()
            await db.connection.execute("DROP TRIGGER fail_restore")
            await db.connection.commit()
            await asyncio.wait_for(restored.wait(), 2)
            assert (await service.get()).effective_log_level == "INFO"
        finally:
            await service.close()


async def test_partial_durable_logging_never_fills_model_defaults(tmp_path):
    import json

    async with Database(tmp_path / "test.db") as db:
        service = SettingsService(SettingsStore(db), profile(), RuntimeGate())
        await service.prepare()
        payload = (await service.get()).model_dump()
        del payload["logging"]["retention_seconds"]
        await db.connection.execute(
            "UPDATE vehicle_settings SET payload = ?", (json.dumps(payload),)
        )
        await db.connection.commit()
        with pytest.raises(ValueError, match="unavailable"):
            await SettingsService(SettingsStore(db), profile(), RuntimeGate()).prepare()


async def test_partial_logging_request_is_rejected(tmp_path):
    async with Database(tmp_path / "test.db") as db:
        service = SettingsService(SettingsStore(db), profile(), RuntimeGate())
        await service.prepare()
        partial = SettingsRequest("partial", 0, "logging", {"level": "WARNING"})
        assert (await service.apply(partial)).reason == "invalid_values"


async def test_cancellation_after_physical_commit_still_publishes(tmp_path, monkeypatch):
    committed, release = asyncio.Event(), asyncio.Event()
    async with Database(tmp_path / "test.db") as db:
        service = SettingsService(SettingsStore(db), profile(), RuntimeGate())
        await service.runtime.update(stationary=True, executing_motion_task=False)
        await service.prepare()
        original_commit = db.connection.commit

        async def paused_commit():
            await original_commit()
            committed.set()
            await release.wait()

        monkeypatch.setattr(db.connection, "commit", paused_commit)
        task = asyncio.create_task(service.apply(request()))
        await committed.wait()
        task.cancel()
        await asyncio.sleep(0)
        assert not task.done()
        release.set()
        with pytest.raises(asyncio.CancelledError):
            await task
        assert (await service.get()).driving_version == 1
        assert (await service.result("r")).snapshot.driving_version == 1


async def test_second_writer_cannot_overwrite_published_owner(tmp_path):
    async with Database(tmp_path / "test.db") as db:
        first = SettingsService(SettingsStore(db), profile(), RuntimeGate())
        second = SettingsService(SettingsStore(db), profile(), RuntimeGate())
        await first.prepare()
        await second.prepare()
        await first.apply(request(group="logging", level="WARNING"))
        with pytest.raises(ValueError, match="another settings writer"):
            await second.apply(request("second", group="logging", level="ERROR"))
        assert (await first.get()).logging.level == "WARNING"


async def test_injected_runtime_observation_is_evaluated_under_guard_at_submit(tmp_path):
    from senior_control.settings.service import RuntimeCheck

    evidence = [RuntimeCheck(stationary=True, executing_motion_task=False)]
    gate = RuntimeGate(observe=lambda: evidence[0])
    async with Database(tmp_path / "test.db") as db:
        service = SettingsService(SettingsStore(db), profile(), gate)
        await service.prepare()
        evidence[0] = RuntimeCheck()  # e.g. previous stationary evidence expired
        assert (await service.apply(request())).reason == "stop_required"
        evidence[0] = RuntimeCheck(stationary=True, executing_motion_task=False)
        assert (await service.apply(request("fresh"))).reason == "applied"


async def test_published_image_is_immutable_and_io_free_during_pending_commit(tmp_path):
    entered, release = asyncio.Event(), asyncio.Event()

    class PausingStore(SettingsStore):
        async def write_snapshot(self, connection, snapshot):
            entered.set()
            await release.wait()
            await super().write_snapshot(connection, snapshot)

    async with Database(tmp_path / "test.db") as db:
        service = SettingsService(PausingStore(db), profile(), RuntimeGate())
        await service.prepare()
        previous = service.published_snapshot
        task = asyncio.create_task(service.apply(request(group="logging", level="WARNING")))
        await entered.wait()
        assert service.published_snapshot is previous
        with pytest.raises(ValueError):
            previous.logging.level = "ERROR"
        release.set()
        await task
        assert service.published_snapshot.logging.level == "WARNING"
        assert previous.logging.level == "INFO"
