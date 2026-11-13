import asyncio
import json
from dataclasses import replace

import pytest

from senior_control.settings.models import DrivingSettings, LoggingSettings, SettingsSnapshot
from senior_control.vehicle_logs.models import LogInput, LogQuery
from senior_control.vehicle_logs.service import VehicleLogService
from senior_control.vehicle_logs.store import VehicleLogStore


class Authorizer:
    async def require_vehicle(self, credential, vehicle_id, action):
        if credential != "valid-session" or vehicle_id != "robot":
            raise PermissionError("vehicle access denied")


def settings():
    return SettingsSnapshot(
        driving=DrivingSettings(
            low_linear_mps=0.5,
            high_linear_mps=1,
            low_angular_rps=1,
            high_angular_rps=1,
            reverse_linear_mps=0.3,
        ),
        logging=LoggingSettings(),
    )


def event(number=0, **changes):
    return replace(
        LogInput(
            level="INFO",
            module="navigation",
            summary=f"event {number}",
            detail="details",
            source_utc=1000,
        ),
        **changes,
    )


async def test_cross_vehicle_log_query_denied(tmp_path):
    async with VehicleLogStore(tmp_path / "logs") as store:
        service = VehicleLogService(store, "robot", Authorizer(), settings)
        await service.start()
        try:
            for vehicle, token in [
                ("other", "valid-session"),
                ("robot", "forged"),
                ("robot", "authorized=True"),
            ]:
                with pytest.raises(PermissionError):
                    await service.query(token, vehicle, LogQuery())
                with pytest.raises(PermissionError):
                    await service.download(token, vehicle, LogQuery())
                with pytest.raises(PermissionError):
                    await service.follow(token, vehicle, LogQuery())
        finally:
            await service.close()


async def test_redaction_filter_stable_cursor_and_download(tmp_path):
    async with VehicleLogStore(tmp_path / "logs") as store:
        service = VehicleLogService(store, "robot", Authorizer(), settings)
        await service.start()
        try:
            assert service.enqueue(
                event(
                    1,
                    detail='token=abc password="two words" Bearer xyz pairing_key:pair123',
                    fields={"nested": {"device_credential": "hidden"}, "safe": "api_key=qwerty"},
                )
            )
            assert service.enqueue(event(2, level="ERROR", module="motion"))
            await service.flush()
            page = await service.query("valid-session", "robot", LogQuery(limit=1))
            wire = page.model_dump_json()
            for secret in ["abc", "two words", "xyz", "pair123", "hidden", "qwerty"]:
                assert secret not in wire
            assert "[REDACTED]" in wire
            assert page.has_more
            second = await service.query("valid-session", "robot", LogQuery(after=page.next_cursor))
            assert second.entries[0].summary == "event 2"
            assert not second.has_more
            filtered = await service.query("valid-session", "robot", LogQuery(levels=("ERROR",)))
            assert [item.summary for item in filtered.entries if item.kind == "event"] == [
                "event 2"
            ]
            download = await service.download("valid-session", "robot", LogQuery())
            assert b"pair123" not in download.data and b"[REDACTED]" in download.data
            assert len(download.data) <= 2 * 1024 * 1024
        finally:
            await service.close()


async def test_log_flood_does_not_delay_estop(tmp_path):
    entered, release, control = asyncio.Event(), asyncio.Event(), asyncio.Event()

    class SlowStore(VehicleLogStore):
        async def append(self, entries):
            entered.set()
            await release.wait()
            return await super().append(entries)

    async with SlowStore(tmp_path / "logs") as store:
        service = VehicleLogService(store, "robot", Authorizer(), settings)
        await service.start()
        try:
            service.enqueue(event())
            await entered.wait()
            # Independent fake control event advances while disk IO is held indefinitely.
            asyncio.get_running_loop().call_soon(control.set)
            accepted = sum(service.enqueue(event(i)) for i in range(5000))
            assert accepted <= 1024
            assert service.queue_depth <= 1024
            await asyncio.wait_for(control.wait(), 0.2)
            assert not release.is_set()
            release.set()
            await service.flush()
            page = await service.query("valid-session", "robot", LogQuery())
            assert any(
                item.kind == "gap" and item.loss_reason == "queue_full" for item in page.entries
            )
        finally:
            release.set()
            await service.close()


async def test_cursor_gap_after_retention_and_capacity(tmp_path):
    now = [1000.0]
    async with VehicleLogStore(tmp_path / "logs", disk_limit_bytes=65536) as store:
        service = VehicleLogService(store, "robot", Authorizer(), settings, utc_now=lambda: now[0])
        await service.start()
        try:
            for i in range(20):
                service.enqueue(event(i, detail="界" * 1000))
            await service.flush()
            page = await service.query("valid-session", "robot", LogQuery())
            assert any(gap.reason == "evicted" for gap in page.gaps)
            assert sum(p.stat().st_size for p in (tmp_path / "logs").iterdir()) <= 65536
            now[0] += 604801
            await service.cleanup()
            page = await service.query("valid-session", "robot", LogQuery())
            assert not page.entries and page.gaps
            assert not page.has_more
        finally:
            await service.close()


async def test_utf8_oversize_storage_failure_and_follow_overflow(tmp_path):
    failed = [True]

    class FailingStore(VehicleLogStore):
        async def append(self, entries):
            if failed[0]:
                raise OSError("disk full token=must-not-leak")
            return await super().append(entries)

    async with FailingStore(tmp_path / "logs") as store:
        service = VehicleLogService(store, "robot", Authorizer(), settings)
        await service.start()
        try:
            assert not service.enqueue(event(detail="界" * 1366))
            service.enqueue(event())
            await asyncio.sleep(0.1)
            pending = await service.query("valid-session", "robot", LogQuery())
            assert pending.gaps
            assert "must-not-leak" not in pending.model_dump_json()
            failed[0] = False
            await service.flush()
            follower = await service.follow(
                "valid-session", "robot", LogQuery(after=pending.next_cursor)
            )
            for i in range(10):
                service.enqueue(event(i))
                await service.flush()
            assert follower.outbox_depth <= 4
            pages = [await follower.next_page() for _ in range(follower.outbox_depth)]
            assert any(gap.reason == "slow_subscriber" for page in pages for gap in page.gaps)
            await follower.close()
        finally:
            await service.close()


async def test_header_redaction_and_structured_json_are_valid(tmp_path):
    async with VehicleLogStore(tmp_path / "logs") as store:
        service = VehicleLogService(store, "robot", Authorizer(), settings)
        await service.start()
        try:
            service.enqueue(
                event(
                    detail=(
                        "Authorization: Bearer header-secret\n"
                        "Cookie: session=cookie-secret; other=also-secret"
                    ),
                    fields={"nested": {"pairing_key": "pair-secret"}},
                )
            )
            await service.flush()
            page = await service.query("valid-session", "robot", LogQuery())
            wire = page.model_dump_json()
            for secret in ["header-secret", "cookie-secret", "also-secret", "pair-secret"]:
                assert secret not in wire
            assert json.loads(page.entries[0].fields_json)["nested"]["pairing_key"] == "[REDACTED]"
        finally:
            await service.close()


async def test_repeated_follow_overflow_preserves_earliest_missing_cursor(tmp_path):
    async with VehicleLogStore(tmp_path / "logs") as store:
        service = VehicleLogService(store, "robot", Authorizer(), settings)
        await service.start()
        try:
            subscription = await service.follow("valid-session", "robot", LogQuery())
            for i in range(13):
                service.enqueue(event(i))
                await service.flush()
            page = await subscription.next_page()
            gaps = [gap for gap in page.gaps if gap.reason == "slow_subscriber"]
            assert gaps and gaps[0].after == 0 and gaps[0].through >= 8
            await subscription.close()
        finally:
            await service.close()


async def test_page_and_download_byte_limits_have_lossless_continuations(tmp_path):
    async with VehicleLogStore(tmp_path / "logs") as store:
        service = VehicleLogService(store, "robot", Authorizer(), settings)
        await service.start()
        try:
            for i in range(180):
                assert service.enqueue(event(i, detail="\x01" * 4096))
            await service.flush()
            cursor, seen = 0, []
            while True:
                page = await service.query("valid-session", "robot", LogQuery(after=cursor))
                assert len(page.model_dump_json().encode()) <= 256 * 1024
                assert len(page.entries) <= 200 and page.scanned <= 200
                seen.extend(entry.sequence for entry in page.entries)
                cursor = page.next_cursor
                if not page.has_more:
                    break
            assert seen == list(range(1, 181))
            cursor, downloaded = 0, []
            first = True
            while True:
                result = await service.download("valid-session", "robot", LogQuery(after=cursor))
                assert len(result.data) <= 2 * 1024 * 1024
                if first:
                    assert result.has_more
                    first = False
                downloaded.extend(json.loads(line)["sequence"] for line in result.data.splitlines())
                cursor = result.next_cursor
                if not result.has_more:
                    break
            assert downloaded == list(range(1, 181))
        finally:
            await service.close()


async def test_sparse_filters_advance_bounded_scan_and_reject_paths(tmp_path):
    from pydantic import ValidationError

    with pytest.raises(ValidationError):
        LogQuery(path="/etc/passwd")
    with pytest.raises(ValidationError):
        LogQuery(limit=201)
    async with VehicleLogStore(tmp_path / "logs") as store:
        service = VehicleLogService(store, "robot", Authorizer(), settings)
        await service.start()
        try:
            for i in range(250):
                service.enqueue(event(i))
            await service.flush()
            page = await service.query("valid-session", "robot", LogQuery(module="not-present"))
            assert not page.entries and page.next_cursor == 200 and page.has_more
            last = await service.query(
                "valid-session", "robot", LogQuery(after=200, module="not-present")
            )
            assert not last.entries and last.next_cursor == 250 and not last.has_more
        finally:
            await service.close()


async def test_query_cancellation_releases_quota_and_follow_revocation(tmp_path):
    entered, release = asyncio.Event(), asyncio.Event()
    blocked = [False]

    class SlowQueryStore(VehicleLogStore):
        async def query(self, query):
            if blocked[0]:
                entered.set()
                await release.wait()
            return await super().query(query)

    class RevocableAuthorizer(Authorizer):
        revoked = False

        async def require_vehicle(self, credential, vehicle_id, action):
            if self.revoked:
                raise PermissionError("revoked")
            await super().require_vehicle(credential, vehicle_id, action)

    async with SlowQueryStore(tmp_path / "logs") as store:
        authorizer = RevocableAuthorizer()
        service = VehicleLogService(store, "robot", authorizer, settings)
        await service.start()
        try:
            subscription = await service.follow("valid-session", "robot", LogQuery())
            blocked[0] = True
            first = asyncio.create_task(service.query("valid-session", "robot", LogQuery()))
            await entered.wait()
            second = asyncio.create_task(service.query("valid-session", "robot", LogQuery()))
            await asyncio.sleep(0)
            with pytest.raises(RuntimeError, match="quota"):
                await service.query("valid-session", "robot", LogQuery())
            first.cancel()
            with pytest.raises(asyncio.CancelledError):
                await first
            release.set()
            await second
            await service.query("valid-session", "robot", LogQuery())
            authorizer.revoked = True
            with pytest.raises(PermissionError):
                await subscription.next_page()
            assert subscription.outbox_depth == 1  # bounded close notification
        finally:
            release.set()
            await service.close()


async def test_utf8_boundary_threaded_producer_and_restart_cursor(tmp_path):
    path = tmp_path / "logs"
    async with VehicleLogStore(path) as store:
        service = VehicleLogService(store, "robot", Authorizer(), settings)
        await service.start()
        try:
            accepted = await asyncio.to_thread(service.enqueue, event(detail="界" * 1365 + "x"))
            assert accepted
            assert not service.enqueue(event(detail="界" * 1365 + "xx"))
            await service.flush()
            page = await service.query("valid-session", "robot", LogQuery())
            cursor = page.next_cursor
            assert any(entry.summary == "event 0" for entry in page.entries)
            assert any(entry.loss_reason == "oversize" for entry in page.entries)
        finally:
            await service.close()
    async with VehicleLogStore(path) as store:
        service = VehicleLogService(store, "robot", Authorizer(), settings)
        await service.start()
        try:
            service.enqueue(event(99))
            await service.flush()
            page = await service.query("valid-session", "robot", LogQuery(after=cursor))
            assert any(entry.loss_reason == "restart_unknown" for entry in page.entries)
            assert any(entry.summary == "event 99" for entry in page.entries)
            assert all(entry.sequence > cursor for entry in page.entries)
        finally:
            await service.close()


async def test_export_rechecks_permission_after_io(tmp_path):
    class RevocableAuthorizer(Authorizer):
        revoked = False

        async def require_vehicle(self, credential, vehicle_id, action):
            if self.revoked:
                raise PermissionError("revoked during IO")
            await super().require_vehicle(credential, vehicle_id, action)

    authorizer = RevocableAuthorizer()
    revoke = [False]

    class RevokingStore(VehicleLogStore):
        async def query(self, query):
            page = await super().query(query)
            if revoke[0]:
                authorizer.revoked = True
            return page

    async with RevokingStore(tmp_path / "logs") as store:
        service = VehicleLogService(store, "robot", authorizer, settings)
        await service.start()
        try:
            service.enqueue(event())
            await service.flush()
            revoke[0] = True
            with pytest.raises(PermissionError):
                await service.download("valid-session", "robot", LogQuery())
        finally:
            await service.close()


async def test_follow_replay_and_live_append_have_no_duplicate_or_hidden_backfill(tmp_path):
    entered, release = asyncio.Event(), asyncio.Event()
    block = [False]

    class SlowReadStore(VehicleLogStore):
        async def query(self, query):
            page = await super().query(query)
            if block[0]:
                entered.set()
                await release.wait()
            return page

    async with SlowReadStore(tmp_path / "logs") as store:
        service = VehicleLogService(store, "robot", Authorizer(), settings)
        await service.start()
        try:
            service.enqueue(event(1))
            await service.flush()
            block[0] = True
            following = asyncio.create_task(service.follow("valid-session", "robot", LogQuery()))
            await entered.wait()
            service.enqueue(event(2))
            block[0] = False
            release.set()
            subscription = await following
            await service.flush()
            pages = [await subscription.next_page() for _ in range(subscription.outbox_depth)]
            assert [entry.summary for page in pages for entry in page.entries] == [
                "event 1",
                "event 2",
            ]
            await subscription.close()
            replay = await service.follow("valid-session", "robot", LogQuery(limit=1))
            first = await replay.next_page()
            assert first.has_more
            assert any(
                gap.reason == "follow_backfill" and gap.after == 1 and gap.through == 2
                for gap in first.gaps
            )
            await replay.close()
        finally:
            release.set()
            await service.close()


async def test_real_sqlite_full_records_loss_and_recovers(tmp_path):
    async with VehicleLogStore(tmp_path / "logs") as store:
        service = VehicleLogService(store, "robot", Authorizer(), settings)
        await service.start()
        try:
            async with store._connection.execute("PRAGMA page_count") as cursor:
                count = (await cursor.fetchone())[0]
            await store._connection.execute(f"PRAGMA max_page_count={count}")
            assert service.enqueue(event(detail="x" * 4096))
            await service.flush()
            page = await service.query("valid-session", "robot", LogQuery())
            assert any(
                entry.loss_reason == "storage_error" and entry.loss_count == 1
                for entry in page.entries
            )
            assert not any(entry.kind == "event" for entry in page.entries)
            await store._connection.execute("PRAGMA max_page_count=16384")
            assert service.enqueue(event(7))
            await service.flush()
            page = await service.query("valid-session", "robot", LogQuery())
            assert any(entry.summary == "event 7" for entry in page.entries)
        finally:
            await service.close()


async def test_download_scan_budget_is_explicit_for_empty_filtered_result(tmp_path):
    async with VehicleLogStore(tmp_path / "logs") as store:
        service = VehicleLogService(store, "robot", Authorizer(), settings)
        await service.start()
        try:
            for base in range(0, 3500, 500):
                for i in range(base, base + 500):
                    service.enqueue(event(i))
                await service.flush()
            result = await service.download("valid-session", "robot", LogQuery(module="absent"))
            assert result.data == b"" and result.has_more and result.next_cursor == 3200
            last = await service.download(
                "valid-session", "robot", LogQuery(after=3200, module="absent")
            )
            assert last.data == b"" and not last.has_more and last.next_cursor == 3500
        finally:
            await service.close()


async def test_uses_settings_published_policy_and_no_second_debug_owner(tmp_path):
    from senior_control.settings.models import SettingsProfile, SettingsRequest
    from senior_control.settings.service import RuntimeGate, SettingsService
    from senior_control.settings.store import SettingsStore
    from senior_control.storage.database import Database

    now = [1000.0]
    async with (
        Database(tmp_path / "settings.db") as database,
        VehicleLogStore(tmp_path / "logs") as store,
    ):
        profile = SettingsProfile(settings().driving, settings().driving)
        configuration = SettingsService(
            SettingsStore(database), profile, RuntimeGate(), utc_now=lambda: now[0]
        )
        await configuration.prepare()
        service = VehicleLogService(
            store, "robot", Authorizer(), lambda: configuration.published_snapshot
        )
        await service.start()
        try:
            assert not service.enqueue(event(1, level="DEBUG"))
            await configuration.apply(
                SettingsRequest(
                    "enable",
                    0,
                    "logging",
                    {
                        "level": "INFO",
                        "retention_seconds": 3600,
                        "debug_enabled": True,
                        "debug_restore_seconds": 60,
                    },
                )
            )
            assert service.enqueue(event(2, level="DEBUG"))
            now[0] = 1061
            await configuration.get()  # expiry is exclusively SettingsService-owned
            assert not service.enqueue(event(3, level="DEBUG"))
            # Neither the settings DB write lock nor the runtime gate is a log IO lock.
            async with database.write_lock, configuration.runtime.lock:
                await asyncio.wait_for(service.flush(), 1)
                page = await asyncio.wait_for(
                    service.query("valid-session", "robot", LogQuery()),
                    1,
                )
            assert [entry.summary for entry in page.entries] == ["event 2"]
        finally:
            await service.close()


async def test_redaction_expansion_and_metadata_cannot_exceed_limits_or_leak(tmp_path):
    async with VehicleLogStore(tmp_path / "logs") as store:
        service = VehicleLogService(store, "robot", Authorizer(), settings)
        await service.start()
        try:
            assert not service.enqueue(event(detail="token=a " * 512))
            assert not service.enqueue(event(request_id="token:metadata-secret"))
            await service.flush()
            page = await service.query("valid-session", "robot", LogQuery())
            assert "metadata-secret" not in page.model_dump_json()
            assert all(len(entry.detail.encode()) <= 4096 for entry in page.entries)
        finally:
            await service.close()


async def test_live_page_does_not_claim_high_water_is_complete_before_final_entry(tmp_path):
    async with VehicleLogStore(tmp_path / "logs") as store:
        service = VehicleLogService(store, "robot", Authorizer(), settings)
        await service.start()
        try:
            subscription = await service.follow("valid-session", "robot", LogQuery(limit=2))
            for i in range(3):
                service.enqueue(event(i))
            await service.flush()
            first = await subscription.next_page()
            assert first.next_cursor < first.high_water
            assert first.has_more
            assert [entry.sequence for entry in first.entries] == [1, 2]
            last = await subscription.next_page()
            assert [entry.sequence for entry in last.entries] == [3]
            assert last.next_cursor == last.high_water == 3 and not last.has_more
            await subscription.close()
        finally:
            await service.close()


async def test_retention_failure_remains_visible_after_writer_recovers(tmp_path):
    fail = [True]

    class BrokenPruneStore(VehicleLogStore):
        async def prune(self, cutoff_utc):
            if fail[0]:
                raise OSError("retention disk failure")
            await super().prune(cutoff_utc)

    async with BrokenPruneStore(tmp_path / "logs") as store:
        service = VehicleLogService(store, "robot", Authorizer(), settings)
        await service.start()
        try:
            with pytest.raises(OSError):
                await service.cleanup()
            service.enqueue(event())
            await service.flush()
            page = await service.query("valid-session", "robot", LogQuery())
            assert any(gap.reason == "log_cleanup_unavailable" for gap in page.gaps)
            fail[0] = False
            await service.cleanup()
            page = await service.query("valid-session", "robot", LogQuery())
            assert not any(gap.reason == "log_cleanup_unavailable" for gap in page.gaps)
        finally:
            await service.close()


async def test_short_live_batch_is_one_page_without_false_slow_consumer_gap(tmp_path):
    async with VehicleLogStore(tmp_path / "logs") as store:
        service = VehicleLogService(store, "robot", Authorizer(), settings)
        await service.start()
        try:
            subscription = await service.follow("valid-session", "robot", LogQuery())
            waiting = asyncio.create_task(subscription.next_page())
            for i in range(64):
                service.enqueue(event(i))
            await service.flush()
            page = await waiting
            assert [item.sequence for item in page.entries] == list(range(1, 65))
            assert not page.gaps and not page.has_more
            assert page.next_cursor == page.high_water == 64
            assert subscription.outbox_depth == 0
            await subscription.close()
        finally:
            await service.close()


@pytest.mark.parametrize(
    "damage",
    [
        "DELETE FROM log_meta",
        "DROP TABLE log_meta",
        "DROP TABLE log_entry",
        "UPDATE log_meta SET version=2",
        "UPDATE log_meta SET high_water=0",
        "UPDATE log_meta SET high_water=65",
        "UPDATE log_meta SET floor=3",
        "UPDATE log_meta SET floor=-1",
        "UPDATE log_meta SET payload_bytes=-1",
        "UPDATE log_meta SET payload_bytes=1",
        "DELETE FROM log_entry WHERE sequence=32",
    ],
)
async def test_existing_damaged_store_is_unavailable_without_recreation(tmp_path, damage):
    import sqlite3

    from senior_control.vehicle_logs.models import prepare_entry

    directory = tmp_path / "logs"
    async with VehicleLogStore(directory) as store:
        await store.append(tuple(prepare_entry(event(i), 1000.0) for i in range(64)))
    path = directory / "vehicle-logs.sqlite3"
    with sqlite3.connect(path) as connection:
        connection.execute(damage)
    before = path.read_bytes()
    with pytest.raises(ValueError, match="unavailable"):
        async with VehicleLogStore(directory):
            pass
    assert path.read_bytes() == before


@pytest.mark.parametrize("detail,limit", [("界" * 1365, 200), ("\x01" * 4096, 200), ("short", 7)])
async def test_fast_waiting_follower_receives_necessary_pages_losslessly(tmp_path, detail, limit):
    async with VehicleLogStore(tmp_path / "logs") as store:
        service = VehicleLogService(store, "robot", Authorizer(), settings)
        await service.start()
        try:
            subscription = await service.follow("valid-session", "robot", LogQuery(limit=limit))

            async def receive():
                pages = []
                while not pages or pages[-1].next_cursor < 64:
                    pages.append(await subscription.next_page())
                return pages

            receiving = asyncio.create_task(receive())
            await asyncio.sleep(0)
            for i in range(64):
                service.enqueue(event(i, detail=detail))
            await service.flush()
            pages = await asyncio.wait_for(receiving, 2)
            assert len(pages) > 1
            assert [entry.sequence for page in pages for entry in page.entries] == list(
                range(1, 65)
            )
            for page in pages:
                assert not page.gaps
                assert len(page.entries) <= limit
                assert len(page.model_dump_json().encode()) <= 256 * 1024
                assert page.high_water == 64
                assert page.has_more == (page.next_cursor < 64)
            assert subscription.outbox_depth == 0
            await subscription.close()
        finally:
            await service.close()


async def test_filtered_batch_advances_cursor_over_trailing_filtered_rows(tmp_path):
    async with VehicleLogStore(tmp_path / "logs") as store:
        service = VehicleLogService(store, "robot", Authorizer(), settings)
        await service.start()
        try:
            subscription = await service.follow(
                "valid-session", "robot", LogQuery(levels=("ERROR",), limit=2)
            )
            for i in range(64):
                service.enqueue(event(i, level="ERROR" if i < 3 else "INFO"))
            await service.flush()
            first, last = await subscription.next_page(), await subscription.next_page()
            assert [entry.sequence for entry in first.entries] == [1, 2]
            assert first.next_cursor == 2 and first.has_more
            assert [entry.sequence for entry in last.entries] == [3]
            assert last.next_cursor == last.high_water == 64 and not last.has_more
            assert not first.gaps and not last.gaps
            await subscription.close()
        finally:
            await service.close()


async def test_slow_follower_gap_covers_discarded_pages_not_delivered_batch(tmp_path):
    async with VehicleLogStore(tmp_path / "logs") as store:
        service = VehicleLogService(store, "robot", Authorizer(), settings)
        await service.start()
        try:
            subscription = await service.follow("valid-session", "robot", LogQuery())
            for _ in range(9):
                for i in range(64):
                    service.enqueue(event(i))
                await service.flush()
            assert subscription.outbox_depth <= 4
            page = await subscription.next_page()
            assert [entry.sequence for entry in page.entries] == list(range(513, 577))
            assert page.gaps[0].reason == "slow_subscriber"
            assert page.gaps[0].after == 0 and page.gaps[0].through == 512
            assert len(page.model_dump_json().encode()) <= 256 * 1024
            await subscription.close()
        finally:
            await service.close()


async def test_existing_valid_empty_and_fully_pruned_store_keep_metadata(tmp_path):
    from senior_control.vehicle_logs.models import prepare_entry

    directory = tmp_path / "logs"
    async with VehicleLogStore(directory):
        pass
    async with VehicleLogStore(directory) as store:
        page = await store.query(LogQuery())
        assert page.high_water == page.next_cursor == 0 and not page.entries
        await store.append(tuple(prepare_entry(event(i), 1000.0) for i in range(64)))
        await store.prune(1001)
    async with VehicleLogStore(directory) as store:
        page = await store.query(LogQuery())
        assert not page.entries and page.high_water == page.next_cursor == 64
        assert page.gaps[0].through == 64
        appended = await store.append((prepare_entry(event(65), 1002.0),))
        assert appended[0].sequence == 65


async def test_existing_empty_file_is_not_initialized(tmp_path):
    directory = tmp_path / "logs"
    directory.mkdir()
    path = directory / "vehicle-logs.sqlite3"
    path.write_bytes(b"")
    with pytest.raises(ValueError, match="unavailable"):
        async with VehicleLogStore(directory):
            pass
    assert path.read_bytes() == b""
