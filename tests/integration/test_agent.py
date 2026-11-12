"""Task 14 robot agent contract.

Interface assumed by these tests:

* ``Agent(robot_id, *, connector, journal, tasks, home, maps, state, motion, ...)`` owns no
  cloud database and no rclpy executor. Every port is injected.
* ``await agent.dispatch(envelope)`` returns ``OperationResult`` only when the robot
  transaction committed, or when the stored result of an earlier identical operation is
  replayed. A rejection raises ``DomainError``.
* ``await agent.run()`` connects through ``connector``, publishes a full snapshot on every
  connection, and answers each command with a ``ResponseEnvelope`` on the same link.
"""

from __future__ import annotations

import asyncio
import time
from collections.abc import AsyncIterator
from dataclasses import dataclass, replace
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import pytest
from agent_fakes import (
    MAP,
    ROBOT,
    FakeConnector,
    FakeHome,
    FakeLink,
    FakeMaps,
    FakeMotion,
    FakeState,
    FakeTasks,
    operation,
    simple,
    stop,
    until,
    velocity,
)
from pydantic import ValidationError

from senior_control.agent import Agent, RobotSnapshot
from senior_control.agent.dispatcher import Deadlines, StoreTaskPort
from senior_control.agent.journal import AgentJournal
from senior_control.agent.snapshots import (
    SnapshotCache,
    TaskSummary,
    snapshot_envelope,
    snapshot_from_envelope,
)
from senior_control.errors import DomainError, ErrorCode
from senior_control.models import OperationResult, OperationStatus
from senior_control.protocol import SnapshotEnvelope
from senior_control.storage.database import Database
from senior_control.tasks import Schedule, TaskDefinition, TaskStore


@dataclass
class Rig:
    agent: Agent
    connector: FakeConnector
    journal: AgentJournal
    tasks: FakeTasks
    home: FakeHome
    maps: FakeMaps
    motion: FakeMotion
    state: FakeState


@pytest.fixture
async def rig(tmp_path: Path) -> AsyncIterator[Rig]:
    async with Database(tmp_path / "robot.db") as database:
        journal = AgentJournal(database, robot_id=ROBOT)
        await journal.prepare()
        connector = FakeConnector()
        tasks, home, maps, motion, state = (
            FakeTasks(),
            FakeHome(),
            FakeMaps(),
            FakeMotion(),
            FakeState(),
        )
        agent = Agent(
            ROBOT,
            connector=connector,
            journal=journal,
            tasks=tasks,
            home=home,
            maps=maps,
            state=state,
            motion=motion,
            source_clock_verified=True,
            deadlines=Deadlines(edit=1.0, map=0.4, safety=0.5),
            heartbeat_seconds=0.02,
            retry_delay=0.01,
        )
        yield Rig(agent, connector, journal, tasks, home, maps, motion, state)


async def start(rig: Rig, *links: FakeLink) -> asyncio.Task[None]:
    for link in links:
        rig.connector.add(link)
    task = asyncio.ensure_future(rig.agent.run())
    await until(lambda: rig.agent.connected)
    return task


async def finish(task: asyncio.Task[None], rig: Rig) -> None:
    rig.agent.request_shutdown()
    await asyncio.wait_for(task, timeout=2)


def test_agent_exposes_contract() -> None:
    assert asyncio.iscoroutinefunction(Agent.run)
    assert asyncio.iscoroutinefunction(Agent.dispatch)
    assert asyncio.iscoroutinefunction(Agent.publish_snapshot)
    fields = set(RobotSnapshot.__dataclass_fields__)
    assert {
        "robot_id",
        "connected",
        "localized",
        "map_ref",
        "home",
        "task_revision",
        "execution_status",
        "collected_at",
    } <= fields


async def test_edit_ack_only_after_device_commit(rig: Rig) -> None:
    link = FakeLink(epoch=1)
    task = await start(rig, link)
    rig.tasks.gate = asyncio.Event()
    link.inbox.put_nowait(operation("task.create", "op-ack", {"task_id": "t1"}))
    await until(lambda: len(rig.tasks.calls) == 1)
    await asyncio.sleep(0.05)
    assert link.responses == []
    assert rig.tasks.committed == []
    stored = await rig.journal.get("op-ack")
    assert stored is not None and stored.status is not OperationStatus.SUCCEEDED

    rig.tasks.gate.set()
    await until(lambda: len(link.responses) == 1)
    assert rig.tasks.committed == ["create"]
    reply = link.responses[0].payload
    assert reply.ok is True and reply.status == "succeeded" and reply.operation_id == "op-ack"
    stored = await rig.journal.get("op-ack")
    assert stored is not None and stored.status is OperationStatus.SUCCEEDED
    await finish(task, rig)


async def test_failed_commit_is_never_reported_as_success(rig: Rig) -> None:
    link = FakeLink(epoch=1)
    task = await start(rig, link)
    rig.tasks.fail = DomainError(ErrorCode.VALIDATION, "bad schedule")
    link.inbox.put_nowait(operation("task.create", "op-bad", {"task_id": "t1"}))
    await until(lambda: len(link.responses) == 1)
    reply = link.responses[0].payload
    assert reply.ok is False and reply.error is ErrorCode.VALIDATION
    stored = await rig.journal.get("op-bad")
    assert stored is not None and stored.status is OperationStatus.FAILED
    await finish(task, rig)


async def test_offline_edit_fails_not_queued(rig: Rig) -> None:
    assert not rig.agent.connected
    with pytest.raises(DomainError) as caught:
        await rig.agent.dispatch(operation("task.create", "op-offline", {"task_id": "t1"}))
    assert caught.value.code is ErrorCode.OFFLINE
    assert rig.tasks.calls == []
    assert await rig.journal.get("op-offline") is None

    task = await start(rig, FakeLink(epoch=1))
    await asyncio.sleep(0.1)
    assert rig.tasks.calls == []
    await finish(task, rig)


async def test_operation_id_deduplicates(rig: Rig) -> None:
    task = await start(rig, FakeLink(epoch=1))
    first = await rig.agent.dispatch(
        operation("task.update", "op-dup", {"task_id": "t"}, expected_revision=1)
    )
    again = await rig.agent.dispatch(
        operation("task.update", "op-dup", {"task_id": "t"}, expected_revision=1)
    )
    assert first == again == OperationResult("op-dup", OperationStatus.SUCCEEDED)
    assert len(rig.tasks.calls) == 1
    await finish(task, rig)


async def test_same_operation_id_with_different_content_conflicts(rig: Rig) -> None:
    task = await start(rig, FakeLink(epoch=1))
    await rig.agent.dispatch(
        operation("task.update", "op-1", {"task_id": "a"}, expected_revision=1)
    )
    with pytest.raises(DomainError) as caught:
        await rig.agent.dispatch(
            operation("task.update", "op-1", {"task_id": "b"}, expected_revision=1)
        )
    assert caught.value.code is ErrorCode.CONFLICT
    assert len(rig.tasks.calls) == 1
    await finish(task, rig)


async def test_expected_revision_rejects_stale_write(rig: Rig) -> None:
    task = await start(rig, FakeLink(epoch=1))
    await rig.agent.dispatch(
        operation("task.update", "op-new", {"task_id": "t"}, expected_revision=1)
    )
    assert rig.tasks.revision == 2
    with pytest.raises(DomainError) as caught:
        await rig.agent.dispatch(
            operation("task.update", "op-stale", {"task_id": "t"}, expected_revision=1)
        )
    assert caught.value.code is ErrorCode.CONFLICT
    assert rig.tasks.revision == 2
    stored = await rig.journal.get("op-stale")
    assert stored is not None and stored.status is OperationStatus.FAILED
    await finish(task, rig)


async def test_reconnect_fetches_full_snapshot(rig: Rig) -> None:
    first, second = FakeLink(epoch=1), FakeLink(epoch=2)
    task = await start(rig, first)
    await until(lambda: len(first.snapshots) == 1)
    assert first.snapshots[0].connected is True
    assert first.snapshots[0].map_ref == MAP
    assert first.snapshots[0].task_revision == 4

    rig.state.task_revision = 9
    first.drop()
    await until(lambda: not rig.agent.connected)
    rig.connector.add(second)
    await until(lambda: len(second.snapshots) == 1)
    snapshot = second.snapshots[0]
    assert isinstance(snapshot, RobotSnapshot)
    assert snapshot.task_revision == 9
    assert snapshot.home is not None and snapshot.localized is True
    assert abs(snapshot.collected_at - time.time()) < 5
    assert first.closed
    await finish(task, rig)


async def test_reconnect_rejects_old_command_but_returns_committed_result(rig: Rig) -> None:
    first, second = FakeLink(epoch=1), FakeLink(epoch=2)
    task = await start(rig, first)
    committed = operation("task.update", "op-done", {"task_id": "t"}, epoch=1, expected_revision=1)
    await rig.agent.dispatch(committed)
    first.drop()
    await until(lambda: not rig.agent.connected)
    rig.connector.add(second)
    await until(lambda: rig.agent.connected and len(second.snapshots) == 1)

    replay = await rig.agent.dispatch(committed)
    assert replay == OperationResult("op-done", OperationStatus.SUCCEEDED)
    assert len(rig.tasks.calls) == 1

    with pytest.raises(DomainError) as caught:
        await rig.agent.dispatch(operation("task.create", "op-late", {"task_id": "n"}, epoch=1))
    assert caught.value.code is ErrorCode.CONFLICT
    assert len(rig.tasks.calls) == 1

    queried = await rig.agent.dispatch(operation("operation.query", "op-done", epoch=2))
    assert queried.status is OperationStatus.SUCCEEDED
    await finish(task, rig)


async def test_unknown_operation_query_is_reported(rig: Rig) -> None:
    task = await start(rig, FakeLink(epoch=1))
    with pytest.raises(DomainError) as caught:
        await rig.agent.dispatch(operation("operation.query", "never-issued"))
    assert caught.value.code is ErrorCode.OPERATION_UNKNOWN
    await finish(task, rig)


async def test_long_map_export_does_not_block_heartbeat_or_stop(rig: Rig) -> None:
    link = FakeLink(epoch=1)
    task = await start(rig, link)
    rig.maps.gate = asyncio.Event()
    export = asyncio.ensure_future(
        rig.agent.dispatch(
            operation(
                "map.export",
                "op-export",
                {"map_id": MAP.map_id, "reference_revision": 3, "name": "export-1"},
            )
        )
    )
    await until(lambda: rig.maps.calls == ["export"])
    beats = link.heartbeats
    started = time.monotonic()
    result = await rig.agent.dispatch(stop())
    assert time.monotonic() - started < 0.1
    assert result.status is OperationStatus.SUCCEEDED
    assert rig.motion.events.count("stop:req-stop") == 1
    await asyncio.sleep(0.1)
    assert link.heartbeats >= beats + 3
    assert not export.done()

    with pytest.raises(DomainError) as caught:
        await export
    assert caught.value.code is ErrorCode.TIMEOUT
    stored = await rig.journal.get("op-export")
    assert stored is not None and stored.status is OperationStatus.NEEDS_REVIEW
    await finish(task, rig)


async def test_other_robot_heartbeat_not_eligible(rig: Rig) -> None:
    link = FakeLink(epoch=1)
    task = await start(rig, link)
    rig.agent.note_heartbeat("robot-2")
    assert rig.agent.eligible_for_motion is False
    with pytest.raises(DomainError) as caught:
        await rig.agent.dispatch(velocity())
    assert caught.value.code is ErrorCode.SAFETY_REJECTED
    await asyncio.sleep(0.05)
    assert not [event for event in rig.motion.events if event.startswith("drive")]

    rig.agent.note_heartbeat(ROBOT)
    assert rig.agent.eligible_for_motion is True
    await rig.agent.dispatch(velocity())
    await until(lambda: any(event.startswith("drive") for event in rig.motion.events))
    assert "drive:3:0.2:0.0" in rig.motion.events
    await finish(task, rig)


async def test_heartbeat_send_success_does_not_restore_eligibility(rig: Rig) -> None:
    from senior_control.agent.connection import HeartbeatFrame

    link = FakeLink(epoch=4)
    task = await start(rig, link)
    rig.agent.note_heartbeat(ROBOT)
    original = link.send_heartbeat
    failed = asyncio.Event()
    release = asyncio.Event()

    async def flaky() -> None:
        if not failed.is_set():
            failed.set()
            raise ConnectionError("auth heartbeat failed")
        await release.wait()
        await original()

    link.send_heartbeat = flaky  # type: ignore[method-assign]
    try:
        await until(lambda: not rig.agent.eligible_for_motion)
        await until(lambda: "disable:4" in rig.motion.events)
        with pytest.raises(DomainError) as caught:
            await rig.agent.dispatch(simple("control.acquire", seq=1, epoch=4))
        assert caught.value.code is ErrorCode.SAFETY_REJECTED
        release.set()
        await original()
        assert not rig.agent.eligible_for_motion
        link.inbox.put_nowait(HeartbeatFrame(ROBOT, time.monotonic() + 0.8))
        await until(lambda: not rig.agent.connected)
        assert not rig.agent.eligible_for_motion
    finally:
        release.set()
        await finish(task, rig)


async def test_dropped_link_disables_its_epoch(rig: Rig) -> None:
    link = FakeLink(epoch=4)
    task = await start(rig, link)
    rig.agent.note_heartbeat(ROBOT)
    link.drop()
    await until(lambda: "disable:4" in rig.motion.events)
    with pytest.raises(DomainError) as caught:
        await rig.agent.dispatch(simple("navigation.start", seq=2, epoch=4))
    assert caught.value.code is ErrorCode.OFFLINE
    assert not [event for event in rig.motion.events if event.startswith("forward")]
    await finish(task, rig)


async def test_velocity_older_than_the_command_ttl_is_not_driven(rig: Rig) -> None:
    task = await start(rig, FakeLink(epoch=1))
    rig.agent.note_heartbeat(ROBOT)
    late = velocity()
    object.__setattr__(late, "expires_at", time.time() + 4)
    with pytest.raises(DomainError) as caught:
        await rig.agent.dispatch(late)
    assert caught.value.code is ErrorCode.TIMEOUT
    await asyncio.sleep(0.05)
    assert not [event for event in rig.motion.events if event.startswith("drive")]
    await finish(task, rig)


async def test_wrong_robot_envelope_is_rejected(rig: Rig) -> None:
    task = await start(rig, FakeLink(epoch=1))
    with pytest.raises(DomainError) as caught:
        await rig.agent.dispatch(operation("task.create", "op-x", robot_id="robot-2"))
    assert caught.value.code is ErrorCode.VALIDATION
    assert rig.tasks.calls == []
    await finish(task, rig)


async def test_expired_command_is_rejected(rig: Rig) -> None:
    task = await start(rig, FakeLink(epoch=1))
    with pytest.raises(DomainError) as caught:
        await rig.agent.dispatch(operation("task.create", "op-old", expires_in=-1))
    assert caught.value.code is ErrorCode.TIMEOUT
    assert rig.tasks.calls == []
    await finish(task, rig)


async def test_published_snapshot_carries_the_home_edit_revision(rig: Rig) -> None:
    """The page can only send the revision the store will compare."""
    task = await start(rig, FakeLink(epoch=1))
    unset = await rig.agent.publish_snapshot(ROBOT)
    assert unset.home_revision == 0
    frame = snapshot_envelope(
        unset,
        robot_id=ROBOT,
        session_id="session-1",
        connection_epoch=1,
        request_id="snap-0",
        seq=1,
        expires_at=9e9,
    )
    assert frame.payload.home_revision == 0
    assert snapshot_from_envelope(frame).home_revision == 0

    rig.home.revision = 4
    published = await rig.agent.publish_snapshot(ROBOT)
    assert published.home_revision == 4
    frame = snapshot_envelope(
        published,
        robot_id=ROBOT,
        session_id="session-1",
        connection_epoch=1,
        request_id="snap-4",
        seq=2,
        expires_at=9e9,
    )
    assert frame.payload.home_revision == 4
    assert snapshot_from_envelope(frame).home_revision == 4
    await finish(task, rig)


async def test_publish_snapshot_checks_robot(rig: Rig) -> None:
    task = await start(rig, FakeLink(epoch=1))
    snapshot = await rig.agent.publish_snapshot(ROBOT)
    assert snapshot.robot_id == ROBOT and snapshot.connected is True
    with pytest.raises(DomainError):
        await rig.agent.publish_snapshot("robot-2")
    await finish(task, rig)


def test_cloud_cache_marks_offline_snapshot_stale() -> None:
    now = [1000.0]
    cache = SnapshotCache(stale_after=5.0, clock=lambda: now[0])
    assert cache.get(ROBOT) is None
    snapshot = RobotSnapshot(
        robot_id=ROBOT,
        connected=True,
        localized=True,
        map_ref=MAP,
        home=None,
        task_revision=2,
        execution_status="idle",
        collected_at=999.0,
    )
    cache.store(snapshot)
    live = cache.get(ROBOT)
    assert live is not None and live.stale is False and live.last_synced_at == 1000.0

    now[0] = 1003.0
    cache.mark_offline(ROBOT)
    cached = cache.get(ROBOT)
    assert cached is not None and cached.stale is True
    assert cached.last_synced_at == 1000.0 and cached.snapshot.task_revision == 2

    now[0] = 1010.0
    cache.store(snapshot)
    refreshed = cache.get(ROBOT)
    assert refreshed is not None and refreshed.stale is False
    now[0] = 1100.0
    aged = cache.get(ROBOT)
    assert aged is not None and aged.stale is True


async def test_task_edit_commits_in_robot_database(tmp_path: Path) -> None:
    """The adapter reuses TaskStore; the revision guard comes from the robot's SQLite."""
    async with Database(tmp_path / "robot.db") as database:
        store = TaskStore(database, robot_id=ROBOT)
        await store.prepare()
        journal = AgentJournal(database, robot_id=ROBOT)
        await journal.prepare()
        connector = FakeConnector()
        agent = Agent(
            ROBOT,
            connector=connector,
            journal=journal,
            tasks=StoreTaskPort(store, robot_id=ROBOT),
            home=FakeHome(),
            maps=FakeMaps(),
            state=FakeState(),
            motion=FakeMotion(),
            heartbeat_seconds=0.02,
            retry_delay=0.01,
        )
        connector.add(FakeLink(epoch=1))
        runner = asyncio.ensure_future(agent.run())
        await until(lambda: agent.connected)
        created = datetime(2026, 10, 27, tzinfo=timezone.utc)
        body = {
            "task_id": "patrol",
            "map_id": MAP.map_id,
            "reference_revision": MAP.reference_revision,
            "waypoint_ids": ["wp-1"],
            "schedule": {"kind": "cron", "expression": "0 8 * * *", "timezone": "Asia/Shanghai"},
            "timezone": "Asia/Shanghai",
            "enabled": True,
            "created_at": created.isoformat(),
        }
        await agent.dispatch(operation("task.create", "op-create", body))
        stored = await store.get("patrol")
        assert stored is not None and stored.revision == 1 and stored.enabled is True

        await agent.dispatch(
            operation(
                "task.enable",
                "op-off",
                {"task_id": "patrol", "enabled": False},
                expected_revision=1,
            )
        )
        stored = await store.get("patrol")
        assert stored is not None and stored.revision == 2 and stored.enabled is False

        with pytest.raises(DomainError) as caught:
            await agent.dispatch(
                operation(
                    "task.enable",
                    "op-stale",
                    {"task_id": "patrol", "enabled": True},
                    expected_revision=1,
                )
            )
        assert caught.value.code is ErrorCode.CONFLICT
        stored = await store.get("patrol")
        assert stored is not None and stored.enabled is False and stored.revision == 2
        assert isinstance(stored, TaskDefinition) and isinstance(stored.schedule, Schedule)
        agent.request_shutdown()
        await asyncio.wait_for(runner, timeout=2)


async def test_shutdown_disables_motion_with_the_link_epoch(rig: Rig) -> None:
    task = await start(rig, FakeLink(epoch=7))
    await finish(task, rig)
    assert rig.motion.events[-3:] == ["disable:7", "exit", "close"]


def task_body(task_id: str = "patrol") -> dict[str, object]:
    return {
        "task_id": task_id,
        "map_id": MAP.map_id,
        "reference_revision": MAP.reference_revision,
        "waypoint_ids": ["wp-1"],
        "schedule": {"kind": "cron", "expression": "0 8 * * *", "timezone": "Asia/Shanghai"},
        "timezone": "Asia/Shanghai",
        "enabled": True,
        "created_at": datetime(2026, 10, 27, tzinfo=timezone.utc).isoformat(),
    }


@dataclass
class StoreRig:
    agent: Agent
    store: TaskStore
    journal: AgentJournal
    state: FakeState
    runner: asyncio.Task[None]
    database: Database
    connector: FakeConnector
    first: FakeLink


@pytest.fixture
async def store_rig(tmp_path: Path) -> AsyncIterator[StoreRig]:
    async with Database(tmp_path / "robot.db") as database:
        store = TaskStore(database, robot_id=ROBOT)
        await store.prepare()
        journal = AgentJournal(database, robot_id=ROBOT)
        await journal.prepare()
        first = FakeLink(epoch=1)
        connector = FakeConnector(first)
        state = FakeState(runnable_task_ids=("patrol", "other"))
        agent = Agent(
            ROBOT,
            connector=connector,
            journal=journal,
            tasks=StoreTaskPort(store, robot_id=ROBOT),
            home=FakeHome(),
            maps=FakeMaps(),
            state=state,
            motion=FakeMotion(),
            heartbeat_seconds=0.02,
            retry_delay=0.01,
        )
        runner = asyncio.ensure_future(agent.run())
        await until(lambda: agent.connected)
        yield StoreRig(agent, store, journal, state, runner, database, connector, first)
        agent.request_shutdown()
        await asyncio.wait_for(runner, timeout=2)


async def test_task_delete_disables_keeps_the_row_and_hides_it(store_rig: StoreRig) -> None:
    agent, store, journal = store_rig.agent, store_rig.store, store_rig.journal
    await agent.dispatch(operation("task.create", "op-create", task_body()))
    result = await agent.dispatch(
        operation("task.delete", "op-del", {"task_id": "patrol"}, expected_revision=1)
    )
    assert result.status is OperationStatus.SUCCEEDED
    kept = await store.get("patrol")
    assert kept is not None and kept.enabled is False and kept.revision == 2
    assert await journal.deleted_task_ids() == frozenset({"patrol"})
    snapshot = await agent.publish_snapshot(ROBOT)
    assert snapshot.runnable_task_ids == ("other",)

    with pytest.raises(DomainError) as caught:
        await agent.dispatch(
            operation(
                "task.enable",
                "op-back",
                {"task_id": "patrol", "enabled": True},
                expected_revision=2,
            )
        )
    assert caught.value.code is ErrorCode.CONFLICT
    stored = await store.get("patrol")
    assert stored is not None and stored.enabled is False


async def test_task_delete_wrong_revision_or_missing_task_is_a_conflict(
    store_rig: StoreRig,
) -> None:
    agent, store, journal = store_rig.agent, store_rig.store, store_rig.journal
    await agent.dispatch(operation("task.create", "op-create", task_body()))
    with pytest.raises(DomainError) as stale:
        await agent.dispatch(
            operation("task.delete", "op-stale", {"task_id": "patrol"}, expected_revision=5)
        )
    assert stale.value.code is ErrorCode.CONFLICT
    with pytest.raises(DomainError) as missing:
        await agent.dispatch(
            operation("task.delete", "op-none", {"task_id": "ghost"}, expected_revision=1)
        )
    assert missing.value.code is ErrorCode.CONFLICT
    kept = await store.get("patrol")
    assert kept is not None and kept.enabled is True and kept.revision == 1
    assert await journal.deleted_task_ids() == frozenset()


async def test_task_delete_rolls_back_if_the_tombstone_fails(store_rig: StoreRig) -> None:
    agent, store, journal = store_rig.agent, store_rig.store, store_rig.journal
    await agent.dispatch(operation("task.create", "op-create", task_body()))
    async with store_rig.database.write_lock:
        connection = store_rig.database.connection
        await connection.execute(
            """
            CREATE TRIGGER reject_tombstone BEFORE INSERT ON agent_deleted_task
            BEGIN SELECT RAISE(ABORT, 'tombstone write failed'); END
            """
        )
        await connection.commit()

    with pytest.raises(DomainError) as caught:
        await agent.dispatch(
            operation("task.delete", "op-del", {"task_id": "patrol"}, expected_revision=1)
        )
    assert caught.value.code is ErrorCode.OPERATION_UNKNOWN
    kept = await store.get("patrol")
    assert kept is not None and kept.enabled and kept.revision == 1
    assert await journal.deleted_task_ids() == frozenset()
    entry = await journal.get("op-del")
    assert entry is not None and entry.status is OperationStatus.NEEDS_REVIEW


async def test_reconnect_hides_deleted_task_and_preserves_snapshot_revision(
    store_rig: StoreRig, monkeypatch: pytest.MonkeyPatch
) -> None:
    agent, store = store_rig.agent, store_rig.store
    collect = store_rig.state.collect

    async def collect_with_stored_tasks():
        state = await collect()
        tasks = await store.list_tasks()
        return replace(
            state,
            task_revision=sum(task.revision for task in tasks),
            tasks=tuple(TaskSummary(task.task_id, task.revision, task.enabled) for task in tasks),
        )

    monkeypatch.setattr(store_rig.state, "collect", collect_with_stored_tasks)
    await agent.dispatch(operation("task.create", "op-create", task_body()))
    await agent.dispatch(operation("task.create", "op-other", task_body("other")))
    before = await agent.publish_snapshot(ROBOT)
    assert {item.task_id for item in before.tasks} == {"patrol", "other"}
    command = operation("task.delete", "op-del", {"task_id": "patrol"}, expected_revision=1)
    assert (await agent.dispatch(command)).status is OperationStatus.SUCCEEDED
    assert (await agent.dispatch(command)).status is OperationStatus.SUCCEEDED

    store_rig.first.drop()
    await until(lambda: not agent.connected)
    second = FakeLink(epoch=2)
    store_rig.connector.add(second)
    await until(lambda: len(second.snapshots) == 1)
    snapshot = second.snapshots[0]
    assert snapshot.tasks == (TaskSummary("other", 1, True),)
    assert snapshot.runnable_task_ids == ("other",)
    assert snapshot.task_revision == before.task_revision + 1


@pytest.mark.parametrize("available", [False, True])
async def test_snapshot_keeps_revision_and_tombstones_from_one_task_cut(
    store_rig: StoreRig, monkeypatch: pytest.MonkeyPatch, available: bool
) -> None:
    from senior_control.agent.composition import SnapshotState
    from senior_control.agent.ports import IpcState
    from senior_control.waypoints.store import WaypointStore

    class RuntimeClient:
        async def request(self, kind: str, **fields: Any) -> dict[str, Any]:
            if not available:
                raise OSError("robot state unavailable")
            assert kind == "state.get"
            return {
                "v": 1,
                "ok": True,
                "sampled_at": time.monotonic(),
                "localized": False,
                "safety_clear": True,
                "manual_driving": False,
                "map_id": MAP.map_id,
                "reference_revision": MAP.reference_revision,
                "execution_status": "idle",
            }

    agent = store_rig.agent
    await agent.dispatch(operation("task.create", "op-create", task_body()))
    waypoints = WaypointStore(store_rig.database, robot_id=ROBOT)
    await waypoints.prepare()
    production = SnapshotState(IpcState(RuntimeClient()), store_rig.store, waypoints)
    captured = asyncio.Event()
    resume = asyncio.Event()

    async def pause_after_collect():
        state = await production.collect()
        captured.set()
        await resume.wait()
        return state

    monkeypatch.setattr(store_rig.state, "collect", pause_after_collect)
    pending = asyncio.create_task(agent.publish_snapshot(ROBOT))
    try:
        await asyncio.wait_for(captured.wait(), timeout=1)
        command = operation("task.delete", "op-del", {"task_id": "patrol"}, expected_revision=1)
        assert (await agent.dispatch(command)).status is OperationStatus.SUCCEEDED
    finally:
        resume.set()
    before = await pending
    assert before.execution_status == ("idle" if available else "unavailable")
    assert before.task_revision == 1
    assert before.tasks == (TaskSummary("patrol", 1, True),)

    monkeypatch.setattr(store_rig.state, "collect", production.collect)
    after = await agent.publish_snapshot(ROBOT)
    assert after.task_revision == 2
    assert after.tasks == ()


async def test_task_delete_is_robot_scoped(store_rig: StoreRig) -> None:
    await store_rig.agent.dispatch(operation("task.create", "op-create", task_body()))
    foreign = TaskStore(store_rig.database, robot_id="other-robot")
    await foreign.prepare()
    with pytest.raises(DomainError) as caught:
        await foreign.delete("patrol", 1, operation_id="other-delete")
    assert caught.value.code is ErrorCode.CONFLICT
    current = await store_rig.store.get("patrol")
    assert current is not None and current.enabled and current.revision == 1
    assert await store_rig.journal.deleted_task_ids() == frozenset()


class CachingLink(FakeLink):
    """A link whose far end is a SnapshotCache, as the relay's Hub is."""

    def __init__(self, epoch: int, cache: SnapshotCache) -> None:
        super().__init__(epoch)
        self.cache = cache

    async def send_snapshot(self, snapshot: Any) -> None:
        await super().send_snapshot(snapshot)
        self.cache.store(snapshot)


async def idle_agent(tmp_path: Path, link: FakeLink, state: FakeState, **extra: Any) -> Agent:
    database = await Database(tmp_path / "robot.db").__aenter__()
    journal = AgentJournal(database, robot_id=ROBOT)
    try:
        await journal.prepare()
    except BaseException:
        await database.__aexit__(None, None, None)
        raise

    async def cleanup() -> None:
        await database.__aexit__(None, None, None)

    return Agent(
        ROBOT,
        connector=FakeConnector(link),
        journal=journal,
        tasks=FakeTasks(),
        home=FakeHome(),
        maps=FakeMaps(),
        state=state,
        motion=FakeMotion(),
        heartbeat_seconds=0.02,
        retry_delay=0.01,
        cleanup=cleanup,
        **extra,
    )


async def test_idle_connected_robot_stays_fresh_past_stale_after(tmp_path: Path) -> None:
    cache = SnapshotCache(stale_after=0.3)
    link = CachingLink(1, cache)
    agent = await idle_agent(
        tmp_path, link, FakeState(), snapshot_seconds=0.05, state_poll_seconds=0.02
    )
    runner = asyncio.ensure_future(agent.run())
    await until(lambda: agent.connected)
    for _ in range(4):
        await asyncio.sleep(0.2)  # no edit, no state change; the clock passes stale_after
        cached = cache.get(ROBOT)
        assert cached is not None
        assert cached.stale is False and cached.snapshot.connected is True
    agent.request_shutdown()
    await asyncio.wait_for(runner, timeout=2)
    # Offline still becomes stale, once the robot stops refreshing and the cache is told.
    cache.mark_offline(ROBOT)
    gone = cache.get(ROBOT)
    assert gone is not None and gone.stale is True and gone.snapshot.connected is False
    await asyncio.sleep(0.4)
    assert cache.get(ROBOT) is not None and cache.get(ROBOT).stale is True  # type: ignore[union-attr]


@pytest.mark.parametrize("failure", [OSError, asyncio.TimeoutError])
async def test_failed_collect_does_not_end_the_keepalive(
    tmp_path: Path, failure: type[Exception]
) -> None:
    class FlakyState(FakeState):
        async def collect(self) -> Any:
            if self.calls == 1:  # the first collect (the connect publish) already succeeded
                self.calls += 1
                raise failure("state port gone")  # a fresh instance: no traceback is kept
            return await super().collect()

    cache = SnapshotCache(stale_after=0.3)
    link = CachingLink(1, cache)
    state = FlakyState()
    agent = await idle_agent(tmp_path, link, state, snapshot_seconds=0.05, state_poll_seconds=0.02)
    runner = asyncio.ensure_future(agent.run())
    try:
        await until(lambda: agent.connected)
        await until(lambda: state.calls >= 2)  # the failing collect has happened
        sent = len(link.snapshots)
        await until(lambda: len(link.snapshots) >= sent + 2)  # later snapshots still arrive
        for _ in range(3):
            await asyncio.sleep(0.2)  # past stale_after, with the link up
            cached = cache.get(ROBOT)
            assert cached is not None
            assert cached.stale is False and cached.snapshot.connected is True
    finally:
        agent.request_shutdown()
        await asyncio.wait_for(runner, timeout=2)


async def test_state_change_is_pushed_without_an_edit(tmp_path: Path) -> None:
    link = FakeLink(1)
    state = FakeState()
    agent = await idle_agent(tmp_path, link, state, snapshot_seconds=30.0, state_poll_seconds=0.02)
    runner = asyncio.ensure_future(agent.run())
    await until(lambda: len(link.snapshots) == 1)
    await asyncio.sleep(0.15)
    assert len(link.snapshots) == 1  # nothing changed, so nothing is re-sent before the keepalive
    state.localized = False
    await until(lambda: len(link.snapshots) == 2)
    assert link.snapshots[1].localized is False
    state.execution_status = "navigating"
    await until(lambda: len(link.snapshots) == 3)
    assert link.snapshots[2].execution_status == "navigating"
    state.navigation_ready = True
    await until(lambda: len(link.snapshots) == 4)
    assert link.snapshots[3].navigation_ready is True
    agent.request_shutdown()
    await asyncio.wait_for(runner, timeout=2)


async def test_replay_of_needs_review_is_not_acknowledged_as_success(rig: Rig) -> None:
    link = FakeLink(epoch=1)
    task = await start(rig, link)
    rig.maps.gate = asyncio.Event()
    export = operation(
        "map.export",
        "op-review",
        {"map_id": MAP.map_id, "reference_revision": 3, "name": "export-1"},
    )
    link.inbox.put_nowait(export)
    await until(lambda: len(link.responses) == 1)  # the deadline passed
    assert link.responses[0].payload.ok is False
    assert link.responses[0].payload.error is ErrorCode.TIMEOUT

    link.inbox.put_nowait(export)
    await until(lambda: len(link.responses) == 2)
    retry = link.responses[1].payload
    assert retry.ok is False and retry.error is ErrorCode.OPERATION_UNKNOWN
    assert retry.status == "needs_review"
    assert rig.maps.calls == ["export"]

    link.inbox.put_nowait(operation("operation.query", "op-review"))
    await until(lambda: len(link.responses) == 3)
    queried = link.responses[2].payload
    assert queried.ok is True and queried.status == "needs_review"
    with pytest.raises(DomainError) as caught:
        await rig.agent.dispatch(export)
    assert caught.value.code is ErrorCode.OPERATION_UNKNOWN
    await finish(task, rig)


async def test_replay_of_a_running_operation_is_a_conflict(rig: Rig) -> None:
    link = FakeLink(epoch=1)
    task = await start(rig, link)
    rig.tasks.gate = asyncio.Event()
    edit = operation("task.create", "op-running", {"task_id": "t1"})
    link.inbox.put_nowait(edit)
    await until(lambda: len(rig.tasks.calls) == 1)
    link.inbox.put_nowait(edit)
    await until(lambda: len(link.responses) == 1)
    duplicate = link.responses[0].payload
    assert duplicate.ok is False and duplicate.error is ErrorCode.CONFLICT
    assert duplicate.status == "running"
    assert len(rig.tasks.calls) == 1

    link.inbox.put_nowait(operation("operation.query", "op-running"))
    await until(lambda: len(link.responses) == 2)
    assert link.responses[1].payload.ok is True and link.responses[1].payload.status == "running"

    rig.tasks.gate.set()
    await until(lambda: len(link.responses) == 3)
    done = link.responses[2].payload
    assert done.ok is True and done.status == "succeeded" and done.operation_id == "op-running"
    link.inbox.put_nowait(edit)
    await until(lambda: len(link.responses) == 4)
    assert link.responses[3].payload.ok is True and link.responses[3].payload.status == "succeeded"
    await finish(task, rig)


async def test_velocity_with_future_send_time_is_rejected(rig: Rig) -> None:
    task = await start(rig, FakeLink(epoch=1))
    try:
        rig.agent.note_heartbeat(ROBOT)
        command = velocity()
        object.__setattr__(command, "expires_at", time.time() + 10.0)
        with pytest.raises(DomainError) as caught:
            await rig.agent.dispatch(command)
        assert caught.value.code is ErrorCode.TIMEOUT
    finally:
        await finish(task, rig)


async def test_authenticated_reply_renews_ros_eligibility(rig: Rig) -> None:
    from senior_control.agent.connection import HeartbeatFrame

    link = FakeLink(epoch=1)
    task = await start(rig, link)
    try:
        deadline = time.monotonic() + 0.7
        link.inbox.put_nowait(HeartbeatFrame(ROBOT, deadline))
        await until(lambda: "heartbeat:1" in rig.motion.events)
        assert rig.agent.eligible_for_motion
    finally:
        await finish(task, rig)


async def test_failed_heartbeat_closes_session_and_forces_reconnect(rig: Rig) -> None:
    from senior_control.agent.connection import HeartbeatFrame

    class BrokenLink(FakeLink):
        async def send_heartbeat(self) -> None:
            raise ConnectionError("half open")

        async def close(self) -> None:
            await super().close()
            self.drop()

    link = BrokenLink(epoch=1)
    replacement = FakeLink(epoch=2)
    rig.connector.add(link)
    rig.connector.add(replacement)
    task = asyncio.create_task(rig.agent.run())
    try:
        await until(lambda: link.closed)
        await until(lambda: rig.agent.connected and rig.agent._epoch == 2)
        assert not rig.agent.eligible_for_motion
        replacement.inbox.put_nowait(HeartbeatFrame(ROBOT, time.monotonic() + 0.7))
        await until(lambda: rig.agent.eligible_for_motion)
        assert "disable:1" in rig.motion.events
    finally:
        await finish(task, rig)


async def test_slow_snapshot_does_not_block_heartbeat_reply(tmp_path: Path) -> None:
    from senior_control.agent.connection import HeartbeatFrame

    class SlowState(FakeState):
        async def collect(self) -> Any:
            await asyncio.Event().wait()
            return await super().collect()

    link = FakeLink(1)
    agent = await idle_agent(tmp_path, link, SlowState())
    task = asyncio.create_task(agent.run())
    try:
        await until(lambda: agent.connected)
        link.inbox.put_nowait(HeartbeatFrame(ROBOT, time.monotonic() + 0.7))
        await until(lambda: agent.eligible_for_motion, timeout=0.5)
    finally:
        agent.request_shutdown()
        await asyncio.wait_for(task, 2)


async def test_disconnect_cancels_inflight_old_epoch_operations(rig: Rig, monkeypatch) -> None:
    closing = asyncio.Event()
    persist = asyncio.Event()
    original_finish = rig.journal.finish

    async def gated_finish(*args, **kwargs):
        closing.set()
        await persist.wait()
        return await original_finish(*args, **kwargs)

    monkeypatch.setattr(rig.journal, "finish", gated_finish)
    link = FakeLink(epoch=1)
    task = await start(rig, link)
    try:
        rig.maps.gate = asyncio.Event()
        link.inbox.put_nowait(
            operation(
                "map.export",
                "old-export",
                {"map_id": MAP.map_id, "reference_revision": 3, "name": "export"},
            )
        )
        await until(lambda: bool(rig.maps.calls))
        inflight = set(rig.agent._inflight)
        assert inflight
        link.drop()
        await asyncio.wait_for(closing.wait(), timeout=2)
        # Cancellation must revoke the old link before waiting for journal I/O.
        assert not rig.agent.connected
        assert not rig.agent.eligible_for_motion
        assert any(not operation.done() for operation in inflight)
        persist.set()
        _, pending = await asyncio.wait(inflight, timeout=2)
        assert not pending
        # A regular map timeout returns an error response, rather than cancelling
        # the task. It must not satisfy this disconnect assertion.
        assert all(operation.cancelled() for operation in inflight)
        assert not rig.agent._inflight
        entry = await rig.journal.get("old-export")
        assert entry is not None and entry.status is OperationStatus.NEEDS_REVIEW
        assert entry.error is None
    finally:
        persist.set()
        await finish(task, rig)


async def test_heartbeat_expiring_during_ros_ack_never_grants_eligibility(rig: Rig) -> None:
    from senior_control.agent.connection import HeartbeatFrame

    link = FakeLink(epoch=1)
    task = await start(rig, link)

    async def slow_ack(epoch: int, deadline: float) -> None:
        await asyncio.sleep(0.05)

    rig.motion.server_heartbeat = slow_ack  # type: ignore[method-assign]
    try:
        link.inbox.put_nowait(HeartbeatFrame(ROBOT, time.monotonic() + 0.02))
        await asyncio.sleep(0.1)
        assert not rig.agent.eligible_for_motion
    finally:
        await finish(task, rig)


async def test_velocity_fails_closed_without_verified_source_clock(rig: Rig) -> None:
    link = FakeLink(epoch=1)
    rig.agent._source_clock_verified = False
    task = await start(rig, link)
    try:
        rig.agent.note_heartbeat(ROBOT)
        with pytest.raises(DomainError) as caught:
            await rig.agent.dispatch(velocity())
        assert caught.value.code is ErrorCode.UNAVAILABLE
        assert not any(event.startswith("drive:") for event in rig.motion.events)
    finally:
        await finish(task, rig)


async def test_overload_bounds_work_and_reserves_stop_capacity(rig: Rig) -> None:
    from senior_control.agent.main import MAX_INFLIGHT

    link = FakeLink(epoch=1)
    release = asyncio.Event()

    async def held(link: FakeLink, envelope: Any) -> None:
        await release.wait()

    rig.agent._answer = held  # type: ignore[method-assign]
    try:
        for seq in range(MAX_INFLIGHT):
            rig.agent._accept(link, operation("task.create", f"op-{seq}"))
        rig.agent._accept(link, stop())
        assert len(rig.agent._inflight) == MAX_INFLIGHT + 1
        for _ in range(500):
            with pytest.raises(ConnectionError):
                rig.agent._accept(link, operation("task.create", "overflow"))
        assert len(rig.agent._inflight) == MAX_INFLIGHT + 1
    finally:
        release.set()
        await asyncio.gather(*rig.agent._inflight, return_exceptions=True)


async def test_late_ros_ack_cannot_restore_after_heartbeat_failure(rig: Rig) -> None:
    from senior_control.agent.connection import HeartbeatFrame

    link = FakeLink(epoch=1)
    task = await start(rig, link)
    entered = asyncio.Event()
    release = asyncio.Event()

    async def delayed_ack(epoch: int, deadline: float) -> None:
        entered.set()
        await release.wait()

    async def failed_send() -> None:
        raise ConnectionError("lost")

    rig.motion.server_heartbeat = delayed_ack  # type: ignore[method-assign]
    try:
        link.inbox.put_nowait(HeartbeatFrame(ROBOT, time.monotonic() + 0.8))
        await asyncio.wait_for(entered.wait(), 0.5)
        link.send_heartbeat = failed_send  # type: ignore[method-assign]
        await until(lambda: "disable:1" in rig.motion.events)
        release.set()
        await asyncio.sleep(0.05)
        assert not rig.agent.eligible_for_motion
    finally:
        release.set()
        await finish(task, rig)


async def test_heartbeat_survives_full_work_slots_before_overload_closes_session(rig: Rig) -> None:
    from senior_control.agent.connection import HeartbeatFrame
    from senior_control.agent.main import MAX_INFLIGHT

    link = FakeLink(epoch=1)
    release = asyncio.Event()

    async def held(link: FakeLink, envelope: Any) -> None:
        await release.wait()

    rig.agent._answer = held  # type: ignore[method-assign]
    task = await start(rig, link)
    try:
        for seq in range(MAX_INFLIGHT):
            link.inbox.put_nowait(operation("task.create", f"load-{seq}"))
        link.inbox.put_nowait(stop())
        link.inbox.put_nowait(HeartbeatFrame(ROBOT, time.monotonic() + 0.7))
        await until(lambda: "heartbeat:1" in rig.motion.events)
        assert len(rig.agent._inflight) <= MAX_INFLIGHT + 1
        link.inbox.put_nowait(operation("task.create", "overload"))
        await until(lambda: link.closed)
        assert "disable:1" in rig.motion.events
    finally:
        release.set()
        await finish(task, rig)


async def test_disconnect_and_reconnect_records_loss_between_scans(rig: Rig) -> None:
    from senior_control.agent.connection import HeartbeatFrame

    lost = asyncio.Event()

    async def persist_loss(generation: int) -> None:
        lost.set()

    rig.agent._on_eligibility_lost = persist_loss
    first, second = FakeLink(1), FakeLink(2)
    task = await start(rig, first, second)
    try:
        first.inbox.put_nowait(HeartbeatFrame(ROBOT, time.monotonic() + 0.8))
        await until(lambda: rig.agent.eligible_for_motion)
        before = rig.agent.continuity_generation
        first.drop()
        await asyncio.wait_for(lost.wait(), 1)
        await until(lambda: rig.agent._epoch == 2)
        second.inbox.put_nowait(HeartbeatFrame(ROBOT, time.monotonic() + 0.8))
        await until(lambda: rig.agent.eligible_for_motion)
        assert rig.agent.continuity_generation > before
    finally:
        await finish(task, rig)


async def test_agent_release_is_bounded_for_cancellation_resistant_work(rig: Rig) -> None:
    entered, release = asyncio.Event(), asyncio.Event()

    async def stubborn() -> None:
        entered.set()
        while not release.is_set():
            try:
                await release.wait()
            except asyncio.CancelledError:
                pass

    task = asyncio.create_task(stubborn())
    rig.agent._inflight.add(task)
    await entered.wait()
    try:
        await asyncio.wait_for(rig.agent._release(), timeout=1.5)
        assert rig.agent.runtime_failed
        assert not task.done()
    finally:
        release.set()
        await asyncio.gather(task, return_exceptions=True)


async def test_agent_release_is_bounded_for_stubborn_cleanup(rig: Rig) -> None:
    release = asyncio.Event()

    async def stubborn_cleanup() -> None:
        while not release.is_set():
            try:
                await release.wait()
            except asyncio.CancelledError:
                pass

    rig.agent._cleanup = stubborn_cleanup
    task = asyncio.create_task(rig.agent._release())
    try:
        done, _ = await asyncio.wait({task}, timeout=1.5)
        assert task in done
        assert rig.agent.runtime_failed
    finally:
        release.set()
        await task


async def test_eligibility_gap_during_heartbeat_ack_invalidates_generation(rig: Rig) -> None:
    from senior_control.agent.connection import HeartbeatFrame, HeartbeatLedger

    class FakeClock:
        now = 0.0

        def monotonic(self) -> float:
            return self.now

    clock = FakeClock()
    rig.agent._clock = clock
    rig.agent._ledger = HeartbeatLedger(clock, 1.0)
    link = FakeLink(1)
    runner = await start(rig, link)
    gate, entered = asyncio.Event(), asyncio.Event()
    calls = [0]

    async def delayed(epoch: int, deadline: float) -> None:
        calls[0] += 1
        if calls[0] == 2:
            entered.set()
            await gate.wait()

    rig.motion.server_heartbeat = delayed  # type: ignore[method-assign]
    try:
        link.inbox.put_nowait(HeartbeatFrame(ROBOT, 0.8))
        await until(lambda: rig.agent.eligible_for_motion)
        link.inbox.put_nowait(HeartbeatFrame(ROBOT, 1.5))
        await asyncio.wait_for(entered.wait(), 1)
        clock.now = 0.81
        assert not rig.agent.eligible_for_motion
        gate.set()
        await asyncio.sleep(0.05)
        assert rig.agent._continuity_generation > 0
        assert not rig.agent.eligible_for_motion
    finally:
        gate.set()
        await finish(runner, rig)


async def test_background_fault_cannot_be_undone_by_late_heartbeat_ack(rig: Rig) -> None:
    from senior_control.agent.connection import HeartbeatFrame

    fail, entered, release = asyncio.Event(), asyncio.Event(), asyncio.Event()

    async def failing() -> None:
        await fail.wait()
        raise RuntimeError("background failure")

    async def delayed(epoch: int, deadline: float) -> None:
        entered.set()
        try:
            await release.wait()
        except asyncio.CancelledError:
            await release.wait()

    rig.agent._background["task-loop"] = failing
    rig.motion.server_heartbeat = delayed  # type: ignore[method-assign]
    link = FakeLink(1)
    task = await start(rig, link)
    try:
        link.inbox.put_nowait(HeartbeatFrame(ROBOT, time.monotonic() + 0.8))
        await asyncio.wait_for(entered.wait(), 1)
        fail.set()
        await until(lambda: rig.agent._stopping)
        release.set()
        await asyncio.wait_for(task, 2)
        assert not rig.agent.eligible_for_motion
        assert "disable:1" in rig.motion.events
    finally:
        release.set()
        if not task.done():
            rig.agent.request_shutdown()
            await asyncio.wait_for(task, 2)


async def test_control_acquire_ignores_browser_clock(rig: Rig) -> None:
    link = FakeLink(epoch=1)
    task = await start(rig, link)
    try:
        request = simple("control.acquire", seq=1)
        request.expires_at = 1.0
        rig.agent._admit(request)
    finally:
        await finish(task, rig)


async def test_deadline_is_bound_at_receipt_before_async_dispatch(rig: Rig) -> None:
    link = FakeLink(epoch=1)
    task = await start(rig, link)
    try:
        rig.agent.note_heartbeat(ROBOT)
        original = rig.agent._clock.monotonic()
        link.local_command_deadline = lambda deadline: original - 0.01  # type: ignore[attr-defined]
        command = velocity()
        command.payload.challenge_id = "issued-token"
        command.payload.server_deadline = time.time() + 0.2
        await rig.agent._receive_command(link, command)
        await until(lambda: any(r.request_id == command.request_id for r in link.responses))
        reply = next(r for r in link.responses if r.request_id == command.request_id)
        assert not reply.payload.ok
        assert not any(event.startswith("drive:") for event in rig.motion.events)
    finally:
        await finish(task, rig)


async def test_recovery_channel_keeps_authenticated_heartbeat_without_motion(rig: Rig) -> None:
    link = FakeLink(epoch=1)
    task = await start(rig, link)
    try:
        rig.agent.recovery_required = True
        before = link.heartbeats
        await rig.agent._beat()
        assert link.heartbeats == before + 1
        assert not rig.agent.eligible_for_motion
    finally:
        await finish(task, rig)


async def test_pending_safety_intent_starts_restricted_status_channel(rig: Rig) -> None:
    await rig.journal.safety.mark()
    link = FakeLink(epoch=1)
    task = await start(rig, link)
    try:
        assert rig.agent.recovery_required
        await rig.agent._beat()
        assert link.heartbeats > 0
        assert not rig.agent.eligible_for_motion
        assert await rig.journal.safety.pending()
    finally:
        await finish(task, rig)


async def test_production_velocity_uses_cloud_deadline_without_clock_bypass(rig: Rig) -> None:
    link = FakeLink(epoch=1)
    rig.agent._source_clock_verified = False
    task = await start(rig, link)
    try:
        rig.agent.note_heartbeat(ROBOT)
        link.local_command_deadline = lambda deadline: time.monotonic() + 0.15  # type: ignore[attr-defined]
        command = velocity()
        command.payload.challenge_id = "issued-token"
        command.payload.server_deadline = time.time() + 0.2
        await rig.agent._receive_command(link, command)
        await until(lambda: any(r.request_id == command.request_id for r in link.responses))
        reply = next(r for r in link.responses if r.request_id == command.request_id)
        assert reply.payload.ok
        assert any(event.startswith("drive:") for event in rig.motion.events)
        assert (
            rig.agent.metrics.series()[("agent_operations_total", "drive.velocity", "succeeded")]
            == 1
        )
    finally:
        await finish(task, rig)


async def test_clock_unqualified_heartbeat_keeps_status_without_motion(rig: Rig) -> None:
    from senior_control.agent.connection import HeartbeatFrame

    class ClockLink(FakeLink):
        clock_valid = True

        def command_clock_valid(self) -> bool:
            return self.clock_valid

    link = ClockLink(epoch=1)
    task = await start(rig, link)
    try:
        link.inbox.put_nowait(HeartbeatFrame(ROBOT, time.monotonic() + 0.7))
        await until(lambda: rig.agent.eligible_for_motion)
        link.clock_valid = False
        link.inbox.put_nowait(HeartbeatFrame(ROBOT, time.monotonic() + 0.7))
        await until(lambda: "disable:1" in rig.motion.events)
        assert not rig.agent.eligible_for_motion
        assert rig.agent.connected
        assert not link.closed
        assert link.snapshots
        assert "exit" not in rig.motion.events
    finally:
        await finish(task, rig)


async def test_recovery_after_five_failed_attempts_preserves_ros_adapter(rig: Rig) -> None:
    from senior_control.agent.connection import HeartbeatFrame

    links = [FakeLink(epoch) for epoch in range(1, 8)]
    for bad in links[1:6]:
        bad.drop()
    for link in links:
        rig.connector.add(link)

    task = asyncio.create_task(rig.agent.run())
    try:
        await until(lambda: rig.agent.connected)
        links[0].inbox.put_nowait(HeartbeatFrame(ROBOT, time.monotonic() + 0.7))
        await until(lambda: rig.agent.eligible_for_motion)
        links[0].drop()
        await until(lambda: all(bad.closed for bad in links[:6]))
        assert not rig.agent.eligible_for_motion
        assert "exit" not in rig.motion.events
        await until(lambda: rig.agent._epoch == 7)
        links[6].inbox.put_nowait(HeartbeatFrame(ROBOT, time.monotonic() + 0.7))
        await until(lambda: rig.agent.eligible_for_motion)
        assert "heartbeat:7" in rig.motion.events
        assert "exit" not in rig.motion.events
    finally:
        await finish(task, rig)


async def test_unconfigured_video_offer_does_not_close_the_agent_link(rig: Rig) -> None:
    from senior_control.protocol import SdpPayload, VideoEnvelope, VideoSdpEnvelope

    class VideoLink(FakeLink):
        video: asyncio.Queue[VideoEnvelope]

        def __init__(self) -> None:
            super().__init__(epoch=1)
            self.video = asyncio.Queue()

        async def send_video(self, envelope: VideoEnvelope) -> None:
            await self.video.put(envelope)

    link = VideoLink()
    task = await start(rig, link)
    try:
        link.inbox.put_nowait(
            VideoSdpEnvelope(
                v=1,
                type="video.offer",
                robot_id=ROBOT,
                session_id="viewer",
                request_id="offer",
                seq=1,
                connection_epoch=1,
                expires_at=time.time() + 5,
                payload=SdpPayload(sdp="v=0"),
            )
        )
        result = await asyncio.wait_for(link.video.get(), 0.5)
        assert result.type == "video.close"
        assert result.session_id == "viewer"
        assert rig.agent.connected
    finally:
        await finish(task, rig)


async def test_navigation_readiness_roundtrip_and_stale_cache(rig: Rig) -> None:
    task = await start(rig, FakeLink(epoch=1))
    try:
        current = await rig.agent.publish_snapshot(ROBOT)

        def frame(snapshot: RobotSnapshot, *, stale: bool = False) -> SnapshotEnvelope:
            return snapshot_envelope(
                snapshot,
                robot_id=ROBOT,
                session_id="session-1",
                connection_epoch=1,
                request_id="snap-ready",
                seq=1,
                expires_at=9e9,
                stale=stale,
            )

        legacy = frame(current)
        assert "navigation_ready" not in legacy.model_dump()["payload"]
        relayed_legacy = SnapshotEnvelope.model_validate_json(legacy.model_dump_json())
        assert snapshot_from_envelope(relayed_legacy).navigation_ready is None
        for ready in (False, True):
            rig.state.navigation_ready = ready
            live = await rig.agent.publish_snapshot(ROBOT)
            wire = frame(live)
            assert wire.model_dump()["payload"]["navigation_ready"] is ready
            decoded = SnapshotEnvelope.model_validate_json(wire.model_dump_json())
            relayed = snapshot_from_envelope(decoded)
            assert relayed.navigation_ready is ready
            assert frame(relayed, stale=True).model_dump()["payload"]["navigation_ready"] is False

        invalid = frame(live).model_dump()
        for value in ("true", 1, None):
            invalid["payload"]["navigation_ready"] = value
            with pytest.raises(ValidationError):
                SnapshotEnvelope.model_validate(invalid)
    finally:
        await finish(task, rig)


@pytest.fixture
async def snapshot_polls(tmp_path, monkeypatch, request):
    import importlib
    from types import SimpleNamespace

    from senior_control.protocol import RobotPosePayload, ScanPointPayload

    main = importlib.import_module("senior_control.agent.main")
    actual_asyncio = asyncio
    now = [0.0]
    sleeps = asyncio.Queue()
    initial_failure = getattr(request, "param", None)

    class GeometryState(FakeState):
        def __init__(self):
            super().__init__()
            self.pose = RobotPosePayload(x=0.0, y=0.0, yaw=0.0)
            self.points = [ScanPointPayload(x=1.0, y=2.0)]
            self.reason = None
            self.generation = "live-generation"
            self.hold = None
            self.entered = asyncio.Event()
            self.initial_failed = False

        async def collect(self):
            if initial_failure == "collect" and not self.initial_failed:
                self.initial_failed = True
                raise OSError("initial source unavailable")
            value = replace(
                await super().collect(),
                robot_pose=self.pose,
                scan_points=tuple(self.points),
                navigation_block_reason=self.reason,
                map_generation=self.generation,
            )
            if self.hold is not None:
                hold, self.hold = self.hold, None
                self.entered.set()
                await hold.wait()
            return value

    class TimedLink(FakeLink):
        def __init__(self, epoch):
            super().__init__(epoch)
            self.sent_at = []
            self.initial_failed = False

        async def send_snapshot(self, snapshot):
            assert not self.closed
            if initial_failure == "send" and not self.initial_failed:
                self.initial_failed = True
                raise OSError("initial send unavailable")
            self.sent_at.append(now[0])
            await super().send_snapshot(snapshot)

    class ControlledAsyncio:
        def __getattr__(self, name):
            return getattr(actual_asyncio, name)

        def get_running_loop(self):
            return SimpleNamespace(time=lambda: now[0])

        async def sleep(self, delay):
            assert delay == 0.25
            future = actual_asyncio.get_running_loop().create_future()
            sleeps.put_nowait(future)
            await future
            now[0] += delay

    state, link = GeometryState(), TimedLink(1)
    agent = await idle_agent(tmp_path, link, state)
    agent._link, agent._epoch = link, link.epoch
    monkeypatch.setattr(main, "asyncio", ControlledAsyncio())
    task = asyncio.create_task(agent._keep_snapshot_fresh(link))
    tasks = [task]
    pending = await asyncio.wait_for(sleeps.get(), 1)

    async def tick():
        nonlocal pending
        pending.set_result(None)
        pending = await asyncio.wait_for(sleeps.get(), 1)

    async def resume():
        pending.set_result(None)

    try:
        yield SimpleNamespace(
            agent=agent,
            link=link,
            state=state,
            task=task,
            tick=tick,
            resume=resume,
            sleeps=sleeps,
            tasks=tasks,
            new_link=TimedLink,
        )
    finally:
        for running in tasks:
            running.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)
        agent._link = None
        await agent._cleanup()


@pytest.mark.parametrize(
    "change", ["pose", "laser", "reason", "pose-none", "laser-clear", "generation"]
)
async def test_perception_change_pushes_at_the_next_existing_poll(snapshot_polls, change):
    from senior_control.protocol import RobotPosePayload

    runtime = snapshot_polls
    assert len(runtime.link.snapshots) == 1
    if change == "pose":
        runtime.state.pose = RobotPosePayload(x=0.1, y=0.0, yaw=0.0)
    elif change == "laser":
        runtime.state.points[0].x = 3.0
    elif change == "reason":
        runtime.state.reason = "sensor evidence unavailable"
    elif change == "pose-none":
        runtime.state.pose = None
    elif change == "laser-clear":
        runtime.state.points.clear()
    else:
        runtime.state.generation = "replacement-generation"
        runtime.state.pose = None
        runtime.state.points.clear()
    await runtime.tick()
    assert len(runtime.link.snapshots) == 2
    assert runtime.link.sent_at == [0.0, 0.25]
    await runtime.tick()
    assert len(runtime.link.snapshots) == 2


async def test_snapshot_change_key_does_not_alias_mutable_producer_geometry(snapshot_polls):
    runtime = snapshot_polls
    runtime.state.pose.x = 0.1
    await runtime.tick()
    assert len(runtime.link.snapshots) == 2
    runtime.state.pose.x = 0.2
    await runtime.tick()
    assert len(runtime.link.snapshots) == 3
    runtime.state.points[0].y = 4.0
    await runtime.tick()
    assert len(runtime.link.snapshots) == 4
    runtime.state.points[0].y = 5.0
    await runtime.tick()
    assert len(runtime.link.snapshots) == 5


async def test_unchanged_geometry_keeps_the_existing_two_second_keepalive(snapshot_polls):
    runtime = snapshot_polls
    for _ in range(7):
        await runtime.tick()
        assert len(runtime.link.snapshots) == 1
    await runtime.tick()
    assert runtime.link.sent_at == [0.0, 2.0]
    for _ in range(7):
        await runtime.tick()
    assert len(runtime.link.snapshots) == 2
    await runtime.tick()
    assert runtime.link.sent_at == [0.0, 2.0, 4.0]


async def test_rapid_geometry_changes_coalesce_to_one_snapshot_per_existing_poll(snapshot_polls):
    runtime = snapshot_polls
    for step in range(1, 9):
        before = len(runtime.link.snapshots)
        for value in range(100):
            runtime.state.pose.x = step + value / 1000
            runtime.state.points[0].x = value / 1000
        assert len(runtime.link.snapshots) == before
        await runtime.tick()
        assert len(runtime.link.snapshots) == before + 1
        assert runtime.link.snapshots[-1].robot_pose.x == step + 0.099
    assert runtime.link.sent_at == [index * 0.25 for index in range(9)]
    assert runtime.state.calls == 9  # one initial collect and exactly one per existing poll


async def test_retired_epoch_collect_cannot_send_or_revive_old_geometry(snapshot_polls):
    from senior_control.protocol import RobotPosePayload

    runtime = snapshot_polls
    runtime.state.hold = asyncio.Event()
    release = runtime.state.hold
    await runtime.resume()
    await asyncio.wait_for(runtime.state.entered.wait(), 1)
    await runtime.link.close()
    replacement = runtime.new_link(2)
    runtime.agent._link, runtime.agent._epoch = replacement, replacement.epoch
    runtime.state.pose = RobotPosePayload(x=9.0, y=0.0, yaw=0.0)
    release.set()
    await asyncio.wait_for(runtime.task, 1)
    assert len(runtime.link.snapshots) == 1
    assert replacement.snapshots == []
    fresh = asyncio.create_task(runtime.agent._keep_snapshot_fresh(replacement))
    runtime.tasks.append(fresh)
    await asyncio.wait_for(runtime.sleeps.get(), 1)
    assert len(replacement.snapshots) == 1
    assert replacement.snapshots[0].robot_pose.x == 9.0
    assert len(runtime.link.snapshots) == 1


@pytest.mark.parametrize("snapshot_polls", ["collect", "send"], indirect=True)
async def test_initial_snapshot_failure_retries_at_first_successful_poll(snapshot_polls):
    runtime = snapshot_polls
    assert runtime.link.snapshots == []
    await runtime.tick()
    assert len(runtime.link.snapshots) == 1
    assert runtime.link.sent_at == [0.25]
    await runtime.tick()
    assert len(runtime.link.snapshots) == 1
