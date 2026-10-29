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
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

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
    stop,
    until,
    velocity,
)

from senior_control.agent import Agent, RobotSnapshot
from senior_control.agent.dispatcher import Deadlines, StoreTaskPort
from senior_control.agent.journal import AgentJournal
from senior_control.agent.snapshots import SnapshotCache
from senior_control.errors import DomainError, ErrorCode
from senior_control.models import OperationResult, OperationStatus
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
    assert rig.motion.events.count("stop") == 1
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
