"""Durable emergency-stop admission over the real local IPC and arbiter."""

import asyncio
import time
from pathlib import Path
from typing import Any

import pytest
from agent_fakes import (
    ROBOT,
    FakeConnector,
    FakeHome,
    FakeLink,
    FakeMaps,
    FakeState,
    FakeTasks,
    until,
)
from test_shutdown import ArbiterRig, RosSide, _load_adapter
from test_shutdown import socket_path as socket_path

from senior_control.agent import Agent
from senior_control.agent.connection import IpcMotion
from senior_control.agent.journal import AgentJournal
from senior_control.runtime.ipc import IpcClient, IpcServer
from senior_control.storage.database import Database


async def assert_status_only_recovery(agent: Agent) -> None:
    runner = asyncio.create_task(agent.run())
    try:
        await until(lambda: agent.connected)
        assert agent.recovery_required and not agent.eligible_for_motion
    finally:
        agent.request_shutdown()
        await asyncio.wait_for(runner, 3)


@pytest.mark.parametrize("invalidate", ["expired_heartbeat", "disable"])
async def test_received_estop_survives_epoch_invalidation(
    tmp_path: Path, socket_path: Path, invalidate: str
) -> None:
    """An already received fresh estop must latch even if the epoch is disabled in transit."""
    from agent_fakes import simple

    from senior_control.agent.connection import HeartbeatFrame
    from senior_control.safety.arbiter import Arbiter

    rig = ArbiterRig()
    rig.arbiter = Arbiter(ROBOT, 0.2, 1.0, 0.3, require_server_heartbeat=True)
    rig.core = _load_adapter().AdapterCore(
        robot_id=ROBOT,
        submit=lambda envelope, received_at=None: rig.arbiter.accept(
            envelope, rig.now, received_at=received_at
        ),
        disconnect=rig.arbiter.disconnect,
        request_exit=lambda: None,
        server_heartbeat=lambda epoch, deadline: rig.arbiter.server_heartbeat(
            epoch, rig.now, deadline
        ),
    )
    entered, release = asyncio.Event(), asyncio.Event()

    async def handle(message: dict[str, Any]) -> dict[str, Any]:
        if message["type"] == "estop":
            entered.set()
            await release.wait()
        return await rig.core.handle(message)

    class Clock:
        def monotonic(self) -> float:
            return rig.now

    server = IpcServer(socket_path, handle)
    await server.start()
    link = FakeLink(1)
    try:
        async with Database(tmp_path / "robot.db") as database:
            journal = AgentJournal(database, robot_id=ROBOT)
            await journal.prepare()
            motion = IpcMotion(
                IpcClient(socket_path),
                heartbeat_client=IpcClient(socket_path),
                disable_client=IpcClient(socket_path),
            )
            agent = Agent(
                ROBOT,
                connector=FakeConnector(link),
                journal=journal,
                tasks=FakeTasks(),
                home=FakeHome(),
                maps=FakeMaps(),
                state=FakeState(),
                motion=motion,
                clock=Clock(),
                heartbeat_seconds=0.05,
            )
            runner = asyncio.create_task(agent.run())
            control = IpcClient(socket_path)
            try:
                await until(lambda: agent.connected)
                link.inbox.put_nowait(HeartbeatFrame(ROBOT, 100.8))
                await until(lambda: agent.eligible_for_motion)
                await agent.dispatch(simple("control.acquire", seq=1))
                link.inbox.put_nowait(simple("safety.estop", seq=2))
                await asyncio.wait_for(entered.wait(), 1)
                if invalidate == "expired_heartbeat":
                    rig.now = 100.9
                    response = await control.request(
                        "server_heartbeat", connection_epoch=1, deadline=101.7
                    )
                    assert not response["ok"]
                else:
                    assert (await control.request("disable_motion", connection_epoch=1))["ok"]
                release.set()
                assert agent._safety_task is not None
                await asyncio.wait_for(agent._safety_task, 1)
                assert rig.arbiter.mode(rig.now) == "estop"
                assert await journal.safety.pending()
            finally:
                release.set()
                agent.request_shutdown()
                await asyncio.wait_for(runner, 3)
                await control.close()
            next_link = FakeLink(2)
            restarted = ipc_agent(journal, FakeConnector(next_link), socket_path, rig)
            await assert_status_only_recovery(restarted)
    finally:
        await server.close()


async def test_unresolved_estop_prevents_agent_restart_after_ipc_timeout(
    tmp_path: Path, socket_path: Path
) -> None:
    """A stopped IPC consumer cannot let a restarted agent authorize a new epoch."""
    from agent_fakes import simple

    from senior_control.agent.connection import HeartbeatFrame
    from senior_control.agent.dispatcher import Deadlines

    entered, release = asyncio.Event(), asyncio.Event()
    side = RosSide([])

    async def handle(message: dict[str, Any]) -> dict[str, Any]:
        if message["type"] == "estop":
            entered.set()
            await release.wait()
        return await side.handle(message)

    server = IpcServer(socket_path, handle)
    await server.start()
    path = tmp_path / "robot.db"
    try:
        async with Database(path) as database:
            journal = AgentJournal(database, robot_id=ROBOT)
            await journal.prepare()
            first_link = FakeLink(1)
            agent = Agent(
                ROBOT,
                connector=FakeConnector(first_link),
                journal=journal,
                tasks=FakeTasks(),
                home=FakeHome(),
                maps=FakeMaps(),
                state=FakeState(),
                motion=IpcMotion(
                    IpcClient(socket_path),
                    heartbeat_client=IpcClient(socket_path),
                    disable_client=IpcClient(socket_path),
                ),
                deadlines=Deadlines(safety=0.1),
                heartbeat_seconds=0.05,
                retry_delay=0.01,
            )
            runner = asyncio.create_task(agent.run())
            try:
                await until(lambda: agent.connected)
                first_link.inbox.put_nowait(HeartbeatFrame(ROBOT, time.monotonic() + 0.8))
                await until(lambda: agent.eligible_for_motion)
                first_link.inbox.put_nowait(simple("safety.estop", seq=1))
                await asyncio.wait_for(entered.wait(), 1)
                await asyncio.wait_for(runner, 2)
            finally:
                release.set()
                agent.request_shutdown()
                if not runner.done():
                    await asyncio.wait_for(runner, 2)

        async with Database(path) as database:
            journal = AgentJournal(database, robot_id=ROBOT)
            await journal.prepare()
            second_link = FakeLink(2)
            restarted = Agent(
                ROBOT,
                connector=FakeConnector(second_link),
                journal=journal,
                tasks=FakeTasks(),
                home=FakeHome(),
                maps=FakeMaps(),
                state=FakeState(),
                motion=IpcMotion(IpcClient(socket_path)),
                heartbeat_seconds=0.05,
                retry_delay=0.01,
            )
            await assert_status_only_recovery(restarted)
    finally:
        release.set()
        await server.close()


def heartbeat_arbiter_rig() -> ArbiterRig:
    from senior_control.safety.arbiter import Arbiter

    rig = ArbiterRig()
    rig.arbiter = Arbiter(ROBOT, 0.2, 1.0, 0.3, require_server_heartbeat=True)
    rig.core = _load_adapter().AdapterCore(
        robot_id=ROBOT,
        submit=lambda envelope, received_at=None: rig.arbiter.accept(
            envelope, rig.now, received_at=received_at
        ),
        disconnect=rig.arbiter.disconnect,
        request_exit=lambda: None,
        server_heartbeat=lambda epoch, deadline: rig.arbiter.server_heartbeat(
            epoch, rig.now, deadline
        ),
    )
    return rig


def ipc_agent(
    journal: AgentJournal,
    connector: FakeConnector,
    socket_path: Path,
    rig: ArbiterRig,
    **extra: Any,
) -> Agent:
    class Clock:
        def monotonic(self) -> float:
            return rig.now

    return Agent(
        ROBOT,
        connector=connector,
        journal=journal,
        tasks=FakeTasks(),
        home=FakeHome(),
        maps=FakeMaps(),
        state=FakeState(),
        motion=IpcMotion(
            IpcClient(socket_path),
            heartbeat_client=IpcClient(socket_path),
            disable_client=IpcClient(socket_path),
        ),
        clock=Clock(),
        heartbeat_seconds=0.05,
        retry_delay=0.01,
        **extra,
    )


async def test_replay_is_resolved_and_old_epoch_cannot_disrupt_new_session(
    tmp_path: Path, socket_path: Path
) -> None:
    from agent_fakes import simple

    from senior_control.agent.connection import HeartbeatFrame
    from senior_control.errors import ErrorCode

    rig = heartbeat_arbiter_rig()
    server = IpcServer(socket_path, rig.core.handle)
    await server.start()
    first, second = FakeLink(1), FakeLink(2)
    try:
        async with Database(tmp_path / "robot.db") as database:
            journal = AgentJournal(database, robot_id=ROBOT)
            await journal.prepare()
            agent = ipc_agent(journal, FakeConnector(first, second), socket_path, rig)
            runner = asyncio.create_task(agent.run())
            try:
                await until(lambda: agent.connected)
                first.inbox.put_nowait(HeartbeatFrame(ROBOT, 100.8))
                await until(lambda: agent.eligible_for_motion)
                await agent.dispatch(simple("control.acquire", seq=5))
                first.inbox.put_nowait(simple("safety.estop", seq=4))
                await until(lambda: bool(first.responses))
                assert first.responses[-1].payload.error is ErrorCode.CONFLICT
                # The conflict response is sent before durable replay cleanup completes.
                assert agent._safety_task is not None
                await asyncio.wait_for(asyncio.shield(agent._safety_task), 2)
                assert not await journal.safety.pending()
                assert not agent.recovery_required
                first.drop()
                await until(lambda: first.closed and agent._link is second)
                second.inbox.put_nowait(HeartbeatFrame(ROBOT, 100.8))
                await until(lambda: agent.eligible_for_motion)
                await agent.dispatch(simple("control.acquire", seq=1, epoch=2))
                second.inbox.put_nowait(simple("safety.estop", seq=6, epoch=1))
                await until(lambda: bool(second.responses))
                assert second.responses[-1].payload.error is ErrorCode.CONFLICT
                assert agent.eligible_for_motion
                assert not await journal.safety.pending()
                assert rig.arbiter.mode(rig.now) == "idle"
            finally:
                agent.request_shutdown()
                await asyncio.wait_for(runner, 3)
    finally:
        await server.close()


@pytest.mark.parametrize(
    ("kind", "overflow"),
    [("safety.estop", False), ("safety.estop", True), ("drive.stop", True)],
)
async def test_burst_of_64_safety_requests_with_blocked_ipc_retains_recovery_obligation(
    tmp_path: Path, socket_path: Path, kind: str, overflow: bool
) -> None:
    from agent_fakes import simple

    from senior_control.agent.connection import HeartbeatFrame
    from senior_control.agent.dispatcher import Deadlines
    from senior_control.agent.main import MAX_SAFETY_PENDING

    rig = heartbeat_arbiter_rig()
    entered, release = asyncio.Event(), asyncio.Event()
    dequeued, allow_dispatch = asyncio.Event(), asyncio.Event()

    async def handle(message: dict[str, Any]) -> dict[str, Any]:
        if message["type"] in {"estop", "stop"}:
            entered.set()
            await release.wait()
        return await rig.core.handle(message)

    server = IpcServer(socket_path, handle)
    await server.start()
    try:
        async with Database(tmp_path / "robot.db") as database:
            journal = AgentJournal(database, robot_id=ROBOT)
            await journal.prepare()
            link = FakeLink(1)
            agent = ipc_agent(
                journal, FakeConnector(link), socket_path, rig, deadlines=Deadlines(safety=0.2)
            )
            original_dispatch = agent.dispatch

            async def gated_dispatch(envelope: Any, **kwargs: Any) -> Any:
                if envelope.type == kind:
                    # Dequeue the first safety request, then fill the pending capacity
                    # before starting its real 200 ms blocked-IPC deadline. Otherwise
                    # this capacity test accidentally times 64 durable SQLite commits.
                    dequeued.set()
                    await allow_dispatch.wait()
                return await original_dispatch(envelope, **kwargs)

            agent.dispatch = gated_dispatch  # type: ignore[method-assign]
            runner = asyncio.create_task(agent.run())
            try:
                await until(lambda: agent.connected)
                link.inbox.put_nowait(HeartbeatFrame(ROBOT, 100.8))
                await until(lambda: agent.eligible_for_motion)
                link.inbox.put_nowait(simple(kind, seq=1))
                await asyncio.wait_for(dequeued.wait(), 1)
                assert not entered.is_set()
                for index in range(MAX_SAFETY_PENDING):
                    link.inbox.put_nowait(
                        simple(kind, seq=1).model_copy(update={"session_id": f"burst-{index}"})
                    )
                await until(lambda: len(agent._safety_pending) == MAX_SAFETY_PENDING)
                allow_dispatch.set()
                await asyncio.wait_for(entered.wait(), 1)
                assert len(agent._safety_pending) == MAX_SAFETY_PENDING
                if overflow:
                    link.inbox.put_nowait(
                        simple(kind, seq=1).model_copy(update={"session_id": "overflow"})
                    )
                else:
                    link.drop()
                await asyncio.wait_for(runner, 3)
                assert agent.recovery_required
                assert await journal.safety.pending()
                assert agent._safety_task is not None and agent._safety_task.done()
            finally:
                allow_dispatch.set()
                release.set()
                agent.request_shutdown()
                if not runner.done():
                    await asyncio.wait_for(runner, 3)
                if agent._safety_task is not None:
                    await asyncio.gather(agent._safety_task, return_exceptions=True)

        async with Database(tmp_path / "robot.db") as database:
            journal = AgentJournal(database, robot_id=ROBOT)
            await journal.prepare()
            next_link = FakeLink(2)
            restarted = ipc_agent(journal, FakeConnector(next_link), socket_path, rig)
            await assert_status_only_recovery(restarted)
    finally:
        await server.close()


async def test_storage_timeout_revokes_motion_and_disables_ros(
    tmp_path: Path, socket_path: Path
) -> None:
    from agent_fakes import simple

    from senior_control.agent.connection import HeartbeatFrame
    from senior_control.agent.dispatcher import Deadlines

    rig = heartbeat_arbiter_rig()
    server = IpcServer(socket_path, rig.core.handle)
    await server.start()
    entered, release = asyncio.Event(), asyncio.Event()
    try:
        async with Database(tmp_path / "robot.db") as database:
            journal = AgentJournal(database, robot_id=ROBOT)
            await journal.prepare()

            async def blocked_write() -> None:
                entered.set()
                await release.wait()

            journal.safety.mark = blocked_write  # type: ignore[method-assign]
            first, second = FakeLink(1), FakeLink(2)
            agent = ipc_agent(
                journal,
                FakeConnector(first, second),
                socket_path,
                rig,
                deadlines=Deadlines(safety=0.1),
            )
            runner = asyncio.create_task(agent.run())
            try:
                await until(lambda: agent.connected)
                first.inbox.put_nowait(HeartbeatFrame(ROBOT, 100.8))
                await until(lambda: agent.eligible_for_motion)
                await agent.dispatch(simple("control.acquire", seq=1))
                first.inbox.put_nowait(simple("safety.estop", seq=2))
                await asyncio.wait_for(entered.wait(), 1)
                assert not agent.eligible_for_motion
                await asyncio.wait_for(runner, 2)
                assert rig.core.motion_disabled
                assert agent.auth_rejected and agent.runtime_failed
                assert not second.closed and agent._last_epoch == 1
                assert not any(response.payload.ok for response in first.responses)
            finally:
                release.set()
                agent.request_shutdown()
                if not runner.done():
                    await asyncio.wait_for(runner, 2)
    finally:
        await server.close()


async def test_estop_before_first_ros_heartbeat_latches_and_stays_online(
    tmp_path: Path, socket_path: Path
) -> None:
    from agent_fakes import simple

    rig = heartbeat_arbiter_rig()
    server = IpcServer(socket_path, rig.core.handle)
    await server.start()
    try:
        async with Database(tmp_path / "robot.db") as database:
            journal = AgentJournal(database, robot_id=ROBOT)
            await journal.prepare()
            first, second = FakeLink(1), FakeLink(2)
            agent = ipc_agent(journal, FakeConnector(first, second), socket_path, rig)
            runner = asyncio.create_task(agent.run())
            try:
                await until(lambda: agent.connected)
                first.inbox.put_nowait(simple("safety.estop", seq=1))
                await until(lambda: agent.recovery_required)
                assert agent.recovery_required
                assert await journal.safety.pending()
                assert rig.arbiter.mode(rig.now) == "estop"
                assert agent.connected and not agent.eligible_for_motion
                assert not second.closed and agent._last_epoch == 1
            finally:
                agent.request_shutdown()
                if not runner.done():
                    await asyncio.wait_for(runner, 2)
    finally:
        await server.close()


async def test_canceled_inflight_estop_keeps_persisted_intent(
    tmp_path: Path, socket_path: Path
) -> None:
    from agent_fakes import simple

    from senior_control.agent.connection import HeartbeatFrame

    rig = heartbeat_arbiter_rig()
    entered, release = asyncio.Event(), asyncio.Event()

    async def handle(message: dict[str, Any]) -> dict[str, Any]:
        if message["type"] == "estop":
            entered.set()
            await release.wait()
        return await rig.core.handle(message)

    server = IpcServer(socket_path, handle)
    await server.start()
    try:
        async with Database(tmp_path / "robot.db") as database:
            journal = AgentJournal(database, robot_id=ROBOT)
            await journal.prepare()
            link = FakeLink(1)
            agent = ipc_agent(journal, FakeConnector(link), socket_path, rig)
            runner = asyncio.create_task(agent.run())
            try:
                await until(lambda: agent.connected)
                link.inbox.put_nowait(HeartbeatFrame(ROBOT, 100.8))
                await until(lambda: agent.eligible_for_motion)
                link.inbox.put_nowait(simple("safety.estop", seq=1))
                await asyncio.wait_for(entered.wait(), 1)
                assert await journal.safety.pending()
                assert agent._safety_task is not None
                agent._safety_task.cancel()
                await asyncio.wait_for(runner, 2)
                assert agent.recovery_required
                assert await journal.safety.pending()
            finally:
                release.set()
                agent.request_shutdown()
                if not runner.done():
                    await asyncio.wait_for(runner, 2)
    finally:
        await server.close()


async def test_replay_clear_racing_new_estop_cannot_erase_its_intent(
    tmp_path: Path, socket_path: Path
) -> None:
    from agent_fakes import simple

    from senior_control.agent.connection import HeartbeatFrame

    rig = heartbeat_arbiter_rig()
    server = IpcServer(socket_path, rig.core.handle)
    await server.start()
    clear_started, release_clear = asyncio.Event(), asyncio.Event()
    try:
        async with Database(tmp_path / "robot.db") as database:
            journal = AgentJournal(database, robot_id=ROBOT)
            await journal.prepare()
            original_clear = journal.safety.clear_rejected

            async def delayed_clear() -> None:
                clear_started.set()
                await release_clear.wait()
                await original_clear()

            journal.safety.clear_rejected = delayed_clear  # type: ignore[method-assign]
            link = FakeLink(1)
            agent = ipc_agent(journal, FakeConnector(link), socket_path, rig)
            runner = asyncio.create_task(agent.run())
            try:
                await until(lambda: agent.connected)
                link.inbox.put_nowait(HeartbeatFrame(ROBOT, 100.8))
                await until(lambda: agent.eligible_for_motion)
                await agent.dispatch(simple("control.acquire", seq=5))
                link.inbox.put_nowait(simple("safety.estop", seq=4))
                await asyncio.wait_for(clear_started.wait(), 1)
                link.inbox.put_nowait(
                    simple("safety.estop", seq=1).model_copy(update={"session_id": "page-b"})
                )
                await until(lambda: link.inbox.empty())
                assert not agent.eligible_for_motion
                release_clear.set()
                await until(lambda: rig.arbiter.mode(rig.now) == "estop")
                assert await journal.safety.pending()
            finally:
                release_clear.set()
                agent.request_shutdown()
                await asyncio.wait_for(runner, 2)
    finally:
        await server.close()


async def test_canceled_replay_clear_cannot_erase_new_estop_obligation(
    tmp_path: Path, socket_path: Path
) -> None:
    from agent_fakes import simple

    from senior_control.agent.connection import HeartbeatFrame

    rig = heartbeat_arbiter_rig()
    server = IpcServer(socket_path, rig.core.handle)
    await server.start()
    cleared, release = asyncio.Event(), asyncio.Event()
    try:
        async with Database(tmp_path / "robot.db") as database:
            journal = AgentJournal(database, robot_id=ROBOT)
            await journal.prepare()
            original_clear = journal.safety.clear_rejected

            async def interrupted_clear() -> None:
                await original_clear()
                cleared.set()
                await release.wait()

            journal.safety.clear_rejected = interrupted_clear  # type: ignore[method-assign]
            link = FakeLink(1)
            agent = ipc_agent(journal, FakeConnector(link), socket_path, rig)
            runner = asyncio.create_task(agent.run())
            try:
                await until(lambda: agent.connected)
                link.inbox.put_nowait(HeartbeatFrame(ROBOT, 100.8))
                await until(lambda: agent.eligible_for_motion)
                await agent.dispatch(simple("control.acquire", seq=5))
                link.inbox.put_nowait(simple("safety.estop", seq=4))
                await asyncio.wait_for(cleared.wait(), 1)
                link.inbox.put_nowait(
                    simple("safety.estop", seq=1).model_copy(update={"session_id": "page-b"})
                )
                await until(lambda: agent._safety_admitting)
                assert agent._safety_task is not None
                agent._safety_task.cancel()
                await asyncio.wait_for(runner, 3)
                assert agent.recovery_required
                assert await journal.safety.pending()
            finally:
                release.set()
                agent.request_shutdown()
                if not runner.done():
                    await asyncio.wait_for(runner, 2)
    finally:
        await server.close()


async def test_replay_clear_lock_counts_toward_new_estop_deadline(
    tmp_path: Path, socket_path: Path
) -> None:
    from agent_fakes import simple

    from senior_control.agent.connection import HeartbeatFrame
    from senior_control.agent.dispatcher import Deadlines

    rig = heartbeat_arbiter_rig()
    disabled, clear_started, release = asyncio.Event(), asyncio.Event(), asyncio.Event()

    async def handle(message: dict[str, Any]) -> dict[str, Any]:
        result = await rig.core.handle(message)
        if message["type"] == "disable_motion":
            disabled.set()
        return result

    server = IpcServer(socket_path, handle)
    await server.start()
    try:
        async with Database(tmp_path / "robot.db") as database:
            journal = AgentJournal(database, robot_id=ROBOT)
            await journal.prepare()
            original_clear, original_mark = journal.safety.clear_rejected, journal.safety.mark
            marks = 0

            async def late_clear() -> None:
                clear_started.set()
                try:
                    await release.wait()
                except asyncio.CancelledError:
                    await release.wait()
                await original_clear()

            async def delayed_mark() -> None:
                nonlocal marks
                marks += 1
                if marks > 1:
                    await release.wait()
                await original_mark()

            journal.safety.clear_rejected = late_clear  # type: ignore[method-assign]
            journal.safety.mark = delayed_mark  # type: ignore[method-assign]
            first, second = FakeLink(1), FakeLink(2)
            agent = ipc_agent(
                journal,
                FakeConnector(first, second),
                socket_path,
                rig,
                deadlines=Deadlines(safety=0.1),
            )
            runner = asyncio.create_task(agent.run())
            try:
                await until(lambda: agent.connected)
                first.inbox.put_nowait(HeartbeatFrame(ROBOT, 100.8))
                await until(lambda: agent.eligible_for_motion)
                await agent.dispatch(simple("control.acquire", seq=5))
                first.inbox.put_nowait(simple("safety.estop", seq=4))
                await asyncio.wait_for(clear_started.wait(), 1)
                new_estop = simple("safety.estop", seq=1, request_id="new-estop").model_copy(
                    update={"session_id": "page-b"}
                )
                started = asyncio.get_running_loop().time()
                first.inbox.put_nowait(new_estop)
                await until(lambda: agent._safety_admitting)
                assert not agent.eligible_for_motion
                await asyncio.wait_for(disabled.wait(), 0.15)
                assert asyncio.get_running_loop().time() - started < 0.15
                assert agent.recovery_required and agent.auth_rejected
                assert not any(
                    r.request_id == "new-estop" and r.payload.ok for r in first.responses
                )
                release.set()
                await asyncio.wait_for(runner, 2)
                if agent._stragglers:
                    await asyncio.wait_for(
                        asyncio.gather(*agent._stragglers, return_exceptions=True), 2
                    )
                assert await journal.safety.pending()
                assert not any(
                    r.request_id == "new-estop" and r.payload.ok for r in first.responses
                )
                restarted = ipc_agent(journal, FakeConnector(second), socket_path, rig)
                await assert_status_only_recovery(restarted)
            finally:
                release.set()
                agent.request_shutdown()
                if not runner.done():
                    await asyncio.wait_for(runner, 3)
                if agent._stragglers:
                    await asyncio.wait_for(
                        asyncio.gather(*agent._stragglers, return_exceptions=True), 2
                    )
    finally:
        await server.close()


async def test_estop_mark_uses_only_time_left_after_replay_clear(
    tmp_path: Path, socket_path: Path
) -> None:
    from agent_fakes import simple

    from senior_control.agent.connection import HeartbeatFrame
    from senior_control.agent.dispatcher import Deadlines

    rig = heartbeat_arbiter_rig()
    disabled, clear_started, release_clear = asyncio.Event(), asyncio.Event(), asyncio.Event()
    mark_started, release_mark = asyncio.Event(), asyncio.Event()

    async def handle(message: dict[str, Any]) -> dict[str, Any]:
        result = await rig.core.handle(message)
        if message["type"] == "disable_motion":
            disabled.set()
        return result

    server = IpcServer(socket_path, handle)
    await server.start()
    try:
        async with Database(tmp_path / "robot.db") as database:
            journal = AgentJournal(database, robot_id=ROBOT)
            await journal.prepare()
            original_clear, original_mark = journal.safety.clear_rejected, journal.safety.mark
            marks = 0

            async def delayed_clear() -> None:
                clear_started.set()
                await release_clear.wait()
                await original_clear()

            async def delayed_mark() -> None:
                nonlocal marks
                marks += 1
                if marks > 1:
                    mark_started.set()
                    await release_mark.wait()
                await original_mark()

            journal.safety.clear_rejected = delayed_clear  # type: ignore[method-assign]
            journal.safety.mark = delayed_mark  # type: ignore[method-assign]
            link = FakeLink(1)
            agent = ipc_agent(
                journal,
                FakeConnector(link),
                socket_path,
                rig,
                deadlines=Deadlines(safety=0.3),
            )
            runner = asyncio.create_task(agent.run())
            try:
                await until(lambda: agent.connected)
                link.inbox.put_nowait(HeartbeatFrame(ROBOT, 100.8))
                await until(lambda: agent.eligible_for_motion)
                await agent.dispatch(simple("control.acquire", seq=5))
                link.inbox.put_nowait(simple("safety.estop", seq=4))
                await asyncio.wait_for(clear_started.wait(), 1)
                link.inbox.put_nowait(
                    simple("safety.estop", seq=1, request_id="new-estop").model_copy(
                        update={"session_id": "page-b"}
                    )
                )
                await until(lambda: agent._safety_admitting)
                await asyncio.sleep(0.18)
                release_clear.set()
                await asyncio.wait_for(mark_started.wait(), 1)
                await asyncio.wait_for(disabled.wait(), 0.24)
                assert agent.recovery_required and agent.auth_rejected
                assert not any(r.request_id == "new-estop" and r.payload.ok for r in link.responses)
            finally:
                release_clear.set()
                release_mark.set()
                agent.request_shutdown()
                if not runner.done():
                    await asyncio.wait_for(runner, 3)
                if agent._stragglers:
                    await asyncio.wait_for(
                        asyncio.gather(*agent._stragglers, return_exceptions=True), 2
                    )
    finally:
        await server.close()


async def test_canceled_new_receipt_does_not_let_old_clear_erase_marker(
    tmp_path: Path, socket_path: Path
) -> None:
    from agent_fakes import simple

    rig = heartbeat_arbiter_rig()
    async with Database(tmp_path / "robot.db") as database:
        journal = AgentJournal(database, robot_id=ROBOT)
        await journal.prepare()
        await journal.safety.mark()
        original_clear = journal.safety.clear_rejected
        clear_started, release_clear = asyncio.Event(), asyncio.Event()

        async def delayed_clear() -> None:
            clear_started.set()
            await release_clear.wait()
            await original_clear()

        journal.safety.clear_rejected = delayed_clear  # type: ignore[method-assign]
        link = FakeLink(1)
        agent = ipc_agent(journal, FakeConnector(link), socket_path, rig)
        clearing = asyncio.create_task(agent._clear_replayed_safety())
        try:
            await asyncio.wait_for(clear_started.wait(), 1)
            receipt = asyncio.create_task(
                agent._receive_command(link, simple("safety.estop", seq=1))
            )
            await until(lambda: agent._safety_admitting)
            receipt.cancel()
            with pytest.raises(asyncio.CancelledError):
                await receipt
            assert agent.recovery_required
            release_clear.set()
            await asyncio.wait_for(clearing, 1)
            assert await journal.safety.pending()
            assert not link.responses
        finally:
            release_clear.set()
            await asyncio.gather(clearing, return_exceptions=True)
            if agent._stragglers:
                await asyncio.wait_for(
                    asyncio.gather(*agent._stragglers, return_exceptions=True), 2
                )


async def test_canceling_replay_cleanup_cannot_cancel_marker_repair(
    tmp_path: Path, socket_path: Path
) -> None:
    from agent_fakes import simple

    rig = heartbeat_arbiter_rig()
    async with Database(tmp_path / "robot.db") as database:
        journal = AgentJournal(database, robot_id=ROBOT)
        await journal.prepare()
        await journal.safety.mark()
        original_clear, original_mark = journal.safety.clear_rejected, journal.safety.mark
        clear_started, release_clear = asyncio.Event(), asyncio.Event()
        repair_started, release_repair = asyncio.Event(), asyncio.Event()

        async def delayed_clear() -> None:
            clear_started.set()
            await release_clear.wait()
            await original_clear()

        async def delayed_repair() -> None:
            repair_started.set()
            await release_repair.wait()
            await original_mark()

        journal.safety.clear_rejected = delayed_clear  # type: ignore[method-assign]
        journal.safety.mark = delayed_repair  # type: ignore[method-assign]
        link = FakeLink(1)
        agent = ipc_agent(journal, FakeConnector(link), socket_path, rig)
        clearing = asyncio.create_task(agent._clear_replayed_safety())
        receipt: asyncio.Task[None] | None = None
        try:
            await asyncio.wait_for(clear_started.wait(), 1)
            receipt = asyncio.create_task(
                agent._receive_command(link, simple("safety.estop", seq=1))
            )
            await until(lambda: agent._safety_admitting)
            release_clear.set()
            await asyncio.wait_for(repair_started.wait(), 1)
            clearing.cancel()
            with pytest.raises(asyncio.CancelledError):
                await clearing
            receipt.cancel()
            await asyncio.gather(receipt, return_exceptions=True)
            release_repair.set()
            if agent._stragglers:
                await asyncio.wait_for(
                    asyncio.gather(*agent._stragglers, return_exceptions=True), 1
                )
            assert agent.recovery_required
            assert await journal.safety.pending()
        finally:
            release_clear.set()
            release_repair.set()
            await asyncio.gather(clearing, return_exceptions=True)
            if receipt is not None:
                await asyncio.gather(receipt, return_exceptions=True)
            if agent._stragglers:
                await asyncio.wait_for(
                    asyncio.gather(*agent._stragglers, return_exceptions=True), 2
                )


async def test_estop_actual_mark_uses_remaining_lock_budget(
    tmp_path: Path, socket_path: Path
) -> None:
    from agent_fakes import simple

    from senior_control.agent.connection import HeartbeatFrame
    from senior_control.agent.dispatcher import Deadlines

    rig = heartbeat_arbiter_rig()
    disabled, mark_started, release_mark = asyncio.Event(), asyncio.Event(), asyncio.Event()

    async def handle(message: dict[str, Any]) -> dict[str, Any]:
        result = await rig.core.handle(message)
        if message["type"] == "disable_motion":
            disabled.set()
        return result

    server = IpcServer(socket_path, handle)
    await server.start()
    try:
        async with Database(tmp_path / "robot.db") as database:
            journal = AgentJournal(database, robot_id=ROBOT)
            await journal.prepare()
            original_mark = journal.safety.mark

            async def delayed_mark() -> None:
                mark_started.set()
                await release_mark.wait()
                await original_mark()

            journal.safety.mark = delayed_mark  # type: ignore[method-assign]
            link = FakeLink(1)
            agent = ipc_agent(
                journal,
                FakeConnector(link),
                socket_path,
                rig,
                deadlines=Deadlines(safety=0.3),
            )
            runner = asyncio.create_task(agent.run())
            held = False
            try:
                await until(lambda: agent.connected)
                link.inbox.put_nowait(HeartbeatFrame(ROBOT, 100.8))
                await until(lambda: agent.eligible_for_motion)
                await agent._safety_lock.acquire()
                held = True
                link.inbox.put_nowait(simple("safety.estop", seq=1, request_id="new-estop"))
                await until(lambda: agent._safety_admitting)
                await asyncio.sleep(0.18)
                agent._safety_lock.release()
                held = False
                await asyncio.wait_for(mark_started.wait(), 1)
                await asyncio.wait_for(disabled.wait(), 0.24)
                assert agent.recovery_required and agent.auth_rejected
                assert not any(r.request_id == "new-estop" and r.payload.ok for r in link.responses)
            finally:
                if held:
                    agent._safety_lock.release()
                release_mark.set()
                agent.request_shutdown()
                if not runner.done():
                    await asyncio.wait_for(runner, 3)
                if agent._stragglers:
                    await asyncio.wait_for(
                        asyncio.gather(*agent._stragglers, return_exceptions=True), 2
                    )
    finally:
        await server.close()


async def test_canceling_estop_as_lock_is_granted_does_not_leak_the_lock(
    tmp_path: Path, socket_path: Path
) -> None:
    from agent_fakes import simple

    rig = heartbeat_arbiter_rig()
    async with Database(tmp_path / "robot.db") as database:
        journal = AgentJournal(database, robot_id=ROBOT)
        await journal.prepare()
        link = FakeLink(1)
        agent = ipc_agent(journal, FakeConnector(link), socket_path, rig)
        await agent._safety_lock.acquire()
        receipt = asyncio.create_task(agent._receive_command(link, simple("safety.estop", seq=1)))
        await until(lambda: agent._safety_admitting)
        await asyncio.sleep(0)
        agent._safety_lock.release()
        await asyncio.sleep(0)
        assert agent._safety_lock.locked() and not receipt.done()
        receipt.cancel()
        with pytest.raises(asyncio.CancelledError):
            await receipt
        await asyncio.wait_for(agent._safety_lock.acquire(), 0.2)
        agent._safety_lock.release()
        assert agent.recovery_required and not link.responses


async def test_repeated_cleanup_cancellation_cannot_let_late_clear_erase_repair(
    tmp_path: Path, socket_path: Path
) -> None:
    from agent_fakes import simple

    rig = heartbeat_arbiter_rig()
    async with Database(tmp_path / "robot.db") as database:
        journal = AgentJournal(database, robot_id=ROBOT)
        await journal.prepare()
        await journal.safety.mark()
        original_clear = journal.safety.clear_rejected
        clear_started, cancel_seen, release_clear, clear_done = (
            asyncio.Event(),
            asyncio.Event(),
            asyncio.Event(),
            asyncio.Event(),
        )

        async def late_clear() -> None:
            clear_started.set()
            try:
                await release_clear.wait()
            except asyncio.CancelledError:
                cancel_seen.set()
                await release_clear.wait()
            await original_clear()
            clear_done.set()

        journal.safety.clear_rejected = late_clear  # type: ignore[method-assign]
        link = FakeLink(1)
        agent = ipc_agent(journal, FakeConnector(link), socket_path, rig)
        clearing = asyncio.create_task(agent._clear_replayed_safety())
        receipt: asyncio.Task[None] | None = None
        try:
            await asyncio.wait_for(clear_started.wait(), 1)
            receipt = asyncio.create_task(
                agent._receive_command(link, simple("safety.estop", seq=1))
            )
            await until(lambda: agent._safety_admitting)
            receipt.cancel()
            await asyncio.gather(receipt, return_exceptions=True)
            clearing.cancel()
            await asyncio.gather(clearing, return_exceptions=True)
            await asyncio.wait_for(cancel_seen.wait(), 1)
            worker = next(iter(agent._safety_cleanups))
            worker.cancel()
            await asyncio.sleep(0.05)
            release_clear.set()
            await asyncio.wait_for(clear_done.wait(), 1)
            await asyncio.gather(worker, return_exceptions=True)
            assert await journal.safety.pending()
            assert agent.recovery_required and not link.responses
        finally:
            release_clear.set()
            await asyncio.gather(clearing, return_exceptions=True)
            if receipt is not None:
                await asyncio.gather(receipt, return_exceptions=True)
            if agent._stragglers:
                await asyncio.wait_for(
                    asyncio.gather(*agent._stragglers, return_exceptions=True), 2
                )


async def test_canceling_cleanup_worker_during_repair_keeps_the_marker(
    tmp_path: Path, socket_path: Path
) -> None:
    from agent_fakes import simple

    rig = heartbeat_arbiter_rig()
    async with Database(tmp_path / "robot.db") as database:
        journal = AgentJournal(database, robot_id=ROBOT)
        await journal.prepare()
        await journal.safety.mark()
        original_clear, original_mark = journal.safety.clear_rejected, journal.safety.mark
        clear_started, release_clear = asyncio.Event(), asyncio.Event()
        repair_started, release_repair = asyncio.Event(), asyncio.Event()

        async def delayed_clear() -> None:
            clear_started.set()
            await release_clear.wait()
            await original_clear()

        async def delayed_repair() -> None:
            repair_started.set()
            await release_repair.wait()
            await original_mark()

        journal.safety.clear_rejected = delayed_clear  # type: ignore[method-assign]
        journal.safety.mark = delayed_repair  # type: ignore[method-assign]
        link = FakeLink(1)
        agent = ipc_agent(journal, FakeConnector(link), socket_path, rig)
        clearing = asyncio.create_task(agent._clear_replayed_safety())
        receipt: asyncio.Task[None] | None = None
        try:
            await asyncio.wait_for(clear_started.wait(), 1)
            receipt = asyncio.create_task(
                agent._receive_command(link, simple("safety.estop", seq=1))
            )
            await until(lambda: agent._safety_admitting)
            receipt.cancel()
            await asyncio.gather(receipt, return_exceptions=True)
            release_clear.set()
            await asyncio.wait_for(repair_started.wait(), 1)
            worker = next(iter(agent._safety_cleanups))
            worker.cancel()
            await asyncio.sleep(0)
            release_repair.set()
            await asyncio.gather(clearing, return_exceptions=True)
            assert agent.recovery_required and await journal.safety.pending()
        finally:
            release_clear.set()
            release_repair.set()
            await asyncio.gather(clearing, return_exceptions=True)
            if receipt is not None:
                await asyncio.gather(receipt, return_exceptions=True)
            if agent._stragglers:
                await asyncio.wait_for(
                    asyncio.gather(*agent._stragglers, return_exceptions=True), 2
                )


async def test_estop_lock_timeout_disables_without_waiting_for_lock_owner(
    tmp_path: Path, socket_path: Path
) -> None:
    from agent_fakes import simple

    from senior_control.agent.connection import HeartbeatFrame
    from senior_control.agent.dispatcher import Deadlines

    rig = heartbeat_arbiter_rig()
    disabled = asyncio.Event()

    async def handle(message: dict[str, Any]) -> dict[str, Any]:
        result = await rig.core.handle(message)
        if message["type"] == "disable_motion":
            disabled.set()
        return result

    server = IpcServer(socket_path, handle)
    await server.start()
    try:
        async with Database(tmp_path / "robot.db") as database:
            journal = AgentJournal(database, robot_id=ROBOT)
            await journal.prepare()
            first, second = FakeLink(1), FakeLink(2)
            agent = ipc_agent(
                journal,
                FakeConnector(first, second),
                socket_path,
                rig,
                deadlines=Deadlines(safety=0.1),
            )
            runner = asyncio.create_task(agent.run())
            held = False
            try:
                await until(lambda: agent.connected)
                first.inbox.put_nowait(HeartbeatFrame(ROBOT, 100.8))
                await until(lambda: agent.eligible_for_motion)
                await journal.safety.mark()
                await agent._safety_lock.acquire()
                held = True
                first.inbox.put_nowait(simple("safety.estop", seq=1, request_id="new-estop"))
                await until(lambda: agent._safety_admitting)
                assert not agent.eligible_for_motion
                await asyncio.wait_for(disabled.wait(), 0.15)
                assert agent.recovery_required and agent.auth_rejected
                assert await journal.safety.pending()
                assert not any(
                    r.request_id == "new-estop" and r.payload.ok for r in first.responses
                )
                agent._safety_lock.release()
                held = False
                await asyncio.wait_for(runner, 2)
                restarted = ipc_agent(journal, FakeConnector(second), socket_path, rig)
                await assert_status_only_recovery(restarted)
            finally:
                if held:
                    agent._safety_lock.release()
                agent.request_shutdown()
                if not runner.done():
                    await asyncio.wait_for(runner, 3)
    finally:
        await server.close()


async def test_failed_latch_write_preserves_agent_intent_for_exact_retry(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    from agent_fakes import FakeMotion, simple

    from senior_control.agent.connection import MotionRejected
    from senior_control.errors import ErrorCode
    from senior_control.protocol import SimpleEnvelope
    from senior_control.safety.arbiter import PersistentEstop

    path = tmp_path / "estop.json"
    PersistentEstop.initialize(path, ROBOT)
    latch = PersistentEstop(path, ROBOT)
    rig = heartbeat_arbiter_rig()
    original_write = latch.estop

    class LocalMotion(FakeMotion):
        async def estop(self, envelope: SimpleEnvelope) -> None:
            result = latch.commit(envelope, rig.arbiter.accept(envelope, rig.now))
            if not result.accepted:
                raise MotionRejected("estop", result.error)

    def fail_write() -> None:
        raise OSError("injected durable write failure")

    async def start(journal: AgentJournal, link: FakeLink) -> tuple[Agent, asyncio.Task[None]]:
        agent = Agent(
            ROBOT,
            connector=FakeConnector(link),
            journal=journal,
            tasks=FakeTasks(),
            home=FakeHome(),
            maps=FakeMaps(),
            state=FakeState(),
            motion=LocalMotion(),
        )
        runner = asyncio.create_task(agent.run())
        await until(lambda: agent.connected)
        return agent, runner

    request = simple("safety.estop", seq=1)
    journal_path = tmp_path / "agent.db"
    try:
        async with Database(journal_path) as database:
            journal = AgentJournal(database, robot_id=ROBOT)
            await journal.prepare()
            monkeypatch.setattr(latch, "estop", fail_write)
            link = FakeLink(1)
            agent, runner = await start(journal, link)
            try:
                link.inbox.put_nowait(request)
                await until(lambda: agent.recovery_required)
                retry = latch.commit(request, rig.arbiter.accept(request, rig.now))
                assert retry.error is ErrorCode.UNAVAILABLE
                assert await journal.safety.pending()
            finally:
                agent.request_shutdown()
                await asyncio.wait_for(runner, 2)

        async with Database(journal_path) as database:
            journal = AgentJournal(database, robot_id=ROBOT)
            assert await journal.safety.pending()
            link = FakeLink(1)
            agent, runner = await start(journal, link)
            try:
                monkeypatch.setattr(latch, "estop", original_write)
                link.inbox.put_nowait(request)
                await until(lambda: any(reply.payload.ok for reply in link.responses))
                assert await journal.safety.pending()
                replay = latch.commit(request, rig.arbiter.accept(request, rig.now))
                assert replay.error is ErrorCode.CONFLICT
            finally:
                agent.request_shutdown()
                await asyncio.wait_for(runner, 2)
    finally:
        latch.close()

    reopened = PersistentEstop(path, ROBOT)
    try:
        assert reopened.latched and reopened.revision == 1
    finally:
        reopened.close()
