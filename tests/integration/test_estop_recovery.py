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
            await asyncio.wait_for(restarted.run(), 2)
            assert restarted.recovery_required and restarted._last_epoch == 0
            assert not next_link.closed
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
            runner = asyncio.create_task(restarted.run())
            try:
                await asyncio.wait_for(asyncio.shield(runner), 1)
                assert not restarted.connected
                assert restarted.auth_rejected or restarted.runtime_failed
            finally:
                restarted.request_shutdown()
                if not runner.done():
                    await asyncio.wait_for(runner, 2)
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
            runner = asyncio.create_task(agent.run())
            try:
                await until(lambda: agent.connected)
                link.inbox.put_nowait(HeartbeatFrame(ROBOT, 100.8))
                await until(lambda: agent.eligible_for_motion)
                link.inbox.put_nowait(simple(kind, seq=1))
                await asyncio.wait_for(entered.wait(), 1)
                for index in range(MAX_SAFETY_PENDING):
                    link.inbox.put_nowait(
                        simple(kind, seq=1).model_copy(update={"session_id": f"burst-{index}"})
                    )
                await until(lambda: len(agent._safety_pending) == MAX_SAFETY_PENDING)
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
            await asyncio.wait_for(restarted.run(), 2)
            assert restarted.recovery_required
            assert not restarted.connected
            assert not next_link.closed
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


async def test_estop_before_first_ros_heartbeat_fails_closed(
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
                await asyncio.wait_for(runner, 2)
                assert agent.recovery_required
                assert await journal.safety.pending()
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
