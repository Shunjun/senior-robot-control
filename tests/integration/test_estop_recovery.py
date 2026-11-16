"""Emergency-stop delivery over local IPC remains independent of epoch retirement."""

import asyncio
from pathlib import Path
from typing import Any

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
from test_shutdown import ArbiterRig
from test_shutdown import socket_path as socket_path

from senior_control.agent import Agent
from senior_control.agent.connection import IpcMotion
from senior_control.agent.journal import AgentJournal
from senior_control.runtime.ipc import IpcClient, IpcServer
from senior_control.storage.database import Database


async def test_received_estop_survives_epoch_invalidation(
    tmp_path: Path, socket_path: Path
) -> None:
    """An already received fresh estop must latch even if the epoch is disabled in transit."""
    from agent_fakes import simple

    from senior_control.safety.arbiter import Arbiter

    rig = ArbiterRig()
    rig.arbiter = Arbiter(ROBOT, 300)
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
                disable_client=IpcClient(socket_path),
                intent_client=IpcClient(socket_path),
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
                heartbeat_ms=50,
            )
            runner = asyncio.create_task(agent.run())
            control = IpcClient(socket_path)
            try:
                await until(lambda: agent.connected)
                await agent.dispatch(simple("control.acquire", seq=1))
                link.inbox.put_nowait(simple("safety.estop", seq=2))
                await asyncio.wait_for(entered.wait(), 1)
                # Explicit retirement can arrive while admitted emergency IPC waits.
                assert (await control.request("disable_motion", connection_epoch=1))["ok"]
                release.set()
                assert agent._safety_task is not None
                await asyncio.wait_for(agent._safety_task, 1)
                assert rig.arbiter.mode(rig.now) == "idle"
                assert not rig.arbiter.task_navigation_active
            finally:
                release.set()
                agent.request_shutdown()
                await asyncio.wait_for(runner, 3)
                await control.close()
    finally:
        await server.close()
