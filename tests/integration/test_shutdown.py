"""Task 14 shutdown ordering and the local ROS IPC.

Interface assumed by these tests:

* ``IpcServer(path, handler)`` and ``IpcClient(path)`` speak length-prefixed JSON frames
  with ``"v": 1`` over a Unix socket. No TCP port is ever bound.
* ``IpcMotion(client)`` is the agent's motion port; ``disable`` waits for the ROS side's
  acknowledgement and ``exit`` asks the ROS side to leave.
* ``Agent.request_shutdown()`` is synchronous and revokes motion immediately; the
  SIGTERM handler installed by ``Agent.install_signal_handlers()`` calls it.
"""

from __future__ import annotations

import asyncio
import os
import shutil
import signal
import struct
import tempfile
import time
from collections.abc import AsyncIterator, Iterator
from pathlib import Path
from typing import Any

import pytest
from agent_fakes import (
    ROBOT,
    FakeConnector,
    FakeHome,
    FakeLink,
    FakeMaps,
    FakeMotion,
    FakeState,
    FakeTasks,
    stop,
    until,
)

from senior_control.agent import Agent
from senior_control.agent.connection import IpcMotion
from senior_control.agent.journal import AgentJournal
from senior_control.runtime.ipc import MAX_FRAME, IpcClient, IpcError, IpcServer
from senior_control.storage.database import Database


@pytest.fixture
def socket_path() -> Iterator[Path]:
    # AF_UNIX paths are short; pytest's tmp_path can exceed the limit.
    directory = Path(tempfile.mkdtemp(prefix="sr", dir="/tmp"))
    yield directory / "a.sock"
    shutil.rmtree(directory, ignore_errors=True)


class RosSide:
    """Stands in for agent_adapter: records what the agent asked of the ROS process."""

    def __init__(self, events: list[str]) -> None:
        self.events = events
        self.messages: list[dict[str, Any]] = []

    async def handle(self, message: dict[str, Any]) -> dict[str, Any]:
        self.messages.append(message)
        kind = message["type"]
        if kind in {"disable_motion", "exit", "stop", "estop"}:
            self.events.append(f"ros:{kind}")
        return {"ok": True}


@pytest.fixture
async def ros(socket_path: Path) -> AsyncIterator[tuple[RosSide, list[str]]]:
    events: list[str] = []
    side = RosSide(events)
    server = IpcServer(socket_path, side.handle)
    await server.start()
    yield side, events
    await server.close()


async def build(
    tmp_path: Path, socket_path: Path, events: list[str], **extra: Any
) -> tuple[Agent, FakeConnector, Database]:
    database = await Database(tmp_path / "robot.db").__aenter__()
    journal = AgentJournal(database, robot_id=ROBOT)
    await journal.prepare()
    connector = FakeConnector()
    client = IpcClient(socket_path)
    agent = Agent(
        ROBOT,
        connector=connector,
        journal=journal,
        tasks=FakeTasks(),
        home=FakeHome(),
        maps=FakeMaps(),
        state=FakeState(),
        motion=IpcMotion(client),
        heartbeat_seconds=0.02,
        retry_delay=0.01,
        **extra,
    )
    return agent, connector, database


async def test_ipc_frames_are_versioned_and_length_limited(socket_path: Path) -> None:
    server = IpcServer(socket_path, RosSide([]).handle)
    await server.start()
    assert oct(os.stat(socket_path).st_mode & 0o777) == "0o600"
    client = IpcClient(socket_path)
    assert (await client.request("ping"))["ok"] is True

    reader, writer = await asyncio.open_unix_connection(str(socket_path))
    writer.write(struct.pack(">I", MAX_FRAME + 1))
    await writer.drain()
    assert await asyncio.wait_for(reader.read(), timeout=1) == b""
    writer.close()

    reader, writer = await asyncio.open_unix_connection(str(socket_path))
    body = b'{"v": 2, "type": "ping"}'
    writer.write(struct.pack(">I", len(body)) + body)
    await writer.drain()
    header = await asyncio.wait_for(reader.readexactly(4), timeout=1)
    reply = await reader.readexactly(struct.unpack(">I", header)[0])
    assert b'"ok": false' in reply or b'"ok":false' in reply
    writer.close()

    with pytest.raises(IpcError):
        await client.request("ping", blob="x" * (MAX_FRAME + 1))
    await client.close()
    await server.close()
    assert not socket_path.exists()


async def test_shutdown_disables_motion_before_ros_exit(
    tmp_path: Path, socket_path: Path, ros: tuple[RosSide, list[str]]
) -> None:
    side, events = ros
    agent, connector, database = await build(tmp_path, socket_path, events)
    async with database:
        connector.add(FakeLink(epoch=1))
        runner = asyncio.ensure_future(agent.run())
        await until(lambda: agent.connected)
        agent.request_shutdown()
        await asyncio.wait_for(runner, timeout=2)
    assert events == ["ros:disable_motion", "ros:exit"]
    assert [m["type"] for m in side.messages if m["type"] != "ping"][-2:] == [
        "disable_motion",
        "exit",
    ]
    assert not agent.motion_allowed


async def test_sigterm_revokes_motion_before_cleanup(
    tmp_path: Path, socket_path: Path, ros: tuple[RosSide, list[str]]
) -> None:
    _, events = ros
    seen: dict[str, Any] = {}

    async def cleanup() -> None:
        seen["allowed_at_cleanup"] = agent.motion_allowed
        seen["events_at_cleanup"] = list(events)

    agent, connector, database = await build(tmp_path, socket_path, events, cleanup=cleanup)
    async with database:
        connector.add(FakeLink(epoch=1))
        agent.install_signal_handlers()
        runner = asyncio.ensure_future(agent.run())
        await until(lambda: agent.connected)
        signal.raise_signal(signal.SIGTERM)
        await asyncio.wait_for(runner, timeout=2)
    assert seen["allowed_at_cleanup"] is False
    assert seen["events_at_cleanup"][:1] == ["ros:disable_motion"]
    assert events[-1] == "ros:exit"


async def test_cancel_propagates_and_releases_resources(
    tmp_path: Path, socket_path: Path, ros: tuple[RosSide, list[str]]
) -> None:
    _, events = ros
    agent, connector, database = await build(tmp_path, socket_path, events)
    link = FakeLink(epoch=1)
    before = set(asyncio.all_tasks())
    async with database:
        connector.add(link)
        runner = asyncio.ensure_future(agent.run())
        await until(lambda: agent.connected)
        runner.cancel()
        with pytest.raises(asyncio.CancelledError):
            await runner
    assert link.closed
    assert events[:1] == ["ros:disable_motion"]
    assert not agent.motion_allowed
    await asyncio.sleep(0.05)
    leaked = [t for t in asyncio.all_tasks() - before if not t.done()]
    leaked = [t for t in leaked if t is not asyncio.current_task()]
    assert not [
        t for t in leaked if "agent" in repr(t.get_coro()) or "supervisor" in repr(t.get_coro())
    ]


async def test_slow_encoder_does_not_delay_stop(tmp_path: Path) -> None:
    """A slow port in a background slot must not sit in front of stop."""
    release = asyncio.Event()
    started = asyncio.Event()

    async def slow_encoder() -> None:
        started.set()
        await release.wait()

    motion = FakeMotion()
    async with Database(tmp_path / "robot.db") as database:
        journal = AgentJournal(database, robot_id=ROBOT)
        await journal.prepare()
        connector = FakeConnector(FakeLink(epoch=1))
        agent = Agent(
            ROBOT,
            connector=connector,
            journal=journal,
            tasks=FakeTasks(),
            home=FakeHome(),
            maps=FakeMaps(),
            state=FakeState(),
            motion=motion,
            background={"encoder": slow_encoder},
            heartbeat_seconds=0.02,
            retry_delay=0.01,
        )
        runner = asyncio.ensure_future(agent.run())
        await until(lambda: agent.connected and started.is_set())
        began = time.monotonic()
        await agent.dispatch(stop())
        assert time.monotonic() - began < 0.1
        assert "stop" in motion.events
        release.set()
        agent.request_shutdown()
        await asyncio.wait_for(runner, timeout=2)


def _load_adapter() -> Any:
    import importlib.util
    import sys

    path = Path(__file__).parents[2] / "ros2/src/senior_robot/senior_robot/agent_adapter.py"
    spec = importlib.util.spec_from_file_location("agent_adapter_under_test", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    assert "rclpy" not in sys.modules
    return module


async def test_adapter_refuses_motion_after_disable_and_never_imports_rclpy_at_load() -> None:
    from senior_control.models import Velocity

    published: list[Velocity] = []
    exits: list[bool] = []
    core = _load_adapter().AdapterCore(
        robot_id=ROBOT,
        publish=published.append,
        submit=lambda envelope: None,
        request_exit=lambda: exits.append(True),
    )
    assert (await core.handle({"type": "drive", "vx": 0.2, "wz": 0.0}))["ok"] is True
    assert (await core.handle({"type": "disable_motion"}))["ok"] is True
    refused = await core.handle({"type": "drive", "vx": 0.2, "wz": 0.0})
    assert refused == {"ok": False, "error": "motion_disabled"}
    assert published[-1] == Velocity(0.0, 0.0) and Velocity(0.2, 0.0) not in published[2:]
    await core.handle({"type": "exit"})
    assert exits == [True]
