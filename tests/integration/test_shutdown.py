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
        assert "stop:req-stop" in motion.events
        release.set()
        agent.request_shutdown()
        await asyncio.wait_for(runner, timeout=2)


ADAPTER = Path(__file__).parents[2] / "ros2/src/senior_robot/senior_robot/agent_adapter.py"
COMMAND_NODE = Path(__file__).parents[2] / "ros2/src/senior_robot/senior_robot/command_node.py"


def _load_adapter() -> Any:
    import importlib.util
    import sys

    spec = importlib.util.spec_from_file_location("agent_adapter_under_test", ADAPTER)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    assert "rclpy" not in sys.modules
    return module


class ArbiterRig:
    """AdapterCore on the real arbiter, wired as CommandNode wires it: submit and disconnect."""

    def __init__(self) -> None:
        from senior_control.safety.arbiter import Arbiter

        self.now = 100.0
        self.arbiter = Arbiter(ROBOT, 0.2, 1.0, 0.3)
        self.exits: list[bool] = []
        self.core = _load_adapter().AdapterCore(
            robot_id=ROBOT,
            submit=lambda envelope: self.arbiter.accept(envelope, self.now).accepted,
            disconnect=self.arbiter.disconnect,
            request_exit=lambda: self.exits.append(True),
        )

    def tick(self, advance: float = 0.0) -> tuple[float, float]:
        self.now += advance
        velocity_ = self.arbiter.tick(self.now)
        return (velocity_.vx, velocity_.wz)

    async def send(self, kind: str, envelope: Any = None, **fields: Any) -> dict[str, Any]:
        from senior_control.protocol import serialize_envelope

        message: dict[str, Any] = {"v": 1, "type": kind, **fields}
        if envelope is not None:
            message["envelope"] = serialize_envelope(envelope).decode()
        reply: dict[str, Any] = await self.core.handle(message)
        return reply


async def test_adapter_drive_goes_through_the_arbiter_lease() -> None:
    from agent_fakes import simple, velocity

    rig = ArbiterRig()
    assert (await rig.send("command", simple("control.acquire", seq=1)))["ok"] is True
    assert (await rig.send("drive", velocity(seq=2)))["ok"] is True
    assert rig.tick() == (0.2, 0.0)
    # The arbiter's TTL still applies: a raw topic write would have outlived it.
    assert rig.tick(advance=0.5) == (0.0, 0.0)


async def test_adapter_rejects_drive_that_is_not_a_valid_velocity_envelope() -> None:
    from agent_fakes import simple, velocity

    rig = ArbiterRig()
    await rig.send("command", simple("control.acquire", seq=1))
    assert (await rig.send("drive", velocity(seq=2, robot_id="robot-2")))["ok"] is False
    assert (await rig.send("drive", simple("drive.stop", seq=2)))["ok"] is False
    assert (await rig.core.handle({"v": 1, "type": "drive", "envelope": "{not json"}))[
        "ok"
    ] is False
    assert (await rig.core.handle({"v": 1, "type": "drive", "vx": 0.2, "wz": 0.0}))["ok"] is False
    assert rig.tick() == (0.0, 0.0)


async def test_adapter_stop_and_estop_hold_zero_through_the_arbiter() -> None:
    from agent_fakes import simple, velocity

    rig = ArbiterRig()
    await rig.send("command", simple("control.acquire", seq=1))
    await rig.send("drive", velocity(seq=2))
    assert (await rig.send("stop", simple("drive.stop", seq=3)))["ok"] is True
    assert rig.tick() == (0.0, 0.0) and rig.tick(advance=0.1) == (0.0, 0.0)
    await rig.send("drive", velocity(seq=4))
    assert (await rig.send("estop", simple("safety.estop", seq=5)))["ok"] is True
    assert rig.arbiter.mode(rig.now) == "estop"
    assert rig.tick() == (0.0, 0.0)
    refused = await rig.send("drive", velocity(seq=6))
    assert refused == {"ok": True, "accepted": False}
    assert rig.tick() == (0.0, 0.0)


async def test_adapter_stop_the_arbiter_refuses_still_ends_motion() -> None:
    from agent_fakes import simple, velocity

    rig = ArbiterRig()
    await rig.send("command", simple("control.acquire", seq=5))
    await rig.send("drive", velocity(seq=6))
    assert rig.tick() == (0.2, 0.0)
    # Stale seq: the arbiter refuses it, so the adapter must still bring motion to zero.
    reply = await rig.send("stop", simple("drive.stop", seq=1))
    assert reply["ok"] is True and reply["accepted"] is False
    assert rig.tick() == (0.0, 0.0)
    assert rig.arbiter.mode(rig.now) == "idle"


async def test_adapter_disable_motion_zeroes_through_the_arbiter_and_refuses_drive() -> None:
    from agent_fakes import simple, velocity

    rig = ArbiterRig()
    await rig.send("command", simple("control.acquire", seq=1))
    await rig.send("drive", velocity(seq=2))
    assert (await rig.send("disable_motion", connection_epoch=1))["ok"] is True
    assert rig.tick() == (0.0, 0.0)
    assert rig.arbiter.mode(rig.now) == "idle"
    assert await rig.send("drive", velocity(seq=3)) == {"ok": False, "error": "motion_disabled"}
    assert rig.tick() == (0.0, 0.0)


async def test_adapter_exit_and_release_zero_through_the_arbiter() -> None:
    from agent_fakes import simple, velocity

    rig = ArbiterRig()
    await rig.send("command", simple("control.acquire", seq=1, epoch=4))
    await rig.send("drive", velocity(seq=2, epoch=4))
    assert rig.tick() == (0.2, 0.0)
    # disable_motion named another epoch; the adapter still zeroes the epoch it has seen.
    await rig.send("disable_motion", connection_epoch=0)
    assert rig.tick() == (0.0, 0.0)

    other = ArbiterRig()
    await other.send("command", simple("control.acquire", seq=1))
    await other.send("drive", velocity(seq=2))
    assert (await other.send("exit"))["ok"] is True and other.exits == [True]
    assert other.tick() == (0.0, 0.0)

    last = ArbiterRig()
    await last.send("command", simple("control.acquire", seq=1))
    await last.send("drive", velocity(seq=2))
    last.core.release()
    assert last.tick() == (0.0, 0.0)


def test_adapter_has_no_raw_velocity_write() -> None:
    adapter = ADAPTER.read_text()
    assert "publish_velocity" not in adapter and "Twist" not in adapter
    assert "publish_velocity" not in COMMAND_NODE.read_text()


async def test_ipc_motion_sends_validated_envelopes_and_the_epoch(
    socket_path: Path, ros: tuple[RosSide, list[str]]
) -> None:
    from agent_fakes import simple, velocity

    from senior_control.protocol import validate_envelope

    side, _ = ros
    motion = IpcMotion(IpcClient(socket_path))
    await motion.drive(velocity(seq=4))
    await motion.stop(simple("drive.stop", seq=5))
    await motion.estop(simple("safety.estop", seq=6))
    await motion.disable(9)
    await motion.close()
    kinds = [message["type"] for message in side.messages]
    assert kinds == ["drive", "stop", "estop", "disable_motion"]
    drive = validate_envelope(side.messages[0]["envelope"].encode(), ROBOT)
    assert drive.type == "drive.velocity" and drive.seq == 4
    assert validate_envelope(side.messages[1]["envelope"].encode(), ROBOT).type == "drive.stop"
    assert side.messages[3]["connection_epoch"] == 9
