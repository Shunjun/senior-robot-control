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
from contextlib import asynccontextmanager
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
    try:
        await journal.prepare()
    except BaseException:
        await database.__aexit__(None, None, None)
        raise
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


@asynccontextmanager
async def closing_database(database: Database) -> AsyncIterator[None]:
    try:
        yield
    finally:
        await database.__aexit__(None, None, None)


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
    async with closing_database(database):
        connector.add(FakeLink(epoch=1))
        runner = asyncio.ensure_future(agent.run())
        await until(lambda: agent.connected)
        agent.request_shutdown()
        await asyncio.wait_for(runner, timeout=2)
    assert events == ["ros:disable_motion", "ros:disable_motion", "ros:exit"]
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
    async with closing_database(database):
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
    async with closing_database(database):
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
        try:
            await until(lambda: agent.connected and started.is_set())
            began = time.monotonic()
            await agent.dispatch(stop())
            assert time.monotonic() - began < 0.1
            assert "stop:req-stop" in motion.events
        finally:
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
            submit=lambda envelope, received_at=None: self.arbiter.accept(
                envelope, self.now, received_at=received_at
            ),
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
    assert refused["ok"] is False
    assert rig.tick() == (0.0, 0.0)


async def test_old_epoch_stop_and_estop_do_not_drop_the_new_lease() -> None:
    """An old page must not tear down the generation that is driving, or look successful."""
    from agent_fakes import simple, velocity

    rig = ArbiterRig()
    await rig.send("command", simple("control.acquire", seq=1, epoch=2))
    await rig.send("drive", velocity(epoch=2, seq=2))
    assert rig.tick() == (0.2, 0.0)
    stop = await rig.send("stop", simple("drive.stop", seq=1, epoch=1))
    estop = await rig.send("estop", simple("safety.estop", seq=2, epoch=1))
    assert stop["ok"] is False
    assert estop["ok"] is False
    assert rig.tick() == (0.2, 0.0)
    assert rig.arbiter.mode(rig.now) == "manual"


async def test_accepted_estop_locks_without_releasing_the_lease() -> None:
    """A real estop latches. A rejected stop during that latch must not drop the lease."""
    from agent_fakes import simple, velocity

    rig = ArbiterRig()
    await rig.send("command", simple("control.acquire", seq=1, epoch=2))
    await rig.send("drive", velocity(epoch=2, seq=2))
    estop = await rig.send("estop", simple("safety.estop", seq=3, epoch=2))
    assert estop["ok"] is True
    assert rig.arbiter.mode(rig.now) == "estop"
    assert rig.tick() == (0.0, 0.0)
    refused = await rig.send("stop", simple("drive.stop", seq=4, epoch=2))
    assert refused["ok"] is False
    reset = await rig.send("command", simple("safety.reset", seq=5, epoch=2))
    assert reset["ok"] is True
    assert rig.arbiter.mode(rig.now) != "estop"
    resumed = await rig.send("drive", velocity(epoch=2, seq=6))
    assert resumed["ok"] is True
    assert rig.tick() == (0.2, 0.0)


async def test_rejected_drive_is_not_reported_as_success() -> None:
    from agent_fakes import simple, velocity

    rig = ArbiterRig()
    await rig.send("command", simple("control.acquire", seq=1))
    await rig.send("estop", simple("safety.estop", seq=2))
    refused = await rig.send("drive", velocity(seq=3))
    assert refused["ok"] is False
    assert rig.tick() == (0.0, 0.0)


async def test_adapter_stop_the_arbiter_refuses_still_ends_motion() -> None:
    from agent_fakes import simple, velocity

    rig = ArbiterRig()
    await rig.send("command", simple("control.acquire", seq=5))
    await rig.send("drive", velocity(seq=6))
    assert rig.tick() == (0.2, 0.0)
    # A stale stop is not a success and does not tear down the lease that is driving.
    reply = await rig.send("stop", simple("drive.stop", seq=1))
    assert reply["ok"] is False
    assert rig.tick() == (0.2, 0.0)
    assert rig.arbiter.mode(rig.now) == "manual"


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
    await rig.send("disable_motion", connection_epoch=4)
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


async def test_disabled_adapter_requires_new_epoch_acquire_to_reenable() -> None:
    from agent_fakes import simple, velocity

    rig = ArbiterRig()
    assert (await rig.send("command", simple("control.acquire", seq=1, epoch=1)))["ok"]
    assert (await rig.send("drive", velocity(seq=2, epoch=1)))["ok"]
    await rig.send("disable_motion", connection_epoch=1)
    assert not (await rig.send("command", simple("control.acquire", seq=3, epoch=1)))["ok"]
    assert (await rig.send("command", simple("control.acquire", seq=1, epoch=2)))["ok"]
    assert rig.tick() == (0.0, 0.0)
    assert (await rig.send("drive", velocity(seq=2, epoch=2)))["ok"]
    assert rig.tick() == (0.2, 0.0)


async def test_stale_disable_does_not_disable_new_epoch() -> None:
    from agent_fakes import simple, velocity

    rig = ArbiterRig()
    assert (await rig.send("command", simple("control.acquire", seq=1, epoch=1)))["ok"]
    assert (await rig.send("command", simple("control.acquire", seq=1, epoch=2)))["ok"]
    assert (await rig.send("drive", velocity(seq=2, epoch=2)))["ok"]
    await rig.send("disable_motion", connection_epoch=1)
    assert rig.tick() == (0.2, 0.0)
    assert (await rig.send("drive", velocity(seq=3, epoch=2)))["ok"]


async def test_rejected_epoch_does_not_block_valid_reenable() -> None:
    from agent_fakes import simple, velocity

    rig = ArbiterRig()
    assert (await rig.send("command", simple("control.acquire", seq=1, epoch=1)))["ok"]
    assert not (await rig.send("drive", velocity(seq=1, epoch=99)))["ok"]
    await rig.send("disable_motion", connection_epoch=1)
    assert (await rig.send("command", simple("control.acquire", seq=1, epoch=2)))["ok"]


async def test_exit_permanently_rejects_later_acquire() -> None:
    from agent_fakes import simple

    rig = ArbiterRig()
    assert (await rig.send("command", simple("control.acquire", seq=1, epoch=1)))["ok"]
    await rig.send("exit")
    assert not (await rig.send("command", simple("control.acquire", seq=1, epoch=2)))["ok"]


async def test_disable_before_acquire_blocks_same_epoch_until_new_handshake() -> None:
    from agent_fakes import simple, velocity

    rig = ArbiterRig()
    assert (await rig.send("disable_motion", connection_epoch=2))["ok"]
    assert (await rig.send("disable_motion", connection_epoch=2))["ok"]
    assert not (await rig.send("command", simple("control.acquire", seq=1, epoch=2)))["ok"]
    assert not (await rig.send("drive", velocity(seq=2, epoch=2)))["ok"]
    assert (await rig.send("command", simple("control.acquire", seq=1, epoch=3)))["ok"]
    assert rig.tick() == (0.0, 0.0)


async def test_newer_disable_revokes_older_lease_and_rejects_its_own_epoch() -> None:
    from agent_fakes import simple, velocity

    rig = ArbiterRig()
    assert (await rig.send("command", simple("control.acquire", seq=1, epoch=1)))["ok"]
    assert (await rig.send("drive", velocity(seq=2, epoch=1)))["ok"]
    assert rig.tick() == (0.2, 0.0)
    assert (await rig.send("disable_motion", connection_epoch=2))["ok"]
    assert rig.tick() == (0.0, 0.0)
    assert (await rig.send("disable_motion", connection_epoch=1))["ok"]
    assert not (await rig.send("command", simple("control.acquire", seq=1, epoch=2)))["ok"]
    assert (await rig.send("command", simple("control.acquire", seq=1, epoch=3)))["ok"]
    assert (await rig.send("drive", velocity(seq=2, epoch=3)))["ok"]
    assert rig.tick() == (0.2, 0.0)


async def test_adapter_server_heartbeat_is_epoch_bound() -> None:
    rig = ArbiterRig()
    rig.arbiter = __import__("senior_control.safety.arbiter", fromlist=["Arbiter"]).Arbiter(
        ROBOT, 0.2, 1.0, 0.3, require_server_heartbeat=True
    )
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
    assert (await rig.send("server_heartbeat", connection_epoch=1, deadline=100.8))["ok"]
    assert not (await rig.send("server_heartbeat", connection_epoch=0, deadline=100.8))["ok"]


async def test_cloud_heartbeat_drives_ros_eligibility_and_half_open_stops(
    tmp_path: Path, socket_path: Path
) -> None:
    from agent_fakes import simple, velocity

    from senior_control.agent.websocket import WebSocketLink
    from senior_control.cloud.limits import CloudLimits
    from senior_control.cloud.relay import Hub
    from senior_control.models import Velocity
    from senior_control.safety.arbiter import Arbiter

    arbiter = Arbiter(ROBOT, 0.1, 1.0, 1.0, require_server_heartbeat=True)
    adapter = _load_adapter().AdapterCore(
        robot_id=ROBOT,
        submit=lambda envelope, received_at=None: arbiter.accept(
            envelope, time.monotonic(), received_at=received_at
        ),
        disconnect=arbiter.disconnect,
        request_exit=lambda: None,
        server_heartbeat=lambda epoch, deadline: arbiter.server_heartbeat(
            epoch, time.monotonic(), deadline
        ),
    )
    server = IpcServer(socket_path, adapter.handle)
    await server.start()
    hub = Hub(CloudLimits())
    connection = hub.attach_agent(ROBOT)

    class Wire:
        dropping = False
        closed = False

        async def send(self, data: str) -> None:
            if not self.dropping:
                hub.handle_agent_frame(connection, data.encode())

        async def recv(self) -> str:
            frame = await connection.outbox.get()
            if isinstance(frame, int):
                raise ConnectionError("closed")
            return frame.decode()

        async def close(self) -> None:
            self.closed = True
            connection.outbox.close(1000)

    wire = Wire()
    link = WebSocketLink(wire, ROBOT, connection.epoch, heartbeat_timeout=0.4)
    async with Database(tmp_path / "robot.db") as database:
        journal = AgentJournal(database, robot_id=ROBOT)
        await journal.prepare()

        async def connect() -> WebSocketLink:
            return link

        agent = Agent(
            ROBOT,
            connector=connect,
            journal=journal,
            tasks=FakeTasks(),
            home=FakeHome(),
            maps=FakeMaps(),
            state=FakeState(),
            motion=IpcMotion(IpcClient(socket_path)),
            heartbeat_seconds=0.05,
            retry_delay=1.0,
            source_clock_verified=True,
        )
        runner = asyncio.create_task(agent.run())
        try:
            await until(lambda: agent.eligible_for_motion)
            await agent.dispatch(simple("control.acquire", seq=1))
            await agent.dispatch(velocity(seq=2))
            assert arbiter.tick(time.monotonic()).vx > 0
            wire.dropping = True
            await until(lambda: arbiter.tick(time.monotonic()) == Velocity(0.0, 0.0), timeout=2)
            await until(lambda: not agent.eligible_for_motion, timeout=2)
        finally:
            agent.request_shutdown()
            await asyncio.wait_for(runner, 2)
    await server.close()


async def test_adapter_navigation_reports_unavailable_without_ros_consumer() -> None:
    from agent_fakes import simple

    rig = ArbiterRig()
    rig.core = _load_adapter().AdapterCore(
        robot_id=ROBOT,
        submit=lambda envelope, received_at=None: rig.arbiter.accept(
            envelope, rig.now, received_at=received_at
        ),
        disconnect=rig.arbiter.disconnect,
        request_exit=lambda: None,
        server_heartbeat=lambda epoch, deadline: True,
    )
    assert (await rig.send("server_heartbeat", connection_epoch=1, deadline=100.8))["ok"]
    assert (await rig.send("command", simple("control.acquire", seq=1)))["ok"]
    result = await rig.send("command", simple("navigation.start", seq=2))
    assert result == {"ok": False, "error": "unavailable"}
    assert rig.arbiter.mode(rig.now) == "idle"


async def test_background_failure_revokes_ros_heartbeat_and_stops_agent(
    tmp_path: Path, socket_path: Path
) -> None:
    from senior_control.agent.connection import HeartbeatFrame
    from senior_control.models import Velocity
    from senior_control.safety.arbiter import Arbiter

    arbiter = Arbiter(ROBOT, 0.1, 1.0, 1.0, require_server_heartbeat=True)
    adapter = _load_adapter().AdapterCore(
        robot_id=ROBOT,
        submit=lambda envelope, received_at=None: arbiter.accept(
            envelope, time.monotonic(), received_at=received_at
        ),
        disconnect=arbiter.disconnect,
        request_exit=lambda: None,
        server_heartbeat=lambda epoch, deadline: arbiter.server_heartbeat(
            epoch, time.monotonic(), deadline
        ),
    )
    server = IpcServer(socket_path, adapter.handle)
    await server.start()
    trigger = asyncio.Event()

    async def background() -> None:
        await trigger.wait()
        raise RuntimeError("database failure")

    link = FakeLink(epoch=1)
    async with Database(tmp_path / "robot.db") as database:
        journal = AgentJournal(database, robot_id=ROBOT)
        await journal.prepare()
        agent = Agent(
            ROBOT,
            connector=FakeConnector(link),
            journal=journal,
            tasks=FakeTasks(),
            home=FakeHome(),
            maps=FakeMaps(),
            state=FakeState(),
            motion=IpcMotion(IpcClient(socket_path)),
            heartbeat_seconds=0.02,
            background={"tasks": background},
            source_clock_verified=True,
        )
        runner = asyncio.create_task(agent.run())
        try:
            await until(lambda: agent.connected)
            link.inbox.put_nowait(HeartbeatFrame(ROBOT, time.monotonic() + 0.8))
            await until(lambda: agent.eligible_for_motion)
            trigger.set()
            await asyncio.wait_for(asyncio.shield(runner), 1.5)
            assert adapter.motion_disabled
            assert arbiter.tick(time.monotonic()) == Velocity(0.0, 0.0)
        finally:
            agent.request_shutdown()
            if not runner.done():
                await asyncio.wait_for(runner, 2)
    await server.close()


async def test_buffered_stop_then_estop_latches_estop(tmp_path: Path, socket_path: Path) -> None:
    from agent_fakes import simple

    from senior_control.agent.connection import HeartbeatFrame
    from senior_control.safety.arbiter import Arbiter

    arbiter = Arbiter(ROBOT, 0.1, 1.0, 1.0, require_server_heartbeat=True)
    adapter = _load_adapter().AdapterCore(
        robot_id=ROBOT,
        submit=lambda envelope, received_at=None: arbiter.accept(
            envelope, time.monotonic(), received_at=received_at
        ),
        disconnect=arbiter.disconnect,
        request_exit=lambda: None,
        server_heartbeat=lambda epoch, deadline: arbiter.server_heartbeat(
            epoch, time.monotonic(), deadline
        ),
    )
    server = IpcServer(socket_path, adapter.handle)
    await server.start()
    link = FakeLink(epoch=1)
    async with Database(tmp_path / "robot.db") as database:
        journal = AgentJournal(database, robot_id=ROBOT)
        await journal.prepare()
        agent = Agent(
            ROBOT,
            connector=FakeConnector(link),
            journal=journal,
            tasks=FakeTasks(),
            home=FakeHome(),
            maps=FakeMaps(),
            state=FakeState(),
            motion=IpcMotion(IpcClient(socket_path)),
            heartbeat_seconds=0.05,
        )
        runner = asyncio.create_task(agent.run())
        try:
            await until(lambda: agent.connected)
            link.inbox.put_nowait(HeartbeatFrame(ROBOT, time.monotonic() + 0.8))
            await until(lambda: agent.eligible_for_motion)
            await agent.dispatch(simple("control.acquire", seq=1))
            link.inbox.put_nowait(simple("drive.stop", seq=2))
            link.inbox.put_nowait(simple("safety.estop", seq=3))
            await until(lambda: arbiter.mode(time.monotonic()) == "estop", timeout=0.5)
        finally:
            agent.request_shutdown()
            await asyncio.wait_for(runner, 2)
    await server.close()


async def test_disconnect_keeps_buffered_estop_while_stop_ipc_is_blocked(
    tmp_path: Path, socket_path: Path
) -> None:
    from agent_fakes import simple

    from senior_control.agent.connection import HeartbeatFrame
    from senior_control.safety.arbiter import Arbiter

    arbiter = Arbiter(ROBOT, 0.1, 1.0, 1.0, require_server_heartbeat=True)
    core = _load_adapter().AdapterCore(
        robot_id=ROBOT,
        submit=lambda envelope, received_at=None: arbiter.accept(
            envelope, time.monotonic(), received_at=received_at
        ),
        disconnect=arbiter.disconnect,
        request_exit=lambda: None,
        server_heartbeat=lambda epoch, deadline: arbiter.server_heartbeat(
            epoch, time.monotonic(), deadline
        ),
    )
    entered, release = asyncio.Event(), asyncio.Event()

    async def handle(message: dict[str, Any]) -> dict[str, Any]:
        if message["type"] == "stop":
            entered.set()
            await release.wait()
        return await core.handle(message)

    server = IpcServer(socket_path, handle)
    await server.start()
    link = FakeLink(epoch=1)
    async with Database(tmp_path / "robot.db") as database:
        journal = AgentJournal(database, robot_id=ROBOT)
        await journal.prepare()
        agent = Agent(
            ROBOT,
            connector=FakeConnector(link),
            journal=journal,
            tasks=FakeTasks(),
            home=FakeHome(),
            maps=FakeMaps(),
            state=FakeState(),
            motion=IpcMotion(IpcClient(socket_path)),
            heartbeat_seconds=0.05,
        )
        runner = asyncio.create_task(agent.run())
        try:
            await until(lambda: agent.connected)
            link.inbox.put_nowait(HeartbeatFrame(ROBOT, time.monotonic() + 0.8))
            await until(lambda: agent.eligible_for_motion)
            await agent.dispatch(simple("control.acquire", seq=1))
            link.inbox.put_nowait(simple("drive.stop", seq=2))
            await asyncio.wait_for(entered.wait(), 1)
            link.inbox.put_nowait(simple("safety.estop", seq=3))
            link.drop()
            await asyncio.sleep(0.02)
            release.set()
            await until(lambda: arbiter.mode(time.monotonic()) == "estop")
        finally:
            release.set()
            agent.request_shutdown()
            await asyncio.wait_for(runner, 2)
    await server.close()


async def test_background_fault_disables_ros_while_safety_ipc_is_blocked(
    tmp_path: Path, socket_path: Path
) -> None:
    from agent_fakes import simple

    from senior_control.agent.connection import HeartbeatFrame
    from senior_control.safety.arbiter import Arbiter

    arbiter = Arbiter(ROBOT, 0.1, 1.0, 1.0, require_server_heartbeat=True)
    core = _load_adapter().AdapterCore(
        robot_id=ROBOT,
        submit=lambda envelope, received_at=None: arbiter.accept(
            envelope, time.monotonic(), received_at=received_at
        ),
        disconnect=arbiter.disconnect,
        request_exit=lambda: None,
        server_heartbeat=lambda epoch, deadline: arbiter.server_heartbeat(
            epoch, time.monotonic(), deadline
        ),
    )
    entered, release, fail = asyncio.Event(), asyncio.Event(), asyncio.Event()

    async def handle(message: dict[str, Any]) -> dict[str, Any]:
        if message["type"] == "stop":
            entered.set()
            await release.wait()
        return await core.handle(message)

    async def background() -> None:
        await fail.wait()
        raise RuntimeError("background failure")

    server = IpcServer(socket_path, handle)
    await server.start()
    link = FakeLink(1)
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
            background={"tasks": background},
            heartbeat_seconds=0.05,
        )
        runner = asyncio.create_task(agent.run())
        try:
            await until(lambda: agent.connected)
            link.inbox.put_nowait(HeartbeatFrame(ROBOT, time.monotonic() + 0.8))
            await until(lambda: agent.eligible_for_motion)
            await agent.dispatch(simple("control.acquire", seq=1))
            link.inbox.put_nowait(simple("drive.stop", seq=2))
            await asyncio.wait_for(entered.wait(), 1)
            fail.set()
            await until(lambda: core.motion_disabled, timeout=0.3)
        finally:
            release.set()
            agent.request_shutdown()
            await asyncio.wait_for(runner, 3)
    await server.close()


async def test_fault_disables_ros_before_delayed_heartbeat_ack_returns(
    tmp_path: Path, socket_path: Path
) -> None:
    from agent_fakes import simple

    from senior_control.agent.connection import HeartbeatFrame
    from senior_control.safety.arbiter import Arbiter

    arbiter = Arbiter(ROBOT, 0.1, 1.0, 1.0, require_server_heartbeat=True)
    core = _load_adapter().AdapterCore(
        robot_id=ROBOT,
        submit=lambda envelope, received_at=None: arbiter.accept(
            envelope, time.monotonic(), received_at=received_at
        ),
        disconnect=arbiter.disconnect,
        request_exit=lambda: None,
        server_heartbeat=lambda epoch, deadline: arbiter.server_heartbeat(
            epoch, time.monotonic(), deadline
        ),
    )
    entered, release, fail = asyncio.Event(), asyncio.Event(), asyncio.Event()
    count = [0]

    async def handle(message: dict[str, Any]) -> dict[str, Any]:
        result = await core.handle(message)
        if message["type"] == "server_heartbeat":
            count[0] += 1
            if count[0] == 2:
                entered.set()
                await release.wait()
        return result

    async def background() -> None:
        await fail.wait()
        raise RuntimeError("database failure")

    server = IpcServer(socket_path, handle)
    await server.start()
    link = FakeLink(1)
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
            background={"tasks": background},
            heartbeat_seconds=0.05,
        )
        runner = asyncio.create_task(agent.run())
        try:
            await until(lambda: agent.connected)
            link.inbox.put_nowait(HeartbeatFrame(ROBOT, time.monotonic() + 0.8))
            await until(lambda: agent.eligible_for_motion)
            await agent.dispatch(simple("control.acquire", seq=1))
            link.inbox.put_nowait(HeartbeatFrame(ROBOT, time.monotonic() + 0.8))
            await asyncio.wait_for(entered.wait(), 1)
            fail.set()
            await until(lambda: core.motion_disabled, timeout=0.3)
            release.set()
            await asyncio.wait_for(runner, 2)
            assert not agent.eligible_for_motion
            assert not (
                await core.handle(
                    {
                        "type": "server_heartbeat",
                        "connection_epoch": 1,
                        "deadline": time.monotonic() + 0.5,
                    }
                )
            )["ok"]
        finally:
            release.set()
            agent.request_shutdown()
            if not runner.done():
                await asyncio.wait_for(runner, 2)
    await server.close()


@pytest.mark.parametrize("second_session", ["page-a", "page-b"])
@pytest.mark.parametrize("first_inflight", [False, True])
@pytest.mark.parametrize("fill", [False, True])
async def test_newer_or_other_session_estop_is_not_swallowed(
    tmp_path: Path, socket_path: Path, second_session: str, first_inflight: bool, fill: bool
) -> None:
    from agent_fakes import simple

    from senior_control.agent.main import MAX_SAFETY_PENDING
    from senior_control.safety.arbiter import Arbiter

    rig = ArbiterRig()
    rig.arbiter = Arbiter(ROBOT, 0.2, 1.0, 1.0, require_server_heartbeat=True)
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
    assert (await rig.send("server_heartbeat", connection_epoch=1, deadline=100.8))["ok"]
    assert (
        await rig.send(
            "command", simple("control.acquire", seq=5).model_copy(update={"session_id": "page-a"})
        )
    )["ok"]
    entered, release = asyncio.Event(), asyncio.Event()

    async def handle(message: dict[str, Any]) -> dict[str, Any]:
        if first_inflight and message["type"] == "estop" and not entered.is_set():
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
            agent = Agent(
                ROBOT,
                connector=FakeConnector(link),
                journal=journal,
                tasks=FakeTasks(),
                home=FakeHome(),
                maps=FakeMaps(),
                state=FakeState(),
                motion=IpcMotion(IpcClient(socket_path)),
            )
            agent._last_epoch = 1
            first = simple("safety.estop", seq=4).model_copy(update={"session_id": "page-a"})
            second = simple(
                "safety.estop",
                seq=6 if second_session == "page-a" else 1,
            ).model_copy(update={"session_id": second_session})
            try:
                agent._accept(link, first)
                if first_inflight:
                    await asyncio.wait_for(entered.wait(), 1)
                agent._accept(link, second)
                if fill:
                    for index in range(MAX_SAFETY_PENDING - len(agent._safety_pending)):
                        agent._accept(
                            link,
                            simple("safety.estop", seq=1).model_copy(
                                update={"session_id": f"other-{index}"}
                            ),
                        )
                    with pytest.raises(ConnectionError, match="safety capacity"):
                        agent._accept(
                            link,
                            simple("safety.estop", seq=1).model_copy(
                                update={"session_id": "overflow"}
                            ),
                        )
                    assert agent.auth_rejected and agent.runtime_failed
                    assert len(agent._safety_pending) == MAX_SAFETY_PENDING
                release.set()
                assert agent._safety_task is not None
                await asyncio.wait_for(agent._safety_task, 1)
                assert rig.arbiter.mode(rig.now) == "estop"
            finally:
                release.set()
                await agent._release()
    finally:
        await server.close()


async def test_adapter_identity_must_match_paired_robot() -> None:
    rig = ArbiterRig()
    assert (await rig.core.handle({"type": "identity", "robot_id": ROBOT}))["ok"]
    assert not (await rig.core.handle({"type": "identity", "robot_id": "other-car"}))["ok"]


async def test_cloud_challenge_to_production_agent_and_real_ipc_arbiter(
    tmp_path: Path, socket_path: Path
) -> None:
    import json

    from agent_fakes import simple

    from senior_control.agent.websocket import WebSocketLink
    from senior_control.cloud.limits import CloudLimits
    from senior_control.cloud.relay import Hub
    from senior_control.protocol import serialize_envelope
    from senior_control.safety.arbiter import Arbiter

    arbiter = Arbiter(ROBOT, 0.1, 1.0, 0.3, require_server_heartbeat=True)
    adapter = _load_adapter().AdapterCore(
        robot_id=ROBOT,
        submit=lambda envelope, received_at=None, local_deadline=None: arbiter.accept(
            envelope, time.monotonic(), received_at=received_at, local_deadline=local_deadline
        ),
        disconnect=arbiter.disconnect,
        request_exit=lambda: None,
        server_heartbeat=lambda epoch, deadline: arbiter.server_heartbeat(
            epoch, time.monotonic(), deadline
        ),
        require_source_deadline=True,
    )
    server = IpcServer(socket_path, adapter.handle)
    await server.start()
    hub = Hub(CloudLimits())
    connection = hub.attach_agent(ROBOT)
    controller = hub.attach_controller(ROBOT, "authenticated")
    assert controller is not None

    class Wire:
        async def send(self, data: str) -> None:
            hub.handle_agent_frame(connection, data.encode())

        async def recv(self) -> str:
            data = await connection.outbox.get()
            if isinstance(data, int):
                raise ConnectionError("closed")
            return data.decode()

        async def close(self) -> None:
            connection.outbox.close(1000)

    async def response(request_id: str) -> dict[str, Any]:
        while True:
            frame = await asyncio.wait_for(controller.outbox.get(), 2.0)
            assert isinstance(frame, bytes)
            parsed = json.loads(frame)
            if parsed["type"] == "response" and parsed["request_id"] == request_id:
                return parsed["payload"]

    link = WebSocketLink(Wire(), ROBOT, connection.epoch, heartbeat_timeout=0.4)
    async with Database(tmp_path / "robot.db") as database:
        journal = AgentJournal(database, robot_id=ROBOT)
        await journal.prepare()

        async def connect() -> WebSocketLink:
            return link

        agent = Agent(
            ROBOT,
            connector=connect,
            journal=journal,
            tasks=FakeTasks(),
            home=FakeHome(),
            maps=FakeMaps(),
            state=FakeState(),
            motion=IpcMotion(IpcClient(socket_path)),
            heartbeat_seconds=0.05,
            retry_delay=1.0,
            source_clock_verified=False,
        )
        runner = asyncio.create_task(agent.run())
        try:
            await until(lambda: agent.eligible_for_motion)
            acquire = simple("control.acquire", seq=1, request_id="acquire")
            hub.handle_control_frame(controller, serialize_envelope(acquire))
            assert (await response("acquire"))["ok"]
            challenge = simple("drive.challenge", seq=2, request_id="challenge")
            hub.handle_control_frame(controller, serialize_envelope(challenge))
            grant = await response("challenge")
            assert grant["ok"]
            velocity = json.loads(serialize_envelope(acquire))
            velocity.update(
                type="drive.velocity",
                request_id="velocity",
                seq=3,
                payload={"vx": 0.1, "wz": 0.0, "challenge_id": grant["challenge_id"]},
            )
            hub.handle_control_frame(controller, json.dumps(velocity).encode())
            assert (await response("velocity"))["ok"]
            assert arbiter.tick(time.monotonic()).vx == 0.1
            await until(lambda: arbiter.tick(time.monotonic()).vx == 0.0, timeout=1.0)
        finally:
            agent.request_shutdown()
            await asyncio.wait_for(runner, 2)
    await server.close()
