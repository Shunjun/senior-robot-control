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
from vehicle_manager.agent.local_adapters import IpcMotion
from vehicle_manager.agent.production_bindings import AgentJournal
from vehicle_manager.agent.runtime_bindings import Agent
from vehicle_manager.storage.bindings import Database

from vehicle_core.errors import DomainError
from vehicle_core.ipc import MAX_FRAME, IpcClient, IpcError, IpcServer


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
        motion=IpcMotion(client, intent_client=IpcClient(socket_path)),
        heartbeat_ms=20,
        retry_delay_ms=10,
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
            heartbeat_ms=20,
            retry_delay_ms=10,
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
COMMAND_NODE = (
    Path(__file__).parents[2]
    / "ros2/src/senior_motion_control/senior_motion_control/command_node.py"
)


def _load_adapter() -> Any:
    import importlib.util
    import sys

    spec = importlib.util.spec_from_file_location("agent_adapter_under_test", ADAPTER)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    assert "rclpy" not in sys.modules
    return module


@pytest.mark.parametrize(
    ("callback_seconds", "expected_count"), [(0.0, 16), (0.0025, 2), (0.01, 1)]
)
def test_adapter_batches_ready_callbacks_with_a_count_and_time_budget(
    callback_seconds: float, expected_count: int
) -> None:
    from copy import deepcopy

    adapter = _load_adapter()
    samples = [
        {"stamp": index, "received": index + 0.01, "metadata": {"source": "original"}}
        for index in range(32)
    ]
    original = deepcopy(samples)

    class Executor:
        now = 0.0

        def __init__(self) -> None:
            self.processed: list[dict[str, Any]] = []

        def spin_once(self, *, timeout_sec: float) -> None:
            assert timeout_sec == 0.0
            self.processed.append(samples[len(self.processed)])
            self.now += callback_seconds

    executor = Executor()
    adapter._spin_ready_callbacks(executor, clock=lambda: executor.now)
    assert len(executor.processed) == expected_count
    assert all(sample is samples[index] for index, sample in enumerate(executor.processed))
    assert samples == original


def test_adapter_idle_callback_polling_stays_bounded_when_spin_returns_none() -> None:
    adapter = _load_adapter()

    class IdleExecutor:
        calls = 0

        def spin_once(self, *, timeout_sec: float) -> None:
            assert timeout_sec == 0.0
            self.calls += 1

    executor = IdleExecutor()
    adapter._spin_ready_callbacks(executor, clock=lambda: 0.0)
    assert executor.calls == 16


class ArbiterRig:
    """Real estop arbiter and IPC; manual replies are fixtures for storage tests.

    Physical manual authority/sequence/TTL is covered with CommandNode in
    test_manual_wire. This rig never executes velocity commands.
    """

    def __init__(self):
        from senior_motion_control.arbiter import Arbiter

        self.now = 100.0
        self.arbiter = Arbiter(ROBOT, 300)
        from senior_motion_control.state_machine import MotionStateMachine

        self.machine = MotionStateMachine()
        self.events = 0
        self.exits = []
        self.epoch = 0
        self.core = _load_adapter().AdapterCore(
            robot_id=ROBOT,
            submit=self.submit,
            disconnect=self.disconnect,
            request_exit=lambda: self.exits.append(True),
            manual=lambda message: {
                "ok": True,
                "authority_id": "storage-test",
                "state": "prepared",
            },
        )

    def submit(self, envelope, received_at=None):
        from senior_motion_control.contracts import MotionEvent, MotionEventKind

        if envelope.type == "safety.estop":
            self.events += 1
            self.machine.reduce(
                MotionEvent(MotionEventKind.SOFTWARE_ESTOP, envelope.session_id, 0, self.events),
                now_steady=self.now,
            )
        return self.arbiter.accept(envelope, self.now)

    def disconnect(self, epoch):
        pass  # Connection retirement does not clear a durable emergency stop.

    async def send(self, kind, envelope=None, **fields):
        from senior_contracts.wire import serialize_envelope

        message = {"v": 1, "type": kind, **fields}
        if envelope is not None:
            message["envelope"] = serialize_envelope(envelope).decode()
        return await self.core.handle(message)


def test_adapter_has_no_raw_velocity_write() -> None:
    adapter = ADAPTER.read_text()
    assert "publish_velocity" not in adapter and "Twist" not in adapter
    assert "publish_velocity" not in COMMAND_NODE.read_text()


async def test_background_failure_keeps_agent_reporting(tmp_path: Path, socket_path: Path) -> None:
    from vehicle_manager.agent.connection import HeartbeatFrame

    local = ArbiterRig()
    local.now = time.monotonic()
    _arbiter, adapter = local.arbiter, local.core
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
            motion=IpcMotion(IpcClient(socket_path), intent_client=IpcClient(socket_path)),
            heartbeat_ms=20,
            background={"tasks": background},
        )
        runner = asyncio.create_task(agent.run())
        try:
            await until(lambda: agent.connected)
            link.inbox.put_nowait(HeartbeatFrame(ROBOT))
            await until(lambda: link.inbox.empty())
            trigger.set()
            await until(lambda: "tasks" in agent.module_faults)
            beats = link.heartbeats
            await until(lambda: link.heartbeats > beats)
            assert not runner.done() and not link.closed
            assert not agent.auth_rejected
        finally:
            agent.request_shutdown()
            if not runner.done():
                await asyncio.wait_for(runner, 2)
    await server.close()


async def test_buffered_stop_then_estop_latches_estop(tmp_path: Path, socket_path: Path) -> None:
    from agent_fakes import simple
    from vehicle_manager.agent.connection import HeartbeatFrame

    local = ArbiterRig()
    local.now = time.monotonic()
    adapter = local.core
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
            motion=IpcMotion(IpcClient(socket_path), intent_client=IpcClient(socket_path)),
            heartbeat_ms=50,
        )
        runner = asyncio.create_task(agent.run())
        try:
            await until(lambda: agent.connected)
            link.inbox.put_nowait(HeartbeatFrame(ROBOT))
            await until(lambda: link.inbox.empty())
            await agent.dispatch(simple("control.acquire", seq=1))
            link.inbox.put_nowait(simple("drive.stop", seq=2))
            link.inbox.put_nowait(simple("safety.estop", seq=3))
            await until(lambda: local.machine.state.mode.value == "estop", timeout=0.5)
        finally:
            agent.request_shutdown()
            await asyncio.wait_for(runner, 2)
    await server.close()


async def test_disconnect_keeps_buffered_estop_while_stop_ipc_is_blocked(
    tmp_path: Path, socket_path: Path
) -> None:
    from agent_fakes import simple
    from vehicle_manager.agent.connection import HeartbeatFrame

    local = ArbiterRig()
    local.now = time.monotonic()
    core = local.core
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
            motion=IpcMotion(IpcClient(socket_path), intent_client=IpcClient(socket_path)),
            heartbeat_ms=50,
        )
        runner = asyncio.create_task(agent.run())
        try:
            await until(lambda: agent.connected)
            link.inbox.put_nowait(HeartbeatFrame(ROBOT))
            await until(lambda: link.inbox.empty())
            await agent.dispatch(simple("control.acquire", seq=1))
            link.inbox.put_nowait(simple("drive.stop", seq=2))
            await asyncio.wait_for(entered.wait(), 1)
            link.inbox.put_nowait(simple("safety.estop", seq=3))
            link.drop()
            await asyncio.sleep(0.02)
            release.set()
            await until(lambda: local.machine.state.mode.value == "estop")
        finally:
            release.set()
            agent.request_shutdown()
            await asyncio.wait_for(runner, 2)
    await server.close()


async def test_background_fault_preserves_cloud_while_safety_ipc_is_blocked(
    tmp_path: Path, socket_path: Path
) -> None:
    from agent_fakes import simple
    from vehicle_manager.agent.connection import HeartbeatFrame

    local = ArbiterRig()
    local.now = time.monotonic()
    _arbiter, core = local.arbiter, local.core
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
            background={"tasks": background},
            heartbeat_ms=50,
        )
        runner = asyncio.create_task(agent.run())
        try:
            await until(lambda: agent.connected)
            link.inbox.put_nowait(HeartbeatFrame(ROBOT))
            await until(lambda: link.inbox.empty())
            await agent.dispatch(simple("control.acquire", seq=1))
            link.inbox.put_nowait(simple("drive.stop", seq=2))
            await asyncio.wait_for(entered.wait(), 1)
            fail.set()
            await until(lambda: "tasks" in agent.module_faults)
            assert not runner.done() and not link.closed
            release.set()
            await until(lambda: bool(link.responses))
        finally:
            release.set()
            agent.request_shutdown()
            await asyncio.wait_for(runner, 3)
    await server.close()


@pytest.mark.parametrize("second_session", ["page-a", "page-b"])
@pytest.mark.parametrize("first_inflight", [False, True])
@pytest.mark.parametrize("fill", [False, True])
async def test_newer_or_other_session_estop_is_not_swallowed(
    tmp_path: Path, socket_path: Path, second_session: str, first_inflight: bool, fill: bool
) -> None:
    from agent_fakes import simple
    from senior_motion_control.arbiter import Arbiter
    from vehicle_manager.agent.main import MAX_SAFETY_PENDING

    rig = ArbiterRig()
    rig.arbiter = Arbiter(ROBOT, 1000)
    rig.core = _load_adapter().AdapterCore(
        robot_id=ROBOT,
        submit=rig.submit,
        disconnect=rig.disconnect,
        request_exit=lambda: None,
        manual=lambda message: {"ok": True, "authority_id": "storage-test", "state": "prepared"},
    )
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
                motion=IpcMotion(IpcClient(socket_path), intent_client=IpcClient(socket_path)),
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
                                update={
                                    "session_id": f"other-{index}",
                                    "request_id": f"other-{index}",
                                }
                            ),
                        )
                    with pytest.raises(DomainError, match="safety capacity"):
                        agent._accept(
                            link,
                            simple("safety.estop", seq=1).model_copy(
                                update={"session_id": "overflow", "request_id": "overflow"}
                            ),
                        )
                    assert not agent.auth_rejected and not agent._stop.is_set()
                    assert len(agent._safety_pending) == MAX_SAFETY_PENDING
                release.set()
                assert agent._safety_task is not None
                await asyncio.wait_for(agent._safety_task, 1)
                assert rig.machine.state.mode.value == "estop"
            finally:
                release.set()
                await agent._release()
    finally:
        await server.close()


async def test_adapter_identity_must_match_paired_robot() -> None:
    rig = ArbiterRig()
    assert (await rig.core.handle({"type": "identity", "robot_id": ROBOT}))["ok"]
    assert not (await rig.core.handle({"type": "identity", "robot_id": "other-car"}))["ok"]


@pytest.mark.parametrize("domain", [None, 73])
async def test_adapter_identity_reports_its_own_runtime_domain_only_for_matching_robot(domain):
    core = _load_adapter().AdapterCore(
        robot_id=ROBOT,
        submit=lambda *args: None,
        disconnect=lambda epoch: None,
        request_exit=lambda: None,
        ros_domain=domain,
    )
    expected = {"ok": True} if domain is None else {"ok": True, "ros_domain": domain}
    assert await core.handle({"type": "identity", "robot_id": ROBOT}) == expected
    assert await core.handle({"type": "identity", "robot_id": "other-robot"}) == {"ok": False}


async def test_capacity_fault_then_immediate_release_delivers_every_accepted_estop(
    tmp_path, socket_path
):
    from agent_fakes import simple
    from vehicle_manager.agent.main import MAX_SAFETY_PENDING

    owner = ArbiterRig()
    server = IpcServer(socket_path, owner.core.handle)
    await server.start()
    agent, _, database = await build(tmp_path, socket_path, [])
    agent._last_epoch = 1
    link = FakeLink(1)
    try:
        for seq in range(1, MAX_SAFETY_PENDING + 1):
            agent._accept(link, simple("safety.estop", seq=seq))
        with pytest.raises(DomainError, match="safety capacity"):
            agent._accept(link, simple("safety.estop", seq=MAX_SAFETY_PENDING + 1))
        assert owner.machine.state.mode.value == "idle"  # No task has been scheduled yet.
        await agent._release()
        assert owner.machine.state.mode.value == "estop"
        assert owner.events == MAX_SAFETY_PENDING
        assert not agent._safety_pending
        assert agent._safety_task.done()
    finally:
        await database.__aexit__(None, None, None)
        await server.close()


async def test_real_transport_failure_reports_unconfirmed_without_success_reply(
    tmp_path, socket_path, caplog
):
    from agent_fakes import simple

    agent, _, database = await build(tmp_path, socket_path, [])
    agent._last_epoch = 1
    link = FakeLink(1)
    agent._link, agent._epoch = link, 1
    try:
        agent._accept(link, simple("safety.estop", seq=1))
        await agent._safety_task
        await until(lambda: bool(link.responses))
        assert not any(response.payload.ok for response in link.responses)
        assert not agent.runtime_failed and not agent.auth_rejected
        assert "safety delivery not confirmed" in caplog.text
        await agent._release()
    finally:
        await database.__aexit__(None, None, None)


async def test_blocked_estop_transport_has_bounded_release_and_explicit_unconfirmed_log(
    tmp_path, socket_path, caplog
):
    from agent_fakes import simple
    from vehicle_manager.agent.dispatcher import Deadlines

    entered, release = asyncio.Event(), asyncio.Event()
    owner = ArbiterRig()

    async def handle(message):
        if message["type"] == "estop":
            entered.set()
            await release.wait()
        return await owner.core.handle(message)

    server = IpcServer(socket_path, handle)
    await server.start()
    agent, _, database = await build(
        tmp_path, socket_path, [], deadlines=Deadlines(safety_ms=300.0)
    )
    agent._last_epoch = 1
    link = FakeLink(1)
    agent._link, agent._epoch = link, 1
    try:
        agent._accept(link, simple("safety.estop", seq=1))
        await asyncio.wait_for(entered.wait(), 1)
        # The in-flight request retained its transport timeout; exercise the
        # shorter shutdown drain/cancel boundary independently.
        agent._deadlines = Deadlines(safety_ms=30.0)
        began = time.monotonic()
        await asyncio.wait_for(agent._release(), 0.5)
        assert time.monotonic() - began < 0.5
        assert owner.machine.state.mode.value == "idle"
        assert not agent.auth_rejected and agent._safety_task.done()
        assert not any(response.payload.ok for response in link.responses)
        assert "vehicle owner state is unconfirmed" in caplog.text
    finally:
        release.set()
        await database.__aexit__(None, None, None)
        await server.close()
