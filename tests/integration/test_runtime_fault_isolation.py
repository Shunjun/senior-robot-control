"""Real Agent composition/log service; only OS children and remote sockets are fixtures."""

import ast
import asyncio
import json
import os
import tempfile
from pathlib import Path
from types import SimpleNamespace

import pytest
from agent_fakes import FakeConnector, FakeLink, until
from vehicle_manager.agent import production as composition
from vehicle_manager.agent.pairing import Credentials
from vehicle_manager.agent.production_bindings import AgentLogConnection
from vehicle_manager.agent.runtime_bindings import Agent

from vehicle_core.ipc import IpcClient
from vehicle_core.lifecycle_events import READ_FD, LifecycleEvents, report_ros_exit


class LogSocket:
    def __init__(self):
        self.requests = asyncio.Queue()
        self.replies = asyncio.Queue()

    def __aiter__(self):
        return self

    async def __anext__(self):
        return await self.requests.get()

    async def send(self, raw):
        await self.replies.put(json.loads(raw))


class Child:
    def __init__(self, name, runner=None, shutdown=None):
        self.name, self.runner, self.shutdown = name, runner, shutdown
        self.returncode = None
        self.done = asyncio.Event()
        self.pid = 900001
        self.stopped = False
        if runner:
            runner.add_done_callback(lambda task: self.exit(task.result()))

    async def wait(self):
        await self.done.wait()
        return self.returncode

    def exit(self, code):
        self.returncode = code
        self.done.set()

    def terminate(self):
        self.stopped = True
        if self.shutdown:
            self.shutdown()
        else:
            self.exit(0)

    def send_signal(self, _signum):
        self.terminate()

    def kill(self):
        self.terminate()


@pytest.mark.parametrize(
    ("kind", "failure"),
    [
        (kind, failure)
        for kind in ("real", "simulation")
        for failure in ("spawn", "identity", "exit", "agent_exit")
    ]
    + [("real", "adapter_exit")],
)
async def test_supervise_local_fault_keeps_actual_agent_and_log_transport(
    monkeypatch, kind, failure
):
    import senior_robot.service_runtime as helper
    import vehicle_manager.simulation_runtime as simulation_cli

    with tempfile.TemporaryDirectory(prefix="isolation-") as directory:
        root = Path(directory).resolve()
        link, log_socket = FakeLink(1), LogSocket()
        agents, children = [], {}
        local_ready = asyncio.Event()
        spawn_calls = []

        def installed(agent):
            agents.append(agent)

        async def logs_run(self, url, token):
            await self.serve(log_socket)

        monkeypatch.setattr(Agent, "install_signal_handlers", installed)
        monkeypatch.setattr(composition, "WebSocketConnector", lambda *a: FakeConnector(link))
        monkeypatch.setattr(AgentLogConnection, "run", logs_run)
        credentials = Credentials(robot_id="robot-a", device_token="secret")
        monkeypatch.setattr(helper, "read_credentials", lambda path: credentials)
        monkeypatch.setattr(helper, "_motion_command", lambda *a: ["motion"])
        monkeypatch.setattr(helper, "_adapter_command", lambda *a: ["adapter"])
        monkeypatch.setattr(helper, "_ros_launch_command", lambda *a: ["camera"])
        monkeypatch.setattr(simulation_cli, "_notify", lambda message: None)

        async def group_stop(child):
            if child is not None:
                child.terminate()
                await child.wait()

        monkeypatch.setattr(helper, "_terminate_motion_stack", group_stop)

        async def ready(*args):
            if failure == "identity":
                raise ValueError("foreign adapter identity")
            local_ready.set()
            return None

        monkeypatch.setattr(helper, "_wait_for_adapter", ready)
        monkeypatch.setattr(simulation_cli, "_wait_for_adapter", ready)

        async def spawn(*args, env, **options):
            is_agent = args[0] == "agent" or "vehicle_manager.agent" in args
            name = (
                "agent"
                if is_agent
                else ("ros" if "simulation_motion.launch.py" in args else args[0])
            )
            spawn_calls.append(name)
            if name in {"motion", "ros"} and failure == "spawn":
                raise OSError("fixture ROS spawn failed")
            if is_agent:
                assert not children  # Report plane starts before every local child.
                monkeypatch.setenv(READ_FD, str(os.dup(int(env[READ_FD]))))
                monkeypatch.delenv("SENIOR_VERIFIED_ADAPTER_RUN", raising=False)
                config = composition.AgentConfig(
                    "robot-a",
                    "wss://cloud.test/ws/agent",
                    "secret",
                    root / "agent.db",
                    root / "agent.sock",
                )
                runner = asyncio.create_task(composition.run_agent(config))
                child = Child(name, runner, lambda: agents[0].request_shutdown())
            else:
                assert options["start_new_session"] is True
                child = Child(name)
            children[name] = child
            return child

        monkeypatch.setattr(helper.asyncio, "create_subprocess_exec", spawn)
        env = dict(os.environ)
        if kind == "real":
            profile = SimpleNamespace(ros_domain=17, ros_setup=None)
            runner = asyncio.create_task(helper.supervise(root, ["agent"], env, profile=profile))
        else:
            profile = SimpleNamespace(
                socket=root / "agent.sock",
                credentials=root / "device.json",
                video=False,
                ros_domain=73,
                cloud_url="wss://cloud.test/ws/agent",
                database=root / "agent.db",
                ros_setup=root / "setup.bash",
            )
            runner = asyncio.create_task(simulation_cli.supervise(profile, credentials))
        try:
            await until(lambda: runner.done() or (agents and link.snapshots))
            if runner.done():
                await runner
                pytest.fail("supervisor ended before reporting")
            agent = agents[0]
            if failure == "agent_exit":
                children["agent"].exit(78)
                assert await asyncio.wait_for(runner, 4) == 78
                assert all(c.stopped for n, c in children.items() if n != "agent")
                return
            if failure in {"exit", "adapter_exit"}:
                await local_ready.wait()
                name = (
                    "adapter"
                    if failure == "adapter_exit"
                    else ("motion" if kind == "real" else "ros")
                )
                children[name].exit(9)
                module, phase = name, "exited"
            else:
                module, phase = ("motion" if kind == "real" else "ros"), "unavailable"
            await until(lambda: agent.module_faults.get(module, {}).get("phase") == phase)
            await until(
                lambda: any(
                    (s.vehicle_ui or {}).get("module_faults", {}).get(module, {}).get("phase")
                    == phase
                    for s in link.snapshots
                )
            )
            beats = link.heartbeats
            await until(lambda: link.heartbeats > beats)
            assert agent.connection_epoch == 1 and not link.closed and not runner.done()
            if failure in {"exit", "adapter_exit"}:
                assert children[module].stopped
            # Query through the real log wire/grant/service/store, not a logger mock.
            found = False
            for _ in range(100):
                await log_socket.requests.put(
                    json.dumps(
                        {"client": "a" * 32, "action": "query", "query": {"after": 0, "limit": 100}}
                    )
                )
                reply = await asyncio.wait_for(log_socket.replies.get(), 2)
                assert reply["ok"]
                found = any(
                    f"runtime module {module}: {phase}" in e["summary"]
                    for e in reply["data"]["entries"]
                )
                if found:
                    break
                await asyncio.sleep(0.01)
            assert found
            assert spawn_calls.count("agent") == 1
            assert spawn_calls.count("motion" if kind == "real" else "ros") == 1
            if kind == "real":
                client = IpcClient(root / "service.sock")
                try:
                    assert (await client.request("status"))["running"]
                    assert (await client.request("stop"))["ok"]
                finally:
                    await client.close()
            else:
                # Same callback as SIGTERM without sending a signal to pytest.
                asyncio.get_running_loop()._signal_handlers[15]._run()
            assert await asyncio.wait_for(runner, 4) == 0
            assert children["agent"].stopped
            assert not (root / "service.sock").exists()
        finally:
            if not runner.done():
                runner.cancel()
            await asyncio.gather(runner, return_exceptions=True)
            for agent in agents:
                agent.request_shutdown()
            await asyncio.gather(
                *(c.runner for c in children.values() if c.runner), return_exceptions=True
            )


@pytest.mark.parametrize("code", [0, 7])
def test_launch_exit_only_reports_lifecycle(monkeypatch, code):
    events = LifecycleEvents()
    try:
        monkeypatch.setenv("SENIOR_RUNTIME_EVENTS_WRITE_FD", str(events.writer))
        assert report_ros_exit(SimpleNamespace(pid=4321, returncode=code), None) == []
        raw = os.read(events.reader, 2048)
        assert json.loads(raw)["returncode"] == code
        for filename in ("safety_motion.launch.py", "simulation_motion.launch.py"):
            package = "senior_sim" if filename.startswith("simulation_") else "senior_robot"
            source = Path("ros2/src", package, "launch", filename).read_text()
            assert "OnProcessExit(on_exit=report_ros_exit)" in source
            # Cleanup OnShutdown handlers do not request a launch-wide shutdown.
            assert not any(
                isinstance(node, ast.Call)
                and (
                    (isinstance(node.func, ast.Name) and node.func.id == "Shutdown")
                    or (isinstance(node.func, ast.Attribute) and node.func.attr == "Shutdown")
                )
                for node in ast.walk(ast.parse(source))
            )
    finally:
        events.close()


@pytest.mark.parametrize("retired", [False, True])
async def test_task_failure_uses_actual_durable_pause_owner(tmp_path, retired):
    from test_task_ownership import start
    from vehicle_manager.storage.bindings import Database

    async with Database(tmp_path / "task.db") as database:
        runtime, executor, nav, live, identity = await start(database)
        nav.retired = retired
        assert await composition.pause_failed_tasks(executor) is retired
        state = await executor.lifecycle.store.load(identity)
        assert state.phase == ("paused" if retired else "revoking")
        assert state.stopped_for == "pause"
        assert [e.kind for e in nav.effects].count("plan") == 1
        assert any(e.kind == "revoke" for e in nav.effects)
        if not retired:
            assert await executor.lifecycle.store.active_effects(identity)


async def test_observer_eof_is_reported_by_actual_agent_without_exiting(monkeypatch, tmp_path):
    from agent_fakes import FakeHome, FakeMaps, FakeMotion, FakeState, FakeTasks
    from vehicle_manager.agent.production_bindings import AgentJournal
    from vehicle_manager.storage.bindings import Database

    from vehicle_core.lifecycle_events import observe

    events = LifecycleEvents()
    reader = os.dup(events.reader)
    link = FakeLink(1)
    async with Database(tmp_path / "robot.db") as database:
        journal = AgentJournal(database, robot_id="robot-a")
        await journal.prepare()

        async def observer():
            await observe(reader, agent.report_runtime_event)

        agent = Agent(
            "robot-a",
            connector=FakeConnector(link),
            journal=journal,
            tasks=FakeTasks(),
            home=FakeHome(),
            maps=FakeMaps(),
            state=FakeState(),
            motion=FakeMotion(),
            background={"observer": observer},
            heartbeat_ms=20,
        )
        runner = asyncio.create_task(agent.run())
        try:
            await until(lambda: agent.connected)
            events.close()
            await until(lambda: "observer" in agent.module_faults)
            beats = link.heartbeats
            await until(lambda: link.heartbeats > beats)
            assert not runner.done() and not link.closed
        finally:
            agent.request_shutdown()
            await asyncio.wait_for(runner, 2)
            events.close()


async def test_manual_transport_failure_retires_authority_without_cloud_disconnect(
    monkeypatch, tmp_path
):
    from agent_fakes import FakeHome, FakeMaps, FakeMotion, FakeState, FakeTasks, velocity
    from vehicle_manager.agent.production_bindings import AgentJournal
    from vehicle_manager.storage.bindings import Database

    from vehicle_core.ipc import IpcError

    class Motion(FakeMotion):
        sends = 0

        async def send_intent(self, *args, **kwargs):
            self.sends += 1
            raise IpcError("intent EOF")

    link, motion = FakeLink(1), Motion()
    async with Database(tmp_path / "robot.db") as database:
        journal = AgentJournal(database, robot_id="robot-1")
        await journal.prepare()
        agent = Agent(
            "robot-1",
            connector=FakeConnector(link),
            journal=journal,
            tasks=FakeTasks(),
            home=FakeHome(),
            maps=FakeMaps(),
            state=FakeState(),
            motion=motion,
            heartbeat_ms=20,
        )
        runner = asyncio.create_task(agent.run())
        try:
            await until(lambda: agent.connected)
            link.inbox.put_nowait(velocity())
            await until(lambda: "manual" in agent.module_faults)
            link.inbox.put_nowait(velocity(seq=20))
            beats = link.heartbeats
            await until(lambda: link.heartbeats > beats and link.inbox.empty())
            assert motion.sends == 1 and "forward:control.release" in motion.events
            assert not link.closed and not runner.done()
        finally:
            agent.request_shutdown()
            await asyncio.wait_for(runner, 2)


@pytest.mark.parametrize("module", ["tasks", "logs"])
async def test_failed_background_module_stops_without_stopping_report_plane(tmp_path, module):
    from agent_fakes import FakeHome, FakeMaps, FakeMotion, FakeState, FakeTasks
    from vehicle_manager.agent.production_bindings import AgentJournal
    from vehicle_manager.storage.bindings import Database

    calls = []
    trigger = asyncio.Event()

    async def background():
        calls.append(module)
        await trigger.wait()
        raise RuntimeError(f"{module} failed")

    link = FakeLink(1)
    async with Database(tmp_path / "robot.db") as database:
        journal = AgentJournal(database, robot_id="robot-a")
        await journal.prepare()
        agent = Agent(
            "robot-a",
            connector=FakeConnector(link),
            journal=journal,
            tasks=FakeTasks(),
            home=FakeHome(),
            maps=FakeMaps(),
            state=FakeState(),
            motion=FakeMotion(),
            background={module: background},
            heartbeat_ms=20,
        )
        runner = asyncio.create_task(agent.run())
        try:
            await until(lambda: agent.connected)
            trigger.set()
            await until(lambda: module in agent.module_faults)
            beats = link.heartbeats
            await until(lambda: link.heartbeats > beats)
            assert calls == [module] and not runner.done() and not link.closed
        finally:
            agent.request_shutdown()
            await asyncio.wait_for(runner, 2)


async def test_retired_straggler_cannot_issue_new_motion_or_reply(tmp_path):
    from agent_fakes import FakeHome, FakeMaps, FakeMotion, FakeState, FakeTasks, simple
    from vehicle_manager.agent.dispatcher import Deadlines
    from vehicle_manager.agent.production_bindings import AgentJournal
    from vehicle_manager.storage.bindings import Database

    from vehicle_core.errors import DomainError
    from vehicle_core.values import OperationResult, OperationStatus

    first, second = FakeLink(1), FakeLink(2)
    connector, motion = FakeConnector(first), FakeMotion()
    entered, release = asyncio.Event(), asyncio.Event()
    errors = []
    async with Database(tmp_path / "robot.db") as database:
        journal = AgentJournal(database, robot_id="robot-1")
        await journal.prepare()
        agent = Agent(
            "robot-1",
            connector=connector,
            journal=journal,
            tasks=FakeTasks(),
            home=FakeHome(),
            maps=FakeMaps(),
            state=FakeState(),
            motion=motion,
            deadlines=Deadlines(safety_ms=30.0),
            heartbeat_ms=20,
            retry_delay_ms=1,
        )
        runner = asyncio.create_task(agent.run())

        async def stubborn():
            entered.set()
            while not release.is_set():
                try:
                    await release.wait()
                except asyncio.CancelledError:
                    pass
            try:
                await agent.dispatch(simple("control.acquire", seq=1))
            except DomainError as exc:
                errors.append(exc)
            await agent._reply(
                first,
                simple("drive.stop", seq=2),
                result=OperationResult("old", OperationStatus.SUCCEEDED),
            )

        worker = None
        try:
            await until(lambda: agent.connected)
            worker = agent._spawn(stubborn())
            await entered.wait()
            first.drop()
            connector.add(second)
            await until(lambda: agent.connection_epoch == 2)
            assert worker in agent._stragglers and worker in agent._inflight
            release.set()
            await worker
            assert errors and "forward:control.acquire" not in motion.events
            assert not first.responses and not second.responses
            assert not runner.done() and not second.closed
        finally:
            release.set()
            agent.request_shutdown()
            await asyncio.wait_for(runner, 2)
            if worker:
                await worker


@pytest.mark.parametrize("capacity", [False, True])
async def test_safety_delivery_fault_preserves_cloud_and_never_claims_success(tmp_path, capacity):
    from agent_fakes import FakeHome, FakeMaps, FakeMotion, FakeState, FakeTasks, simple
    from vehicle_manager.agent.main import MAX_SAFETY_PENDING
    from vehicle_manager.agent.production_bindings import AgentJournal
    from vehicle_manager.storage.bindings import Database

    from vehicle_core.ipc import IpcError

    entered, release = asyncio.Event(), asyncio.Event()

    class Motion(FakeMotion):
        deliveries = 0

        async def estop_received(self, envelope, *, received_steady):
            self.deliveries += 1
            entered.set()
            if capacity:
                await release.wait()
            raise IpcError("estop outcome unconfirmed")

    link, motion = FakeLink(1), Motion()
    async with Database(tmp_path / "robot.db") as database:
        from test_task_ownership import start

        _, executor, nav, _, identity = await start(database)

        async def pause_owner(name):
            assert name == "motion_delivery"
            await composition.pause_failed_tasks(executor)

        journal = AgentJournal(database, robot_id="robot-1")
        await journal.prepare()
        agent = Agent(
            "robot-1",
            connector=FakeConnector(link),
            journal=journal,
            tasks=FakeTasks(),
            home=FakeHome(),
            maps=FakeMaps(),
            state=FakeState(),
            motion=motion,
            background_fault=pause_owner,
            heartbeat_ms=20,
        )
        runner = asyncio.create_task(agent.run())
        try:
            await until(lambda: agent.connected)
            link.inbox.put_nowait(simple("safety.estop", seq=1))
            if capacity:
                await entered.wait()
                for seq in range(2, MAX_SAFETY_PENDING + 3):
                    link.inbox.put_nowait(simple("safety.estop", seq=seq))
                await until(lambda: len(agent._safety_pending) == MAX_SAFETY_PENDING)
                await until(lambda: any(r.payload.error for r in link.responses))
                assert len(agent._safety_pending) <= MAX_SAFETY_PENDING
                release.set()
            await until(lambda: "motion_delivery" in agent.module_faults)
            beats = link.heartbeats
            await until(lambda: link.heartbeats > beats)
            assert not agent.auth_rejected and not agent.runtime_failed
            assert not runner.done() and not link.closed
            assert not any(r.payload.ok for r in link.responses)
            await until(lambda: agent._safety_fault_task and agent._safety_fault_task.done())
            assert (await executor.lifecycle.store.load(identity)).phase == "revoking"
            assert [e.kind for e in nav.effects].count("plan") == 1
        finally:
            release.set()
            agent.request_shutdown()
            await asyncio.wait_for(runner, 2)


async def test_reporting_ready_cannot_revive_failed_local_module(tmp_path):
    from agent_fakes import FakeHome, FakeMaps, FakeMotion, FakeState, FakeTasks
    from vehicle_manager.agent.production_bindings import AgentJournal
    from vehicle_manager.storage.bindings import Database

    async with Database(tmp_path / "robot.db") as database:
        journal = AgentJournal(database, robot_id="robot-a")
        await journal.prepare()
        agent = Agent(
            "robot-a",
            connector=FakeConnector(),
            journal=journal,
            tasks=FakeTasks(),
            home=FakeHome(),
            maps=FakeMaps(),
            state=FakeState(),
            motion=FakeMotion(),
        )
        await agent._background_fault("logs", "log driver stopped")
        agent.report_runtime_event({"module": "logs", "phase": "ready", "detail": "reported"})
        assert "logs" in agent.failed_modules
        assert agent.module_faults["logs"]["phase"] == "failed"
