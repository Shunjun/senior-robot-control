"""Actual Python runtime/composition + SQLite wiring, not a Gazebo result."""

import asyncio
import importlib
from collections.abc import AsyncIterator
from contextlib import asynccontextmanager
from dataclasses import replace
from datetime import timedelta
from pathlib import Path
from typing import Any, Literal

import pytest
from senior_navigation.route import RouteProgress
from test_task_lifecycle_store import NOW, setup_store
from vehicle_manager.agent.production import build_task_runtime
from vehicle_manager.storage.bindings import Database
from vehicle_manager.storage.domain_bindings import TaskStore
from vehicle_manager.tasks.executor import TaskExecutor
from vehicle_manager.tasks.lifecycle import Effect
from vehicle_manager.tasks.models import ExecutionContext, TaskDefinition
from vehicle_manager.tasks.runtime import TaskRuntime

from vehicle_core.values import Pose2D


class Navigation:
    def __init__(self) -> None:
        self.effects: list[Effect] = []
        self.definitions: list[TaskDefinition] = []
        self.progress: RouteProgress | None = None
        self.retired = False
        self.plan_result: bool | None = True

    async def task_poses(self, task: TaskDefinition) -> tuple[Pose2D, ...]:
        return tuple(Pose2D(i, 0, 0) for i in range(len(task.waypoint_ids)))

    async def task_effect(self, effect: Effect, task: TaskDefinition) -> bool | None:
        self.effects.append(effect)
        self.definitions.append(task)
        if effect.kind == "plan":
            assert effect.map_ref is not None
            self.progress = RouteProgress(
                execution_id=effect.task_id,
                task_id=task.task_id,
                map_generation=effect.map_ref,
                status="running",
                task_epoch=effect.epoch,
                effect_id=effect.effect_id,
            )
            return self.plan_result
        if effect.kind == "terminal_release":
            proof = self.retired or (
                self.progress is not None and self.progress.controller_output_exited
            )
            return True if proof else None
        return self.retired or None

    async def task_effect_status(self, effect: Effect) -> bool | None:
        if effect.kind == "terminal_release":
            return (
                True
                if self.retired or (self.progress and self.progress.controller_output_exited)
                else None
            )
        return (self.retired or None) if effect.kind == "revoke" else True

    async def status(self, execution_id: str) -> RouteProgress | None:
        return self.progress


async def start(
    database: Database,
) -> tuple[TaskRuntime, TaskExecutor, Navigation, ExecutionContext, str]:
    _, task = await setup_store(database)
    nav = Navigation()
    executor_tasks = TaskStore(database, robot_id="robot")
    runtime, executor = build_task_runtime(database, executor_tasks, nav, boot_id="normal-boot")
    live = ExecutionContext(
        True,
        True,
        True,
        False,
        task.map_ref,
        map_generation="map",
        localization_generation="map",
        pose_ref="current-pose",
        task_lifecycle_version=2,
    )
    await runtime.tick(NOW + timedelta(seconds=59.5), 0, live)
    await runtime.tick(NOW + timedelta(seconds=60), 0.5, live)
    records = await executor_tasks.list_executions()
    return runtime, executor, nav, live, records[0].execution_id


@pytest.mark.parametrize("outage", ["socket", "heartbeat"])
async def test_runtime_runs_offline_and_completes_same_identity(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, outage: str
) -> None:
    from agent_fakes import FakeHome, FakeMaps, FakeMotion, FakeState, FakeTasks, until
    from heartbeat_test_support import HeartbeatClock, RelaySocket, relay_hub
    from vehicle_manager.agent.production_bindings import AgentJournal, WebSocketLink
    from vehicle_manager.agent.runtime_bindings import Agent

    clock = HeartbeatClock(monkeypatch)
    hub = relay_hub(monkeypatch, clock)
    first = RelaySocket(hub, "robot", drop_replies=outage == "heartbeat")
    links: asyncio.Queue[Any] = asyncio.Queue()
    links.put_nowait(WebSocketLink(first, "robot", 1, clock=clock.monotonic))
    async with Database(tmp_path / "task.db") as database:
        runtime, executor, nav, live, identity = await start(database)
        assert nav.progress is not None and nav.progress.status == "running"
        assert executor.lifecycle is not None
        receipt = await executor.lifecycle.store.query(f"schedule:{identity}")

        class State(FakeState):
            async def collect(self):
                return replace(await super().collect(), route_progress=nav.progress)

        journal = AgentJournal(database, robot_id="robot")
        await journal.prepare()
        agent = Agent(
            "robot",
            connector=links.get,
            journal=journal,
            tasks=FakeTasks(),
            home=FakeHome(),
            maps=FakeMaps(),
            state=State(),
            motion=FakeMotion(),
            clock=clock,
            heartbeat_ms=20,
            retry_delay_ms=1,
        )
        runner = asyncio.create_task(agent.run())
        try:
            await until(lambda: bool(first.sent))
            if outage == "socket":
                await first.close()
            else:
                clock.advance(14.999)
                await agent._beat()
                assert agent.connected and not first.closed
                clock.advance(15.0)
            await until(lambda: not agent.connected and first.closed)
            # Derive offline from a real closed relay transport and completed Agent session.
            assert first.connection.outbox.closed
            assert (await executor.lifecycle.store.load(identity)).phase == "running"
            nav.progress = nav.progress.model_copy(
                update={
                    "completed_action_ids": ["0:a", "1:b"],
                    "status": "succeeded",
                    "controller_output_exited": True,
                }
            )
            offline = replace(live, connected=agent.connected)
            await runtime.tick(NOW + timedelta(seconds=60.5), 1, offline)
            pending = (await executor.lifecycle.store.load(identity)).pending
            assert pending is not None and pending.kind == "terminal_release"
            await runtime.tick(NOW + timedelta(seconds=60.6), 1.1, offline)
            state = await executor.lifecycle.store.load(identity)
            assert state.phase == "succeeded" and state.completed == ("0:a", "1:b")
            second = RelaySocket(hub, "robot")
            links.put_nowait(WebSocketLink(second, "robot", 2, clock=clock.monotonic))
            await until(lambda: agent.connection_epoch == 2)
            await until(lambda: any(frame["type"] == "snapshot" for frame in second.sent))
            snapshot = next(frame for frame in second.sent if frame["type"] == "snapshot")
            synced = snapshot["payload"]["route_progress"]
            assert synced["execution_id"] == nav.progress.execution_id
            assert synced["task_id"] == nav.progress.task_id and synced["status"] == "succeeded"
            await runtime.tick(NOW + timedelta(seconds=61), 1.5, replace(live, connected=True))
            assert await executor.lifecycle.store.query(f"schedule:{identity}") == receipt
            assert [effect.kind for effect in nav.effects] == [
                "plan",
                "terminal_release",
                "terminal_settled",
            ]
        finally:
            agent.request_shutdown()
            await asyncio.wait_for(runner, 2)


async def test_takeover_waits_for_output_ack_then_explicit_resume_uses_remaining(
    tmp_path: Path,
) -> None:
    import pytest

    from vehicle_core.errors import DomainError

    async with Database(tmp_path / "task.db") as database:
        runtime, executor, nav, live, identity = await start(database)
        assert nav.progress is not None
        nav.progress = nav.progress.model_copy(update={"completed_action_ids": ["0:a"]})
        with pytest.raises(DomainError):
            await executor.control_current("takeover", "manual")
        assert executor.lifecycle is not None
        assert (await executor.lifecycle.store.load(identity)).phase == "revoking"
        nav.retired = True
        await runtime.tick(NOW + timedelta(seconds=60.5), 1, live)
        await executor.control_current("takeover", "manual")
        assert (await executor.lifecycle.store.load(identity)).phase == "paused"
        await runtime.tick(NOW + timedelta(seconds=61), 1.5, replace(live, connected=False))
        assert (await executor.lifecycle.store.load(identity)).phase == "paused"
        executor.context = replace(live, pose_ref="new-pose")
        await executor.control_current("resume", "continue")
        assert nav.effects[-1].kind == "plan" and nav.effects[-1].remaining == ("1:b",)
        assert nav.effects[-1].pose_ref == "new-pose"


async def test_existing_ipc_receiver_records_actions_and_task_epoch(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    from senior_motion_control.arbiter import Arbiter
    from vehicle_manager.agent.local_adapters import IpcNavigation

    from vehicle_core.values import MapRef, OperationResult, OperationStatus, Pose2D

    monkeypatch.syspath_prepend(str(Path("ros2/src/senior_robot").resolve()))
    ros_nav = importlib.import_module("senior_robot.navigation_adapter")
    ros_ipc = importlib.import_module("senior_robot.agent_adapter")
    goals: list[asyncio.Future[bool]] = []
    arbiter = Arbiter("robot", 300)

    class Goals:
        async def start(self, goal: Any) -> str:
            goals.append(asyncio.get_running_loop().create_future())
            return str(len(goals) - 1)

        async def result(self, identity: str) -> bool:
            return await goals[int(identity)]

        async def cancel(self, identity: str) -> OperationResult:
            return OperationResult(identity, OperationStatus.SUCCEEDED)

    @asynccontextmanager
    async def controller(identity: str) -> AsyncIterator[str]:
        yield "/control/navigation/run_" + "a" * 32 + "/cmd_vel"

    def bind(identity: str, topic: str, guard: Any) -> bool:
        retired.discard(identity)
        return arbiter.begin_navigation(identity, 0, task_epoch=routes.current.task_epoch)

    retired: set[str] = set()

    def clear(identity: str) -> None:
        arbiter.end_navigation(identity)
        retired.add(identity)

    routes = ros_nav.RouteService(
        ros_nav.NavigationAdapter(Goals()),
        qualified=lambda *_: True,
        prepare=controller,
        bind_source=bind,
        clear_source=clear,
        source_active=lambda key: arbiter.navigation_source_active(key, 0),
        source_retired=lambda key: key in retired,
        release_identity=lambda key, epoch, _effect, outcome, **_wire: (
            key in retired and epoch == 2 and outcome == "succeeded"
        ),
    )
    core = ros_ipc.AdapterCore(
        robot_id="robot",
        submit=lambda *_: None,
        disconnect=lambda _: None,
        request_exit=lambda: None,
        routes=routes,
    )

    class Client:
        async def request(self, kind: str, **fields: Any) -> dict[str, Any]:
            reply: dict[str, Any] = await core.handle({"type": kind, **fields})
            return reply

    async with Database(tmp_path / "task.db") as database:
        durable, _ = await setup_store(database)
        ipc = IpcNavigation(Client(), tasks=durable.tasks)
        runtime, executor = build_task_runtime(database, durable.tasks, ipc, boot_id="ipc-test")
        executor.context = ExecutionContext(
            True,
            True,
            True,
            False,
            MapRef("map", 1),
            map_generation="map",
            localization_generation="map",
            pose_ref="current",
            task_lifecycle_version=2,
        )
        identity = await executor.submit_route(
            "op", "fingerprint", MapRef("map", 1), "map", (Pose2D(1, 0, 0), Pose2D(2, 0, 0))
        )
        assert routes.current.task_epoch == 1 and routes.current.effect_id is not None
        for _ in range(20):
            if goals:
                break
            await asyncio.sleep(0)
        goals[0].set_result(True)
        for _ in range(20):
            if len(goals) == 2:
                break
            await asyncio.sleep(0.01)
        assert len(goals) == 2
        await executor.control_current("pause", "pause-op")
        assert executor.lifecycle is not None
        paused = await executor.lifecycle.store.load(identity)
        assert paused.phase == "paused" and paused.completed == ("0:0",)
        executor.context = replace(executor.context, pose_ref="resume-from-now")
        await executor.control_current("resume", "resume-op")
        for _ in range(20):
            if len(goals) == 3:
                break
            await asyncio.sleep(0.01)
        assert len(goals) == 3
        assert routes.current.task_epoch == 2
        assert routes._current.plan.action_ids == ["1:1"]
        assert routes._current.plan.pose_ref == "resume-from-now"
        goals[2].set_result(True)
        for _ in range(20):
            if routes.current is None:
                break
            await asyncio.sleep(0.01)
        progress = routes.status(identity)
        assert progress.completed_action_ids == ["1:1"]
        assert progress.controller_output_exited
        await runtime.tick(NOW, 0, executor.context)
        await runtime.tick(NOW, 0.1, executor.context)  # Separate identity-release outbox pass.
        assert executor.lifecycle is not None
        finished = await executor.lifecycle.store.load(identity)
        assert finished.phase == "succeeded" and finished.completed == ("0:0", "1:1")
        assert await durable.tasks.list_tasks() != ()  # only the user's scheduled definition
        assert len(await durable.tasks.list_tasks()) == 1


async def test_dispatcher_acceptance_and_query_share_task_transaction(tmp_path: Path) -> None:
    from types import SimpleNamespace
    from typing import cast

    from agent_fakes import operation
    from vehicle_manager.agent.dispatcher import Deadlines, HomePort
    from vehicle_manager.agent.exported_workspace import WorkspaceReader
    from vehicle_manager.agent.local_adapters import IpcState
    from vehicle_manager.agent.local_state import RuntimeState
    from vehicle_manager.agent.production_bindings import AgentJournal
    from vehicle_manager.agent.request_bindings import (
        Dispatcher,
        LifecycleNavigationOperations,
        MapPort,
        StoreTaskPort,
        TaskOperationResult,
    )
    from vehicle_manager.observability import Metrics
    from vehicle_manager.storage.domain_bindings import WaypointStore

    from vehicle_core.values import MapRef, OperationStatus

    async with Database(tmp_path / "task.db") as database:
        durable, _ = await setup_store(database)
        nav = Navigation()
        runtime, executor = build_task_runtime(database, durable.tasks, nav, boot_id="dispatch")
        executor.context = ExecutionContext(
            True,
            True,
            True,
            False,
            MapRef("map", 1),
            map_generation="map",
            localization_generation="map",
            pose_ref="pose",
            task_lifecycle_version=2,
        )

        async def read() -> RuntimeState:
            from senior_contracts.wire import RobotPosePayload

            return RuntimeState(
                True,
                True,
                False,
                MapRef("map", 1),
                "idle",
                "map",
                "map",
                True,
                robot_pose=RobotPosePayload(x=0.0, y=0.0, yaw=0.0),
                task_lifecycle_version=2,
                task_evidence_ready=True,
            )

        async def free(*args: Any) -> None:
            pass

        state = cast(IpcState, SimpleNamespace(read=read))
        workspace = cast(WorkspaceReader, SimpleNamespace(verify_free_target=free))
        waypoints = WaypointStore(database, robot_id="robot")
        await waypoints.prepare()
        journal = AgentJournal(database, robot_id="robot")
        await journal.prepare()
        actions = LifecycleNavigationOperations(executor, state, workspace, waypoints)
        admissions: list[str] = []
        dispatcher = Dispatcher(
            "robot",
            journal=journal,
            tasks=StoreTaskPort(durable.tasks, robot_id="robot"),
            home=cast(HomePort, None),
            maps=cast(MapPort, None),
            deadlines=Deadlines(),
            metrics=Metrics(),
            map_root=tmp_path,
            admit=lambda envelope: admissions.append(envelope.type),
            navigation=actions,
        )
        envelope = operation(
            "navigation.goal",
            "normal-goal",
            {
                "map_id": "map",
                "reference_revision": 1,
                "grid_id": "map",
                "x": 1.0,
                "y": 2.0,
                "yaw": 0.0,
            },
            robot_id="robot",
        )
        accepted = await dispatcher.run(envelope)
        assert (
            isinstance(accepted, TaskOperationResult)
            and accepted.status is OperationStatus.ACCEPTED
        )
        assert accepted.task_state["phase"] == "running"
        entry = await journal.get("normal-goal")
        assert entry is not None and entry.fingerprint
        assert len(nav.effects) == 1
        replay = await dispatcher.run(envelope)
        assert replay.status is OperationStatus.ACCEPTED and len(nav.effects) == 1
        queried = await dispatcher.run(
            operation("operation.query", "normal-goal", robot_id="robot")
        )
        assert (
            isinstance(queried, TaskOperationResult) and queried.status is OperationStatus.RUNNING
        )
        assert admissions == ["navigation.goal"]

        # Exercise the actual Agent._reply and both wire validators, beyond the
        # dispatcher/service return value which previously hid the data contract bug.
        from agent_fakes import FakeConnector, FakeHome, FakeLink, FakeMaps, FakeMotion, FakeState
        from senior_contracts.wire import ResponseEnvelope, serialize_envelope, validate_envelope
        from vehicle_manager.agent.connection import Link
        from vehicle_manager.agent.runtime_bindings import Agent

        class WireLink(FakeLink):
            async def send_response(self, response: ResponseEnvelope) -> None:
                decoded = validate_envelope(serialize_envelope(response), "robot")
                assert isinstance(decoded, ResponseEnvelope)
                self.responses.append(decoded)

        wire_link = WireLink(epoch=1)
        agent = Agent(
            "robot",
            connector=cast(Any, FakeConnector()),
            journal=journal,
            tasks=StoreTaskPort(durable.tasks, robot_id="robot"),
            home=FakeHome(),
            maps=FakeMaps(),
            state=FakeState(),
            motion=FakeMotion(),
            navigation=actions,
        )
        agent._link, agent._epoch = wire_link, 1
        for kind, result in [
            ("navigation.goal", accepted),
            ("navigation.return_home", accepted),
            ("operation.query", queried),
        ]:
            reply_to = envelope.model_copy(update={"type": kind})
            await agent._reply(cast(Link, wire_link), reply_to, result=result)
            assert wire_link.responses[-1].payload.data == result.task_state
        from vehicle_core.values import OperationResult

        await agent._reply(
            cast(Link, wire_link),
            envelope.model_copy(update={"type": "operation.query"}),
            result=OperationResult("legacy-operation", OperationStatus.SUCCEEDED),
        )
        assert wire_link.responses[-1].payload.data is None
        assert nav.progress is not None
        nav.progress = nav.progress.model_copy(
            update={
                "completed_action_ids": ["0:0"],
                "status": "succeeded",
                "controller_output_exited": True,
            }
        )
        await runtime.tick(NOW, 0, replace(executor.context, connected=False))
        await runtime.tick(NOW, 0.1, replace(executor.context, connected=False))
        completed = await dispatcher.run(
            operation("operation.query", "normal-goal", robot_id="robot")
        )
        assert completed.status is OperationStatus.SUCCEEDED
        assert [effect.kind for effect in nav.effects] == ["plan", "terminal_release"]
        assert len(await durable.tasks.list_tasks()) == 1


async def test_runtime_queries_unknown_plan_without_second_submission(tmp_path: Path) -> None:
    async with Database(tmp_path / "task.db") as database:
        durable, task = await setup_store(database)
        nav = Navigation()
        nav.plan_result = None
        runtime, executor = build_task_runtime(database, durable.tasks, nav, boot_id="unknown")
        live = ExecutionContext(
            True,
            True,
            True,
            False,
            task.map_ref,
            map_generation="map",
            localization_generation="map",
            pose_ref="pose",
            task_lifecycle_version=2,
        )
        await runtime.tick(NOW + timedelta(seconds=59), 0, live)
        await runtime.tick(NOW + timedelta(seconds=60), 1, live)
        assert executor.lifecycle is not None
        identity = nav.effects[0].task_id
        assert (await executor.lifecycle.store.load(identity)).phase == "starting"
        await runtime.tick(NOW + timedelta(seconds=60.5), 1.5, replace(live, connected=False))
        assert (await executor.lifecycle.store.load(identity)).phase == "running"
        assert [effect.kind for effect in nav.effects] == ["plan"]


async def test_runtime_restart_pauses_without_automatic_plan(tmp_path: Path) -> None:
    async with Database(tmp_path / "task.db") as database:
        runtime, executor, nav, live, identity = await start(database)
        assert executor.lifecycle is not None
        nav.retired = True
        resumed_runtime, resumed_executor = build_task_runtime(
            database, executor.lifecycle.store.tasks, nav, boot_id="next-boot"
        )
        await resumed_runtime.tick(NOW + timedelta(seconds=61), 2, live)
        assert resumed_executor.lifecycle is not None
        state = await resumed_executor.lifecycle.store.load(identity)
        assert state.phase == "paused" and state.stopped_for == "restart"
        assert [effect.kind for effect in nav.effects] == ["plan", "revoke"]


async def test_accepted_targets_are_immutable_when_future_schedule_is_edited(
    tmp_path: Path,
) -> None:
    async with Database(tmp_path / "task.db") as database:
        runtime, executor, nav, live, identity = await start(database)
        assert executor.lifecycle is not None
        store = executor.lifecycle.store.tasks
        definition = await store.get("definition")
        assert definition is not None
        await store.update(
            replace(definition, waypoint_ids=("changed", "targets")), definition.revision
        )
        nav.retired = True
        await executor.control_current("pause", "pause")
        await executor.control_current("resume", "resume")
        assert nav.definitions[-1].waypoint_ids == ("a", "b")
        assert nav.effects[-1].remaining == ("0:a", "1:b")


async def test_status_timeout_keeps_accepted_task_for_next_tick(tmp_path: Path) -> None:
    async with Database(tmp_path / "task.db") as database:
        runtime, executor, nav, live, identity = await start(database)
        original = nav.status

        async def unavailable(execution_id: str) -> RouteProgress | None:
            raise asyncio.TimeoutError()

        nav.status = unavailable  # type: ignore[method-assign]
        await runtime.tick(NOW + timedelta(seconds=60.5), 1, live)
        assert executor.lifecycle is not None
        assert (await executor.lifecycle.store.load(identity)).phase == "running"
        nav.status = original  # type: ignore[method-assign]
        await runtime.tick(NOW + timedelta(seconds=61), 1.5, live)
        assert len([effect for effect in nav.effects if effect.kind == "plan"]) == 1


@pytest.mark.parametrize("kind", ["navigation.goal", "navigation.return_home", "operation.query"])
def test_task_result_survives_actual_wire_response(
    kind: Literal["navigation.goal", "navigation.return_home", "operation.query"],
) -> None:
    from senior_contracts.wire import ResponsePayload

    receipt = {
        "execution_id": "owned-task",
        "phase": "running",
        "epoch": 1,
        "completed": ["a"],
        "remaining": ["b"],
    }
    result = ResponsePayload(request_type=kind, ok=True, status="accepted", data=receipt)
    assert result.model_dump()["data"] == receipt


@pytest.mark.parametrize("stop_action", ["pause", "takeover"])
async def test_authenticated_task_commands_normal_lifecycle_and_receipt(
    tmp_path: Path,
    stop_action: str,
) -> None:
    import json
    import time
    from types import SimpleNamespace
    from typing import cast

    from agent_fakes import FakeConnector, FakeHome, FakeLink, FakeMaps, FakeMotion, FakeState
    from senior_contracts.wire import RobotPosePayload, validate_envelope
    from vehicle_manager.agent.connection import Link
    from vehicle_manager.agent.exported_workspace import WorkspaceReader
    from vehicle_manager.agent.local_adapters import IpcState
    from vehicle_manager.agent.local_state import RuntimeState
    from vehicle_manager.agent.production_bindings import AgentJournal
    from vehicle_manager.agent.request_bindings import LifecycleNavigationOperations, StoreTaskPort
    from vehicle_manager.agent.runtime_bindings import Agent
    from vehicle_manager.storage.domain_bindings import WaypointStore

    async with Database(tmp_path / "task.db") as database:
        runtime, executor, nav, live, identity = await start(database)
        assert executor.lifecycle is not None
        service = executor.lifecycle
        journal = AgentJournal(database, robot_id="robot")
        await journal.prepare()
        waypoints = WaypointStore(database, robot_id="robot")
        await waypoints.prepare()

        async def read() -> RuntimeState:
            return RuntimeState(
                True,
                True,
                False,
                live.active_map,
                "idle",
                "map",
                "map",
                True,
                robot_pose=RobotPosePayload(x=3.0, y=4.0, yaw=0.0),
                task_lifecycle_version=2,
                task_evidence_ready=True,
            )

        actions = LifecycleNavigationOperations(
            executor,
            cast(IpcState, SimpleNamespace(read=read)),
            cast(WorkspaceReader, None),
            waypoints,
        )
        agent = Agent(
            "robot",
            connector=cast(Any, FakeConnector()),
            journal=journal,
            tasks=StoreTaskPort(service.store.tasks, robot_id="robot"),
            home=FakeHome(),
            maps=FakeMaps(),
            state=FakeState(),
            motion=FakeMotion(),
            navigation=actions,
        )
        link = FakeLink(epoch=1)
        agent._link, agent._epoch = cast(Link, link), 1

        def request(action: str, version: int, request_id: str) -> Any:
            return validate_envelope(
                json.dumps(
                    {
                        "v": 1,
                        "robot_id": "robot",
                        "session_id": "session",
                        "connection_epoch": 1,
                        "request_id": "transport-" + request_id,
                        "seq": 1,
                        "expires_at": time.time() + 30,
                        "type": "task.execution.command",
                        "payload": {
                            "request_id": request_id,
                            "expected_version": version,
                            "task_id": identity,
                            "action": action,
                        },
                    }
                ).encode(),
                "robot",
            )

        state = await service.store.load(identity)
        command = request(stop_action, state.version, "explicit-stop")
        await agent._answer(cast(Link, link), command)
        receipt = link.responses[-1].payload.data
        assert receipt == {
            "request_id": "explicit-stop",
            "task_id": identity,
            "accepted": True,
            "reason": "accepted",
            "version": state.version + 1,
        }
        assert (await service.store.load(identity)).phase == "revoking"
        await agent._answer(cast(Link, link), command)
        assert link.responses[-1].payload.data == receipt
        await agent._answer(
            cast(Link, link), request(stop_action, state.version + 1, "explicit-stop")
        )
        assert not link.responses[-1].payload.ok
        assert link.responses[-1].payload.error is not None

        nav.retired = True
        await runtime.tick(NOW + timedelta(seconds=60.5), 1, live)
        state = await service.store.load(identity)
        assert state.phase == "paused"
        view = await actions.execution_view(identity)
        assert view == {
            "task_id": identity,
            "version": state.version,
            "phase": "paused",
            "completed": 0,
            "total": 2,
            "stopped_for": stop_action,
            "allowed_actions": ["resume", "cancel", "takeover"],
        }
        await agent._answer(cast(Link, link), request("resume", state.version, "explicit-resume"))
        assert link.responses[-1].payload.data is not None
        assert link.responses[-1].payload.data["accepted"] is True
        assert nav.effects[-1].kind == "plan"
        assert nav.effects[-1].pose_ref == '{"x":3.0,"y":4.0,"yaw":0.0}'

        state = await service.store.load(identity)
        await agent._answer(cast(Link, link), request("cancel", state.version, "explicit-cancel"))
        assert link.responses[-1].payload.data is not None
        assert link.responses[-1].payload.data["accepted"] is True
        await runtime.tick(NOW + timedelta(seconds=61), 1.5, live)
        assert (await service.store.load(identity)).phase == "canceled"
        assert (await actions.execution_view(identity))["allowed_actions"] == []


@pytest.mark.parametrize(
    "data",
    [
        {"execution_id": "x", "phase": "running"},
        {"execution_id": "x", "phase": "running", "epoch": -1, "completed": [], "remaining": []},
        {
            "execution_id": "x",
            "phase": "running",
            "epoch": 1,
            "completed": ["a"],
            "remaining": ["a"],
        },
        {"arbitrary": "unrelated payload"},
    ],
)
def test_task_navigation_response_rejects_unvalidated_data(data: dict[str, Any]) -> None:
    from pydantic import ValidationError
    from senior_contracts.wire import ResponsePayload

    with pytest.raises(ValidationError):
        ResponsePayload(request_type="navigation.goal", ok=True, data=data)
