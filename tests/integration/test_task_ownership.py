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
from test_task_lifecycle_store import NOW, setup_store

from senior_control.agent.composition import build_task_runtime
from senior_control.models import Pose2D
from senior_control.navigation import RouteProgress
from senior_control.storage.database import Database
from senior_control.tasks.executor import TaskExecutor
from senior_control.tasks.lifecycle import Effect
from senior_control.tasks.models import ExecutionContext, TaskDefinition
from senior_control.tasks.runtime import TaskRuntime
from senior_control.tasks.store import TaskStore


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
        return self.retired or None

    async def task_effect_status(self, effect: Effect) -> bool | None:
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


async def test_runtime_runs_offline_and_completes_same_identity(tmp_path: Path) -> None:
    async with Database(tmp_path / "task.db") as database:
        runtime, executor, nav, live, identity = await start(database)
        assert nav.progress is not None and nav.progress.status == "running"
        assert executor.lifecycle is not None
        receipt = await executor.lifecycle.store.query(f"schedule:{identity}")
        nav.progress = nav.progress.model_copy(
            update={
                "completed_action_ids": ["0:a", "1:b"],
                "status": "succeeded",
                "controller_output_exited": True,
            }
        )
        await runtime.tick(NOW + timedelta(seconds=60.5), 1, replace(live, connected=False))
        state = await executor.lifecycle.store.load(identity)
        assert state.phase == "succeeded" and state.completed == ("0:a", "1:b")
        assert await executor.lifecycle.store.query(f"schedule:{identity}") == receipt
        assert [effect.kind for effect in nav.effects] == ["plan"]


async def test_takeover_waits_for_output_ack_then_explicit_resume_uses_remaining(
    tmp_path: Path,
) -> None:
    import pytest

    from senior_control.errors import DomainError

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
    from senior_control.agent.ports import IpcNavigation
    from senior_control.models import MapRef, OperationResult, OperationStatus, Pose2D
    from senior_control.safety.arbiter import Arbiter

    monkeypatch.syspath_prepend(str(Path("ros2/src/senior_robot").resolve()))
    ros_nav = importlib.import_module("senior_robot.navigation_adapter")
    ros_ipc = importlib.import_module("senior_robot.agent_adapter")
    goals: list[asyncio.Future[bool]] = []
    arbiter = Arbiter("robot", 0.2, 5, 0.3, require_server_heartbeat=True)

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
        connection_epoch=lambda: 99,
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
        assert executor.lifecycle is not None
        finished = await executor.lifecycle.store.load(identity)
        assert finished.phase == "succeeded" and finished.completed == ("0:0", "1:1")
        assert await durable.tasks.list_tasks() != ()  # only the user's scheduled definition
        assert len(await durable.tasks.list_tasks()) == 1


async def test_dispatcher_acceptance_and_query_share_task_transaction(tmp_path: Path) -> None:
    from types import SimpleNamespace
    from typing import cast

    from agent_fakes import operation

    from senior_control.agent.dispatcher import (
        Deadlines,
        Dispatcher,
        HomePort,
        LifecycleNavigationOperations,
        MapPort,
        StoreTaskPort,
        TaskOperationResult,
    )
    from senior_control.agent.journal import AgentJournal
    from senior_control.agent.ports import IpcState, RuntimeState
    from senior_control.agent.workspace import WorkspaceReader
    from senior_control.models import MapRef, OperationStatus
    from senior_control.observability import Metrics
    from senior_control.waypoints.store import WaypointStore

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
            from senior_control.protocol import RobotPosePayload

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

        from senior_control.agent.connection import Link
        from senior_control.agent.main import Agent
        from senior_control.protocol import ResponseEnvelope, serialize_envelope, validate_envelope

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
        for kind, result in [
            ("navigation.goal", accepted),
            ("navigation.return_home", accepted),
            ("operation.query", queried),
        ]:
            reply_to = envelope.model_copy(update={"type": kind})
            await agent._reply(cast(Link, wire_link), reply_to, result=result)
            assert wire_link.responses[-1].payload.data == result.task_state
        from senior_control.models import OperationResult

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
        completed = await dispatcher.run(
            operation("operation.query", "normal-goal", robot_id="robot")
        )
        assert completed.status is OperationStatus.SUCCEEDED
        assert len(nav.effects) == 1 and len(await durable.tasks.list_tasks()) == 1


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


async def test_actual_manual_adapter_waits_for_task_retirement(tmp_path: Path) -> None:
    from agent_fakes import simple

    from senior_control.agent.ports import TaskAwareMotion
    from senior_control.errors import DomainError
    from senior_control.protocol import validate_envelope
    from senior_control.safety.arbiter import Arbiter

    async with Database(tmp_path / "task.db") as database:
        runtime, executor, nav, live, identity = await start(database)
        arbiter = Arbiter("robot", 0.2, 5, 0.3, require_server_heartbeat=True)
        arbiter.server_heartbeat(1, 0, 1)
        assert arbiter.begin_navigation(identity, 0, task_epoch=1)
        calls: list[str] = []

        class Client:
            async def request(self, kind: str, **fields: Any) -> dict[str, Any]:
                calls.append(kind)
                envelope = validate_envelope(fields["envelope"].encode(), "robot")
                result = arbiter.accept(envelope, 0)
                return {"ok": result.accepted}

        motion = TaskAwareMotion(Client())  # type: ignore[arg-type]
        motion.bind_tasks(executor)
        acquire = simple("control.acquire", seq=1, robot_id="robot", request_id="manual")
        with pytest.raises(DomainError):
            await motion.forward(acquire)
        assert calls == [] and arbiter.task_navigation_active
        nav.retired = True
        arbiter.end_navigation(identity)
        await motion.forward(acquire)
        assert calls == ["command"]
        assert executor.lifecycle is not None
        assert (await executor.lifecycle.store.load(identity)).stopped_for == "takeover"


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
    from senior_control.protocol import ResponsePayload

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

    from senior_control.agent.connection import Link
    from senior_control.agent.dispatcher import LifecycleNavigationOperations, StoreTaskPort
    from senior_control.agent.journal import AgentJournal
    from senior_control.agent.main import Agent
    from senior_control.agent.ports import IpcState, RuntimeState
    from senior_control.agent.workspace import WorkspaceReader
    from senior_control.protocol import RobotPosePayload, validate_envelope
    from senior_control.waypoints.store import WaypointStore

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

    from senior_control.protocol import ResponsePayload

    with pytest.raises(ValidationError):
        ResponsePayload(request_type="navigation.goal", ok=True, data=data)
