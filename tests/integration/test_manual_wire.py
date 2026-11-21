"""Authenticated ASGI relay -> actual Agent/IPC/CommandNode/T2 classes; no ROS runtime."""

import asyncio
import importlib.util
import json
import socket
import sys
import time
from collections.abc import AsyncIterator, Awaitable, Callable
from contextlib import asynccontextmanager
from pathlib import Path
from types import ModuleType, SimpleNamespace
from typing import Any, cast
from unittest.mock import AsyncMock, MagicMock

import pytest
from senior_contracts.wire import ResponseEnvelope, serialize_envelope, validate_envelope
from senior_motion_control.arbiter import Arbiter
from senior_motion_control.manual_control import ManualIngress, MotionRuntimeReceiver
from test_cloud import Cloud, cloud  # noqa: F401
from vehicle_manager.settings.models import simulation_settings_profile

from vehicle_core.runtime_state import MotionRuntimeState


@asynccontextmanager
async def ipc_pair(
    handler: Callable[[dict[str, Any]], Awaitable[dict[str, Any]]],
    monkeypatch: pytest.MonkeyPatch,
) -> AsyncIterator[Any]:
    """Real length-framed IPC over a connected local pair; no bind/listener."""
    from vehicle_core.ipc import IpcClient, IpcServer

    left, right = socket.socketpair()
    client_reader, client_writer = await asyncio.open_connection(sock=left)
    server_reader, server_writer = await asyncio.open_connection(sock=right)
    server = IpcServer(Path("/tmp/t2-unbound-test.sock"), handler)
    client = IpcClient(Path("/tmp/t2-unbound-test.sock"))
    serving = asyncio.create_task(server._serve(server_reader, server_writer))

    client._reader, client._writer = client_reader, client_writer
    try:
        yield client
    finally:
        await client.close()
        await server.close()
        serving.cancel()
        await asyncio.gather(serving, return_exceptions=True)
        await client_writer.wait_closed()
        await server_writer.wait_closed()


def runtime(now: float, sequence: int = 1, **changes: Any) -> MotionRuntimeState:
    return MotionRuntimeState.model_validate(
        {
            "schema": 1,
            "robot_id": "robot",
            "boot_id": "t5-boot",
            "sequence": sequence,
            "sampled_steady": now,
            "mode": "idle",
            "reason": "standby",
            "stationary": True,
            "healthy": True,
            "evidence_stamp_steady": now,
            "evidence_deadline_steady": now + 0.25,
            "estop_revision": 0,
            "fault_revision": 0,
            "confirmed_estop_status": "clear",
            "allowed_actions": ["acquire"],
            "task_id": None,
            "output_generation": 1,
            "body_reference": None,
            "wheel_linear_mps": None,
            **changes,
        }
    )


@pytest.fixture
def node(monkeypatch: pytest.MonkeyPatch) -> Any:
    modules: dict[str, dict[str, Any]] = {
        "rclpy": {},
        "geometry_msgs.msg": {"Twist": object},
        "rclpy.clock": {"Clock": object, "ClockType": object},
        "rclpy.node": {"Node": object},
        "rclpy.parameter": {"Parameter": object},
        "rclpy.subscription": {"Subscription": object},
        "rclpy.qos": {"qos_profile_sensor_data": SimpleNamespace(depth=5)},
        "std_msgs.msg": {"String": object},
        "senior_motion_msgs.msg": {
            "DriveExecutionFeedback": object,
            "FinalCommand": type("FinalCommand", (), {}),
        },
        "senior_robot.motion_lifecycle": {"with_dds_info": lambda callback: callback},
    }
    for name, attributes in modules.items():
        stub = ModuleType(name)
        stub.__dict__.update(attributes)
        monkeypatch.setitem(sys.modules, name, stub)
    path = Path("ros2/src/senior_motion_control/senior_motion_control/command_node.py")
    spec = importlib.util.spec_from_file_location("manual_command_under_test", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    monkeypatch.setitem(sys.modules, spec.name, module)
    spec.loader.exec_module(module)
    value = module.CommandNode.__new__(module.CommandNode)
    clock = SimpleNamespace(now=10.0)
    value._monotonic_clock = SimpleNamespace(monotonic=lambda: clock.now)
    value._owner_boot = "command-boot"
    value._generation, value._active = 1, False
    value._last_close_source, value._close_reason = ("idle", "", 0, 0.0), "prepared"
    value._navigation_source, value._drive_sample = None, None
    value._retirements, value._last_server_epoch = {}, 0
    value._terminal_requests, value._retirement_core = {}, None
    value._terminal_collections, value._settled_sequence = {}, 0
    value._retirement_feedback_gid = None
    value._manual_ingress = ManualIngress("robot", "command-boot", ttl=0.3)
    value._manual_target = None
    value._consuming_runtime = False
    value._runtime_subscription = MagicMock(msg_type=object, raw=False)
    value._runtime_subscription.handle.take_message.return_value = None
    value._motion_runtime = MotionRuntimeReceiver()
    value._motion_runtime.accept(runtime(clock.now), b"t5", clock.now)
    value._arbiter = Arbiter("robot", 300)
    output: list[Any] = []
    value._final_publisher = SimpleNamespace(publish=output.append)
    value._publish = lambda velocity: None
    value.get_parameter = lambda name: SimpleNamespace(value="robot")
    return SimpleNamespace(value=value, clock=clock, output=output)


def envelope(
    kind: str,
    payload: dict[str, Any],
    *,
    seq: int = 1,
    request: str = "request",
    robot: str = "robot",
    generation: int | None = 1,
) -> Any:
    data = {
        "v": 1,
        "robot_id": robot,
        "session_id": "controller",
        "connection_epoch": 1,
        "request_id": request,
        "seq": seq,
        "expires_at": time.time() + 30,
        "type": kind,
        "payload": payload,
    }
    if generation is not None:
        data["controller_generation"] = generation
    return validate_envelope(json.dumps(data).encode(), robot)


def call(node: Any, frame: Any, received: float | None = None) -> dict[str, Any]:
    return cast(
        dict[str, Any],
        node.value.manual_request(
            {
                "envelope": serialize_envelope(frame).decode(),
                "received_steady": received,
                "deadline_steady": received + 0.3 if received is not None else None,
                "driving": simulation_settings_profile().driving_defaults.model_dump(),
            }
        ),
    )


def test_prepared_waits_then_original_deadline_release_and_new_authority(node: Any) -> None:
    prepared = call(node, envelope("control.acquire", {}))
    assert prepared["state"] == "prepared"
    identity = prepared["authority_id"]
    assert not any(getattr(output, "vx", 0) for output in node.output)
    node.clock.now = 20.0
    node.value._motion_runtime.accept(runtime(20, 2), b"t5", 20)
    assert call(node, envelope("control.acquire", {}))["authority_id"] == identity
    intent = {
        "authority_id": identity,
        "gesture": 1,
        "sequence": 1,
        "linear": 1.0,
        "turn": 1.0,
        "gear": "low",
    }
    assert call(node, envelope("drive.intent", intent, seq=2), 19.99)["ok"] is True
    actual = node.output[-1]
    assert actual.source_kind == "manual" and actual.source_id == identity
    assert actual.vx == 5 / 9 and actual.wz == 1
    assert actual.manual_linear == 1
    assert actual.manual_linear_limit == 5 / 9
    assert actual.degraded_linear_limit == 0.3
    assert actual.received_steady == 19.99 and actual.steady_deadline == 20.29
    assert call(node, envelope("control.release", {"authority_id": identity}, seq=3))["ok"] is True
    assert node.output[-1].close_reason == "release"
    assert call(node, envelope("drive.intent", {**intent, "sequence": 2}, seq=4), 20)["ok"] is False
    second = call(node, envelope("control.acquire", {}, seq=5, request="second"))
    assert second["state"] == "prepared" and second["authority_id"] != identity
    # Releasing an earlier authority cannot close its successor.
    call(node, envelope("control.release", {"authority_id": identity}, seq=6))
    assert node.value.manual_view()["authority_id"] == second["authority_id"]
    assert node.value.manual_view()["state"] == "prepared"


async def test_real_authenticated_relay_agent_ipc_and_native_owner(
    cloud: Cloud,  # noqa: F811 - imported pytest fixture
    tmp_path: Path,
    node: Any,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from agent_fakes import FakeConnector, FakeHome, FakeMaps
    from vehicle_manager.agent.local_adapters import IpcNavigation, IpcState, TaskAwareMotion
    from vehicle_manager.agent.production import SnapshotState, build_task_runtime
    from vehicle_manager.agent.production_bindings import AgentJournal
    from vehicle_manager.agent.request_bindings import StoreTaskPort
    from vehicle_manager.agent.runtime_bindings import Agent
    from vehicle_manager.agent.snapshots import snapshot_envelope
    from vehicle_manager.settings.service import RuntimeGate, SettingsService
    from vehicle_manager.settings.store import SettingsStore
    from vehicle_manager.storage.bindings import Database
    from vehicle_manager.storage.domain_bindings import TaskStore, WaypointStore

    monkeypatch.syspath_prepend(str(Path("ros2/src/senior_robot").resolve()))
    import importlib

    AdapterCore = importlib.import_module("senior_robot.agent_adapter").AdapterCore

    csrf = await cloud.login()
    robot_id, token = await cloud.pair(csrf)
    server_agent = await cloud.agent(token)
    assert await server_agent.connect()
    browser = await cloud.control(robot_id)
    other = await cloud.control(robot_id)
    assert await browser.connect() and await other.connect()
    node.value.get_parameter = lambda name: SimpleNamespace(value=robot_id)
    node.value._manual_ingress = ManualIngress(robot_id, "command-boot", ttl=0.3)
    node.value._arbiter = Arbiter(robot_id, 300)
    node.value._motion_runtime = MotionRuntimeReceiver()
    node.value._motion_runtime.accept(runtime(10, robot_id=robot_id), b"t5", 10)

    def state() -> dict[str, Any]:
        return {
            "v": 1,
            "ok": True,
            "sampled_at": node.clock.now,
            "localized": False,
            "safety_clear": True,
            "manual_driving": False,
            "map_id": None,
            "reference_revision": None,
            "execution_status": "idle",
            "motion_runtime": node.value.motion_runtime(),
            "manual_authority": node.value.manual_view(),
        }

    core = AdapterCore(
        robot_id=robot_id,
        submit=node.value.submit,
        disconnect=node.value.disconnect,
        request_exit=lambda: None,
        state_provider=state,
        manual=node.value.manual_request,
    )

    class Link:
        epoch = 1

        def command_clock_valid(self) -> bool:
            return False  # Native manual never uses the old challenge/RTT clock qualifier.

        async def send_response(self, response: ResponseEnvelope) -> None:
            await server_agent.send(serialize_envelope(response))

        async def send_snapshot(self, snapshot: Any) -> None:
            await server_agent.send(
                serialize_envelope(
                    snapshot_envelope(
                        snapshot,
                        robot_id=robot_id,
                        session_id="agent",
                        connection_epoch=1,
                        request_id="snapshot",
                        seq=0,
                        expires_at=time.time() + 30,
                    )
                )
            )

    async with (
        ipc_pair(core.handle, monkeypatch) as requester,
        ipc_pair(core.handle, monkeypatch) as intent_client,
        Database(tmp_path / "car.db") as database,
    ):
        store = TaskStore(database, robot_id=robot_id)
        await store.prepare()
        waypoints = WaypointStore(database, robot_id=robot_id)
        await waypoints.prepare()
        journal = AgentJournal(database, robot_id=robot_id)
        await journal.prepare()
        navigation = IpcNavigation(requester, waypoints, tasks=store)
        _, executor = build_task_runtime(database, store, navigation, boot_id="wire")
        motion = TaskAwareMotion(cast(Any, requester), intent_client=intent_client)
        motion.bind_tasks(executor)
        settings = SettingsService(
            SettingsStore(database), simulation_settings_profile(), RuntimeGate()
        )
        await settings.prepare()
        motion.settings = settings
        agent: Agent
        snapshots = SnapshotState(
            IpcState(requester, clock=lambda: node.clock.now),
            store,
            waypoints,
            connected=lambda: agent.connected,
            epoch=lambda: agent.connection_epoch,
            manual_enabled=True,
            clock=lambda: node.clock.now,
        )
        agent = Agent(
            robot_id,
            connector=cast(Any, FakeConnector()),
            journal=journal,
            tasks=StoreTaskPort(store, robot_id=robot_id),
            home=FakeHome(),
            maps=FakeMaps(),
            state=snapshots,
            motion=motion,
            clock=cast(Any, SimpleNamespace(monotonic=lambda: node.clock.now)),
        )
        link = Link()
        agent._link, agent._epoch = cast(Any, link), 1
        await agent.publish_snapshot(robot_id)
        assert await browser.receive() is not None
        assert await other.receive() is not None

        async def request(
            kind: str, payload: dict[str, Any], seq: int, *, received: float | None = None
        ) -> ResponseEnvelope:
            frame = envelope(
                kind, payload, robot=robot_id, generation=None, seq=seq, request=str(seq)
            )
            await browser.send(serialize_envelope(frame))
            raw = await server_agent.receive()
            assert raw is not None
            admitted = validate_envelope(raw, robot_id)
            from senior_contracts.wire import ManualIntentEnvelope, RecoveryEnvelope, SimpleEnvelope

            assert isinstance(admitted, (ManualIntentEnvelope, RecoveryEnvelope, SimpleEnvelope))
            if admitted.type in {"control.acquire", "control.release", "drive.intent"}:
                assert admitted.controller_generation == 1
            if received is not None:
                admitted._received_steady = received
            await agent._receive_command(cast(Any, link), admitted)
            if kind == "drive.intent":
                from agent_fakes import until

                await until(lambda: node.value._manual_target is not None)
                assert browser._outbound.empty()
                return None
            response = await browser.receive()
            assert response is not None
            result = validate_envelope(response, robot_id)
            assert isinstance(result, ResponseEnvelope)
            return result

        prepared = await request("control.acquire", {}, 1)
        assert prepared.payload.ok and prepared.payload.data is not None
        authority = prepared.payload.data["authority_id"]
        assert (
            await other.receive() is None
        )  # Authority ACK is not broadcast to another controller.
        await agent.publish_snapshot(robot_id)
        owned_frame = await browser.receive()
        peer_frame = await other.receive()
        assert owned_frame is not None and peer_frame is not None
        owned = validate_envelope(owned_frame, robot_id)
        peer = validate_envelope(peer_frame, robot_id)
        assert owned.type == "snapshot" and peer.type == "snapshot"
        assert owned.payload.vehicle_ui is not None and peer.payload.vehicle_ui is not None
        assert owned.payload.vehicle_ui["manual_authority"]["authority_id"] == authority
        assert peer.payload.vehicle_ui["manual_authority"] is None
        assert b"manual_controller_generation" not in owned_frame
        assert b"manual_agent_epoch" not in owned_frame
        assert b"manual_controller_generation" not in peer_frame
        assert b"manual_agent_epoch" not in peer_frame
        original_wall = agent._wall_clock
        agent._wall_clock = lambda: original_wall() + 60  # Not a motion-clock qualifier.
        moved = await request(
            "drive.intent",
            {
                "authority_id": authority,
                "gesture": 1,
                "sequence": 1,
                "linear": 1.0,
                "turn": -0.5,
                "gear": "high",
            },
            2,
        )
        assert moved is None
        agent._wall_clock = original_wall
        assert node.output[-1].vx == 25 / 18 and node.output[-1].wz == -0.5
        released = await request("control.release", {"authority_id": authority}, 3)
        assert released.payload.ok and node.output[-1].close_reason == "release"
        estop = await request("safety.estop", {}, 4, received=9.7)
        assert estop.payload.ok and node.output[-1].request_id == "4"
        assert node.output[-1].received_steady == 9.7
        node.value.get_publishers_info_by_topic = lambda topic: [
            SimpleNamespace(
                node_name="motion_reporter",
                node_namespace="/",
                endpoint_gid=b"t5",
            )
        ]
        stopped = runtime(
            10,
            2,
            robot_id=robot_id,
            mode="estop",
            reason="software_estop",
            confirmed_estop_status="software_latched",
            estop_revision=1,
            output_generation=node.value._generation,
            allowed_actions=["estop_reset"],
        )
        node.value._on_motion_runtime(
            SimpleNamespace(data=stopped.model_dump_json()), {"publisher_gid": b"t5"}
        )
        await agent.publish_snapshot(robot_id)
        stopped_frame = await browser.receive()
        assert stopped_frame is not None
        stopped_view = validate_envelope(stopped_frame, robot_id)
        assert stopped_view.type == "snapshot" and stopped_view.payload.vehicle_ui is not None
        token = stopped_view.payload.vehicle_ui["estop_revision"]
        assert token > 1  # Transport token, translated to the exact raw T5 revision.
        reset = await request("safety.estop.reset", {"revision": token}, 5)
        assert reset.payload.ok and node.output[-1].close_reason == "estop_reset"
        assert node.output[-1].gesture == 1
        assert node.value._motion_runtime.value.mode == "estop"
        recovered = runtime(10, 3, robot_id=robot_id, estop_revision=2)
        node.value._on_motion_runtime(
            SimpleNamespace(data=recovered.model_dump_json()), {"publisher_gid": b"t5"}
        )
        await agent.publish_snapshot(robot_id)
        assert await browser.receive() is not None
        again = await request("control.acquire", {}, 6)
        assert again.payload.ok and again.payload.data is not None
        assert again.payload.data["authority_id"] != authority
        replayed = await request("safety.estop", {}, 4, received=10)
        assert replayed.payload.ok
        assert node.value.manual_view()["state"] == "prepared"
        fresh = await request("safety.estop", {}, 7, received=10)
        assert fresh.payload.ok
        assert node.value._pending_estop_request is not None
        await browser.send(
            serialize_envelope(
                envelope(
                    "control.acquire",
                    {},
                    robot=robot_id,
                    generation=999,
                    seq=4,
                    request="forged-context",
                )
            )
        )
        assert await browser.receive() is None
        retired = await server_agent.receive()
        assert retired is not None
        retirement = validate_envelope(retired, robot_id)
        assert retirement.type == "control.release" and retirement.payload.authority_id is None
        assert retirement.controller_generation == 1
        await agent._answer(cast(Any, link), retirement)
        assert node.value.manual_view()["state"] == "closed"
        await settings.close()


def test_expiry_and_controller_retirement_do_not_reopen(node: Any) -> None:
    prepared = call(node, envelope("control.acquire", {}))
    identity = prepared["authority_id"]
    sample = {
        "authority_id": identity,
        "gesture": 1,
        "sequence": 1,
        "linear": 0.4,
        "turn": 0.0,
        "gear": "low",
    }
    assert call(node, envelope("drive.intent", sample, seq=2), 10)["ok"] is True
    original = node.output[-1].steady_deadline
    node.clock.now = original
    node.value._motion_runtime.accept(runtime(original, 2), b"t5", original)
    node.value._publish_manual_target()
    assert node.output[-1].close_reason == "expiry"
    assert (
        call(node, envelope("drive.intent", {**sample, "sequence": 2}, seq=3), original)["ok"]
        is False
    )
    # Relay-generated release with no authority retires even a prepared context.
    call(node, envelope("control.release", {}, seq=4))
    assert call(node, envelope("control.acquire", {}, request="late", seq=5))["ok"] is False
    replacement = call(node, envelope("control.acquire", {}, request="new", seq=6, generation=2))
    assert replacement["state"] == "prepared" and replacement["authority_id"] != identity


def test_estop_ack_does_not_recover_until_real_t5_state_then_new_input(node: Any) -> None:
    prepared = call(node, envelope("control.acquire", {}))
    identity = prepared["authority_id"]
    node.value.submit(envelope("safety.estop", {}, generation=None))
    assert node.output[-1].close_reason == "estop"
    previous_barrier = vars(node.output[-1]).copy()
    node.value.publish_command()
    assert vars(node.output[-1]) == previous_barrier  # No same-sequence payload mutation.
    node.value.get_publishers_info_by_topic = lambda topic: [
        SimpleNamespace(
            node_name="motion_reporter",
            node_namespace="/",
            endpoint_gid=b"t5",
        )
    ]
    stopped = runtime(
        10,
        2,
        mode="estop",
        reason="software_estop",
        confirmed_estop_status="software_latched",
        estop_revision=1,
        output_generation=node.value._generation,
        allowed_actions=["estop_reset"],
    )
    node.value._on_motion_runtime(
        SimpleNamespace(data=stopped.model_dump_json(by_alias=True)), {"publisher_gid": b"t5"}
    )
    reset = call(node, envelope("safety.estop.reset", {"revision": 1}, generation=None))
    assert reset["ok"] is True and node.output[-1].close_reason == "estop_reset"
    during_report = call(node, envelope("control.acquire", {}, request="while-latched"))
    assert during_report["ok"] is True
    assert during_report["authority_id"] != identity
    recovered = runtime(10, 3, estop_revision=2)
    node.value._on_motion_runtime(
        SimpleNamespace(data=recovered.model_dump_json(by_alias=True)), {"publisher_gid": b"t5"}
    )
    assert node.value.navigation_qualified()
    assert node.value.manual_view()["authority_id"] == during_report["authority_id"]
    assert node.value.manual_view()["state"] == "prepared"
    fresh = call(node, envelope("control.acquire", {}, request="fresh"))
    assert fresh["state"] == "prepared" and fresh["authority_id"] != identity
    first = node.value._estop_requests["request"][0]
    original_fields = vars(first).copy()
    node.clock.now += 0.01
    node.value.submit(envelope("safety.estop", {}, generation=None), received_at=node.clock.now)
    assert vars(node.output[-1]) == original_fields
    assert node.value._motion_runtime.value.confirmed_estop_status == "clear"
    assert node.value.manual_view()["state"] == "prepared"
    node.value.submit(
        envelope("safety.estop", {}, generation=None, request="new-estop", seq=10),
        received_at=node.clock.now,
    )
    assert node.output[-1].request_id == "new-estop" and node.output[-1].gesture == 0
    assert node.output[-1].source_kind in {"manual", "idle"}
    assert node.output[-1].received_steady == node.clock.now
    generation = node.output[-1].generation
    node.value.submit(
        envelope("safety.estop", {}, generation=None, request="another-estop", seq=11),
        received_at=node.clock.now,
    )
    assert node.output[-1].request_id == "another-estop" and node.output[-1].generation > generation


async def test_native_acquire_consumes_durable_task_retirement_before_ipc(
    tmp_path: Path,
    node: Any,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    import importlib

    from test_task_ownership import start
    from vehicle_manager.agent.local_adapters import TaskAwareMotion
    from vehicle_manager.settings.service import RuntimeGate, SettingsService
    from vehicle_manager.settings.store import SettingsStore
    from vehicle_manager.storage.bindings import Database

    from vehicle_core.errors import DomainError, ErrorCode

    monkeypatch.syspath_prepend(str(Path("ros2/src/senior_robot").resolve()))
    AdapterCore = importlib.import_module("senior_robot.agent_adapter").AdapterCore
    core = AdapterCore(
        robot_id="robot",
        submit=node.value.submit,
        disconnect=node.value.disconnect,
        request_exit=lambda: None,
        manual=node.value.manual_request,
    )

    class Requester:
        async def request(self, kind: str, **fields: Any) -> dict[str, Any]:
            return cast(dict[str, Any], await core.handle({"type": kind, **fields}))

    async with Database(tmp_path / "takeover.db") as database:
        _, executor, navigation, _, identity = await start(database)
        assert navigation.progress is not None and executor.lifecycle is not None
        navigation.progress = navigation.progress.model_copy(
            update={"completed_action_ids": ["0:a"]}
        )
        settings = SettingsService(
            SettingsStore(database), simulation_settings_profile(), RuntimeGate()
        )
        await settings.prepare()
        motion = TaskAwareMotion(cast(Any, Requester()), intent_client=AsyncMock())
        motion.bind_tasks(executor)
        motion.settings = settings
        request = envelope("control.acquire", {}, request="task-takeover")
        with pytest.raises(DomainError) as blocked:
            await motion.manual(request)
        assert blocked.value.code is ErrorCode.UNAVAILABLE
        assert node.value.manual_view() is None
        state = await executor.lifecycle.store.load(identity)
        assert state.phase == "revoking" and state.stopped_for == "takeover"
        navigation.retired = True  # Controlled navigation port confirms its actual boundary.
        receipt = await motion.manual(request)
        assert receipt["state"] == "prepared"
        state = await executor.lifecycle.store.load(identity)
        assert state.phase == "paused" and state.completed == ("0:a",)
        await motion.manual(envelope("control.release", {"authority_id": receipt["authority_id"]}))
        assert (await executor.lifecycle.store.load(identity)).phase == "paused"
        assert len([effect for effect in navigation.effects if effect.kind == "plan"]) == 1
        await settings.close()


def test_missing_report_allows_prepare_without_fabricating_body_reference(node: Any) -> None:
    node.value._motion_runtime.value = None
    assert call(node, envelope("control.acquire", {}))["ok"] is True
    assert node.value.manual_view()["state"] == "prepared"
    assert node.value.motion_runtime() is None


def test_simulation_policy_is_explicit_and_caps_requested_speeds() -> None:
    from vehicle_manager.agent.production import AgentConfig
    from vehicle_manager.settings.models import DrivingSettings

    policy = simulation_settings_profile()
    assert policy.driving_defaults.high_linear_mps == 25 / 18
    with pytest.raises(ValueError):
        policy.validate_driving(
            DrivingSettings(
                low_linear_mps=1.5,
                high_linear_mps=1.5,
                low_angular_rps=1,
                high_angular_rps=1,
                reverse_linear_mps=0.3,
            )
        )
    with pytest.raises(ValueError):
        policy.validate_driving(
            policy.driving_defaults.model_copy(update={"reverse_linear_mps": 0.4})
        )
    config = {
        "SENIOR_ROBOT_ID": "robot",
        "SENIOR_CLOUD_URL": "wss://control.test/ws/agent",
        "SENIOR_DEVICE_TOKEN": "token",
        "SENIOR_DATABASE_PATH": "/tmp/car.db",
        "SENIOR_AGENT_SOCKET": "/tmp/adapter.sock",
    }
    assert not AgentConfig.from_environment(config).simulation_settings
    assert AgentConfig.from_environment(
        {**config, "SENIOR_SIMULATION_SETTINGS": "1"}
    ).simulation_settings


@pytest.mark.parametrize("mode,reason", [("execution_fault", "fault"), ("estop", "estop")])
async def test_t5_stop_during_feedback_retirement_cannot_auto_resume(
    tmp_path: Path,
    mode: str,
    reason: str,
) -> None:
    from dataclasses import replace
    from datetime import timedelta

    from senior_contracts.wire import RobotPosePayload
    from test_task_lifecycle_store import NOW
    from test_task_ownership import start
    from vehicle_manager.agent.local_state import RuntimeState
    from vehicle_manager.storage.bindings import Database

    async with Database(tmp_path / "stop-priority.db") as database:
        driver, executor, navigation, live, identity = await start(database)
        assert executor.lifecycle is not None
        await driver.tick(NOW + timedelta(seconds=60.5), 1, replace(live, localized=False))
        assert (await executor.lifecycle.store.load(identity)).phase == "revoking"
        now = time.monotonic()
        observation = runtime(
            now,
            mode=mode,
            healthy=False,
            allowed_actions=[],
            confirmed_estop_status="software_latched" if mode == "estop" else "clear",
        )
        view = RuntimeState(
            True,
            True,
            False,
            live.active_map,
            "running",
            "map",
            "map",
            False,
            robot_pose=RobotPosePayload(x=0.0, y=0.0, yaw=0.0),
            task_lifecycle_version=2,
            task_evidence_ready=True,
            motion_runtime=observation,
            motion_runtime_version=1,
        )
        navigation.retired = True
        await driver.tick(NOW + timedelta(seconds=61), 1.5, view.for_tasks(True))
        stopped = await executor.lifecycle.store.load(identity)
        assert stopped.phase == "paused" and stopped.stopped_for == "feedback"
        assert view.for_tasks(True).motion_halt_reason is None
        # A historic report does not turn feedback retirement into a new explicit stop.
        assert len([effect for effect in navigation.effects if effect.kind == "plan"]) == 1


async def test_native_websocket_connector_does_not_negotiate_old_motion_clock(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    import websockets.asyncio.client
    from vehicle_manager.agent.production_bindings import WebSocketConnector

    sent: list[str] = []

    class Socket:
        response = SimpleNamespace(headers={"x-connection-epoch": "4", "x-drive-timing": "1"})

        async def send(self, text: str) -> None:
            sent.append(text)

        async def close(self) -> None:
            pass

    async def connect(url: str, **kwargs: Any) -> Any:
        assert kwargs["additional_headers"] == {"Authorization": "Bearer trusted-device"}
        return Socket()

    monkeypatch.setattr(websockets.asyncio.client, "connect", connect)
    link = await WebSocketConnector("wss://control.test", "trusted-device", "robot")()
    await link.send_heartbeat()
    assert "drive_t1" not in json.loads(sent[-1])
    assert "drive_confirmation" not in json.loads(sent[-1])
    await link.close()


async def test_relay_coalesced_intents_settle_owner_without_exhausting_pending() -> None:
    from cloud.limits import CloudLimits
    from cloud.runtime_relay import Hub

    hub = Hub(CloudLimits(outbox_frames=4))
    agent = hub.attach_agent("robot")
    agent.motion_intent = True
    agent.heartbeat_at = time.monotonic()
    owner = hub.attach_controller("robot", "owner")
    peer = hub.attach_controller("robot", "peer")
    assert owner is not None and peer is not None
    assert json.loads(await owner.outbox.get())["type"] == "control.connection"
    assert json.loads(await peer.outbox.get())["type"] == "control.connection"
    for seq in range(1, 101):
        hub.handle_control_frame(
            owner,
            serialize_envelope(
                envelope(
                    "drive.intent",
                    {
                        "authority_id": "authority",
                        "gesture": 1,
                        "sequence": seq,
                        "linear": 1,
                        "turn": 0,
                        "gear": "low",
                    },
                    seq=seq,
                    request=str(seq),
                    generation=None,
                )
            ),
        )
        assert not owner.manual_requests
        assert agent.outbox.pending == 1
        assert owner.outbox.pending == 0
        assert not peer.manual_requests and peer.outbox.pending == 0
    latest = await agent.outbox.get()
    assert isinstance(latest, bytes) and validate_envelope(latest, "robot").request_id == "100"
    assert not owner.manual_requests and owner.outbox.pending == 0


@pytest.mark.parametrize("drop", ["release", "replace", "evict", "close"])
async def test_relay_dropped_intent_settles_only_its_original_owner(drop: str) -> None:
    from cloud.limits import CloudLimits
    from cloud.runtime_relay import Hub

    hub = Hub(CloudLimits(outbox_frames=1))
    agent = hub.attach_agent("robot")
    agent.motion_intent = True
    agent.heartbeat_at = time.monotonic()
    owner = hub.attach_controller("robot", "owner")
    peer = hub.attach_controller("robot", "peer")
    assert owner is not None and peer is not None
    assert json.loads(await owner.outbox.get())["type"] == "control.connection"
    assert json.loads(await peer.outbox.get())["type"] == "control.connection"
    hub.handle_control_frame(
        owner,
        serialize_envelope(
            envelope(
                "drive.intent",
                {
                    "authority_id": "authority",
                    "gesture": 1,
                    "sequence": 1,
                    "linear": 1,
                    "turn": 0,
                    "gear": "low",
                },
                generation=None,
            )
        ),
    )
    if drop == "release":
        hub.handle_control_frame(
            peer,
            serialize_envelope(
                envelope(
                    "control.release",
                    {"authority_id": "peer-authority"},
                    seq=2,
                    request="release",
                    generation=None,
                )
            ),
        )
    elif drop == "replace":
        replacement = envelope(
            "drive.intent",
            {
                "authority_id": "authority",
                "gesture": 1,
                "sequence": 2,
                "linear": 1,
                "turn": 0,
                "gear": "low",
            },
            seq=2,
            request="replacement",
        )
        assert agent.outbox.put(serialize_envelope(replacement), "drive.intent")
    elif drop == "evict":
        assert agent.outbox.put(b"unrelated", "heartbeat")
    else:
        agent.outbox.close(1000)
    assert not owner.manual_requests
    assert owner.outbox.pending == 0 and peer.outbox.pending == 0


@pytest.mark.parametrize("closing", ["control.release", "drive.stop", "safety.estop"])
async def test_agent_receive_discards_replaced_intents_without_receipts(
    tmp_path: Path, closing: str
):
    from agent_fakes import (
        FakeConnector,
        FakeHome,
        FakeLink,
        FakeMaps,
        FakeMotion,
        FakeState,
        FakeTasks,
        until,
    )
    from vehicle_manager.agent.production_bindings import AgentJournal
    from vehicle_manager.agent.runtime_bindings import Agent
    from vehicle_manager.storage.bindings import Database

    async with Database(tmp_path / "queued.db") as database:
        journal = AgentJournal(database, robot_id="robot")
        await journal.prepare()
        entered, release = asyncio.Event(), asyncio.Event()
        seen = []

        class HeldMotion(FakeMotion):
            async def send_intent(self, envelope, *, received_steady, deadline_steady):
                seen.append((envelope.request_id, received_steady, deadline_steady))
                entered.set()
                await release.wait()

        motion = HeldMotion()
        agent = Agent(
            "robot",
            connector=cast(Any, FakeConnector()),
            journal=journal,
            tasks=FakeTasks(),
            home=FakeHome(),
            maps=FakeMaps(),
            state=FakeState(),
            motion=motion,
            clock=cast(Any, SimpleNamespace(monotonic=lambda: 10.0)),
        )
        link = FakeLink(1)
        agent._link, agent._epoch = cast(Any, link), 1
        for seq in range(1, 4):
            sample = envelope(
                "drive.intent",
                dict(
                    authority_id="authority", gesture=1, sequence=seq, linear=1, turn=0, gear="low"
                ),
                seq=seq,
                request=str(seq),
            )
            sample._received_steady = 9.95
            await agent._receive_command(cast(Any, link), sample)
            if seq == 1:
                await asyncio.wait_for(entered.wait(), 1)
        assert link.responses == []
        assert agent._queued_intent[1].request_id == "3"
        await agent._receive_command(
            cast(Any, link),
            envelope(
                closing,
                {"authority_id": "authority"} if closing == "control.release" else {},
                seq=4,
                request="close",
                generation=1 if closing == "control.release" else None,
            ),
        )
        assert agent._queued_intent is None
        release.set()
        await agent._intent_task
        await until(lambda: any(r.request_id == "close" for r in link.responses))
        assert [r.request_id for r in link.responses] == ["close"]
        assert len(seen) == 1 and seen[0][0] == "1"
        assert seen[0][1] == 9.95 and seen[0][2] == pytest.approx(10.25)
        if agent._safety_task is not None:
            await agent._safety_task


def test_inflight_estop_retransmits_original_event_until_output_owner_confirms(node):
    def cannot_cleanup_yet():
        raise AssertionError("navigation cleanup must not supersede the in-flight estop")

    node.value._navigation_source = SimpleNamespace(qualified=cannot_cleanup_yet)
    request = envelope("safety.estop", {}, generation=None, request="inflight", seq=9)
    assert node.value.submit(request, received_at=10).accepted
    original = node.output[-1]
    node.value.disconnect(1)
    node.value.publish_command()
    assert node.output[-1] is original
    assert node.value._pending_estop_request is original
    node.value.get_publishers_info_by_topic = lambda _: [
        SimpleNamespace(node_name="motion_reporter", node_namespace="/", endpoint_gid=b"t5")
    ]
    clear = runtime(10, 2, output_generation=original.generation + 1)
    node.value._on_motion_runtime(SimpleNamespace(data=clear.model_dump_json()), {})
    assert node.value._pending_estop_request is original
    assert not node.value.task_motion_allowed(continuing=True)
    stopped = runtime(
        10,
        3,
        output_generation=original.generation,
        mode="estop",
        confirmed_estop_status="software_latched",
        estop_revision=1,
        allowed_actions=[],
    )
    node.value._on_motion_runtime(SimpleNamespace(data=stopped.model_dump_json()), {})
    assert node.value._pending_estop_request is None
    assert node.value._motion_runtime.value.mode == "estop"
    assert node.value.navigation_qualified()
    assert node.value.task_motion_allowed(continuing=True)


async def test_agent_queued_intent_keeps_original_300ms_cutoff(tmp_path):
    from agent_fakes import (
        FakeConnector,
        FakeHome,
        FakeLink,
        FakeMaps,
        FakeMotion,
        FakeState,
        FakeTasks,
    )
    from vehicle_manager.agent.production_bindings import AgentJournal
    from vehicle_manager.agent.runtime_bindings import Agent
    from vehicle_manager.storage.bindings import Database

    clock = SimpleNamespace(now=10.0)
    entered, released = asyncio.Event(), asyncio.Event()
    delivered = []

    class HeldTransport(FakeMotion):
        async def send_intent(self, envelope, *, received_steady, deadline_steady):
            delivered.append((envelope.seq, received_steady, deadline_steady))
            entered.set()
            await released.wait()

    async with Database(tmp_path / "original-deadline.db") as database:
        journal = AgentJournal(database, robot_id="robot")
        await journal.prepare()
        agent = Agent(
            "robot",
            connector=cast(Any, FakeConnector()),
            journal=journal,
            tasks=FakeTasks(),
            home=FakeHome(),
            maps=FakeMaps(),
            state=FakeState(),
            motion=HeldTransport(),
            clock=cast(Any, SimpleNamespace(monotonic=lambda: clock.now)),
        )
        link = FakeLink(1)
        agent._link, agent._epoch = cast(Any, link), 1
        try:
            for seq, received in ((1, 10.0), (2, 10.1)):
                clock.now = received
                sample = envelope(
                    "drive.intent",
                    dict(
                        authority_id="authority",
                        gesture=1,
                        sequence=seq,
                        linear=1,
                        turn=0,
                        gear="low",
                    ),
                    seq=seq,
                )
                sample._received_steady = received
                await agent._receive_command(cast(Any, link), sample)
                if seq == 1:
                    await asyncio.wait_for(entered.wait(), 1)
            assert agent._queued_intent[-1] == pytest.approx(10.4)
            clock.now = 10.4
            released.set()
            await agent._intent_task
            assert delivered == [(1, 10.0, 10.3)]
            assert agent._queued_intent is None and link.responses == []
        finally:
            released.set()
            await agent._release()


async def test_heartbeat_window_keeps_manual_delivery_then_reconnect_retires_old_context(
    node: Any, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    from agent_fakes import FakeHome, FakeMaps, FakeState, FakeTasks, until
    from heartbeat_test_support import HeartbeatClock, RelaySocket, relay_hub
    from test_task_ownership import Navigation
    from vehicle_manager.agent.local_adapters import TaskAwareMotion
    from vehicle_manager.agent.production import build_task_runtime
    from vehicle_manager.agent.production_bindings import AgentJournal, WebSocketLink
    from vehicle_manager.agent.runtime_bindings import Agent
    from vehicle_manager.settings.service import RuntimeGate, SettingsService
    from vehicle_manager.settings.store import SettingsStore
    from vehicle_manager.storage.bindings import Database
    from vehicle_manager.storage.domain_bindings import TaskStore

    clock = HeartbeatClock(monkeypatch)
    hub = relay_hub(monkeypatch, clock)
    first = RelaySocket(hub, "robot")
    link = WebSocketLink(first, "robot", first.connection.epoch, clock=clock.monotonic)
    connections: asyncio.Queue[Any] = asyncio.Queue()
    connections.put_nowait(link)
    disabled: list[int] = []

    # Keep the actual IPC framing and native manual owner from this existing fixture.
    AdapterCore = importlib.import_module("senior_robot.agent_adapter").AdapterCore

    core = AdapterCore(
        robot_id="robot",
        submit=node.value.submit,
        disconnect=node.value.disconnect,
        request_exit=lambda: None,
        manual=node.value.manual_request,
    )

    async def tracked_handle(message: dict[str, Any]) -> dict[str, Any]:
        if message["type"] == "disable_motion":
            disabled.append(message["connection_epoch"])
        return await core.handle(message)

    async with (
        ipc_pair(tracked_handle, monkeypatch) as client,
        ipc_pair(tracked_handle, monkeypatch) as intent_client,
        Database(tmp_path / "heartbeat-manual.db") as database,
    ):
        journal = AgentJournal(database, robot_id="robot")
        await journal.prepare()
        settings = SettingsService(
            SettingsStore(database),
            simulation_settings_profile(),
            RuntimeGate(),
        )
        await settings.prepare()
        motion = TaskAwareMotion(client, intent_client=intent_client)
        motion.settings = settings
        tasks = TaskStore(database, robot_id="robot")
        await tasks.prepare()
        _, executor = build_task_runtime(database, tasks, Navigation(), boot_id="heartbeat")
        motion.bind_tasks(executor)
        agent = Agent(
            "robot",
            connector=connections.get,
            journal=journal,
            tasks=FakeTasks(),
            home=FakeHome(),
            maps=FakeMaps(),
            state=FakeState(),
            motion=motion,
            clock=clock,
            heartbeat_ms=20,
            retry_delay_ms=1,
        )
        runner = asyncio.create_task(agent.run())
        controller = hub.attach_controller("robot", "controller-session")
        assert controller is not None

        async def command(kind: str, payload: dict[str, Any], seq: int, epoch: int) -> None:
            frame = envelope(kind, payload, generation=None, seq=seq, request=str(seq))
            frame.connection_epoch = epoch
            hub.handle_control_frame(controller, serialize_envelope(frame))

        async def response() -> Any:
            while True:
                raw = await controller.outbox.get()
                assert isinstance(raw, bytes)
                data = json.loads(raw)
                if data["type"] == "response":
                    return data["payload"]

        try:
            await until(
                lambda: (
                    any(frame["type"] == "heartbeat.request" for frame in first.sent)
                    and not link._pending
                )
            )
            first.drop_replies = True
            await command("control.acquire", {}, 1, 1)
            prepared = await asyncio.wait_for(response(), 1)
            assert prepared["ok"]
            authority = prepared["data"]["authority_id"]
            # Fresh inputs span both retired thresholds, while subsequent replies are missing.
            for seq, now in enumerate([10 + i / 10 for i in range(1, 50)] + [14.999], start=2):
                clock.advance(now)
                node.clock.now = now
                node.value._motion_runtime.accept(runtime(now, seq), b"t5", now)
                await command(
                    "drive.intent",
                    {
                        "authority_id": authority,
                        "gesture": 1,
                        "sequence": seq,
                        "linear": 0.4,
                        "turn": 0.0,
                        "gear": "low",
                    },
                    seq,
                    1,
                )
                await until(lambda seq=seq: bool(node.output) and node.output[-1].sequence == seq)
                assert node.output[-1].vx > 0 and not disabled
                assert not first.closed and agent.connection_epoch == 1
            clock.advance(15.0)
            await until(lambda: first.closed and disabled == [1])
            assert not agent.connected
            assert node.value.manual_view()["state"] == "closed"
            second = RelaySocket(hub, "robot")
            connections.put_nowait(
                WebSocketLink(
                    second,
                    "robot",
                    second.connection.epoch,
                    clock=clock.monotonic,
                )
            )
            await until(lambda: agent.connection_epoch == 2 and hub.is_online("robot"))
            node.clock.now = 15.0
            node.value._motion_runtime.accept(runtime(15.0, 100), b"t5", 15.0)
            count = len(node.output)
            await command(
                "drive.intent",
                {
                    "authority_id": authority,
                    "gesture": 1,
                    "sequence": 100,
                    "linear": 0.4,
                    "turn": 0.0,
                    "gear": "low",
                },
                100,
                1,
            )
            # Intent has no per-sample response, including rejection.
            await asyncio.sleep(0.02)
            assert len(node.output) == count
            # Even a current transport epoch cannot resurrect the old authority.
            await command(
                "drive.intent",
                {
                    "authority_id": authority,
                    "gesture": 1,
                    "sequence": 101,
                    "linear": 0.4,
                    "turn": 0.0,
                    "gear": "low",
                },
                101,
                2,
            )
            await asyncio.sleep(0.02)
            assert len(node.output) == count
            await command("control.acquire", {}, 102, 2)
            prepared = await asyncio.wait_for(response(), 1)
            assert prepared["ok"]
            fresh_authority = prepared["data"]["authority_id"]
            assert fresh_authority != authority
            await command(
                "drive.intent",
                {
                    "authority_id": fresh_authority,
                    "gesture": 1,
                    "sequence": 103,
                    "linear": 0.4,
                    "turn": 0.0,
                    "gear": "low",
                },
                103,
                2,
            )
            await until(lambda: len(node.output) > count and node.output[-1].vx > 0)
        finally:
            agent.request_shutdown()
            await asyncio.wait_for(runner, 2)
            await settings.close()
