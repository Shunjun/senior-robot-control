"""Review regressions at the actual relay and optional SQLite initialization boundaries."""

import asyncio
import time
from pathlib import Path
from typing import Any

import pytest
from agent_fakes import FakeConnector, FakeLink, until
from test_manual_wire import envelope, runtime

from senior_control.agent import composition
from senior_control.agent.connection import HeartbeatFrame
from senior_control.agent.main import Agent
from senior_control.cloud.limits import CloudLimits
from senior_control.cloud.relay import Hub
from senior_control.protocol import ResponseEnvelope, serialize_envelope, validate_envelope
from senior_control.runtime.ipc import IpcClient


def frame(robot: str, kind: str, payload: dict[str, Any], seq: int) -> bytes:
    return serialize_envelope(
        envelope(kind, payload, robot=robot, generation=None, seq=seq, request=f"wire-{seq}")
    )


def relay() -> tuple[Hub, Any, Any, Any]:
    hub = Hub(CloudLimits(outbox_frames=4))
    agent = hub.attach_agent("robot")
    agent.heartbeat_at = time.monotonic()
    owner, peer = hub.attach_controller("robot", "owner"), hub.attach_controller("robot", "peer")
    assert owner is not None and peer is not None
    return hub, agent, owner, peer


async def failure(owner: Any, sequence: int) -> None:
    raw = await asyncio.wait_for(owner.outbox.get(), 0.3)
    assert isinstance(raw, bytes)
    reply = validate_envelope(raw, "robot")
    assert isinstance(reply, ResponseEnvelope)
    assert reply.request_id == f"wire-{sequence}"
    assert reply.payload.request_type == "settings.get"
    assert reply.payload.ok is False and reply.payload.error is not None
    assert reply.controller_generation is None


async def test_settings_refused_enqueue_settles_owner_without_leaking_eight_slots() -> None:
    hub, agent, owner, peer = relay()
    for seq in range(100, 104):
        hub.handle_control_frame(peer, frame("robot", "safety.estop", {}, seq))
    assert agent.outbox.pending == 4
    for seq in range(1, 11):
        hub.handle_control_frame(owner, frame("robot", "settings.get", {}, seq))
        await failure(owner, seq)
        assert not owner.settings_requests
        assert peer.outbox.pending == 0
    # Once capacity is free, the same authenticated owner can issue a real read.
    await agent.outbox.get()
    hub.handle_control_frame(owner, frame("robot", "settings.get", {}, 11))
    assert len(owner.settings_requests) == 1
    assert agent.outbox.pending == 4


@pytest.mark.parametrize("drop", ["protected", "close"])
async def test_settings_discard_settles_only_the_original_connection(drop: str) -> None:
    hub, agent, owner, peer = relay()
    # Both controllers deliberately use the same browser identifiers. The cloud's
    # controller generation must select the actual owner, including on discard.
    for controller in (owner, peer):
        hub.handle_control_frame(controller, frame("robot", "settings.get", {}, 1))
    for seq in (100, 101):
        hub.handle_control_frame(peer, frame("robot", "safety.estop", {}, seq))
    if drop == "protected":
        hub.handle_control_frame(peer, frame("robot", "safety.estop", {}, 102))
        await failure(owner, 1)
        assert not owner.settings_requests
        assert len(peer.settings_requests) == 1 and peer.outbox.pending == 0
    else:
        agent.outbox.close(1000)
        await failure(owner, 1)
        await failure(peer, 1)
        assert not owner.settings_requests and not peer.settings_requests


async def test_directory_log_database_does_not_abort_composed_control(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    log_path = tmp_path / "logs" / "vehicle-logs.sqlite3"
    log_path.mkdir(parents=True)
    link = FakeLink(1)
    connector = FakeConnector(link)
    agents: list[Agent] = []
    heartbeat_seen = asyncio.Event()
    monkeypatch.setattr(composition, "WebSocketConnector", lambda *args, **kwargs: connector)
    monkeypatch.setattr(Agent, "install_signal_handlers", lambda self: agents.append(self))

    async def ipc(self: IpcClient, kind: str, **fields: Any) -> dict[str, Any]:
        # Only the external ROS IPC peer is replaced. Real composition, Agent.run,
        # supervisor, SettingsService, SQLite and optional log open remain intact.
        if kind == "state.get":
            now = time.monotonic()
            return {
                "v": 1,
                "ok": True,
                "sampled_at": now,
                "localized": False,
                "safety_clear": True,
                "manual_driving": False,
                "map_id": None,
                "reference_revision": None,
                "execution_status": "idle",
                "motion_runtime": runtime(now, robot_id="robot").model_dump(),
                "motion_runtime_version": 1,
                "manual_protocol_version": 1,
            }
        if kind == "manual":
            return {"ok": True, "authority_id": "test-authority", "state": "prepared"}
        if kind == "server_heartbeat":
            heartbeat_seen.set()
        if kind == "drive_feedback":
            return {"ok": True, "feedback": []}
        assert kind in {"identity", "server_heartbeat", "disable_motion", "exit"}
        return {"ok": True}

    monkeypatch.setattr(IpcClient, "request", ipc)
    config = composition.AgentConfig(
        "robot",
        "wss://cloud.test/ws/agent",
        "device",
        tmp_path / "vehicle.db",
        Path("/tmp/log-review.sock"),
        simulation_settings=True,
    )
    running = asyncio.create_task(composition.run_agent(config))
    try:
        for _ in range(100):
            if running.done():
                await running  # Surface the actual initialization exception in RED.
            if link.snapshots:
                break
            await asyncio.sleep(0.01)
        assert agents and agents[0].connected and link.snapshots
        snapshot = link.snapshots[-1]
        assert snapshot.vehicle_ui is not None
        assert "settings.v1" in snapshot.vehicle_ui["capabilities"]
        assert "logs.v1" not in snapshot.vehicle_ui["capabilities"]
        await until(lambda: link.heartbeats > 0)
        settings = agents[0]._settings
        assert settings is not None and settings._timer is not None
        # task_loop uses this exact gate. Hold it to expose the lock boundary:
        # acquire waits, while the real Agent receiver still delivers a heartbeat
        # through its separate IPC client to the controlled ROS boundary.
        async with settings.runtime.lock:
            await link.inbox.put(envelope("control.acquire", {}, request="gate-acquire"))
            await link.inbox.put(HeartbeatFrame("robot", time.monotonic() + 2))
            await asyncio.wait_for(heartbeat_seen.wait(), 0.3)
            assert not link.responses
        await until(lambda: bool(link.responses))
        assert link.responses[0].request_id == "gate-acquire"
        assert link.responses[0].payload.ok
        agents[0].request_shutdown()
        assert await asyncio.wait_for(running, 2) == 0
        assert settings._timer is None and link.closed
        assert log_path.is_dir()  # The optional failure never deletes or repairs it.
    finally:
        if not running.done():
            running.cancel()
        await asyncio.gather(running, return_exceptions=True)
