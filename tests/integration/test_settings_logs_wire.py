"""Actual authenticated ASGI/Agent/services/SQLite; IPC runtime is a controlled test boundary.

No ROS, VM, server or physical motion is started by these tests.
"""

import asyncio
import json
import logging
import time
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
from typing import Any, cast

import pytest
from agent_fakes import FakeConnector, FakeHome, FakeMaps, FakeMotion, FakeState, FakeTasks
from test_cloud import ORIGIN, Cloud, WsClient
from test_cloud import cloud as cloud
from test_manual_wire import runtime

from senior_control.agent.journal import AgentJournal
from senior_control.agent.logs_wire import AgentLogConnection, AgentLogHandler, CloudLogGrants
from senior_control.agent.main import Agent
from senior_control.agent.ports import RuntimeState
from senior_control.agent.settings_wire import SettingsRuntime
from senior_control.protocol import ResponseEnvelope, serialize_envelope, validate_envelope
from senior_control.settings.models import SettingsRequest, simulation_settings_profile
from senior_control.settings.service import RuntimeGate, SettingsService
from senior_control.settings.store import SettingsStore
from senior_control.storage.database import Database
from senior_control.tasks import TaskStore
from senior_control.tasks.lifecycle_store import LifecycleStore
from senior_control.vehicle_logs.models import LogInput, LogQuery
from senior_control.vehicle_logs.service import VehicleLogService
from senior_control.vehicle_logs.store import VehicleLogStore


def frame(robot: str, kind: str, payload: dict[str, Any], seq: int) -> bytes:
    return json.dumps(
        {
            "v": 1,
            "robot_id": robot,
            "session_id": "settings-viewer",
            "connection_epoch": 1,
            "request_id": f"wire-{seq}",
            "seq": seq,
            "expires_at": time.time() + 30,
            "type": kind,
            "payload": payload,
        }
    ).encode()


async def settings_rig(
    database: Database, robot: str
) -> tuple[SettingsService, SettingsRuntime, Any]:
    store = TaskStore(database, robot_id=robot)
    await store.prepare()
    lifecycle = LifecycleStore(store, boot_id="wire")
    await lifecycle.prepare()
    live = RuntimeState(False, True, False, None, "idle", motion_runtime=runtime(time.monotonic()))
    source = SimpleNamespace(value=live)

    async def read() -> RuntimeState:
        return cast(RuntimeState, source.value)

    source.read = read
    observer = SettingsRuntime(cast(Any, source), lifecycle)
    service = SettingsService(
        SettingsStore(database),
        simulation_settings_profile(),
        RuntimeGate(observer.check, refresh=observer.refresh),
    )
    await service.prepare()
    return service, observer, source


async def test_authenticated_settings_apply_read_result_and_reopen(
    cloud: Cloud, tmp_path: Path
) -> None:
    csrf = await cloud.login()
    robot, token = await cloud.pair(csrf)
    server = await cloud.agent(token)
    browser, other = await cloud.control(robot), await cloud.control(robot)
    assert await server.connect() and await browser.connect() and await other.connect()
    path = tmp_path / "vehicle.db"
    async with Database(path) as database:
        settings, observer, source = await settings_rig(database, robot)
        journal = AgentJournal(database, robot_id=robot)
        await journal.prepare()

        class Link:
            epoch = 1

            async def send_response(self, response: ResponseEnvelope) -> None:
                await server.send(serialize_envelope(response))

        link = Link()
        agent = Agent(
            robot,
            connector=FakeConnector(),
            journal=journal,
            tasks=FakeTasks(),
            home=FakeHome(),
            maps=FakeMaps(),
            state=FakeState(),
            motion=FakeMotion(),
            settings=settings,
        )
        agent._link, agent._epoch = cast(Any, link), 1

        async def request(kind: str, payload: dict[str, Any], seq: int) -> Any:
            source.value = replace(
                source.value, motion_runtime=runtime(time.monotonic(), robot_id=robot)
            )
            await browser.send(frame(robot, kind, payload, seq))
            raw = await server.receive()
            assert raw is not None
            await agent._receive_command(cast(Any, link), validate_envelope(raw, robot))
            raw = await browser.receive()
            assert raw is not None
            response = validate_envelope(raw, robot)
            assert isinstance(response, ResponseEnvelope) and response.payload.ok
            assert await other.receive(0.01) is None
            return response.payload.data

        initial = await request("settings.get", {}, 1)
        assert initial["can_apply_driving"] is True
        values = initial["snapshot"]["driving"] | {"low_linear_mps": 0.4}
        payload = {
            "request_id": "save",
            "expected_version": 0,
            "group": "driving",
            "values": values,
            "restore_defaults": False,
        }
        receipt = await request("settings.apply", payload, 2)
        assert receipt["reason"] == "applied"
        assert await request("settings.apply", payload, 3) == receipt
        assert await request("settings.result", {"request_id": "save"}, 4) == receipt
        assert await request("settings.result", {"request_id": "missing"}, 5) is None
        assert (await request("settings.get", {}, 6))["snapshot"]["driving"] == values
        conflict = await request("settings.apply", payload | {"request_id": "new"}, 7)
        assert conflict["reason"] == "version_conflict"
        # Logging is independent of stationarity and consumes the same settings owner.
        source.value = replace(source.value, manual_driving=True)
        logging_values = initial["snapshot"]["logging"] | {
            "level": "ERROR",
            "retention_seconds": 3600,
        }
        saved_logging = await request(
            "settings.apply",
            {
                "request_id": "log-policy",
                "expected_version": 0,
                "group": "logging",
                "values": logging_values,
                "restore_defaults": False,
            },
            8,
        )
        assert saved_logging["reason"] == "applied"
        await settings.close()
    async with Database(path) as database:
        reopened, _, _ = await settings_rig(database, robot)
        assert (await reopened.get()).driving.low_linear_mps == 0.4
        assert (await reopened.get()).logging.level == "ERROR"
        assert await reopened.result("save") is not None
        await reopened.close()


@pytest.mark.parametrize("boundary", ["stale", "future", "released_not_still", "prepared", "task"])
async def test_real_runtime_gate_rejects_unsafe_write(tmp_path: Path, boundary: str) -> None:
    async with Database(tmp_path / "vehicle.db") as database:
        service, observer, source = await settings_rig(database, "robot")
        now = time.monotonic()
        if boundary == "stale":
            source.value = replace(source.value, motion_runtime=runtime(now - 1))
        elif boundary == "future":
            source.value = replace(source.value, motion_runtime=runtime(now + 1))
        elif boundary == "released_not_still":
            source.value = replace(source.value, motion_runtime=runtime(now, stationary=False))
        elif boundary == "prepared":
            source.value = replace(source.value, manual_authority={"state": "prepared"})
        else:
            source.value = replace(source.value, motion_runtime=runtime(now, mode="task"))
        result = await service.apply(
            SettingsRequest(
                "blocked", 0, "driving", simulation_settings_profile().driving_defaults.model_dump()
            )
        )
        assert result.reason == "stop_required"
        await service.close()


class LogSocket:
    def __init__(self, peer: WsClient) -> None:
        self.peer = peer

    def __aiter__(self) -> "LogSocket":
        return self

    async def __anext__(self) -> str:
        raw = await self.peer.receive(5)
        if raw is None:
            raise StopAsyncIteration
        return raw.decode()

    async def send(self, raw: str) -> None:
        await self.peer.send(raw.encode())


async def log_peer(cloud: Cloud, path: str, robot: str, token: str | None = None) -> WsClient:
    headers = (
        {"Authorization": f"Bearer {token}"}
        if token
        else {
            "Origin": ORIGIN,
            "Cookie": cloud.cookie_header(),
        }
    )
    peer = WsClient(cloud.app, path, headers, f"robot_id={robot}")
    cloud.sockets.append(peer)
    assert await peer.connect()
    return peer


async def test_authenticated_real_agent_logs_query_follow_pause_download_and_revocation(
    cloud: Cloud, tmp_path: Path
) -> None:
    csrf = await cloud.login()
    robot, token = await cloud.pair(csrf)
    server = await log_peer(cloud, "/ws/agent/logs", robot, token)
    async with (
        Database(tmp_path / "vehicle.db") as database,
        VehicleLogStore(tmp_path / "logs") as store,
    ):
        settings, _, _ = await settings_rig(database, robot)
        grants = CloudLogGrants()
        service = VehicleLogService(store, robot, grants, lambda: settings.published_snapshot)
        await service.start()
        connection = AgentLogConnection(service, grants)
        serving = asyncio.create_task(connection.serve(LogSocket(server)))
        handler = AgentLogHandler(service)
        record = logging.LogRecord(
            "senior_control.agent",
            logging.INFO,
            "",
            1,
            "actual Agent event token=private",
            (),
            None,
        )
        record.event, record.request_id = "agent_command_result", "wire-op"
        handler.emit(record)
        await service.flush()
        query = LogQuery().model_dump(mode="json")

        async def operation(action: str, values: dict[str, Any]) -> tuple[WsClient, Any]:
            peer = await log_peer(cloud, "/ws/logs", robot)
            await peer.send(json.dumps({"action": action, "query": values}).encode())
            raw = await peer.receive(1)
            assert raw is not None
            result = json.loads(raw)
            assert result["ok"] is True
            return peer, result["data"]

        peer, page = await operation("query", query)
        assert page["entries"][0]["request_id"] == "wire-op"
        assert "private" not in json.dumps(page)
        await peer.aclose()
        follow, page = await operation("follow", query)
        after = page["next_cursor"]
        service.enqueue(
            LogInput("INFO", "senior_control.agent", "next event", source_utc=time.time())
        )
        await service.flush()
        raw = await follow.receive(1)
        assert raw is not None and json.loads(raw)["data"]["next_cursor"] > after
        after = json.loads(raw)["data"]["next_cursor"]
        await follow.aclose()  # UI pause owns this close; source collection continues.
        service.enqueue(
            LogInput("INFO", "senior_control.agent", "during pause", source_utc=time.time())
        )
        await service.flush()
        resumed, page = await operation("follow", query | {"after": after})
        assert [e["summary"] for e in page["entries"]] == ["during pause"]
        export, download = await operation("download", query | {"module": "senior_control.agent"})
        assert len(download["data"].splitlines()) == 3
        assert download["has_more"] is False
        await export.aclose()
        for index in range(20):
            service.enqueue(LogInput("INFO", "segment", f"segment {index}", source_utc=time.time()))
        await service.flush()
        first, segment = await operation("download", query | {"module": "segment", "limit": 1})
        assert segment["has_more"] is True and len(segment["data"].splitlines()) == 16
        second, tail = await operation(
            "download",
            query
            | {
                "module": "segment",
                "limit": 1,
                "after": segment["next_cursor"],
            },
        )
        assert tail["has_more"] is False and len(tail["data"].splitlines()) == 4
        await first.aclose()
        await second.aclose()
        unsafe = await log_peer(cloud, "/ws/logs", robot)
        await unsafe.send(
            json.dumps({"action": "download", "query": query, "path": "/etc/passwd"}).encode()
        )
        assert await unsafe.receive(1) is None and unsafe.closed
        denied = WsClient(cloud.app, "/ws/logs", {"Origin": ORIGIN}, f"robot_id={robot}")
        cloud.sockets.append(denied)
        assert not await denied.connect()
        # Consume the already-authorized segment event sent before revocation.
        assert await resumed.receive(1) is not None
        # Actual session revoke prevents later delivery and closes idle followers.
        response = await cloud.client.delete("/api/session", headers={"X-CSRF-Token": csrf})
        assert response.status_code == 204
        assert await resumed.receive(1.5) is None and resumed.closed
        await server.aclose()
        await asyncio.wait_for(serving, 1)
        assert not grants.active
        await service.close()
        await settings.close()


async def test_settings_gate_uses_real_framed_ipc_and_durable_task_phase(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    from test_manual_wire import ipc_pair
    from test_task_lifecycle_store import CONTEXT, activate, setup_store

    from senior_control.agent.ports import IpcState
    from senior_control.tasks.lifecycle import Command

    sample = {"expired": False}

    async def state_reply(request: dict[str, Any]) -> dict[str, Any]:
        assert request["type"] == "state.get"
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
            "motion_runtime": runtime(now - 1 if sample["expired"] else now).model_dump(),
            "motion_runtime_version": 1,
            "manual_protocol_version": 1,
        }

    async with (
        Database(tmp_path / "vehicle.db") as database,
        ipc_pair(state_reply, monkeypatch) as ipc,
    ):
        lifecycle, task = await setup_store(database)
        observer = SettingsRuntime(IpcState(ipc), lifecycle)
        service = SettingsService(
            SettingsStore(database),
            simulation_settings_profile(),
            RuntimeGate(observer.check, refresh=observer.refresh),
        )
        await service.prepare()

        async def write(identity: str, version: int) -> Any:
            return await service.apply(
                SettingsRequest(
                    identity,
                    version,
                    "driving",
                    simulation_settings_profile().driving_defaults.model_dump(),
                )
            )

        assert (await write("idle", 0)).reason == "applied"
        sample["expired"] = True
        assert (await write("expired", 1)).reason == "stop_required"
        sample["expired"] = False
        await activate(lifecycle, task)
        assert (await write("running", 1)).reason == "stop_required"
        before = await lifecycle.load("execution")
        stopped = await lifecycle.command(
            Command("pause", "pause", "execution"), lambda: CONTEXT, expected_version=before.version
        )
        assert (await write("revoking", 1)).reason == "stop_required"
        effect = stopped.effects[0]
        await lifecycle.claim_effect("execution", effect.effect_id)
        paused = await lifecycle.acknowledge(
            "execution",
            effect.effect_id,
            effect.epoch,
            True,
            lambda: CONTEXT,
            expected_version=stopped.state.version,
        )
        assert paused.state.phase == "paused"
        assert (await write("paused", 1)).reason == "applied"
        assert (await lifecycle.load("execution")).phase == "paused"
        await service.close()


async def test_log_policy_and_restore_use_the_settings_owner(tmp_path: Path) -> None:
    async with (
        Database(tmp_path / "vehicle.db") as database,
        VehicleLogStore(tmp_path / "logs") as store,
    ):
        settings, _, _ = await settings_rig(database, "robot")
        utc = [time.time()]
        settings._utc_now = lambda: utc[0]
        settings.start_debug_restore()
        grants = CloudLogGrants()
        service = VehicleLogService(store, "robot", grants, lambda: settings.published_snapshot)
        await service.start()
        values = settings.published_snapshot.logging.model_dump() | {"level": "ERROR"}
        assert (
            await settings.apply(SettingsRequest("error", 0, "logging", values))
        ).reason == "applied"
        info = LogInput("INFO", "senior_control.agent", "filtered", source_utc=time.time())
        assert service.enqueue(info) is False
        values |= {"debug_enabled": True, "debug_restore_seconds": 60}
        assert (
            await settings.apply(SettingsRequest("debug", 1, "logging", values))
        ).reason == "applied"
        assert service.enqueue(info) is True
        utc[0] += 61
        await asyncio.sleep(0.3)  # Existing sole owner timer, no connected browser.
        assert settings.published_snapshot.logging.debug_enabled is False
        assert settings.published_snapshot.logging.level == "ERROR"
        assert service.enqueue(info) is False
        await service.close()
        await settings.close()
