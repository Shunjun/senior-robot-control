"""Task 13 cloud relay contract: single admin, pairing, device sockets, Origin and CSRF.

Interface assumed by these tests (the stubs only declare it):

* ``create_app(settings, *, database, admin_password_hash, allowed_origins)`` is a
  synchronous factory. Nothing starts until the ASGI lifespan runs.
* ``POST /api/session {"password"}`` -> 200 ``{"csrf_token"}`` and a session cookie.
  Every write after login carries ``X-CSRF-Token``.
* ``POST /api/pairing-codes`` -> 201 ``{"code"}``.
* ``POST /api/devices/pair {"code", "name"}`` -> 200 ``{"robot_id", "device_token"}``;
  a second redemption of one code -> 409.
* ``GET /api/robots`` -> 200 ``{"robots": [{"id", "name"}]}``;
  ``DELETE /api/robots/{id}`` -> 204 and revokes the device credential.
* ``/ws/agent`` takes ``Authorization: Bearer <device_token>``. The n-th live socket
  of one robot owns epoch n; frames from older sockets are dropped.
* ``/ws/control?robot_id=<id>`` takes the session cookie plus an allowed ``Origin``.
"""

from __future__ import annotations

import asyncio
import json
import time
from collections.abc import AsyncIterator, MutableMapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import httpx
import pytest
from fastapi import FastAPI
from pydantic import SecretStr

from senior_control.cloud.app import create_app
from senior_control.cloud.auth import hash_password
from senior_control.config import CloudSettings
from senior_control.errors import ErrorCode
from senior_control.protocol import (
    EmptyPayload,
    ResponseEnvelope,
    ResponsePayload,
    SimpleEnvelope,
    serialize_envelope,
    validate_envelope,
)
from senior_control.storage.database import Database

ORIGIN = "https://cloud.test"
PASSWORD = "correct horse battery staple"
SOCKET_WAIT = 0.3


# ---------------------------------------------------------------- ASGI WebSocket driver


class WsClient:
    """Minimal in-process ASGI WebSocket peer; no server, no network."""

    def __init__(self, app: FastAPI, path: str, headers: dict[str, str], query: str = "") -> None:
        self._app = app
        self._inbound: asyncio.Queue[MutableMapping[str, Any]] = asyncio.Queue()
        self._outbound: asyncio.Queue[MutableMapping[str, Any]] = asyncio.Queue()
        self._scope: dict[str, Any] = {
            "type": "websocket",
            "asgi": {"version": "3.0"},
            "scheme": "wss",
            "http_version": "1.1",
            "path": path,
            "raw_path": path.encode(),
            "root_path": "",
            "query_string": query.encode(),
            "headers": [(k.lower().encode(), v.encode()) for k, v in headers.items()],
            "client": ("127.0.0.1", 50000),
            "server": ("cloud.test", 443),
            "subprotocols": [],
        }
        self._task: asyncio.Task[None] | None = None
        self.accepted = False
        self.closed = False

    async def connect(self) -> bool:
        self._task = asyncio.ensure_future(
            self._app(self._scope, self._inbound.get, self._outbound.put)
        )
        await self._inbound.put({"type": "websocket.connect"})
        message = await self._next(2.0)
        if message is not None and message["type"] == "websocket.accept":
            self.accepted = True
        else:
            self.closed = True
        return self.accepted

    async def _next(self, timeout: float) -> MutableMapping[str, Any] | None:
        try:
            return await asyncio.wait_for(self._outbound.get(), timeout)
        except asyncio.TimeoutError:
            if self._task is not None and self._task.done():
                return {"type": "websocket.close"}
            return None

    async def send(self, raw: bytes) -> None:
        await self._inbound.put({"type": "websocket.receive", "text": raw.decode()})

    async def receive(self, timeout: float = SOCKET_WAIT) -> bytes | None:
        """Next text frame, or None on silence or close."""
        message = await self._next(timeout)
        if message is None:
            return None
        if message["type"] == "websocket.send":
            text = message.get("text")
            return text.encode() if text is not None else message.get("bytes")
        self.closed = True
        return None

    async def aclose(self) -> None:
        await self._inbound.put({"type": "websocket.disconnect", "code": 1000})
        if self._task is not None:
            try:
                await asyncio.wait_for(self._task, 1.0)
            except (asyncio.TimeoutError, Exception):
                self._task.cancel()


# ------------------------------------------------------------------------ fixtures


@dataclass
class Cloud:
    app: FastAPI
    client: httpx.AsyncClient
    sockets: list[WsClient]

    async def login(self, password: str = PASSWORD) -> str:
        response = await self.client.post("/api/session", json={"password": password})
        assert response.status_code == 200, response.status_code
        return str(response.json()["csrf_token"])

    async def pair(self, csrf: str, name: str = "robot") -> tuple[str, str]:
        code = await self.client.post("/api/pairing-codes", json={}, headers={"X-CSRF-Token": csrf})
        assert code.status_code == 201, code.status_code
        paired = await self.client.post(
            "/api/devices/pair", json={"code": code.json()["code"], "name": name}
        )
        assert paired.status_code == 200, paired.status_code
        body = paired.json()
        return str(body["robot_id"]), str(body["device_token"])

    def cookie_header(self) -> str:
        return "; ".join(f"{c.name}={c.value}" for c in self.client.cookies.jar)

    async def agent(self, token: str) -> WsClient:
        socket = WsClient(self.app, "/ws/agent", {"Authorization": f"Bearer {token}"})
        self.sockets.append(socket)
        return socket

    async def control(
        self, robot_id: str, *, origin: str | None = ORIGIN, cookie: str | None = None
    ) -> WsClient:
        headers = {"Cookie": self.cookie_header() if cookie is None else cookie}
        if origin is not None:
            headers["Origin"] = origin
        socket = WsClient(self.app, "/ws/control", headers, f"robot_id={robot_id}")
        self.sockets.append(socket)
        return socket


@pytest.fixture
async def cloud(tmp_path: Path) -> AsyncIterator[Cloud]:
    settings = CloudSettings(secret=SecretStr("s" * 32))
    async with Database(tmp_path / "cloud.db") as database:
        app = create_app(
            settings,
            database=database,
            admin_password_hash=hash_password(PASSWORD),
            allowed_origins=(ORIGIN,),
        )
        transport = httpx.ASGITransport(app=app)
        async with app.router.lifespan_context(app):
            async with httpx.AsyncClient(transport=transport, base_url=ORIGIN) as client:
                state = Cloud(app, client, [])
                yield state
                for socket in state.sockets:
                    await socket.aclose()


def stop_frame(robot_id: str, seq: int = 1, epoch: int = 1) -> bytes:
    return serialize_envelope(
        SimpleEnvelope(
            v=1,
            robot_id=robot_id,
            session_id="browser-session",
            connection_epoch=epoch,
            request_id=f"req-{seq}",
            seq=seq,
            expires_at=time.time() + 5.0,
            type="drive.stop",
            payload=EmptyPayload(),
        )
    )


def ok_frame(robot_id: str, seq: int, epoch: int) -> bytes:
    return serialize_envelope(
        ResponseEnvelope(
            v=1,
            robot_id=robot_id,
            session_id="agent-session",
            connection_epoch=epoch,
            request_id=f"req-{seq}",
            seq=seq,
            expires_at=time.time() + 5.0,
            type="response",
            payload=ResponsePayload(request_type="drive.stop", ok=True),
        )
    )


# --------------------------------------------------------------------------- tests


async def test_pair_code_atomic_single_use(cloud: Cloud) -> None:
    csrf = await cloud.login()
    code = await cloud.client.post("/api/pairing-codes", json={}, headers={"X-CSRF-Token": csrf})
    assert code.status_code == 201
    body = {"code": code.json()["code"], "name": "robot"}

    first, second = await asyncio.gather(
        cloud.client.post("/api/devices/pair", json=body),
        cloud.client.post("/api/devices/pair", json=body),
    )

    assert sorted([first.status_code, second.status_code]) == [200, 409]
    later = await cloud.client.post("/api/devices/pair", json=body)
    assert later.status_code == 409
    robots = await cloud.client.get("/api/robots")
    assert [r["name"] for r in robots.json()["robots"]] == ["robot"]


async def test_rate_limit_invalid_login(cloud: Cloud) -> None:
    good = await cloud.client.post("/api/session", json={"password": PASSWORD})
    assert good.status_code == 200
    await cloud.client.delete("/api/session", headers={"X-CSRF-Token": good.json()["csrf_token"]})

    statuses = []
    for _ in range(20):
        response = await cloud.client.post("/api/session", json={"password": "wrong"})
        statuses.append(response.status_code)

    assert statuses[0] == 401
    assert 429 in statuses
    assert statuses.index(429) <= 10
    locked = await cloud.client.post("/api/session", json={"password": PASSWORD})
    assert locked.status_code == 429
    for path in ("/api/register", "/api/signup", "/api/users", "/api/session/register"):
        response = await cloud.client.post(path, json={"password": "new"})
        assert response.status_code in (404, 405)


async def test_session_cookie_flags(cloud: Cloud) -> None:
    response = await cloud.client.post("/api/session", json={"password": PASSWORD})
    assert response.status_code == 200
    cookie = response.headers["set-cookie"].lower()
    assert "secure" in cookie
    assert "httponly" in cookie
    assert "samesite=strict" in cookie or "samesite=lax" in cookie


async def test_write_without_csrf_fails(cloud: Cloud) -> None:
    csrf = await cloud.login()
    missing = await cloud.client.post("/api/pairing-codes", json={})
    wrong = await cloud.client.post(
        "/api/pairing-codes", json={}, headers={"X-CSRF-Token": "forged"}
    )
    valid = await cloud.client.post("/api/pairing-codes", json={}, headers={"X-CSRF-Token": csrf})
    assert missing.status_code == 403
    assert wrong.status_code == 403
    assert valid.status_code == 201


async def test_unauthenticated_writes_rejected(cloud: Cloud) -> None:
    response = await cloud.client.post("/api/pairing-codes", json={})
    assert response.status_code == 401
    assert (await cloud.client.get("/api/robots")).status_code == 401


async def test_websocket_checks_origin(cloud: Cloud) -> None:
    robot_id, _ = await cloud.pair(await cloud.login())

    good = await cloud.control(robot_id)
    assert await good.connect() is True
    for origin in ("https://evil.test", None):
        bad = await cloud.control(robot_id, origin=origin)
        assert await bad.connect() is False


async def test_revoked_device_cannot_reconnect(cloud: Cloud) -> None:
    csrf = await cloud.login()
    robot_id, token = await cloud.pair(csrf)
    before = await cloud.agent(token)
    assert await before.connect() is True

    revoked = await cloud.client.delete(f"/api/robots/{robot_id}", headers={"X-CSRF-Token": csrf})
    assert revoked.status_code == 204

    assert await before.receive() is None
    assert before.closed is True
    after = await cloud.agent(token)
    assert await after.connect() is False


async def test_browser_cannot_impersonate_agent(cloud: Cloud) -> None:
    csrf = await cloud.login()
    robot_id, token = await cloud.pair(csrf)
    agent = await cloud.agent(token)
    assert await agent.connect() is True
    browser = await cloud.control(robot_id)
    assert await browser.connect() is True

    # A browser session cannot open the agent socket, and a device token cannot open control.
    as_agent = WsClient(cloud.app, "/ws/agent", {"Cookie": cloud.cookie_header(), "Origin": ORIGIN})
    assert await as_agent.connect() is False
    cloud.sockets.append(as_agent)
    as_browser = await cloud.control(robot_id, cookie=f"device={token}")
    assert await as_browser.connect() is False

    # A browser frame that claims to be an agent response is never relayed or accepted.
    await browser.send(ok_frame(robot_id, seq=7, epoch=1))
    assert await agent.receive() is None
    reply = await browser.receive()
    assert reply is not None
    envelope = validate_envelope(reply, robot_id)
    assert isinstance(envelope, ResponseEnvelope)
    assert envelope.payload.ok is False
    assert envelope.payload.error is ErrorCode.VALIDATION


async def test_two_sockets_old_epoch_dropped(cloud: Cloud) -> None:
    csrf = await cloud.login()
    robot_id, token = await cloud.pair(csrf)
    browser = await cloud.control(robot_id)
    assert await browser.connect() is True
    old = await cloud.agent(token)
    assert await old.connect() is True
    new = await cloud.agent(token)
    assert await new.connect() is True

    await old.send(ok_frame(robot_id, seq=1, epoch=1))
    assert await browser.receive() is None
    await new.send(ok_frame(robot_id, seq=2, epoch=2))
    relayed = await browser.receive()
    assert relayed is not None
    assert validate_envelope(relayed, robot_id).seq == 2

    await browser.send(stop_frame(robot_id, seq=3))
    assert await old.receive() is None
    command = await new.receive()
    assert command is not None
    assert validate_envelope(command, robot_id).seq == 3


async def test_wrong_robot_command_rejected(cloud: Cloud) -> None:
    csrf = await cloud.login()
    robot_a, token_a = await cloud.pair(csrf, "a")
    robot_b, token_b = await cloud.pair(csrf, "b")
    agent_a = await cloud.agent(token_a)
    agent_b = await cloud.agent(token_b)
    assert await agent_a.connect() is True
    assert await agent_b.connect() is True
    browser = await cloud.control(robot_a)
    assert await browser.connect() is True

    await browser.send(stop_frame(robot_b, seq=1))
    assert await agent_a.receive() is None
    assert await agent_b.receive() is None
    reply = await browser.receive()
    assert reply is not None
    envelope = validate_envelope(reply, robot_b)
    assert isinstance(envelope, ResponseEnvelope)
    assert envelope.payload.ok is False
    assert envelope.payload.error is ErrorCode.VALIDATION

    await browser.send(stop_frame(robot_a, seq=2))
    delivered = await agent_a.receive()
    assert delivered is not None
    assert json.loads(delivered)["robot_id"] == robot_a
