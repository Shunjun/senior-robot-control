"""Cloud ASGI application factory. Calling it starts nothing."""

import re
from collections.abc import AsyncIterator, Awaitable, Callable, MutableMapping, Sequence
from contextlib import asynccontextmanager
from typing import Any

from fastapi import FastAPI, HTTPException, Request, WebSocket
from fastapi.responses import JSONResponse, Response
from pydantic import BaseModel, ConfigDict, Field

from senior_control.cloud.auth import (
    SESSION_STATEMENTS,
    FailureLimiter,
    SessionRecord,
    SessionStore,
    TokenDigest,
    is_password_hash,
    verify_password_async,
)
from senior_control.cloud.limits import CloudLimits
from senior_control.cloud.pairing import PAIRING_STATEMENTS, PairingBusy, PairingService
from senior_control.cloud.registry import ROBOT_STATEMENTS, RobotRegistry
from senior_control.cloud.relay import CLOSE_OVERLOAD, CLOSE_POLICY, Hub, serve_connection
from senior_control.config import CloudSettings
from senior_control.storage.database import Database
from senior_control.storage.migrations import Migration, migrate
from senior_control.storage.schema import DOMAIN_MIGRATIONS

SESSION_COOKIE = "__Host-session"
CSRF_HEADER = "X-CSRF-Token"
CLOUD_MIGRATION = Migration(4, SESSION_STATEMENTS + PAIRING_STATEMENTS + ROBOT_STATEMENTS)

_ASGIReceive = Callable[[], Awaitable[dict[str, Any]]]
_ASGISend = Callable[[MutableMapping[str, Any]], Awaitable[None]]


class LoginBody(BaseModel):
    model_config = ConfigDict(extra="forbid")
    password: str = Field(min_length=1, max_length=256)


class PairBody(BaseModel):
    model_config = ConfigDict(extra="forbid")
    code: str = Field(min_length=1, max_length=64)
    name: str = Field(min_length=1, max_length=64)


class _BodyTooLarge(Exception):
    pass


class BodyLimitMiddleware:
    """Reject HTTP bodies over the configured bound, declared or streamed."""

    def __init__(self, app: Any, max_bytes: int) -> None:
        self.app = app
        self.max_bytes = max_bytes

    async def __call__(self, scope: dict[str, Any], receive: _ASGIReceive, send: _ASGISend) -> None:
        if scope["type"] != "http":
            await self.app(scope, receive, send)
            return
        declared = dict(scope["headers"]).get(b"content-length")
        too_large = JSONResponse({"detail": "request body too large"}, status_code=413)
        if declared is not None and (not declared.isdigit() or int(declared) > self.max_bytes):
            await too_large(scope, receive, send)
            return
        total = 0
        started = False

        async def limited() -> dict[str, Any]:
            nonlocal total
            message = await receive()
            if message["type"] == "http.request":
                total += len(message.get("body", b""))
                if total > self.max_bytes:
                    raise _BodyTooLarge
            return message

        async def tracking(message: MutableMapping[str, Any]) -> None:
            nonlocal started
            if message["type"] == "http.response.start":
                started = True
            await send(message)

        try:
            await self.app(scope, limited, tracking)
        except _BodyTooLarge:
            if not started:
                await too_large(scope, receive, send)


def _bearer(header: str | None) -> str | None:
    if header is None:
        return None
    match = re.fullmatch(r"Bearer ([A-Za-z0-9_\-]{1,256})", header.strip())
    return match.group(1) if match else None


def create_app(
    settings: CloudSettings,
    *,
    database: Database,
    admin_password_hash: str,
    allowed_origins: Sequence[str],
    limits: CloudLimits | None = None,
) -> FastAPI:
    """Build the app. The database is opened and migrated by the lifespan, not here."""
    if not is_password_hash(admin_password_hash):
        raise ValueError("admin password hash is required")
    limits = limits or CloudLimits()
    origins = frozenset(origin.rstrip("/") for origin in allowed_origins)
    digest = TokenDigest(settings.secret.get_secret_value().encode())
    sessions = SessionStore(database, digest, limits.session_ttl_seconds)
    registry = RobotRegistry(database, digest)
    pairing = PairingService(
        database,
        digest,
        ttl_seconds=limits.pairing_code_ttl_seconds,
        max_live=limits.max_live_pairing_codes,
    )
    hub = Hub(limits)
    login_limiter = FailureLimiter(
        per_client=limits.login_failures_per_client,
        global_limit=limits.login_failures_global,
        window_seconds=limits.failure_window_seconds,
        tracked_clients=limits.tracked_clients,
    )
    pair_limiter = FailureLimiter(
        per_client=limits.pair_failures_per_client,
        global_limit=limits.pair_failures_global,
        window_seconds=limits.failure_window_seconds,
        tracked_clients=limits.tracked_clients,
    )

    @asynccontextmanager
    async def lifespan(app: FastAPI) -> AsyncIterator[None]:
        owned = not hasattr(database, "connection")
        if owned:
            await database.__aenter__()
        try:
            await migrate(database, [*DOMAIN_MIGRATIONS, CLOUD_MIGRATION])
            yield
        finally:
            hub.close_all()
            if owned:
                await database.__aexit__(None, None, None)

    app = FastAPI(lifespan=lifespan, docs_url=None, redoc_url=None, openapi_url=None)
    app.add_middleware(BodyLimitMiddleware, max_bytes=limits.max_body_bytes)

    def client_key(request: Request) -> str:
        return request.client.host if request.client else "unknown"

    def check_origin(request: Request) -> None:
        origin = request.headers.get("origin")
        if origin is not None and origin.rstrip("/") not in origins:
            raise HTTPException(403, "origin not allowed")

    async def authorize(request: Request, *, write: bool) -> SessionRecord:
        record = await sessions.lookup(request.cookies.get(SESSION_COOKIE))
        if record is None:
            raise HTTPException(401, "login required")
        if write:
            check_origin(request)
            if not sessions.csrf_matches(record, request.headers.get(CSRF_HEADER)):
                raise HTTPException(403, "csrf token required")
        return record

    def blocked(limiter: FailureLimiter, request: Request) -> None:
        wait = limiter.retry_after(client_key(request))
        if wait:
            raise HTTPException(429, "too many attempts", headers={"Retry-After": str(wait)})

    @app.post("/api/session")
    async def login(body: LoginBody, request: Request) -> Response:
        check_origin(request)
        blocked(login_limiter, request)
        if not await verify_password_async(body.password, admin_password_hash):
            login_limiter.record_failure(client_key(request))
            raise HTTPException(401, "invalid credentials")
        login_limiter.reset(client_key(request))
        token, csrf = await sessions.create()
        response = JSONResponse({"csrf_token": csrf})
        response.set_cookie(
            SESSION_COOKIE,
            token,
            max_age=int(limits.session_ttl_seconds),
            path="/",
            secure=True,
            httponly=True,
            samesite="strict",
        )
        return response

    @app.delete("/api/session")
    async def logout(request: Request) -> Response:
        record = await authorize(request, write=True)
        await sessions.delete(record.token_hash)
        hub.close_session(record.token_hash)
        response = Response(status_code=204)
        response.delete_cookie(
            SESSION_COOKIE, path="/", secure=True, httponly=True, samesite="strict"
        )
        return response

    @app.post("/api/pairing-codes", status_code=201)
    async def issue_code(request: Request) -> dict[str, Any]:
        await authorize(request, write=True)
        try:
            code = await pairing.issue()
        except PairingBusy:
            raise HTTPException(429, "too many pending pairing codes") from None
        return {"code": code, "expires_in": int(limits.pairing_code_ttl_seconds)}

    @app.post("/api/devices/pair")
    async def pair_device(body: PairBody, request: Request) -> dict[str, str]:
        blocked(pair_limiter, request)
        redeemed = await pairing.redeem(body.code, body.name)
        if redeemed is None:
            pair_limiter.record_failure(client_key(request))
            raise HTTPException(409, "pairing code is invalid, expired or already used")
        return {"robot_id": redeemed[0], "device_token": redeemed[1]}

    @app.get("/api/robots")
    async def list_robots(request: Request) -> dict[str, Any]:
        await authorize(request, write=False)
        robots = await registry.list_active()
        return {
            "robots": [
                {"id": r.robot_id, "name": r.name, "online": hub.is_online(r.robot_id)}
                for r in robots
            ]
        }

    @app.delete("/api/robots/{robot_id}")
    async def revoke_robot(robot_id: str, request: Request) -> Response:
        await authorize(request, write=True)
        if not await registry.revoke(robot_id):
            raise HTTPException(404, "robot not found")
        hub.revoke_robot(robot_id)
        return Response(status_code=204)

    @app.websocket("/ws/agent")
    async def ws_agent(websocket: WebSocket) -> None:
        robot = await registry.authenticate(_bearer(websocket.headers.get("authorization")))
        if robot is None:
            await websocket.close(code=CLOSE_POLICY)
            return
        connection = hub.attach_agent(robot.robot_id)
        try:
            if await registry.get_active(robot.robot_id) is None:
                await websocket.close(code=CLOSE_POLICY)
                return
            await websocket.accept(
                headers=[(b"x-connection-epoch", str(connection.epoch).encode())]
            )
            await serve_connection(websocket, connection, hub)
        finally:
            hub.detach(connection)

    @app.websocket("/ws/control")
    async def ws_control(websocket: WebSocket, robot_id: str) -> None:
        origin = websocket.headers.get("origin")
        if origin is None or origin.rstrip("/") not in origins:
            await websocket.close(code=CLOSE_POLICY)
            return
        record = await sessions.lookup(websocket.cookies.get(SESSION_COOKIE))
        if record is None or await registry.get_active(robot_id) is None:
            await websocket.close(code=CLOSE_POLICY)
            return
        connection = hub.attach_controller(robot_id, record.token_hash)
        if connection is None:
            await websocket.close(code=CLOSE_OVERLOAD)
            return
        try:
            if await registry.get_active(robot_id) is None:
                await websocket.close(code=CLOSE_POLICY)
                return
            await websocket.accept()
            await serve_connection(websocket, connection, hub)
        finally:
            hub.detach(connection)

    return app
