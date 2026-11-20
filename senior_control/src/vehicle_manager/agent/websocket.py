"""Authenticated cloud WebSocket link."""

import asyncio
import json
import math
import secrets
import time
from collections.abc import Callable
from typing import Any, Generic, Protocol

from senior_contracts.wire import (
    DEVICE_REVOKED_CLOSE_CODE,
    HEARTBEAT_WINDOW_SECONDS,
    MAX_MESSAGE_BYTES,
    Envelope,
    ResponseEnvelope,
    VideoEnvelope,
    serialize_envelope,
    validate_frame,
)

from vehicle_manager.agent.ports import HeartbeatT, LinkT, SnapshotT, WebSocketPorts
from vehicle_manager.agent.supervisor import AuthRejected


class ProtocolRejected(ConnectionError):
    """A wire protocol failure on the current connection."""


class Socket(Protocol):
    async def send(self, data: str) -> None: ...
    async def recv(self) -> str | bytes: ...
    async def close(self) -> None: ...


# A blocked transport owns this resource budget; it is not an online evidence window.
SOCKET_SEND_TIMEOUT_SECONDS = 5.0


class WebSocketLink(Generic[SnapshotT, HeartbeatT]):
    def __init__(
        self,
        socket: Socket,
        robot_id: str,
        epoch: int,
        *,
        ports: WebSocketPorts[SnapshotT, HeartbeatT],
        wall_clock: Callable[[], float] = time.time,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._ports = ports
        self._socket = socket
        self._robot_id = robot_id
        self.epoch = epoch
        self._wall = wall_clock
        self._clock = clock
        self._online_deadline = clock() + HEARTBEAT_WINDOW_SECONDS
        self._pending: dict[str, tuple[float, float]] = {}
        self._heartbeat_timer: asyncio.TimerHandle | None = None
        self._heartbeat_expired: asyncio.Future[None] | None = None
        self._send_lock = ports.send_lock()
        ports.receive_diagnostic(self)

    async def _send(self, data: str) -> None:
        async with self._send_lock:
            await asyncio.wait_for(self._socket.send(data), SOCKET_SEND_TIMEOUT_SECONDS)

    def _expiry_signal(self) -> asyncio.Future[None]:
        if self._heartbeat_expired is None:
            self._heartbeat_expired = asyncio.get_running_loop().create_future()
            self._arm_heartbeat_timer()
        return self._heartbeat_expired

    def _cancel_heartbeat_timer(self) -> None:
        if self._heartbeat_timer is not None:
            self._heartbeat_timer.cancel()
            self._heartbeat_timer = None

    def _arm_heartbeat_timer(self) -> None:
        self._cancel_heartbeat_timer()
        self._heartbeat_timer = asyncio.get_running_loop().call_later(
            max(0.0, self._online_deadline - self._clock()), self._expire_heartbeat
        )

    def _expire_heartbeat(self) -> None:
        expired = self._heartbeat_expired
        if expired is not None and not expired.done():
            expired.set_result(None)

    def _check_heartbeat(self) -> None:
        if self._clock() >= self._online_deadline or self._expiry_signal().done():
            raise ConnectionError("heartbeat reply timed out")

    async def send_heartbeat(self) -> None:
        self._check_heartbeat()
        now = self._clock()
        # Sending keeps its cadence while earlier replies are in flight. Retain only
        # request identities that can still supply evidence within the online window.
        self._pending = {
            nonce: sent
            for nonce, sent in self._pending.items()
            if now - sent[0] < HEARTBEAT_WINDOW_SECONDS
        }
        nonce = secrets.token_hex(16)
        sent_at = self._wall()
        self._pending[nonce] = now, sent_at
        request = {
            "v": 1,
            "type": "heartbeat.request",
            "robot_id": self._robot_id,
            "connection_epoch": self.epoch,
            "nonce": nonce,
            "sent_at": sent_at,
        }
        await self._send(json.dumps(request))

    async def recv(self) -> Envelope | VideoEnvelope | HeartbeatT:
        from websockets.exceptions import ConnectionClosed

        try:
            self._check_heartbeat()
            expired = self._expiry_signal()
            reader = asyncio.ensure_future(self._socket.recv())
            try:
                waiters: set[asyncio.Future[Any]] = {reader, expired}
                await asyncio.wait(waiters, return_when=asyncio.FIRST_COMPLETED)
                if expired.done():
                    raise ConnectionError("heartbeat reply timed out")
                self._check_heartbeat()
                raw = reader.result()
            finally:
                if not reader.done():
                    reader.cancel()
                await asyncio.gather(reader, return_exceptions=True)
        except ConnectionClosed as exc:
            if exc.rcvd is not None and exc.rcvd.code == DEVICE_REVOKED_CLOSE_CODE:
                raise AuthRejected("device credential revoked") from exc
            raise ConnectionError("cloud connection closed") from exc
        if len(raw) > MAX_MESSAGE_BYTES:
            raise ProtocolRejected("frame too large")
        try:
            data = json.loads(raw)
            if isinstance(data, dict) and data.get("type") == "heartbeat.reply":
                nonce = data.get("nonce")
                pending = self._pending.get(nonce) if isinstance(nonce, str) else None
                now = self._clock()
                fields = {
                    "v",
                    "type",
                    "robot_id",
                    "connection_epoch",
                    "nonce",
                    "sent_at",
                    "server_time",
                }
                if (
                    not isinstance(nonce, str)
                    or pending is None
                    or set(data) != fields
                    or data["v"] != 1
                    or data["robot_id"] != self._robot_id
                    or data["connection_epoch"] != self.epoch
                    or data["sent_at"] != pending[1]
                    or isinstance(data["server_time"], bool)
                    or not isinstance(data["server_time"], (int, float))
                ):
                    raise ConnectionError("invalid heartbeat reply")
                elapsed = now - pending[0]
                server_time = data["server_time"]
                if not math.isfinite(server_time) or not 0 <= elapsed < HEARTBEAT_WINDOW_SECONDS:
                    raise ConnectionError("invalid heartbeat reply timestamp or reply timed out")
                del self._pending[nonce]
                self._online_deadline = now + HEARTBEAT_WINDOW_SECONDS
                self._arm_heartbeat_timer()
                return self._ports.heartbeat(self._robot_id)
            envelope = validate_frame(raw.encode() if isinstance(raw, str) else raw, self._robot_id)
            if not isinstance(envelope, VideoEnvelope) and envelope.connection_epoch != self.epoch:
                raise ValueError("wrong connection epoch")
            return envelope
        except (ValueError, TypeError, KeyError) as exc:
            raise ProtocolRejected("invalid cloud frame") from exc

    async def send_response(self, envelope: ResponseEnvelope) -> None:
        await self._send(serialize_envelope(envelope).decode())

    async def send_video(self, envelope: VideoEnvelope) -> None:
        await self._send(serialize_envelope(envelope).decode())

    async def send_snapshot(self, snapshot: SnapshotT) -> None:
        envelope = self._ports.snapshot(
            snapshot,
            robot_id=self._robot_id,
            session_id="agent",
            connection_epoch=self.epoch,
            request_id="snapshot",
            seq=0,
            expires_at=self._wall() + 60.0,
        )
        await self._send(serialize_envelope(envelope).decode())

    async def close(self) -> None:
        self._cancel_heartbeat_timer()
        await self._socket.close()


class WebSocketConnector(Generic[LinkT]):
    """Open an authenticated link and read the cloud-assigned connection epoch."""

    def __init__(
        self,
        url: str,
        token: str,
        robot_id: str,
        *,
        link_factory: Callable[[Socket, str, int], LinkT],
    ) -> None:
        self._link_factory = link_factory
        self._url = url
        self._token = token
        self._robot_id = robot_id

    async def __call__(self) -> LinkT:
        from websockets.asyncio.client import connect
        from websockets.exceptions import InvalidStatus

        try:
            socket = await connect(
                self._url,
                additional_headers={"Authorization": f"Bearer {self._token}"},
                open_timeout=5,
                close_timeout=1,
                max_size=MAX_MESSAGE_BYTES,
                max_queue=16,
                ping_interval=None,
            )
        except InvalidStatus as exc:
            if exc.response.status_code in (401, 403):
                raise AuthRejected("device credential rejected") from exc
            raise ConnectionError("cloud handshake failed") from exc
        try:
            assert socket.response is not None
            epoch = int(socket.response.headers["x-connection-epoch"])
            if epoch < 1:
                raise ValueError("invalid epoch")
        except (AssertionError, KeyError, ValueError) as exc:
            await socket.close()
            raise ProtocolRejected("invalid cloud handshake") from exc
        return self._link_factory(
            socket,
            self._robot_id,
            epoch,
        )
