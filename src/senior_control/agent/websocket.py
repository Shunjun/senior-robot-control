"""Authenticated cloud WebSocket link."""

import asyncio
import json
import math
import secrets
import time
from collections.abc import Callable
from typing import Protocol

from senior_control.agent.connection import HeartbeatFrame
from senior_control.agent.snapshots import RobotSnapshot, snapshot_envelope
from senior_control.protocol import (
    MAX_MESSAGE_BYTES,
    Envelope,
    ResponseEnvelope,
    serialize_envelope,
    validate_envelope,
)
from senior_control.runtime.supervisor import AuthRejected


class ProtocolRejected(AuthRejected, ConnectionError):
    """A permanent wire protocol or credential rejection."""


class Socket(Protocol):
    async def send(self, data: str) -> None: ...
    async def recv(self) -> str | bytes: ...
    async def close(self) -> None: ...


class WebSocketLink:
    def __init__(
        self,
        socket: Socket,
        robot_id: str,
        epoch: int,
        *,
        wall_clock: Callable[[], float] = time.time,
        clock: Callable[[], float] = time.monotonic,
        heartbeat_timeout: float = 0.8,
        max_clock_offset: float = 0.05,
    ) -> None:
        self._socket = socket
        self._robot_id = robot_id
        self.epoch = epoch
        self._wall = wall_clock
        self._clock = clock
        self._timeout = heartbeat_timeout
        self._max_offset = max_clock_offset
        self._pending: tuple[str, float, float] | None = None
        self._clock_anchor: tuple[float, float] | None = None
        self._send_lock = asyncio.Lock()

    async def _send(self, data: str) -> None:
        async with self._send_lock:
            await asyncio.wait_for(self._socket.send(data), self._timeout)

    async def send_heartbeat(self) -> None:
        now = self._clock()
        if self._pending is not None:
            if now - self._pending[1] > self._timeout:
                raise ConnectionError("heartbeat reply timed out")
            return
        nonce = secrets.token_hex(16)
        sent_at = self._wall()
        self._pending = nonce, now, sent_at
        await self._send(
            json.dumps(
                {
                    "v": 1,
                    "type": "heartbeat.request",
                    "robot_id": self._robot_id,
                    "connection_epoch": self.epoch,
                    "nonce": nonce,
                    "sent_at": sent_at,
                }
            )
        )

    async def recv(self) -> Envelope | HeartbeatFrame:
        from websockets.exceptions import ConnectionClosed

        try:
            raw = await self._socket.recv()
        except ConnectionClosed as exc:
            if exc.rcvd is not None and exc.rcvd.code in {1007, 1008, 1009}:
                raise AuthRejected("cloud rejected the connection") from exc
            raise ConnectionError("cloud connection closed") from exc
        if len(raw) > MAX_MESSAGE_BYTES:
            raise ProtocolRejected("frame too large")
        try:
            data = json.loads(raw)
            if isinstance(data, dict) and data.get("type") == "heartbeat.reply":
                pending = self._pending
                now = self._clock()
                if (
                    pending is None
                    or set(data)
                    != {
                        "v",
                        "type",
                        "robot_id",
                        "connection_epoch",
                        "nonce",
                        "sent_at",
                        "server_time",
                    }
                    or data["v"] != 1
                    or data["robot_id"] != self._robot_id
                    or data["connection_epoch"] != self.epoch
                    or data["nonce"] != pending[0]
                    or data["sent_at"] != pending[2]
                    or isinstance(data["server_time"], bool)
                    or not isinstance(data["server_time"], (int, float))
                ):
                    raise ValueError("invalid heartbeat reply")
                elapsed = now - pending[1]
                if (
                    not math.isfinite(data["server_time"])
                    or abs(self._wall() - (pending[2] + elapsed)) > self._max_offset
                    or not 0 <= elapsed <= self._timeout
                    or abs(data["server_time"] - (pending[2] + elapsed / 2)) + elapsed / 2
                    > self._max_offset
                ):
                    raise ValueError("heartbeat clock offset or delay outside bounds")
                self._clock_anchor = (self._wall(), now)
                self._pending = None
                return HeartbeatFrame(self._robot_id, pending[1] + self._timeout)
            envelope = validate_envelope(
                raw.encode() if isinstance(raw, str) else raw, self._robot_id
            )
            if envelope.connection_epoch != self.epoch:
                raise ValueError("wrong connection epoch")
            return envelope
        except (ValueError, TypeError, KeyError) as exc:
            raise ProtocolRejected("invalid cloud frame") from exc

    def command_clock_valid(self) -> bool:
        anchor = self._clock_anchor
        if anchor is None:
            return False
        elapsed = self._clock() - anchor[1]
        return (
            0 <= elapsed <= self._timeout
            and abs(self._wall() - (anchor[0] + elapsed)) <= self._max_offset
        )

    async def send_response(self, envelope: ResponseEnvelope) -> None:
        await self._send(serialize_envelope(envelope).decode())

    async def send_snapshot(self, snapshot: RobotSnapshot) -> None:
        envelope = snapshot_envelope(
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
        await self._socket.close()


class WebSocketConnector:
    """Open an authenticated link and read the cloud-assigned connection epoch."""

    def __init__(self, url: str, token: str, robot_id: str) -> None:
        self._url = url
        self._token = token
        self._robot_id = robot_id

    async def __call__(self) -> WebSocketLink:
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
            raise AuthRejected("invalid cloud handshake") from exc
        return WebSocketLink(socket, self._robot_id, epoch)
