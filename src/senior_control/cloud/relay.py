"""Live sockets: bounded outboxes, agent epochs, and command forwarding."""

import asyncio
import json
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Literal, cast

from fastapi import WebSocket

from senior_control.agent.snapshots import SnapshotCache, snapshot_from_envelope
from senior_control.cloud.limits import CloudLimits
from senior_control.errors import ErrorCode
from senior_control.protocol import (
    Envelope,
    ResponseEnvelope,
    ResponsePayload,
    SnapshotEnvelope,
    serialize_envelope,
    validate_envelope,
)

SAFETY_TYPES = frozenset({"drive.stop", "safety.estop"})
VELOCITY = "drive.velocity"

CLOSE_NORMAL = 1000
CLOSE_POLICY = 1008
CLOSE_BAD_FRAME = 1007
CLOSE_TOO_BIG = 1009
CLOSE_OVERLOAD = 1013
CLOSE_SUPERSEDED = 4000


class Outbox:
    """Bounded frame queue. Velocity frames coalesce; safety frames evict others."""

    def __init__(self, limit: int) -> None:
        self._limit = limit
        self._items: deque[tuple[str, bytes]] = deque()
        self._event = asyncio.Event()
        self._close_code: int | None = None

    @property
    def pending(self) -> int:
        return len(self._items)

    @property
    def closed(self) -> bool:
        return self._close_code is not None

    def put(self, frame: bytes, kind: str) -> bool:
        """Queue a frame; False means the caller must report overload."""
        if self._close_code is not None:
            return False
        if kind == VELOCITY:
            self._items = deque(item for item in self._items if item[0] != VELOCITY)
        if len(self._items) >= self._limit and not self._evict(kind):
            return False
        self._items.append((kind, frame))
        self._event.set()
        return True

    def _evict(self, incoming: str) -> bool:
        for index, (kind, _) in enumerate(self._items):
            if kind == VELOCITY:
                del self._items[index]
                return True
        if incoming in SAFETY_TYPES:
            for index, (kind, _) in enumerate(self._items):
                if kind not in SAFETY_TYPES:
                    del self._items[index]
                    return True
        return False

    def close(self, code: int) -> None:
        if self._close_code is None:
            self._close_code = code
            self._items.clear()
            self._event.set()

    async def get(self) -> bytes | int:
        """Next frame, or the close code once the outbox is closed."""
        while True:
            if self._close_code is not None:
                return self._close_code
            if self._items:
                return self._items.popleft()[1]
            self._event.clear()
            await self._event.wait()


@dataclass(eq=False)
class Connection:
    role: Literal["agent", "control"]
    robot_id: str
    outbox: Outbox
    epoch: int = 0
    session_hash: str | None = None


@dataclass
class _Link:
    agent: Connection | None = None
    epoch: int = 0
    controllers: set[Connection] = field(default_factory=set)


class Hub:
    def __init__(self, limits: CloudLimits, snapshots: SnapshotCache | None = None) -> None:
        self._limits = limits
        self.snapshots = snapshots if snapshots is not None else SnapshotCache()
        self._links: dict[str, _Link] = {}

    def _link(self, robot_id: str) -> _Link:
        link = self._links.get(robot_id)
        if link is None:
            link = self._links[robot_id] = _Link()
        return link

    def is_online(self, robot_id: str) -> bool:
        link = self._links.get(robot_id)
        return link is not None and link.agent is not None

    def attach_agent(self, robot_id: str) -> Connection:
        """The n-th live agent socket of a robot owns epoch n; the older one is closed."""
        link = self._link(robot_id)
        link.epoch += 1
        connection = Connection("agent", robot_id, Outbox(self._limits.outbox_frames), link.epoch)
        previous, link.agent = link.agent, connection
        if previous is not None:
            previous.outbox.close(CLOSE_SUPERSEDED)
        return connection

    def attach_controller(self, robot_id: str, session_hash: str) -> Connection | None:
        link = self._link(robot_id)
        if len(link.controllers) >= self._limits.max_controllers_per_robot:
            return None
        connection = Connection(
            "control", robot_id, Outbox(self._limits.outbox_frames), session_hash=session_hash
        )
        link.controllers.add(connection)
        return connection

    def detach(self, connection: Connection) -> None:
        link = self._links.get(connection.robot_id)
        if link is not None:
            if link.agent is connection:
                link.agent = None
                self.snapshots.mark_offline(connection.robot_id)
            link.controllers.discard(connection)
        connection.outbox.close(CLOSE_NORMAL)

    def revoke_robot(self, robot_id: str) -> None:
        link = self._links.get(robot_id)
        if link is None:
            return
        if link.agent is not None:
            link.agent.outbox.close(CLOSE_POLICY)
        for controller in link.controllers:
            controller.outbox.close(CLOSE_POLICY)

    def close_session(self, session_hash: str) -> None:
        for link in self._links.values():
            for controller in link.controllers:
                if controller.session_hash == session_hash:
                    controller.outbox.close(CLOSE_POLICY)

    def close_all(self) -> None:
        for link in self._links.values():
            if link.agent is not None:
                link.agent.outbox.close(CLOSE_NORMAL)
            for controller in link.controllers:
                controller.outbox.close(CLOSE_NORMAL)

    def handle_agent_frame(self, connection: Connection, raw: bytes) -> None:
        link = self._links.get(connection.robot_id)
        if link is None or link.agent is not connection:
            return
        if len(raw) > self._limits.max_ws_message_bytes:
            connection.outbox.close(CLOSE_TOO_BIG)
            return
        try:
            envelope = validate_envelope(raw, connection.robot_id)
        except ValueError:
            connection.outbox.close(CLOSE_BAD_FRAME)
            return
        if envelope.connection_epoch != connection.epoch and isinstance(
            envelope, (SnapshotEnvelope, ResponseEnvelope)
        ):
            return
        if isinstance(envelope, SnapshotEnvelope):
            self.snapshots.store(snapshot_from_envelope(envelope))
            return
        if not isinstance(envelope, ResponseEnvelope):
            connection.outbox.close(CLOSE_POLICY)
            return
        for controller in list(link.controllers):
            if not controller.outbox.put(raw, "response"):
                controller.outbox.close(CLOSE_OVERLOAD)

    def handle_control_frame(self, connection: Connection, raw: bytes) -> None:
        if len(raw) > self._limits.max_ws_message_bytes:
            connection.outbox.close(CLOSE_TOO_BIG)
            return
        try:
            claimed = json.loads(raw)["robot_id"]
            if not isinstance(claimed, str):
                raise ValueError("robot_id must be text")
            envelope = validate_envelope(raw, claimed)
        except (ValueError, KeyError, TypeError):
            connection.outbox.close(CLOSE_BAD_FRAME)
            return
        if isinstance(envelope, SnapshotEnvelope):
            connection.outbox.close(CLOSE_POLICY)
            return
        if envelope.robot_id != connection.robot_id or isinstance(envelope, ResponseEnvelope):
            self._reject(connection, envelope, ErrorCode.VALIDATION)
            return
        if envelope.expires_at < time.time():
            self._reject(connection, envelope, ErrorCode.TIMEOUT)
            return
        link = self._links.get(connection.robot_id)
        agent = link.agent if link is not None else None
        if agent is None:
            self._reject(connection, envelope, ErrorCode.OFFLINE)
        elif not agent.outbox.put(raw, envelope.type):
            self._reject(connection, envelope, ErrorCode.CONFLICT)

    def _reject(self, connection: Connection, envelope: Envelope, error: ErrorCode) -> None:
        request_type: Any = (
            envelope.payload.request_type
            if isinstance(envelope, ResponseEnvelope)
            else envelope.type
        )
        reply = ResponseEnvelope(
            v=1,
            robot_id=envelope.robot_id,
            session_id=envelope.session_id,
            connection_epoch=envelope.connection_epoch,
            request_id=envelope.request_id,
            seq=envelope.seq,
            expires_at=envelope.expires_at,
            type="response",
            payload=ResponsePayload(request_type=request_type, ok=False, error=error),
        )
        if not connection.outbox.put(serialize_envelope(reply), "response"):
            connection.outbox.close(CLOSE_OVERLOAD)


async def _pump_in(websocket: WebSocket, connection: Connection, hub: Hub) -> None:
    while True:
        message = cast(dict[str, Any], await websocket.receive())
        if message["type"] == "websocket.disconnect":
            return
        data = message.get("bytes")
        if data is None:
            text = message.get("text")
            if text is None:
                continue
            data = text.encode()
        if connection.role == "agent":
            hub.handle_agent_frame(connection, data)
        else:
            hub.handle_control_frame(connection, data)


async def _pump_out(websocket: WebSocket, connection: Connection) -> None:
    while True:
        item = await connection.outbox.get()
        if isinstance(item, int):
            try:
                await websocket.close(code=item)
            except RuntimeError:
                pass
            return
        await websocket.send_text(item.decode())


async def serve_connection(websocket: WebSocket, connection: Connection, hub: Hub) -> None:
    """Run one accepted socket until either side ends it, then release every task."""
    reader = asyncio.ensure_future(_pump_in(websocket, connection, hub))
    writer = asyncio.ensure_future(_pump_out(websocket, connection))
    try:
        await asyncio.wait({reader, writer}, return_when=asyncio.FIRST_COMPLETED)
    finally:
        reader.cancel()
        writer.cancel()
        await asyncio.gather(reader, writer, return_exceptions=True)
        hub.detach(connection)
