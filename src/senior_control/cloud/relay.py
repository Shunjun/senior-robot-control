"""Live sockets: bounded outboxes, agent epochs, and command forwarding."""

import asyncio
import json
import logging
import secrets
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Literal, cast

from fastapi import WebSocket
from pydantic import ValidationError

from senior_control.agent.snapshots import (
    SnapshotCache,
    snapshot_envelope,
    snapshot_from_envelope,
)
from senior_control.cloud.limits import CloudLimits
from senior_control.errors import ErrorCode
from senior_control.protocol import (
    DEVICE_REVOKED_CLOSE_CODE,
    DRIVE_CHALLENGE_SECONDS,
    WORKSPACE_REQUEST_TYPES,
    EmptyPayload,
    Envelope,
    HeartbeatRequest,
    ResponseEnvelope,
    ResponsePayload,
    SimpleEnvelope,
    SnapshotEnvelope,
    VelocityEnvelope,
    VideoCandidateEnvelope,
    VideoCloseEnvelope,
    VideoEnvelope,
    VideoSdpEnvelope,
    WorkspaceEnvelope,
    serialize_envelope,
    validate_frame,
)

logger = logging.getLogger(__name__)

SAFETY_TYPES = frozenset({"drive.stop", "safety.estop"})
LEASE_TYPES = frozenset({"control.acquire", "control.release"})
VELOCITY = "drive.velocity"
VIDEO = "video"
VIDEO_CLOSE = "video.close"
SNAPSHOT = "snapshot"
WORKSPACE = "workspace"
WORKSPACE_REPLY_SECONDS = 5.0
HEARTBEAT_WINDOW_SECONDS = 2.0
# Five default backoffs (31s), five failed handshakes (up to 25s) and heartbeat margin.
RECONNECT_GRACE_SECONDS = 60.0
LEASE_ACK_SECONDS = 1.0
SNAPSHOT_TTL_SECONDS = 60.0
VIDEO_CLOSE_TTL_SECONDS = 300.0
PROTECTED_TYPES = SAFETY_TYPES | LEASE_TYPES | {VIDEO_CLOSE}

CLOSE_NORMAL = 1000
CLOSE_POLICY = 1008
CLOSE_BAD_FRAME = 1007
CLOSE_TOO_BIG = 1009
CLOSE_OVERLOAD = 1013
CLOSE_SUPERSEDED = 4000


class Outbox:
    """Bounded frame queue. Velocity frames coalesce; safety frames evict others.

    Video signaling has its own kind. It never replaces a velocity sample, it is the first thing
    a full queue gives up for any other frame, and it is refused rather than evicting one.
    A pushed snapshot replaces any snapshot still queued: only the newest cache state matters.
    A video close is the exception: it evicts video and survives both a stop and a socket close,
    because it is what makes the vehicle stop sending pictures.
    """

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
        if kind in SAFETY_TYPES or kind in LEASE_TYPES:
            self.discard_velocity()
        if kind in (VELOCITY, SNAPSHOT):
            self._items = deque(item for item in self._items if item[0] != kind)
        if len(self._items) >= self._limit and not self._evict(kind):
            return False
        if kind in SAFETY_TYPES:
            # Give safety frames priority without reversing their command sequence.
            index = next(
                (
                    index
                    for index, (queued, _) in enumerate(self._items)
                    if queued not in SAFETY_TYPES
                ),
                len(self._items),
            )
            self._items.insert(index, (kind, frame))
        elif kind == WORKSPACE:
            self._items.append((kind, frame))
        else:
            index = next(
                (index for index, (queued, _) in enumerate(self._items) if queued == WORKSPACE),
                len(self._items),
            )
            self._items.insert(index, (kind, frame))
        self._event.set()
        return True

    def discard_velocity(self) -> None:
        self._items = deque(item for item in self._items if item[0] != VELOCITY)

    def _evict(self, incoming: str) -> bool:
        if incoming in {VIDEO, WORKSPACE}:
            return False
        for index, (kind, _) in enumerate(self._items):
            if kind == WORKSPACE:
                del self._items[index]
                return True
        for index, (kind, _) in enumerate(self._items):
            if kind == VIDEO:
                del self._items[index]
                return True
        for index, (kind, _) in enumerate(self._items):
            if kind == VELOCITY:
                del self._items[index]
                return True
        if incoming in PROTECTED_TYPES:
            for index, (kind, _) in enumerate(self._items):
                if kind not in PROTECTED_TYPES:
                    del self._items[index]
                    return True
        return False

    def close(self, code: int) -> None:
        if self._close_code is None:
            self._close_code = code
            self._items = deque(item for item in self._items if item[0] == VIDEO_CLOSE)
            self._event.set()

    async def get(self) -> bytes | int:
        """Next frame, or the close code once the outbox is closed and drained."""
        while True:
            if self._items:
                return self._items.popleft()[1]
            if self._close_code is not None:
                return self._close_code
            self._event.clear()
            await self._event.wait()


@dataclass(eq=False)
class Connection:
    role: Literal["agent", "control"]
    robot_id: str
    outbox: Outbox
    epoch: int = 0
    session_hash: str | None = None
    video_session_id: str | None = None
    heartbeat_at: float | None = None
    started_at: float = 0.0
    wall_origin: float = 0.0
    lease_session: str | None = None
    challenges: dict[str, tuple[str, int, float, float]] = field(default_factory=dict)
    highest_seq: int = 0
    safety_session: str = field(default_factory=lambda: "safety-" + secrets.token_hex(16))
    """Signaling id of this socket's video offer. Answers and candidates route by it."""

    def reconnect_deadline(self, now: float) -> float:
        if self.heartbeat_at is None:
            return 0.0
        disconnected_at = min(now, self.heartbeat_at + HEARTBEAT_WINDOW_SECONDS)
        return disconnected_at + RECONNECT_GRACE_SECONDS

    def server_time(self, monotonic_now: float) -> float:
        return self.wall_origin + (monotonic_now - self.started_at)


@dataclass
class _LeaseRequest:
    controller: Connection
    envelope: SimpleEnvelope
    synthetic: bool = False
    deadline: float = field(default_factory=lambda: time.monotonic() + LEASE_ACK_SECONDS)


@dataclass
class _WorkspaceRequest:
    controller: Connection
    envelope: WorkspaceEnvelope
    deadline: float = field(default_factory=lambda: time.monotonic() + WORKSPACE_REPLY_SECONDS)


@dataclass
class _Link:
    agent: Connection | None = None
    epoch: int = 0
    controllers: set[Connection] = field(default_factory=set)
    lease_request: _LeaseRequest | None = None
    workspace_request: _WorkspaceRequest | None = None
    reconnect_until: float = 0.0


class Hub:
    def __init__(self, limits: CloudLimits, snapshots: SnapshotCache | None = None) -> None:
        self._limits = limits
        self.snapshots = snapshots if snapshots is not None else SnapshotCache()
        self._links: dict[str, _Link] = {}

    def _stop_viewer_video(self, connection: Connection) -> None:
        """Tell the agent that this viewer's video is over; the media does not pass through here."""
        session_id, connection.video_session_id = connection.video_session_id, None
        if connection.role != "control" or session_id is None:
            return
        link = self._links.get(connection.robot_id)
        agent = link.agent if link is not None else None
        if agent is None:
            return
        frame = serialize_envelope(
            VideoCloseEnvelope(
                v=1,
                robot_id=connection.robot_id,
                session_id=session_id,
                connection_epoch=agent.epoch,
                request_id="video-close",
                seq=0,
                expires_at=time.time() + VIDEO_CLOSE_TTL_SECONDS,
                type="video.close",
                payload=EmptyPayload(),
            )
        )
        if not agent.outbox.put(frame, VIDEO_CLOSE):
            logger.warning("video close could not be queued for robot %s", connection.robot_id)

    def _link(self, robot_id: str) -> _Link:
        link = self._links.get(robot_id)
        if link is None:
            link = self._links[robot_id] = _Link()
        return link

    def is_online(self, robot_id: str) -> bool:
        link = self._links.get(robot_id)
        agent = link.agent if link is not None else None
        return (
            agent is not None
            and not agent.outbox.closed
            and agent.heartbeat_at is not None
            and time.monotonic() - agent.heartbeat_at <= HEARTBEAT_WINDOW_SECONDS
        )

    def is_reconnecting(self, robot_id: str) -> bool:
        link = self._links.get(robot_id)
        if link is None or self.is_online(robot_id):
            return False
        now = time.monotonic()
        deadline = link.agent.reconnect_deadline(now) if link.agent else 0.0
        return now < max(link.reconnect_until, deadline)

    def _push_snapshot(self, robot_id: str, only: Connection | None = None) -> None:
        """Send the cache's view of a robot to its control sockets, or to the one given.

        The frame carries the cache's own ``stale`` and ``last_synced_at``, and the epoch of the
        connected agent. With no agent the epoch is 0, which no agent owns.
        """
        cached = self.snapshots.get(robot_id)
        link = self._links.get(robot_id)
        if cached is None or link is None:
            return
        agent = link.agent
        envelope = snapshot_envelope(
            cached.snapshot,
            robot_id=robot_id,
            session_id="cloud-cache",
            connection_epoch=agent.epoch if agent is not None else 0,
            request_id="cloud-cache",
            seq=0,
            expires_at=time.time() + SNAPSHOT_TTL_SECONDS,
            stale=cached.stale,
            last_synced_at=cached.last_synced_at,
        )
        frame = serialize_envelope(envelope)
        for controller in list(link.controllers) if only is None else [only]:
            if not controller.outbox.put(frame, SNAPSHOT):
                controller.outbox.close(CLOSE_OVERLOAD)

    def attach_agent(self, robot_id: str, *, epoch: int | None = None) -> Connection:
        """The n-th live agent socket of a robot owns epoch n; the older one is closed."""
        link = self._link(robot_id)
        if epoch is None:
            epoch = link.epoch + 1
        if epoch <= link.epoch:
            raise ValueError("connection epoch must increase")
        link.epoch = epoch
        connection = Connection(
            "agent",
            robot_id,
            Outbox(self._limits.outbox_frames),
            link.epoch,
            started_at=time.monotonic(),
            wall_origin=time.time(),
        )
        previous, link.agent = link.agent, connection
        link.workspace_request = None
        link.lease_request = None
        for controller in link.controllers:
            controller.lease_session = None
            controller.challenges.clear()
        if previous is not None:
            if previous.heartbeat_at is not None:
                link.reconnect_until = previous.reconnect_deadline(connection.started_at)
            previous.outbox.close(CLOSE_SUPERSEDED)
        self._push_snapshot(robot_id)
        return connection

    def expire_stale_agent(self, connection: Connection) -> None:
        link = self._links.get(connection.robot_id)
        if link is None or link.agent is not connection:
            return
        last = (
            connection.heartbeat_at
            if connection.heartbeat_at is not None
            else connection.started_at
        )
        now = time.monotonic()
        if now - last > HEARTBEAT_WINDOW_SECONDS or (
            link.lease_request is not None and now >= link.lease_request.deadline
        ):
            self.detach(connection)

    def attach_controller(self, robot_id: str, session_hash: str) -> Connection | None:
        link = self._link(robot_id)
        if len(link.controllers) >= self._limits.max_controllers_per_robot:
            return None
        connection = Connection(
            "control", robot_id, Outbox(self._limits.outbox_frames), session_hash=session_hash
        )
        link.controllers.add(connection)
        self._push_snapshot(robot_id, only=connection)
        return connection

    def detach(self, connection: Connection) -> None:
        link = self._links.get(connection.robot_id)
        if link is not None:
            link.controllers.discard(connection)
            if connection.role == "control" and connection.lease_session and link.agent is not None:
                link.agent.outbox.discard_velocity()
                self._retire_lease(link, connection)
            if link.agent is connection:
                if connection.heartbeat_at is not None:
                    link.reconnect_until = connection.reconnect_deadline(time.monotonic())
                link.agent = None
                link.lease_request = None
                link.workspace_request = None
                for controller in link.controllers:
                    controller.lease_session = None
                    controller.challenges.clear()
                self.snapshots.mark_offline(connection.robot_id)
                self._push_snapshot(connection.robot_id)
        self._stop_viewer_video(connection)
        connection.outbox.close(CLOSE_NORMAL)

    def _retire_lease(self, link: _Link, controller: Connection) -> None:
        agent, session = link.agent, controller.lease_session
        if agent is None or session is None or link.lease_request is not None:
            return
        release = SimpleEnvelope(
            v=1,
            robot_id=controller.robot_id,
            session_id=session,
            connection_epoch=agent.epoch,
            request_id="socket-closed-" + secrets.token_hex(8),
            seq=controller.highest_seq + 1,
            expires_at=time.time() + 1.0,
            type="control.release",
            payload=EmptyPayload(),
        )
        if agent.outbox.put(serialize_envelope(release), release.type):
            link.lease_request = _LeaseRequest(controller, release, synthetic=True)
        else:
            agent.outbox.close(CLOSE_OVERLOAD)

    def revoke_robot(self, robot_id: str) -> None:
        link = self._links.get(robot_id)
        if link is None:
            return
        for controller in link.controllers:
            self._stop_viewer_video(controller)
            controller.outbox.close(CLOSE_POLICY)
        link.reconnect_until = 0.0
        if link.agent is not None:
            link.agent.heartbeat_at = None
            link.agent.outbox.close(DEVICE_REVOKED_CLOSE_CODE)

    def close_session(self, session_hash: str) -> None:
        for link in self._links.values():
            for controller in link.controllers:
                if controller.session_hash == session_hash:
                    self._stop_viewer_video(controller)
                    controller.outbox.close(CLOSE_POLICY)

    def close_all(self) -> None:
        for link in self._links.values():
            for controller in link.controllers:
                self._stop_viewer_video(controller)
                controller.outbox.close(CLOSE_NORMAL)
            if link.agent is not None:
                link.agent.outbox.close(CLOSE_NORMAL)

    def handle_agent_frame(self, connection: Connection, raw: bytes) -> None:
        link = self._links.get(connection.robot_id)
        if link is None or link.agent is not connection or connection.outbox.closed:
            return
        if len(raw) > self._limits.max_ws_message_bytes:
            connection.outbox.close(CLOSE_TOO_BIG)
            return
        try:
            data = json.loads(raw)
            if isinstance(data, dict) and data.get("type") == "heartbeat.request":
                heartbeat = HeartbeatRequest.model_validate(data)
                if (
                    heartbeat.robot_id != connection.robot_id
                    or heartbeat.connection_epoch != connection.epoch
                ):
                    connection.outbox.close(CLOSE_POLICY)
                    return
                reply = heartbeat.model_dump()
                reply["type"] = "heartbeat.reply"
                now = time.monotonic()
                reply["server_time"] = connection.server_time(now)
                if not connection.outbox.put(json.dumps(reply).encode(), "heartbeat"):
                    connection.outbox.close(CLOSE_OVERLOAD)
                    return
                connection.heartbeat_at = now
                link.reconnect_until = 0.0
                return
            envelope = validate_frame(raw, connection.robot_id)
        except (ValueError, ValidationError):
            connection.outbox.close(CLOSE_BAD_FRAME)
            return
        if envelope.connection_epoch != connection.epoch and isinstance(
            envelope,
            (
                SnapshotEnvelope,
                ResponseEnvelope,
                VideoSdpEnvelope,
                VideoCandidateEnvelope,
                VideoCloseEnvelope,
            ),
        ):
            return
        if isinstance(envelope, SnapshotEnvelope):
            self.snapshots.store(snapshot_from_envelope(envelope))
            self._push_snapshot(connection.robot_id)
            return
        if isinstance(envelope, (VideoSdpEnvelope, VideoCandidateEnvelope, VideoCloseEnvelope)):
            if envelope.type == "video.offer":
                connection.outbox.close(CLOSE_POLICY)
                return
            # Signaling belongs to the one viewer whose offer opened this id. A frame that does
            # not fit is dropped: ICE and the viewer's retry recover, and the control socket,
            # which also carries stop and heartbeat replies, stays open.
            ending = envelope.type == "video.close"
            for controller in list(link.controllers):
                if controller.video_session_id == envelope.session_id:
                    controller.outbox.put(raw, VIDEO_CLOSE if ending else VIDEO)
                    if ending:
                        controller.video_session_id = None
            return
        if not isinstance(envelope, ResponseEnvelope):
            connection.outbox.close(CLOSE_POLICY)
            return
        if envelope.payload.request_type in WORKSPACE_REQUEST_TYPES:
            pending = link.workspace_request
            if pending is not None and (
                envelope.payload.request_type == pending.envelope.type
                and envelope.request_id == pending.envelope.request_id
                and envelope.session_id == pending.envelope.session_id
                and envelope.seq == pending.envelope.seq
            ):
                link.workspace_request = None
                if time.monotonic() < pending.deadline:
                    pending.controller.outbox.put(raw, WORKSPACE)
            return
        self._handle_lease_response(link, connection, envelope)
        for controller in list(link.controllers):
            if not controller.outbox.put(raw, "response"):
                controller.outbox.close(CLOSE_OVERLOAD)

    def _handle_lease_response(
        self, link: _Link, agent: Connection, response: ResponseEnvelope
    ) -> None:
        pending = link.lease_request
        if pending is None:
            return
        request = pending.envelope
        if (
            response.payload.request_type != request.type
            or response.request_id != request.request_id
            or response.session_id != request.session_id
            or response.seq != request.seq
        ):
            return
        link.lease_request = None
        controller = pending.controller
        if response.payload.ok:
            controller.lease_session = (
                request.session_id if request.type == "control.acquire" else None
            )
        elif pending.synthetic:
            # The retired controller cannot retry. Revoke the agent link and its motion lease.
            agent.outbox.close(CLOSE_NORMAL)
            return
        if controller not in link.controllers:
            self._retire_lease(link, controller)

    def handle_control_frame(self, connection: Connection, raw: bytes) -> None:
        link = self._links.get(connection.robot_id)
        if link is None or connection not in link.controllers or connection.outbox.closed:
            return
        if len(raw) > self._limits.max_ws_message_bytes:
            connection.outbox.close(CLOSE_TOO_BIG)
            return
        try:
            data = json.loads(raw)
            claimed = data["robot_id"]
            if not isinstance(claimed, str):
                raise ValueError("robot_id must be text")
            envelope = validate_frame(raw, claimed)
        except (ValueError, KeyError, TypeError):
            connection.outbox.close(CLOSE_BAD_FRAME)
            return
        if isinstance(envelope, SnapshotEnvelope):
            connection.outbox.close(CLOSE_POLICY)
            return
        if envelope.seq == 2**63 - 1:
            self._reject(connection, envelope, ErrorCode.VALIDATION)
            return
        connection.highest_seq = max(connection.highest_seq, envelope.seq)
        if (
            envelope.robot_id != connection.robot_id
            or isinstance(envelope, ResponseEnvelope)
            or envelope.type == "video.answer"
        ):
            self._reject(connection, envelope, ErrorCode.VALIDATION)
            return
        if (
            envelope.type
            not in {
                "control.acquire",
                "control.release",
                "drive.challenge",
                VELOCITY,
                *SAFETY_TYPES,
            }
            and envelope.expires_at < time.time()
        ):
            self._reject(connection, envelope, ErrorCode.TIMEOUT)
            return
        agent = link.agent
        if agent is not None:
            self.expire_stale_agent(agent)
        if agent is None or agent.outbox.closed:
            self._reject(connection, envelope, ErrorCode.OFFLINE)
            return
        if (
            link.reconnect_until
            and agent.heartbeat_at is None
            and envelope.type in LEASE_TYPES | {"drive.challenge", VELOCITY}
        ):
            self._reject(connection, envelope, ErrorCode.OFFLINE)
            return
        if isinstance(envelope, WorkspaceEnvelope):
            pending_read = link.workspace_request
            if (
                envelope.connection_epoch != agent.epoch
                or (pending_read is not None and time.monotonic() < pending_read.deadline)
                or not agent.outbox.put(raw, WORKSPACE)
            ):
                self._reject(connection, envelope, ErrorCode.CONFLICT)
            else:
                link.workspace_request = _WorkspaceRequest(connection, envelope)
            return
        if isinstance(envelope, SimpleEnvelope):
            pending = link.lease_request
            copied_session = any(
                controller is not connection and controller.lease_session == envelope.session_id
                for controller in link.controllers
            ) or (
                pending is not None
                and pending.controller is not connection
                and pending.envelope.session_id == envelope.session_id
            )
            if copied_session:
                if envelope.type != "safety.estop":
                    self._reject(connection, envelope, ErrorCode.CONFLICT)
                    return
                # Preserve emergency stop access without advancing the lease owner's sequence.
                envelope = envelope.model_copy(update={"session_id": connection.safety_session})
        if envelope.type in SAFETY_TYPES:
            # A safety-only command can target the authenticated current socket without
            # relying on a browser snapshot; it can never grant motion authority.
            envelope = envelope.model_copy(update={"connection_epoch": agent.epoch})
            raw = serialize_envelope(envelope)
        if envelope.type == "drive.challenge":
            if (
                connection.lease_session != envelope.session_id
                or envelope.connection_epoch != agent.epoch
                or link.lease_request is not None
            ):
                self._reject(connection, envelope, ErrorCode.CONFLICT)
                return
            mono = time.monotonic()
            connection.challenges = {k: v for k, v in connection.challenges.items() if v[3] > mono}
            if len(connection.challenges) >= 4:
                self._reject(connection, envelope, ErrorCode.CONFLICT)
                return
            token = secrets.token_urlsafe(24)
            expires = mono + DRIVE_CHALLENGE_SECONDS
            deadline = agent.server_time(expires)
            connection.challenges[token] = (envelope.session_id, agent.epoch, deadline, expires)
            reply = ResponseEnvelope(
                **envelope.model_dump(exclude={"type", "payload"}),
                type="response",
                payload=ResponsePayload(
                    request_type="drive.challenge",
                    ok=True,
                    challenge_id=token,
                    server_deadline=deadline,
                ),
            )
            if not connection.outbox.put(serialize_envelope(reply), "response"):
                connection.outbox.close(CLOSE_OVERLOAD)
            return
        if isinstance(envelope, VelocityEnvelope):
            challenge_id = envelope.payload.challenge_id
            grant = connection.challenges.pop(challenge_id, None) if challenge_id else None
            if (
                grant is None
                or "server_deadline" in data["payload"]
                or envelope.payload.server_deadline is not None
                or connection.lease_session != envelope.session_id
                or grant[0] != envelope.session_id
                or grant[1] != agent.epoch
                or envelope.connection_epoch != agent.epoch
                or time.monotonic() >= grant[3]
            ):
                self._reject(connection, envelope, ErrorCode.CONFLICT)
                return
            envelope.payload.server_deadline = grant[2]
            raw = serialize_envelope(envelope)
        elif envelope.type in SAFETY_TYPES:
            for controller in link.controllers:
                controller.challenges.clear()
        elif envelope.type in LEASE_TYPES:
            if (
                envelope.connection_epoch != agent.epoch
                or link.lease_request is not None
                or (
                    envelope.type == "control.release"
                    and connection.lease_session != envelope.session_id
                )
                or (
                    envelope.type == "control.acquire"
                    and any(
                        controller.lease_session is not None
                        and (
                            controller is not connection
                            or controller.lease_session != envelope.session_id
                        )
                        for controller in link.controllers
                    )
                )
            ):
                self._reject(connection, envelope, ErrorCode.CONFLICT)
                return
            connection.challenges.clear()
        kind: str = envelope.type
        if isinstance(envelope, (VideoSdpEnvelope, VideoCandidateEnvelope, VideoCloseEnvelope)):
            kind = VIDEO_CLOSE if envelope.type == "video.close" else VIDEO
            if not self._claim_video(connection, link, envelope):
                self._reject(connection, envelope, ErrorCode.VALIDATION)
                return
        if not agent.outbox.put(raw, kind):
            self._reject(connection, envelope, ErrorCode.CONFLICT)
        elif isinstance(envelope, SimpleEnvelope) and kind in LEASE_TYPES:
            link.lease_request = _LeaseRequest(connection, envelope)
        elif kind == VIDEO_CLOSE:
            connection.video_session_id = None

    @staticmethod
    def _claim_video(
        connection: Connection,
        link: _Link | None,
        envelope: VideoSdpEnvelope | VideoCandidateEnvelope | VideoCloseEnvelope,
    ) -> bool:
        """An offer names the signaling id of this socket; later frames must use the same id."""
        if envelope.type == "video.offer":
            taken = link is not None and any(
                other is not connection and other.video_session_id == envelope.session_id
                for other in link.controllers
            )
            if taken:
                return False
            connection.video_session_id = envelope.session_id
            return True
        return connection.video_session_id == envelope.session_id

    def _reject(
        self, connection: Connection, envelope: Envelope | VideoEnvelope, error: ErrorCode
    ) -> None:
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
        if isinstance(envelope, WorkspaceEnvelope):
            connection.outbox.put(serialize_envelope(reply), WORKSPACE)
        elif not connection.outbox.put(serialize_envelope(reply), "response"):
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


async def _watch_heartbeat(connection: Connection, hub: Hub) -> None:
    while True:
        await asyncio.sleep(0.2)
        hub.expire_stale_agent(connection)


async def serve_connection(websocket: WebSocket, connection: Connection, hub: Hub) -> None:
    """Run one accepted socket until either side ends it, then release every task."""
    reader = asyncio.ensure_future(_pump_in(websocket, connection, hub))
    writer = asyncio.ensure_future(_pump_out(websocket, connection))
    watcher = (
        asyncio.ensure_future(_watch_heartbeat(connection, hub))
        if connection.role == "agent"
        else None
    )
    try:
        await asyncio.wait({reader, writer}, return_when=asyncio.FIRST_COMPLETED)
    finally:
        reader.cancel()
        writer.cancel()
        if watcher is not None:
            watcher.cancel()
        await asyncio.gather(
            reader, writer, *([watcher] if watcher is not None else []), return_exceptions=True
        )
        hub.detach(connection)
