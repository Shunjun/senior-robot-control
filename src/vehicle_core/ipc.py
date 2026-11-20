"""Local IPC between the agent process and the ROS-side adapter.

Frames are a 4-byte big-endian length followed by UTF-8 JSON with ``"v": 1``. The transport
is a Unix domain socket owned by the current user; nothing here can bind a network port.
A frame longer than ``MAX_FRAME`` is never read: the peer is disconnected instead.
"""

import asyncio
import json
import logging
import math
import os
import socket
import stat
import struct
import time
from collections.abc import Awaitable, Callable
from pathlib import Path
from typing import Any

from vehicle_core.diagnostics import PendingCounter

# //Compatible: preserve the existing IPC logger and its routing.
logger = logging.getLogger("senior_control.runtime.ipc")

PROTOCOL_VERSION = 1
MAX_FRAME = 8192
# sun_path is 104 bytes on macOS and 108 on Linux.
MAX_PATH = 100
_HEADER = struct.Struct(">I")

Message = dict[str, Any]
Handler = Callable[[Message], Awaitable[Message]]


class IpcError(Exception):
    pass


def _check_path(path: Path) -> str:
    text = str(path)
    if not path.is_absolute() or len(text.encode()) > MAX_PATH:
        raise ValueError("ipc socket path must be absolute and short")
    return text


def _encode(message: Message) -> bytes:
    try:
        body = json.dumps(message, separators=(",", ":"), allow_nan=False).encode()
    except (TypeError, ValueError) as exc:
        raise IpcError("message is not encodable") from exc
    if len(body) > MAX_FRAME:
        raise IpcError("frame too large")
    return _HEADER.pack(len(body)) + body


def _finite_float(text: str) -> float:
    value = float(text)
    if not math.isfinite(value):
        raise ValueError("non-finite JSON number")
    return value


async def _read(reader: asyncio.StreamReader) -> Message:
    try:
        header = await reader.readexactly(_HEADER.size)
        (length,) = _HEADER.unpack(header)
        if length > MAX_FRAME:
            raise IpcError("frame too large")
        body = await reader.readexactly(length)
    except asyncio.IncompleteReadError as exc:
        raise IpcError("connection closed") from exc
    try:
        message = json.loads(body, parse_float=_finite_float, parse_constant=_finite_float)
    except ValueError as exc:
        raise IpcError("frame is not json") from exc
    if not isinstance(message, dict):
        raise IpcError("frame is not an object")
    return message


class IpcServer:
    """ROS listener: request replies and a dedicated latest-intent notification stream."""

    def __init__(self, path: Path, handler: Handler, *, replace_existing: bool = True) -> None:
        if type(replace_existing) is not bool:
            raise ValueError("replace_existing must be a boolean")
        self._path = path
        self._text = _check_path(path)
        self._handler = handler
        self._handler_counter = PendingCounter("ipc.server.handlers")
        self._replace_existing = replace_existing
        self._server: asyncio.AbstractServer | None = None
        self._start_task: asyncio.Task[asyncio.CancelledError | None] | None = None
        self._closing = False
        self._closed = asyncio.Event()
        self._closed.set()
        self._socket_identity: tuple[int, int] | None = None
        self._connections: set[asyncio.Task[None]] = set()

    async def start(self) -> None:
        if (
            self._server is not None
            or self._closing
            or (self._start_task is not None and not self._start_task.done())
        ):
            raise RuntimeError("ipc server already started or starting")
        task = asyncio.create_task(self._start())
        self._start_task = task
        try:
            cancellation = await task
            if cancellation is not None:
                raise cancellation
        finally:
            if self._start_task is task:
                self._start_task = None

    async def _start(self) -> asyncio.CancelledError | None:
        pending_socket: socket.socket | None = None
        try:
            # //Compatible: legacy callers may replace a socket; isolated simulations opt out.
            if self._replace_existing:
                try:
                    if stat.S_ISSOCK(os.stat(self._path).st_mode):
                        os.unlink(self._path)
                except FileNotFoundError:
                    pass
            pending_socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            pending_socket.setblocking(False)
            previous = os.umask(0o177)
            try:
                pending_socket.bind(self._text)
            finally:
                os.umask(previous)
            # //Compatible: record ownership before asyncio can switch to another starter.
            socket_stat = os.lstat(self._path)
            if not stat.S_ISSOCK(socket_stat.st_mode):
                raise OSError("ipc path is not the bound socket")
            self._socket_identity = (socket_stat.st_dev, socket_stat.st_ino)
            self._server = await asyncio.start_unix_server(self._serve, sock=pending_socket)
            pending_socket = None  # The asyncio server now owns the socket.
            try:
                current = os.lstat(self._path)
            except FileNotFoundError:
                current = None
            if (
                current is not None
                and stat.S_ISSOCK(current.st_mode)
                and (current.st_dev, current.st_ino) == self._socket_identity
            ):
                os.chmod(self._path, 0o600)
        except BaseException as error:
            if pending_socket is not None:
                pending_socket.close()
            try:
                await self._cleanup()
            except BaseException as cleanup_error:
                if isinstance(error, asyncio.CancelledError):
                    error.__cause__ = cleanup_error
                    return error
                raise error from cleanup_error
            if isinstance(error, asyncio.CancelledError):
                return error
            raise
        return None

    async def close(self) -> None:
        while self._closing:
            await self._closed.wait()
        self._closing = True
        self._closed.clear()
        try:
            task = self._start_task
            if task is not None and not task.done():
                task.cancel()
                try:
                    cancellation = await asyncio.shield(task)
                except asyncio.CancelledError:
                    if not task.cancelled():
                        raise
                    cancellation = None
                if cancellation is not None and cancellation.__cause__ is not None:
                    raise cancellation.__cause__
            await self._cleanup()
        finally:
            self._closing = False
            self._closed.set()

    async def _cleanup(self) -> None:
        server, self._server = self._server, None
        identity = self._socket_identity
        if server is not None:
            server.close()
        for task in list(self._connections):
            task.cancel()
        try:
            await asyncio.gather(*self._connections, return_exceptions=True)
            if server is not None:
                await server.wait_closed()
        finally:
            # //Compatible: only unlink the socket created by this server instance.
            if identity is not None:
                try:
                    current = os.lstat(self._path)
                    if (
                        stat.S_ISSOCK(current.st_mode)
                        and (current.st_dev, current.st_ino) == identity
                    ):
                        os.unlink(self._path)
                except FileNotFoundError:
                    pass
                self._socket_identity = None

    async def _serve(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        task = asyncio.current_task()
        assert task is not None
        self._connections.add(task)
        latest: Message | None = None
        consumer: asyncio.Task[None] | None = None
        stream: bool | None = None
        intent_queue = PendingCounter("ipc.server.manual_intent", 1)

        async def consume() -> None:
            nonlocal latest
            while latest is not None:
                sample, latest = latest, None
                intent_queue.start(id(sample), time.monotonic())
                try:
                    reply = await self._answer(sample)
                    if reply.get("ok") is not True:
                        logger.warning("manual intent delivery rejected: %s", reply.get("error"))
                finally:
                    intent_queue.finish(id(sample), time.monotonic())

        try:
            while True:
                try:
                    request = await _read(reader)
                except IpcError:
                    return
                # An identity handshake precedes either dedicated connection mode.
                if stream is None and request.get("type") == "identity":
                    writer.write(_encode(await self._answer(request)))
                    await writer.drain()
                    continue
                notification = request.get("type") == "manual_intent"
                if stream is None:
                    stream = notification
                elif stream != notification:
                    # Dedicated intent connections cannot mix replies with notifications.
                    return
                if notification:
                    if latest is not None:
                        intent_queue.discard(id(latest), time.monotonic(), overwritten=True)
                    latest = request
                    intent_queue.enqueue(id(request), time.monotonic())
                    if consumer is None or consumer.done():
                        consumer = asyncio.create_task(consume())
                else:
                    writer.write(_encode(await self._answer(request)))
                    await writer.drain()
        except (ConnectionError, IpcError):
            return
        finally:
            if latest is not None:
                intent_queue.discard(id(latest), time.monotonic())
            latest = None
            if consumer is not None:
                consumer.cancel()
                await asyncio.gather(consumer, return_exceptions=True)
            self._connections.discard(task)
            writer.close()

    async def _answer(self, request: Message) -> Message:
        if request.get("v") != PROTOCOL_VERSION or not isinstance(request.get("type"), str):
            return {"v": PROTOCOL_VERSION, "ok": False, "error": "bad_request"}
        try:
            token = object()
            self._handler_counter.enqueue(token, time.monotonic())
            self._handler_counter.start(token, time.monotonic())
            try:
                reply = await self._handler(request)
            finally:
                self._handler_counter.finish(token, time.monotonic())
        except Exception:
            return {"v": PROTOCOL_VERSION, "ok": False, "error": "internal"}
        return {"v": PROTOCOL_VERSION, **reply}


class IpcClient:
    """Agent-side requester. One request in flight; a failure drops the connection."""

    def __init__(
        self,
        path: Path,
        *,
        timeout_ms: float = 1000.0,
        robot_id: str | None = None,
        ros_domain: int | None = None,
        owner_record: str | None = None,
    ) -> None:
        self._owner_record = owner_record
        self._robot_id = robot_id
        self._ros_domain = ros_domain
        self._text = _check_path(path)
        self._timeout = timeout_ms / 1000
        self._lock = asyncio.Lock()
        self._queue_diagnostic = PendingCounter("ipc.client.lock_wait")
        self._reader: asyncio.StreamReader | None = None
        self._writer: asyncio.StreamWriter | None = None
        self._send_only: bool | None = None

    async def request(self, kind: str, **fields: Any) -> Message:
        if self._send_only is True:
            raise IpcError("intent connection cannot carry requests")
        self._send_only = False
        frame = _encode({"v": PROTOCOL_VERSION, "type": kind, **fields})
        token = object()
        self._queue_diagnostic.enqueue(token, time.monotonic())
        try:
            async with self._lock:
                self._queue_diagnostic.start(token, time.monotonic())
                try:
                    return await asyncio.wait_for(self._exchange(frame), timeout=self._timeout)
                except (asyncio.TimeoutError, OSError, IpcError) as exc:
                    self._drop()
                    raise IpcError(f"ipc request {kind} failed") from exc
                except asyncio.CancelledError:
                    self._drop()
                    raise
        finally:
            self._queue_diagnostic.finish(token, time.monotonic())

    async def send(self, kind: str, **fields: Any) -> None:
        """Write one intent notification, without reading an execution receipt.

        The Agent owns a single latest-value slot while local transport is busy.
        On the server a dedicated reader replaces queued values while one handoff runs.
        """
        if kind != "manual_intent" or self._send_only is False:
            raise IpcError("notifications require a dedicated intent connection")
        self._send_only = True
        frame = _encode({"v": PROTOCOL_VERSION, "type": kind, **fields})
        async with self._lock:
            try:
                await asyncio.wait_for(self._write(frame), timeout=self._timeout)
            except (asyncio.TimeoutError, OSError, IpcError) as exc:
                self._drop()
                raise IpcError("ipc intent transport failed") from exc
            except asyncio.CancelledError:
                self._drop()
                raise

    async def _write(self, frame: bytes) -> None:
        if self._writer is None or self._reader is None:
            identity_before = None
            if self._owner_record:
                # The supervisor writes this existing marker only after PID/UID peer proof.
                path = Path(self._text)
                try:
                    info = path.lstat()
                    identity_before = [info.st_dev, info.st_ino, info.st_uid]
                    recorded = json.loads(path.with_suffix(".owner.json").read_bytes())
                    if (
                        not stat.S_ISSOCK(info.st_mode)
                        or info.st_uid != os.geteuid()
                        or not isinstance(recorded, dict)
                        or recorded.get("identity") != identity_before
                        or recorded.get("run_id") != self._owner_record
                    ):
                        raise IpcError("adapter peer has not been verified by its owner")
                except (OSError, ValueError) as exc:
                    raise IpcError("adapter ownership record unavailable") from exc
            self._reader, self._writer = await asyncio.open_unix_connection(self._text)
            if identity_before is not None:
                info = Path(self._text).lstat()
                if [info.st_dev, info.st_ino, info.st_uid] != identity_before:
                    raise IpcError("adapter socket changed during connection")
            if self._robot_id is not None:
                # Each new connection proves identity before any business/stream frame.
                self._writer.write(
                    _encode({"v": PROTOCOL_VERSION, "type": "identity", "robot_id": self._robot_id})
                )
                await self._writer.drain()
                identity = await _read(self._reader)
                if (
                    identity.get("v") != PROTOCOL_VERSION
                    or identity.get("ok") is not True
                    or (
                        self._ros_domain is not None
                        and (
                            type(identity.get("ros_domain")) is not int
                            or identity["ros_domain"] != self._ros_domain
                        )
                    )
                ):
                    raise IpcError("ROS adapter identity/domain mismatch")
        self._writer.write(frame)
        await self._writer.drain()

    async def _exchange(self, frame: bytes) -> Message:
        await self._write(frame)
        assert self._reader is not None
        reply = await _read(self._reader)
        if reply.get("v") != PROTOCOL_VERSION:
            raise IpcError("unsupported ipc version")
        return reply

    def _drop(self) -> None:
        writer, self._writer, self._reader = self._writer, None, None
        if writer is not None:
            writer.close()

    async def close(self) -> None:
        self._drop()


def _identity(path: Path) -> list[int]:
    info = path.lstat()
    if not stat.S_ISSOCK(info.st_mode) or info.st_uid != os.geteuid():
        raise ValueError(f"foreign IPC path: {path.name}")
    return [info.st_dev, info.st_ino, info.st_uid]


def _record(path: Path, expected: list[int] | None = None, *, run_id: str | None = None) -> None:
    identity = _identity(path)
    if expected is not None and identity != expected:
        raise ValueError("IPC socket changed before ownership recording")
    marker = path.with_suffix(".owner.json")
    temporary = marker.with_suffix(".tmp")
    # The enclosing private directory and instance lock protect this ownership record.
    fd = os.open(temporary, os.O_CREAT | os.O_TRUNC | os.O_WRONLY | os.O_NOFOLLOW, 0o600)
    with os.fdopen(fd, "w") as file:
        json.dump(identity if run_id is None else {"identity": identity, "run_id": run_id}, file)
    os.replace(temporary, marker)


async def _prepare_socket(path: Path, *, expected_run_id: str | None = None) -> None:
    """Called under the instance lock; never replace a live or unrecognized socket."""
    if not os.path.lexists(path):
        return
    identity = _identity(path)
    probe = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    probe.setblocking(False)
    try:
        await asyncio.wait_for(asyncio.get_running_loop().sock_connect(probe, str(path)), 1)
    except ConnectionRefusedError:
        marker = path.with_suffix(".owner.json")
        try:
            owned = json.loads(marker.read_text())
            if expected_run_id is not None and (
                not isinstance(owned, dict) or owned.get("run_id") != expected_run_id
            ):
                owned = None
            # //Compatible: pre-isolation ownership records contain only the identity list.
            if isinstance(owned, dict):
                owned = owned.get("identity")
        except (OSError, ValueError):
            owned = None
        if owned != identity or _identity(path) != identity:
            raise ValueError(f"unrecognized stale IPC socket: {path.name}") from None
        path.unlink()
    else:
        raise ValueError(f"active IPC socket: {path.name}; stop its owner first")
    finally:
        probe.close()
