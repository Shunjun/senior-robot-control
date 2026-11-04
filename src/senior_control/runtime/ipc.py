"""Local IPC between the agent process and the ROS-side adapter.

Frames are a 4-byte big-endian length followed by UTF-8 JSON with ``"v": 1``. The transport
is a Unix domain socket owned by the current user; nothing here can bind a network port.
A frame longer than ``MAX_FRAME`` is never read: the peer is disconnected instead.
"""

import asyncio
import json
import os
import socket
import stat
import struct
from collections.abc import Awaitable, Callable
from pathlib import Path
from typing import Any

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
        message = json.loads(body)
    except ValueError as exc:
        raise IpcError("frame is not json") from exc
    if not isinstance(message, dict):
        raise IpcError("frame is not an object")
    return message


class IpcServer:
    """ROS-side listener. The handler answers one request per frame."""

    def __init__(self, path: Path, handler: Handler) -> None:
        self._path = path
        self._text = _check_path(path)
        self._handler = handler
        self._server: asyncio.AbstractServer | None = None
        self._start_task: asyncio.Task[asyncio.CancelledError | None] | None = None
        self._closing = False
        self._closed = asyncio.Event()
        self._closed.set()
        self._socket_identity: tuple[int, int] | None = None
        self._connections: set[asyncio.Task[None]] = set()

    async def start(self) -> None:
        if self._server is not None or self._closing or (
            self._start_task is not None and not self._start_task.done()
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
            if (current is not None and stat.S_ISSOCK(current.st_mode)
                    and (current.st_dev, current.st_ino) == self._socket_identity):
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
                    if (stat.S_ISSOCK(current.st_mode)
                            and (current.st_dev, current.st_ino) == identity):
                        os.unlink(self._path)
                except FileNotFoundError:
                    pass
                self._socket_identity = None

    async def _serve(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        task = asyncio.current_task()
        assert task is not None
        self._connections.add(task)
        try:
            while True:
                try:
                    request = await _read(reader)
                except IpcError:
                    return
                writer.write(_encode(await self._answer(request)))
                await writer.drain()
        except (ConnectionError, IpcError):
            return
        finally:
            self._connections.discard(task)
            writer.close()

    async def _answer(self, request: Message) -> Message:
        if request.get("v") != PROTOCOL_VERSION or not isinstance(request.get("type"), str):
            return {"v": PROTOCOL_VERSION, "ok": False, "error": "bad_request"}
        try:
            reply = await self._handler(request)
        except Exception:
            return {"v": PROTOCOL_VERSION, "ok": False, "error": "internal"}
        return {"v": PROTOCOL_VERSION, **reply}


class IpcClient:
    """Agent-side requester. One request in flight; a failure drops the connection."""

    def __init__(self, path: Path, *, timeout: float = 1.0) -> None:
        self._text = _check_path(path)
        self._timeout = timeout
        self._lock = asyncio.Lock()
        self._reader: asyncio.StreamReader | None = None
        self._writer: asyncio.StreamWriter | None = None

    async def request(self, kind: str, **fields: Any) -> Message:
        frame = _encode({"v": PROTOCOL_VERSION, "type": kind, **fields})
        async with self._lock:
            try:
                return await asyncio.wait_for(self._exchange(frame), timeout=self._timeout)
            except (asyncio.TimeoutError, OSError, IpcError) as exc:
                self._drop()
                raise IpcError(f"ipc request {kind} failed") from exc
            except asyncio.CancelledError:
                self._drop()
                raise

    async def _exchange(self, frame: bytes) -> Message:
        if self._writer is None or self._reader is None:
            self._reader, self._writer = await asyncio.open_unix_connection(self._text)
        self._writer.write(frame)
        await self._writer.drain()
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
