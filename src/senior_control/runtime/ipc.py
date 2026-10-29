"""Local IPC between the agent process and the ROS-side adapter.

Frames are a 4-byte big-endian length followed by UTF-8 JSON with ``"v": 1``. The transport
is a Unix domain socket owned by the current user; nothing here can bind a network port.
A frame longer than ``MAX_FRAME`` is never read: the peer is disconnected instead.
"""

import asyncio
import json
import os
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
        self._connections: set[asyncio.Task[None]] = set()

    async def start(self) -> None:
        if self._server is not None:
            raise RuntimeError("ipc server already started")
        try:
            if stat.S_ISSOCK(os.stat(self._path).st_mode):
                os.unlink(self._path)
        except FileNotFoundError:
            pass
        previous = os.umask(0o177)
        try:
            self._server = await asyncio.start_unix_server(self._serve, path=self._text)
        finally:
            os.umask(previous)
        os.chmod(self._path, 0o600)

    async def close(self) -> None:
        server, self._server = self._server, None
        if server is not None:
            server.close()
        for task in list(self._connections):
            task.cancel()
        await asyncio.gather(*self._connections, return_exceptions=True)
        if server is not None:
            await server.wait_closed()
        try:
            os.unlink(self._path)
        except FileNotFoundError:
            pass

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
