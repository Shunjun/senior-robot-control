"""Independent authenticated log relay. No control outbox or control epoch changes."""

import asyncio
import json
import secrets
from collections.abc import Awaitable, Callable
from contextlib import suppress
from dataclasses import dataclass, field
from typing import Any

from fastapi import WebSocket, WebSocketDisconnect

from senior_control.vehicle_logs.wire import LOG_WIRE_BYTES, LogRequest


@dataclass(eq=False)
class LogPeer:
    queue: asyncio.Queue[dict[str, Any]] = field(default_factory=lambda: asyncio.Queue(32))
    closed: asyncio.Event = field(default_factory=asyncio.Event)

    def offer(self, frame: dict[str, Any]) -> bool:
        if self.closed.is_set():
            return False
        try:
            self.queue.put_nowait(frame)
            return True
        except asyncio.QueueFull:
            self.closed.set()
            return False


class LogRelay:
    def __init__(self) -> None:
        self.agents: dict[str, LogPeer] = {}
        self.viewers: dict[str, tuple[str, LogPeer]] = {}

    def close_all(self) -> None:
        for peer in self.agents.values():
            peer.closed.set()
        for _, peer in self.viewers.values():
            peer.closed.set()

    async def agent(
        self, socket: WebSocket, robot: str, valid: Callable[[], Awaitable[bool]]
    ) -> None:
        peer = LogPeer()
        old = self.agents.get(robot)
        if old:
            old.closed.set()
        self.agents[robot] = peer
        await socket.accept()

        async def read() -> None:
            while True:
                raw = await socket.receive_text()
                if len(raw.encode()) > LOG_WIRE_BYTES:
                    raise ValueError("log frame too large")
                frame = await asyncio.to_thread(json.loads, raw)
                if not isinstance(frame, dict) or set(frame) != {"client", "ok", "data"}:
                    raise ValueError("invalid log reply")
                target = self.viewers.get(frame["client"])
                if target and target[0] == robot:
                    if not target[1].offer({"ok": frame["ok"], "data": frame["data"]}):
                        peer.offer({"client": frame["client"], "action": "close"})

        try:
            await self._serve(socket, peer, read, valid)
        finally:
            if self.agents.get(robot) is peer:
                self.agents.pop(robot, None)
                for vehicle, viewer in self.viewers.values():
                    if vehicle == robot:
                        viewer.closed.set()

    async def viewer(
        self, socket: WebSocket, robot: str, valid: Callable[[], Awaitable[bool]]
    ) -> None:
        agent = self.agents.get(robot)
        if agent is None or agent.closed.is_set() or len(self.viewers) >= 32:
            await socket.close(code=1013)
            return
        client, peer = secrets.token_hex(16), LogPeer(queue=asyncio.Queue(4))
        self.viewers[client] = robot, peer
        await socket.accept()

        async def read() -> None:
            # One bounded operation per independent socket; pause closes follow.
            raw = await socket.receive_text()
            if len(raw.encode()) > 8192:
                raise ValueError("log request too large")
            request = LogRequest.model_validate_json(raw)
            if not await valid() or not agent.offer(
                {"client": client, **request.model_dump(mode="json")}
            ):
                raise PermissionError("log access unavailable")
            await socket.receive_text()
            raise ValueError("one log operation per socket")

        try:
            await self._serve(socket, peer, read, valid)
        finally:
            self.viewers.pop(client, None)
            agent.offer({"client": client, "action": "close"})

    @staticmethod
    async def _serve(
        socket: WebSocket,
        peer: LogPeer,
        read: Callable[[], Awaitable[None]],
        valid: Callable[[], Awaitable[bool]],
    ) -> None:
        async def write() -> None:
            while True:
                frame = await peer.queue.get()
                raw = await asyncio.to_thread(json.dumps, frame, ensure_ascii=False)
                if not await valid():
                    raise PermissionError("log authorization expired")
                await asyncio.wait_for(socket.send_text(raw), 5)

        async def watch() -> None:
            while not peer.closed.is_set():
                if not await valid():
                    return
                with suppress(asyncio.TimeoutError):
                    await asyncio.wait_for(peer.closed.wait(), 1)

        tasks = [
            asyncio.ensure_future(read()),
            asyncio.create_task(write()),
            asyncio.create_task(watch()),
        ]
        try:
            await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
        finally:
            peer.closed.set()
            for task in tasks:
                task.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)
            with suppress(WebSocketDisconnect, RuntimeError):
                await socket.close(code=1000)
