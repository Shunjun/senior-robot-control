"""Real relay/WebSocketLink frames with an explicitly advanced online clock."""

import asyncio
import json
from typing import Any

from senior_control.agent.websocket import WebSocketLink
from senior_control.cloud.limits import CloudLimits
from senior_control.cloud.relay import Hub


class HeartbeatClock:
    def __init__(self, monkeypatch: Any, now: float = 10.0) -> None:
        self.now = now
        self.timers: list[tuple[float, Any, Any]] = []
        loop = asyncio.get_running_loop()
        original = loop.call_later

        def call_later(delay: float, callback: Any, *args: Any, **kwargs: Any) -> Any:
            if isinstance(getattr(callback, "__self__", None), WebSocketLink):
                handle = asyncio.Handle(callback, args, loop)
                self.timers.append((self.now + delay, handle, lambda: callback(*args)))
                return handle
            return original(delay, callback, *args, **kwargs)

        monkeypatch.setattr(loop, "call_later", call_later)

    def monotonic(self) -> float:
        return self.now

    def advance(self, now: float) -> None:
        self.now = now
        for deadline, handle, fire in list(self.timers):
            if deadline <= now and not handle.cancelled():
                handle.cancel()
                fire()


class RelaySocket:
    """Authenticated Hub connection, preserving real encoded transport and close events."""

    def __init__(self, hub: Hub, robot: str, *, drop_replies: bool = False) -> None:
        self.hub = hub
        self.connection = hub.attach_agent(robot)
        self.drop_replies = drop_replies
        self.closed = False
        self.sent: list[dict[str, Any]] = []

    async def send(self, data: str) -> None:
        self.sent.append(json.loads(data))
        self.hub.handle_agent_frame(self.connection, data.encode())

    async def recv(self) -> str:
        while True:
            raw = await self.connection.outbox.get()
            if isinstance(raw, int):
                raise ConnectionError("relay socket closed")
            if self.drop_replies and json.loads(raw)["type"] == "heartbeat.reply":
                continue
            return raw.decode()

    async def close(self) -> None:
        self.closed = True
        self.hub.detach(self.connection)


def relay_hub(monkeypatch: Any, clock: HeartbeatClock) -> Hub:
    import time
    from types import SimpleNamespace

    monkeypatch.setattr(
        "senior_control.cloud.relay.time",
        SimpleNamespace(monotonic=clock.monotonic, time=time.time),
    )
    return Hub(CloudLimits())
