"""Log source and dedicated cloud transport; neither awaits the control connection.

ROS /rosout is not wired here. This source records actual Python Agent events.
"""

import asyncio
import json
import logging
from typing import Any
from urllib.parse import urlsplit, urlunsplit

from senior_control.settings.models import Level
from senior_control.vehicle_logs.models import LogInput
from senior_control.vehicle_logs.service import Action, VehicleLogService
from senior_control.vehicle_logs.wire import LOG_WIRE_BYTES, LogRequest


def log_url(control_url: str) -> str:
    url = urlsplit(control_url)
    return urlunsplit((url.scheme, url.netloc, "/ws/agent/logs", "", ""))


class CloudLogGrants:
    """Opaque grants only received over the device's authenticated TLS cloud link.

    Cloud checks real session/device revocation at receipt, before each delivery,
    and while idle. Close/cancel revokes the local grant; payload has no auth bool.
    """

    def __init__(self) -> None:
        self.active: set[str] = set()

    async def require_vehicle(self, credential: str, vehicle_id: str, action: Action) -> None:
        if credential not in self.active:
            raise PermissionError("log transport grant expired")


class AgentLogHandler(logging.Handler):
    def __init__(self, service: VehicleLogService) -> None:
        super().__init__(logging.DEBUG)
        self.service = service

    def emit(self, record: logging.LogRecord) -> None:
        levels: dict[int, Level] = {
            10: "DEBUG",
            20: "INFO",
            30: "WARNING",
            40: "ERROR",
            50: "CRITICAL",
        }
        try:
            self.service.enqueue(
                LogInput(
                    level=levels.get(record.levelno, "INFO"),
                    module=record.name,
                    summary=record.getMessage()[:512],
                    source_utc=record.created,
                    fields={
                        "event": getattr(record, "event", "agent_log"),
                        "request_type": getattr(record, "request_type", None),
                        "result": getattr(record, "result", None),
                    },
                    request_id=getattr(record, "request_id", None),
                    task_id=getattr(record, "task_id", None),
                )
            )
        except Exception:
            # A logging source must never break its control caller or recursively log.
            return


class AgentLogConnection:
    def __init__(self, service: VehicleLogService, grants: CloudLogGrants) -> None:
        self.service, self.grants = service, grants
        self.connected = False

    async def serve(self, socket: Any) -> None:
        pending: dict[str, asyncio.Task[None]] = {}
        send_lock = asyncio.Lock()

        async def run(client: str, request: LogRequest) -> None:
            subscription = None

            async def send(data: dict[str, Any]) -> None:
                async with send_lock:
                    raw = await asyncio.to_thread(
                        json.dumps, {"client": client, "ok": True, "data": data}, ensure_ascii=False
                    )
                    await asyncio.wait_for(socket.send(raw), 5)

            try:
                if request.action == "follow":
                    subscription = await self.service.follow(
                        client, self.service.vehicle_id, request.query
                    )
                    while True:
                        await send((await subscription.next_page()).model_dump(mode="json"))
                elif request.action == "download":
                    page = await self.service.download(
                        client, self.service.vehicle_id, request.query
                    )
                    await send(
                        {
                            "data": page.data.decode("utf-8"),
                            "next_cursor": page.next_cursor,
                            "has_more": page.has_more,
                            "gaps": [g.model_dump() for g in page.gaps],
                        }
                    )
                else:
                    result = await self.service.query(
                        client, self.service.vehicle_id, request.query
                    )
                    await send(result.model_dump(mode="json"))
            except (ValueError, RuntimeError, PermissionError):
                async with send_lock:
                    await socket.send(json.dumps({"client": client, "ok": False, "data": None}))
            finally:
                if subscription:
                    await subscription.close()
                self.grants.active.discard(client)

        self.connected = True
        try:
            async for raw in socket:
                if len(raw) > LOG_WIRE_BYTES:
                    raise ValueError("log request too large")
                frame = json.loads(raw)
                client = frame.pop("client")
                if not isinstance(client, str) or len(client) != 32:
                    raise ValueError("invalid log grant")
                if frame == {"action": "close"}:
                    self.grants.active.discard(client)
                    task = pending.pop(client, None)
                    if task:
                        task.cancel()
                        await asyncio.gather(task, return_exceptions=True)
                    continue
                for key, task in list(pending.items()):
                    if task.done():
                        task.result()
                        pending.pop(key)
                if client in pending or len(pending) >= 32:
                    raise ValueError("log operation quota")
                request = LogRequest.model_validate_json(json.dumps(frame))
                self.grants.active.add(client)
                pending[client] = asyncio.create_task(run(client, request))
        finally:
            self.connected = False
            self.grants.active.clear()
            for task in pending.values():
                task.cancel()
            await asyncio.gather(*pending.values(), return_exceptions=True)

    async def run(self, url: str, token: str) -> None:
        from websockets.asyncio.client import connect

        while True:
            try:
                async with connect(
                    log_url(url),
                    additional_headers={"Authorization": f"Bearer {token}"},
                    open_timeout=5,
                    close_timeout=1,
                    max_size=LOG_WIRE_BYTES,
                    max_queue=4,
                ) as socket:
                    await self.serve(socket)
            except asyncio.CancelledError:
                raise
            except Exception:
                # Independent best effort logs never fail the control supervisor.
                await asyncio.sleep(1)
