"""In-memory ports for the robot agent tests. Nothing here touches ROS, a socket, or a network."""

from __future__ import annotations

import asyncio
import time
from collections.abc import Awaitable, Callable, Mapping
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from senior_control.agent.snapshots import LiveState
from senior_control.errors import DomainError, ErrorCode
from senior_control.models import MapRef, Pose2D
from senior_control.protocol import (
    EmptyPayload,
    Envelope,
    OperationEnvelope,
    OperationPayload,
    ResponseEnvelope,
    SimpleEnvelope,
    VelocityEnvelope,
    VelocityPayload,
)
from senior_control.waypoints import Waypoint

ROBOT = "robot-1"
MAP = MapRef("map-1", 3)


def operation(
    kind: str,
    operation_id: str,
    body: dict[str, Any] | None = None,
    *,
    epoch: int = 1,
    expected_revision: int | None = None,
    robot_id: str = ROBOT,
    expires_in: float = 30.0,
) -> OperationEnvelope:
    return OperationEnvelope(
        v=1,
        robot_id=robot_id,
        session_id="session-1",
        connection_epoch=epoch,
        request_id=f"req-{operation_id}",
        seq=1,
        expires_at=time.time() + expires_in,
        type=kind,  # type: ignore[arg-type]
        payload=OperationPayload(
            operation_id=operation_id, expected_revision=expected_revision, body=body or {}
        ),
    )


def stop(epoch: int = 1, robot_id: str = ROBOT) -> SimpleEnvelope:
    return SimpleEnvelope(
        v=1,
        robot_id=robot_id,
        session_id="session-1",
        connection_epoch=epoch,
        request_id="req-stop",
        seq=2,
        expires_at=time.time() + 30,
        type="drive.stop",
        payload=EmptyPayload(),
    )


def velocity(epoch: int = 1, robot_id: str = ROBOT) -> VelocityEnvelope:
    return VelocityEnvelope(
        v=1,
        robot_id=robot_id,
        session_id="session-1",
        connection_epoch=epoch,
        request_id="req-velocity",
        seq=3,
        expires_at=time.time() + 30,
        type="drive.velocity",
        payload=VelocityPayload(vx=0.2, wz=0.0),
    )


async def until(predicate: Callable[[], bool], timeout: float = 2.0) -> None:
    deadline = time.monotonic() + timeout
    while not predicate():
        if time.monotonic() > deadline:
            raise AssertionError("condition was not reached in time")
        await asyncio.sleep(0.005)


class FakeLink:
    def __init__(self, epoch: int) -> None:
        self.epoch = epoch
        self.inbox: asyncio.Queue[Envelope | object | None] = asyncio.Queue()
        self.responses: list[ResponseEnvelope] = []
        self.snapshots: list[Any] = []
        self.heartbeats = 0
        self.closed = False

    async def recv(self) -> Envelope | object:
        item = await self.inbox.get()
        if item is None:
            raise ConnectionError("link dropped")
        return item

    async def send_response(self, envelope: ResponseEnvelope) -> None:
        self.responses.append(envelope)

    async def send_snapshot(self, snapshot: Any) -> None:
        self.snapshots.append(snapshot)

    async def send_heartbeat(self) -> None:
        self.heartbeats += 1

    async def close(self) -> None:
        self.closed = True

    def drop(self) -> None:
        self.inbox.put_nowait(None)


class FakeConnector:
    """Hands out one link per connection attempt; later attempts wait for the next link."""

    def __init__(self, *links: FakeLink) -> None:
        self._queue: asyncio.Queue[FakeLink] = asyncio.Queue()
        for link in links:
            self._queue.put_nowait(link)

    def add(self, link: FakeLink) -> None:
        self._queue.put_nowait(link)

    async def __call__(self) -> FakeLink:
        return await self._queue.get()


class FakeMotion:
    def __init__(self, events: list[str] | None = None) -> None:
        self.events: list[str] = events if events is not None else []

    async def drive(self, vx: float, wz: float) -> None:
        self.events.append(f"drive:{vx}:{wz}")

    async def stop(self) -> None:
        self.events.append("stop")

    async def estop(self) -> None:
        self.events.append("estop")

    async def forward(self, envelope: Envelope) -> None:
        self.events.append(f"forward:{envelope.type}")

    async def disable(self) -> None:
        self.events.append("disable")

    async def exit(self) -> None:
        self.events.append("exit")

    async def call(self, name: str) -> Any:
        return name

    async def close(self) -> None:
        self.events.append("close")


class FakeTasks:
    def __init__(self) -> None:
        self.calls: list[tuple[str, Mapping[str, Any], int | None]] = []
        self.committed: list[str] = []
        self.revision = 1
        self.gate: asyncio.Event | None = None
        self.fail: DomainError | None = None

    async def _commit(self, name: str, body: Mapping[str, Any], expected: int | None) -> int:
        self.calls.append((name, body, expected))
        if self.gate is not None:
            await self.gate.wait()
        if self.fail is not None:
            raise self.fail
        if expected is not None and expected != self.revision:
            raise DomainError(ErrorCode.CONFLICT, "revision conflict")
        self.revision += 1
        self.committed.append(name)
        return self.revision

    async def create(self, body: Mapping[str, Any]) -> int:
        return await self._commit("create", body, None)

    async def update(self, body: Mapping[str, Any], expected_revision: int) -> int:
        return await self._commit("update", body, expected_revision)

    async def delete(self, task_id: str, expected_revision: int) -> int:
        return await self._commit("delete", {"task_id": task_id}, expected_revision)

    async def set_enabled(self, task_id: str, enabled: bool, expected_revision: int) -> int:
        return await self._commit(
            "enable", {"task_id": task_id, "enabled": enabled}, expected_revision
        )


class FakeHome:
    def __init__(self) -> None:
        self.calls = 0

    async def set_current(self) -> None:
        self.calls += 1


class FakeMaps:
    def __init__(self) -> None:
        self.gate: asyncio.Event | None = None
        self.calls: list[str] = []

    async def _run(self, name: str) -> None:
        self.calls.append(name)
        if self.gate is not None:
            await self.gate.wait()

    async def export(self, operation_id: str, map_ref: MapRef, destination: Path) -> None:
        await self._run("export")

    async def import_(self, operation_id: str, path: Path) -> None:
        await self._run("import")

    async def clear(self, operation_id: str, expected: MapRef) -> None:
        await self._run("clear")


@dataclass
class FakeState:
    task_revision: int = 4
    localized: bool = True
    execution_status: str = "idle"
    calls: int = field(default=0)

    async def collect(self) -> LiveState:
        self.calls += 1
        home = Waypoint(
            id="home",
            kind="home",
            pose=Pose2D(0.0, 0.0, 0.0),
            map_ref=MAP,
            anchor_id="anchor-1",
            source_robot_id=ROBOT,
            status="ready",
        )
        return LiveState(
            localized=self.localized,
            map_ref=MAP,
            home=home,
            task_revision=self.task_revision,
            execution_status=self.execution_status,
        )


NoCleanup = Callable[[], Awaitable[None]]
