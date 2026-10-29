"""Agent seams: the cloud link, per-robot heartbeat eligibility, and the ROS IPC motion port."""

from dataclasses import dataclass
from typing import Any, Protocol

from senior_control.agent.snapshots import RobotSnapshot
from senior_control.clock import Clock
from senior_control.protocol import (
    Envelope,
    ResponseEnvelope,
    SimpleEnvelope,
    VelocityEnvelope,
    serialize_envelope,
)
from senior_control.runtime.ipc import IpcClient, IpcError


@dataclass(frozen=True)
class HeartbeatFrame:
    """A cloud heartbeat addressed to one robot."""

    robot_id: str


class Link(Protocol):
    """One authenticated cloud socket. ``epoch`` is the number the relay assigned to it."""

    epoch: int

    async def recv(self) -> Envelope | HeartbeatFrame:
        """Next frame. A dropped socket raises ``ConnectionError``."""
        ...

    async def send_response(self, envelope: ResponseEnvelope) -> None: ...

    async def send_snapshot(self, snapshot: RobotSnapshot) -> None: ...

    async def send_heartbeat(self) -> None: ...

    async def close(self) -> None: ...


class HeartbeatLedger:
    """Remembers when each robot id last had a heartbeat; ids never borrow each other's."""

    def __init__(self, clock: Clock, window: float) -> None:
        if window <= 0:
            raise ValueError("heartbeat window must be positive")
        self._clock = clock
        self._window = window
        self._seen: dict[str, float] = {}

    def note(self, robot_id: str) -> None:
        self._seen[robot_id] = self._clock.monotonic()

    def eligible(self, robot_id: str) -> bool:
        seen = self._seen.get(robot_id)
        return seen is not None and self._clock.monotonic() - seen <= self._window


class MotionPort(Protocol):
    """The only way the agent reaches motion: through the ROS-side adapter."""

    async def drive(self, envelope: VelocityEnvelope) -> None: ...

    async def stop(self, envelope: SimpleEnvelope) -> None: ...

    async def estop(self, envelope: SimpleEnvelope) -> None: ...

    async def forward(self, envelope: Envelope) -> None: ...

    async def disable(self, connection_epoch: int) -> None: ...

    async def exit(self) -> None: ...

    async def call(self, name: str) -> Any: ...

    async def close(self) -> None: ...


class IpcMotion:
    """``MotionPort`` over the local socket. It never touches rclpy or ``/cmd_vel``."""

    def __init__(self, client: IpcClient) -> None:
        self._client = client

    async def _ok(self, kind: str, **fields: Any) -> dict[str, Any]:
        reply = await self._client.request(kind, **fields)
        if reply.get("ok") is not True:
            raise IpcError(f"ros side refused {kind}")
        return reply

    async def drive(self, envelope: VelocityEnvelope) -> None:
        await self._ok("drive", envelope=serialize_envelope(envelope).decode())

    async def stop(self, envelope: SimpleEnvelope) -> None:
        await self._ok("stop", envelope=serialize_envelope(envelope).decode())

    async def estop(self, envelope: SimpleEnvelope) -> None:
        await self._ok("estop", envelope=serialize_envelope(envelope).decode())

    async def forward(self, envelope: Envelope) -> None:
        await self._ok("command", envelope=serialize_envelope(envelope).decode())

    async def disable(self, connection_epoch: int) -> None:
        await self._ok("disable_motion", connection_epoch=connection_epoch)

    async def exit(self) -> None:
        await self._ok("exit")

    async def call(self, name: str) -> Any:
        return (await self._ok("ros_call", name=name)).get("result")

    async def close(self) -> None:
        await self._client.close()
