"""Agent link and client consumption shapes; no local IPC execution or ROS imports."""

from dataclasses import dataclass
from typing import Any, Protocol

from senior_contracts.wire import (
    Envelope,
    ManualIntentEnvelope,
    ResponseEnvelope,
    SimpleEnvelope,
    VideoEnvelope,
)

from vehicle_manager.agent.snapshots import RobotSnapshot


@dataclass(frozen=True)
class HeartbeatFrame:
    """A cloud heartbeat addressed to one robot."""

    robot_id: str


class Link(Protocol):
    """One authenticated cloud socket. ``epoch`` is the number the relay assigned to it."""

    epoch: int

    async def recv(self) -> Envelope | VideoEnvelope | HeartbeatFrame:
        """Next frame. A dropped socket raises ``ConnectionError``."""
        ...

    async def send_response(self, envelope: ResponseEnvelope) -> None: ...

    async def send_video(self, envelope: VideoEnvelope) -> None: ...

    async def send_snapshot(self, snapshot: RobotSnapshot) -> None: ...

    async def send_heartbeat(self) -> None: ...

    async def close(self) -> None: ...


class MotionPort(Protocol):
    """The only way the agent reaches motion: through the ROS-side adapter."""

    async def manual(
        self,
        envelope: Envelope,
        *,
        received_steady: float | None = None,
        deadline_steady: float | None = None,
        **fields: Any,
    ) -> dict[str, Any]: ...

    async def learning_available(self) -> bool: ...

    async def learning(
        self, envelope: Envelope, *, received_steady: float, deadline_steady: float
    ) -> dict[str, Any]: ...

    async def send_intent(
        self,
        envelope: ManualIntentEnvelope,
        *,
        received_steady: float,
        deadline_steady: float,
    ) -> None: ...

    async def estop_received(self, envelope: SimpleEnvelope, *, received_steady: float) -> None: ...

    async def stop(self, envelope: SimpleEnvelope) -> None: ...

    async def estop(self, envelope: SimpleEnvelope) -> None: ...

    async def disable(self, connection_epoch: int) -> None: ...

    async def exit(self) -> None: ...

    async def call(self, name: str) -> Any: ...

    async def close(self) -> None: ...
