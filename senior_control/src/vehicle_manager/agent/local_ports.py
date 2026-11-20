"""Only caller-owned transport/errors and concrete navigation constructors.

The Unix codec, peer identity and all local value classes stay with their owners.
These inputs bind each management adapter to those same original objects.
"""

from collections.abc import Callable
from dataclasses import dataclass
from typing import Any, Protocol

from vehicle_manager.agent.request_ports import Requester


class MotionClient(Requester, Protocol):
    async def send(self, kind: str, **fields: Any) -> None: ...
    async def close(self) -> None: ...


@dataclass(frozen=True)
class MotionErrors:
    domain: type[Any]
    ipc: type[Exception]
    rejected: type[Any]


@dataclass(frozen=True)
class NavigationValues:
    error: type[Any]
    ipc_error: type[Exception]
    pose: Callable[[float, float, float], Any]
    route_plan: Callable[..., Any]
    route_rejected: type[Exception]
