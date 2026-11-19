"""Only the existing agent foundations' caller-owned values and diagnostics."""

import asyncio
from collections.abc import Callable
from dataclasses import dataclass
from typing import Generic, Protocol, TypeVar

from senior_contracts.wire import SnapshotEnvelope

SnapshotT = TypeVar("SnapshotT")
HeartbeatT = TypeVar("HeartbeatT")
LinkT = TypeVar("LinkT")
EntryT = TypeVar("EntryT")


class StatusReading(Protocol):
    @property
    def value(self) -> str: ...


StatusT = TypeVar("StatusT", bound=StatusReading)


@dataclass(frozen=True)
class WebSocketPorts(Generic[SnapshotT, HeartbeatT]):
    heartbeat: Callable[[str], HeartbeatT]
    snapshot: Callable[..., SnapshotEnvelope]
    send_lock: Callable[[], asyncio.Lock]
    receive_diagnostic: Callable[[object], None]


@dataclass(frozen=True)
class JournalPorts(Generic[EntryT, StatusT]):
    entry: Callable[..., EntryT]
    status: Callable[[str], StatusT]
    active: tuple[StatusT, ...]
    running: StatusT
    needs_review: StatusT
