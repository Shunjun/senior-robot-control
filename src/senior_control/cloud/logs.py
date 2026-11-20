"""//Compatible: historical imports forward to the sole cloud implementation."""

import asyncio
from dataclasses import dataclass, field
from typing import Any

from cloud.logs import LogPeer as _LogPeer
from cloud.logs import LogRelay as _LogRelay

from senior_control.runtime.diagnostics import DiagnosticAsyncQueue


@dataclass(eq=False)
class LogPeer(_LogPeer):
    """//Compatible: preserve the old default queue; no duplicated relay business."""

    queue: asyncio.Queue[dict[str, Any]] = field(
        default_factory=lambda: DiagnosticAsyncQueue("cloud.logs_outbox", 32)
    )


class LogRelay(_LogRelay):
    """//Compatible: inject the original bounded, instrumented queues."""

    def __init__(self) -> None:
        super().__init__(queue_factory=DiagnosticAsyncQueue)
