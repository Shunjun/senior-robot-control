"""Explicit per-Agent bindings to existing values, diagnostics and media effects.

No default registry or replacement state owner; the production boundary provides
these original constructors for each Agent instance.
"""

from collections.abc import Callable, Hashable, MutableMapping
from dataclasses import dataclass
from typing import Any, Generic, Protocol, TypeVar

from senior_contracts.wire import Envelope, VideoEnvelope

from vehicle_manager.agent.connection import Link
from vehicle_manager.agent.dispatcher import Dispatcher, LifecycleNavigationOperations
from vehicle_manager.agent.ports import StatusReading
from vehicle_manager.agent.request_ports import Metrics
from vehicle_manager.vehicle_logs.diagnostics import QueueCounter


class OperationReading(Protocol):
    @property
    def operation_id(self) -> str: ...
    @property
    def status(self) -> StatusReading: ...


ResultT = TypeVar("ResultT", bound=OperationReading)


class Clock(Protocol):
    def monotonic(self) -> float: ...


class IntentCounter(QueueCounter, Protocol):
    def discard(self, key: Hashable, now: float, *, overwritten: bool = False) -> None: ...


class Signaling(Protocol):
    def submit(self, frame: VideoEnvelope) -> None: ...
    def retire(self) -> None: ...
    async def close(self) -> None: ...


@dataclass(frozen=True)
class AgentRuntimePorts(Generic[ResultT]):
    error: type[Any]
    status: Any
    result: Callable[..., ResultT]
    manual_result: Callable[..., ResultT]
    manual_result_type: type[Any]
    task_result: type[Any]
    not_committed: type[Any]
    motion_rejected: type[Any]
    ipc_error: type[Exception]
    dispatcher: Callable[..., Dispatcher[ResultT]]
    lifecycle_navigation: type[LifecycleNavigationOperations[ResultT]]
    clock: Callable[[], Clock]
    metrics: Callable[[], Metrics]
    counter: Callable[[str, int], IntentCounter]
    safety_pending: Callable[
        [str, int], MutableMapping[tuple[str, int, str], tuple[Link, Envelope]]
    ]
    report_queues: Callable[[object, int, int], None]
    video_signaling: Callable[..., Signaling]
