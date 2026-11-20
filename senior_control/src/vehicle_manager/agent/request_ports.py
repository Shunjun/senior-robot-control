"""Only caller-owned constructors and map read effects needed by management."""

from collections.abc import Awaitable, Callable, Mapping
from dataclasses import dataclass
from typing import Any, Generic, Protocol, TypeVar

from vehicle_manager.tasks.ports import MapReference, PoseReading
from vehicle_manager.vehicle_logs.diagnostics import QueueCounter


class Requester(Protocol):
    async def request(self, kind: str, **fields: Any) -> dict[str, Any]: ...


class Metrics(Protocol):
    def inc(self, name: str, *, operation: str = "none", result: str = "none") -> None: ...


ResultT = TypeVar("ResultT")


@dataclass(frozen=True)
class RequestValues(Generic[ResultT]):
    map_reference: Callable[[str, int], MapReference]
    error: type[Any]
    status: Any
    result: Callable[..., ResultT]
    not_committed: Callable[..., Exception]
    task_result: Callable[..., ResultT]
    direct_navigation: Callable[..., Callable[[str, str, Mapping[str, Any]], Awaitable[None]]]
    counter: Callable[[str], QueueCounter]


@dataclass(frozen=True)
class WorkspacePorts:
    map_reference: Callable[[str, int], MapReference]
    pose: Callable[[float, float, float], PoseReading]
    error: type[Any]
    ipc_error: type[Exception]
    map_reply: type[Any]
    manifest: type[Any]
    tile: type[Any]
    waypoint_create: type[Any]
    max_grid_cells: int
    tile_size: int
    exported_read: Callable[[Any, str, Mapping[str, Any]], dict[str, Any]]


class HomeServicePort(Protocol):
    async def set_current(self, pose: Any, *, expected_revision: int) -> object: ...
    async def home_revision(self) -> int: ...
