"""Only the original waypoint inputs and per-store constructors."""

from collections.abc import Awaitable, Callable
from dataclasses import dataclass
from typing import Generic, Protocol, TypeVar

from senior_contracts.errors import ErrorCode

from vehicle_manager.tasks.ports import MapReference, PoseReading

WaypointT = TypeVar("WaypointT")
StoredT = TypeVar("StoredT")
PoseT = TypeVar("PoseT", bound=PoseReading)


class BoundWaypointReading(Protocol):
    @property
    def waypoint_id(self) -> str: ...

    @property
    def pose(self) -> PoseReading: ...


class BindingReading(Protocol):
    @property
    def waypoint(self) -> BoundWaypointReading: ...

    @property
    def anchor_pose(self) -> PoseReading: ...


@dataclass(frozen=True)
class WaypointStorePorts(Generic[WaypointT, StoredT, PoseT]):
    """Keep caller-owned value classes and domain preparation on the same database.

    The result types follow the injected constructors; no replacement values or
    copied admission objects are introduced by the storage implementation.
    """

    prepare_schema: Callable[[], Awaitable[None]]
    map_reference: Callable[[str, int], MapReference]
    pose: Callable[[float, float, float], PoseT]
    waypoint: Callable[..., WaypointT]
    stored: Callable[[WaypointT, PoseT, bool | None, str | None], StoredT]
    error: Callable[[ErrorCode, str], Exception]
