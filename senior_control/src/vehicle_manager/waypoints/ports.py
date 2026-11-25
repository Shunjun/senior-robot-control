"""Only the original waypoint inputs and per-store constructors."""

from collections.abc import Awaitable, Callable
from dataclasses import dataclass
from typing import Generic, Protocol, TypeVar

from senior_contracts.errors import ErrorCode

from vehicle_manager.tasks.ports import MapReference, PoseReading

WaypointT = TypeVar("WaypointT")
StoredT = TypeVar("StoredT")
PoseT = TypeVar("PoseT", bound=PoseReading)
MapT = TypeVar("MapT", bound=MapReference)
_PoseCo = TypeVar("_PoseCo", bound=PoseReading, covariant=True)
_PoseContra = TypeVar("_PoseContra", bound=PoseReading, contravariant=True)
_MapContra = TypeVar("_MapContra", bound=MapReference, contravariant=True)


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


@dataclass(frozen=True)
class WaypointValuePorts(Generic[MapT, PoseT]):
    """Per-request-owner original local value and exception constructors."""

    map_reference: Callable[[str, int], MapT]
    pose: Callable[[float, float, float], PoseT]
    error: Callable[[ErrorCode, str], Exception]


class HomePoseReading(Protocol):
    @property
    def sampled_at(self) -> float: ...
    @property
    def map_id(self) -> str: ...
    @property
    def reference_revision(self) -> int: ...
    @property
    def anchor_id(self) -> str: ...
    @property
    def x(self) -> float: ...
    @property
    def y(self) -> float: ...
    @property
    def yaw(self) -> float: ...
    @property
    def anchor_x(self) -> float: ...
    @property
    def anchor_y(self) -> float: ...
    @property
    def anchor_yaw(self) -> float: ...
    @property
    def map_generation(self) -> str | None: ...


class HomeStateReading(Protocol):
    @property
    def localized(self) -> bool: ...
    @property
    def map_ref(self) -> MapReference | None: ...
    @property
    def map_generation(self) -> str | None: ...
    @property
    def localization_generation(self) -> str | None: ...


class NavigationStateReading(HomeStateReading, Protocol):
    @property
    def map_grid_id(self) -> str | None: ...
    @property
    def navigation_ready(self) -> bool: ...
    @property
    def manual_driving(self) -> bool: ...
    @property
    def safety_clear(self) -> bool: ...


class HomeStatePort(Protocol):
    async def read(self) -> HomeStateReading: ...


class NavigationStatePort(Protocol):
    async def read(self) -> NavigationStateReading: ...


class HomeWriter(Protocol[_PoseContra]):
    async def home_revision(self) -> int: ...

    async def save_home(
        self,
        *,
        waypoint_id: str | None,
        pose: _PoseContra,
        map_ref: MapReference,
        anchor_id: str,
        anchor_pose: _PoseContra,
        source_robot_id: str,
        status: str,
        adopted: bool,
        latitude: float | None,
        longitude: float | None,
        expected_revision: int | None = None,
    ) -> object: ...


class HomeWaypointReading(Protocol[_PoseCo]):
    @property
    def status(self) -> str: ...
    @property
    def map_ref(self) -> MapReference: ...
    @property
    def pose(self) -> _PoseCo: ...


class HomeReading(Protocol[_PoseCo]):
    @property
    def adopted(self) -> bool | None: ...
    @property
    def link_status(self) -> str | None: ...
    @property
    def waypoint(self) -> HomeWaypointReading[_PoseCo]: ...


class HomeReader(Protocol[_PoseCo]):
    async def home(self) -> HomeReading[_PoseCo] | None: ...


class WorkspaceTargetPort(Protocol[_MapContra, _PoseContra]):
    async def verify_free_target(
        self, ref: _MapContra, grid_id: str, pose: _PoseContra
    ) -> None: ...

    async def verify_navigation_target(
        self, ref: _MapContra, grid_id: str, pose: _PoseContra
    ) -> None: ...


class NavigationTargetPort(Protocol[_MapContra, _PoseContra]):
    async def start_poses(
        self,
        execution_id: str,
        map_ref: _MapContra,
        map_generation: str,
        poses: tuple[_PoseContra, ...],
    ) -> None: ...

    async def cancel_current(self) -> None: ...
