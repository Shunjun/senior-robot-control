"""Only the existing Home service's live owners, values and effects."""

import asyncio
from collections.abc import Awaitable, Callable
from dataclasses import dataclass
from typing import Generic, Protocol, TypeVar

from senior_contracts.errors import ErrorCode

from vehicle_manager.tasks.ports import MapReference, PoseReading
from vehicle_manager.waypoints.ports import BindingReading, BoundWaypointReading, HomeWriter


class SavedWaypointReading(Protocol):
    @property
    def id(self) -> str: ...
    @property
    def pose(self) -> PoseReading: ...
    @property
    def map_ref(self) -> MapReference: ...
    @property
    def anchor_id(self) -> str: ...
    @property
    def source_robot_id(self) -> str: ...
    @property
    def status(self) -> str: ...


class OperationReading(Protocol):
    @property
    def operation_id(self) -> str: ...
    @property
    def status(self) -> object: ...


WaypointT = TypeVar("WaypointT", bound=SavedWaypointReading)
PoseT = TypeVar("PoseT", bound=PoseReading)
ResultT = TypeVar("ResultT")
LinkT = TypeVar("LinkT")
GoalT = TypeVar("GoalT")
OperationT = TypeVar("OperationT", bound=OperationReading)
_WaypointCo = TypeVar("_WaypointCo", bound=SavedWaypointReading, covariant=True)
_PoseCo = TypeVar("_PoseCo", bound=PoseReading, covariant=True)
_PoseContra = TypeVar("_PoseContra", bound=PoseReading, contravariant=True)
_OperationCo = TypeVar("_OperationCo", bound=OperationReading, covariant=True)
_GoalContra = TypeVar("_GoalContra", contravariant=True)


class StartupReading(Protocol[_PoseCo]):
    @property
    def moved(self) -> bool: ...
    @property
    def continuous(self) -> bool: ...
    @property
    def trusted(self) -> bool: ...
    @property
    def relative_pose(self) -> _PoseCo | None: ...
    @property
    def latitude(self) -> float | None: ...
    @property
    def longitude(self) -> float | None: ...


class LocalizedReading(Protocol[_PoseCo]):
    @property
    def pose(self) -> _PoseCo: ...
    @property
    def map_ref(self) -> MapReference: ...
    @property
    def anchor_id(self) -> str: ...
    @property
    def anchor_pose(self) -> _PoseCo: ...
    @property
    def healthy(self) -> bool: ...
    @property
    def latitude(self) -> float | None: ...
    @property
    def longitude(self) -> float | None: ...
    @property
    def map_generation(self) -> str | None: ...


class ImportedHomeReading(Protocol[_PoseCo]):
    @property
    def kind(self) -> str: ...
    @property
    def source_robot_id(self) -> str: ...
    @property
    def waypoint_id(self) -> str: ...
    @property
    def pose(self) -> _PoseCo: ...
    @property
    def map_ref(self) -> MapReference: ...
    @property
    def anchor_id(self) -> str: ...


class StoredHomeReading(Protocol[_WaypointCo]):
    @property
    def waypoint(self) -> _WaypointCo: ...
    @property
    def anchor_pose(self) -> PoseReading: ...
    @property
    def adopted(self) -> bool | None: ...
    @property
    def link_status(self) -> str | None: ...


class HomeStore(HomeWriter[_PoseContra], Protocol[_WaypointCo, _PoseContra]):
    async def prepare(self) -> None: ...
    async def home(self) -> StoredHomeReading[_WaypointCo] | None: ...
    async def waypoint(self, waypoint_id: str) -> StoredHomeReading[_WaypointCo] | None: ...
    async def sync_poses(self, bindings: tuple[BindingReading, ...]) -> None: ...
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
    ) -> _WaypointCo: ...
    async def save_waypoint(
        self,
        *,
        waypoint_id: str,
        pose: _PoseContra,
        map_ref: MapReference,
        anchor_id: str,
        anchor_pose: _PoseContra,
        source_robot_id: str,
        status: str,
        latitude: float | None,
        longitude: float | None,
        create_only: bool = False,
    ) -> _WaypointCo: ...


class HomeAnchoredReading(BoundWaypointReading, Protocol):
    @property
    def review_required(self) -> bool: ...
    @property
    def anchor_id(self) -> str: ...


class HomeBindingReading(BindingReading, Protocol):
    @property
    def waypoint(self) -> HomeAnchoredReading: ...


class HomeSnapshotReading(Protocol):
    @property
    def waypoint_bindings(self) -> tuple[HomeBindingReading, ...]: ...


class HomeReferenceReading(Protocol):
    @property
    def map_ref(self) -> MapReference: ...
    @property
    def map_generation(self) -> str: ...
    @property
    def review_required(self) -> bool: ...


class HomeNavigation(Protocol[_OperationCo]):
    async def cancel(self, execution_id: str) -> _OperationCo: ...


class QualifiedNavigation(Protocol[_GoalContra]):
    async def navigate_qualified(self, goal: _GoalContra) -> str: ...


class HomeCoordination(Protocol[OperationT]):
    """The original mutable coordination owner returned by this DB and robot id."""

    @property
    def lock(self) -> asyncio.Lock: ...

    connected: bool
    disconnect_generation: int
    cancel_required: bool
    cancel_attempt_generation: int
    cancel_owner_token: str | None
    cancel_owner: Callable[[str], Awaitable[OperationT]] | None


@dataclass(frozen=True)
class HomeServicePorts(Generic[WaypointT, PoseT, ResultT, LinkT, GoalT]):
    """Live map effects and concrete constructors, scoped to this service instance."""

    reference: HomeReferenceReading
    snapshot: Callable[[], HomeSnapshotReading]
    place: Callable[..., None]
    apply_anchor_update: Callable[..., object]
    bind_waypoint: Callable[[WaypointT, PoseT], None]
    pose: Callable[[float, float, float], PoseT]
    result: Callable[..., ResultT]
    link: Callable[..., LinkT]
    goal: Callable[..., GoalT]
    error: type[Exception]
    error_code: Callable[[Exception], ErrorCode]
    succeeded_status: object
