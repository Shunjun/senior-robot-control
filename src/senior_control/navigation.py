"""Sequential execution of poses from one qualified map reference."""

from collections.abc import Awaitable, Callable, Sequence
from typing import Annotated, Literal

from pydantic import BaseModel, ConfigDict, Field, model_validator

from senior_control.errors import DomainError, ErrorCode
from senior_control.models import MapRef, Pose2D

MAX_ROUTE_WAYPOINTS = 64
RouteStatus = Literal["starting", "running", "succeeded", "failed", "cancel_pending", "canceled"]


class RouteRejected(DomainError):
    """The route was rejected before the receiver reserved it."""

    def __init__(self) -> None:
        super().__init__(ErrorCode.UNAVAILABLE, "qualified route was not admitted")


class RoutePlan(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True, allow_inf_nan=False)

    execution_id: str = Field(min_length=1, max_length=128)
    task_id: str | None = Field(default=None, min_length=1, max_length=128)
    map_id: str = Field(min_length=1, max_length=128)
    reference_revision: int = Field(ge=0)
    # //Compatible: version 1 routes retain their original connection namespace.
    connection_epoch: int | None = Field(default=None, ge=1)
    task_epoch: int | None = Field(default=None, ge=1)
    effect_id: str | None = Field(default=None, min_length=1, max_length=128)
    action_ids: list[str] = Field(default_factory=list, max_length=MAX_ROUTE_WAYPOINTS)
    pose_ref: str | None = None
    map_generation: str = Field(min_length=1, max_length=128)
    poses: list[Annotated[list[float], Field(min_length=3, max_length=3)]] = Field(
        min_length=1, max_length=MAX_ROUTE_WAYPOINTS
    )

    @model_validator(mode="after")
    def task_identity(self) -> "RoutePlan":
        if self.task_epoch is not None:
            if not self.effect_id or self.connection_epoch is not None:
                raise ValueError("task execution must have its own effect/epoch namespace")
            if len(self.action_ids) != len(self.poses) or len(set(self.action_ids)) != len(
                self.action_ids
            ):
                raise ValueError("each route action needs a stable unique identity")
        return self

    @property
    def map_ref(self) -> MapRef:
        return MapRef(self.map_id, self.reference_revision)


class RouteProgress(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True)

    execution_id: str = Field(min_length=1, max_length=128)
    task_id: str | None = Field(default=None, min_length=1, max_length=128)
    map_generation: str = Field(min_length=1, max_length=128)
    status: RouteStatus
    active_waypoint_index: int | None = Field(default=None, ge=0, lt=MAX_ROUTE_WAYPOINTS)
    task_epoch: int | None = None
    effect_id: str | None = None
    completed_action_ids: list[str] = Field(default_factory=list)
    sources_fenced_through_epoch: int | None = None
    controller_output_exited: bool = False


class QualifiedRoute:
    def __init__(
        self,
        navigate: Callable[[Pose2D], Awaitable[bool]],
        qualified: Callable[[MapRef, str], bool],
    ) -> None:
        self._navigate = navigate
        self._qualified = qualified

    async def run(self, ref: MapRef, generation: str, poses: Sequence[Pose2D]) -> bool:
        route = tuple(poses)
        if not route or not generation or not self._qualified(ref, generation):
            return False
        for pose in route:
            if not await self._navigate(pose) or not self._qualified(ref, generation):
                return False
        return True
