"""Sequential execution of poses from one qualified map reference."""

from typing import Annotated, Literal

from pydantic import BaseModel, ConfigDict, Field, model_validator

from senior_contracts.errors import DomainError, ErrorCode
from senior_contracts.values import MapRef

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
    task_epoch: int = Field(ge=1)
    effect_id: str = Field(min_length=1, max_length=128)
    action_ids: list[str] = Field(default_factory=list, max_length=MAX_ROUTE_WAYPOINTS)
    pose_ref: str | None = None
    map_generation: str = Field(min_length=1, max_length=128)
    poses: list[Annotated[list[float], Field(min_length=3, max_length=3)]] = Field(
        min_length=1, max_length=MAX_ROUTE_WAYPOINTS
    )

    @model_validator(mode="after")
    def task_identity(self) -> "RoutePlan":
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
    # //Compatible: cached progress can lack task identity; it never admits a route.
    task_epoch: int | None = None
    effect_id: str | None = None
    completed_action_ids: list[str] = Field(default_factory=list)
    sources_fenced_through_epoch: int | None = None
    controller_output_exited: bool = False
