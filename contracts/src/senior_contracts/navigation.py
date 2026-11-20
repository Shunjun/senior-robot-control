"""Route progress fields actually carried by cloud/vehicle snapshot messages."""

from typing import Literal

from pydantic import BaseModel, ConfigDict, Field

MAX_ROUTE_WAYPOINTS = 64
RouteStatus = Literal["starting", "running", "succeeded", "failed", "cancel_pending", "canceled"]


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
