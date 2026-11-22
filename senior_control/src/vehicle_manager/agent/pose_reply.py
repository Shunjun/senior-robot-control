"""Strict local pose reply validation for management Home capture."""

from typing import Literal

from pydantic import BaseModel, ConfigDict


class PoseReply(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True, allow_inf_nan=False)
    v: Literal[1]
    ok: Literal[True]
    sampled_at: float
    healthy: Literal[True]
    map_id: str
    reference_revision: int
    anchor_id: str
    x: float
    y: float
    yaw: float
    anchor_x: float
    anchor_y: float
    anchor_yaw: float
    # //Compatible: legacy pose IPC remains readable but cannot authorize Home.
    map_generation: str | None = None
