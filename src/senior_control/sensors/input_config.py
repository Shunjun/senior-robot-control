"""Explicit optional source bindings; no topic discovery enables a motion input."""

from pydantic import BaseModel, ConfigDict, Field


class OptionalMotionInput(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)
    # GPS Odometry must retain one original physical acquisition family. A fused
    # body twist with unreported dynamic heading dependencies is not this contract.
    origin_family: str | None = Field(default=None, min_length=1, max_length=128)
    topic: str = Field(pattern=r"^/[A-Za-z_][A-Za-z0-9_/]*$")
    publisher: str = Field(pattern=r"^[A-Za-z_][A-Za-z0-9_]*$")
    namespace: str = Field(default="/", pattern=r"^/(?:[A-Za-z_][A-Za-z0-9_/]*)?$")
