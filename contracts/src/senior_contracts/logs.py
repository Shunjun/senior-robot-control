"""The query shape used on both authenticated cloud/vehicle log sockets."""

from typing import Literal

from pydantic import BaseModel, ConfigDict, Field

LOG_WIRE_BYTES = 5 * 1024 * 1024
Level = Literal["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"]


class LogQuery(BaseModel):
    model_config = ConfigDict(frozen=True, extra="forbid", strict=True, allow_inf_nan=False)

    after: int = Field(default=0, ge=0, le=2**63 - 1)
    limit: int = Field(default=200, ge=1, le=200)
    levels: tuple[Level, ...] = Field(default=(), max_length=5)
    module: str | None = Field(default=None, max_length=128)
    request_id: str | None = Field(default=None, max_length=128)
    task_id: str | None = Field(default=None, max_length=128)
    since_utc: float | None = None
    until_utc: float | None = None


class LogRequest(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True)
    action: str = Field(pattern="^(query|download|follow)$")
    query: LogQuery
