"""Bounded log socket request shared by cloud and Agent."""

from pydantic import BaseModel, ConfigDict, Field

from senior_control.vehicle_logs.models import LogQuery

LOG_WIRE_BYTES = 5 * 1024 * 1024


class LogRequest(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True)
    action: str = Field(pattern="^(query|download|follow)$")
    query: LogQuery
