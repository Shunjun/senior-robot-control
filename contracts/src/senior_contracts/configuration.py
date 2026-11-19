"""Immutable driving configuration supplied across process boundaries."""

from pydantic import BaseModel, ConfigDict, Field, model_validator


class FrozenModel(BaseModel):
    model_config = ConfigDict(frozen=True, extra="forbid", strict=True, allow_inf_nan=False)


class DrivingSettings(FrozenModel):
    low_linear_mps: float = Field(ge=0)
    high_linear_mps: float = Field(ge=0)
    low_angular_rps: float = Field(ge=0)
    high_angular_rps: float = Field(ge=0)
    reverse_linear_mps: float = Field(ge=0)
    # //Compatible: pre-degraded-setting callers use the new product default.
    degraded_linear_mps: float = Field(default=0.3, ge=0)

    @model_validator(mode="after")
    def ordered(self) -> "DrivingSettings":
        if self.low_linear_mps > self.high_linear_mps:
            raise ValueError("low linear speed must not exceed high linear speed")
        return self

    def reverse_limit(self, *, high: bool) -> float:
        return min(self.high_linear_mps if high else self.low_linear_mps, self.reverse_linear_mps)
