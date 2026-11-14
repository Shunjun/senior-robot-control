"""Bounded product settings. Profile limits are policy, not physical proof."""

from dataclasses import dataclass
from typing import Literal

from pydantic import BaseModel, ConfigDict, Field, model_validator

Group = Literal["driving", "logging"]
Level = Literal["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"]


class FrozenModel(BaseModel):
    model_config = ConfigDict(frozen=True, extra="forbid", strict=True, allow_inf_nan=False)


class DrivingSettings(FrozenModel):
    low_linear_mps: float = Field(ge=0)
    high_linear_mps: float = Field(ge=0)
    low_angular_rps: float = Field(ge=0)
    high_angular_rps: float = Field(ge=0)
    reverse_linear_mps: float = Field(ge=0)

    @model_validator(mode="after")
    def ordered(self) -> "DrivingSettings":
        if self.low_linear_mps > self.high_linear_mps:
            raise ValueError("low linear speed must not exceed high linear speed")
        return self

    def reverse_limit(self, *, high: bool) -> float:
        return min(self.high_linear_mps if high else self.low_linear_mps, self.reverse_linear_mps)


class LoggingSettings(FrozenModel):
    level: Level = "INFO"
    retention_seconds: int = Field(default=604800, ge=3600, le=2592000)
    debug_enabled: bool = False
    debug_restore_seconds: int = Field(default=300, ge=60, le=3600)


@dataclass(frozen=True)
class SettingsProfile:
    driving_defaults: DrivingSettings
    driving_maximum: DrivingSettings
    logging_defaults: LoggingSettings = LoggingSettings()
    driving_minimum: DrivingSettings = DrivingSettings(
        low_linear_mps=0,
        high_linear_mps=0,
        low_angular_rps=0,
        high_angular_rps=0,
        reverse_linear_mps=0,
    )

    def __post_init__(self) -> None:
        if self.logging_defaults.debug_enabled:
            raise ValueError("defaults cannot enable temporary debug")
        self.validate_driving(self.driving_defaults)

    def validate_driving(self, value: DrivingSettings) -> None:
        for field, number in value.model_dump().items():
            if (
                not getattr(self.driving_minimum, field)
                <= number
                <= getattr(self.driving_maximum, field)
            ):
                raise ValueError(f"{field} outside vehicle profile range")


class SettingsSnapshot(FrozenModel):
    driving: DrivingSettings
    logging: LoggingSettings
    driving_version: int = Field(default=0, ge=0)
    logging_version: int = Field(default=0, ge=0)
    debug_original_level: Level | None = None
    debug_deadline_utc: float | None = None

    @model_validator(mode="after")
    def debug_consistent(self) -> "SettingsSnapshot":
        metadata = self.debug_original_level is not None and self.debug_deadline_utc is not None
        if self.logging.debug_enabled != metadata:
            raise ValueError("inconsistent debug metadata")
        if not self.logging.debug_enabled and (
            self.debug_original_level is not None or self.debug_deadline_utc is not None
        ):
            raise ValueError("unexpected debug metadata")
        return self

    @property
    def effective_log_level(self) -> Level:
        return "DEBUG" if self.logging.debug_enabled else self.logging.level

    def version(self, group: Group) -> int:
        return self.driving_version if group == "driving" else self.logging_version


@dataclass(frozen=True)
class SettingsRequest:
    request_id: str
    expected_version: int
    group: Group
    values: dict[str, object]
    restore_defaults: bool = False


class SettingsCapabilities(FrozenModel):
    driving_defaults: DrivingSettings
    driving_minimum: DrivingSettings
    driving_maximum: DrivingSettings
    logging_defaults: LoggingSettings
    retention_min_seconds: int = 3600
    retention_max_seconds: int = 2592000
    debug_restore_min_seconds: int = 60
    debug_restore_max_seconds: int = 3600


class SettingsResult(FrozenModel):
    request_id: str
    group: Group
    reason: str
    snapshot: SettingsSnapshot
    capabilities: SettingsCapabilities


def simulation_settings_profile() -> SettingsProfile:
    """Explicit simulation product policy, never measured hardware capability.

    Composition must opt in to this profile. The controller still constrains
    combined wheel requests and physical execution; real vehicles need a profile.
    """
    defaults = DrivingSettings(
        low_linear_mps=5 / 9,
        high_linear_mps=25 / 18,
        low_angular_rps=1,
        high_angular_rps=1,
        reverse_linear_mps=0.3,
    )
    maximum = DrivingSettings(
        low_linear_mps=25 / 18,
        high_linear_mps=25 / 18,
        low_angular_rps=2,
        high_angular_rps=2,
        reverse_linear_mps=0.3,
    )
    return SettingsProfile(defaults, maximum)
