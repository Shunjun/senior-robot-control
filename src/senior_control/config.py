"""Explicit configuration loading; importing this module reads no environment."""

import os
from collections.abc import Mapping
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Literal

import yaml
from pydantic import BaseModel, ConfigDict, Field, SecretStr, ValidationError, model_validator


class DeviceProfile(BaseModel):
    model_config = ConfigDict(extra="forbid", allow_inf_nan=False, hide_input_in_errors=True)
    mode: Literal["simulation", "hardware"] = "simulation"
    footprint: list[tuple[float, float]] = Field(default_factory=list)
    topics: dict[str, str] = Field(default_factory=dict)
    frames: dict[str, str] = Field(default_factory=dict)
    sensor_coverage: dict[str, float] = Field(default_factory=dict)


class AppSettings(BaseModel):
    model_config = ConfigDict(extra="forbid", allow_inf_nan=False, hide_input_in_errors=True)
    secret: SecretStr
    heartbeat_seconds: float = Field(default=0.2, gt=0)
    disconnect_seconds: float = Field(default=1.0, gt=0)
    command_ttl_seconds: float = Field(default=0.3, gt=0)
    perception_stale_seconds: float = Field(default=0.5, gt=0)
    device: DeviceProfile = Field(default_factory=DeviceProfile)

    @model_validator(mode="after")
    def check_timing(self) -> "AppSettings":
        if self.heartbeat_seconds >= self.disconnect_seconds:
            raise ValueError("heartbeat must be shorter than disconnect threshold")
        return self


@dataclass(frozen=True)
class SettingsSource:
    yaml_path: Path | None = None
    environment: Mapping[str, str] | None = None
    cli: Mapping[str, Any] = field(default_factory=dict)


def load_settings(source: SettingsSource) -> AppSettings:
    """Load explicit YAML, environment and CLI settings in ascending priority."""
    values: dict[str, Any] = {}
    if source.yaml_path is not None:
        with source.yaml_path.open(encoding="utf-8") as stream:
            loaded = yaml.safe_load(stream)
        if not isinstance(loaded, dict):
            raise ValueError("configuration must be a mapping")
        values.update(loaded)
    env = source.environment if source.environment is not None else os.environ
    for name in AppSettings.model_fields:
        key = "SENIOR_" + name.upper()
        if key in env:
            values[name] = env[key]
    values.update(source.cli)
    try:
        return AppSettings.model_validate(values)
    except ValidationError as exc:
        raise ValueError("invalid configuration") from exc
