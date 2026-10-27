"""Explicit configuration loading; importing this module reads no environment."""

import os
from collections.abc import Mapping
from dataclasses import dataclass, field
from math import pi
from pathlib import Path
from typing import Any, Literal

import yaml
from pydantic import BaseModel, ConfigDict, Field, SecretStr, ValidationError, model_validator


class ConfigModel(BaseModel):
    model_config = ConfigDict(extra="forbid", allow_inf_nan=False, hide_input_in_errors=True)


class DeviceProfile(ConfigModel):
    mode: Literal["simulation", "hardware"] = "simulation"
    footprint: list[tuple[float, float]] = Field(default_factory=list)
    topics: dict[str, str] = Field(default_factory=dict)
    frames: dict[str, str] = Field(default_factory=dict)
    sensor_coverage: dict[str, float] = Field(default_factory=dict)

    @model_validator(mode="after")
    def hardware_requirements(self) -> "DeviceProfile":
        if any(not 0 < span <= 2 * pi for span in self.sensor_coverage.values()):
            raise ValueError("sensor coverage spans must be in (0, 2π]")
        if not self.sensor_coverage.keys() <= self.topics.keys():
            raise ValueError("sensor coverage must reference configured topics")
        if self.mode == "hardware":
            if "scan" not in self.sensor_coverage:
                raise ValueError("hardware requires scan coverage")
            points = self.footprint
            area = (
                abs(
                    sum(
                        points[i][0] * points[(i + 1) % len(points)][1]
                        - points[(i + 1) % len(points)][0] * points[i][1]
                        for i in range(len(points))
                    )
                )
                if points
                else 0
            )
            if len(points) < 3 or area == 0 or not self.frames or not self.topics:
                raise ValueError("hardware requires footprint, frames and topics")
            if any(
                not k.strip() or not v.strip()
                for mapping in (self.frames, self.topics)
                for k, v in mapping.items()
            ):
                raise ValueError("hardware frame and topic names must be nonempty")
        return self


class CommonSettings(ConfigModel):
    secret: SecretStr

    @model_validator(mode="after")
    def nonempty_secret(self) -> "CommonSettings":
        if not self.secret.get_secret_value().strip():
            raise ValueError("secret is required")
        return self


class CloudSettings(CommonSettings):
    role: Literal["cloud"] = "cloud"


class AgentSettings(CommonSettings):
    role: Literal["agent"] = "agent"
    heartbeat_seconds: float = Field(default=0.2, gt=0)
    disconnect_seconds: float = Field(default=1.0, gt=0)
    command_ttl_seconds: float = Field(default=0.3, gt=0)
    perception_stale_seconds: float = Field(default=0.5, gt=0)
    device: DeviceProfile = Field(default_factory=DeviceProfile)

    @model_validator(mode="after")
    def check_timing(self) -> "AgentSettings":
        if self.heartbeat_seconds >= self.disconnect_seconds:
            raise ValueError("heartbeat must be shorter than disconnect threshold")
        return self


AppSettings = AgentSettings | CloudSettings


@dataclass(frozen=True)
class SettingsSource:
    yaml_path: Path | None = None
    environment: Mapping[str, str] | None = None
    secrets_dir: Path | None = None
    cli: Mapping[str, Any] = field(default_factory=dict)


def _merge(base: dict[str, Any], incoming: Mapping[str, Any]) -> None:
    for key, value in incoming.items():
        if isinstance(value, Mapping) and isinstance(base.get(key), dict):
            _merge(base[key], value)
        else:
            base[key] = value


def load_settings(source: SettingsSource) -> AppSettings:
    """Load YAML, mounted secrets, environment, then CLI; never load cwd .env."""
    values: dict[str, Any] = {}
    if source.yaml_path is not None:
        try:
            with source.yaml_path.open(encoding="utf-8") as stream:
                loaded = yaml.safe_load(stream)
        except yaml.YAMLError:
            raise ValueError("invalid YAML configuration") from None
        if not isinstance(loaded, dict):
            raise ValueError("configuration must be a mapping")
        values.update(loaded)
    if source.secrets_dir is not None:
        secret = source.secrets_dir / "SENIOR_SECRET"
        if secret.is_file():
            values["secret"] = secret.read_text(encoding="utf-8").strip()
    env = source.environment if source.environment is not None else os.environ
    for key, value in env.items():
        if not key.startswith("SENIOR_"):
            continue
        parts = key[7:].lower().split("__")
        nested: dict[str, Any] = {parts[-1]: value}
        for part in reversed(parts[:-1]):
            nested = {part: nested}
        _merge(values, nested)
    _merge(values, source.cli)
    model = CloudSettings if values.get("role") == "cloud" else AgentSettings
    try:
        return model.model_validate(values)
    except ValidationError as exc:
        raise ValueError("invalid configuration") from exc
