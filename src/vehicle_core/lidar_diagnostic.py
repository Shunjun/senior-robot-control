"""Explicit, process-wide selection for the temporary lidar diagnostic branch."""

import os
from collections.abc import Mapping
from pathlib import Path
from typing import Any

ENVIRONMENT_KEY = "SENIOR_TEMPORARY_LIDAR_BYPASS"
PARAMETER = "temporary_lidar_bypass"


def strict_bool(value: Any) -> bool:
    """Accept only a real boolean from YAML or a declared ROS parameter."""
    if type(value) is not bool:
        raise ValueError(f"{PARAMETER} must be a boolean")
    return value


def selected(
    configured: bool | None = None, *, environment: Mapping[str, str] | None = None
) -> bool:
    """Resolve the explicit selection and reject conflicting process settings."""
    environment = os.environ if environment is None else environment
    if configured is not None:
        strict_bool(configured)
    raw = environment.get(ENVIRONMENT_KEY)
    if raw is None:
        return False if configured is None else configured
    if raw not in {"true", "false"}:
        raise ValueError(f"{ENVIRONMENT_KEY} must be true or false")
    enabled = raw == "true"
    if configured is not None and configured != enabled:
        raise ValueError(f"{PARAMETER} conflicts with {ENVIRONMENT_KEY}")
    return enabled


def from_parameters(parameters: Mapping[str, Any]) -> bool:
    """Read the safety owner's selection, with the normal default disabled."""
    return selected(strict_bool(parameters.get(PARAMETER, False)))


def from_file(path: Path, *, environment: Mapping[str, str] | None = None) -> bool:
    """Read only the mode before any hardware or learning resource is created."""
    import yaml

    value = yaml.safe_load(path.read_bytes())
    if not isinstance(value, dict):
        raise ValueError("safety_parameters must contain per-node ros__parameters")
    entry = value.get("safety_gate", {})
    parameters = entry.get("ros__parameters", {}) if isinstance(entry, dict) else None
    if not isinstance(parameters, dict):
        raise ValueError("safety_gate.ros__parameters must be a mapping")
    return selected(strict_bool(parameters.get(PARAMETER, False)), environment=environment)


def require_lidar(operation: str) -> None:
    """Reject independent lidar consumers before they allocate runtime resources."""
    if selected():
        raise ValueError(f"{operation} unavailable: {PARAMETER}=true (lidar disabled)")
