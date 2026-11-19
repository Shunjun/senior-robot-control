"""Pinned real map/localization inputs; no generated identity or healthy defaults."""

import hashlib
import json
from pathlib import Path

from pydantic import BaseModel, ConfigDict, Field

from senior_control.mapping.workspace import WorkspaceGrid
from senior_control.models import MapRef, Pose2D


class NavigationReferenceConfig(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True, allow_inf_nan=False)
    grid_path: Path
    grid_sha256: str = Field(pattern=r"^[a-f0-9]{64}$")
    map_publisher: str = Field(pattern=r"^[A-Za-z_][A-Za-z0-9_]*$")
    pose_publisher: str = Field(pattern=r"^[A-Za-z_][A-Za-z0-9_]*$")
    source_namespace: str = Field(
        pattern=r"^/(?:[A-Za-z_][A-Za-z0-9_]*(?:/[A-Za-z_][A-Za-z0-9_]*)*)?$"
    )
    pose_topic: str = Field(pattern=r"^/[A-Za-z_][A-Za-z0-9_/]*$")
    max_age_s: float = Field(gt=0, le=5)
    max_position_variance: float = Field(gt=0)
    max_yaw_variance: float = Field(gt=0)
    anchor_id: str = Field(min_length=1)
    anchor_pose: Pose2D
    evidence_basis: str = Field(min_length=1)

    def grid(self) -> WorkspaceGrid:
        if not self.grid_path.is_absolute():
            raise ValueError("navigation grid_path must be absolute")
        with self.grid_path.open("rb") as stream:
            data = stream.read(32 * 1024 * 1024 + 1)
        if len(data) > 32 * 1024 * 1024 or hashlib.sha256(data).hexdigest() != self.grid_sha256:
            raise ValueError("navigation grid size or SHA-256 mismatch")
        try:
            values = json.loads(data)
            if not isinstance(values, dict):
                raise ValueError("grid must be an object")
            values["map_ref"] = MapRef(**values["map_ref"])
            values["origin"] = Pose2D(**values["origin"])
            values["cells"] = tuple(values["cells"])
            grid = WorkspaceGrid(**values)
        except (ValueError, TypeError, KeyError, OverflowError) as exc:
            raise ValueError("invalid pinned navigation WorkspaceGrid JSON") from exc
        if grid.display_only:
            raise ValueError("a display-only export cannot qualify real navigation")
        return grid


def load_navigation_reference(path: Path) -> NavigationReferenceConfig:
    if not path.is_absolute():
        raise ValueError("navigation_reference must be absolute")
    config = NavigationReferenceConfig.model_validate_json(path.read_bytes())
    config.grid()
    return config
