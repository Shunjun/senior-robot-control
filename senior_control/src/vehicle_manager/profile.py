"""Original operator profile and unified-directory path rules."""

import json
from pathlib import Path
from typing import Literal

from pydantic import BaseModel, ConfigDict


class VehicleProfile(BaseModel):
    model_config = ConfigDict(extra="forbid", hide_input_in_errors=True)
    kind: Literal["real", "simulation"]
    credentials: Path
    ros_setup: Path | None = None
    vendor_ros_setup: Path = Path("/home/wheeltec/wheeltec_ros2/install/setup.bash")
    database: Path | None = None
    socket: Path | None = None
    cloud_url: str | None = None
    ros_domain: int | None = None
    motion_profile: Path | None = None
    safety_parameters: Path | None = None
    navigation_reference: Path | None = None
    navigation_parameters: Path | None = None
    video: bool = False
    control_dir: Path | None = None
    project_dir: Path | None = None
    container: str | None = None


def load_profile(path: Path, *, control_dir: Path | None = None) -> VehicleProfile:
    values = json.loads(path.read_bytes())
    if not isinstance(values, dict):
        raise ValueError("profile must be an object")
    if control_dir is None and path.name == "config.json":
        candidate = path.parent.absolute()
        if "credentials" not in values or values.get("credentials") == str(
            candidate / "device.json"
        ):
            control_dir = candidate
    if control_dir is not None:
        if not control_dir.is_absolute():
            raise ValueError("control directory must be absolute")
        paths = {
            "control_dir": control_dir,
            "credentials": control_dir / "device.json",
            "database": control_dir / "agent.db",
            "socket": control_dir / "agent.sock",
        }
        for name, value in paths.items():
            if name in values:
                if not isinstance(values[name], (str, Path)) or Path(values[name]) != value:
                    raise ValueError("data paths must stay in the control directory")
            values[name] = value
        if values.get("project_dir") and not values.get("ros_setup"):
            if not isinstance(values["project_dir"], (str, Path)):
                raise ValueError("project_dir must be a path")
            values["ros_setup"] = Path(values["project_dir"]) / "ros2/install/setup.bash"
    for name in (
        "motion_profile",
        "safety_parameters",
        "navigation_reference",
        "navigation_parameters",
    ):
        if values.get(name):
            selected = Path(values[name])
            values[name] = selected if selected.is_absolute() else path.parent.absolute() / selected
    if values.get("kind") == "simulation" and values.get("container") is not None:
        raise ValueError("container deployment is only supported for real profiles")
    # //Compatible: explicit legacy --profile files retain their individual paths.
    return VehicleProfile.model_validate(values)
