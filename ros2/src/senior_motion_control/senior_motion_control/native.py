"""Resolve installed native control executables without importing ROS at module load."""

from pathlib import Path


def native_executable(name: str) -> Path:
    from ament_index_python.packages import get_package_prefix

    if name not in {"motion_output_guard", "final_command_guard"}:
        raise ValueError("unknown native control executable")
    return Path(get_package_prefix("senior_motion_control")) / "lib/senior_motion_control" / name
