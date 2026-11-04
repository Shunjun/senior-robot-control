"""Read-only inventory. No launch, hardware access, or ROS daemon startup."""

import json
import os
import platform
import shutil
import subprocess
import xml.etree.ElementTree as ET
from glob import glob
from pathlib import Path
from typing import Any

PACKAGES = (
    "ros_gz_sim",
    "ros_gz_bridge",
    "gz_ros2_control",
    "controller_manager",
    "diff_drive_controller",
    "joint_state_broadcaster",
    "robot_state_publisher",
    "xacro",
)


def package_inventory() -> dict[str, dict[str, Any]]:
    """Inspect sourced ament prefixes without importing ROS or starting a daemon."""
    prefixes = [Path(p) for p in os.getenv("AMENT_PREFIX_PATH", "").split(os.pathsep) if p]
    result: dict[str, dict[str, Any]] = {}
    for name in PACKAGES:
        result[name] = {"available": False, "version": None, "prefix": None}
        for prefix in prefixes:
            manifest = prefix / "share" / name / "package.xml"
            if not manifest.is_file():
                continue
            try:
                version = ET.parse(manifest).getroot().findtext("version")
            except (ET.ParseError, OSError):
                version = None
            result[name] = {"available": True, "version": version, "prefix": str(prefix)}
            break
    return result


def detect_environment() -> dict[str, Any]:
    ros2 = shutil.which("ros2")
    topics = None
    if ros2:
        try:
            result = subprocess.run(
                [ros2, "topic", "list", "--no-daemon"],
                capture_output=True,
                text=True,
                check=True,
                timeout=5,
            )
            topics = sorted(result.stdout.splitlines())
        except (OSError, subprocess.SubprocessError):
            pass
    packages = package_inventory()
    missing = [name for name, details in packages.items() if not details["available"]]
    return {
        "packages": packages,
        "matrix": {
            "target": "Ubuntu 22.04 / ROS Humble / Gazebo Fortress",
            "missing_ros_packages": missing,
            "fortress_version": None,
            "fortress_version_status": "not verified; inspect installed ignition-gazebo6 version",
            "runtime_ready": False if missing or os.getenv("ROS_DISTRO") != "humble" else None,
        },
        "platform": platform.platform(),
        "ros": {
            "available": ros2 is not None,
            "executable": ros2,
            "distro": os.getenv("ROS_DISTRO"),
        },
        "gazebo": {"ign": shutil.which("ign"), "gz": shutil.which("gz")},
        "devices": {"serial": sorted(glob("/dev/ttyUSB*") + glob("/dev/ttyACM*"))},
        "topics": topics,
    }


if __name__ == "__main__":
    print(json.dumps(detect_environment(), indent=2))
