"""Read-only inventory. No launch, hardware access, or ROS daemon startup."""

import json
import os
import platform
import shutil
import subprocess
from glob import glob
from typing import Any


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
    return {
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
