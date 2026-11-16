#!/usr/bin/env python3
"""Prepare reviewed Nav2 controller/MPPI/AMCL packages without installing or upgrading ROS."""

import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys
import urllib.request
from pathlib import Path

BUNDLE = Path(__file__).resolve().parent / "nav2_overlay"


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def prepare(source: Path, destination: Path, *, download: bool = False) -> Path:
    lock = json.loads((BUNDLE / "manifest.json").read_text())
    patches = (
        ("navigation-monotonic.patch", "patch_sha256"),
        ("system-monotonic.patch", "system_patch_sha256"),
    )
    for name, checksum in patches:
        if digest(BUNDLE / name) != lock[checksum]:
            raise ValueError(f"overlay patch checksum mismatch: {name}")
    for name, expected in lock["files"].items():
        path = source / name
        if download and not path.exists():
            url = (
                "https://raw.githubusercontent.com/ros-navigation/navigation2/"
                + lock["commit"]
                + "/"
                + name
            )
            with urllib.request.urlopen(url, timeout=40) as response:
                data = response.read()
            if hashlib.sha256(data).hexdigest() != expected:
                raise ValueError(f"download checksum mismatch: {name}")
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(data)
        if not path.is_file() or digest(path) != expected:
            raise ValueError(f"Nav2 {lock['version']} source checksum mismatch: {name}")
    # Never overwrite a caller's source tree or silently patch another release.
    if destination.exists():
        raise ValueError(f"destination already exists: {destination}")
    destination.mkdir(parents=True)
    for name in lock["files"]:
        target = destination / name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source / name, target)
    for name, _ in patches:
        subprocess.run(
            ["patch", "--batch", "--fuzz=0", "-p1", "-i", str(BUNDLE / name)],
            cwd=destination,
            check=True,
        )
    for name, expected in lock["patched_files"].items():
        if digest(destination / name) != expected:
            raise ValueError(f"patched checksum mismatch: {name}")
    marker = {key: lock[key] for key in ("commit", "version", "contract")}
    package = destination / "nav2_controller"
    for name in lock["packages"]:
        (destination / name / "navigation-contract.json").write_text(
            json.dumps(marker, indent=2) + "\n"
        )
    print(f"Prepared Nav2 {lock['version']} at {lock['commit']}: {package}")
    return package


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True, help="verified source cache")
    parser.add_argument("--destination", type=Path, required=True, help="new overlay src directory")
    parser.add_argument(
        "--download", action="store_true", help="download only locked package files"
    )
    parser.add_argument(
        "--build", action="store_true", help="build the three locked packages in sourced Humble"
    )
    args = parser.parse_args()
    if args.build:
        if sys.platform != "linux" or os.environ.get("ROS_DISTRO") != "humble":
            raise RuntimeError("building requires Linux with sourced ROS Humble")
        sys.path.insert(0, str(BUNDLE.parents[1] / "ros2/src/senior_robot"))
        from senior_robot.navigation_overlay import require_nav2_abi

        require_nav2_abi()
    prepare(args.source, args.destination, download=args.download)
    if args.build:
        workspace = args.destination.resolve().parent
        subprocess.run(
            [
                "colcon",
                "--log-base",
                str(workspace / "log"),
                "build",
                "--base-paths",
                str(args.destination.resolve()),
                "--build-base",
                str(workspace / "build"),
                "--install-base",
                str(workspace / "install"),
                "--packages-select",
                "nav2_controller",
                "nav2_mppi_controller",
                "nav2_amcl",
                "--allow-overriding",
                "nav2_controller",
                "nav2_mppi_controller",
                "nav2_amcl",
                "--cmake-args",
                "-DBUILD_TESTING=OFF",
            ],
            check=True,
        )


if __name__ == "__main__":
    main()
