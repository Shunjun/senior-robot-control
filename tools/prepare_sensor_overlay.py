#!/usr/bin/env python3
"""Stage locked selected vendor packages in a new overlay; never modify vendor materials."""

import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

BUNDLE = Path(__file__).resolve().parent / "sensor_overlay"


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def prepare(source: Path, destination: Path, *, optional: tuple[str, ...] = ()) -> list[Path]:
    lock = json.loads((BUNDLE / "manifest.json").read_text())
    if any(name not in lock["optional_packages"] for name in optional):
        raise ValueError("unknown optional sensor package")
    selected = lock["default_packages"] + [lock["optional_packages"][name] for name in optional]
    patches = ["driver-monotonic.patch"] + [f"optional-{name}.patch" for name in optional]
    for name in patches:
        if digest(BUNDLE / name) != lock["patches"][name]:
            raise ValueError(f"sensor patch checksum mismatch: {name}")
    for package in selected:
        for name, expected in lock["packages"][package].items():
            path = source / name
            if path.is_symlink() or not path.is_file() or digest(path) != expected:
                raise ValueError(f"vendor source mismatch: {name}")
    if destination.exists():
        raise ValueError(f"destination already exists: {destination}")
    destination.mkdir(parents=True)
    for package in selected:
        for name in lock["packages"][package]:
            target = destination / name
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(source / name, target)
    for name in patches:
        subprocess.run(
            ["patch", "--batch", "--fuzz=0", "-p1", "-i", str(BUNDLE / name)],
            cwd=destination,
            check=True,
        )
    for name, hashes in lock["files"].items():
        if not any(name.startswith(package + "/") for package in selected):
            continue
        if digest(destination / name) != hashes["after"]:
            raise ValueError(f"patched vendor source mismatch: {name}")
    packages = [destination / name for name in selected]
    print(f"Prepared {len(packages)} selected vendor packages; no build or deployment performed")
    return packages


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True, help="read-only vendor src root")
    parser.add_argument("--destination", type=Path, required=True, help="new overlay src directory")
    parser.add_argument(
        "--build", action="store_true", help="build in an already sourced Humble environment"
    )
    parser.add_argument("--optional", action="append", choices=("gps", "ahrs"), default=[])
    args = parser.parse_args()
    if args.build and (sys.platform != "linux" or os.environ.get("ROS_DISTRO") != "humble"):
        raise RuntimeError("building requires Linux with sourced ROS Humble")
    packages = prepare(args.source, args.destination, optional=tuple(dict.fromkeys(args.optional)))
    if args.build:
        from xml.etree import ElementTree

        names = [ElementTree.parse(p / "package.xml").findtext("name") for p in packages]
        workspace = args.destination.resolve().parent
        subprocess.run(
            [
                "colcon",
                "--log-base",
                str(workspace / "log"),
                "build",
                "--base-paths",
                *(str(p) for p in packages),
                "--build-base",
                str(workspace / "build"),
                "--install-base",
                str(workspace / "install"),
                "--packages-select",
                *names,
                "--allow-overriding",
                *names,
                "--cmake-args",
                "-DBUILD_TESTING=OFF",
            ],
            check=True,
        )


if __name__ == "__main__":
    main()
