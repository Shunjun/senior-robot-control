#!/usr/bin/env python3
"""Stage exact official sources and minimal patches; optionally build, never deploy."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import platform
import shutil
import subprocess
import urllib.request
from pathlib import Path

BUNDLE = Path(__file__).resolve().parent / "simulation_overlay"
BASELINES = {
    "libignition-gazebo6-dev": "6.17.0",
    "libignition-sensors6-dev": "6.8.1",
    "ros-humble-ros-gz-bridge": "0.244.20",
    "ros-humble-gz-ros2-control": "0.7.21",
    "ros-humble-diff-drive-controller": "2.50.0",
}


def sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def relative(value: str) -> Path:
    path = Path(value)
    if path.is_absolute() or ".." in path.parts:
        raise ValueError(f"unsafe source path: {value}")
    return path


def run(command: list[str], *, cwd: Path, env: dict[str, str] | None = None) -> None:
    subprocess.run(command, cwd=cwd, env=env, check=True)


def check_dependencies() -> dict[str, str]:
    if platform.system() != "Linux" or os.environ.get("ROS_DISTRO") != "humble":
        raise RuntimeError("build requires Linux with sourced ROS 2 Humble")
    if os.environ.get("GZ_VERSION", "fortress") not in ("", "fortress"):
        raise RuntimeError("this fixed overlay supports Fortress only")
    versions = {}
    for package, baseline in BASELINES.items():
        installed = subprocess.check_output(
            ["dpkg-query", "-W", "-f=${Version}", package], text=True
        ).strip()
        upstream = installed.split(":", 1)[-1].split("-", 1)[0]
        if upstream != baseline:
            raise RuntimeError(
                f"{package}: installed {installed}, expected {baseline}; "
                "no automatic upgrade or patch"
            )
        versions[package] = installed
    return versions


def audit_shared_epoch_links(prefix: Path, core_build: Path, env: dict[str, str]) -> dict[str, str]:
    """Check built DT_NEEDED resolution; this does not load a running Gazebo world."""
    expected = Path((core_build / "epoch-library-path.txt").read_text().strip()).resolve()
    library_env = dict(env)
    library_env["LD_LIBRARY_PATH"] = (
        str(prefix / "lib")
        + os.pathsep
        + str(expected.parent)
        + os.pathsep
        + env.get("LD_LIBRARY_PATH", "")
    )
    result = {}
    for name in (
        "libignition-gazebo6.so.6",
        "libignition-gazebo-sensors-system.so",
        "libignition-gazebo-imu-system.so",
        "libignition-gazebo-odometry-publisher-system.so",
    ):
        path = prefix / "lib" / name
        output = subprocess.check_output(["ldd", str(path)], env=library_env, text=True)
        matches = [
            line.split("=>", 1)[1].strip().split()[0]
            for line in output.splitlines()
            if "libsimulation_timing.so.0 =>" in line
        ]
        if len(matches) != 1 or Path(matches[0]).resolve() != expected:
            raise RuntimeError(
                f"{name}: reset/producer epoch library does not resolve to {expected}"
            )
        result[name] = str(expected)
    symbols = subprocess.check_output(["nm", "-D", "--defined-only", str(expected)], text=True)
    if "reset_simulation_epoch" not in symbols or "simulation_epoch" not in symbols:
        raise RuntimeError("Shared epoch library is missing its public reset/producer API")
    return result


def build(destination: Path) -> None:
    versions = check_dependencies()
    prefix = destination / "install"
    env = dict(os.environ)
    for variable in ("CMAKE_PREFIX_PATH", "AMENT_PREFIX_PATH"):
        env[variable] = str(prefix) + os.pathsep + env.get(variable, "")
    # senior_motion_msgs from this exact workspace must already be built/sourced.
    run(
        [
            "cmake",
            "-S",
            str(destination / "gz-sensors"),
            "-B",
            str(destination / "build-sensors"),
            f"-DCMAKE_INSTALL_PREFIX={prefix}",
            "-DBUILD_TESTING=OFF",
            "-DBUILD_DOCS=OFF",
        ],
        cwd=destination,
        env=env,
    )
    run(["cmake", "--build", str(destination / "build-sensors")], cwd=destination, env=env)
    run(["cmake", "--install", str(destination / "build-sensors")], cwd=destination, env=env)
    run(
        [
            "cmake",
            "-S",
            str(destination / "core-build"),
            "-B",
            str(destination / "build-core"),
            f"-DGZ_SIM_SOURCE={destination / 'gz-sim'}",
            f"-DCMAKE_INSTALL_PREFIX={prefix}",
        ],
        cwd=destination,
        env=env,
    )
    run(["cmake", "--build", str(destination / "build-core")], cwd=destination, env=env)
    run(["cmake", "--install", str(destination / "build-core")], cwd=destination, env=env)
    run(
        [
            "cmake",
            "-S",
            str(destination / "system-build"),
            "-B",
            str(destination / "build-systems"),
            f"-DGZ_SIM_SOURCE={destination / 'gz-sim'}",
            f"-DCMAKE_INSTALL_PREFIX={prefix}",
        ],
        cwd=destination,
        env=env,
    )
    run(["cmake", "--build", str(destination / "build-systems")], cwd=destination, env=env)
    run(["cmake", "--install", str(destination / "build-systems")], cwd=destination, env=env)
    run(
        [
            "colcon",
            "build",
            "--base-paths",
            str(destination / "ros_gz" / "ros_gz_bridge"),
            str(destination / "gz_ros2_control" / "gz_ros2_control"),
            str(destination / "ros2_controllers" / "diff_drive_controller"),
            "--merge-install",
            "--install-base",
            str(prefix),
            "--cmake-args",
            "-DBUILD_TESTING=OFF",
        ],
        cwd=destination,
        env=env,
    )
    epoch_links = audit_shared_epoch_links(prefix, destination / "build-core", env)
    (destination / "build-evidence.json").write_text(
        json.dumps(
            {
                "dpkg": versions,
                "built": True,
                "shared_epoch_link_resolution": epoch_links,
                "plugin_abi_runtime_verified": False,
                "dds_runtime_verified": False,
            },
            indent=2,
        )
        + "\n"
    )


def prepare(cache: Path, destination: Path, fetch: bool) -> None:
    manifest = json.loads((BUNDLE / "manifest.json").read_text())
    if destination.exists():
        raise ValueError("destination must not exist; refusing to overwrite sources")
    # Validate every original before creating a staging directory or applying patches.
    if sha((BUNDLE / "CMakeLists.txt").read_bytes()) != manifest["gz-sim"]["build_adapter_sha256"]:
        raise ValueError("system build adapter SHA256 mismatch")
    if (
        sha((BUNDLE / "core.CMakeLists.txt").read_bytes())
        != manifest["gz-sim"]["core_build_adapter_sha256"]
    ):
        raise ValueError("core build adapter SHA256 mismatch")
    sources = {}
    for project, info in manifest.items():
        for name, expected in info["files"].items():
            path = relative(name)
            original = cache / project / path
            if original.is_symlink():
                raise ValueError(f"symlink source rejected: {original}")
            if original.exists():
                data = original.read_bytes()
            elif fetch:
                repository = info["repository"].replace(
                    "https://github.com/", "https://raw.githubusercontent.com/"
                )
                url = f"{repository}/{info['commit']}/{name}"
                with urllib.request.urlopen(url, timeout=60) as response:
                    data = response.read()
            else:
                raise ValueError(
                    f"missing fixed source: {original}; use --fetch to download only manifest files"
                )
            if sha(data) != expected:
                raise ValueError(f"source SHA256 mismatch: {project}/{name}")
            sources[project, path] = data
        patch = BUNDLE / (project + ".patch")
        if sha(patch.read_bytes()) != info["patch_sha256"]:
            raise ValueError(f"patch SHA256 mismatch: {patch}")
    destination.mkdir(parents=True)
    for (project, name), data in sources.items():
        target = destination / project / name
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(data)
    for project, info in manifest.items():
        run(
            [
                "patch",
                "--batch",
                "--forward",
                "--fuzz=0",
                "-p1",
                "-i",
                str(BUNDLE / (project + ".patch")),
            ],
            cwd=destination / project,
        )
        for name, expected in info["patched_files"].items():
            target = destination / project / relative(name)
            if sha(target.read_bytes()) != expected:
                raise ValueError(f"patched SHA256 mismatch: {project}/{name}")
    (destination / "core-build").mkdir()
    shutil.copyfile(BUNDLE / "core.CMakeLists.txt", destination / "core-build" / "CMakeLists.txt")
    (destination / "system-build").mkdir()
    shutil.copyfile(BUNDLE / "CMakeLists.txt", destination / "system-build" / "CMakeLists.txt")
    (destination / "source-manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(
        f"Prepared {len(sources)} fixed source files; exact patch contexts and final SHA256 "
        "verified; no build/deployment performed"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--cache", type=Path, required=True, help="original selected source cache (read only)"
    )
    parser.add_argument("--output", type=Path, required=True, help="new external staging directory")
    parser.add_argument(
        "--fetch",
        action="store_true",
        help="download missing exact manifest files from official immutable URLs",
    )
    parser.add_argument(
        "--build",
        action="store_true",
        help="Linux/Humble only; build selected overlay packages, never deploy",
    )
    args = parser.parse_args()
    destination = args.output.resolve()
    prepare(args.cache.resolve(), destination, args.fetch)
    if args.build:
        build(destination)


if __name__ == "__main__":
    main()
