"""Read-only profile and ROS graph diagnostics.

The doctor never starts ROS nodes, devices, downloads, or model loading. Live
checks are reported as ``not_run`` when the ROS CLI is unavailable.
"""

from __future__ import annotations

import json
import platform
import re
import shutil
import subprocess
from pathlib import Path
from typing import Literal, NamedTuple

import yaml

from senior_control.config import DeviceProfile

Status = Literal["passed", "failed", "not_run"]


class DiagnosticCheck(NamedTuple):
    status: Status
    detail: str


class InspectionData(NamedTuple):
    versions: dict[str, str]
    topics: tuple[str, ...]
    publishers: dict[str, tuple[str, ...]]
    transforms: tuple[tuple[str, str, str], ...]
    clock: str | None
    chassis_watchdog: bool | None
    model_state: str | None


class DiagnosticReport(NamedTuple):
    profile_path: Path
    mode: str | None
    checks: dict[str, DiagnosticCheck]

    @property
    def ok(self) -> bool:
        return all(check.status != "failed" for check in self.checks.values())

    def as_dict(self) -> dict[str, object]:
        return {
            "profile_path": str(self.profile_path),
            "mode": self.mode,
            "ok": self.ok,
            "checks": {
                name: {"status": check.status, "detail": check.detail}
                for name, check in self.checks.items()
            },
        }


class _UniqueKeyLoader(yaml.SafeLoader):
    pass


def _mapping(loader: yaml.SafeLoader, node: yaml.nodes.MappingNode) -> dict[object, object]:
    result: dict[object, object] = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=False)
        if key in result:
            raise ValueError(f"duplicate profile key: {key}")
        result[key] = loader.construct_object(value_node, deep=False)
    return result


_UniqueKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
    _mapping,
)


def _load_profile(path: Path) -> DeviceProfile:
    try:
        raw = yaml.load(path.read_text(encoding="utf-8"), Loader=_UniqueKeyLoader)
    except (OSError, yaml.YAMLError, ValueError) as exc:
        raise ValueError(str(exc)) from None
    if not isinstance(raw, dict):
        raise ValueError("profile must be a mapping")
    try:
        return DeviceProfile.model_validate(raw)
    except Exception as exc:
        raise ValueError("invalid device profile") from exc


def _run_ros(*arguments: str) -> str:
    completed = subprocess.run(
        ("ros2", *arguments),
        check=True,
        capture_output=True,
        text=True,
        timeout=5,
    )
    return completed.stdout


def _publisher_names(topic: str) -> tuple[str, ...]:
    output = _run_ros("topic", "info", topic, "--verbose")
    names = re.findall(r"Node name:\s*(\S+)", output)
    return tuple(names)


def _inspect_live_system() -> InspectionData | None:
    """Inspect an existing ROS graph without changing it."""
    if shutil.which("ros2") is None:
        return None
    try:
        topics = tuple(line.strip() for line in _run_ros("topic", "list").splitlines() if line)
        publishers = {topic: _publisher_names(topic) for topic in topics}
        ros_version = _run_ros("--help").splitlines()[0].strip() or "installed"
    except (OSError, subprocess.SubprocessError):
        return None
    return InspectionData(
        versions={"python": platform.python_version(), "ros2": ros_version},
        topics=topics,
        publishers=publishers,
        transforms=(),
        clock=None,
        chassis_watchdog=None,
        model_state=None,
    )


def _duplicates(values: tuple[str, ...]) -> tuple[str, ...]:
    return tuple(sorted(value for value in set(values) if values.count(value) > 1))


def doctor(profile_path: Path) -> DiagnosticReport:
    """Validate one ``DeviceProfile`` and inspect an already-running graph."""
    checks: dict[str, DiagnosticCheck] = {}
    try:
        profile = _load_profile(profile_path)
    except ValueError as exc:
        checks["profile"] = DiagnosticCheck("failed", str(exc))
        for name in (
            "versions",
            "topics",
            "tf",
            "clock",
            "publisher_counts",
            "chassis_watchdog",
            "coverage",
            "model_state",
        ):
            checks[name] = DiagnosticCheck("not_run", "profile validation failed")
        return DiagnosticReport(profile_path, None, checks)

    checks["profile"] = DiagnosticCheck("passed", f"{profile.mode} profile is valid")
    checks["coverage"] = DiagnosticCheck(
        "passed",
        ", ".join(f"{name}={span:g}rad" for name, span in profile.sensor_coverage.items())
        or "no sensor coverage declared",
    )
    inspection = _inspect_live_system()
    if inspection is None:
        for name in (
            "versions",
            "topics",
            "tf",
            "clock",
            "publisher_counts",
            "chassis_watchdog",
            "model_state",
        ):
            checks[name] = DiagnosticCheck("not_run", "live ROS graph was not available")
        return DiagnosticReport(profile_path, profile.mode, checks)

    checks["versions"] = DiagnosticCheck(
        "passed", ", ".join(f"{key}={value}" for key, value in inspection.versions.items())
    )
    checks["topics"] = DiagnosticCheck("passed", ", ".join(inspection.topics) or "no topics")
    cmd_publishers = inspection.publishers.get("/cmd_vel", ())
    if len(cmd_publishers) != 1 or cmd_publishers[0].rsplit("/", 1)[-1] != "safety_node":
        found = ", ".join(cmd_publishers) or "none"
        checks["publisher_counts"] = DiagnosticCheck(
            "failed",
            f"/cmd_vel publishers must be safety_node only; found {found}",
        )
    else:
        checks["publisher_counts"] = DiagnosticCheck("passed", "/cmd_vel: safety_node")

    children = tuple(child for _, child, _ in inspection.transforms)
    duplicate_children = _duplicates(children)
    checks["tf"] = DiagnosticCheck(
        "failed" if duplicate_children else "passed",
        (
            f"duplicate TF publishers for child frames: {', '.join(duplicate_children)}"
            if duplicate_children
            else f"{len(inspection.transforms)} child transforms have one publisher"
        ),
    )
    checks["clock"] = (
        DiagnosticCheck("passed", inspection.clock)
        if inspection.clock is not None
        else DiagnosticCheck("not_run", "clock source was not supplied by the live graph")
    )
    checks["chassis_watchdog"] = (
        DiagnosticCheck(
            "passed" if inspection.chassis_watchdog else "failed",
            "enabled" if inspection.chassis_watchdog else "disabled",
        )
        if inspection.chassis_watchdog is not None
        else DiagnosticCheck("not_run", "chassis watchdog state was not supplied")
    )
    checks["model_state"] = (
        DiagnosticCheck("passed", inspection.model_state)
        if inspection.model_state is not None
        else DiagnosticCheck("not_run", "model state was not supplied")
    )
    return DiagnosticReport(profile_path, profile.mode, checks)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("profile", type=Path)
    arguments = parser.parse_args()
    result = doctor(arguments.profile)
    print(json.dumps(result.as_dict(), ensure_ascii=False, indent=2))
    raise SystemExit(0 if result.ok else 1)
