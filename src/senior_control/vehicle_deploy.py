"""Host systemd deployment using a single private data directory."""

from __future__ import annotations

import json
import os
import re
import shutil
import stat
import subprocess
import sys
from pathlib import Path
from typing import TYPE_CHECKING, Any
from urllib.parse import urlsplit

from senior_control.agent.pairing import read_credentials
from senior_control.runtime.ipc import MAX_PATH

if TYPE_CHECKING:
    from senior_control.cli import VehicleProfile


def private_directory(root: Path) -> None:
    info = root.lstat()
    if not root.is_absolute() or root.resolve() != root or not stat.S_ISDIR(info.st_mode):
        raise ValueError("control directory must be an absolute, non-symlink directory")
    if info.st_mode & 0o077:
        raise ValueError("control directory must be private (chmod 700)")


def container_directory(root: Path, mounts: list[dict[str, Any]]) -> Path:
    candidates = []
    for mount in mounts:
        if mount.get("Type") != "bind":
            continue
        source = Path(mount["Source"])
        if root.is_relative_to(source):
            candidates.append((len(source.parts), mount, root.relative_to(source)))
    if not candidates:
        raise ValueError("control directory needs an existing Docker bind mount")
    _, selected, suffix = max(candidates, key=lambda item: item[0])
    if selected.get("RW") is not True:
        raise ValueError("control directory bind mount must be writable")
    return Path(selected["Destination"]) / suffix


def inspect_container(name: str) -> dict[str, Any]:
    if not re.fullmatch(r"[a-zA-Z0-9][a-zA-Z0-9_.-]{0,127}", name):
        raise ValueError("invalid container name")
    result = subprocess.run(
        ["docker", "inspect", name], check=True, capture_output=True, text=True, timeout=10
    )
    items = json.loads(result.stdout)
    if not isinstance(items, list) or len(items) != 1 or not isinstance(items[0], dict):
        raise ValueError("invalid Docker container inspection")
    return items[0]


def _path_text(path: Path) -> str:
    value = str(path)
    if not path.is_absolute() or any(c in value for c in "\n\r\0\"'\\"):
        raise ValueError("paths must be absolute and single-line")
    return value


def _argument(value: str | Path, *, command: bool = False) -> str:
    text = str(value)
    if any(c in text for c in "\n\r\0"):
        raise ValueError("invalid service argument")
    text = text.replace("%", "%%")
    if command:
        text = text.replace("$", "$$")
    if re.fullmatch(r"[a-zA-Z0-9_./:=@-]+", text):
        return text
    return '"' + text.replace("\\", "\\\\").replace('"', '\\"') + '"'


def _command(arguments: list[str]) -> str:
    return " ".join(_argument(value, command=True) for value in arguments)


def _project(profile: VehicleProfile) -> Path:
    if profile.project_dir is None:
        raise ValueError("project_dir must identify the installed project in its runtime")
    _path_text(profile.project_dir)
    return profile.project_dir


def environment(profile: VehicleProfile, runtime_root: Path) -> str:
    cloud = urlsplit(profile.cloud_url or "")
    if cloud.scheme != "wss" or not cloud.hostname or cloud.username or cloud.password:
        raise ValueError("cloud_url must use authenticated TLS")
    if any(c in (profile.cloud_url or "") for c in "\n\r\0"):
        raise ValueError("cloud_url must be single-line")
    values = {
        "SENIOR_CREDENTIALS_FILE": _path_text(runtime_root / "device.json"),
        "SENIOR_DATABASE_PATH": _path_text(runtime_root / "agent.db"),
        "SENIOR_AGENT_SOCKET": _path_text(runtime_root / "agent.sock"),
        "SENIOR_CLOUD_URL": profile.cloud_url,
    }
    return "".join(f"{key}={value}\n" for key, value in values.items())


def render_service(profile: VehicleProfile, user: str, *, container_root: Path | None) -> str:
    assert profile.control_dir is not None
    root = profile.control_dir
    project = _project(profile)
    python = project / ".ros-venv/bin/python"
    env_file = root / "agent.env"
    lines = ["[Unit]", "Description=Senior Control robot agent", "After=network-online.target"]
    lines += [
        "Wants=network-online.target",
        f"ConditionPathExists={_argument(env_file)}",
        "",
        "[Service]",
        "Type=simple",
    ]
    if profile.container is not None:
        if not re.fullmatch(r"[a-zA-Z0-9][a-zA-Z0-9_.-]{0,127}", profile.container):
            raise ValueError("invalid container name")
        assert container_root is not None
        docker = shutil.which("docker") or "/usr/bin/docker"
        lines.insert(2, "Requires=docker.service")
        lines[3] = "After=docker.service network-online.target"
        lines += [
            "User=root",
            "ExecStartPre=" + _command([docker, "start", profile.container]),
        ]
        run = [
            docker,
            "exec",
            "--env-file",
            str(env_file),
            "--workdir",
            str(project),
            profile.container,
            str(python),
            "-m",
            "senior_control.docker_start_helper",
            "run",
            "--control-dir",
            str(container_root),
        ]
        stop = [
            docker,
            "exec",
            profile.container,
            str(python),
            "-m",
            "senior_control.docker_start_helper",
            "stop",
            "--control-dir",
            str(container_root),
        ]
        lines += [
            "ExecStart=" + _command(run),
            "ExecStop=" + _command(stop),
        ]
    else:
        lines += [
            f"User={user}",
            f"WorkingDirectory={_argument(project)}",
            f"EnvironmentFile={_argument(env_file)}",
            "ExecStart="
            + _command(
                [
                    str(python),
                    "-m",
                    "senior_control.docker_start_helper",
                    "run",
                    "--control-dir",
                    str(root),
                ]
            ),
            "KillMode=mixed",
        ]
    lines += [
        "Restart=on-failure",
        "RestartPreventExitStatus=78",
        "RestartSec=5",
        "TimeoutStopSec=30",
        "UMask=0077",
        "NoNewPrivileges=true",
        "ProtectSystem=strict",
        "ProtectHome=read-only",
        f"ReadWritePaths={_argument(root)}",
        "",
        "[Install]",
        "WantedBy=multi-user.target",
        "",
    ]
    return "\n".join(lines)


def _runtime_root(profile: VehicleProfile) -> Path:
    assert profile.control_dir is not None
    if profile.container is None:
        return profile.control_dir
    inspection = inspect_container(profile.container)
    return container_directory(profile.control_dir, inspection["Mounts"])


def _validate_paths(profile: VehicleProfile, runtime_root: Path) -> None:
    _project(profile)
    for root in (profile.control_dir, runtime_root):
        assert root is not None
        _path_text(root)
        if len(os.fsencode(root / "service.sock")) > MAX_PATH:
            raise ValueError("control directory is too long for Unix sockets")
    environment(profile, runtime_root)


def deploy(profile: VehicleProfile, unit: str, *, dry_run: bool) -> int:
    from senior_control.cli import UNIT_ROOT, _install_once

    if profile.kind != "real" or profile.control_dir is None:
        raise ValueError("deployment requires a real control directory profile")
    root = profile.control_dir
    runtime_root = _runtime_root(profile)
    _validate_paths(profile, runtime_root)
    contents = render_service(profile, str(root.stat().st_uid), container_root=runtime_root)
    unit_file = root / unit
    if dry_run:
        print(f"Environment: {root / 'agent.env'}")
        print(f"Service: {unit_file} (linked by systemd on the host)")
        print(f"Runtime data directory: {runtime_root}")
        print(contents)
        return 0
    if sys.platform != "linux" or os.geteuid() != 0:
        raise ValueError("deployment requires root on the Linux systemd host")
    private_directory(root)
    read_credentials(root / "device.json")
    if (root / "device.json").stat().st_uid != root.stat().st_uid:
        raise ValueError("credentials and control directory must have the same owner")
    if profile.container is None:
        python = _project(profile) / ".ros-venv/bin/python"
        if not os.access(python, os.X_OK):
            raise ValueError("project .ros-venv/bin/python is unavailable")
    else:
        check = subprocess.run(
            [
                "docker",
                "exec",
                profile.container,
                str(_project(profile) / ".ros-venv/bin/python"),
                "-c",
                "import senior_control.docker_start_helper",
            ],
            check=False,
            capture_output=True,
            timeout=15,
        )
        if check.returncode:
            raise ValueError("container must be running with this project installed")
    if os.path.lexists(UNIT_ROOT / unit):
        raise FileExistsError("a service with this identity already exists")
    written = []
    linked = False
    try:
        for file, data, mode in (
            (root / "agent.env", environment(profile, runtime_root), 0o600),
            (unit_file, contents, 0o644),
        ):
            _install_once(file, data, mode)
            written.append(file)
        subprocess.run(["systemctl", "link", str(unit_file)], check=True, timeout=10)
        linked = True
        subprocess.run(["systemctl", "daemon-reload"], check=True, timeout=10)
    except BaseException:
        link = UNIT_ROOT / unit
        if linked and link.is_symlink() and link.resolve() == unit_file:
            link.unlink()
        for file in reversed(written):
            file.unlink()
        raise
    print(f"Deployed {unit}. Start explicitly with senior-robot start --control-dir {root}")
    return 0


def doctor(profile: VehicleProfile) -> int:
    assert profile.control_dir is not None
    try:
        private_directory(profile.control_dir)
        read_credentials(profile.credentials)
        root = _runtime_root(profile)
        _validate_paths(profile, root)
        argv = [
            str(_project(profile) / ".ros-venv/bin/python"),
            "-c",
            "import senior_control.agent.main",
        ]
        if profile.container is not None:
            argv = ["docker", "exec", profile.container, *argv]
        result = subprocess.run(
            argv,
            check=False,
            capture_output=True,
            timeout=15,
        )
        if result.returncode:
            raise ValueError("container Python is unavailable")
    except (OSError, ValueError, subprocess.SubprocessError):
        print(
            "Agent unavailable. Check the runtime, bind mount, "
            "private directory and project .ros-venv."
        )
        return 1
    print(
        "Agent Python and control directory verified. Start manages the ROS adapter and Agent; "
        "ROS readiness is checked at startup."
    )
    return 0
