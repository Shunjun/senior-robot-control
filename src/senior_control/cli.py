"""Operator commands for explicitly selected vehicle profiles."""

import argparse
import asyncio
import os
import re
import shutil
import stat
import subprocess
import sys
from getpass import getpass
from pathlib import Path
from typing import Literal
from urllib.parse import urlsplit

from pydantic import BaseModel, ConfigDict

from senior_control.agent.pairing import pair_device, read_credentials


class VehicleProfile(BaseModel):
    model_config = ConfigDict(extra="forbid", hide_input_in_errors=True)
    kind: Literal["real", "simulation"]
    credentials: Path
    ros_setup: Path | None = None
    database: Path | None = None
    socket: Path | None = None
    cloud_url: str | None = None
    estop_state: Path | None = None
    ros_domain: int | None = None


def _pair(profile: VehicleProfile, cloud_url: str, name: str) -> None:
    if not profile.credentials.is_absolute():
        raise ValueError("credential path must be absolute")
    if os.path.lexists(profile.credentials):
        raise FileExistsError
    url = urlsplit(cloud_url)
    if url.scheme not in {"https", "wss"} or not url.hostname or url.username or url.password:
        raise ValueError("cloud URL must use TLS")
    if not name.strip():
        raise ValueError("device name is required")
    code = getpass("Pairing code: ")
    asyncio.run(pair_device(url._replace(scheme="wss").geturl(), code, name, profile.credentials))


def _doctor(profile: VehicleProfile) -> int:
    problems: list[str] = []
    try:
        if not profile.credentials.is_absolute():
            raise ValueError
        read_credentials(profile.credentials)
    except (OSError, ValueError):
        problems.append("Credentials: provide a private file created by pair.")
    if profile.ros_setup is None or not (
        profile.ros_setup.is_absolute() and profile.ros_setup.is_file()
    ):
        problems.append("ROS build: set ros_setup to the installed workspace setup script.")
    if shutil.which("ros2") is None:
        problems.append("ROS: source the installed ROS environment and run doctor again.")
    if profile.database is None or not (
        profile.database.is_absolute() and profile.database.parent.is_dir()
    ):
        problems.append("Database: provide an absolute path in an existing state directory.")
    if profile.socket is None or not (
        profile.socket.is_absolute()
        and len(os.fsencode(profile.socket)) <= 100
        and profile.socket.parent.is_dir()
    ):
        problems.append("IPC: provide a short absolute socket path in an existing directory.")
    if profile.kind == "simulation":
        if not (shutil.which("gz") or shutil.which("ign")):
            problems.append("Gazebo: source the simulator environment and run doctor again.")
        if shutil.which("xvfb-run") is None:
            problems.append("Display: provide xvfb-run for headless simulation.")
        if profile.credentials.parent.is_dir():
            disk = shutil.disk_usage(profile.credentials.parent)
            print(f"Free disk: {disk.free / 2**30:.1f} GiB")
        if sys.platform == "linux":
            for line in Path("/proc/meminfo").read_text().splitlines():
                if line.startswith("MemAvailable:"):
                    print(f"Available memory: {int(line.split()[1]) / 2**20:.1f} GiB")
                    break
    if problems:
        print("\n".join(problems))
        return 1
    print("Profile paths, credentials and required executables are available.")
    return 0


SERVICE_ROOT = Path("/opt/senior-control")
ENV_ROOT = Path("/etc/senior-control")
UNIT_ROOT = Path("/etc/systemd/system")


def _unit(profile_path: Path) -> str:
    if not re.fullmatch(r"[a-z0-9][a-z0-9-]{0,39}", profile_path.stem):
        raise ValueError("profile filename must be a lowercase service name")
    return f"senior-agent-{profile_path.stem}.service"


def _install_once(path: Path, contents: str, mode: int) -> None:
    import tempfile

    fd, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as output:
            os.fchmod(output.fileno(), mode)
            output.write(contents)
            output.flush()
            os.fsync(output.fileno())
        os.link(temporary, path)
    finally:
        os.unlink(temporary)


def _deploy(profile: VehicleProfile, profile_path: Path, *, dry_run: bool) -> int:
    if profile.kind != "real":
        raise ValueError("deploy requires a real vehicle profile")
    unit = _unit(profile_path)
    state_root = Path("/var/lib/senior-control") / profile_path.stem
    socket_root = Path("/run/senior-control") / profile_path.stem
    fields = {
        "SENIOR_CREDENTIALS_FILE": (profile.credentials, state_root),
        "SENIOR_DATABASE_PATH": (profile.database, state_root),
        "SENIOR_AGENT_SOCKET": (profile.socket, socket_root),
    }
    values: dict[str, str] = {}
    for key, (path, root) in fields.items():
        if path is None or path.parent != root or not re.fullmatch(r"[a-zA-Z0-9_./-]+", str(path)):
            raise ValueError("invalid vehicle path")
        values[key] = str(path)
    if profile.credentials == profile.database:
        raise ValueError("database and credential paths must differ")
    if profile.socket is None or len(os.fsencode(profile.socket)) > 100:
        raise ValueError("socket path is too long")
    cloud = urlsplit(profile.cloud_url or "")
    if (
        cloud.scheme != "wss"
        or not cloud.hostname
        or cloud.username
        or cloud.password
        or not re.fullmatch(r"wss://[a-zA-Z0-9.:/\[\]_-]+", profile.cloud_url or "")
    ):
        raise ValueError("cloud URL must use TLS")
    values["SENIOR_CLOUD_URL"] = cloud.geturl()
    env_file = ENV_ROOT / f"{profile_path.stem}.env"
    unit_file = UNIT_ROOT / unit
    if dry_run:
        print(f"Would create {env_file} and {unit_file}.")
        return 0

    if sys.platform != "linux" or os.geteuid() != 0:
        raise ValueError("real deployment requires root on Linux")
    import pwd

    account = pwd.getpwnam("senior-control")
    for directory in (state_root, socket_root):
        info = directory.stat()
        if (
            directory.resolve() != directory
            or not stat.S_ISDIR(info.st_mode)
            or info.st_uid != account.pw_uid
            or info.st_mode & 0o022
        ):
            raise ValueError("service directories must belong to senior-control")
    credential = profile.credentials.lstat()
    if credential.st_uid != account.pw_uid or stat.S_IMODE(credential.st_mode) != 0o600:
        raise ValueError("credential must belong to senior-control with mode 0600")
    read_credentials(profile.credentials)
    if not os.access(SERVICE_ROOT / ".venv/bin/python", os.X_OK):
        raise ValueError("service Python is unavailable")
    template = (SERVICE_ROOT / "deploy/senior-agent.service").read_text(encoding="utf-8")
    unit_contents = template.replace("/etc/senior-control/agent.env", str(env_file))
    env_contents = "".join(f"{key}={value}\n" for key, value in values.items())
    written: list[Path] = []
    try:
        for path, data, mode in (
            (env_file, env_contents, 0o600),
            (unit_file, unit_contents, 0o644),
        ):
            _install_once(path, data, mode)
            written.append(path)
        subprocess.run(["systemctl", "daemon-reload"], check=True, timeout=10)
    except BaseException:
        for path in reversed(written):
            path.unlink(missing_ok=True)
        raise
    print(f"Deployed {unit}.")
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="senior-robot")
    commands = parser.add_subparsers(dest="command", required=True)
    pair = commands.add_parser("pair", help="exchange a one-time code over HTTPS")
    pair.add_argument("--profile", required=True, type=Path)
    pair.add_argument("--cloud-url", required=True)
    pair.add_argument("--name", required=True)
    doctor = commands.add_parser("doctor", help="check the profile without starting services")
    doctor.add_argument("--profile", required=True, type=Path)
    deploy = commands.add_parser("deploy", help="install the selected real vehicle service")
    deploy.add_argument("--profile", required=True, type=Path)
    deploy.add_argument("--dry-run", action="store_true")
    for command in ("start", "stop", "status"):
        service = commands.add_parser(command)
        service.add_argument("--profile", required=True, type=Path)
    args = parser.parse_args(argv)
    try:
        profile = VehicleProfile.model_validate_json(args.profile.read_bytes())
        if args.command == "doctor":
            return _doctor(profile)
        if args.command == "deploy":
            return _deploy(profile, args.profile, dry_run=args.dry_run)
        if args.command != "pair":
            if profile.kind == "simulation":
                from senior_control.simulation_cli import lifecycle

                return lifecycle(args.command, args.profile, profile)
            if profile.kind != "real" or sys.platform != "linux":
                raise ValueError("systemd lifecycle requires a real Linux profile")
            unit = _unit(args.profile)
            if not (UNIT_ROOT / unit).is_file():
                raise ValueError("deploy the selected profile first")
            if args.command == "status":
                from senior_control.simulation_cli import service_status

                return service_status(unit, profile, user=False)
            return subprocess.run(
                ["systemctl", args.command, unit],
                check=False,
                timeout=30,
            ).returncode
        _pair(profile, args.cloud_url, args.name)
    except FileExistsError:
        message = (
            "Pairing skipped. Credential file already exists."
            if args.command == "pair"
            else "Deployment skipped. A destination file already exists."
        )
        print(message, file=sys.stderr)
        return 1
    except (OSError, ValueError, EOFError, KeyboardInterrupt, KeyError, subprocess.SubprocessError):
        message = (
            "Could not read a valid vehicle profile."
            if args.command != "pair"
            else "Pairing failed. Check the profile, TLS URL and pairing code."
        )
        print(message, file=sys.stderr)
        return 1
    print("Device paired.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
