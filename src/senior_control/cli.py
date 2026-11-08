"""Operator commands for explicitly selected vehicle profiles."""

import argparse
import asyncio
import os
import shutil
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
    if profile.kind == "simulation" and not (shutil.which("gz") or shutil.which("ign")):
        problems.append("Gazebo: source the simulator environment and run doctor again.")
    if problems:
        print("\n".join(problems))
        return 1
    print("Profile paths, credentials and required executables are available.")
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
    args = parser.parse_args(argv)
    try:
        profile = VehicleProfile.model_validate_json(args.profile.read_bytes())
        if args.command == "doctor":
            return _doctor(profile)
        _pair(profile, args.cloud_url, args.name)
    except FileExistsError:
        print("Pairing skipped. Credential file already exists.", file=sys.stderr)
        return 1
    except (OSError, ValueError, EOFError, KeyboardInterrupt):
        message = (
            "Could not read a valid vehicle profile."
            if args.command == "doctor"
            else "Pairing failed. Check the profile, TLS URL and pairing code."
        )
        print(message, file=sys.stderr)
        return 1
    print("Device paired.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
