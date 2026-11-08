"""Operator commands for explicitly selected vehicle profiles."""

import argparse
import asyncio
import os
import sys
from getpass import getpass
from pathlib import Path
from typing import Literal
from urllib.parse import urlsplit

from pydantic import BaseModel, ConfigDict

from senior_control.agent.pairing import pair_device


class VehicleProfile(BaseModel):
    model_config = ConfigDict(extra="forbid", hide_input_in_errors=True)
    kind: Literal["real", "simulation"]
    credentials: Path


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


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="senior-robot")
    commands = parser.add_subparsers(dest="command", required=True)
    pair = commands.add_parser("pair", help="exchange a one-time code over HTTPS")
    pair.add_argument("--profile", required=True, type=Path)
    pair.add_argument("--cloud-url", required=True)
    pair.add_argument("--name", required=True)
    args = parser.parse_args(argv)
    try:
        profile = VehicleProfile.model_validate_json(args.profile.read_bytes())
        _pair(profile, args.cloud_url, args.name)
    except FileExistsError:
        print("Pairing skipped. Credential file already exists.", file=sys.stderr)
        return 1
    except (OSError, ValueError, EOFError, KeyboardInterrupt):
        print("Pairing failed. Check the profile, TLS URL and pairing code.", file=sys.stderr)
        return 1
    print("Device paired.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
