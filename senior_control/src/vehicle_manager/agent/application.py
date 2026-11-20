"""Original Agent command-line driver over explicit per-call production assembly.

Configuration and execution are supplied by the installed compatibility boundary;
this module does not invent a default runtime or start a second state owner.
"""

import argparse
import asyncio
import os
import sys
from collections.abc import Callable, Coroutine, Mapping
from pathlib import Path
from typing import Any, TypeVar

from vehicle_manager.observability import configure_logging

ConfigurationT = TypeVar("ConfigurationT")


def main(
    *,
    configuration: Callable[[Mapping[str, str]], ConfigurationT],
    run_agent: Callable[[ConfigurationT], Coroutine[Any, Any, int]],
) -> int:
    """Load explicit configuration and run the owned agent composition."""
    parser = argparse.ArgumentParser(description="Robot cloud agent")
    parser.add_argument("--robot-id")
    parser.add_argument("--cloud-url")
    parser.add_argument("--database-path")
    parser.add_argument("--socket-path")
    parser.add_argument("--token-file", type=Path)
    parser.add_argument("--credentials-file", type=Path)
    parser.add_argument("--pair", action="store_true")
    parser.add_argument("--device-name")
    args = parser.parse_args()
    values = dict(os.environ)
    for name, value in (
        ("SENIOR_ROBOT_ID", args.robot_id),
        ("SENIOR_CLOUD_URL", args.cloud_url),
        ("SENIOR_DATABASE_PATH", args.database_path),
        ("SENIOR_AGENT_SOCKET", args.socket_path),
    ):
        if value is not None:
            values[name] = value
    try:
        from vehicle_manager.agent.pairing import pair_device, read_credentials, read_token

        credentials_file = args.credentials_file
        if credentials_file is None and values.get("SENIOR_CREDENTIALS_FILE"):
            credentials_file = Path(values["SENIOR_CREDENTIALS_FILE"])
        if args.pair:
            import getpass

            cloud_url = values.get("SENIOR_CLOUD_URL")
            if not cloud_url or credentials_file is None or not args.device_name:
                raise ValueError("pair requires cloud URL, device name and credentials file")
            code = getpass.getpass("Pairing code: ")
            robot_id = asyncio.run(pair_device(cloud_url, code, args.device_name, credentials_file))
            print(f"Paired robot {robot_id}")
            return 0
        if credentials_file is not None:
            credentials = read_credentials(credentials_file)
            values["SENIOR_ROBOT_ID"] = credentials.robot_id
            values["SENIOR_DEVICE_TOKEN"] = credentials.device_token
        if args.token_file is not None:
            values["SENIOR_DEVICE_TOKEN"] = read_token(args.token_file)
        config = configuration(values)
    except (OSError, ValueError) as exc:
        print(f"invalid agent configuration: {exc}", file=sys.stderr)
        return 78
    configure_logging()
    return asyncio.run(run_agent(config))
