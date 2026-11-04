"""Explicit one-time provisioning of a new simulation's safety state."""

import argparse
from pathlib import Path

from senior_control.safety.arbiter import PersistentEstop


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--simulation", action="store_true", required=True)
    parser.add_argument("--robot-id", required=True)
    parser.add_argument("--state", required=True, type=Path)
    args = parser.parse_args(argv)
    PersistentEstop.initialize(args.state, args.robot_id)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
