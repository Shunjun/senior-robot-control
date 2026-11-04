#!/usr/bin/env python3
"""Validate the restricted, literal Compose credential format without exposing values."""

import re
import sys
from pathlib import Path

ALLOWED = {
    "ACME_EMAIL",
    "ENABLE_TURN",
    "SENIOR_SECRET",
    "ADMIN_PASSWORD_HASH",
    "TURN_LISTEN_IP",
    "TURN_EXTERNAL_IP",
    "TURN_REALM",
    "TURN_SHARED_SECRET",
}
REQUIRED = {"SENIOR_SECRET", "ADMIN_PASSWORD_HASH"}
EMAIL = re.compile(r"[A-Za-z0-9._%+-]+@[A-Za-z0-9.-]+\.[A-Za-z]{2,}")
LINE = re.compile(r"([A-Z_]+)='([A-Za-z0-9._~:@/+=$%-]+)'")
HASH = re.compile(r"scrypt\$16384\$8\$1\$[0-9a-f]{32}\$[0-9a-f]{64}")


def main() -> int:
    try:
        content = Path(sys.argv[1]).read_text(encoding="utf-8")
        values = {}
        for number, line in enumerate(content.splitlines(), 1):
            if not line or line.startswith("#"):
                continue
            match = LINE.fullmatch(line)
            if not match or match[1] not in ALLOWED or match[1] in values:
                raise ValueError(f"invalid credential entry at line {number}")
            values[match[1]] = match[2]
        if "\r" in content or not REQUIRED <= values.keys():
            raise ValueError("missing required credentials or invalid line endings")
        if len(values["SENIOR_SECRET"].encode()) < 32 or not HASH.fullmatch(
            values["ADMIN_PASSWORD_HASH"]
        ):
            raise ValueError("invalid secret length or scrypt hash format")
        if "TURN_SHARED_SECRET" in values and len(values["TURN_SHARED_SECRET"].encode()) < 32:
            raise ValueError("invalid TURN secret length")
        if not EMAIL.fullmatch(values.get("ACME_EMAIL", "")):
            print("Credential file requires a valid ACME_EMAIL", file=sys.stderr)
            return 1
        enable_turn = values.get("ENABLE_TURN", "false")
        if enable_turn not in {"true", "false"}:
            print("Credential file ENABLE_TURN must be true or false", file=sys.stderr)
            return 1
    except (OSError, UnicodeError, ValueError, IndexError) as exc:
        print(f"Credential file rejected: {exc.__class__.__name__}", file=sys.stderr)
        return 1
    print(enable_turn)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
