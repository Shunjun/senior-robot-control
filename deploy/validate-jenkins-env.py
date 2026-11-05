#!/usr/bin/env python3
"""Validate the restricted, literal Compose credential format without exposing values."""

import re
import sys
from pathlib import Path

ALLOWED = {
    "DOMAIN",
    "ACME_EMAIL",
    "ENABLE_TURN",
    "SENIOR_SECRET",
    "ADMIN_PASSWORD_HASH",
    "TURN_LISTEN_IP",
    "TURN_EXTERNAL_IP",
    "TURN_REALM",
    "TURN_SHARED_SECRET",
}
REQUIRED = {"DOMAIN", "ACME_EMAIL", "SENIOR_SECRET", "ADMIN_PASSWORD_HASH"}
TURN_REQUIRED = {"TURN_REALM", "TURN_LISTEN_IP", "TURN_EXTERNAL_IP", "TURN_SHARED_SECRET"}
DOMAIN = re.compile(
    r"(?:[A-Za-z0-9](?:[A-Za-z0-9-]{0,61}[A-Za-z0-9])?\.)+"
    r"[A-Za-z0-9](?:[A-Za-z0-9-]{0,61}[A-Za-z0-9])?"
)
EMAIL = re.compile(r"[A-Za-z0-9._%+-]+@[A-Za-z0-9.-]+\.[A-Za-z]{2,}")
LINE = re.compile(r"([A-Z_]+)='([A-Za-z0-9._~:@/+=$%-]+)'")
HASH = re.compile(r"scrypt\$16384\$8\$1\$[0-9a-f]{32}\$[0-9a-f]{64}")


class CredentialFormatError(Exception):
    """Validation failure with a message safe to include in build logs."""


def main() -> int:
    try:
        content = Path(sys.argv[1]).read_text(encoding="utf-8")
        values = {}
        for number, line in enumerate(content.splitlines(), 1):
            if not line or line.startswith("#"):
                continue
            match = LINE.fullmatch(line)
            if not match:
                raise CredentialFormatError(
                    f"line {number} must use NAME='value' with supported characters"
                )
            key, value = match.groups()
            if key not in ALLOWED:
                raise CredentialFormatError(f"unsupported setting on line {number}")
            if key in values:
                raise CredentialFormatError(f"duplicate setting {key} on line {number}")
            values[key] = value
        missing = REQUIRED - values.keys()
        if missing:
            raise CredentialFormatError(f"missing required settings: {', '.join(sorted(missing))}")
        if not DOMAIN.fullmatch(values["DOMAIN"]):
            raise CredentialFormatError("DOMAIN must be a DNS hostname without a scheme or port")
        if len(values["SENIOR_SECRET"].encode()) < 32:
            raise CredentialFormatError("SENIOR_SECRET must be at least 32 bytes")
        if not HASH.fullmatch(values["ADMIN_PASSWORD_HASH"]):
            raise CredentialFormatError("ADMIN_PASSWORD_HASH must be a complete scrypt hash")
        if "TURN_SHARED_SECRET" in values and len(values["TURN_SHARED_SECRET"].encode()) < 32:
            raise CredentialFormatError("TURN_SHARED_SECRET must be at least 32 bytes")
        if not EMAIL.fullmatch(values.get("ACME_EMAIL", "")):
            print("Credential file requires a valid ACME_EMAIL", file=sys.stderr)
            return 1
        enable_turn = values.get("ENABLE_TURN", "false")
        if enable_turn not in {"true", "false"}:
            print("Credential file ENABLE_TURN must be true or false", file=sys.stderr)
            return 1
        missing_turn = TURN_REQUIRED - values.keys() if enable_turn == "true" else set()
        if missing_turn:
            raise CredentialFormatError(f"missing TURN settings: {', '.join(sorted(missing_turn))}")
    except CredentialFormatError as exc:
        print(f"Credential file rejected: {exc}", file=sys.stderr)
        return 1
    except (OSError, UnicodeError, ValueError, IndexError) as exc:
        print(f"Credential file rejected: {exc.__class__.__name__}", file=sys.stderr)
        return 1
    print(values["DOMAIN"], enable_turn)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
