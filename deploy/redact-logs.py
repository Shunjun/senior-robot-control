#!/usr/bin/env python3
"""Suppress credential-file values in deployment diagnostics."""

import sys
from pathlib import Path

secrets = []
for line in Path(sys.argv[1]).read_text().splitlines():
    line = line.strip()
    if not line or line.startswith("#") or "=" not in line:
        continue
    value = line.split("=", 1)[1].strip()
    for candidate in (value, value.strip("\"'")):
        if candidate:
            secrets.append(candidate)
for line in sys.stdin:
    for secret in sorted(secrets, key=len, reverse=True):
        line = line.replace(secret, "[redacted]")
    sys.stdout.write(line)
