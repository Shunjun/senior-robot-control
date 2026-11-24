#!/bin/sh
# Prepare the deployed Python version and this checkout's commit formatting hook.
set -eu

ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
cd "$ROOT"
uv sync --frozen --python 3.10 --group dev
uv run --no-sync lefthook install
