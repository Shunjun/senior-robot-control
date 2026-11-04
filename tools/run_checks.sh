#!/bin/sh
set -eu

ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
PROFILE=${1:-"$ROOT/config/simulation.yaml"}

cd "$ROOT"
if [ -x .venv/bin/python ]; then
  PYTHON=.venv/bin/python
else
  PYTHON="uv run python"
fi

# shellcheck disable=SC2086
$PYTHON tools/doctor.py "$PROFILE"
# shellcheck disable=SC2086
$PYTHON -m pytest tests/integration/test_deployment_config.py -q
