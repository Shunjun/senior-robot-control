"""Immutable v1-v5 SQL history: compatibility data, never task or Home execution code."""

import json
from importlib.resources import files

from cloud.migrations import Migration


def historical_migrations() -> list[Migration]:
    rows = json.loads(files("cloud").joinpath("historical_schema.json").read_text())
    return [Migration(row["version"], tuple(row["statements"])) for row in rows]
