"""External-only matcher features; this does not classify or clear obstacles.

The start owner publishes the EXISTING physical polygon once per run. Raw scan
obstacle consumers retain every return; matcher candidates omit body-interior
and boundary-uncertain points so those cannot certify independent translation.
"""

import json
import os
from functools import lru_cache
from pathlib import Path

from shapely.geometry import Point, box


def boundary_path(claim: Path) -> Path:
    return Path(str(claim) + ".features.json")


def write_feature_boundary(claim: Path, vertices, error_m: float) -> None:
    path = boundary_path(claim)
    pending = Path(str(path) + ".pending")
    try:
        fd = os.open(pending, os.O_CREAT | os.O_WRONLY | os.O_TRUNC, 0o600)
        with os.fdopen(fd, "w") as file:
            json.dump(dict(physical_polygon=vertices, error_m=error_m), file)
        os.replace(pending, path)
    except OSError as exc:
        raise ValueError("physical_boundary_publish_failed") from exc


@lru_cache(maxsize=4)
def _boundary(path: str, revision: int):
    data = json.loads(Path(path).read_text())
    from senior_control.safety.self_learning import SelfReturnModel

    validated = SelfReturnModel(data["physical_polygon"], error_m=data["error_m"])
    return validated.body, validated.error


def external_feature_ranges(observation, claim: Path):
    path = boundary_path(claim)
    try:
        stat = path.stat()
        if stat.st_size > 16384:
            raise ValueError("physical_boundary_size")
        body, error = _boundary(str(path), stat.st_mtime_ns)
    except (OSError, KeyError, TypeError) as exc:
        raise ValueError("awaiting_start_physical_boundary") from exc
    ranges = list(observation.scan.ranges)
    left, bottom, right, top = body.bounds
    for ray in observation.rays:
        if ray.endpoint is None:
            continue
        x, y = ray.endpoint
        if x + error < left or x - error > right or y + error < bottom or y - error > top:
            continue
        support = Point(x, y) if error == 0 else box(x - error, y - error, x + error, y + error)
        if not body.disjoint(support):
            ranges[ray.index] = float("nan")
    return tuple(ranges)
