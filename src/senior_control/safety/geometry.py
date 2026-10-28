"""Conservative footprint sweeps for hard-boundary checks."""

import math
from collections.abc import Sequence

from shapely.affinity import rotate, translate
from shapely.geometry import Polygon
from shapely.geometry.base import BaseGeometry
from shapely.ops import unary_union


def _valid_body(footprint: Sequence[tuple[float, float]]) -> Polygon:
    body = Polygon(footprint)
    if not body.is_valid or body.is_empty or body.area <= 0:
        raise ValueError("invalid footprint sweep")
    return body


def _inflate(union: BaseGeometry, gap: float, margin: float) -> BaseGeometry:
    inflate = gap + margin
    if inflate > 0.0:
        # Mitre joins stay polygonal and still cover the round neighborhood.
        return union.buffer(inflate, join_style="mitre", mitre_limit=8.0)
    return union


def swept_footprint(
    footprint: Sequence[tuple[float, float]],
    *,
    yaw: float,
    yaw_step: float,
    margin: float = 0.0,
) -> BaseGeometry:
    """Sweep a body about the origin and inflate past the corner arc.

    Endpoint samples leave a chord in the gap between two rectangles. The
    uncovered distance grows linearly with the step, so the arc-to-chord
    sagitta is not enough. ``2 r sin(step / 4)`` is the distance from a
    mid-arc point to the nearer sample and covers that gap. ``margin`` is
    obstacle-position uncertainty, not a braking distance.
    """
    if (
        not math.isfinite(yaw)
        or not math.isfinite(yaw_step)
        or not math.isfinite(margin)
        or yaw_step <= 0.0
        or margin < 0.0
    ):
        raise ValueError("invalid footprint sweep")
    body = _valid_body(footprint)
    steps = max(1, math.ceil(abs(yaw) / yaw_step - 1e-12))
    actual_step = yaw / steps
    samples: list[BaseGeometry] = [body]
    for index in range(1, steps + 1):
        samples.append(rotate(body, actual_step * index, origin=(0.0, 0.0), use_radians=True))
    radius = max(math.hypot(point[0], point[1]) for point in body.exterior.coords)
    gap = 2.0 * radius * math.sin(abs(actual_step) / 4.0)
    return _inflate(unary_union(samples), gap, margin)


def swept_trajectory(
    footprint: Sequence[tuple[float, float]],
    *,
    distance: float,
    yaw: float,
    step: float,
    margin: float = 0.0,
) -> BaseGeometry:
    """Sweep the body along a constant-curvature arc of signed distance and yaw.

    The final sample is the footprint at the stopping pose. Inflation covers
    the corner arc between samples about the instantaneous center. This is a
    conservative envelope, not a measured braking curve.
    """
    if (
        not math.isfinite(distance)
        or not math.isfinite(yaw)
        or not math.isfinite(step)
        or not math.isfinite(margin)
        or step <= 0.0
        or margin < 0.0
        or abs(yaw) <= 1e-12
    ):
        raise ValueError("invalid trajectory sweep")
    body = _valid_body(footprint)
    path_radius = distance / yaw
    steps = max(1, math.ceil(abs(yaw) / step - 1e-12))
    actual_step = yaw / steps
    samples: list[BaseGeometry] = []
    for index in range(steps + 1):
        psi = actual_step * index
        samples.append(
            translate(
                rotate(body, psi, origin=(0.0, 0.0), use_radians=True),
                xoff=path_radius * math.sin(psi),
                yoff=path_radius * (1.0 - math.cos(psi)),
            )
        )
    reach = max(math.hypot(point[0], point[1] - path_radius) for point in body.exterior.coords)
    gap = 2.0 * reach * math.sin(abs(actual_step) / 4.0)
    return _inflate(unary_union(samples), gap, margin)
