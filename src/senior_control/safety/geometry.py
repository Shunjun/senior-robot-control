"""Conservative footprint sweeps for hard-boundary checks."""

import math
from collections.abc import Sequence

from shapely.affinity import rotate
from shapely.geometry import Polygon
from shapely.geometry.base import BaseGeometry
from shapely.ops import unary_union


def swept_footprint(
    footprint: Sequence[tuple[float, float]],
    *,
    yaw: float,
    yaw_step: float,
    margin: float = 0.0,
) -> BaseGeometry:
    """Sweep a body about the origin and inflate by step sagitta plus margin.

    Endpoint samples alone miss a corner that crosses an obstacle between
    steps. The added radius is ``r * (1 - cos(step / 2))``, which covers that
    arc. ``margin`` is obstacle-position uncertainty, not a braking distance.
    """
    body = Polygon(footprint)
    if (
        not math.isfinite(yaw)
        or not math.isfinite(yaw_step)
        or not math.isfinite(margin)
        or yaw_step <= 0
        or margin < 0
        or not body.is_valid
        or body.is_empty
        or body.area <= 0
    ):
        raise ValueError("invalid footprint sweep")
    steps = max(1, math.ceil(abs(yaw) / yaw_step - 1e-12))
    actual_step = yaw / steps
    samples: list[BaseGeometry] = [body]
    for index in range(1, steps + 1):
        samples.append(rotate(body, actual_step * index, origin=(0.0, 0.0), use_radians=True))
    radius = max(math.hypot(point[0], point[1]) for point in body.exterior.coords)
    sagitta = radius * (1.0 - math.cos(abs(actual_step) / 2.0))
    union = unary_union(samples)
    inflate = margin + sagitta
    if inflate > 0:
        return union.buffer(inflate, quad_segs=16)
    return union
