"""Conservative fast-path bounds; uncertainty always returns to the geometry library."""

import math
import sys
from collections.abc import Sequence

from shapely.geometry.base import BaseGeometry

Point2D = tuple[float, float]
Bounds = tuple[float, float, float, float]
_MAX_SAFE_COORDINATE = math.sqrt(sys.float_info.max) / 16.0


def _safe_bounds(bounds: Sequence[float]) -> Bounds | None:
    if len(bounds) != 4 or not all(map(math.isfinite, bounds)):
        return None
    xmin, ymin, xmax, ymax = bounds
    if max(abs(xmin), abs(ymin), abs(xmax), abs(ymax)) > _MAX_SAFE_COORDINATE:
        return None
    if xmin > xmax or ymin > ymax:
        return None
    return xmin, ymin, xmax, ymax


def _well_conditioned(bounds: Bounds, area: float) -> bool:
    if not math.isfinite(area) or area <= 0.0:
        return False
    xmin, ymin, xmax, ymax = bounds
    scale = max(1.0, abs(xmin), abs(ymin), abs(xmax), abs(ymax))
    span = max(1.0, xmax - xmin, ymax - ymin)
    # This ULP guard chooses only the existing GEOS slow path. It is not an
    # area tolerance, coverage criterion, or permission to ignore geometry.
    return area > 8.0 * math.ulp(scale) * span


def mask_bounds(mask: BaseGeometry) -> Bounds | None:
    bounds = _safe_bounds(mask.bounds)
    return bounds if bounds is not None and _well_conditioned(bounds, mask.area) else None


def triangle_bounds(points: tuple[Point2D, Point2D, Point2D], area: float) -> Bounds | None:
    (ax, ay), (bx, by), (cx, cy) = points
    values = (ax, ay, bx, by, cx, cy)
    if not all(map(math.isfinite, values)) or max(map(abs, values)) > _MAX_SAFE_COORDINATE:
        return None
    bounds = (min(ax, bx, cx), min(ay, by, cy), max(ax, bx, cx), max(ay, by, cy))
    return bounds if _well_conditioned(bounds, area) else None


def segment_bounds(origin: Point2D, end: Point2D) -> Bounds | None:
    x0, y0 = origin
    x1, y1 = end
    values = (x0, y0, x1, y1)
    if not all(map(math.isfinite, values)) or max(map(abs, values)) > _MAX_SAFE_COORDINATE:
        return None
    bounds = (min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1))
    scale = max(1.0, abs(x0), abs(y0), abs(x1), abs(y1))
    if max(bounds[2] - bounds[0], bounds[3] - bounds[1]) <= 8.0 * math.ulp(scale):
        return None
    return bounds


def strictly_disjoint(first: Bounds | None, second: Bounds | None) -> bool:
    if first is None or second is None:
        return False
    # Strict inequalities retain all touching and boundary cases for GEOS.
    return (
        first[2] < second[0] or second[2] < first[0] or first[3] < second[1] or second[3] < first[1]
    )
