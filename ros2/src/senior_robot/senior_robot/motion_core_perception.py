"""Measured and explicitly contracted NO_RETURN patches; unknown remains unknown."""

import math
from dataclasses import replace

from shapely.geometry import LineString, Polygon

from senior_control.motion.constraints import FreePatch, NoReturnPolicy, PatchKind
from senior_control.safety.self_filter import FilteredObservation, RayEvidence


def _supported_rays(
    observation: FilteredObservation, policy: NoReturnPolicy
) -> tuple[RayEvidence, ...]:
    """Keep original hits; add separately labelled ideal-sensor no-return rays."""
    if policy is NoReturnPolicy.UNKNOWN:
        return observation.free_rays
    scan = observation.scan
    masks = tuple(Polygon(vertices) for vertices in observation.self_masks)
    result = []
    for ray in observation.rays:
        if ray.kind in {"hit", "clear"}:
            result.append(ray)
        elif ray.kind == "invalid" and scan.ranges[ray.index] == math.inf:
            angle = scan.sensor_to_body.yaw + scan.angle_min + ray.index * scan.angle_increment
            endpoint = (
                ray.origin[0] + scan.range_max * math.cos(angle),
                ray.origin[1] + scan.range_max * math.sin(angle),
            )
            if any(mask.intersects(LineString((ray.origin, endpoint))) for mask in masks):
                continue
            # This endpoint bounds free space; it is never a finite hit/obstacle.
            result.append(RayEvidence(ray.index, ray.origin, endpoint, "clear"))
    return tuple(result)


def scan_free_patches(
    observation: FilteredObservation, *, no_return_policy: NoReturnPolicy = NoReturnPolicy.UNKNOWN
) -> tuple[FreePatch, ...]:
    """Merge overlapping adjacent-ray windows BEFORE sensor-error erosion.

    Eroding individual one-beam triangles would erase all near-body free space.
    A window uses its minimum measured range; its near chord stays OUTSIDE
    range_min. Windows never span missing/self/unknown rays or large angular gaps.
    This retains the explicit calibrated interpolation assumption of scan ingress.
    """
    step = abs(observation.scan.angle_increment)
    if not observation.scan.interpolate or not 0 < step <= min(
        0.01, observation.scan.max_angular_gap
    ):
        return ()
    width = max(2, math.ceil(0.4 / step))
    stride = max(1, width // 4)
    runs: list[list[RayEvidence]] = []
    for ray in _supported_rays(observation, no_return_policy):
        if ray.endpoint is None:
            continue
        if not runs or ray.index != runs[-1][-1].index + 1:
            runs.append([])
        runs[-1].append(ray)
    # A complete revolution has a physical seam, not an unknown wedge.
    # Join only supported rays on both sides of that seam; invalid/self gaps
    # elsewhere remain separate unknown gaps.
    count = len(observation.scan.ranges)
    inclusive = abs((count - 1) * step - 2 * math.pi) <= step / 2
    periodic = abs(count * step - 2 * math.pi) <= step / 2
    if (
        runs
        and (inclusive or periodic)
        and runs[0][0].index == 0
        and runs[-1][-1].index == count - 1
    ):
        period = count - 1 if inclusive else count
        start = 1 if inclusive else 0
        if len(runs) == 1:
            extension = runs[0][start : start + width]
            runs[0].extend(replace(ray, index=ray.index + period) for ray in extension)
        else:
            # Join only the measured seam. A self/invalid gap elsewhere remains
            # a separate run and is never bridged by this operation.
            joined = runs[-1] + [replace(ray, index=ray.index + period) for ray in runs[0][start:]]
            runs = [*runs[1:-1], joined]
    result = []
    for run in runs:
        if len(run) < 2:
            continue
        starts = list(range(0, max(1, len(run) - width + 1), stride))
        starts.append(max(0, len(run) - width))
        for start in sorted(set(starts)):
            rays = run[start : start + width]
            a, b = rays[0], rays[-1]
            if a.endpoint is None or b.endpoint is None:
                continue
            half = (b.index - a.index) * step / 2
            if not 0 < half < math.pi / 2:
                continue
            ox, oy = a.origin
            distances = [
                math.hypot(r.endpoint[0] - ox, r.endpoint[1] - oy)
                for r in rays
                if r.endpoint is not None
            ]
            far = min(distances)
            near = observation.scan.range_min / math.cos(half)
            if far <= near:
                continue
            ra, rb = distances[0], distances[-1]
            ua = ((a.endpoint[0] - ox) / ra, (a.endpoint[1] - oy) / ra)
            ub = ((b.endpoint[0] - ox) / rb, (b.endpoint[1] - oy) / rb)
            vertices = (
                (ox + ua[0] * near, oy + ua[1] * near),
                (ox + ua[0] * far, oy + ua[1] * far),
                (ox + ub[0] * far, oy + ub[1] * far),
                (ox + ub[0] * near, oy + ub[1] * near),
            )
            kind = (
                PatchKind.NO_RETURN
                if any(r.kind == "clear" for r in rays)
                else PatchKind.OBSERVED_FREE
            )
            result.append(FreePatch(vertices, kind))
    return tuple(result)
