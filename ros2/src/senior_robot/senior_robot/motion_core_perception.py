"""Measured and explicitly contracted NO_RETURN patches; unknown remains unknown."""

import math
from dataclasses import dataclass, replace

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


def prepare_scan_geometry(filtered, sensor_error, policy, metadata):
    """Pure heavy geometry: no mutable learning model crosses this boundary."""
    from senior_control.motion.constraints import prepare_perception

    patches = scan_free_patches(filtered, no_return_policy=policy)
    hits = tuple(ray.endpoint for ray in filtered.external_hits if ray.endpoint is not None)
    return prepare_perception(
        patches,
        hits,
        error_m=sensor_error,
        no_return_policy=policy,
        max_patches=2048,
        max_vertices=8192,
        max_hits=2048,
        **metadata,
    )


def fast_scan_guard(observation, policy, *, error_m):
    """One radial visibility polygon, one hole and batched occupied squares.

    This only tightens a completed slow result. Neighbour minima keep the
    boundary inside the same adjacent-ray interpolation contract as slow patches;
    unknown/self rays collapse to the near boundary, never become free space.
    No per-window buffer/union or directional candidate search is done here.
    """
    from shapely import box as batch_box
    from shapely.geometry import GeometryCollection

    scan = observation.scan
    count, step = len(observation.rays), abs(scan.angle_increment)
    if (
        count > 2048
        or count < 2
        or not scan.interpolate
        or not 0 < step <= min(0.01, scan.max_angular_gap)
    ):
        return GeometryCollection(), GeometryCollection()
    if not math.isfinite(error_m) or error_m < 0:
        raise ValueError("invalid_fast_scan_error")
    span = (count - 1) * step
    if span > 2 * math.pi + step / 2:
        raise ValueError("invalid_fast_scan_span")
    periodic = abs(count * step - 2 * math.pi) <= step / 2
    inclusive = abs(span - 2 * math.pi) <= step / 2
    rays = observation.rays
    count = len(rays)
    full = periodic or inclusive
    near = scan.range_min / math.cos(step)
    origin = (scan.sensor_to_body.x, scan.sensor_to_body.y)
    ranges, hits = [], []
    for ray in rays:
        if ray.kind == "hit" and ray.endpoint is not None:
            ranges.append(max(near, math.dist(origin, ray.endpoint)))
            hits.append(ray.endpoint)
        elif ray.kind == "clear" or (
            ray.kind == "invalid"
            and scan.ranges[ray.index] == math.inf
            and policy is NoReturnPolicy.VALIDATED_FREE_TO_RANGE
        ):
            ranges.append(scan.range_max)
        else:
            ranges.append(near)
    if inclusive:
        ranges[0] = min(ranges[0], ranges[-1])
        ranges.pop()
        count -= 1
    outer, inner = [], []
    for i in range(count):
        adjacent = [ranges[i]]
        for j in (i - 1, i + 1):
            if full or 0 <= j < count:
                adjacent.append(ranges[j % count])
        far = max(near, min(adjacent))
        angle = scan.sensor_to_body.yaw + scan.angle_min + i * scan.angle_increment
        direction = math.cos(angle), math.sin(angle)
        outer.append((origin[0] + far * direction[0], origin[1] + far * direction[1]))
        inner.append((origin[0] + near * direction[0], origin[1] + near * direction[1]))
    if not full:
        outer = [origin, *outer, origin]
        inner = [origin, *inner, origin]
    outside = Polygon(outer)
    if not outside.is_valid:
        # Radially ordered rays cannot cross except at zero-radius unknown
        # sectors. Split those touching components without filling the gaps.
        from shapely import make_valid

        outside = make_valid(outside)
    inside = Polygon(inner) if near > 0 else GeometryCollection()
    if not inside.is_valid:
        raise ValueError("invalid_fast_scan_polygon")
    free = outside.difference(inside)
    if error_m:
        free = free.buffer(-error_m)
    if hits:
        x, y = zip(*hits, strict=True)
        occupied = (
            GeometryCollection(
                batch_box(
                    [v - error_m for v in x],
                    [v - error_m for v in y],
                    [v + error_m for v in x],
                    [v + error_m for v in y],
                )
            )
            if error_m
            else GeometryCollection()
        )
        if not error_m:
            from shapely.geometry import MultiPoint

            occupied = MultiPoint(hits)
    else:
        occupied = GeometryCollection()
    return free, occupied


@dataclass(frozen=True)
class ScanGeometryResult:
    key: tuple
    snapshot: object
    perception: object


class ScanGeometry:
    """One flight plus latest pending; sequencing cannot starve completed geometry.

    A scan sequence orders results, but source/frame/filter scope fences them.
    The caller MUST tighten returned geometry against its latest raw scan and
    retain the result's original receipt and deadline before granting output.
    """

    def __init__(self, worker=None):
        from senior_control.motion.process_resources import ProcessWorker

        self.worker = worker if worker is not None else ProcessWorker(name="motion-scan-geometry")
        self.flight = None
        self.flight_key = None
        self.flight_snapshot = None
        self.pending = None
        self.ready = None

    @staticmethod
    def scope(key):
        return key[0], *key[2:]

    def update(self, key, task, *, snapshot=None):
        self.pending = (key, task, snapshot)
        if self.flight is not None and self.flight.done():
            try:
                value = self.flight.result()
            except Exception:
                value = None
            if (
                value is not None
                and self.scope(self.flight_key) == self.scope(key)
                and self.flight_key[1] <= key[1]
            ):
                self.ready = ScanGeometryResult(self.flight_key, self.flight_snapshot, value)
            self.flight = None
        if self.ready is not None and (
            self.scope(self.ready.key) != self.scope(key) or self.ready.key[1] > key[1]
        ):
            self.ready = None
        if self.flight is None and (self.ready is None or self.ready.key != key):
            self.flight_key, task, self.flight_snapshot = self.pending
            self.pending = None
            self.flight = self.worker.submit(task)
        return self.ready

    def close(self):
        self.worker.close()
