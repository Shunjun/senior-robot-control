"""Discrete body-connected learning and immutable 1cm position filtering."""

import math
from collections.abc import Sequence
from dataclasses import replace

from shapely.geometry import Point, Polygon
from shapely.strtree import STRtree

from senior_perception.learning_record import MAX_ANCHORS
from senior_perception.self_filter import FilteredObservation, Point2D

# User-specified body-return position matching tolerance; not sensor accuracy
# or a margin on the physical body / external obstacle geometry.
SELF_RETURN_MATCH_DISTANCE_M = 0.01
LEARNING_SAMPLE_COUNT = 5


def _match_limit(point: Point2D) -> float:
    # Only floating-point representation roundoff at the 1cm boundary, not an
    # additional physical tolerance. Nearby coordinates lie within these bounds.
    return SELF_RETURN_MATCH_DISTANCE_M + sum(
        math.ulp(value + offset)
        for value in point
        for offset in (-SELF_RETURN_MATCH_DISTANCE_M, SELF_RETURN_MATCH_DISTANCE_M)
    )


def _matches(point: Point2D, references: tuple[Point2D, ...], tree: STRtree) -> bool:
    reference = references[int(tree.nearest(Point(point)))]
    return math.dist(point, reference) <= _match_limit(reference)


class SelfReturnModel:
    def __init__(self, vertices: Sequence[Point2D]) -> None:
        self.body = Polygon(vertices)
        if not self.body.is_valid or self.body.area <= 0:
            raise ValueError("invalid self-return boundary")
        self.bounds = self.body.bounds
        self.learned: tuple[tuple[float, float], ...] = ()
        self._filter_points: tuple[Point2D, ...] = ()
        self._filter_tree: STRtree | None = None

    def interior(self, xy: Point2D) -> bool:
        # Boundary points remain ambiguous; no sensor/extrinsic margin applies.
        x, y = xy
        left, bottom, right, top = self.bounds
        return left < x < right and bottom < y < top and self.body.contains(Point(x, y))

    def learn_samples(self, observations: Sequence[FilteredObservation]) -> int:
        """Classify five motion samples once; no streaming candidate/fixpoint loop."""
        if len(observations) != LEARNING_SAMPLE_COUNT:
            raise ValueError("learning_sample_count")
        self.learned = ()
        reference = observations[0]
        candidates = tuple(
            dict.fromkeys(
                ray.endpoint
                for ray in reference.rays
                if ray.kind == "hit" and ray.endpoint is not None
            )
        )
        if len(candidates) > MAX_ANCHORS:
            raise ValueError("self-return point budget exceeded")
        for observation in observations[1:]:
            points = tuple(
                ray.endpoint
                for ray in observation.rays
                if ray.kind == "hit" and ray.endpoint is not None
            )
            if not points or not candidates:
                return 0
            tree = STRtree(tuple(Point(point) for point in points))
            candidates = tuple(p for p in candidates if _matches(p, points, tree))
        stable = set(candidates)
        supported = replace(
            reference,
            rays=tuple(
                ray if ray.endpoint in stable else replace(ray, kind="invalid")
                for ray in reference.rays
            ),
        )
        self.learned = tuple(dict.fromkeys(self.connected_points(supported)))
        return len(self.learned)

    def connected_points(self, observation: FilteredObservation) -> tuple[Point2D, ...]:
        """Keep spatially continuous stable runs seeded inside the body."""
        runs: list[list[Point2D]] = []
        run: list[Point2D] = []
        previous: int | None = None

        def connected(first: Point2D, second: Point2D) -> bool:
            origin = (observation.scan.sensor_to_body.x, observation.scan.sensor_to_body.y)
            radius = min(math.dist(origin, first), math.dist(origin, second))
            spacing = 2 * radius * math.sin(observation.scan.angle_increment / 2)
            return math.dist(first, second) <= spacing + 2 * SELF_RETURN_MATCH_DISTANCE_M

        for ray in observation.rays:
            if (
                ray.kind != "hit"
                or ray.endpoint is None
                or (previous is not None and ray.index != previous + 1)
                or (run and ray.endpoint is not None and not connected(run[-1], ray.endpoint))
            ):
                if run:
                    runs.append(run)
                run = []
            if ray.kind == "hit" and ray.endpoint is not None:
                run.append(ray.endpoint)
            previous = ray.index
        if run:
            runs.append(run)
        scan = observation.scan
        count = len(scan.ranges)
        step = scan.angle_increment
        span = (count - 1) * step
        periodic_span = count * step
        # LaserScan.angle_increment crosses the wire as float32. Propagate its
        # half-ulp quantization interval through BOTH endpoint conventions;
        # binary64 arithmetic roundoff alone cannot cover n accumulated errors.
        # The float32 subnormal spacing is 2**-149. angle_min cancels from the
        # relative span, so its wire quantization does not enlarge this budget.
        step_error = math.ldexp(1.0, max(-149, math.frexp(step)[1] - 24)) / 2
        arithmetic_error = math.ulp(span) + math.ulp(periodic_span) + math.ulp(math.tau)
        complete = (
            math.isfinite(span)
            and math.isfinite(periodic_span)
            and span <= math.tau + (count - 1) * step_error + arithmetic_error
            and periodic_span >= math.tau - count * step_error - arithmetic_error
        )
        if (
            len(runs) > 1
            and complete
            and len(observation.rays) == count
            and observation.rays[0].index == 0
            and observation.rays[-1].index == count - 1
            and observation.rays[0].kind == observation.rays[-1].kind == "hit"
            and observation.rays[0].endpoint is not None
            and observation.rays[-1].endpoint is not None
            and connected(runs[-1][-1], runs[0][0])
        ):
            runs = [runs[-1] + runs[0], *runs[1:-1]]
        return tuple(
            point for group in runs if any(self.interior(p) for p in group) for point in group
        )

    def filter(self, observation: FilteredObservation, *, provisional: bool) -> FilteredObservation:
        """Match frozen positions directly; scans never modify the learned catalog."""
        if self.learned != self._filter_points:
            self._filter_points = self.learned
            self._filter_tree = (
                STRtree(tuple(Point(p) for p in self.learned)) if self.learned else None
            )
        rays = tuple(
            replace(r, kind="self")
            if r.kind == "hit"
            and r.endpoint is not None
            and (
                (provisional and self.interior(r.endpoint))
                or (
                    self._filter_tree is not None
                    and _matches(r.endpoint, self.learned, self._filter_tree)
                )
            )
            else r
            for r in observation.rays
        )
        return replace(
            observation,
            rays=rays,
            external_hits=tuple(r for r in rays if r.kind == "hit"),
            free_rays=tuple(r for r in rays if r.kind in {"hit", "clear"}),
            occluded_regions=tuple(r for r in rays if r.kind == "self"),
            self_masks=(),
        )
