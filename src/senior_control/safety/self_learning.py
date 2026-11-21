"""Discrete body-connected learning and immutable 1cm position filtering."""

import math
from bisect import bisect_left
from collections.abc import Sequence
from dataclasses import replace

from shapely.geometry import MultiPoint, Point, Polygon
from shapely.strtree import STRtree

from senior_control.motion.learning_record import MAX_ANCHORS
from senior_control.safety.self_filter import FilteredObservation, Point2D
from senior_control.vision.raw_capture import RawCaptureHeader

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
        self.frames: list[tuple[tuple[float, float], ...]] = []
        self.frame_measurements: list[int] = []
        self.frame_keys: list[tuple[str, str, int]] = []
        self.segment_id: str | None = None
        self._capture_origin: tuple[str, str] | None = None
        self._capture_floor: tuple[int, int] | None = None
        self._max_sequence = 0
        self._frontier_host = 0.0
        self._frontier_measurement: int | None = None
        self._clock_generation = 0
        self._candidates: tuple[Point2D, ...] | None = None
        self._observed_count = 0
        self.learned: tuple[tuple[float, float], ...] = ()
        self._filter_points: tuple[Point2D, ...] = ()
        self._filter_tree: STRtree | None = None

    def interior(self, xy: Point2D) -> bool:
        # Boundary points remain ambiguous; no sensor/extrinsic margin applies.
        x, y = xy
        left, bottom, right, top = self.bounds
        return left < x < right and bottom < y < top and self.body.contains(Point(x, y))

    def clear(self) -> None:
        self.learned = ()
        self.frames.clear()
        self.frame_measurements.clear()
        self.frame_keys.clear()
        self.segment_id = None
        self._capture_origin = None
        self._capture_floor = None
        self._max_sequence = 0
        self._frontier_host = 0.0
        self._frontier_measurement = None
        self._candidates = None
        self._observed_count = 0

    def observe(
        self,
        observation: FilteredObservation,
        *,
        capture: RawCaptureHeader,
        segment_id: str,
    ) -> bool:
        points = tuple(
            r.endpoint for r in observation.rays if r.kind == "hit" and r.endpoint is not None
        )
        if len(points) > MAX_ANCHORS:
            raise ValueError("self-return point budget exceeded")
        if self.segment_id is None:
            self.segment_id = segment_id
        if self.segment_id != segment_id:
            return False  # Caller must explicitly establish a new evidence segment.
        origin = capture.bridge_boot_id, capture.source_epoch
        generation = capture.time.clock_generation
        if self._capture_origin is None:
            self._capture_origin = origin
        if origin != self._capture_origin:
            return False
        key = (*origin, capture.sequence)
        measurement = capture.time.stamp_ns
        frontier = self._frontier_measurement
        if measurement in self.frame_measurements or (
            self._capture_floor is not None and measurement <= self._capture_floor[0]
        ):
            return False  # A new delivery sequence cannot create another raw frame.
        if frontier is not None and capture.time.clock_generation != self._clock_generation:
            return False  # Caller must open a new evidence segment on clock rollback.
        self._clock_generation = generation
        if key in self.frame_keys or (
            self._capture_floor is not None
            and (measurement, capture.sequence) < (self._capture_floor[0], self._capture_floor[1])
        ):
            return False
        if capture.sequence > self._max_sequence:
            if capture.end_steady < self._frontier_host or (
                frontier is not None and ((measurement - frontier) / 1e9) < 0
            ):
                raise ValueError("learning_capture_clock_regressed")
            self._max_sequence = capture.sequence
            self._frontier_host, self._frontier_measurement = capture.end_steady, measurement
        elif capture.end_steady > self._frontier_host or (
            frontier is not None and ((measurement - frontier) / 1e9) > 0
        ):
            raise ValueError("learning_capture_sequence_time_mismatch")
        i = bisect_left(
            [
                (time, identity[2])
                for time, identity in zip(self.frame_measurements, self.frame_keys, strict=True)
            ],
            (measurement, capture.sequence),
        )
        connected = self.connected_points(observation)
        if self._candidates is None:
            self._candidates = tuple(dict.fromkeys(connected))
        else:
            cloud = MultiPoint(connected)
            self._candidates = tuple(
                p
                for p in self._candidates
                if not cloud.is_empty and cloud.distance(Point(p)) <= _match_limit(p)
            )
            # A removed unstable bridge is a gap in the STABLE surface. Prune
            # disconnected pieces too, without inventing distance/angle links.
            while self._candidates:
                stable = self._candidates
                tree = STRtree(tuple(Point(p) for p in stable))
                supported = replace(
                    observation,
                    rays=tuple(
                        ray
                        if ray.endpoint is not None and _matches(ray.endpoint, stable, tree)
                        else replace(ray, kind="invalid")
                        for ray in observation.rays
                    ),
                )
                connected_cloud = MultiPoint(self.connected_points(supported))
                self._candidates = tuple(
                    p
                    for p in stable
                    if not connected_cloud.is_empty
                    and connected_cloud.distance(Point(p)) <= _match_limit(p)
                )
                if self._candidates == stable:
                    break
        self._observed_count += 1
        self.frames.insert(i, points)
        self.frame_measurements.insert(i, measurement)
        self.frame_keys.insert(i, key)
        # The fixed candidate intersection retains evidence from EVERY frame,
        # including frames whose capture metadata leaves the bounded window.
        self.retain_recent(64)
        return True

    def retain_recent(self, count: int) -> None:
        trimmed = len(self.frames) > count
        for values in (
            self.frames,
            self.frame_measurements,
            self.frame_keys,
        ):
            del values[:-count]
        if trimmed and self.frame_measurements:
            self._capture_floor = self.frame_measurements[0], self.frame_keys[0][2]

    def begin_segment(self, segment_id: str) -> None:
        if self.segment_id != segment_id:
            self.clear()
            self.segment_id = segment_id

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
        self.learned = tuple(
            dict.fromkeys(
                self.connected_points(
                    supported, max_connection_gap=2 * SELF_RETURN_MATCH_DISTANCE_M
                )
            )
        )
        return len(self.learned)

    def connected_points(
        self, observation: FilteredObservation, *, max_connection_gap: float | None = None
    ) -> tuple[Point2D, ...]:
        """Keep runs seeded inside the body, optionally separating spatial gaps.

        Streaming compatibility retains its historical discrete topology.
        Sampled learning also checks endpoint separation. Neither fills free space.
        """
        runs: list[list[Point2D]] = []
        run: list[Point2D] = []
        previous: int | None = None

        def connected(first: Point2D, second: Point2D) -> bool:
            if max_connection_gap is None:
                return True  # Compatibility for the historical streaming-model API.
            origin = (observation.scan.sensor_to_body.x, observation.scan.sensor_to_body.y)
            radius = min(math.dist(origin, first), math.dist(origin, second))
            spacing = 2 * radius * math.sin(observation.scan.angle_increment / 2)
            return math.dist(first, second) <= spacing + max_connection_gap

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

    def validate(self) -> int:
        """Freeze body-stable returns connected to an interior return in every frame."""
        self.learned = ()
        if self._observed_count < 10:
            return 0
        self.learned = self._candidates or ()
        return len(self.learned)

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
