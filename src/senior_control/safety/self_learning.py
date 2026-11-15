"""Endpoint-local self-return evidence. Physical containment is a startup prior only."""

import math
from bisect import bisect_left
from dataclasses import dataclass, replace
from uuid import uuid4

from shapely.geometry import MultiPoint, Point, Polygon

from senior_control.models import Pose2D
from senior_control.safety.self_filter import FilteredObservation, Point2D


@dataclass(frozen=True)
class SelfReturnRevocation:
    model_id: str
    base_revision: int
    baseline: tuple[tuple[float, float], ...]
    removed: tuple[tuple[float, float], ...]


class SelfReturnModel:
    def __init__(self, vertices, *, error_m: float):
        self.body = Polygon(vertices)
        if (
            not self.body.is_valid
            or self.body.area <= 0
            or not math.isfinite(error_m)
            or error_m < 0
        ):
            raise ValueError("invalid self-return boundary or error")
        self.error = error_m
        self.bounds = self.body.bounds
        self.frames: list[tuple[Pose2D, tuple[tuple[float, float], ...]]] = []
        self.frame_stamps: list[float] = []
        self.frame_errors: list[float] = []
        self.frame_yaw_errors: list[float] = []
        self.frame_observations = []
        self.learned: tuple[tuple[float, float], ...] = ()
        self.revision = 0
        self.model_id = uuid4().hex
        self.first_observation = None

    def interior(self, xy: Point2D) -> bool:
        # A square encloses the circular error support without polygonal buffer
        # under-approximation. Touching the physical boundary remains ambiguous.
        from shapely.geometry import box

        x, y = xy
        left, bottom, right, top = self.bounds
        if not (
            left + self.error < x < right - self.error
            and bottom + self.error < y < top - self.error
        ):
            return False
        support = (
            Point(x, y)
            if self.error == 0
            else box(x - self.error, y - self.error, x + self.error, y + self.error)
        )
        return self.body.contains(support) and self.body.boundary.disjoint(support)

    def clear(self):
        self.learned = ()
        self.frames.clear()
        self.frame_stamps.clear()
        self.frame_errors.clear()
        self.frame_yaw_errors.clear()
        self.frame_observations.clear()
        self.first_observation = None
        self.revision += 1

    def observe(
        self,
        observation: FilteredObservation,
        pose: Pose2D,
        *,
        force=False,
        pose_error_m=0.0,
        pose_yaw_error_rad=0.0,
    ):
        points = tuple(
            r.endpoint for r in observation.rays if r.kind == "hit" and r.endpoint is not None
        )
        if len(points) > 2048:
            raise ValueError("self-return point budget exceeded")
        stamp = observation.scan.stamp
        i = bisect_left(self.frame_stamps, stamp)
        if i < len(self.frame_stamps) and self.frame_stamps[i] == stamp:
            return
        if len(self.frames) >= 5 and not force:
            if any(math.hypot(pose.x - p.x, pose.y - p.y) < 0.005 for p, _ in self.frames):
                return
        if len(self.frames) < 64:
            self.frames.insert(i, (pose, points))
            self.frame_stamps.insert(i, stamp)
            self.frame_errors.insert(i, pose_error_m)
            self.frame_yaw_errors.insert(i, pose_yaw_error_rad)
            self.frame_observations.insert(i, observation)
            self.first_observation = self.frame_observations[0]
            return True
        return False

    def retain_recent(self, count: int):
        for values in (
            self.frames,
            self.frame_stamps,
            self.frame_errors,
            self.frame_yaw_errors,
            self.frame_observations,
        ):
            del values[:-count]
        self.first_observation = self.frame_observations[0] if self.frames else None

    def validate(self, *, independent_motion: bool, motion_error_m=0.0) -> int:
        self.learned = ()
        if not independent_motion or len(self.frames) < 10:
            return 0
        first, last = self.frames[0][0], self.frames[-1][0]
        baseline = math.hypot(last.x - first.x, last.y - first.y)
        tolerance = max(2 * self.error, 0.003)
        if baseline < 0.08:
            return 0
        if abs(math.atan2(math.sin(last.yaw - first.yaw), math.cos(last.yaw - first.yaw))) > 0.05:
            return 0
        clouds = [(p, MultiPoint(points)) for p, points in self.frames]
        selected = []
        candidates = self.frames[-1][1]
        # Bounded classification work; unselected returns remain external.
        stride = max(1, math.ceil(len(candidates) / 128))
        for point in candidates[::stride]:
            if not self.interior(point):
                continue
            if not all(
                not cloud.is_empty and cloud.distance(Point(point)) <= tolerance
                for _, cloud in clouds
            ):
                continue
            # World-fixed predicts a different body point at the first frame.
            # Compare against ALL returns, so repeated surfaces cannot pass just
            # because a beam index stayed constant. World-compatible => ambiguous.
            c, s = math.cos(last.yaw), math.sin(last.yaw)
            wx = last.x + c * point[0] - s * point[1]
            wy = last.y + s * point[0] + c * point[1]
            c, s = math.cos(first.yaw), math.sin(first.yaw)
            predicted = Point(
                c * (wx - first.x) + s * (wy - first.y), -s * (wx - first.x) + c * (wy - first.y)
            )
            # The exact same complete transformed support drives BOTH tests.
            # Rotation uncertainty acts at the point's lever arm in each frame.
            world_error = (
                tolerance
                + motion_error_m
                + self.frame_errors[0]
                + self.frame_errors[-1]
                + 2
                * math.hypot(predicted.x, predicted.y)
                * math.sin(min(math.pi, self.frame_yaw_errors[0]) / 2)
                + 2 * math.hypot(*point) * math.sin(min(math.pi, self.frame_yaw_errors[-1]) / 2)
            )
            if clouds[0][1].distance(predicted) > world_error and self._observed_free(
                predicted, world_error
            ):
                selected.append(point)
        self.learned = tuple(selected)
        self.revision += 1
        return len(self.learned)

    def _observed_free(self, point, error):
        """Absence of a match behind an occluder is not world-model rejection."""
        from shapely.geometry import box

        observation = self.first_observation
        if observation is None:
            return False
        scan = observation.scan
        step = scan.angle_increment
        if not scan.interpolate or step > scan.max_angular_gap or step > 0.01:
            return False
        ox, oy = scan.sensor_to_body.x, scan.sensor_to_body.y
        distance = math.hypot(point.x - ox, point.y - oy)
        if distance <= scan.range_min + error:
            return False
        bearing = math.atan2(point.y - oy, point.x - ox) - scan.sensor_to_body.yaw
        center = (bearing - scan.angle_min) / step
        span = math.asin(min(1.0, math.sqrt(2) * error / distance)) / step
        low, high = math.floor(center - span) - 1, math.ceil(center + span) + 1
        if low < 0 or high >= len(observation.rays) or high - low > 256:
            return False
        rays = observation.rays[low : high + 1]
        if any(r.kind != "hit" or r.endpoint is None for r in rays):
            return False
        half = (high - low) * step / 2
        if not 0 < half < math.pi / 2:
            return False
        far = min(math.dist((ox, oy), r.endpoint) for r in rays) - self.error
        near = scan.range_min / math.cos(half)
        angles = [scan.sensor_to_body.yaw + scan.angle_min + i * step for i in (low, high)]
        a, b = [(math.cos(t), math.sin(t)) for t in angles]
        patch = Polygon(
            (
                (ox + a[0] * near, oy + a[1] * near),
                (ox + a[0] * far, oy + a[1] * far),
                (ox + b[0] * far, oy + b[1] * far),
                (ox + b[0] * near, oy + b[1] * near),
            )
        )
        return far > near and patch.covers(
            box(point.x - error, point.y - error, point.x + error, point.y + error)
        )

    def apply_revocation(self, update: SelfReturnRevocation) -> bool:
        """Only subtract independently verified members, never import copied model state."""
        if (
            update.model_id != self.model_id
            or type(update.base_revision) is not int
            or update.base_revision < 0
            or update.base_revision > self.revision
            or not set(update.removed).issubset(update.baseline)
            or len(set(update.removed)) != len(update.removed)
            or update.base_revision == self.revision
            and update.baseline != self.learned
        ):
            raise ValueError("invalid_self_return_revocation")
        retained = tuple(point for point in self.learned if point not in update.removed)
        if retained == self.learned:
            return False
        self.learned = retained
        self.revision += 1
        return True

    def filter_snapshot(
        self, observation: FilteredObservation, *, provisional: bool
    ) -> tuple[FilteredObservation, SelfReturnRevocation]:
        """Pure observation check; return explicit monotonic deletions for the model owner."""
        hits = [r.endpoint for r in observation.rays if r.kind == "hit" and r.endpoint is not None]
        tolerance = max(2 * self.error, 0.003)
        retained = tuple(p for p in self.learned if any(math.dist(p, h) <= tolerance for h in hits))
        update = SelfReturnRevocation(
            self.model_id,
            self.revision,
            self.learned,
            tuple(point for point in self.learned if point not in retained),
        )
        rays = tuple(
            replace(r, kind="self")
            if r.kind == "hit"
            and r.endpoint is not None
            and self.interior(r.endpoint)
            and (provisional or any(math.dist(r.endpoint, p) <= tolerance for p in retained))
            else r
            for r in observation.rays
        )
        filtered = replace(
            observation,
            rays=rays,
            external_hits=tuple(r for r in rays if r.kind == "hit"),
            free_rays=tuple(r for r in rays if r.kind in {"hit", "clear"}),
            occluded_regions=tuple(r for r in rays if r.kind == "self"),
            # No invented mask polygon: blocked beams themselves preserve unknown.
            self_masks=(),
        )
        return filtered, update

    def filter(self, observation: FilteredObservation, *, provisional: bool) -> FilteredObservation:
        filtered, update = self.filter_snapshot(observation, provisional=provisional)
        self.apply_revocation(update)
        return filtered
