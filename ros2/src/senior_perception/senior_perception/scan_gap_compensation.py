"""Receiver-local, bounded real-surface support for short positive-infinity gaps."""

import math
from dataclasses import dataclass, replace

import numpy as np

from senior_perception.raw_safety_observation import PredictedReturn
from senior_perception.vision.raw_capture import RawCaptureHeader, capture_from_message
from vehicle_core.sample_time import SampleTime
from vehicle_core.sensor_policy import default_policy


@dataclass(frozen=True)
class _Seed:
    point: tuple[float, float]
    window: tuple[tuple[float, float], ...]
    capture: RawCaptureHeader
    angular_step: float
    missing: int = 0


def _pose_at(rows, stamp_ns):
    """Interpolate accepted native odometry only; never extrapolate motion."""
    previous = None
    for row in rows:
        timing = SampleTime(**row["time"])
        pose = (row["x"], row["y"], row["yaw"])
        if not all(math.isfinite(v) for v in pose):
            continue
        stamp = timing.stamp_ns
        if stamp == stamp_ns:
            return pose, row["epoch"]
        if previous is not None:
            first, first_time = previous
            if (
                first_time.stamp_ns < stamp_ns < stamp
                and first["epoch"] == row["epoch"]
                and first_time.clock_generation == timing.clock_generation
                and stamp - first_time.stamp_ns <= default_policy().hard_age_ms * 1_000_000
            ):
                fraction = (stamp_ns - first_time.stamp_ns) / (stamp - first_time.stamp_ns)
                yaw = first["yaw"] + fraction * math.remainder(row["yaw"] - first["yaw"], math.tau)
                return (
                    first["x"] + fraction * (row["x"] - first["x"]),
                    first["y"] + fraction * (row["y"] - first["y"]),
                    yaw,
                ), row["epoch"]
        previous = row, timing
    return None


def _rotation(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array(((c, -s), (s, c)))


def _line(points, error):
    if len(points) < 3:
        return None
    center = points.mean(axis=0)
    centered = points - center
    values, vectors = np.linalg.eigh(centered.T @ centered)
    if values[1] <= np.finfo(float).eps:
        return None
    normal = vectors[:, 0]
    if np.max(np.abs(centered @ normal)) > error:
        return None
    return center, normal


def _index(angle, start, step, count):
    relative = math.remainder(angle - start, math.tau)
    candidates = set()
    for turn in (-math.tau, 0.0, math.tau):
        coordinate = (relative + turn) / step
        index = round(coordinate)
        if 0 <= index < count and abs(index - coordinate) <= 0.5:
            candidates.add(index)
    return next(iter(candidates)) if len(candidates) == 1 else None


class ScanGapCompensator:
    """Real seeds expire after three new acquisitions; snapshots never extend them."""

    def __init__(self, profile):
        self.half_length, self.half_width = profile.length / 2, profile.width / 2
        self.error = profile.sensor_error
        self.clear()

    def clear(self):
        self._lineage = self._key = self._odom_epoch = None
        self._seeds = []
        self._result = None
        self._done = False

    def update(self, raw, poses, *, now):
        # Compensation is optional: malformed geometry/association never withdraws raw input.
        try:
            return self._update(raw, poses, now)
        except (
            ValueError,
            TypeError,
            KeyError,
            AttributeError,
            OverflowError,
            np.linalg.LinAlgError,
        ):
            self.clear()
            return raw

    def _update(self, raw, poses, now):
        capture = capture_from_message(raw.message.raw_capture)
        scan = raw.message
        geometry = (scan.angle_min, scan.angle_increment, scan.range_min, scan.range_max)
        if (
            not all(math.isfinite(v) for v in geometry)
            or scan.angle_increment == 0
            or scan.range_min < 0
            or scan.range_max <= scan.range_min
        ):
            self.clear()
            return raw
        lineage = (
            capture.bridge_boot_id,
            capture.bridge_gid,
            capture.raw_gid,
            capture.source_epoch,
            capture.capture_generation,
            capture.time.clock_generation,
            scan.frame_id,
            scan.sensor_x,
            scan.sensor_y,
            scan.sensor_yaw,
        )
        if lineage != self._lineage:
            self.clear()
            self._lineage = lineage
        key = capture.sequence, capture.stamp_ns
        if key != self._key:
            if self._key is not None and (key[0] <= self._key[0] or key[1] <= self._key[1]):
                self._seeds.clear()
            self._key, self._done, self._result = key, False, raw
            self._seeds = [replace(s, missing=s.missing + 1) for s in self._seeds if s.missing < 3]
        if self._done:
            return self._result
        age = default_policy().age(
            measured_steady=raw.measurement_steady, received_steady=raw.received, now_steady=now
        )
        if age.band == "invalid":
            return raw
        aligned = _pose_at(poses, capture.stamp_ns)
        if aligned is None:
            return raw  # The ordinary publish callback may retry this same acquisition.
        pose, epoch = aligned
        if epoch != self._odom_epoch:
            self._seeds.clear()
            self._odom_epoch = epoch
        body_rotation = _rotation(pose[2])
        origin = np.asarray(pose[:2]) + body_rotation @ (scan.sensor_x, scan.sensor_y)
        rotation = _rotation(pose[2] + scan.sensor_yaw)
        ranges = np.asarray(scan.ranges, dtype=float)
        count = len(ranges)
        angles = scan.angle_min + np.arange(count) * scan.angle_increment
        directions = np.column_stack((np.cos(angles), np.sin(angles)))
        valid = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= scan.range_max)
        # Avoid arithmetic on invalid ranges, which do not describe endpoints.
        points = directions * np.where(valid, ranges, 0)[:, None]
        body = points @ _rotation(scan.sensor_yaw).T + (scan.sensor_x, scan.sensor_y)
        valid &= (np.abs(body[:, 0]) > self.half_length) | (np.abs(body[:, 1]) > self.half_width)
        step = abs(scan.angle_increment)
        wrap = abs(count * step - math.tau) <= step / 2

        def neighbor(index):
            if wrap:
                return index % count
            return index if 0 <= index < count else None

        slots = {}
        # Historical projections can only supply positive-infinity cells.
        # Still establish this acquisition's real seeds below when there are no gaps.
        if np.any(ranges == math.inf):
            for seed in self._seeds:
                point = (np.asarray(seed.point) - origin) @ rotation
                index = _index(
                    math.atan2(point[1], point[0]), scan.angle_min, scan.angle_increment, count
                )
                if index is not None and ranges[index] == math.inf:
                    slots.setdefault(index, []).append(seed)
        retained, predictions = [], []
        for index, seeds in slots.items():
            if len(seeds) != 1:
                continue  # Never choose between different surfaces in one angular cell.
            seed = seeds[0]
            retained.append(seed)
            distance = self._predict(
                index, seed, points, valid, ranges, directions, neighbor, origin, rotation, scan
            )
            if distance is not None:
                predictions.append(PredictedReturn(index, distance, seed.missing, seed.capture))

        world = tuple(map(tuple, points @ rotation.T + origin))
        # Establish adjacent real continuity once, instead of fitting every seed window.
        spacing = 2 * np.minimum(ranges, np.roll(ranges, 1)) * math.sin(step / 2)
        connected = valid & np.roll(valid, 1)
        connected &= np.linalg.norm(points - np.roll(points, 1, axis=0), axis=1) <= (
            np.where(connected, spacing, 0) + 2 * self.error
        )
        if not wrap:
            connected[0] = False
        for index in np.flatnonzero(valid):
            left, right = [], []
            for direction, found in ((-1, left), (1, right)):
                for offset in range(1, 4):
                    adjacent = neighbor(int(index) + direction * offset)
                    if adjacent is None or not valid[adjacent]:
                        break
                    previous = int(index) if not found else found[-1]
                    edge = previous if direction == -1 else adjacent
                    if not connected[edge]:
                        break
                    found.append(adjacent)
            window = list(reversed(left)) + [int(index)] + right
            if len(window) >= 3:
                retained.append(_Seed(world[index], tuple(world[i] for i in window), capture, step))
        self._seeds = retained
        self._result = replace(raw, predictions=tuple(predictions))
        self._done = True
        return self._result

    def _predict(
        self, index, seed, points, valid, ranges, directions, neighbor, origin, rotation, scan
    ):
        # Find one short +Inf gap and its immediately adjacent real support.
        sides, gap = [], {index}
        for direction in (-1, 1):
            cursor, support = index, []
            for _ in range(3):
                cursor = neighbor(cursor + direction)
                if cursor is None or cursor in gap:
                    break
                if ranges[cursor] != math.inf:
                    break
                gap.add(cursor)
            if len(gap) > 3 or (cursor is not None and ranges[cursor] == math.inf):
                return None
            for _ in range(3):
                if cursor is None or not valid[cursor]:
                    break
                support.append(cursor)
                cursor = neighbor(cursor + direction)
            sides.append(support)
        left, right = sides
        internal = len(left) >= 2 and len(right) >= 2
        if not internal and not ((len(left) == 3 and not right) or (len(right) == 3 and not left)):
            return None
        historical = (np.asarray(seed.window) - origin) @ rotation
        if _line(historical, self.error) is None:
            return None
        indices = list(reversed(left)) + right
        current = points[indices]
        fitted = _line(current, self.error)
        if fitted is None:
            return None
        center, normal = fitted
        # Both the original continuous surface and current real supports must agree.
        if np.max(np.abs((historical - center) @ normal)) > self.error:
            return None
        old_spacing = np.max(np.linalg.norm(np.diff(historical, axis=0), axis=1))
        max_spacing = old_spacing * abs(scan.angle_increment) / seed.angular_step + 2 * self.error
        for support in sides:
            if (
                len(support) > 1
                and np.max(np.linalg.norm(np.diff(points[support], axis=0), axis=1)) > max_spacing
            ):
                return None
        denominator = float(normal @ directions[index])
        if abs(denominator) <= 32 * np.finfo(float).eps:
            return None
        distance = float(normal @ center) / denominator
        if not scan.range_min <= distance <= scan.range_max:
            return None
        tangent = np.array((-normal[1], normal[0]))
        position = float((directions[index] * distance) @ tangent)
        old_interval = historical @ tangent
        # Dot products/intersection division can round an exact endpoint outward.
        # This is arithmetic roundoff only, never sensor error or physical padding.
        scale = max(1.0, abs(distance), np.max(np.abs(historical)), np.max(np.abs(current)))
        roundoff = 32 * np.finfo(float).eps * scale
        old_low, old_high = float(old_interval.min()), float(old_interval.max())
        if position < old_low - roundoff or position > old_high + roundoff:
            return None
        interval = current @ tangent
        low, high = float(interval.min()), float(interval.max())
        if internal:
            return distance if low - roundoff <= position <= high + roundoff else None
        extension = max(low - position, position - high, 0.0)
        return distance if extension <= high - low + roundoff else None
