"""Native-bag JSON adapters and scoring-only truth; no ROS or actuator imports."""

import bisect
import math
from collections import defaultdict
from typing import Any

from senior_control.models import Pose2D
from senior_control.motion.kinematics import compose, inverse, logarithm
from senior_control.motion.observations import (
    Covariance,
    CovarianceKind,
    Frame,
    Source,
    SourceStatus,
)
from senior_control.motion.sample_types import DeltaObservation, MotionSample, PointObservation


def stamp(value: dict[str, Any]) -> float:
    return (int(value["sec"]) * 1_000_000_000 + int(value["nanosec"])) / 1e9


class NativeAdapter:
    """Only reviewed Gazebo frames; recording-scoped identity is explicitly synthetic.

    imu_link is fixed to base_link with identity origin in senior_sim/urdf/senior.urdf.xacro.
    Observed wheel/IMU covariance components are retained, never certified by this adapter.
    The complete original matrices remain in the source file (identified by SHA/ordinal).
    Laser producer epoch and displacement covariance retain their original meanings.
    """

    def __init__(self, recording_id: str):
        self.recording_id = recording_id
        self.sequence: dict[str, int] = defaultdict(int)

    def convert(self, record: dict[str, Any], steady: float) -> MotionSample | None:
        topic = record["topic"]
        if topic not in ("/motion/scan", "/wheel/odom", "/imu/data"):
            return None
        m = record["message"]
        t = stamp(m["header"]["stamp"])
        self.sequence[topic] += 1
        seq = self.sequence[topic]
        epoch = f"recording:{self.recording_id}:{topic}:producer_identity_unavailable"
        if topic == "/motion/scan":
            if m["header"]["frame_id"] != "base_link":
                raise ValueError("unsupported scan body frame")
            epoch = str(m["source_epoch"])
            if not m["valid"]:
                return PointObservation(
                    Source.LASER,
                    Frame.BODY,
                    t,
                    steady,
                    epoch,
                    seq,
                    0,
                    SourceStatus.UNAVAILABLE,
                    (),
                    None,
                    None,
                    None,
                )
            d = m["body_delta"]
            return DeltaObservation(
                Source.LASER,
                Frame.BODY,
                stamp(m["interval_start"]),
                t,
                steady,
                epoch,
                seq,
                1,
                SourceStatus.VALID,
                Pose2D(float(d["x"]), float(d["y"]), float(d["theta"])),
                Covariance(
                    tuple(float(x) for x in m["covariance"]),
                    CovarianceKind.FULL,
                    "raw matcher displacement; statistical model unverified",
                    None,
                ),
                None,
                Pose2D(0, 0, 0),
            )
        values: tuple[float, ...]
        covariance: Covariance | None = None
        if topic == "/wheel/odom":
            if m["child_frame_id"] != "base_link":
                raise ValueError("unsupported wheel body frame")
            raw = m["twist"].get("covariance")
            if raw is not None:
                covariance = Covariance(
                    tuple(float(raw[i]) for i in (0, 5, 30, 35)),
                    CovarianceKind.FULL,
                    "raw wheel vx/wz covariance; model unverified",
                    None,
                )
            twist = m["twist"]["twist"]
            source, values = (
                Source.WHEEL,
                (float(twist["linear"]["x"]), float(twist["angular"]["z"])),
            )
        else:
            if m["header"]["frame_id"] not in ("imu_link", "base_link"):
                raise ValueError("unsupported IMU extrinsic")
            raw = m.get("angular_velocity_covariance")
            if raw is not None:
                covariance = Covariance(
                    (float(raw[8]),),
                    CovarianceKind.FULL,
                    "raw IMU z-rate covariance; model unverified",
                    None,
                )
            source, values = Source.IMU, (float(m["angular_velocity"]["z"]),)
        return PointObservation(
            source,
            Frame.BODY,
            t,
            steady,
            epoch,
            seq,
            1,
            SourceStatus.VALID,
            values,
            covariance,
            None,
            None,
        )


class TruthIndex:
    """Evaluation only. Interpolate within a recording clock segment and bounded gap."""

    def __init__(self, max_gap_s: float, capacity: int):
        self.max_gap_s = max_gap_s
        self.capacity = capacity
        self.points: dict[int, list[tuple[float, Pose2D]]] = defaultdict(list)
        self.count = 0

    def add(self, segment: int, t: float, pose: Pose2D) -> None:
        points = self.points[segment]
        if points and t <= points[-1][0]:
            return  # duplicate/reordered truth cannot create a new interpolation support
        if self.count >= self.capacity:
            raise ValueError("bounded truth index capacity exceeded")
        points.append((t, pose))
        self.count += 1

    def add_message(self, segment: int, m: dict[str, Any]) -> None:
        if m["header"]["frame_id"] != "world" or m["child_frame_id"] != "base_link":
            raise ValueError("unsupported truth frame")
        pose = m["pose"]["pose"]
        p, q = pose["position"], pose["orientation"]
        yaw = math.atan2(
            2 * (q["w"] * q["z"] + q["x"] * q["y"]), 1 - 2 * (q["y"] ** 2 + q["z"] ** 2)
        )
        self.add(segment, stamp(m["header"]["stamp"]), Pose2D(float(p["x"]), float(p["y"]), yaw))

    def _pose(self, segment: int, t: float) -> Pose2D | None:
        points = self.points[segment]
        index = bisect.bisect_left(points, t, key=lambda p: p[0])
        if index < len(points) and abs(points[index][0] - t) <= 1e-12:
            return points[index][1]
        if index == 0 or index == len(points):
            return None
        left, right = points[index - 1], points[index]
        if right[0] - left[0] > self.max_gap_s:
            return None
        f = (t - left[0]) / (right[0] - left[0])
        dyaw = math.atan2(
            math.sin(right[1].yaw - left[1].yaw), math.cos(right[1].yaw - left[1].yaw)
        )
        return Pose2D(
            left[1].x + f * (right[1].x - left[1].x),
            left[1].y + f * (right[1].y - left[1].y),
            left[1].yaw + f * dyaw,
        )

    def average(self, segment: int, a: float, b: float) -> tuple[float, float, float] | None:
        if not a < b:
            return None
        start, end = self._pose(segment, a), self._pose(segment, b)
        if start is None or end is None:
            return None
        points = self.points[segment]
        inner = [t for t, _ in points if a < t < b]
        times = [a, *inner, b]
        if any(
            right - left > self.max_gap_s for left, right in zip(times, times[1:], strict=False)
        ):
            return None
        delta = compose(inverse(start), end)
        if abs(delta.yaw) >= math.pi:
            return None
        return logarithm(delta, b - a)
