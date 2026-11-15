"""Capture-time views of startup's independent integration, never freshened at use."""

import math
from bisect import bisect_left
from collections import deque
from dataclasses import dataclass

from senior_control.models import Pose2D


@dataclass(frozen=True)
class LearningPose:
    stamp_ros: float
    pose: Pose2D
    position_error_m: float
    yaw_error_rad: float
    stationary: bool
    independent: bool
    generation: int


class LearningPoseHistory:
    def __init__(self):
        self.samples: deque[LearningPose] = deque(maxlen=2048)

    def add(self, sample: LearningPose):
        if self.samples and sample.stamp_ros <= self.samples[-1].stamp_ros:
            return
        self.samples.append(sample)

    def at(self, stamp_ros: float) -> LearningPose | None:
        samples = list(self.samples)
        times = [round(s.stamp_ros * 1e9) for s in samples]
        stamp = round(stamp_ros * 1e9)
        i = bisect_left(times, stamp)
        if i < len(times) and times[i] == stamp:
            return samples[i]
        if i == 0 or i == len(samples):
            return None  # No extrapolation or tick-time substitution for old captures.
        a, b = samples[i - 1 : i + 1]
        if a.generation != b.generation:
            return None
        ratio = (stamp_ros - a.stamp_ros) / (b.stamp_ros - a.stamp_ros)
        turn = math.atan2(math.sin(b.pose.yaw - a.pose.yaw), math.cos(b.pose.yaw - a.pose.yaw))
        # Interpolation is a software path assumption. Retain the full segment's
        # displacement/turn as additional support rather than exact-pose fiction.
        displacement = math.hypot(b.pose.x - a.pose.x, b.pose.y - a.pose.y)
        return LearningPose(
            stamp_ros,
            Pose2D(
                a.pose.x + ratio * (b.pose.x - a.pose.x),
                a.pose.y + ratio * (b.pose.y - a.pose.y),
                a.pose.yaw + ratio * turn,
            ),
            max(a.position_error_m, b.position_error_m) + displacement,
            max(a.yaw_error_rad, b.yaw_error_rad) + abs(turn),
            a.stationary and b.stationary,
            a.independent and b.independent,
            a.generation,
        )
