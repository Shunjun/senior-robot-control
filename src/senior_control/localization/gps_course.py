"""Single-antenna course initialization; statistical precision is not a truth bound."""

import math
from collections import deque
from dataclasses import dataclass

from senior_control.motion.estimator import Origin
from senior_control.sensors.policy import default_policy


def wrap(value):
    return math.atan2(math.sin(value), math.cos(value))


@dataclass(frozen=True)
class CourseFix:
    east_m: float
    north_m: float
    position_sigma_m: float | None
    east_mps: float | None
    north_mps: float | None
    velocity_sigma_mps: float | None
    origin: Origin


@dataclass(frozen=True)
class CourseHeading:
    yaw_rad: float
    sigma_rad: float
    stamp_ros: float
    deadline_steady: float
    source: str = "single_antenna_course"


class GPSCourseObserver:
    def __init__(self):
        self.anchor = None
        self.previous = None
        self.last_sequence = None
        self.wheel_distance = 0.0
        self.turn = 0.0
        self.heading = None
        self.motion = deque(maxlen=1024)
        self.motion_total = (0.0, 0.0)
        self.anchor_motion = None
        self.motion_direction = 0

    def record_motion(self, *, stamp_ros, wheel_vx, imu_wz, generation):
        """Continuous estimator ticks qualify the whole GPS course window."""
        if wheel_vx is None or imu_wz is None:
            self.motion.clear()
            self.reset()
            return
        direction = 1 if wheel_vx > 0.02 else -1 if wheel_vx < -0.02 else 0
        if not direction or self.motion_direction not in (0, direction):
            self.reset()
        self.motion_direction = direction
        if self.motion:
            previous = self.motion[-1]
            dt = stamp_ros - previous[0]
            if generation != previous[3] or dt < 0 or dt >= default_policy().hard_age_s:
                self.motion.clear()
                self.reset()
            elif dt == 0:
                return
            else:
                self.motion_total = (
                    self.motion_total[0] + wheel_vx * dt,
                    self.motion_total[1] + abs(imu_wz) * dt,
                )
        self.motion.append((stamp_ros, *self.motion_total, generation))

    def reset(self):
        self.anchor = self.previous = self.heading = None
        self.wheel_distance = self.turn = 0.0
        self.anchor_motion = None

    def observe(self, fix: CourseFix, *, wheel_vx, imu_wz, now_ros, now_steady):
        self.heading = None
        r = fix.origin
        age = default_policy().age(
            stamp_ros=r.stamp_ros,
            received_ros=r.received_ros,
            received_steady=r.received_steady,
            now_ros=now_ros,
            now_steady=now_steady,
        )
        key = (r.epoch, r.sequence)
        if self.last_sequence is not None:
            if key[0] == self.last_sequence[0] and key[1] <= self.last_sequence[1]:
                return None
            if key[0] != self.last_sequence[0]:
                self.reset()
        self.last_sequence = key
        sigma = fix.position_sigma_m
        if (
            age.band == "invalid"
            or sigma is None
            or not all(math.isfinite(v) for v in (fix.east_m, fix.north_m, sigma, wheel_vx, imu_wz))
            or sigma <= 0
            or abs(wheel_vx) < 0.02
            or abs(imu_wz) > 0.05
        ):
            self.reset()
            return None
        # Use original capture time, not callback time. GPS can be slower than
        # the estimator; absence of a continuous IMU/wheel window stays unknown.
        records = [m for m in self.motion if m[0] <= r.stamp_ros]
        current = records[-1] if records else None
        if current is None or r.stamp_ros - current[0] >= default_policy().hard_age_s:
            self.reset()
            return None
        if self.anchor is None:
            self.anchor = self.previous = fix
            self.anchor_motion = current
            return None
        if (
            self.anchor_motion is None
            or current[3] != self.anchor_motion[3]
            or not self.motion
            or self.motion[0][0] > self.anchor_motion[0]
        ):
            self.reset()
            return None
        dt = r.stamp_ros - self.previous.origin.stamp_ros
        if dt <= 0 or wheel_vx * self.wheel_distance < 0:
            self.reset()
            return None
        self.previous = fix
        self.wheel_distance = current[1] - self.anchor_motion[1]
        self.turn = current[2] - self.anchor_motion[2]
        dx, dy = fix.east_m - self.anchor.east_m, fix.north_m - self.anchor.north_m
        baseline = math.hypot(dx, dy)
        uncertainty = math.hypot(sigma, self.anchor.position_sigma_m)
        if abs(self.turn) > 0.05 or abs(self.wheel_distance) > 30:
            self.reset()
            return None
        if baseline < 0.2 or math.atan2(2 * uncertainty, baseline) > 0.12:
            return None
        if abs(baseline - abs(self.wheel_distance)) > max(0.05, baseline * 0.35):
            self.reset()
            return None
        course = math.atan2(dy, dx)
        velocity_sigma = fix.velocity_sigma_mps
        if (
            fix.east_mps is not None
            and fix.north_mps is not None
            and velocity_sigma is not None
            and velocity_sigma > 0
            and all(math.isfinite(v) for v in (fix.east_mps, fix.north_mps, velocity_sigma))
        ):
            speed = math.hypot(fix.east_mps, fix.north_mps)
            if speed > 3 * velocity_sigma:
                if abs(wrap(math.atan2(fix.north_mps, fix.east_mps) - course)) > 0.12:
                    self.reset()
                    return None
        self.heading = CourseHeading(
            wrap(course + (math.pi if wheel_vx < 0 else 0)),
            math.atan2(uncertainty, baseline),
            r.stamp_ros,
            age.deadline_steady,
        )
        return self.heading
