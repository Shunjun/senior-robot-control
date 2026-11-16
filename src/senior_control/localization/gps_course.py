"""Single-antenna course initialization; statistical precision is not a truth bound."""

import math
from collections import deque
from dataclasses import dataclass

from senior_control.motion.estimator import Origin
from senior_control.sensors.policy import default_policy
from senior_control.sensors.time_proof import MeasurementInterval, MeasurementTime


def wrap(value: float) -> float:
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
    def __init__(self) -> None:
        self.anchor: CourseFix | None = None
        self.previous: CourseFix | None = None
        self.last_sequence: tuple[str, int] | None = None
        self.wheel_distance = 0.0
        self.turn = 0.0
        self.heading: CourseHeading | None = None
        self.motion: deque[tuple[MeasurementTime, float, float, int, float]] = deque(maxlen=1024)
        self.motion_total = (0.0, 0.0)
        self.anchor_motion: tuple[MeasurementTime, float, float, int, float] | None = None
        self.motion_direction = 0

    def record_motion(
        self,
        *,
        measurement_steady: float,
        wheel_vx: float | None,
        imu_wz: float | None,
        generation: int,
        measurement: MeasurementTime | None = None,
        interval: MeasurementInterval | None = None,
    ) -> None:
        """Continuous estimator ticks qualify the whole GPS course window."""
        if wheel_vx is None or imu_wz is None:
            self.motion.clear()
            self.reset()
            return
        direction = 1 if wheel_vx > 0.02 else -1 if wheel_vx < -0.02 else 0
        if not direction or self.motion_direction not in (0, direction):
            self.reset()
        self.motion_direction = direction
        coordinate = measurement or MeasurementTime("host", None, measurement_steady)
        if self.motion:
            previous = self.motion[-1]
            compatible = coordinate.same_axis(previous[0])
            dt = coordinate.elapsed(previous[0]) if compatible else -1
            covered = (
                not coordinate.clock_domain.startswith("sim_step:")
                or interval is not None
                and interval.end == coordinate
                and interval.start.same_axis(previous[0])
                and interval.start.seconds <= previous[0].seconds <= interval.end.seconds
            )
            if (
                generation != previous[3]
                or dt < 0
                or not covered
                or measurement_steady - previous[4] >= (default_policy().hard_age_ms / 1000)
            ):
                self.motion.clear()
                self.reset()
            elif dt == 0:
                return
            else:
                self.motion_total = (
                    self.motion_total[0] + wheel_vx * dt,
                    self.motion_total[1] + abs(imu_wz) * dt,
                )
        self.motion.append((coordinate, *self.motion_total, generation, measurement_steady))

    def reset(self) -> None:
        self.anchor = self.previous = self.heading = None
        self.wheel_distance = self.turn = 0.0
        self.anchor_motion = None

    def observe(
        self, fix: CourseFix, *, wheel_vx: float, imu_wz: float, now_ros: float, now_steady: float
    ) -> CourseHeading | None:
        self.heading = None
        r = fix.origin
        age = default_policy().age(
            measured_steady=r.measurement_steady,
            received_steady=r.received_steady,
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
        records = [
            m
            for m in self.motion
            if m[0].same_axis(r.coordinate) and m[0].seconds <= r.coordinate.seconds
        ]
        current = records[-1] if records else None
        hard_s = default_policy().hard_age_ms / 1000
        if current is None or r.measurement_steady - current[4] >= hard_s:
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
            or self.motion[0][0].seconds > self.anchor_motion[0].seconds
        ):
            self.reset()
            return None
        assert self.previous is not None
        dt = (
            r.coordinate.elapsed(self.previous.origin.coordinate)
            if r.coordinate.same_axis(self.previous.origin.coordinate)
            else -1
        )
        if dt <= 0 or wheel_vx * self.wheel_distance < 0:
            self.reset()
            return None
        self.previous = fix
        self.wheel_distance = current[1] - self.anchor_motion[1]
        self.turn = current[2] - self.anchor_motion[2]
        dx, dy = fix.east_m - self.anchor.east_m, fix.north_m - self.anchor.north_m
        baseline = math.hypot(dx, dy)
        assert self.anchor.position_sigma_m is not None
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
