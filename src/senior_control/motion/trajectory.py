"""Finite jerk phases in velocity space; no clock, feedback or permission owner."""

import math
from dataclasses import dataclass

from senior_control.models import Velocity
from senior_control.motion.contracts import ResponseBounds


@dataclass(frozen=True)
class TrajectoryStep:
    velocity: Velocity
    acceleration: Velocity


def scale_wheels(v: Velocity, track_width_m: float, wheel_speed_mps: float) -> Velocity:
    """Common scaling preserves the curvature of the combined wheel request."""
    if not all(math.isfinite(x) and x > 0 for x in (track_width_m, wheel_speed_mps)):
        raise ValueError("wheel capability")
    peak = max(abs(v.vx - v.wz * track_width_m / 2), abs(v.vx + v.wz * track_width_m / 2))
    factor = min(1.0, wheel_speed_mps / peak) if peak else 1.0
    return Velocity(v.vx * factor, v.wz * factor)


def _axis(
    v: float, a: float, goal: float, accel: float, decel: float, jerk: float, dt: float
) -> tuple[float, float]:
    # Exact three-phase solution of dv/dt=a, da/dt=j to (goal,0).
    # Release distance in velocity-space decides the jerk direction, including
    # acceleration left over when the goal changes. No numerical catch-up loop.
    if v == goal and a == 0:
        return v, a
    delta = goal - v
    release = a * abs(a) / (2 * jerk)
    if a and abs(delta - release) <= 1e-12 * max(1.0, abs(v), abs(goal)):
        duration = abs(a) / jerk
        h = min(dt, duration)
        j = -math.copysign(jerk, a)
        return (goal, 0.0) if dt >= duration else (v + a * h + j * h * h / 2, a + j * h)
    sign = 1.0 if delta >= release else -1.0
    limit = accel if abs(goal) > abs(v) and v * goal >= 0 else decel
    peak = sign * min(limit, math.sqrt(max(0.0, sign * jerk * delta + a * a / 2)))
    first = (peak - a) / (sign * jerk)
    if first < -1e-10:
        raise ValueError("infeasible_acceleration_transition")
    first = max(0.0, first)
    release_time = abs(peak) / jerk
    ramp_delta = (a + peak) * first / 2 + peak * release_time / 2
    hold = (delta - ramp_delta) / peak if peak else 0.0
    if hold < -1e-9:
        raise ValueError("infeasible_phase")
    phases = ((first, sign * jerk), (max(0.0, hold), 0.0), (release_time, -sign * jerk))
    remaining = dt
    initial = v
    for duration, j in phases:
        h = min(remaining, duration)
        next_v = v + a * h + j * h * h / 2
        if goal == 0 and initial * next_v < -1e-12:
            raise ValueError("infeasible_no_reverse")
        v = next_v
        a += j * h
        remaining -= h
        if remaining <= 1e-12:
            break
    if remaining > 1e-12:
        v, a = goal, 0.0
    if abs(v) < 1e-12:
        v = 0.0
    if abs(a) < 1e-12:
        a = 0.0
    return v, a


class Trajectory:
    """Transactional output state: rejected proposals never advance it."""

    def __init__(self, velocity: Velocity, acceleration: Velocity):
        if not all(
            math.isfinite(x) for x in (velocity.vx, velocity.wz, acceleration.vx, acceleration.wz)
        ):
            raise ValueError("invalid_initial_state")
        self.velocity = velocity
        self.acceleration = acceleration

    def propose(self, goal: Velocity, response: ResponseBounds, dt: float) -> TrajectoryStep:
        values = (
            goal.vx,
            goal.wz,
            dt,
            response.accel,
            response.decel,
            response.jerk,
            response.yaw_accel,
            response.yaw_decel,
            response.yaw_jerk,
        )
        if not all(math.isfinite(x) for x in values) or dt <= 0 or min(values[3:]) <= 0:
            raise ValueError("invalid_trajectory_input")
        vx, ax = _axis(
            self.velocity.vx,
            self.acceleration.vx,
            goal.vx,
            response.accel,
            response.decel,
            response.jerk,
            dt,
        )
        wz, az = _axis(
            self.velocity.wz,
            self.acceleration.wz,
            goal.wz,
            response.yaw_accel,
            response.yaw_decel,
            response.yaw_jerk,
            dt,
        )
        return TrajectoryStep(Velocity(vx, wz), Velocity(ax, az))

    def commit(self, step: TrajectoryStep) -> None:
        self.velocity, self.acceleration = step.velocity, step.acceleration
