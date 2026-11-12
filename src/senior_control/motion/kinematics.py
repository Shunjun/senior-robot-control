"""Small SE(2) primitives, shared by interval resolution and offline evaluation."""

import math

from senior_control.models import Pose2D


def compose(a: Pose2D, b: Pose2D) -> Pose2D:
    c, s = math.cos(a.yaw), math.sin(a.yaw)
    yaw = a.yaw + b.yaw
    return Pose2D(
        a.x + c * b.x - s * b.y, a.y + s * b.x + c * b.y, math.atan2(math.sin(yaw), math.cos(yaw))
    )


def inverse(a: Pose2D) -> Pose2D:
    c, s = math.cos(a.yaw), math.sin(a.yaw)
    return Pose2D(-c * a.x - s * a.y, s * a.x - c * a.y, -a.yaw)


def exponential(vx: float, vy: float, wz: float, dt: float) -> Pose2D:
    theta = wz * dt
    if abs(theta) < 1e-8:
        a, b = 1 - theta * theta / 6, theta / 2 - theta**3 / 24
    else:
        a, b = math.sin(theta) / theta, (1 - math.cos(theta)) / theta
    return Pose2D(dt * (a * vx - b * vy), dt * (b * vx + a * vy), theta)


def logarithm(p: Pose2D, dt: float) -> tuple[float, float, float]:
    if dt <= 0 or abs(p.yaw) >= math.pi:
        raise ValueError("invalid/aliased SE2 interval")
    theta = p.yaw
    a = 1 - theta * theta / 12 if abs(theta) < 1e-8 else theta / 2 / math.tan(theta / 2)
    b = theta / 2
    return ((a * p.x + b * p.y) / dt, (-b * p.x + a * p.y) / dt, theta / dt)


def body_delta(delta: Pose2D, extrinsic: Pose2D) -> Pose2D:
    return compose(compose(extrinsic, delta), inverse(extrinsic))
