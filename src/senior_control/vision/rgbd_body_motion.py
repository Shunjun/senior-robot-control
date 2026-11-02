"""Diagnostic frame-direction conversion; not motion-source qualification."""

from typing import Any

from senior_control.vision.independent_motion import camera_to_body_transform


def body_pose_increment(
    camera_transform: Any, body_from_camera_first: Any, body_from_camera_second: Any
) -> tuple[tuple[float, ...], ...]:
    """Return B1<-B2 from static-point C2<-C1 and measured B<-C extrinsics."""
    import numpy as np

    static_body = np.asarray(
        camera_to_body_transform(camera_transform, body_from_camera_first, body_from_camera_second)
    )
    result = np.eye(4)
    rotation = static_body[:3, :3].T
    result[:3, :3] = rotation
    try:
        with np.errstate(over="raise", invalid="raise"):
            result[:3, 3] = -rotation @ static_body[:3, 3]
    except FloatingPointError as exc:
        raise ValueError("invalid_rigid_transform") from exc
    if not np.isfinite(result).all():
        raise ValueError("invalid_rigid_transform")
    return tuple(tuple(float(value) for value in row) for row in result)


def planar_body_twist(
    body_increment: Any, *, dt: float, max_tilt_rad: float, max_vertical_m: float
) -> tuple[float, float, float]:
    """Apply caller-supplied planar domain limits and SE(2) log, without qualification."""
    import math

    import numpy as np

    from senior_control.models import Pose2D
    from senior_control.safety.motion_estimator import body_twist_delta

    if (
        not all(math.isfinite(x) for x in (dt, max_tilt_rad, max_vertical_m))
        or dt <= 0
        or max_tilt_rad < 0
        or max_vertical_m < 0
    ):
        raise ValueError("invalid_planar_limits")
    matrix = np.asarray(camera_to_body_transform(body_increment, np.eye(4), np.eye(4)))
    tilt = math.atan2(math.hypot(float(matrix[0, 2]), float(matrix[1, 2])), float(matrix[2, 2]))
    if tilt > max_tilt_rad or abs(float(matrix[2, 3])) > max_vertical_m:
        raise ValueError("nonplanar_motion")
    yaw = math.atan2(float(matrix[1, 0]), float(matrix[0, 0]))
    integrated = body_twist_delta(Pose2D(float(matrix[0, 3]), float(matrix[1, 3]), yaw))
    return integrated.x / dt, integrated.y / dt, integrated.yaw / dt
