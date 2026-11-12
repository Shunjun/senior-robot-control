"""Reference selection and target quality constraints, not numerical fusion or permission."""

import math
from dataclasses import dataclass
from enum import Enum

from senior_control.models import BodyTwist
from senior_control.motion.contracts import BodyMotion
from senior_control.motion.observations import AdaptedObservation, AdmissionConfig, Source


class SourceMode(str, Enum):
    UNAVAILABLE = "unavailable"
    RECOVERING = "recovering"
    LASER_REFERENCE = "laser_reference"
    GPS_IMU_REFERENCE = "gps_imu_reference"
    WHEEL_IMU_LIMITED = "wheel_imu_limited"


@dataclass(frozen=True)
class SourceView:
    observation: AdaptedObservation
    ready: bool


@dataclass(frozen=True)
class MotionAssessment:
    mode: SourceMode
    motion: BodyMotion | None
    contributors: tuple[AdaptedObservation, ...]
    target_linear_cap_mps: float | None
    recovery_phase: str
    reason: str
    age_steady_s: float | None
    available_sources: tuple[AdaptedObservation, ...] = ()

    @property
    def uncertainty_supported(self) -> bool:
        return self.motion is not None and self.motion.upper is not None


def select_reference(
    views: dict[Source, SourceView], config: AdmissionConfig, *, now_steady: float
) -> MotionAssessment:
    ready = {source: view.observation for source, view in views.items() if view.ready}

    def paired(first: Source, second: Source) -> bool:
        if first not in ready or second not in ready:
            return False
        a, b = ready[first].original, ready[second].original
        # T3a does not align or predict component means across different windows.
        return abs(a.start_ros - b.start_ros) <= max(
            math.ulp(a.start_ros), math.ulp(b.start_ros)
        ) and abs(a.end_ros - b.end_ros) <= max(math.ulp(a.end_ros), math.ulp(b.end_ros))

    contributors: tuple[AdaptedObservation, ...]
    if Source.LASER in ready:
        primary = ready[Source.LASER]
        contributors = (primary,)
        values = primary.body_values
        mode = SourceMode.LASER_REFERENCE
    elif paired(Source.GPS, Source.IMU):
        primary, angular = ready[Source.GPS], ready[Source.IMU]
        contributors = (primary, angular)
        values = (*primary.body_values, angular.body_values[0])
        mode = SourceMode.GPS_IMU_REFERENCE
    elif paired(Source.WHEEL, Source.IMU):
        primary, angular = ready[Source.WHEEL], ready[Source.IMU]
        contributors = (primary, angular)
        # Same-window component assembly: wheel translation, measured IMU yaw.
        # Cross-source covariance and numerical fusion remain unsupported.
        values = (*primary.body_values[:2], angular.body_values[0])
        mode = SourceMode.WHEEL_IMU_LIMITED
    else:
        complete = (
            Source.LASER in views
            or {Source.GPS, Source.IMU} <= views.keys()
            or {Source.WHEEL, Source.IMU} <= views.keys()
        )
        waiting = complete and any(not view.ready for view in views.values())
        mode = SourceMode.RECOVERING if waiting else SourceMode.UNAVAILABLE
        reason = (
            "interval_alignment_required"
            if complete and not waiting
            else "no_complete_ready_reference"
        )
        return MotionAssessment(
            mode,
            None,
            (),
            None,
            "warming" if waiting else "unavailable",
            reason,
            None,
            tuple(view.observation for view in views.values()),
        )
    original = primary.original
    deadline = min(item.valid_until_steady for item in contributors)
    age = max(now_steady - item.original.received_steady for item in contributors)
    motion = BodyMotion(
        BodyTwist(*values),
        None,
        original.start_ros,
        original.end_ros,
        now_steady,
        deadline,
        original.epoch,
        True,
        mode.value,
    )
    cap = config.degraded_cap_mps if mode is SourceMode.WHEEL_IMU_LIMITED else config.normal_cap_mps
    return MotionAssessment(
        mode,
        motion,
        contributors,
        cap,
        "ready_target",
        mode.value,
        age,
        tuple(view.observation for view in views.values()),
    )
