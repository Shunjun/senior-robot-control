"""Explicit unvalidated Fortress candidate; never selected for a real vehicle.

Geometry/wheel declarations: senior_sim URDF and vehicle-hardware-audit.
D=.3/yaw D=.5/lag=.2 are prior simulation configuration, NOT guarantees.
All gains, uncertainty, acceleration/jerk and drive bounds below are hypotheses
for bounded Linux verification, not measurements or acceptance freezes.
"""

from dataclasses import dataclass

from senior_control.models import Velocity
from senior_control.motion.constraints import NoReturnPolicy
from senior_control.motion.controller import ControllerConfig
from senior_control.motion.fusion import FusionConfig
from senior_control.motion.observations import AdmissionConfig, Source, SourceRule
from senior_control.motion.temporal import TemporalConfig

PROFILE_ID = "fortress-candidate-unvalidated-v1"


@dataclass(frozen=True)
class SimulationMotionProfile:
    profile_id: str = PROFILE_ID
    max_age: float = 0.8
    accel: float = 0.3
    decel: float = 0.3
    jerk: float = 1.0
    yaw_accel: float = 0.5
    yaw_decel: float = 0.5
    yaw_jerk: float = 2.0
    lag: float = 0.2
    cap: float = 25 / 18
    degraded_cap: float = 0.3
    yaw_cap: float = 2.0
    length: float = 0.674
    width: float = 0.552
    linear_error: float = 0.1
    yaw_error: float = 0.1
    sensor_error: float = 0.02
    position_error: float = 0.03
    model_seconds: float = 175.0
    # Independent passive-drive hypotheses, NOT wheel +/- sensor error bounds.
    assumed_body_cap: float = 1.52
    drive_linear_overshoot: float = 0.1
    drive_yaw_overshoot: float = 0.1
    horizon_s: float = 12.0
    orientation_error_rad: float = 0.0
    lateral_drift_mps: float = 0.0
    lateral_stop_s: float | None = None
    perception_basis: str = "simulation-only static scene; original raw scan"
    risk_basis: str = "unvalidated simulation passive drive/error envelope"
    brake_tracking_basis: str = "simulation-only whole drive stop family; unvalidated"
    wheel_publisher: str = "drive"
    imu_publisher: str = "senior_sim_bridge"
    source_namespace: str = "/"
    simulation_only: bool = True
    unvalidated: bool = True

    def perception_no_return_policy(self, frame_id: str, range_max: float) -> NoReturnPolicy:
        # //Compatible: ideal GPU lidar sensors6 6.9.0 SetClamp(false),
        # rendering6 6.6.4 and ros_gz_bridge 0.244.26 retain REP117 +Inf.
        # Shared hardware filtering remains UNKNOWN; NaN/-Inf are never mapped.
        if (
            self.profile_id == PROFILE_ID
            and self.simulation_only
            and self.unvalidated
            and frame_id == "laser_link"
            and range_max == 20.0
        ):
            return NoReturnPolicy.VALIDATED_FREE_TO_RANGE
        return NoReturnPolicy.UNKNOWN

    def reaction_acceleration(self, current: Velocity) -> Velocity:
        # //Compatible: retain the isolated candidate response hypothesis.
        return Velocity(min(self.accel, abs(current.vx)), min(self.yaw_accel, abs(current.wz)))

    def admission(self) -> AdmissionConfig:
        return AdmissionConfig(
            tuple(SourceRule(s, self.max_age, 0.4, 0.1, 2.0) for s in Source),
            5,
            0.5,
            self.cap,
            self.degraded_cap,
            0.2,
            0.1,
            0.05,
            0.3,
            16,
        )

    def temporal(self) -> TemporalConfig:
        return TemporalConfig(2.0, 512, 0.15, 0.15, 0.15, True)

    def fusion(self) -> FusionConfig:
        return FusionConfig((), self.cap, self.degraded_cap, 0.4, 0.2)

    def controller(self) -> ControllerConfig:
        # P only initially. I is not enabled by an assumed firmware PI.
        return ControllerConfig(
            Velocity(0.2, 0.2),
            Velocity(0, 0),
            Velocity(0, 0),
            0.481154,
            1.52,
            0.15,
            0.02,
            0.03,
            5,
            0.5,
            0.3,
            "unvalidated simulation-only passive drive hypothesis",
            True,
        )


def select_simulation_profile(
    name: str, *, simulated_clock: bool, localhost_only: str, domain_id: str
) -> SimulationMotionProfile:
    if name != PROFILE_ID or not simulated_clock or localhost_only != "1" or domain_id != "173":
        raise ValueError("motion core candidate requires explicit isolated simulation context")
    return SimulationMotionProfile()
