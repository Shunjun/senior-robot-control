"""Explicit unvalidated Fortress candidate; never selected for a real vehicle.

Geometry/wheel declarations: senior_sim URDF and vehicle-hardware-audit.
D=.3/yaw D=.5/lag=.2 are prior simulation configuration, NOT guarantees.
All gains, uncertainty, acceleration/jerk and drive bounds below are hypotheses
for bounded Linux verification, not measurements or acceptance freezes.
"""

from dataclasses import dataclass, field

from senior_control.models import Velocity
from senior_control.motion.acceleration_model import AccelerationModel
from senior_control.motion.constraints import NoReturnPolicy
from senior_control.motion.controller import ControllerConfig
from senior_control.motion.fusion import FusionConfig
from senior_control.motion.observations import AdmissionConfig, Source, SourceRule
from senior_control.motion.temporal import TemporalConfig
from senior_control.runtime.diagnostics import DiagnosticsSettings
from senior_control.sensors.input_config import OptionalMotionInput
from senior_control.sensors.policy import default_policy

PROFILE_ID = "fortress-candidate-unvalidated-v1"


@dataclass(frozen=True)
class SimulationMotionProfile:
    profile_id: str = PROFILE_ID
    gps_input: OptionalMotionInput | None = None
    rgbd_input: OptionalMotionInput | None = None
    diagnostics: DiagnosticsSettings = field(default_factory=DiagnosticsSettings)
    accel: float = 0.3
    decel: float = 0.3
    jerk: float = 1.0
    yaw_accel: float = 0.5
    yaw_decel: float = 0.5
    yaw_jerk: float = 2.0
    lag_ms: float = 200.0
    cap: float = 25 / 18
    degraded_cap: float = 0.3
    yaw_cap: float = 2.0
    length: float = 0.674
    width: float = 0.552
    startup_learning_front_occlusion_stop: bool = True
    startup_learning_rear_occlusion_stop: bool = True
    linear_error: float = 0.1
    yaw_error: float = 0.1
    sensor_error: float = 0.02
    position_error: float = 0.03
    # Independent passive-drive hypotheses, NOT wheel +/- sensor error bounds.
    assumed_body_cap: float = 1.52
    drive_linear_overshoot: float = 0.1
    drive_yaw_overshoot: float = 0.1
    horizon_s: float = 12.0
    orientation_error_rad: float = 0.0
    lateral_drift_mps: float = 0.0
    lateral_stop_ms: float | None = None
    perception_basis: str = "simulation-only static scene; original raw scan"
    risk_basis: str = "unvalidated simulation passive drive/error envelope"
    brake_tracking_basis: str = "simulation-only whole drive stop family; unvalidated"
    wheel_publisher: str = "drive"
    imu_publisher: str = "senior_sim_bridge"
    source_namespace: str = "/"
    simulation_only: bool = True
    unvalidated: bool = True

    def __post_init__(self) -> None:
        if (
            type(self.startup_learning_front_occlusion_stop) is not bool
            or type(self.startup_learning_rear_occlusion_stop) is not bool
        ):
            raise ValueError("learning occlusion settings must be boolean")
        # //Compatible: preserve old constructor shape, not its independent age policy.

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

    def acceleration_model(self) -> AccelerationModel:
        # //Compatible: preserve the simulation's existing numeric limits while
        # using the same acceleration policy as hardware.
        return AccelerationModel(self.accel, self.yaw_accel, self.accel, self.yaw_accel)

    @property
    def max_age_ms(self) -> float:
        return default_policy().hard_age_ms

    def admission(self) -> AdmissionConfig:
        return AdmissionConfig(
            tuple(SourceRule(s, self.max_age_ms, 400, 0.1, 2.0) for s in Source),
            1,
            0,
            self.cap,
            self.degraded_cap,
            self.max_age_ms,
            self.max_age_ms,
            0.05,
            16,
        )

    def temporal(self) -> TemporalConfig:
        return TemporalConfig(2.0, 512, self.max_age_ms, self.max_age_ms, self.max_age_ms, True)

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
            0.02,
            0.03,
            0.3,
            "unvalidated simulation-only passive drive hypothesis",
            True,
        )


def select_simulation_profile(
    name: str, *, simulated_clock: bool, localhost_only: str, domain_id: str
) -> SimulationMotionProfile:
    valid_domain = not domain_id or (domain_id.isdecimal() and 0 <= int(domain_id) <= 232)
    if name != PROFILE_ID or not simulated_clock or localhost_only != "1" or not valid_domain:
        raise ValueError("motion core candidate requires explicit local simulation context")
    return SimulationMotionProfile()
