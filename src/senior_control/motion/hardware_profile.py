"""Explicit hardware response evidence, independent of product preferences and ROS.

No measured values are supplied by this module. The same T3/T4/T5 consumers
receive this profile and the isolated simulation profile through the same API.
"""

from pathlib import Path
from typing import Literal

from pydantic import BaseModel, ConfigDict, Field, model_validator

from senior_control.models import Velocity
from senior_control.motion.constraints import NoReturnPolicy
from senior_control.motion.controller import ControllerConfig
from senior_control.motion.fusion import FusionConfig
from senior_control.motion.observations import AdmissionConfig
from senior_control.motion.temporal import TemporalConfig


class HardwareMotionProfile(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True, allow_inf_nan=False)

    schema_version: Literal[1]
    profile_id: str = Field(min_length=1)
    evidence_basis: str = Field(min_length=1)
    operating_conditions: str = Field(min_length=1)
    simulation_only: Literal[False]
    unvalidated: Literal[False]
    max_age: float = Field(gt=0)
    accel: float = Field(gt=0)
    decel: float = Field(gt=0)
    jerk: float = Field(gt=0)
    yaw_accel: float = Field(gt=0)
    yaw_decel: float = Field(gt=0)
    yaw_jerk: float = Field(gt=0)
    lag: float = Field(ge=0)
    cap: float = Field(gt=0)
    degraded_cap: float = Field(gt=0)
    yaw_cap: float = Field(gt=0)
    length: float = Field(gt=0)
    width: float = Field(gt=0)
    linear_error: float = Field(ge=0)
    yaw_error: float = Field(ge=0)
    sensor_error: float = Field(ge=0)
    position_error: float = Field(ge=0)
    model_seconds: float = Field(gt=0)
    # Kept as the consumer's field name: must be an independently established
    # whole-body envelope, never a wheel speed or a simulation assumption.
    assumed_body_cap: float = Field(gt=0)
    drive_linear_overshoot: float = Field(ge=0)
    drive_yaw_overshoot: float = Field(ge=0)
    horizon_s: float = Field(gt=0)
    orientation_error_rad: float = Field(ge=0)
    lateral_drift_mps: float = Field(ge=0)
    lateral_stop_s: float = Field(ge=0)
    reaction_linear_accel: float = Field(ge=0)
    reaction_yaw_accel: float = Field(ge=0)
    watchdog_basis: str = Field(min_length=1)
    hardware_estop_topic: str = Field(pattern=r"^/[A-Za-z_][A-Za-z0-9_/]*$")
    hardware_estop_publisher: str = Field(pattern=r"^[A-Za-z_][A-Za-z0-9_]*$")
    hardware_estop_max_age_s: float = Field(gt=0)
    hardware_estop_basis: str = Field(min_length=1)
    perception_basis: str = Field(min_length=1)
    risk_basis: str = Field(min_length=1)
    brake_tracking_basis: str = Field(min_length=1)
    admission_config: AdmissionConfig
    temporal_config: TemporalConfig
    fusion_config: FusionConfig
    controller_config: ControllerConfig
    wheel_topic: str = Field(pattern=r"^/[A-Za-z_][A-Za-z0-9_/]*$")
    imu_topic: str = Field(pattern=r"^/[A-Za-z_][A-Za-z0-9_/]*$")
    wheel_publisher: str = Field(pattern=r"^[A-Za-z_][A-Za-z0-9_]*$")
    imu_publisher: str = Field(pattern=r"^[A-Za-z_][A-Za-z0-9_]*$")
    source_namespace: str = Field(
        pattern=r"^/(?:[A-Za-z_][A-Za-z0-9_]*(?:/[A-Za-z_][A-Za-z0-9_]*)*)?$"
    )

    @model_validator(mode="after")
    def coherent(self) -> "HardwareMotionProfile":
        if self.profile_id.startswith("fortress-") or self.controller_config.synthetic:
            raise ValueError("simulation/synthetic response models cannot authorize hardware")
        if self.reaction_linear_accel < self.accel or self.reaction_yaw_accel < self.yaw_accel:
            raise ValueError("reaction acceleration bounds must cover commanded acceleration")
        if not self.degraded_cap <= self.cap <= self.assumed_body_cap:
            raise ValueError("degraded <= normal <= independent whole-body envelope required")
        for policy in (self.admission_config, self.fusion_config):
            if (policy.normal_cap_mps, policy.degraded_cap_mps) != (self.cap, self.degraded_cap):
                raise ValueError("source and output caps must use the same hardware policy")
        return self

    def reaction_acceleration(self, current: Velocity) -> Velocity:
        return Velocity(self.reaction_linear_accel, self.reaction_yaw_accel)

    def admission(self) -> AdmissionConfig:
        return self.admission_config

    def temporal(self) -> TemporalConfig:
        return self.temporal_config

    def fusion(self) -> FusionConfig:
        return self.fusion_config

    def controller(self) -> ControllerConfig:
        return self.controller_config

    def perception_no_return_policy(self, frame_id: str, range_max: float) -> NoReturnPolicy:
        # Hardware +Inf is unknown until a separate sensor-specific contract exists.
        return NoReturnPolicy.UNKNOWN


def load_hardware_profile(path: Path) -> HardwareMotionProfile:
    if not path.is_absolute():
        raise ValueError("hardware motion profile path must be absolute")
    return HardwareMotionProfile.model_validate_json(path.read_bytes())
