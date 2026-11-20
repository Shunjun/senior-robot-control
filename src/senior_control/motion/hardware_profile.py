"""Explicit hardware response evidence, independent of product preferences and ROS.

No measured values are supplied by this module. The same T3/T4/T5 consumers
receive the same explicit deployment numerics through this API.
"""

import json
from pathlib import Path
from typing import Literal

from pydantic import BaseModel, ConfigDict, Field, model_validator

from senior_control.models import Velocity
from senior_control.motion.acceleration_model import AccelerationModel
from senior_control.motion.constraints import NoReturnPolicy
from senior_control.motion.controller import ControllerConfig
from senior_control.motion.fusion import FusionConfig
from senior_control.motion.observations import AdmissionConfig, Source, SourceRule
from senior_control.motion.temporal import TemporalConfig
from senior_control.runtime.diagnostics import DiagnosticsSettings
from senior_control.sensors.input_config import OptionalMotionInput
from vehicle_core.sensor_policy import default_policy


class HardwareMotionProfile(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True, allow_inf_nan=False)

    schema_version: Literal[1]
    profile_id: str = Field(min_length=1)
    evidence_basis: str = ""
    operating_conditions: str = ""
    # Software evidence TTL; acceptance still checks original receipts.
    diagnostics: DiagnosticsSettings = Field(default_factory=DiagnosticsSettings)

    @property
    def max_age_ms(self) -> float:
        """Single sensor-policy limit; not an independently editable profile input."""
        return default_policy().hard_age_ms

    accel: float = Field(gt=0)
    decel: float = Field(gt=0)
    jerk: float = Field(gt=0)
    yaw_accel: float = Field(gt=0)
    yaw_decel: float = Field(gt=0)
    yaw_jerk: float = Field(gt=0)
    lag_ms: float = Field(ge=0)
    # Product request ceiling, independently clamped by calibrated physical limits.
    requested_linear_mps: float = Field(default=25 / 18, gt=0)
    degraded_cap: float = Field(gt=0)
    yaw_cap: float = Field(default=2.0, gt=0)
    # Physical body dimensions must be supplied by the vehicle configuration.
    length: float = Field(gt=0)
    width: float = Field(gt=0)
    track_width_m: float = Field(default=0.481154, gt=0)
    wheel_speed_mps: float = Field(gt=0)
    stationary_linear_mps: float = Field(gt=0)
    stationary_yaw_rps: float = Field(gt=0)
    startup_learning_front_occlusion_stop: bool = Field(default=True, strict=True)
    startup_learning_rear_occlusion_stop: bool = Field(default=True, strict=True)
    linear_error: float = Field(ge=0)
    yaw_error: float = Field(ge=0)
    sensor_error: float = Field(ge=0)
    position_error: float = Field(ge=0)
    # Kept as the consumer's field name: must be an independently established
    # whole-body envelope, never a wheel speed or a simulation assumption.
    assumed_body_cap: float = Field(gt=0)
    drive_linear_overshoot: float = Field(ge=0)
    drive_yaw_overshoot: float = Field(ge=0)
    horizon_s: float = Field(
        default=12.0,
        gt=0,
        description="Software stop-search horizon; physical envelope must cover this horizon",
    )
    orientation_error_rad: float = Field(ge=0)
    lateral_drift_mps: float = Field(ge=0)
    lateral_stop_ms: float = Field(ge=0)
    reaction_linear_accel: float = Field(ge=0)
    reaction_yaw_accel: float = Field(ge=0)
    watchdog_mode: Literal["verified", "unknown"] = "unknown"
    watchdog_basis: str = "Target firmware watchdog behavior unconfirmed"
    perception_basis: str = "original raw lidar; hardware no-return remains unknown"
    risk_basis: str = ""
    brake_tracking_basis: str = ""
    # //Compatible: optional legacy declarations must match the single derived configuration.
    fusion_config: FusionConfig | None = None
    controller_config: ControllerConfig | None = None
    gps_input: OptionalMotionInput | None = None
    rgbd_input: OptionalMotionInput | None = None
    wheel_topic: str = Field(default="/odom", pattern=r"^/[A-Za-z_][A-Za-z0-9_/]*$")
    imu_topic: str = Field(default="/imu/data_raw", pattern=r"^/[A-Za-z_][A-Za-z0-9_/]*$")
    wheel_publisher: str = Field(default="wheeltec_robot", pattern=r"^[A-Za-z_][A-Za-z0-9_]*$")
    imu_publisher: str = Field(default="wheeltec_robot", pattern=r"^[A-Za-z_][A-Za-z0-9_]*$")
    source_namespace: str = Field(
        default="/", pattern=r"^/(?:[A-Za-z_][A-Za-z0-9_]*(?:/[A-Za-z_][A-Za-z0-9_]*)*)?$"
    )

    @property
    def cap(self) -> float:
        return min(self.requested_linear_mps, self.assumed_body_cap, self.wheel_speed_mps)

    @model_validator(mode="after")
    def coherent(self) -> "HardwareMotionProfile":
        if self.gps_input is not None and self.gps_input.origin_family is None:
            raise ValueError("GPS input requires its original raw acquisition family")
        if self.lateral_stop_ms / 1000 > self.horizon_s:
            raise ValueError("lateral settling must fit the search horizon")
        if self.reaction_linear_accel < self.accel or self.reaction_yaw_accel < self.yaw_accel:
            raise ValueError("reaction acceleration bounds must cover commanded acceleration")
        if (
            self.stationary_linear_mps <= self.linear_error
            or self.stationary_yaw_rps <= self.yaw_error
        ):
            raise ValueError(
                "physical stillness tolerances must exceed calibrated measurement errors"
            )
        if not self.degraded_cap <= self.cap <= self.assumed_body_cap:
            raise ValueError("degraded <= normal <= independent whole-body envelope required")
        # No independently editable wheel/error/stillness copy may contradict
        # the profile that supplies T4/T5's response and error envelopes.
        for supplied, derived in (
            (self.fusion_config, self.fusion()),
            (self.controller_config, self.controller()),
        ):
            if supplied is not None and supplied != derived:
                raise ValueError(
                    "nested configuration contradicts derived hardware policy; remove duplicate"
                )
        return self

    def acceleration_model(self) -> AccelerationModel:
        return AccelerationModel(
            self.accel, self.yaw_accel, self.reaction_linear_accel, self.reaction_yaw_accel
        )

    def admission(self) -> AdmissionConfig:
        return AdmissionConfig(
            tuple(SourceRule(source, self.max_age_ms, 400, 0.1, 2.0) for source in Source),
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
        return FusionConfig(
            (),
            self.cap,
            self.degraded_cap,
            max(2 * self.linear_error, 1e-6),
            max(2 * self.yaw_error, 1e-6),
        )

    def controller(self) -> ControllerConfig:
        # Initial P-only software tuning; these gains are not braking evidence.
        # All physical quantities and stillness thresholds have one owner here.
        return ControllerConfig(
            Velocity(0.2, 0.2),
            Velocity(0, 0),
            Velocity(0, 0),
            self.track_width_m,
            self.wheel_speed_mps,
            self.stationary_linear_mps - self.linear_error,
            self.stationary_yaw_rps - self.yaw_error,
            max(2 * self.linear_error, self.stationary_linear_mps),
            self.evidence_basis or self.profile_id,
            False,
        )

    def perception_no_return_policy(self, frame_id: str, range_max: float) -> NoReturnPolicy:
        # Hardware +Inf is unknown until a separate sensor-specific contract exists.
        return NoReturnPolicy.UNKNOWN


def load_hardware_profile(path: Path) -> HardwareMotionProfile:
    if not path.is_absolute():
        raise ValueError("hardware motion profile path must be absolute")
    value = json.loads(path.read_bytes())
    return HardwareMotionProfile.model_validate(value)
