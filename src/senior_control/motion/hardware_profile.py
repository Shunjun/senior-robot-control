"""Explicit hardware response evidence, independent of product preferences and ROS.

No measured values are supplied by this module. The same T3/T4/T5 consumers
receive this profile and the isolated simulation profile through the same API.
"""

import json
from pathlib import Path
from typing import Literal

from pydantic import BaseModel, ConfigDict, Field, model_validator

from senior_control.models import Velocity
from senior_control.motion.constraints import NoReturnPolicy
from senior_control.motion.controller import ControllerConfig
from senior_control.motion.fusion import FusionConfig
from senior_control.motion.observations import AdmissionConfig, Source, SourceRule
from senior_control.motion.temporal import TemporalConfig


class HardwareEstopCapability(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)
    mode: Literal["readable", "independent_unreadable", "unknown"] = "unknown"
    basis: str = "Target hardware ESTOP capability unconfirmed; see vehicle-hardware-audit"
    topic: str | None = None
    publisher: str | None = None
    namespace: str = "/"
    max_age_s: float | None = Field(default=None, gt=0)

    @model_validator(mode="after")
    def capability(self) -> "HardwareEstopCapability":
        if self.mode == "readable":
            if (
                not self.topic
                or not self.topic.startswith("/")
                or not self.publisher
                or self.max_age_s is None
            ):
                raise ValueError("readable ESTOP requires actual topic, publisher and age budget")
        elif any(x is not None for x in (self.topic, self.publisher, self.max_age_s)):
            raise ValueError("unreadable/unknown ESTOP must not invent a ROS state source")
        return self


class HardwareMotionProfile(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True, allow_inf_nan=False)

    schema_version: Literal[1]
    profile_id: str = Field(min_length=1)
    evidence_basis: str = ""
    operating_conditions: str = ""
    simulation_only: Literal[False]
    # Software evidence TTL; acceptance still checks original receipts.
    max_age: float = Field(default=0.5, gt=0)
    accel: float = Field(gt=0)
    decel: float = Field(gt=0)
    jerk: float = Field(gt=0)
    yaw_accel: float = Field(gt=0)
    yaw_decel: float = Field(gt=0)
    yaw_jerk: float = Field(gt=0)
    lag: float = Field(ge=0)
    # Product request ceiling, independently clamped by calibrated physical limits.
    requested_linear_mps: float = Field(default=25 / 18, gt=0)
    degraded_cap: float = Field(gt=0)
    yaw_cap: float = Field(default=2.0, gt=0)
    # Nominal project/vendor geometry, never an installation certificate.
    length: float = Field(default=0.674, gt=0)
    width: float = Field(default=0.552, gt=0)
    track_width_m: float = Field(default=0.481154, gt=0)
    wheel_speed_mps: float = Field(gt=0)
    stationary_linear_mps: float = Field(gt=0)
    stationary_yaw_rps: float = Field(gt=0)
    linear_error: float = Field(ge=0)
    yaw_error: float = Field(ge=0)
    sensor_error: float = Field(ge=0)
    position_error: float = Field(ge=0)
    model_seconds: float = Field(
        default=3600.0,
        gt=0,
        description="Startup model lease budget, seconds; not measured response",
    )
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
    lateral_stop_s: float = Field(ge=0)
    reaction_linear_accel: float = Field(ge=0)
    reaction_yaw_accel: float = Field(ge=0)
    watchdog_mode: Literal["verified", "unknown"] = "unknown"
    watchdog_basis: str = "Target firmware watchdog behavior unconfirmed"
    hardware_estop: HardwareEstopCapability = HardwareEstopCapability()
    perception_basis: str = "original raw lidar; hardware no-return remains unknown"
    risk_basis: str = ""
    brake_tracking_basis: str = ""
    # //Compatible: optional legacy declarations must match the single derived configuration.
    admission_config: AdmissionConfig | None = None
    temporal_config: TemporalConfig | None = None
    fusion_config: FusionConfig | None = None
    controller_config: ControllerConfig | None = None
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
        if self.model_seconds <= self.horizon_s or self.lateral_stop_s > self.horizon_s:
            raise ValueError(
                "model lease must cover search horizon and lateral settling must fit it"
            )
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
            (self.admission_config, self.admission()),
            (self.temporal_config, self.temporal()),
            (self.fusion_config, self.fusion()),
            (self.controller_config, self.controller()),
        ):
            if supplied is not None and supplied != derived:
                raise ValueError(
                    "nested configuration contradicts derived hardware policy; remove duplicate"
                )
        return self

    def reaction_acceleration(self, current: Velocity) -> Velocity:
        return Velocity(self.reaction_linear_accel, self.reaction_yaw_accel)

    def admission(self) -> AdmissionConfig:
        return AdmissionConfig(
            tuple(SourceRule(source, self.max_age, 0.4, 0.1, 2.0) for source in Source),
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
        return TemporalConfig(2.0, 512, 0.15, 0.15, 0.25, True)

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
            0.15,
            self.stationary_linear_mps - self.linear_error,
            self.stationary_yaw_rps - self.yaw_error,
            5,
            0.5,
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
    # //Compatible: older generated files carried manual draft/confirmation flags.
    # Ignore these obsolete inputs; they never change numeric control behavior.
    if isinstance(value, dict):
        for key in ("unvalidated", "geometry_confirmed"):
            value.pop(key, None)
        capability = value.get("hardware_estop")
        if isinstance(capability, dict):
            capability.pop("release_requires_controller_restart", None)
            capability.pop("release_interlock_basis", None)
    return HardwareMotionProfile.model_validate(value)
