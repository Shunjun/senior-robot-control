"""Read-only preflight for the real CLI/ROS composition; never starts hardware."""

from pathlib import Path
from typing import Any

import yaml
from pydantic import ValidationError

from senior_control.motion.hardware_profile import HardwareMotionProfile, load_hardware_profile

# Required parameters have no hardware defaults. Sensor and response calibration
# belongs in this file, not in a simulation YAML or product driving preferences.
REQUIRED_PARAMETERS = {
    "scan_receipt": ("scan_topic", "scan_ledger_max_age_s"),
    "scan_adapter": ("body_frame", "max_age_seconds", "self_mask_calibrated", "transform_static"),
    "safety_scan_ingress": ("body_frame", "observation_max_age", "transform_static"),
    "scan_motion_node": (
        "body_frame",
        "scan_max_age",
        "transform_static",
        "max_interval",
        "max_linear_speed",
        "max_yaw_rate",
        "range_limit",
        "min_response",
        "max_residual",
        "min_inlier_fraction",
        "min_spread",
        "min_translation_observability",
        "min_yaw_observability",
        "max_variance",
        "reanchor_distance",
        "reanchor_yaw",
        "min_rays",
    ),
    "motion_fusion": (
        "body_frame",
        "wheel_frame",
        "imu_frame",
        "sensor_max_age",
        "alignment_max_gap",
        "pending_wait",
        "max_variance",
        "imu_zero_variance",
        "min_quality",
        "max_yaw_error",
        "startup_window",
        "wheel_motion_limit",
        "stall_distance",
        "lateral_limit",
        "yaw_limit",
        "progress_stationary_max_linear_speed",
        "progress_stationary_max_yaw_rate",
    ),
    "safety_gate": (
        "body_frame",
        "simulation_rate_hz",
        "footprint_xy",
        "self_mask_calibrated",
        "transform_static",
        "observation_max_age",
        "measurement_max_age",
        "hard_margin",
        "depth_policy",
        "command_ttl_seconds",
    ),
    "command_node": ("heartbeat_seconds", "disconnect_seconds", "command_ttl_seconds"),
}


def real_parameter_defaults(
    profile: HardwareMotionProfile | None = None,
) -> dict[str, dict[str, Any]]:
    """Software policies, separate from measured response/error bounds.

    Matcher acceptance/recovery settings reuse the existing software pipeline;
    they do not certify covariance, braking, installation, or sensor coverage.
    Nominal frames are from the WHEELTEC driver/URDF and require site verification.
    """
    age = profile.max_age if profile else 0.5
    common = {"use_sim_time": False, "body_frame": "base_link"}
    return {
        "scan_receipt": {
            "use_sim_time": False,
            "scan_topic": "/scan",
            "scan_ledger_max_age_s": 3.0,
        },
        "scan_adapter": {
            **common,
            "max_age_seconds": age,
            "self_mask_calibrated": False,
            "transform_static": False,
            "no_return_clear": False,
        },
        "safety_scan_ingress": {**common, "observation_max_age": age, "transform_static": False},
        "scan_motion_node": {
            **common,
            "scan_max_age": age,
            "transform_static": False,
            "max_interval": 0.4,
            "max_linear_speed": profile.assumed_body_cap if profile else None,
            "max_yaw_rate": profile.yaw_cap if profile else None,
            "range_limit": 8.0,
            "min_response": 0.3,
            "max_residual": 0.09,
            "min_inlier_fraction": 0.55,
            "min_spread": 0.04,
            "min_translation_observability": 0.05,
            "min_yaw_observability": 0.01,
            "max_variance": 0.05,
            "reanchor_distance": 0.12,
            "reanchor_yaw": 0.1,
            "min_rays": 100,
        },
        "motion_fusion": {
            **common,
            "wheel_frame": "odom",
            "imu_frame": "gyro_link",
            "sensor_max_age": age,
            "alignment_max_gap": 0.25,
            "pending_wait": 0.25,
            "max_variance": 0.1,
            "imu_zero_variance": 0.01,
            "min_quality": 0.3,
            "max_yaw_error": 0.12,
            "startup_window": 0.6,
            "wheel_motion_limit": 0.004,
            "stall_distance": 0.04,
            "lateral_limit": 0.04,
            "yaw_limit": 0.12,
            "progress_stationary_max_linear_speed": profile.stationary_linear_mps
            if profile
            else None,
            "progress_stationary_max_yaw_rate": profile.stationary_yaw_rps if profile else None,
        },
        "safety_gate": {
            **common,
            "simulation_rate_hz": 20.0,
            "self_mask_calibrated": False,
            "transform_static": False,
            "observation_max_age": age,
            "measurement_max_age": age,
            "hard_margin": 0.0,
            "depth_policy": "laser_only",
            "command_ttl_seconds": 0.3,
            "footprint_xy": (
                [
                    -profile.length / 2,
                    -profile.width / 2,
                    profile.length / 2,
                    -profile.width / 2,
                    profile.length / 2,
                    profile.width / 2,
                    -profile.length / 2,
                    profile.width / 2,
                ]
                if profile
                else None
            ),
        },
        "command_node": {
            "use_sim_time": False,
            "heartbeat_seconds": 0.2,
            "disconnect_seconds": 2.0,
            "command_ttl_seconds": 0.3,
        },
    }


def load_real_parameters(
    path: Path, profile: HardwareMotionProfile | None = None
) -> dict[str, dict[str, Any]]:
    if not path.is_absolute():
        raise ValueError("safety_parameters must be an absolute ROS parameter YAML path")
    value = yaml.safe_load(path.read_bytes())
    if not isinstance(value, dict):
        raise ValueError("safety_parameters must contain per-node ros__parameters")
    result = {}
    defaults = real_parameter_defaults(profile)
    errors = []
    for node, fields in REQUIRED_PARAMETERS.items():
        entry = value.get(node, {})
        params = entry.get("ros__parameters", {}) if isinstance(entry, dict) else {}
        if not isinstance(params, dict):
            params = {}
        params = {**defaults[node], **params}
        result[node] = params
        # Response-related duplicates must retain the model's single value.
        if profile is not None:
            owned = {
                "scan_adapter": ("max_age_seconds",),
                "safety_scan_ingress": ("observation_max_age",),
                "scan_motion_node": ("scan_max_age", "max_linear_speed", "max_yaw_rate"),
                "motion_fusion": (
                    "sensor_max_age",
                    "progress_stationary_max_linear_speed",
                    "progress_stationary_max_yaw_rate",
                ),
                "safety_gate": ("observation_max_age", "measurement_max_age"),
            }
            for field in owned.get(node, ()):
                if params.get(field) != defaults[node][field]:
                    errors.append(
                        f"{node}.{field}: modify the owning hardware profile, not a duplicate"
                    )
        if params.get("use_sim_time") is not False:
            errors.append(f"{node}.use_sim_time must be false")
        for key in fields:
            if key not in params or params[key] is None:
                errors.append(f"{node}.{key} missing")
        if params.get("motion_core_profile") or params.get("hardware_motion_profile"):
            errors.append(
                f"{node}: select the motion model only through config.json.motion_profile"
            )
    # The current core perception path consumes laser patches only. Do not
    # silently claim required depth is included in the T4 sweep.
    if result["safety_gate"].get("depth_policy") != "laser_only":
        errors.append("safety_gate.depth_policy: shared core currently supports laser_only")
    if result["safety_gate"].get("output_topic", "/cmd_vel") != "/cmd_vel":
        errors.append("safety_gate.output_topic must stay /cmd_vel (sole actuator owner)")
    for node in ("command_node", "safety_gate"):
        p = result[node]
        for key, expected in (
            ("command_topic", "/safety/command_in"),
            ("final_command_topic", "/safety/final_command"),
        ):
            if p.get(key, expected) != expected:
                errors.append(f"{node}.{key} must stay {expected}")
    if errors:
        raise ValueError("Real ROS configuration incomplete:\n" + "\n".join(errors))
    return result


def validate_real_runtime(
    motion_profile: Path | None, safety_parameters: Path | None
) -> HardwareMotionProfile:
    errors = []
    profile = None
    params = None
    if motion_profile is None:
        errors.append("motion_profile: absolute path to a measured hardware response JSON required")
    else:
        try:
            profile = load_hardware_profile(motion_profile)
        except ValidationError as exc:
            errors.extend(
                "motion_profile." + ".".join(map(str, e["loc"])) + ": " + e["msg"]
                for e in exc.errors(include_input=False)
            )
        except (OSError, ValueError) as exc:
            errors.append(f"motion_profile: {exc}")
    if safety_parameters is None:
        errors.append("safety_parameters: absolute path to calibrated per-node ROS YAML required")
    else:
        try:
            params = load_real_parameters(safety_parameters, profile)
        except (OSError, ValueError, yaml.YAMLError) as exc:
            errors.append(f"safety_parameters: {exc}")
    if profile is not None and params is not None:
        footprint = params["safety_gate"].get("footprint_xy")
        if (
            not isinstance(footprint, list)
            or len(footprint) < 6
            or len(footprint) % 2
            or len(footprint) > 128
        ):
            errors.append("safety_gate.footprint_xy: bounded polygon coordinate pairs required")
        else:
            try:
                import math

                if not all(math.isfinite(float(x)) for x in footprint):
                    raise ValueError("nonfinite geometry")
                if any(abs(float(x)) > profile.length / 2 for x in footprint[::2]) or any(
                    abs(float(y)) > profile.width / 2 for y in footprint[1::2]
                ):
                    errors.append(
                        "hardware length/width must enclose the full calibrated footprint"
                    )
            except (ValueError, TypeError, OverflowError):
                errors.append("safety_gate.footprint_xy must contain finite numeric coordinates")
    if errors:
        raise ValueError("Real motion configuration rejected:\n" + "\n".join(errors))
    assert profile is not None
    return profile


def validate_navigation(reference: Path | None, parameters: Path | None) -> None:
    if reference is None and parameters is None:
        return  # Manual-only is valid; navigation remains explicitly unavailable.
    if reference is None or parameters is None:
        raise ValueError("navigation_reference and navigation_parameters must be supplied together")
    from senior_control.hardware_navigation import load_navigation_reference

    load_navigation_reference(reference)
    if not parameters.is_absolute():
        raise ValueError("navigation_parameters must be absolute")
    values = yaml.safe_load(parameters.read_bytes())
    for node in ("controller_server", "planner_server", "bt_navigator"):
        if not isinstance(values, dict) or not isinstance(values.get(node), dict):
            raise ValueError(f"navigation_parameters.{node} is required")
        params = values[node].get("ros__parameters", {})
        if not isinstance(params, dict) or params.get("use_sim_time") is not False:
            raise ValueError(f"navigation_parameters.{node}.use_sim_time must be false")
