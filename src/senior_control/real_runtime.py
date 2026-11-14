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


def load_real_parameters(path: Path) -> dict[str, dict[str, Any]]:
    if not path.is_absolute():
        raise ValueError("safety_parameters must be an absolute ROS parameter YAML path")
    value = yaml.safe_load(path.read_bytes())
    if not isinstance(value, dict):
        raise ValueError("safety_parameters must contain per-node ros__parameters")
    result = {}
    errors = []
    for node, fields in REQUIRED_PARAMETERS.items():
        entry = value.get(node, {})
        params = entry.get("ros__parameters", {}) if isinstance(entry, dict) else {}
        if not isinstance(params, dict):
            params = {}
        result[node] = params
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
    for node in ("scan_adapter", "safety_gate"):
        if result[node].get("self_mask_calibrated") is not True:
            errors.append(f"{node}.self_mask_calibrated requires actual mounting calibration")
    for node in ("scan_adapter", "safety_scan_ingress", "scan_motion_node", "safety_gate"):
        if result[node].get("transform_static") is not True:
            errors.append(f"{node}.transform_static requires a verified rigid lidar mount")
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
            params = load_real_parameters(safety_parameters)
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
