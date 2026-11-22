"""Load explicit deployment numerics through one generic profile contract."""

from pathlib import Path

from senior_motion_control.hardware_profile import HardwareMotionProfile, load_hardware_profile


def select_motion_profile(*, hardware_path: str) -> HardwareMotionProfile:
    return load_hardware_profile(Path(hardware_path))


def configure_startup_learning(
    profile: HardwareMotionProfile, *, front_stop: bool, rear_stop: bool
) -> HardwareMotionProfile:
    """Apply the same typed occlusion overrides to every deployment."""
    if type(front_stop) is not bool or type(rear_stop) is not bool:
        raise ValueError("learning occlusion settings must be boolean")
    return profile.model_copy(
        update={
            "startup_learning_front_occlusion_stop": front_stop,
            "startup_learning_rear_occlusion_stop": rear_stop,
        }
    )
