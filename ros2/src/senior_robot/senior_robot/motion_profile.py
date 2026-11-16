"""Select deployment inputs without changing the shared motion or safety owner."""

from pathlib import Path

from senior_control.motion.hardware_profile import HardwareMotionProfile, load_hardware_profile
from senior_robot.simulation_motion_profile import (
    SimulationMotionProfile,
    select_simulation_profile,
)


def select_motion_profile(
    name: str, *, hardware_path: str, simulated_clock: bool, localhost_only: str, domain_id: str
) -> HardwareMotionProfile | SimulationMotionProfile:
    if hardware_path:
        if simulated_clock or name:
            raise ValueError(
                "hardware profile requires use_sim_time=false and no simulation candidate"
            )
        return load_hardware_profile(Path(hardware_path))
    # Simulation uses the configured local domain and requires a simulated clock.
    return select_simulation_profile(
        name, simulated_clock=simulated_clock, localhost_only=localhost_only, domain_id=domain_id
    )


def configure_startup_learning(
    profile: HardwareMotionProfile | SimulationMotionProfile,
    *,
    front_stop: bool,
    rear_stop: bool,
) -> HardwareMotionProfile | SimulationMotionProfile:
    """Apply typed ROS overrides after selecting the file or simulation profile."""
    from dataclasses import replace

    if type(front_stop) is not bool or type(rear_stop) is not bool:
        raise ValueError("learning occlusion settings must be boolean")
    values = dict(
        startup_learning_front_occlusion_stop=front_stop,
        startup_learning_rear_occlusion_stop=rear_stop,
    )
    if isinstance(profile, HardwareMotionProfile):
        return profile.model_copy(update=values)
    return replace(
        profile,
        startup_learning_front_occlusion_stop=front_stop,
        startup_learning_rear_occlusion_stop=rear_stop,
    )
