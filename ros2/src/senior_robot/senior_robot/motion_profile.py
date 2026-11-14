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
    # //Compatible: keep the isolated candidate's original clock/domain admission.
    return select_simulation_profile(
        name, simulated_clock=simulated_clock, localhost_only=localhost_only, domain_id=domain_id
    )
