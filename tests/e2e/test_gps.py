"""GPS and passive heading may estimate pose but never publish velocity."""

import importlib.util
import inspect
from pathlib import Path

from conftest import SimulationProfile

from senior_control.localization.heading import HeadingObserver
from senior_control.models import Velocity


def test_gps_passive_heading_never_moves_robot(
    simulation_profile: SimulationProfile,
) -> None:
    signature = inspect.signature(HeadingObserver)
    assert "publisher" not in signature.parameters
    assert "cmd_vel" not in signature.parameters

    node_path = (
        Path(__file__).resolve().parents[2]
        / "ros2"
        / "src"
        / "senior_robot"
        / "senior_robot"
        / "localization_node.py"
    )
    spec = importlib.util.spec_from_file_location("acceptance_localization_node", node_path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    source = node_path.read_text(encoding="utf-8")
    assert "create_publisher(Twist" not in source
    assert "create_publisher(Velocity" not in source
    observer = HeadingObserver(min_baseline_m=5.0, max_correction_rad=0.35)
    assert not isinstance(observer, Velocity)
    assert len(simulation_profile.config_hash) == 64
