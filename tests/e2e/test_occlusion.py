"""Scenario assets are inert fixtures with distinct Ignition world names."""

from pathlib import Path
from xml.etree import ElementTree

from conftest import SimulationProfile


def test_acceptance_scenes_have_distinct_world_names(
    simulation_profile: SimulationProfile,
) -> None:
    root = Path(__file__).resolve().parents[2] / "ros2" / "src" / "senior_sim" / "worlds"
    expected = {
        "wall_escape.sdf": "wall_escape",
        "narrow_corridor.sdf": "narrow_corridor",
        "dynamic_box.sdf": "dynamic_box",
        "ambiguous_localization.sdf": "ambiguous_localization",
    }
    found: set[str] = set()
    for filename, world_name in expected.items():
        document = ElementTree.parse(root / filename)
        sdf = document.getroot()
        assert sdf.attrib["version"] == "1.7"
        world = sdf.find("world")
        assert world is not None
        assert world.attrib["name"] == world_name
        found.add(world.attrib["name"])
        plugins = {plugin.attrib["filename"] for plugin in world.findall("plugin")}
        assert "ignition-gazebo-physics-system" in plugins
        assert not world.findall("include"), "acceptance scenes must not launch a robot"
    assert len(found) == len(expected)
    assert simulation_profile.source.name == "simulation.yaml"
