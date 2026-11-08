"""Structural checks on the test-owned physics-only fixed stand."""

import math
import xml.etree.ElementTree as ET
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml
from motion_stand_geometry import load_wheel_geometry, wheel_ground_clearance

ROOT = Path(__file__).parents[2]


def test_wheel_clearance_uses_real_collision_geometry_and_chassis_attitude():
    geometry = load_wheel_geometry(ROOT / "ros2/src/senior_sim/urdf/senior.urdf.xacro")
    assert len(geometry.centers) == 4
    assert geometry.radius == pytest.approx(0.076)
    assert geometry.width == pytest.approx(0.06615)
    position = SimpleNamespace(x=0.0, y=0.0, z=0.06)
    level = SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)
    assert wheel_ground_clearance(position, level, geometry, -0.2) == pytest.approx(0.200471)
    roll = 0.2
    tilted = SimpleNamespace(x=math.sin(roll / 2), y=0.0, z=0.0, w=math.cos(roll / 2))
    # A tilted wheel has both its radial and axial extent projected onto z.
    assert wheel_ground_clearance(position, tilted, geometry, -0.2) < 0.16


def _normalized(element):
    return (
        element.tag,
        tuple(sorted(element.attrib.items())),
        (element.text or "").strip(),
        tuple(_normalized(child) for child in element),
    )


def test_fixed_stand_preserves_the_qualified_world_and_has_no_fake_sensor_or_tf():
    base = ET.parse(ROOT / "ros2/src/senior_sim/worlds/motion_qualification.sdf").getroot()
    fixed = ET.parse(ROOT / "tests/ros/motion_fixed_stand.sdf").getroot()
    world = fixed.find("world")
    assert world is not None
    assert world.findtext("gravity") == "0 0 0"
    gravity = world.find("gravity")
    assert gravity is not None
    world.remove(gravity)
    ground = world.find("./model[@name='ground']")
    assert ground is not None and ground.findtext("pose") == "0 0 -0.2 0 0 0"
    ground_pose = ground.find("pose")
    assert ground_pose is not None
    ground.remove(ground_pose)
    stand = world.find("./model[@name='motion_test_stand']")
    assert stand is not None
    assert stand.findtext("static") == "false"
    assert stand.find("./link[@name='anchor']") is not None
    assert float(stand.findtext("./link/inertial/mass", "0")) > 0
    assert all(
        float(stand.findtext(f"./link/inertial/inertia/{axis}", "0")) > 0
        for axis in ("ixx", "iyy", "izz")
    )
    assert stand.findtext("./joint[@name='world_anchor']/parent") == "world"
    assert stand.findtext("./joint[@name='world_anchor']/child") == "anchor"
    assert not stand.findall(".//collision")
    assert not stand.findall(".//sensor")
    plugin = stand.find("./plugin[@name='ignition::gazebo::systems::DetachableJoint']")
    assert plugin is not None
    assert plugin.attrib["filename"] == "ignition-gazebo-detachable-joint-system"
    assert plugin.findtext("parent_link") == "anchor"
    assert plugin.findtext("child_model") == "senior"
    assert plugin.findtext("child_link") == "base_link"
    assert plugin.findtext("detach_topic") == "/qualification/stand_detach"
    assert plugin.findtext("attach_topic") == "/qualification/stand_attach"
    assert plugin.findtext("output_topic") == "/qualification/stand_state"
    world.remove(stand)
    assert _normalized(fixed) == _normalized(base)

    profile = yaml.safe_load((ROOT / "tests/ros/motion_fixed_stand_profile.yaml").read_text())[
        "safety_gate"
    ]["ros__parameters"]
    assert profile == {
        "lateral_braking_deceleration": 0.3,
        "physical_cap_context": "fortress-fixed-stand-v1",
    }
