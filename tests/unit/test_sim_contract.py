"""Portable checks; ROS runtime tests remain a separate Linux gate."""

import importlib.util
from pathlib import Path

import yaml

from senior_control.config import DeviceProfile

ROOT = Path(__file__).resolve().parents[2]


def test_profiles_are_isolated_and_valid() -> None:
    profiles = [
        yaml.safe_load((ROOT / "config" / name).read_text())
        for name in ("simulation.yaml", "hardware.example.yaml")
    ]
    assert [DeviceProfile.model_validate(p).mode for p in profiles] == ["simulation", "hardware"]
    assert profiles[0]["topics"]["scan"] == "/scan"
    assert profiles[1]["topics"]["scan"] != profiles[0]["topics"]["scan"]


def test_environment_detection_never_starts_processes(monkeypatch) -> None:
    spec = importlib.util.spec_from_file_location(
        "detect_environment", ROOT / "tools/detect_environment.py"
    )
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module.shutil, "which", lambda _: None)
    result = module.detect_environment()
    assert result["ros"]["available"] is False
    assert result["topics"] is None


def test_simulation_contract_files_are_wired() -> None:
    import xml.etree.ElementTree as ET

    model = ET.parse(ROOT / "ros2/src/senior_sim/urdf/senior.urdf.xacro").getroot()
    joints = {joint.attrib["name"] for joint in model.findall(".//joint")}
    assert {"laser_joint", "camera_joint", "imu_joint"} <= joints
    plugins = {p.attrib.get("name") for p in model.findall(".//gazebo/plugin")}
    assert "gz_ros2_control::GazeboSimROS2ControlPlugin" in plugins
    assert "ignition::gazebo::systems::OdometryPublisher" in plugins
    controller = yaml.safe_load(
        (ROOT / "ros2/src/senior_sim/config/controllers.yaml").read_text(encoding="utf-8")
    )["drive"]["ros__parameters"]
    assert controller["open_loop"] is False
    assert 0 < controller["cmd_vel_timeout"] <= 0.3
    assert len(controller["left_wheel_names"]) == len(controller["right_wheel_names"]) == 2
    bridges = yaml.safe_load(
        (ROOT / "ros2/src/senior_sim/config/bridge.yaml").read_text(encoding="utf-8")
    )
    assert {"/clock", "/scan", "/imu/data", "/ground_truth/odom"} <= {
        b["ros_topic_name"] for b in bridges
    }
    assert all(b["direction"] == "GZ_TO_ROS" for b in bridges)
    assert not any(b["ros_topic_name"] == "/tf" for b in bridges)


def test_environment_reports_passive_topic_discovery(monkeypatch) -> None:
    from types import SimpleNamespace

    spec = importlib.util.spec_from_file_location(
        "detect_environment", ROOT / "tools/detect_environment.py"
    )
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(
        module.shutil, "which", lambda name: "/opt/ros/bin/ros2" if name == "ros2" else None
    )
    monkeypatch.setattr(
        module.subprocess, "run", lambda *args, **kwargs: SimpleNamespace(stdout="/scan\n/clock\n")
    )
    assert module.detect_environment()["topics"] == ["/clock", "/scan"]
