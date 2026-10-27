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


def test_camera_forward_ray_is_clear_of_chassis_visuals() -> None:
    import xml.etree.ElementTree as ET

    model = ET.parse(ROOT / "ros2/src/senior_sim/urdf/senior.urdf.xacro").getroot()
    camera_joint = model.find("./joint[@name='camera_joint']/origin")
    assert camera_joint is not None
    x, y, z = map(float, camera_joint.attrib["xyz"].split())
    for visual in model.findall("./link[@name='base_link']/visual"):
        origin = visual.find("origin")
        size = visual.find("geometry/box")
        assert origin is not None and size is not None
        center = tuple(map(float, origin.attrib["xyz"].split()))
        dimensions = tuple(map(float, size.attrib["size"].split()))
        intersects = (
            abs(y - center[1]) <= dimensions[1] / 2
            and abs(z - center[2]) <= dimensions[2] / 2
            and x <= center[0] + dimensions[0] / 2
        )
        assert not intersects, "forward camera ray intersects chassis visual"


def test_baseline_has_visible_obstacle() -> None:
    import xml.etree.ElementTree as ET

    world = ET.parse(ROOT / "ros2/src/senior_sim/worlds/baseline.sdf").getroot()
    obstacle = world.find("./world/model[@name='test_obstacle']")
    assert obstacle is not None
    assert obstacle.find("./link/visual/geometry/box/size") is not None


def test_drive_remapping_uses_humble_plugin_configuration() -> None:
    import xml.etree.ElementTree as ET

    model = ET.parse(ROOT / "ros2/src/senior_sim/urdf/senior.urdf.xacro").getroot()
    plugin = model.find("./gazebo/plugin[@name='gz_ros2_control::GazeboSimROS2ControlPlugin']")
    assert plugin is not None
    remaps = {item.text for item in plugin.findall("./ros/remapping")}
    assert {"/drive/cmd_vel_unstamped:=/cmd_vel", "/drive/odom:=/wheel/odom"} <= remaps
    launch = (ROOT / "ros2/src/senior_sim/launch/simulation.launch.py").read_text(encoding="utf-8")
    assert "--controller-ros-args" not in launch


def test_environment_package_versions_from_ament_prefix(tmp_path, monkeypatch) -> None:
    spec = importlib.util.spec_from_file_location(
        "detect_environment", ROOT / "tools/detect_environment.py"
    )
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    package = tmp_path / "share" / "controller_manager"
    package.mkdir(parents=True)
    (package / "package.xml").write_text(
        "<package><name>controller_manager</name><version>2.1.0</version></package>",
        encoding="utf-8",
    )
    monkeypatch.setenv("AMENT_PREFIX_PATH", str(tmp_path))
    monkeypatch.setattr(module.shutil, "which", lambda _: None)
    result = module.detect_environment()
    assert result["packages"]["controller_manager"]["version"] == "2.1.0"
    assert result["packages"]["gz_ros2_control"]["available"] is False
    assert result["matrix"]["runtime_ready"] is False
