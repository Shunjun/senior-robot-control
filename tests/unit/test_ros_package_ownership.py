"""Verify the delivered package boundary and unchanged public ROS payload definitions."""

import ast
import hashlib
import json
from pathlib import Path
from xml.etree import ElementTree

ROOT = Path(__file__).resolve().parents[2]
ROS = ROOT / "ros2/src"
EXPECTED = {
    "senior_interfaces",
    "senior_sensor_ingress",
    "senior_motion_estimation",
    "senior_perception",
    "senior_motion_control",
    "senior_localization",
    "senior_mapping",
    "senior_navigation",
    "senior_robot",
    "senior_sim",
}
RETIRED = {"senior_motion_msgs", "senior_slam_bridge", "senior_scan_matcher"}


def test_ros_packages_have_functional_owners_without_composition_cycles():
    manifests = {
        ElementTree.parse(p).getroot().findtext("name"): p for p in ROS.glob("*/package.xml")
    }
    assert set(manifests) == EXPECTED
    graph = {}
    for name, path in manifests.items():
        tree = ElementTree.parse(path).getroot()
        graph[name] = {item.text for item in tree if item.tag.endswith("depend")} & EXPECTED

    def visit(name, active, done):
        assert name not in active, f"package cycle at {name}: {active}"
        if name in done:
            return
        for dependency in graph[name]:
            visit(dependency, active | {name}, done)
        done.add(name)

    done = set()
    for name in graph:
        visit(name, set(), done)
    for package in EXPECTED - {"senior_robot", "senior_sim", "senior_interfaces"}:
        for path in (ROS / package).rglob("*.py"):
            if "test" in path.parts:
                continue
            for node in ast.walk(ast.parse(path.read_text())):
                modules = (
                    [node.module or ""]
                    if isinstance(node, ast.ImportFrom)
                    else (
                        [item.name for item in node.names] if isinstance(node, ast.Import) else []
                    )
                )
                if package == "senior_perception":
                    assert all(
                        module.split(".")[0]
                        not in {"senior_motion_control", "senior_motion_estimation"}
                        for module in modules
                    ), path
                assert all(
                    module.split(".")[0] not in {"senior_robot", "senior_sim", *RETIRED}
                    for module in modules
                ), path
    for name in RETIRED:
        assert not (ROS / name).exists()
    for name in [
        "vehicle_speed",
        "vehicle_arbitration",
        "motion_core_output",
        "safety_node",
        "motion_core_perception",
        "vehicle_compat",
        "camera_source",
    ]:
        assert not (ROS / "senior_robot/senior_robot" / f"{name}.py").exists()


def test_consolidated_interfaces_preserve_all_message_and_service_fields():
    expected = json.loads((ROOT / "tests/fixtures/ros_interface_fields.json").read_text())
    actual = {}
    for kind in ("msg", "srv"):
        for path in (ROS / "senior_interfaces" / kind).glob(f"*.{kind}"):
            lines = [line.split("#", 1)[0].strip() for line in path.read_text().splitlines()]
            normalized = "\n".join(line for line in lines if line)
            actual[f"{kind}/{path.name}"] = hashlib.sha256(normalized.encode()).hexdigest()
    assert actual == expected
