"""The shared definition reaches Python, native consumers and ROS parameter files."""

import json
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

import yaml

from vehicle_core.ros_wiring import TOPICS, matching_publisher, publisher

ROOT = Path(__file__).resolve().parents[2]


def test_single_definition_generates_native_and_parameter_wiring(tmp_path):
    definition = ROOT / "vehicle_core/ros_wiring.json"
    generator = ROOT / "tools/generate_ros_wiring.py"
    include = tmp_path / "ros_wiring.hpp"
    subprocess.run(
        [sys.executable, str(generator), "--definition", str(definition), "--output", str(include)],
        check=True,
    )
    node = publisher("scan_receipt")
    source = tmp_path / "consumer.cpp"
    source.write_text(
        '#include "ros_wiring.hpp"\n#include <iostream>\n'
        "int main() { using namespace senior_sensor_ingress::wiring; "
        'std::cout << safety_raw_scan << "\\n" << node_scan_receipt << "\\n" << ns_scan_receipt; }'
    )
    binary = tmp_path / "consumer"
    subprocess.run(["c++", "-std=c++17", str(source), "-o", str(binary)], check=True)
    assert subprocess.check_output([str(binary)], text=True).splitlines() == [
        TOPICS.safety_raw_scan,
        node.name,
        node.namespace,
    ]
    output = tmp_path / "slam.yaml"
    subprocess.run(
        [
            sys.executable,
            str(generator),
            "--definition",
            str(definition),
            "--output",
            str(output),
            "--template",
            str(ROOT / "ros2/src/senior_mapping/config/slam.yaml"),
        ],
        check=True,
    )
    params = yaml.safe_load(output.read_text())["slam_toolbox"]["ros__parameters"]
    assert params["qualified_scan_topic"] == TOPICS.senior_slam_qualified_scan
    assert params["scan_topic"] == "/scan"  # Official topic stays literal.
    # A renamed/namespaced internal node receives the same parameters.
    custom = json.loads(definition.read_text())
    custom["publishers"]["raw_receipt"] = {"name": "images", "namespace": "/robot1"}
    custom_definition = tmp_path / "custom.json"
    custom_definition.write_text(json.dumps(custom))
    subprocess.run(
        [
            sys.executable,
            str(generator),
            "--definition",
            str(custom_definition),
            "--output",
            str(output),
            "--template",
            str(ROOT / "ros2/src/senior_sim/config/simulation_rgbd.yaml"),
        ],
        check=True,
    )
    assert "/robot1/images" in yaml.safe_load(output.read_text())
    assert json.loads(definition.read_text())["topics"]["safety_raw_scan"] == TOPICS.safety_raw_scan


def test_shared_receiver_uses_name_namespace_type_and_not_gid():
    endpoint = SimpleNamespace(
        node_name="configured",
        node_namespace="/robot1",
        topic_type="sensor_msgs/msg/LaserScan",
        endpoint_gid=b"",
    )
    assert (
        matching_publisher([endpoint], "configured", "/robot1", "sensor_msgs/msg/LaserScan")
        is endpoint
    )
    assert matching_publisher([endpoint, endpoint], "configured", "/robot1") is None
    assert matching_publisher([endpoint], "configured", "/") is None
    assert matching_publisher([endpoint], "other", "/robot1") is None
    assert matching_publisher([endpoint], "configured", "/robot1", "wrong/type") is None
