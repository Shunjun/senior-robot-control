"""Generate isolated calibration artifacts; never launch or send motion.

Run with ``python -m tools.vehicle_motion_fixture`` from the frozen source root.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import shlex
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

from tools.vehicle_motion_calibrate import CalibrationPlan


def _xml_signature(node: ET.Element) -> tuple[object, ...]:
    return (
        node.tag,
        tuple(sorted(node.attrib.items())),
        (node.text or "").strip(),
        tuple(_xml_signature(child) for child in node),
    )


def verify_scene_only(base: ET.Element, reviewed: ET.Element) -> list[str]:
    """Allow additional static box landmarks; retain every original world element."""
    before, after = base.find("world"), reviewed.find("world")
    if before is None or after is None:
        raise ValueError("one original world required")
    names = {node.get("name") for node in before.findall("model")}
    additions = [node for node in after.findall("model") if node.get("name") not in names]
    allowed = {"model", "static", "pose", "link", "collision", "visual", "geometry", "box", "size"}
    added_names = []
    for model in additions:
        name = model.get("name", "")
        if (
            not name.startswith("candidate_diagonal_")
            or model.findtext("static") != "true"
            or any(node.tag not in allowed for node in model.iter())
        ):
            raise ValueError("reviewed scene permits static box landmarks only")
        added_names.append(name)
    if len(set(added_names)) != len(added_names):
        raise ValueError("duplicate scene model")
    retained = copy.deepcopy(reviewed)
    world = retained.find("world")
    assert world is not None
    for model in list(world.findall("model")):
        if model.get("name") in added_names:
            world.remove(model)
    if _xml_signature(base) != _xml_signature(retained):
        raise ValueError("world physics, plugins and original models must remain unchanged")
    return added_names


def fixture_landmarks(
    plan: CalibrationPlan, corridor_half_length_m: float | None, landmark_radius_m: float
) -> tuple[list[tuple[float, float, float, float]], float]:
    """Construct reviewed synthetic geometry, never infer physical braking bounds."""
    guard = plan.hard_seconds * plan.max_linear_mps + 3.0
    if corridor_half_length_m is not None:
        length = corridor_half_length_m
        if any(s.wz != 0 for s in plan.segments):
            raise ValueError("corridor fixture accepts straight commands only")
        if not math.isfinite(length) or not guard < length - 0.1 or length > 100:
            raise ValueError("corridor end must exceed nominal watchdog travel plus 3 m guard")
        walls = [(length, 0.0, 0.2, 10.0), (-length, 0.0, 0.2, 10.0)]
        # Nonuniform paired features stay inside the matcher's 8 m range along the path.
        for x in range(-math.floor(length) + 2, math.floor(length) - 1, 3):
            walls.extend([(float(x), 4.5, 0.4, 0.6), (x + 0.7, -4.7, 0.7, 0.4)])
        return walls, 2 * length + 20.0
    if not math.isfinite(landmark_radius_m) or not guard < landmark_radius_m - 0.1:
        raise ValueError("review a larger landmark fixture for this stimulus")
    r = landmark_radius_m
    return [(r, 0.0, 0.2, 6.0), (-r, 0.5, 0.2, 5.0), (-1.0, r, 4.0, 0.2), (1.0, -r, 5.0, 0.2)], max(
        50.0, 2 * r + 20.0
    )


def generate_fixture(
    source_root: Path,
    output: Path,
    plan_path: Path,
    *,
    hard_seconds: float = 8.0,
    corridor_half_length_m: float | None = None,
    landmark_radius_m: float = 6.0,
    reviewed_world: Path | None = None,
    reviewed_world_sha256: str | None = None,
) -> dict[str, object]:
    source_root = source_root.resolve()
    if (reviewed_world is None) != (reviewed_world_sha256 is None):
        raise ValueError("reviewed world and exact SHA256 are required together")
    scene_bytes = reviewed_world.read_bytes() if reviewed_world is not None else None
    if scene_bytes is not None and hashlib.sha256(scene_bytes).hexdigest() != reviewed_world_sha256:
        raise ValueError("reviewed world SHA256 mismatch")
    plan = CalibrationPlan.from_dict(json.loads(plan_path.read_text()), hard_seconds=hard_seconds)
    walls, ground_size = fixture_landmarks(plan, corridor_half_length_m, landmark_radius_m)
    output = output.resolve()
    output.mkdir(mode=0o700)  # Never overwrite a previously reviewed fixture.
    sim = source_root / "ros2/src/senior_sim"
    model_source = sim / "urdf/senior.urdf.xacro"
    model_bytes = model_source.read_bytes()
    ET.register_namespace("xacro", "http://www.ros.org/wiki/xacro")
    model = ET.fromstring(model_bytes)
    replaced = 0
    for mapping in model.iter("remapping"):
        if mapping.text == "/drive/cmd_vel_unstamped:=/cmd_vel":
            mapping.text = "/drive/cmd_vel_unstamped:=" + plan.target_topic
            replaced += 1
    if replaced != 1:
        raise ValueError("expected exactly one source actuator remapping")
    # Synthetic friction is explicit. Geometry/mass/inertia stay source-identical.
    for name in ("left_front", "left_back", "right_front", "right_back"):
        gazebo = ET.SubElement(model, "gazebo", reference=f"{name}_wheel")
        ET.SubElement(gazebo, "mu1").text = "1.0"
        ET.SubElement(gazebo, "mu2").text = "1.0"
    ET.ElementTree(model).write(output / "model.xacro", encoding="unicode")
    world_source = sim / "worlds/baseline.sdf"
    world_tree = ET.parse(world_source)
    world = world_tree.find("world")
    assert world is not None
    world.set("name", plan.world)
    for obstacle in list(world.findall("model")):
        if obstacle.get("name") != "ground":
            world.remove(obstacle)
    ground = world.find("model[@name='ground']/link/collision")
    assert ground is not None
    ode = ET.SubElement(ET.SubElement(ET.SubElement(ground, "surface"), "friction"), "ode")
    ET.SubElement(ode, "mu").text = ET.SubElement(ode, "mu2").text = "1.0"
    for plane in world.findall("model[@name='ground']/link/*/geometry/plane/size"):
        plane.text = f"{ground_size} {ground_size}"
    for index, (x, y, width, length) in enumerate(walls):
        wall = ET.SubElement(world, "model", name=f"landmark_{index}")
        ET.SubElement(wall, "static").text = "true"
        ET.SubElement(wall, "pose").text = f"{x} {y} 1 0 0 0"
        link = ET.SubElement(wall, "link", name="link")
        for kind in ("collision", "visual"):
            item = ET.SubElement(link, kind, name=kind)
            ET.SubElement(
                ET.SubElement(ET.SubElement(item, "geometry"), "box"), "size"
            ).text = f"{width} {length} 2"
    additions = []
    if scene_bytes is not None:
        additions = verify_scene_only(world_tree.getroot(), ET.fromstring(scene_bytes))
        (output / "world.sdf").write_bytes(scene_bytes)
    else:
        world_tree.write(output / "world.sdf", encoding="unicode")
    controllers = yaml.safe_load((sim / "config/controllers.yaml").read_text())
    for node in ("drive", "controller_manager"):
        controllers[node]["ros__parameters"]["use_sim_time"] = True
    (output / "controllers.yaml").write_text(yaml.safe_dump(controllers))
    (output / "bridge.yaml").write_bytes((sim / "config/bridge.yaml").read_bytes())
    params = source_root / "ros2/src/senior_robot/config/simulation_motion.yaml"
    scan_parameters = yaml.safe_load(params.read_text())
    search_speed = max(0.5, plan.max_linear_mps * 1.1)
    scan_parameters["scan_motion_node"]["ros__parameters"]["max_linear_speed"] = search_speed
    (output / "scan-motion.yaml").write_text(yaml.safe_dump(scan_parameters))
    params = output / "scan-motion.yaml"
    partition = f"vehicle_motion_calibration_{plan.run_id}"
    launch = f"""# Generated calibration-only composition; no product motion source.
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', {str(plan.ros_domain_id)!r}),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),
        SetEnvironmentVariable('IGN_PARTITION', {partition!r}),
        SetEnvironmentVariable('GZ_PARTITION', {partition!r}),
        ExecuteProcess(cmd=['ign','gazebo','-r','-s',{str(output / "world.sdf")!r}]),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{{'use_sim_time': True, 'robot_description': ParameterValue(Command([
                 'xacro ', {shlex.quote(str(output / "model.xacro"))!r},
                 ' controllers_file:=', {shlex.quote(str(output / "controllers.yaml"))!r},
                 ' rear_attachment:=false']), value_type=str)}}]),
        Node(package='ros_gz_sim', executable='create', arguments=[
             '-name','senior','-topic','robot_description','-z','0.06']),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='senior_sim_bridge',
             parameters=[{{'config_file': {str(output / "bridge.yaml")!r}}}]),
        Node(package='controller_manager', executable='spawner', arguments=[
             'joint_state_broadcaster','--controller-manager','/controller_manager']),
        Node(package='controller_manager', executable='spawner', arguments=[
             'drive','--controller-manager','/controller_manager']),
        Node(package='senior_sensor_ingress', executable='scan_receipt_node',
             parameters=[{str(params)!r}]),
        Node(package='senior_robot', executable='scan_adapter', parameters=[{str(params)!r}]),
        Node(package='senior_scan_matcher', executable='scan_motion_node',
             parameters=[{str(params)!r}]),
    ])
"""
    (output / "calibration.launch.py").write_text(launch)
    (output / "start.sh").write_text(
        '#!/bin/bash\nset -euo pipefail\numask 0077\nsource "$1"\n'
        f'export PYTHONPATH={shlex.quote(str(source_root / "src"))}:"${{PYTHONPATH:-}}"\n'
        f"export ROS_DOMAIN_ID={plan.ros_domain_id} ROS_LOCALHOST_ONLY=1\n"
        f"export IGN_PARTITION={shlex.quote(partition)} GZ_PARTITION={shlex.quote(partition)}\n"
        f"export ROS_HOME={shlex.quote(str(output / 'ros-home'))}\n"
        'export ROS_LOG_DIR="$ROS_HOME/log"\n'
        'mkdir -p "$ROS_LOG_DIR"\n'
        # //Compatible: direct Fortress launch needs the plugin path set by ros_gz_sim's helper.
        'export IGN_GAZEBO_SYSTEM_PLUGIN_PATH="${IGN_GAZEBO_SYSTEM_PLUGIN_PATH:-}:'
        '${LD_LIBRARY_PATH:-}"\n'
        'export GZ_SIM_SYSTEM_PLUGIN_PATH="${GZ_SIM_SYSTEM_PLUGIN_PATH:-}:${LD_LIBRARY_PATH:-}"\n'
        "export LIBGL_ALWAYS_SOFTWARE=1\n"
        f"exec xvfb-run -a ros2 launch {shlex.quote(str(output / 'calibration.launch.py'))}\n"
    )
    manifest: dict[str, object] = {
        "generator_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "source_model_sha256": hashlib.sha256(model_bytes).hexdigest(),
        "source_world_sha256": hashlib.sha256(world_source.read_bytes()).hexdigest(),
        "plan_sha256": hashlib.sha256(plan_path.read_bytes()).hexdigest(),
        "domain": plan.ros_domain_id,
        "partition": partition,
        "target_topic": plan.target_topic,
        "physics": "baseline.sdf 1ms step; ground/wheel mu=mu2=1 synthetic; source mass/inertia",
        "landmarks": walls,
        "hard_seconds": hard_seconds,
        "nominal_watchdog_travel_bound_m": hard_seconds * plan.max_linear_mps,
        "corridor_half_length_m": corridor_half_length_m,
        "landmark_radius_m": landmark_radius_m,
        "matcher_linear_search_mps": search_speed,
        "calibration_executed": False,
        "limits": "low-speed fixture only; contact/response/scan qualification unmeasured",
        "files": {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in output.iterdir()},
    }
    if scene_bytes is not None:
        manifest["reviewed_scene"] = {
            "source": str(reviewed_world),
            "sha256": reviewed_world_sha256,
            "added_static_models": additions,
            "retained_physics_plugins_and_original_models": True,
        }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    return manifest


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--plan", type=Path, required=True)
    parser.add_argument("--hard-seconds", type=float, default=8.0)
    parser.add_argument("--corridor-half-length-m", type=float)
    parser.add_argument("--landmark-radius-m", type=float, default=6.0)
    parser.add_argument("--reviewed-world", type=Path)
    parser.add_argument("--reviewed-world-sha256")
    args = parser.parse_args()
    print(
        json.dumps(
            generate_fixture(
                args.source_root,
                args.output,
                args.plan,
                hard_seconds=args.hard_seconds,
                corridor_half_length_m=args.corridor_half_length_m,
                landmark_radius_m=args.landmark_radius_m,
                reviewed_world=args.reviewed_world,
                reviewed_world_sha256=args.reviewed_world_sha256,
            ),
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
