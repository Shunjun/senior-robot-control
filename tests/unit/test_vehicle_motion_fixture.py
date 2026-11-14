"""Generated calibration artifacts preserve source and isolate both transports."""

import hashlib
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from tools.vehicle_motion_fixture import generate_fixture

ROOT = Path(__file__).parents[2]
PLAN = ROOT / "tests/fixtures/vehicle_motion_calibration/plan.json"


def test_fixture_has_dedicated_actuation_and_reviewable_physics(tmp_path):
    source = ROOT / "ros2/src/senior_sim/urdf/senior.urdf.xacro"
    before = hashlib.sha256(source.read_bytes()).hexdigest()
    output = tmp_path / "fixture"
    manifest = generate_fixture(ROOT, output, PLAN)
    assert hashlib.sha256(source.read_bytes()).hexdigest() == before
    assert manifest["source_model_sha256"] == before
    assert manifest["domain"] == 173
    model = ET.parse(output / "model.xacro")
    mappings = [node.text for node in model.iter("remapping")]
    assert "/drive/cmd_vel_unstamped:=/vehicle_motion_calibration/review1/cmd_vel" in mappings
    assert "/drive/cmd_vel_unstamped:=/cmd_vel" not in mappings
    world = ET.parse(output / "world.sdf")
    assert world.find("world").attrib["name"] == "vehicle_motion_calibration"
    assert float(world.find(".//collision/surface/friction/ode/mu").text) == 1.0
    launch = (output / "calibration.launch.py").read_text()
    assert "IGN_PARTITION" in launch and "GZ_PARTITION" in launch
    assert "vehicle_motion_calibration_review1" in launch
    assert str(ROOT / "src") in (output / "start.sh").read_text()
    assert "scan_motion_node" in launch and "scan_receipt_node" in launch
    assert "safety_node" not in launch and "command_node" not in launch
    assert json.loads((output / "manifest.json").read_text())["calibration_executed"] is False
    with pytest.raises(FileExistsError):
        generate_fixture(ROOT, output, PLAN)


def test_extended_fixture_has_explicit_deadline_corridor_and_scan_search(tmp_path):
    data = json.loads(PLAN.read_text())
    data.update(run_id="high_forward", max_linear_mps=25 / 18)
    data["target_topic"] = "/vehicle_motion_calibration/high_forward/cmd_vel"
    data["field_bounds_m"] = [-45.0, -45.0, 45.0, 45.0]
    data["segments"] = [
        {"seconds": 2.0, "vx": 0.0, "wz": 0.0},
        {"seconds": 7.0, "vx": 25 / 18, "wz": 0.0},
        {"seconds": 6.0, "vx": 0.0, "wz": 0.0},
    ]
    path = tmp_path / "high.json"
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError):
        generate_fixture(ROOT, tmp_path / "default", path)
    fixture = tmp_path / "extended"
    m = generate_fixture(ROOT, fixture, path, hard_seconds=20.0, corridor_half_length_m=40.0)
    assert m["hard_seconds"] == 20.0
    assert m["corridor_half_length_m"] == 40.0
    assert m["matcher_linear_search_mps"] >= 25 / 18
    assert m["nominal_watchdog_travel_bound_m"] == pytest.approx(20 * 25 / 18)
    world = ET.parse(fixture / "world.sdf")
    assert world.find(".//plane/size").text == "100.0 100.0"
    assert len(m["landmarks"]) > 20  # Nearby asymmetric landmarks, not distant empty walls.
    assert "IGN_GAZEBO_SYSTEM_PLUGIN_PATH" in (fixture / "start.sh").read_text()
    assert str(fixture / "scan-motion.yaml") in (fixture / "calibration.launch.py").read_text()
    with pytest.raises(ValueError, match="corridor"):
        generate_fixture(
            ROOT, tmp_path / "small", path, hard_seconds=20.0, corridor_half_length_m=20.0
        )
    data["segments"][1]["wz"] = 0.1
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError, match="straight"):
        generate_fixture(
            ROOT, tmp_path / "turn", path, hard_seconds=20.0, corridor_half_length_m=40.0
        )


@pytest.mark.parametrize(
    "name,seconds,length,radius",
    [
        ("low_forward", 16.0, 20.0, 6.0),
        ("high_forward", 20.0, 40.0, 6.0),
        ("reverse", 12.0, 20.0, 6.0),
        ("turn", 13.0, None, 7.0),
    ],
)
def test_next_plans_generate_without_execution(tmp_path, name, seconds, length, radius):
    p = PLAN.parent / "next" / (name + ".json")
    manifest = generate_fixture(
        ROOT,
        tmp_path / name,
        p,
        hard_seconds=seconds,
        corridor_half_length_m=length,
        landmark_radius_m=radius,
    )
    assert manifest["calibration_executed"] is False
    assert manifest["target_topic"].endswith("/" + name + "/cmd_vel")
    assert manifest["physics"].startswith("baseline.sdf 1ms")


def test_reviewed_world_only_allows_static_scene_additions(tmp_path):
    from tools.vehicle_motion_fixture import verify_scene_only

    base = ET.fromstring(
        '<sdf version="1.7"><world name="w"><physics><max_step_size>0.001</max_step_size>'
        '</physics><plugin filename="physics"/><model name="ground"><static>true</static>'
        "</model></world></sdf>"
    )
    candidate = ET.fromstring(ET.tostring(base))
    world = candidate.find("world")
    panel = ET.SubElement(world, "model", name="candidate_diagonal_0_1")
    ET.SubElement(panel, "static").text = "true"
    ET.SubElement(panel, "pose").text = "0 5.3 1 0 0 0.785398"
    link = ET.SubElement(panel, "link", name="link")
    for kind in ["collision", "visual"]:
        ET.SubElement(
            ET.SubElement(ET.SubElement(ET.SubElement(link, kind), "geometry"), "box"), "size"
        ).text = "3 .2 2"
    assert verify_scene_only(base, candidate) == ["candidate_diagonal_0_1"]
    for change in ["physics", "plugin", "vehicle", "dynamic"]:
        modified = ET.fromstring(ET.tostring(candidate))
        if change == "physics":
            modified.find(".//max_step_size").text = ".01"
        elif change == "plugin":
            ET.SubElement(modified.find("world"), "plugin", filename="actuator")
        elif change == "vehicle":
            modified.find(".//model[@name='ground']/static").text = "false"
        else:
            modified.find(".//model[@name='candidate_diagonal_0_1']/static").text = "false"
        with pytest.raises(ValueError):
            verify_scene_only(base, modified)


def test_scene_bytes_and_manifest_are_bound_to_reviewed_digest(tmp_path):
    base = tmp_path / "base"
    generate_fixture(ROOT, base, PLAN)
    tree = ET.parse(base / "world.sdf")
    panel = ET.SubElement(tree.find("world"), "model", name="candidate_diagonal_test_1")
    ET.SubElement(panel, "static").text = "true"
    ET.SubElement(panel, "pose").text = "0 5.3 1 0 0 .785398"
    link = ET.SubElement(panel, "link", name="link")
    ET.SubElement(
        ET.SubElement(ET.SubElement(ET.SubElement(link, "collision"), "geometry"), "box"), "size"
    ).text = "3 .2 2"
    scene = tmp_path / "reviewed.sdf"
    tree.write(scene, encoding="unicode")
    sha = hashlib.sha256(scene.read_bytes()).hexdigest()
    out = tmp_path / "bound"
    result = generate_fixture(ROOT, out, PLAN, reviewed_world=scene, reviewed_world_sha256=sha)
    assert (out / "world.sdf").read_bytes() == scene.read_bytes()
    assert result["files"]["world.sdf"] == sha
    assert result["reviewed_scene"]["added_static_models"] == ["candidate_diagonal_test_1"]
    assert (
        result["source_model_sha256"]
        == hashlib.sha256(
            (ROOT / "ros2/src/senior_sim/urdf/senior.urdf.xacro").read_bytes()
        ).hexdigest()
    )
    with pytest.raises(ValueError, match="together"):
        generate_fixture(ROOT, tmp_path / "unbound", PLAN, reviewed_world=scene)
    with pytest.raises(ValueError, match="SHA256"):
        generate_fixture(
            ROOT, tmp_path / "wrong", PLAN, reviewed_world=scene, reviewed_world_sha256="0" * 64
        )
