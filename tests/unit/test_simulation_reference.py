import importlib.util
from pathlib import Path

import pytest

from senior_control.models import Pose2D

_SPEC = importlib.util.spec_from_file_location(
    "world_reference", Path("ros2/src/senior_sim/scripts/world_reference.py")
)
_WORLD = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_WORLD)
baseline_grid, map_to_odom = _WORLD.baseline_grid, _WORLD.map_to_odom


def test_baseline_grid_matches_the_pinned_world(tmp_path: Path) -> None:
    world = Path("ros2/src/senior_sim/worlds/baseline.sdf")
    grid = baseline_grid(world, "boot")
    assert grid.map_ref.map_id.startswith("sim-baseline-")
    assert grid.cells[80 * 160 + 120] == 100  # World x=2, y=0, inside the wall.
    assert grid.cells[80 * 160 + 80] == 0
    assert grid.waypoint_writes
    from senior_control.hardware_navigation import load_navigation_reference

    reference = _WORLD.generate_reference(world, tmp_path / "assets")
    generated = load_navigation_reference(reference)
    assert bytes(generated.grid().cells) == grid.cells
    assert generated.pose_topic == "/amcl_pose"
    assert generated.pose_publisher == "amcl"
    pgm = (reference.parent / "map.pgm").read_bytes()
    assert pgm.startswith(b"P5\n160 160\n255\n")
    assert len(pgm.split(b"\n", 3)[-1]) == grid.width * grid.height
    modified = tmp_path / "baseline.sdf"
    modified.write_bytes(world.read_bytes().replace(b"2 0 0.75", b"3 0 0.75"))
    with pytest.raises(ValueError, match="baseline"):
        baseline_grid(modified, "boot")


def test_map_to_odom_applies_the_same_reference_transform() -> None:
    transform = map_to_odom(Pose2D(5, 3, 1.5707963267948966), Pose2D(2, 1, 0))
    assert transform.x == pytest.approx(6)
    assert transform.y == pytest.approx(1)
    assert transform.yaw == pytest.approx(1.5707963267948966)


def test_route_world_raster_includes_cross_walls_and_preserves_start_corridor() -> None:
    world = Path("ros2/src/senior_sim/worlds/motion_qualification.sdf")
    grid = baseline_grid(world, "boot")

    def cell(x: float, y: float) -> int:
        column = int((x - grid.origin.x) / grid.resolution)
        row = int((y - grid.origin.y) / grid.resolution)
        return grid.cells[row * grid.width + column]

    assert cell(0, 3) == 100
    assert cell(-3.5, 0) == 100
    assert cell(2, 1.2) == 100
    assert all(cell(x, 0) == 0 for x in (0, 0.35, 0.7, 1.05))


def test_scan_projection_accounts_for_robot_and_sensor_mount_and_limits_payload():
    import math

    project_scan = _WORLD.project_scan

    points = project_scan(
        Pose2D(1, 2, math.pi / 2),
        Pose2D(0.2, 0, 0),
        [1.0, float("inf"), float("nan"), 20.0],
        0,
        math.pi / 2,
        0.08,
        20,
    )
    assert points == [{"x": 1.0, "y": 3.2}]
    many = project_scan(Pose2D(0, 0, 0), Pose2D(0, 0, 0), [1.0] * 720, 0, 0.01, 0.08, 20)
    assert len(many) <= 64
