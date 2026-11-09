from pathlib import Path

import pytest

from senior_control.mapping.simulation import baseline_grid, map_to_odom
from senior_control.models import Pose2D


def test_baseline_grid_matches_the_pinned_world(tmp_path: Path) -> None:
    world = Path("ros2/src/senior_sim/worlds/baseline.sdf")
    grid = baseline_grid(world, "boot")
    assert grid.map_ref.map_id.startswith("sim-baseline-")
    assert grid.cells[80 * 160 + 120] == 100  # World x=2, y=0, inside the wall.
    assert grid.cells[80 * 160 + 80] == 0
    assert grid.waypoint_writes
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
