import math

import pytest
from shapely.affinity import rotate
from shapely.geometry import Point, Polygon
from shapely.ops import unary_union

from senior_control.safety.geometry import swept_footprint, swept_trajectory


def test_corner_rotation_blocked():
    # Centered square. A quarter turn maps it onto itself, so endpoint samples
    # miss the obstacle. The corner arc reaches y=sqrt(2) ≈ 1.414, and the
    # one-step sagitta is sqrt(2)-1 ≈ 0.414, which includes y=1.2.
    footprint = ((-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0))
    obstacle = Polygon(((-0.05, 1.15), (0.05, 1.15), (0.05, 1.25), (-0.05, 1.25)))
    start = Polygon(footprint)
    end = rotate(start, math.pi / 2, origin=(0.0, 0.0), use_radians=True)
    assert not unary_union([start, end]).intersects(obstacle)

    sweep = swept_footprint(footprint, yaw=math.pi / 2, yaw_step=math.pi / 2, margin=0.0)
    assert sweep.intersects(obstacle)


def test_rectangle_mid_arc_corner_is_inside_sweep():
    # This synthetic rectangle's corner radius is hypot(0.337, 0.27). One step leaves the
    # reviewer's mid-arc sample, angle 0.908 rad at 0.995r, outside the
    # endpoint union. A conservative sweep has to contain that point.
    footprint = ((-0.337, -0.27), (0.337, -0.27), (0.337, 0.27), (-0.337, 0.27))
    radius = math.hypot(0.337, 0.27)
    angle = 0.908
    distance = 0.995 * radius
    point = Point(distance * math.cos(angle), distance * math.sin(angle))
    start = Polygon(footprint)
    end = rotate(start, 0.48, origin=(0.0, 0.0), use_radians=True)
    assert not unary_union([start, end]).intersects(point)

    sweep = swept_footprint(footprint, yaw=0.48, yaw_step=0.48, margin=0.0)
    assert sweep.intersects(point)
    padded = swept_footprint(footprint, yaw=0.48, yaw_step=0.48, margin=0.02)
    assert padded.intersects(point)


def test_tiny_yaw_keeps_inflation_inside_hard_margin():
    # Stopping distance 0.48 m and yaw 0.00205 rad is a 0.01 rad/s stop.
    # One step inflated by the distance to the turning center reaches y=±0.53.
    # The body is only 0.27 m wide; 0.15 m beside it must stay outside.
    footprint = ((-0.337, -0.27), (0.337, -0.27), (0.337, 0.27), (-0.337, 0.27))
    sweep = swept_trajectory(footprint, distance=0.48, yaw=0.00205, step=0.00205, margin=0.02)
    _minx, miny, _maxx, maxy = sweep.bounds
    assert maxy < 0.27 + 0.15
    assert miny > -(0.27 + 0.15)
    assert not sweep.intersects(Point(0.0, 0.27 + 0.15))


@pytest.mark.parametrize("yaw", [-1e-17, 1e-17, -1e-8, 1e-8])
def test_near_zero_yaw_still_conservatively_covers_translation_and_margin(yaw):
    footprint = ((-0.337, -0.276), (0.337, -0.276), (0.337, 0.276), (-0.337, 0.276))
    sweep = swept_trajectory(footprint, distance=0.1, yaw=yaw, step=abs(yaw), margin=0.02)
    for x in (-0.337, 0.0, 0.337, 0.437):
        assert sweep.covers(Point(x, 0.276))
    assert sweep.covers(Point(0.437 + 0.019, 0.0))
    assert not sweep.intersects(Point(0.437 + 0.03, 0.0))
