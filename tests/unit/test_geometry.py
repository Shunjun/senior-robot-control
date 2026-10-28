import math

from shapely.affinity import rotate
from shapely.geometry import Polygon
from shapely.ops import unary_union

from senior_control.safety.geometry import swept_footprint


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
