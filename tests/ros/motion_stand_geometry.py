"""Geometry checks for the owned fixed-stand fixture."""

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass


@dataclass(frozen=True)
class WheelGeometry:
    centers: tuple[tuple[float, float, float], ...]
    radius: float
    width: float


def load_wheel_geometry(path):
    root = ET.parse(path).getroot()
    macro = root.find("{http://www.ros.org/wiki/xacro}macro[@name='wheel']")
    if macro is None:
        raise ValueError("wheel macro unavailable")
    joint = macro.find("joint")
    cylinder = macro.find("link/collision/geometry/cylinder")
    collision_origin = macro.find("link/collision/origin")
    if joint is None or cylinder is None or collision_origin is None:
        raise ValueError("wheel collision geometry unavailable")
    axis = joint.find("axis")
    origin = joint.find("origin")
    if axis is None or origin is None or axis.attrib.get("xyz") != "0 1 0":
        raise ValueError("unexpected wheel rotation axis")
    xyz = origin.attrib.get("xyz", "").split()
    if len(xyz) != 3 or xyz[:2] != ["${x}", "${y}"]:
        raise ValueError("unexpected wheel joint placement")
    roll, pitch, yaw = map(float, collision_origin.attrib.get("rpy", "").split())
    if abs(roll - math.pi / 2) > 1e-9 or pitch != 0 or yaw != 0:
        raise ValueError("unexpected wheel collision orientation")
    radius, width, z = (
        float(cylinder.attrib["radius"]),
        float(cylinder.attrib["length"]),
        float(xyz[2]),
    )
    wheels = root.findall("{http://www.ros.org/wiki/xacro}wheel")
    names = {wheel.attrib["name"] for wheel in wheels}
    if names != {"left_front", "left_back", "right_front", "right_back"}:
        raise ValueError("unexpected wheel set")
    centers = tuple((float(wheel.attrib["x"]), float(wheel.attrib["y"]), z) for wheel in wheels)
    if not all(math.isfinite(value) and value > 0 for value in (radius, width)) or not all(
        math.isfinite(value) for point in centers for value in point
    ):
        raise ValueError("invalid wheel collision dimensions")
    return WheelGeometry(centers, radius, width)


def wheel_ground_clearance(position, orientation, geometry, ground_z):
    qx, qy, qz, qw = orientation.x, orientation.y, orientation.z, orientation.w
    values = (position.x, position.y, position.z, qx, qy, qz, qw, ground_z)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("invalid truth pose")
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if abs(norm - 1.0) > 0.001:
        raise ValueError("invalid truth orientation")
    qx, qy, qz, qw = (value / norm for value in (qx, qy, qz, qw))
    rz_x = 2 * (qx * qz - qy * qw)
    rz_y = 2 * (qy * qz + qx * qw)
    rz_z = 1 - 2 * (qx * qx + qy * qy)
    vertical_radius = geometry.radius * math.sqrt(max(0.0, 1 - rz_y * rz_y))
    vertical_radius += geometry.width * abs(rz_y) / 2
    return min(
        position.z + rz_x * x + rz_y * y + rz_z * z - vertical_radius - ground_z
        for x, y, z in geometry.centers
    )
