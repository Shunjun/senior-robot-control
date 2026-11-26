"""Ordinary synthetic sensors only; receipt/qualification/motion remain production owners.

Pinned SDF collision boxes and the S02 curved front share world coordinates.
Unknown/no intersection is NaN, never inferred free. No native/Q/ACK publisher.
"""

import hashlib
import json
import math
import xml.etree.ElementTree as ET
from collections import deque
from pathlib import Path

BEAMS = 2048
SCAN_PERIOD_NS = 200_000_000


def frozen_episode() -> tuple[dict, ...]:
    """One 60s reference input: 300 scans, stable/motion/stable, one save/restore."""
    return tuple(
        dict(
            index=i,
            offset_ns=i * SCAN_PERIOD_NS,
            beams=BEAMS,
            object_offset_m=(0.0 if i < 100 else min(0.4, (i - 100) * 0.02)),
            phase="stable_A" if i < 100 else "motion" if i < 120 else "stable_B",
            operation="save" if i == 150 else "restore_export" if i == 299 else None,
        )
        for i in range(300)
    )


def world_segments(world: Path) -> tuple:
    """Read original collision geometry, never display/reference preview points."""
    segments = []
    for model in ET.fromstring(world.read_bytes()).findall(".//model"):
        size = model.findtext("link/collision/geometry/box/size")
        if size is None:
            continue
        x, y, _, _, _, yaw = map(float, model.findtext("pose").split())
        width, height, _ = map(float, size.split())
        c, s = math.cos(yaw), math.sin(yaw)
        corners = [
            (x + c * u - s * v, y + s * u + c * v)
            for u, v in (
                (-width / 2, -height / 2),
                (width / 2, -height / 2),
                (width / 2, height / 2),
                (-width / 2, height / 2),
            )
        ]
        segments.extend(zip(corners, corners[1:] + corners[:1], strict=True))
    return tuple(segments)


def mount_from_urdf(path: Path) -> tuple[float, float, float]:
    joint = ET.fromstring(path.read_bytes()).find("joint[@name='laser_joint']")
    assert joint is not None and joint.attrib["type"] == "fixed"
    origin = joint.find("origin")
    xyz = tuple(map(float, origin.attrib["xyz"].split()))
    rpy = tuple(map(float, origin.attrib.get("rpy", "0 0 0").split()))
    assert rpy[:2] == (0.0, 0.0), "planar fixture requires original upright lidar"
    return xyz[0], xyz[1], rpy[2]


def object_segments(offset: float) -> tuple:
    points = [(3.0 + offset + 0.12 * (i / 64 - 0.5) ** 2, (i / 64 - 0.5) * 0.8) for i in range(65)]
    return tuple(zip(points[:-1], points[1:], strict=True))


def raycast(origin, direction, segments) -> float:
    x, y = origin
    dx, dy = direction
    nearest = math.inf
    for (ax, ay), (bx, by) in segments:
        sx, sy = bx - ax, by - ay
        determinant = dx * sy - dy * sx
        if abs(determinant) < 1e-12:
            continue
        qx, qy = ax - x, ay - y
        distance = (qx * sy - qy * sx) / determinant
        position = (qx * dy - qy * dx) / determinant
        if distance >= 0 and 0 <= position <= 1:
            nearest = min(nearest, distance)
    return nearest if 0.08 <= nearest < 20.0 else math.nan


def ranges_for(pose, mount, walls, *, offset=0.0, invalid=False):
    if invalid:
        return (math.nan,) * BEAMS
    x, y, yaw = pose
    mx, my, myaw = mount
    c, s = math.cos(yaw), math.sin(yaw)
    origin = x + c * mx - s * my, y + s * mx + c * my
    segments = walls + object_segments(offset)
    return tuple(
        raycast(
            origin,
            (
                math.cos(yaw + myaw - math.pi + i * math.tau / (BEAMS - 1)),
                math.sin(yaw + myaw - math.pi + i * math.tau / (BEAMS - 1)),
            ),
            segments,
        )
        for i in range(BEAMS)
    )


def freeze(path: Path, world: Path, urdf: Path) -> str:
    """Freeze raw scan arrays plus zero wheel/TF trajectory; None encodes unknown."""
    walls, mount = world_segments(world), mount_from_urdf(urdf)
    metadata = dict(
        schema=1,
        stage="input_metadata",
        rate_hz=5,
        beams=BEAMS,
        wheel_tf_hz=20,
        pose=[0.0, 0.0, 0.0],
        world_sha256=hashlib.sha256(world.read_bytes()).hexdigest(),
        urdf_sha256=hashlib.sha256(urdf.read_bytes()).hexdigest(),
        mount=mount,
        no_return="unknown",
        closed_loop=False,
    )
    with path.open("x") as output:
        output.write(json.dumps(metadata) + "\n")
        for frame in frozen_episode():
            ranges = ranges_for((0, 0, 0), mount, walls, offset=frame["object_offset_m"])
            output.write(
                json.dumps(
                    dict(frame, ranges=[r if math.isfinite(r) else None for r in ranges]),
                    allow_nan=False,
                )
                + "\n"
            )
    path.chmod(0o600)
    return hashlib.sha256(path.read_bytes()).hexdigest()


class SensorSource:
    """Real DDS ordinary sources accepted by the original environment motion profile.

    drive and senior_sim_bridge are ordinary wheel/IMU producers, not receipt owners.
    The real robot_state_publisher is separately launched from the original xacro.
    """

    def __init__(self, world: Path, urdf: Path, trace, *, frozen: Path | None = None):
        import rclpy
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Imu, LaserScan
        from tf2_ros import TransformBroadcaster

        from vehicle_core.values import Pose2D

        self.trace = trace
        self.node = rclpy.create_node("shared_layers_raw_source")
        self.wheel_node = rclpy.create_node("drive")
        self.imu_node = rclpy.create_node("senior_sim_bridge")
        self.nodes = (self.node, self.wheel_node, self.imu_node)
        self.scan = self.node.create_publisher(LaserScan, "/scan", qos_profile_sensor_data)
        self.wheel = self.wheel_node.create_publisher(
            Odometry, "/wheel/odom", qos_profile_sensor_data
        )
        self.imu = self.imu_node.create_publisher(Imu, "/imu/data", qos_profile_sensor_data)
        self.tf = TransformBroadcaster(self.wheel_node)
        self.pose = Pose2D(0.0, 0.0, 0.0)
        self.offset = 0.0
        self.invalid = False
        self.enabled = True
        self.following = False
        self.command = (0.0, 0.0)
        self.command_at = 0
        self.tick = 0
        self.walls, self.mount = world_segments(world), mount_from_urdf(urdf)
        self.frozen = None
        self.frozen_origin_ns = None
        self.operations = deque(maxlen=2)
        self.world, self.urdf = world, urdf
        if frozen:
            self.begin_frozen(frozen)
        self.node.create_subscription(Twist, "/cmd_vel", self._command, 10)
        self.timer = self.node.create_timer(0.05, self._step)
        self.previous_ns = self.node.get_clock().now().nanoseconds

    def begin_frozen(self, path: Path):
        if path.stat().st_size > 32 * 1024 * 1024:
            raise ValueError("frozen input exceeds 32MiB bound")
        rows = [json.loads(line) for line in path.open()]
        metadata, frames = rows[0], rows[1:]
        assert metadata["world_sha256"] == hashlib.sha256(self.world.read_bytes()).hexdigest()
        assert metadata["urdf_sha256"] == hashlib.sha256(self.urdf.read_bytes()).hexdigest()
        expected = frozen_episode()
        assert len(frames) == len(expected)
        for frame, recipe in zip(frames, expected, strict=True):
            assert all(frame[name] == value for name, value in recipe.items())
            assert len(frame["ranges"]) == BEAMS
            assert all(
                r is None or (isinstance(r, (int, float)) and math.isfinite(r) and 0.08 <= r < 20.0)
                for r in frame["ranges"]
            )
        self.frozen = frames
        self.tick = 0
        self.frozen_origin_ns = self.node.get_clock().now().nanoseconds
        self.trace.emit(
            "frozen_input",
            sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
            ros_stamp_origin_ns=self.frozen_origin_ns,
            schedule="original offsets, no receive-time restamp",
        )

    def _command(self, message):
        import time

        self.command = message.linear.x, message.angular.z
        self.command_at = time.monotonic_ns()
        self.trace.emit(
            "control",
            vx=self.command[0],
            wz=self.command[1],
            association="authorization window only; no capture cause",
        )

    def _step(self):
        import time

        from geometry_msgs.msg import TransformStamped
        from nav_msgs.msg import Odometry
        from senior_motion_estimation.kinematics import compose, exponential
        from sensor_msgs.msg import Imu, LaserScan

        stamp = self.node.get_clock().now()
        actual_stamp_ns = stamp.nanoseconds
        if self.frozen is not None:
            from rclpy.time import Time

            stamp = Time(nanoseconds=self.frozen_origin_ns + self.tick * 50_000_000)
        dt = min(0.1, max(0, (stamp.nanoseconds - self.previous_ns) / 1e9))
        self.previous_ns = stamp.nanoseconds
        vx, wz = (
            self.command
            if (
                self.following
                and self.frozen is None
                and time.monotonic_ns() - self.command_at < 300_000_000
            )
            else (0.0, 0.0)
        )
        self.pose = compose(self.pose, exponential(vx, 0.0, wz, dt))
        pose = self.pose
        transform = TransformStamped()
        transform.header.stamp, transform.header.frame_id = stamp.to_msg(), "odom"
        transform.child_frame_id = "base_link"
        transform.transform.translation.x, transform.transform.translation.y = pose.x, pose.y
        transform.transform.rotation.z = math.sin(pose.yaw / 2)
        transform.transform.rotation.w = math.cos(pose.yaw / 2)
        self.tf.sendTransform(transform)
        wheel = Odometry()
        wheel.header, wheel.child_frame_id = transform.header, "base_link"
        wheel.pose.pose.position.x, wheel.pose.pose.position.y = pose.x, pose.y
        wheel.pose.pose.orientation = transform.transform.rotation
        wheel.pose.covariance[0] = wheel.pose.covariance[7] = 0.000001
        wheel.pose.covariance[35] = 0.00000001
        wheel.twist.twist.linear.x, wheel.twist.twist.angular.z = vx, wz
        self.wheel.publish(wheel)
        imu = Imu()
        imu.header.stamp, imu.header.frame_id = stamp.to_msg(), "imu_link"
        imu.orientation = transform.transform.rotation
        imu.angular_velocity.z, imu.linear_acceleration.z = wz, 9.80665
        imu.angular_velocity_covariance[8] = 0.00000001
        self.imu.publish(imu)
        if self.enabled and self.tick % 4 == 0:
            index = self.tick // 4
            if self.frozen is not None and index >= len(self.frozen):
                self.tick += 1
                return
            scan = LaserScan()
            scan.header.stamp, scan.header.frame_id = stamp.to_msg(), "laser_link"
            scan.angle_min, scan.angle_max = -math.pi, math.pi
            scan.angle_increment = math.tau / (BEAMS - 1)
            scan.range_min, scan.range_max, scan.scan_time = 0.08, 20.0, 0.2
            if self.frozen is not None:
                frame = self.frozen[index]
                scan.ranges = [math.nan if r is None else r for r in frame["ranges"]]
                if frame["operation"]:
                    self.operations.append(frame["operation"])
                    self.trace.emit(
                        "input_operation_due", operation=frame["operation"], index=index
                    )
            else:
                scan.ranges = ranges_for(
                    (pose.x, pose.y, pose.yaw),
                    self.mount,
                    self.walls,
                    offset=self.offset,
                    invalid=self.invalid,
                )
            self.scan.publish(scan)
            self.trace.emit(
                "raw_input",
                index=index,
                stamp_ns=stamp.nanoseconds,
                publication_lateness_ns=actual_stamp_ns - stamp.nanoseconds,
                phase="reference" if self.frozen is not None else "functional",
                pose=[pose.x, pose.y, pose.yaw],
                offset=self.offset,
                invalid=self.invalid,
                beam_count=BEAMS,
            )
        self.tick += 1

    def close(self):
        for node in self.nodes:
            node.destroy_node()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--freeze", type=Path, required=True)
    parser.add_argument("--world", type=Path, required=True)
    parser.add_argument("--urdf", type=Path, required=True)
    args = parser.parse_args()
    print(freeze(args.freeze, args.world, args.urdf))
