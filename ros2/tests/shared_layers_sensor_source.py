"""Ordinary synthetic sensors only; receipt/qualification/motion remain production owners.

Pinned SDF collision boxes and the S02 curved front share world coordinates.
Unknown/no intersection is NaN, never inferred free. No native/Q/ACK publisher.
"""

import hashlib
import json
import math
import time
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


# Ordinary values only; receipt identities/qualification are never tape fields.
MOTION_FORMAT = {
    "schema": 2,
    "tick": ["index", "offset_ns", "steady_offset_ns", "dt", "command", "commands", "pose", "twist",
             "object_offset_m", "invalid", "scan"],
    "command": ["ros_offset_ns", "steady_offset_ns", "vx", "wz"],
    "sensors": "original wheel/TF pose+twist; IMU same orientation,wz,gravity=9.80665",
    "scan": "original frozen_episode recipe + ranges; null means unknown",
}
MOTION_FORMAT_SHA256 = hashlib.sha256(
    json.dumps(MOTION_FORMAT, sort_keys=True, separators=(",", ":")).encode()
).hexdigest()


def _write_row(output, row):
    output.write(json.dumps(row, allow_nan=False, separators=(",", ":")) + "\n")


def _validate_command(command):
    if set(command) != set(MOTION_FORMAT["command"]):
        raise ValueError("invalid command fields")
    if any(type(command[n]) is not int for n in ("ros_offset_ns", "steady_offset_ns")):
        raise ValueError("invalid command time")
    if not all(type(command[n]) in (int, float) and math.isfinite(command[n])
               for n in ("vx", "wz")):
        raise ValueError("invalid command value")


def read_motion(rows):
    """Validate bounded per-tick ordinary tape before any replay publication."""
    metadata, *ticks, end = rows
    if (metadata.get("format_sha256") != MOTION_FORMAT_SHA256
            or metadata.get("schema") != 2 or metadata.get("ticks") != 1200
            or metadata.get("beams") != BEAMS):
        raise ValueError("unsupported motion format")
    if end != dict(stage="input_end", ticks=1200, scans=300, complete=True):
        raise ValueError("incomplete ordinary input recording")
    if len(ticks) != 1200:
        raise ValueError("motion input needs all 1200 real ticks")
    previous = previous_steady = -1
    recipes = frozen_episode()
    for i, tick in enumerate(ticks):
        if set(tick) != set(MOTION_FORMAT["tick"]) or tick["index"] != i:
            raise ValueError("invalid ordinary tick fields/index")
        for name, last in (("offset_ns", previous), ("steady_offset_ns", previous_steady)):
            if type(tick[name]) is not int or tick[name] <= last:
                raise ValueError("nonmonotonic recorded tick")
        previous, previous_steady = tick["offset_ns"], tick["steady_offset_ns"]
        if not 0 <= tick["dt"] <= 0.1 or type(tick["invalid"]) is not bool:
            raise ValueError("invalid motion step")
        for name, size in (("pose", 3), ("twist", 2)):
            if len(tick[name]) != size or not all(
                type(v) in (int, float) and math.isfinite(v) for v in tick[name]
            ):
                raise ValueError("invalid ordinary pose/twist")
        if not isinstance(tick["commands"], list) or len(tick["commands"]) > 256:
            raise ValueError("command observation capacity exceeded")
        for command in [tick["command"], *tick["commands"]]:
            if command is not None:
                _validate_command(command)
        scan = tick["scan"]
        if i % 4:
            if scan is not None:
                raise ValueError("extra raw frame")
        else:
            recipe = recipes[i // 4]
            if set(scan) != set(recipe) | {"ranges"} or any(
                scan[n] != v for n, v in recipe.items()
            ):
                raise ValueError("changed original scene/operation recipe")
            if len(scan["ranges"]) != BEAMS or not all(
                r is None or (type(r) in (int, float) and math.isfinite(r) and 0.08 <= r < 20)
                for r in scan["ranges"]
            ):
                raise ValueError("invalid raw ranges")
        if tick["object_offset_m"] != recipes[i // 4]["object_offset_m"]:
            raise ValueError("changed original scene offset")
    return metadata, ticks


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
        self.command_stamp_ns = None
        self.recording = None
        self.motion_ticks = None
        self.input_complete = False
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

    def begin_recording(self, path: Path, provenance: dict):
        """Record one original 1200-tick episode; output observations stay in trace."""
        if self.recording is not None or self.frozen is not None:
            raise ValueError("ordinary input mode already selected")
        self.recording = path.open("x")
        path.chmod(0o600)
        self.recording_path = path
        self.record_origin_ns = self.node.get_clock().now().nanoseconds
        self.record_steady_ns = time.monotonic_ns()
        self.tick = 0
        self.pending_commands = []
        self.recorded_scans = 0
        self.record_episode = frozen_episode()
        self.input_complete = False
        self.following = True
        _write_row(self.recording, dict(
            schema=2, stage="input_metadata", format_sha256=MOTION_FORMAT_SHA256,
            ticks=1200, beams=BEAMS, wheel_tf_hz=20, rate_hz=5,
            world_sha256=hashlib.sha256(self.world.read_bytes()).hexdigest(),
            urdf_sha256=hashlib.sha256(self.urdf.read_bytes()).hexdigest(),
            source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            ros_stamp_origin_ns=self.record_origin_ns,
            host_monotonic_origin_ns=self.record_steady_ns,
            provenance=provenance, closed_loop=True, no_return="unknown",
        ))
        self.trace.emit("motion_record_begin", path=str(path),
                        ros_stamp_origin_ns=self.record_origin_ns)

    def finish_recording(self):
        if self.recording is None:
            return
        complete = self.input_complete and self.recorded_scans == 300
        _write_row(self.recording, dict(stage="input_end", ticks=self.tick,
                   scans=self.recorded_scans, complete=complete))
        self.recording.close()
        self.recording_path.chmod(0o400)
        self.recording = None
        self.trace.emit("motion_record_end", complete=complete,
                        sha256=hashlib.sha256(self.recording_path.read_bytes()).hexdigest())

    def begin_frozen(self, path: Path):
        if path.stat().st_size > 32 * 1024 * 1024:
            raise ValueError("frozen input exceeds 32MiB bound")
        data = path.read_bytes()
        rows = [json.loads(line) for line in data.splitlines()]
        metadata, frames = rows[0], rows[1:]
        assert metadata["world_sha256"] == hashlib.sha256(self.world.read_bytes()).hexdigest()
        assert metadata["urdf_sha256"] == hashlib.sha256(self.urdf.read_bytes()).hexdigest()
        if self.recording is not None:
            raise ValueError("cannot replay while recording")
        if metadata.get("schema") == 2:
            metadata, self.motion_ticks = read_motion(rows)
            frames = [tick["scan"] for tick in self.motion_ticks if tick["scan"] is not None]
        else:
            # //Compatible: schema1 stationary archive and its original hash stay intact.
            self.motion_ticks = None
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
        self.input_complete = False
        self.frozen_origin_ns = self.node.get_clock().now().nanoseconds
        self.trace.emit(
            "frozen_input",
            sha256=hashlib.sha256(data).hexdigest(),
            ros_stamp_origin_ns=self.frozen_origin_ns,
            schedule="original offsets, no receive-time restamp",
        )

    def _command(self, message):
        import time

        self.command = message.linear.x, message.angular.z
        self.command_at = time.monotonic_ns()
        self.command_stamp_ns = self.node.get_clock().now().nanoseconds
        if self.recording is not None and not self.input_complete:
            if len(self.pending_commands) >= 256:
                raise ValueError("command observation capacity exceeded")
            self.pending_commands.append(self._recorded_command())
        self.trace.emit(
            "control",
            vx=self.command[0],
            wz=self.command[1],
            command_stamp_ns=self.command_stamp_ns,
            command_steady_ns=self.command_at,
            association="authorization window only; no capture cause",
        )

    def _step(self):
        import time

        from senior_motion_estimation.kinematics import compose, exponential

        if self.input_complete:
            return
        stamp = self.node.get_clock().now()
        actual_stamp_ns = stamp.nanoseconds
        if self.frozen is not None:
            from rclpy.time import Time

            offset = (self.motion_ticks[self.tick]["offset_ns"]
                      if self.motion_ticks is not None else self.tick * 50_000_000)
            # Wait for the recorded offset; late input keeps its old stamp.
            if self.motion_ticks is not None and actual_stamp_ns < self.frozen_origin_ns + offset:
                return
            stamp = Time(nanoseconds=self.frozen_origin_ns + offset)
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
        if self.motion_ticks is not None:
            from vehicle_core.values import Pose2D

            row = self.motion_ticks[self.tick]
            self.pose = Pose2D(*row["pose"])
            vx, wz = row["twist"]
            self.offset, self.invalid = row["object_offset_m"], row["invalid"]
        else:
            self.pose = compose(self.pose, exponential(vx, 0.0, wz, dt))
        if self.recording is not None:
            self.offset = self.record_episode[self.tick // 4]["object_offset_m"]
        self._publish_motion(stamp, vx, wz)
        recorded_scan = self._publish_scan(stamp, actual_stamp_ns)
        self._record_tick(stamp, dt, vx, wz, recorded_scan)
        self.tick += 1
        if self.tick == 1200 and (self.recording is not None or self.motion_ticks is not None):
            self.input_complete = True

    def _publish_motion(self, stamp, vx, wz):
        from geometry_msgs.msg import TransformStamped
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Imu

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

    def _publish_scan(self, stamp, actual_stamp_ns):
        from sensor_msgs.msg import LaserScan

        recorded_scan = None
        pose = self.pose
        if self.enabled and self.tick % 4 == 0:
            index = self.tick // 4
            if self.frozen is not None and index >= len(self.frozen):
                return None
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
                        "input_operation_due", operation=frame["operation"], index=index,
                        offset_ns=stamp.nanoseconds - self.frozen_origin_ns
                    )
            else:
                scan.ranges = ranges_for(
                    (pose.x, pose.y, pose.yaw),
                    self.mount,
                    self.walls,
                    offset=self.offset,
                    invalid=self.invalid,
                )
            if self.recording is not None:
                recorded_scan = dict(self.record_episode[index], ranges=[
                    r if math.isfinite(r) else None for r in scan.ranges
                ])
                if recorded_scan["operation"]:
                    self.operations.append(recorded_scan["operation"])
                    self.trace.emit("input_operation_due", operation=recorded_scan["operation"],
                                    index=index,
                                    offset_ns=stamp.nanoseconds - self.record_origin_ns)
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
        return recorded_scan

    def _recorded_command(self):
        if self.command_stamp_ns is None:
            return None
        return dict(
            ros_offset_ns=self.command_stamp_ns - self.record_origin_ns,
            steady_offset_ns=self.command_at - self.record_steady_ns,
            vx=self.command[0], wz=self.command[1],
        )

    def _record_tick(self, stamp, dt, vx, wz, recorded_scan):
        if self.recording is not None:
            pose = self.pose
            self.recorded_scans += recorded_scan is not None
            command = self._recorded_command()
            _write_row(self.recording, dict(
                index=self.tick, offset_ns=stamp.nanoseconds - self.record_origin_ns,
                steady_offset_ns=time.monotonic_ns() - self.record_steady_ns,
                dt=dt, command=command, commands=self.pending_commands,
                pose=[pose.x, pose.y, pose.yaw], twist=[vx, wz],
                object_offset_m=self.offset, invalid=self.invalid, scan=recorded_scan,
            ))
            self.pending_commands = []

    def close(self):
        self.finish_recording()
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
