"""Show that one stationary scan can match two positions in a real native archive."""

import json
import math
import os
import secrets
import subprocess
import sys
import time
from pathlib import Path
from tempfile import TemporaryDirectory

import rclpy
from probe_candidate_load import LIVE_NODE, SERIALIZE, pair_digest, request, wait_for_disappearance
from probe_read_graph import (
    BASE,
    LASER,
    MAP,
    ODOM,
    SCAN,
    SERVICE,
    bridge_process,
    read_snapshot,
    scan_in_room,
    transform,
    wait_for_scan,
)
from rclpy.qos import qos_profile_sensor_data
from senior_slam_bridge.srv import ReadGraph
from sensor_msgs.msg import LaserScan
from slam_toolbox.srv import SerializePoseGraph
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


def capture_repeated_rooms(executable, node, graph, serialize, publisher, odom_tf, prefix):
    yaw = 0.37
    separation = 20.0
    with bridge_process(executable):
        assert graph.wait_for_service(timeout_sec=8.0)
        assert serialize.wait_for_service(timeout_sec=4.0)
        first = wait_for_scan(node, graph, publisher, odom_tf, 0.0, yaw, count=1)
        # The second room has the same walls in local coordinates. Translate
        # only its map/odom pose, beyond the first room's correlation region.
        stamp = node.get_clock().now().to_msg()
        odom = transform(stamp, ODOM, BASE, separation, yaw)
        scan = scan_in_room(stamp, 0.0, yaw)
        deadline = time.monotonic() + 8.0
        while True:
            odom_tf.sendTransform(odom)
            publisher.publish(scan)
            rclpy.spin_once(node, timeout_sec=0.05)
            snapshot = read_snapshot(node, graph)
            assert snapshot.boot_id == first.boot_id
            assert snapshot.status == ReadGraph.Response.OK
            if len(snapshot.keyframes) == 2:
                break
            assert time.monotonic() < deadline, "Second room did not produce a keyframe"
        a, b = snapshot.keyframes
        assert abs(a.x - b.x) > 2 * 4.0, "Reference neighborhoods must be disjoint"
        assert (
            request(node, serialize, SerializePoseGraph.Request(filename=str(prefix))).result == 0
        )
    wait_for_disappearance(node, LIVE_NODE, graph, serialize)
    return snapshot, yaw


def match(executable, prefix, digest, scan, reference, query_path):
    query = {
        "seed_x": reference.x + 0.05,
        "seed_y": reference.y - 0.02,
        "seed_yaw": reference.yaw + 0.02,
        "first_angle": scan.angle_min,
        "angle_step": scan.angle_increment,
        "ranges": [value if math.isfinite(value) else None for value in scan.ranges],
    }
    query_path.write_text(json.dumps(query, allow_nan=False), encoding="utf-8")
    result = subprocess.run(
        [
            sys.executable,
            executable,
            str(prefix),
            digest,
            "--map-frame",
            MAP,
            "--odom-frame",
            ODOM,
            "--stationary-scan",
            str(query_path),
        ],
        stdout=subprocess.PIPE,
        timeout=20,
        check=True,
    )
    return json.loads(result.stdout)


def probe(executable, preflight, root):
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    os.environ["ROS_DOMAIN_ID"] = str(secrets.randbelow(8) + 183)
    rclpy.init(args=[])
    node = rclpy.create_node("slam_candidate_ambiguity_probe")
    try:
        publisher = node.create_publisher(LaserScan, SCAN, qos_profile_sensor_data)
        static_tf = StaticTransformBroadcaster(node)
        odom_tf = TransformBroadcaster(node)
        static_tf.sendTransform(transform(node.get_clock().now().to_msg(), BASE, LASER))
        graph = node.create_client(ReadGraph, SERVICE)
        serialize = node.create_client(SerializePoseGraph, SERIALIZE)
        prefix = root / "repeated-rooms"
        snapshot, yaw = capture_repeated_rooms(
            executable, node, graph, serialize, publisher, odom_tf, prefix
        )
        digest = pair_digest(prefix)

        # One held-out scan, unaltered between the two candidate hypotheses.
        stamp = node.get_clock().now().to_msg()
        scan = scan_in_room(stamp, 0.08, yaw + 0.04)
        wrong_scan = scan_in_room(stamp, 0.08, yaw + 0.04)
        wrong_scan.ranges = [0.5] * len(scan.ranges)
        expected = [
            {"id": frame.id, "x": frame.x, "y": frame.y, "yaw": frame.yaw}
            for frame in snapshot.keyframes
        ]
        boots = {snapshot.boot_id}
        fitted = []
        for reference in snapshot.keyframes:
            diagnostics = []
            for sample in (scan, wrong_scan):
                proof = match(preflight, prefix, digest, sample, reference, root / "query.json")
                assert proof["boot_id"] not in boots
                boots.add(proof["boot_id"])
                assert proof["pair_digest"] == digest and proof["keyframes"] == expected
                diagnostic = proof["stationary_match_diagnostic"]
                assert diagnostic["reference_ids"] == [reference.id]
                diagnostics.append(diagnostic)
            correct, wrong = diagnostics
            assert correct["response"] > wrong["response"], diagnostics
            pose = correct["fitted_sensor_pose"]
            assert abs(pose["x"] - (reference.x + 0.08)) <= 0.06, correct
            assert abs(pose["y"] - reference.y) <= 0.06, correct
            assert abs(math.remainder(pose["yaw"] - (reference.yaw + 0.04), math.tau)) <= 0.04
            fitted.append(pose)

        assert abs(fitted[0]["x"] - fitted[1]["x"]) > 2 * 4.0
        assert pair_digest(prefix) == digest
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    with TemporaryDirectory(prefix="candidate-match-ambiguity-") as directory:
        probe(sys.argv[1], sys.argv[2], Path(directory))
