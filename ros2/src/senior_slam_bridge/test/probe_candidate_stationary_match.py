"""Reload a real Humble graph and run one isolated stationary diagnostic per boot."""

import math
import os
import secrets
import sys
from pathlib import Path
from tempfile import TemporaryDirectory

import rclpy
from probe_candidate_load import (
    CANDIDATE_NODE,
    RECEIPT,
    SERIALIZE,
    candidate,
    candidate_arguments,
    capture,
    observe,
    pair_digest,
    request,
    wait_for_disappearance,
)
from probe_read_graph import BASE, LASER, MAP, SCAN, SERVICE, read_snapshot, scan_in_room, transform
from rclpy.qos import qos_profile_sensor_data
from senior_slam_bridge.srv import MatchStationaryScan, ReadGraph, ReadLoadedGraph
from sensor_msgs.msg import LaserScan
from slam_toolbox.srv import SerializePoseGraph
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

MATCH = "/senior/slam/match_stationary_scan"


def probe(executable, root):
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    os.environ["ROS_DOMAIN_ID"] = str(secrets.randbelow(8) + 175)
    rclpy.init(args=[])
    node = rclpy.create_node("slam_candidate_probe")
    try:
        publisher = node.create_publisher(LaserScan, SCAN, qos_profile_sensor_data)
        static_tf = StaticTransformBroadcaster(node)
        odom_tf = TransformBroadcaster(node)
        static_tf.sendTransform(transform(node.get_clock().now().to_msg(), BASE, LASER))
        graph = node.create_client(ReadGraph, SERVICE)
        receipt = node.create_client(ReadLoadedGraph, RECEIPT)
        match = node.create_client(MatchStationaryScan, MATCH)
        serialize = node.create_client(SerializePoseGraph, SERIALIZE)
        prefix = root / "archive"
        prior = capture(executable, node, graph, serialize, publisher, odom_tf, prefix)
        node.destroy_client(serialize)
        digest = pair_digest(prefix)
        boots = {prior.boot_id}
        with candidate(candidate_arguments(executable, prefix, digest, "default")):
            boots.add(observe(node, receipt, graph, prefix, "default", boots, prior))
            assert not match.wait_for_service(timeout_sec=0.3)
        wait_for_disappearance(node, CANDIDATE_NODE, receipt, graph, match)

        query_x, query_yaw = 0.08, 0.41
        scan = scan_in_room(node.get_clock().now().to_msg(), query_x, query_yaw)
        scores = {}
        for scene in ("positive", "wrong"):
            args = candidate_arguments(executable, prefix, digest, scene)
            args.extend(["-p", "candidate_stationary_match:=true"])
            with candidate(args) as process:
                boot = observe(node, receipt, graph, prefix, scene, boots, prior)
                boots.add(boot)
                assert match.wait_for_service(timeout_sec=4)
                ranges = list(scan.ranges) if scene == "positive" else [0.5] * len(scan.ranges)
                query = MatchStationaryScan.Request(
                    boot_id=boot,
                    attempt_id=scene,
                    pair_digest=digest,
                    map_frame=MAP,
                    seed_x=0.05,
                    seed_y=-0.02,
                    seed_yaw=0.39,
                    first_angle=scan.angle_min,
                    angle_step=scan.angle_increment,
                    ranges=ranges,
                )
                for field, incorrect in (
                    ("boot_id", prior.boot_id),
                    ("attempt_id", "foreign"),
                    ("pair_digest", "0" * 64),
                    ("map_frame", "foreign"),
                ):
                    original = getattr(query, field)
                    setattr(query, field, incorrect)
                    denied = request(node, match, query)
                    assert denied.status == MatchStationaryScan.Response.UNAVAILABLE
                    assert not denied.reference_ids
                    setattr(query, field, original)
                query.angle_step = 0.0
                invalid = request(node, match, query)
                assert invalid.status == MatchStationaryScan.Response.INVALID
                assert not invalid.reference_ids
                query.angle_step = scan.angle_increment
                result = request(node, match, query)
                assert result.status == MatchStationaryScan.Response.COMPLETE
                assert math.isfinite(result.response)
                assert list(result.reference_ids) == [prior.keyframes[0].id]
                scores[scene] = result.response
                if scene == "positive":
                    assert abs(result.fitted_x - query_x) <= 0.06
                    assert abs(result.fitted_y) <= 0.06
                    assert abs(math.remainder(result.fitted_yaw - query_yaw, math.tau)) <= 0.04
                spent = request(node, match, query)
                assert spent.status == MatchStationaryScan.Response.SPENT
                assert not spent.reference_ids
                assert read_snapshot(node, graph).keyframes == prior.keyframes
                assert pair_digest(prefix) == digest
                assert process.poll() is None
            assert process.returncode == 0
            wait_for_disappearance(node, CANDIDATE_NODE, receipt, graph, match)
        assert scores["positive"] > scores["wrong"] and scores["positive"] > 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    with TemporaryDirectory(prefix="candidate-stationary-match-") as directory:
        probe(sys.argv[1], Path(directory))
