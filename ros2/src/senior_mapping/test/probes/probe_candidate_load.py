"""DDS probe for one-time startup graph loads on the pinned Humble backend."""

import hashlib
import json
import os
import secrets
import subprocess
import sys
import tempfile
import time
from contextlib import contextmanager
from pathlib import Path

import rclpy
from probe_read_graph import (
    BASE,
    LASER,
    MAP,
    ODOM,
    SCAN,
    SERVICE,
    bridge_process,
    check_no_velocity_publishers,
    read_snapshot,
    transform,
    wait_for_scan,
)
from rclpy.qos import qos_profile_sensor_data
from senior_slam_bridge.srv import ReadGraph, ReadLoadedGraph
from sensor_msgs.msg import LaserScan
from slam_toolbox.srv import SerializePoseGraph
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

RECEIPT = "/senior/slam/read_loaded_graph"
SERIALIZE = "/slam_toolbox/serialize_map"
CANDIDATE_NODE = "slam_graph_candidate"
LIVE_NODE = "slam_toolbox"
LIVE_TOPICS = ("/map", "/tf", "/tf_static", "/slam_toolbox/graph_visualization")


def pair_digest(prefix):
    graph = hashlib.sha256(prefix.with_suffix(".posegraph").read_bytes()).digest()
    data = hashlib.sha256(prefix.with_suffix(".data").read_bytes()).digest()
    return hashlib.sha256(graph + data).hexdigest()


def request(node, client, message):
    pending = client.call_async(message)
    rclpy.spin_until_future_complete(node, pending, timeout_sec=3.0)
    assert pending.done(), "ROS request timed out"
    return pending.result()


def wait_for_disappearance(node, retired_node, *clients):
    deadline = time.monotonic() + 6.0
    while True:
        names = node.get_node_names_and_namespaces()
        publishers = [
            (topic, endpoint.node_namespace, endpoint.node_name)
            for topic in LIVE_TOPICS
            for endpoint in node.get_publishers_info_by_topic(topic)
            if endpoint.node_name == retired_node
        ]
        ready = [client.srv_name for client in clients if client.service_is_ready()]
        if (retired_node, "/") not in names and not publishers and not ready:
            return
        assert time.monotonic() < deadline, (
            f"Retired node {retired_node} is still visible: "
            f"nodes={names}, publishers={publishers}, services={ready}"
        )
        rclpy.spin_once(node, timeout_sec=0.05)


def capture(executable, node, graph, serialize, publisher, odom_tf, prefix):
    with bridge_process(executable):
        assert graph.wait_for_service(timeout_sec=8.0)
        assert serialize.wait_for_service(timeout_sec=4.0)
        snapshot = wait_for_scan(node, graph, publisher, odom_tf, 0.0, 0.37, count=1)
        assert (
            request(node, serialize, SerializePoseGraph.Request(filename=str(prefix))).result == 0
        )
    wait_for_disappearance(node, LIVE_NODE, graph, serialize)
    return snapshot


def candidate_arguments(executable, prefix, digest, attempt):
    arguments = [executable, "--ros-args", "-r", f"__node:={CANDIDATE_NODE}"]
    for name, value in {
        "map_frame": MAP,
        "odom_frame": ODOM,
        "candidate_graph_prefix": str(prefix),
        "candidate_graph_digest": digest,
        "candidate_load_attempt": attempt,
    }.items():
        arguments.extend(["-p", f"{name}:={value}"])
    return arguments


@contextmanager
def candidate(arguments):
    process = subprocess.Popen(arguments)
    try:
        yield process
    finally:
        if process.poll() is None:
            process.terminate()
        try:
            process.wait(timeout=4)
        except subprocess.TimeoutExpired as error:
            process.kill()
            process.wait(timeout=2)
            raise AssertionError("Candidate did not shut down within four seconds") from error


def observe(node, receipt, graph, prefix, attempt, prior_boots, expected):
    assert receipt.wait_for_service(timeout_sec=8.0)
    assert graph.wait_for_service(timeout_sec=4.0)
    loaded = request(node, receipt, ReadLoadedGraph.Request())
    assert loaded.attempt_id == attempt
    assert loaded.pair_digest == pair_digest(prefix)
    assert loaded.boot_id not in prior_boots
    snapshot = read_snapshot(node, graph)
    assert snapshot.boot_id == loaded.boot_id
    assert snapshot.status == ReadGraph.Response.OK
    assert snapshot.keyframes == expected.keyframes
    # The global DDS names-and-types list can include this probe's client endpoints.
    deadline = time.monotonic() + 3.0
    while True:
        services = (
            node.get_service_names_and_types_by_node(CANDIDATE_NODE, "/")
            if (CANDIDATE_NODE, "/") in node.get_node_names_and_namespaces()
            else []
        )
        if {RECEIPT, SERVICE}.issubset(name for name, _ in services):
            break
        assert time.monotonic() < deadline, f"Incomplete candidate service discovery: {services}"
        rclpy.spin_once(node, timeout_sec=0.05)
    assert not any(
        service_type.startswith("slam_toolbox/srv/")
        for _, types in services
        for service_type in types
    ), f"Candidate exposes stock SLAM services: {services}"
    unexpected_publishers = [
        (topic, endpoint.node_namespace, endpoint.node_name)
        for topic in LIVE_TOPICS
        for endpoint in node.get_publishers_info_by_topic(topic)
        if endpoint.node_name != "slam_candidate_probe"
    ]
    assert not unexpected_publishers, f"Unexpected SLAM publishers: {unexpected_publishers}"
    check_no_velocity_publishers(node)
    return loaded.boot_id


def probe(executable, preflight):
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    os.environ["ROS_DOMAIN_ID"] = str(secrets.randbelow(20) + 110)
    rclpy.init(args=[])
    node = rclpy.create_node("slam_candidate_probe")
    try:
        publisher = node.create_publisher(LaserScan, SCAN, qos_profile_sensor_data)
        static_tf = StaticTransformBroadcaster(node)
        odom_tf = TransformBroadcaster(node)
        static_tf.sendTransform(transform(node.get_clock().now().to_msg(), BASE, LASER))
        graph = node.create_client(ReadGraph, SERVICE)
        receipt = node.create_client(ReadLoadedGraph, RECEIPT)
        serialize = node.create_client(SerializePoseGraph, SERIALIZE)
        with tempfile.TemporaryDirectory() as raw:
            root = Path(raw).resolve()
            first, second = root / "first", root / "second"
            prior = capture(executable, node, graph, serialize, publisher, odom_tf, first)
            other = capture(executable, node, graph, serialize, publisher, odom_tf, second)
            node.destroy_client(serialize)
            assert prior.keyframes == other.keyframes
            assert pair_digest(first) != pair_digest(second)

            invalid = root / "invalid"
            invalid.with_suffix(".posegraph").write_bytes(b"invalid graph")
            invalid.with_suffix(".data").write_bytes(b"invalid dataset")
            failures = (
                (first, pair_digest(second)),
                (root / "missing", pair_digest(first)),
                (invalid, pair_digest(invalid)),
            )
            for path, digest in failures:
                with candidate(candidate_arguments(executable, path, digest, "invalid")) as process:
                    assert process.wait(timeout=8) != 0
                assert not receipt.service_is_ready()
                assert not graph.service_is_ready()

            attempts = {prior.boot_id, other.boot_id}
            first_args = candidate_arguments(executable, first, pair_digest(first), "first")
            with candidate(first_args) as process:
                attempts.add(observe(node, receipt, graph, first, "first", attempts, prior))
                assert process.poll() is None
            assert process.returncode == 0
            wait_for_disappearance(node, CANDIDATE_NODE, receipt, graph)

            second_args = candidate_arguments(executable, second, pair_digest(second), "second")
            # Simulate a startup stalled before the native decoder and reap the exact process.
            delay = "import os, sys, time; time.sleep(30); os.execv(sys.argv[1], sys.argv[1:])"
            with candidate([sys.executable, "-c", delay, *second_args]) as abandoned:
                assert not receipt.wait_for_service(timeout_sec=0.4)
            assert abandoned.poll() is not None
            with candidate(second_args) as process:
                observe(node, receipt, graph, second, "second", attempts, other)
                assert process.poll() is None
            assert process.returncode == 0
            wait_for_disappearance(node, CANDIDATE_NODE, receipt, graph)

            command = [
                sys.executable,
                preflight,
                str(first),
                pair_digest(first),
                "--map-frame",
                MAP,
                "--odom-frame",
                ODOM,
            ]
            completed = subprocess.run(command, check=True, stdout=subprocess.PIPE, timeout=20)
            proof = json.loads(completed.stdout)
            assert proof["pair_digest"] == pair_digest(first)
            assert proof["boot_id"] not in attempts
            assert proof["keyframes"] == [
                {"id": frame.id, "x": frame.x, "y": frame.y, "yaw": frame.yaw}
                for frame in prior.keyframes
            ]
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    probe(sys.argv[1], sys.argv[2])
