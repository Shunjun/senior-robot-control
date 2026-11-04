#!/usr/bin/env python3
"""Verify a trusted local SLAM pair in a disposable, isolated Humble candidate."""

import argparse
import json
import math
import os
import re
import secrets
import signal
import subprocess
import sys
import time
import uuid
from pathlib import Path


class PreflightError(RuntimeError):
    pass


def validate_proof(receipt, graph, attempt, digest, map_frame, odom_frame):
    try:
        boot = uuid.UUID(receipt.boot_id)
    except ValueError as error:
        raise PreflightError("Candidate returned an invalid boot ID") from error
    if boot.version != 4 or str(boot) != receipt.boot_id:
        raise PreflightError("Candidate returned an invalid boot ID")
    if (receipt.attempt_id, receipt.pair_digest) != (attempt, digest):
        raise PreflightError("Candidate receipt has an unexpected attempt or digest")
    if graph.boot_id != receipt.boot_id:
        raise PreflightError("Graph and receipt belong to different candidate boots")
    if (graph.map_frame, graph.odom_frame) != (map_frame, odom_frame):
        raise PreflightError("Candidate graph has unexpected frames")
    stamp = graph.sample_stamp
    if (
        graph.status != graph.OK
        or not 0 < len(graph.keyframes) <= 16384
        or graph.snapshot_sequence <= 0
        or stamp.sec <= 0
        or not 0 <= stamp.nanosec < 1_000_000_000
    ):
        raise PreflightError("Candidate did not return a complete graph observation")
    keyframes = []
    last_id = -1
    for frame in graph.keyframes:
        if frame.id <= last_id or not all(math.isfinite(v) for v in (frame.x, frame.y, frame.yaw)):
            raise PreflightError("Candidate graph has invalid or unordered keyframes")
        keyframes.append({"id": frame.id, "x": frame.x, "y": frame.y, "yaw": frame.yaw})
        last_id = frame.id
    return {
        "attempt_id": attempt,
        "boot_id": receipt.boot_id,
        "pair_digest": digest,
        "map_frame": map_frame,
        "odom_frame": odom_frame,
        "snapshot_sequence": graph.snapshot_sequence,
        "sample_stamp": {"sec": stamp.sec, "nanosec": stamp.nanosec},
        "keyframes": keyframes,
    }


def run_candidate(command, observe, canceled, *, startup_seconds=8.0, cleanup_seconds=2.0):
    if canceled():
        raise PreflightError("Preflight canceled")
    deadline = time.monotonic() + startup_seconds
    process = subprocess.Popen(command, stdout=sys.stderr, start_new_session=True)

    def check():
        if canceled():
            raise PreflightError("Preflight canceled")
        if process.poll() is not None:
            raise PreflightError(f"Candidate exited during preflight ({process.returncode})")
        if time.monotonic() >= deadline:
            raise PreflightError("Candidate startup or observation timed out")

    try:
        proof = observe(process, check)
        check()
    finally:
        if process.poll() is None:
            process.terminate()
        try:
            process.wait(timeout=cleanup_seconds)
        except subprocess.TimeoutExpired:
            process.kill()
            try:
                process.wait(timeout=cleanup_seconds)
            except subprocess.TimeoutExpired as error:
                raise PreflightError("Candidate cleanup unconfirmed after SIGKILL") from error
    if process.returncode != 0:
        raise PreflightError(f"Candidate did not shut down cleanly ({process.returncode})")
    if canceled():
        raise PreflightError("Preflight canceled")
    return proof


def read_service(node, client, request, check):
    import rclpy

    while not client.service_is_ready():
        check()
        rclpy.spin_once(node, timeout_sec=0.05)
    check()
    response = client.call_async(request)
    while not response.done():
        check()
        rclpy.spin_once(node, timeout_sec=0.05)
    check()
    return response.result()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("prefix", type=Path, help="Absolute local native file prefix")
    parser.add_argument(
        "digest", help="SHA-256 over the raw SHA-256 digests of .posegraph then .data"
    )
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--odom-frame", default="odom")
    args = parser.parse_args()
    if not args.prefix.is_absolute() or not re.fullmatch("[0-9a-f]{64}", args.digest):
        parser.error("expected an absolute staged prefix and a lowercase SHA-256 digest")
    if (
        not args.map_frame
        or not args.odom_frame
        or args.map_frame == args.odom_frame
        or max(len(args.map_frame), len(args.odom_frame)) > 128
    ):
        parser.error("expected distinct nonempty map and odom frames of at most 128 characters")

    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from senior_slam_bridge.srv import ReadGraph, ReadLoadedGraph

    attempt = secrets.token_hex(16)
    namespace = f"/senior/slam/preflight_{attempt}"
    receipt_service = f"{namespace}/receipt"
    graph_service = f"{namespace}/graph"
    executable = Path(__file__).absolute().with_name("slam_graph_node")
    command = [
        str(executable),
        "--ros-args",
        "-r",
        f"__node:=slam_preflight_{attempt}",
        "-r",
        f"/senior/slam/read_loaded_graph:={receipt_service}",
        "-r",
        f"/senior/slam/read_graph:={graph_service}",
    ]
    for name, value in (
        ("candidate_graph_prefix", str(args.prefix)),
        ("candidate_graph_digest", args.digest),
        ("candidate_load_attempt", attempt),
        ("map_frame", args.map_frame),
        ("odom_frame", args.odom_frame),
    ):
        command.extend(["-p", f"{name}:={json.dumps(value, ensure_ascii=False)}"])

    # Both processes use the same isolated loopback domain and per-attempt names.
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    os.environ["ROS_DOMAIN_ID"] = str(215 + secrets.randbelow(15))
    interrupted = False

    def cancel(_signal, _frame):
        nonlocal interrupted
        interrupted = True

    signal.signal(signal.SIGINT, cancel)
    signal.signal(signal.SIGTERM, cancel)
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    try:
        node = rclpy.create_node(f"slam_preflight_observer_{attempt}")
        try:
            receipt = node.create_client(ReadLoadedGraph, receipt_service)
            graph = node.create_client(ReadGraph, graph_service)

            def observe(_process, check):
                loaded = read_service(node, receipt, ReadLoadedGraph.Request(), check)
                observed = read_service(node, graph, ReadGraph.Request(), check)
                return validate_proof(
                    loaded, observed, attempt, args.digest, args.map_frame, args.odom_frame
                )

            proof = run_candidate(command, observe, lambda: interrupted)
        finally:
            node.destroy_node()
    except (OSError, RuntimeError) as error:
        print(f"Candidate preflight failed: {error}", file=sys.stderr)
        return 1
    finally:
        rclpy.shutdown()
    print(json.dumps(proof, allow_nan=False))
    return 0


if __name__ == "__main__":
    sys.exit(main())
