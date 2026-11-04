#!/usr/bin/env python3
"""Verify a trusted local SLAM pair in a disposable, isolated Humble candidate."""

import argparse
import json
import math
import os
import re
import secrets
import signal
import stat
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


_MAX_SCAN_BYTES = 64 * 1024


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("duplicate JSON key")
        result[key] = value
    return result


def _finite(value):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError("expected a finite JSON number")
    return float(value)


def read_stationary_scan(path):
    """Read one explicit native-convention query; JSON null means no return."""
    try:
        with open(
            path,
            "rb",
            opener=lambda name, flags: os.open(name, flags | os.O_NONBLOCK | os.O_NOFOLLOW),
        ) as stream:
            info = os.fstat(stream.fileno())
            if not stat.S_ISREG(info.st_mode) or info.st_size > _MAX_SCAN_BYTES:
                raise ValueError("expected a regular scan file of at most 64 KiB")
            raw = stream.read(_MAX_SCAN_BYTES + 1)
        if len(raw) > _MAX_SCAN_BYTES:
            raise ValueError("scan exceeds 64 KiB")
        document = json.loads(raw.decode("utf-8"), object_pairs_hook=_unique_object)
        if type(document) is not dict or document.keys() != {
            "seed_x",
            "seed_y",
            "seed_yaw",
            "first_angle",
            "angle_step",
            "ranges",
        }:
            raise ValueError(
                "expected seed_x, seed_y, seed_yaw, first_angle, angle_step and ranges"
            )
        samples = document.pop("ranges")
        if type(samples) is not list or not 32 <= len(samples) <= 2048:
            raise ValueError("expected 32 to 2048 range samples")
        ranges = [math.inf if sample is None else _finite(sample) for sample in samples]
        values = {key: _finite(value) for key, value in document.items()}
        if (
            max(abs(values["seed_x"]), abs(values["seed_y"])) > 1_000_000
            or max(abs(values["seed_yaw"]), abs(values["first_angle"])) > math.tau
            or not 0 < values["angle_step"] * (len(ranges) - 1) <= math.tau + 1e-5
            or any(sample < 0 for sample in ranges)
            or sum(0 < sample < 8 for sample in ranges) < 32
        ):
            raise ValueError("scan values are outside the candidate diagnostic bounds")
        return {**values, "ranges": ranges}
    except (OSError, ValueError, OverflowError, RecursionError) as error:
        raise PreflightError(f"Invalid stationary scan file: {error}") from error


def validate_stationary_diagnostic(result, proof):
    if result.status != result.COMPLETE:
        status = {
            result.UNAVAILABLE: "unavailable",
            result.INVALID: "invalid",
            result.SPENT: "already spent",
        }.get(result.status, "unknown")
        raise PreflightError(f"Stationary match diagnostic {status}")
    ids = list(result.reference_ids)
    keyframe_ids = {frame["id"] for frame in proof["keyframes"]}
    if (
        not all(
            math.isfinite(value)
            for value in (result.response, result.fitted_x, result.fitted_y, result.fitted_yaw)
        )
        or result.response < 0
        or not 1 <= len(ids) <= 8
        or len(set(ids)) != len(ids)
        or not set(ids) <= keyframe_ids
    ):
        raise PreflightError("Invalid stationary match diagnostic result")
    return {
        "response": result.response,
        "fitted_sensor_pose": {
            "x": result.fitted_x,
            "y": result.fitted_y,
            "yaw": result.fitted_yaw,
        },
        "reference_ids": ids,
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
    parser.add_argument(
        "--stationary-scan",
        type=Path,
        help="One JSON scan with a map-frame sensor-pose seed and native Karto angles/ranges",
    )
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

    stationary_scan = None
    if args.stationary_scan is not None:
        try:
            stationary_scan = read_stationary_scan(args.stationary_scan)
        except PreflightError as error:
            parser.error(str(error))

    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from senior_slam_bridge.srv import ReadGraph, ReadLoadedGraph

    attempt = secrets.token_hex(16)
    namespace = f"/senior/slam/preflight_{attempt}"
    receipt_service = f"{namespace}/receipt"
    graph_service = f"{namespace}/graph"
    match_service = f"{namespace}/stationary_match"
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

    if stationary_scan is not None:
        from senior_slam_bridge.srv import MatchStationaryScan

        command.extend(
            [
                "-p",
                "candidate_stationary_match:=true",
                "-r",
                f"/senior/slam/match_stationary_scan:={match_service}",
            ]
        )

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
                proof = validate_proof(
                    loaded, observed, attempt, args.digest, args.map_frame, args.odom_frame
                )
                if stationary_scan is not None:
                    match = node.create_client(MatchStationaryScan, match_service)
                    request = MatchStationaryScan.Request(
                        boot_id=proof["boot_id"],
                        attempt_id=attempt,
                        pair_digest=args.digest,
                        map_frame=args.map_frame,
                        **stationary_scan,
                    )
                    result = read_service(node, match, request, check)
                    proof["stationary_match_diagnostic"] = validate_stationary_diagnostic(
                        result, proof
                    )
                return proof

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
