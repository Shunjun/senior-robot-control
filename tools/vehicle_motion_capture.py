"""Read-only bounded ROS/IPC capture, with explicit missing-stream accounting.

This starts a subscriber node only, never a simulator, ROS daemon or actuator.
JSON nonfinite sensor values are encoded as strings, not silently dropped rays.
"""

from __future__ import annotations

import argparse
import asyncio
import hashlib
import json
import math
import os
import subprocess
import time
from collections import Counter
from pathlib import Path
from typing import Any, TextIO

from senior_control.runtime.ipc import IpcClient, IpcError

TOPICS = {
    "clock": ("/clock", "rosgraph_msgs/msg/Clock"),
    "scan": ("/scan", "sensor_msgs/msg/LaserScan"),
    "scan_motion": ("/motion/scan", "senior_motion_msgs/msg/ScanMotion"),
    "motion": ("/motion/evidence", "senior_motion_msgs/msg/MotionEvidence"),
    "wheel": ("/wheel/odom", "nav_msgs/msg/Odometry"),
    "imu": ("/imu/data", "sensor_msgs/msg/Imu"),
    "command": ("/cmd_vel", "geometry_msgs/msg/Twist"),
    "safety": ("/safety/status", "diagnostic_msgs/msg/DiagnosticArray"),
    "truth": ("/ground_truth/odom", "nav_msgs/msg/Odometry"),
}
MAX_BYTES = 128 * 1024 * 1024


def _path(data: dict[str, Any], *keys: str) -> Any:
    value: Any = data
    for key in keys:
        if not isinstance(value, dict):
            return None
        value = value.get(key)
    return value


def _stamp(value: object) -> float | None:
    if not isinstance(value, dict):
        return None
    sec, nano = value.get("sec"), value.get("nanosec")
    if type(sec) is int and type(nano) is int and 0 <= nano < 10**9:
        return sec + nano / 1e9
    return None


def _safe(value: Any) -> Any:
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)
    if isinstance(value, dict):
        return {
            key: (
                "[redacted]"
                if any(
                    secret in key.lower()
                    for secret in ("password", "credential", "secret", "token", "pairing_key")
                )
                else _safe(item)
            )
            for key, item in value.items()
        }
    if isinstance(value, (list, tuple)):
        return [_safe(item) for item in value]
    return value


def ros_sample(stream: str, message: dict[str, Any], received_steady: float) -> dict[str, Any]:
    stamp = (
        _stamp(message.get("clock"))
        if stream == "clock"
        else _stamp(_path(message, "header", "stamp"))
    )
    data: dict[str, Any] = {}
    if stream in {"wheel", "truth"}:
        data = {
            "vx": _path(message, "twist", "twist", "linear", "x"),
            "wz": _path(message, "twist", "twist", "angular", "z"),
        }
    elif stream == "command":
        data = {"vx": _path(message, "linear", "x"), "wz": _path(message, "angular", "z")}
    elif stream in {"motion", "scan_motion"}:
        start = _stamp(message.get("interval_start"))
        data = {
            "interval_start_ros": start,
            "valid": message.get("valid"),
            "source_epoch": message.get("source_epoch"),
            "reason": message.get("reason"),
        }
        if stream == "motion":
            data.update(
                vx=_path(message, "body_velocity", "linear", "x"),
                wz=_path(message, "body_velocity", "angular", "z"),
            )
        elif stamp is not None and start is not None and stamp > start:
            delta = message.get("body_delta", {})
            x, y, yaw = (delta.get(key) for key in ("x", "y", "theta"))
            if all(type(v) in (int, float) and math.isfinite(v) for v in (x, y, yaw)):
                a = math.sin(yaw) / yaw if abs(yaw) > 1e-8 else 1.0
                b = (1 - math.cos(yaw)) / yaw if abs(yaw) > 1e-8 else yaw / 2
                denominator = a * a + b * b
                if denominator > 1e-12:
                    data.update(
                        vx=(a * x + b * y) / denominator / (stamp - start), wz=yaw / (stamp - start)
                    )
    return {
        "kind": "sample",
        "stream": stream,
        "received_steady": received_steady,
        "stamp_ros": stamp,
        "data": _safe(data),
        "raw": _safe(message),
    }


def capture_summary(counts: dict[str, int], *, ros_error: str | None = None) -> dict[str, Any]:
    missing = [name for name in (*TOPICS, "ipc") if not counts.get(name)]
    return {
        "kind": "summary",
        "counts": dict(counts),
        "missing_streams": missing,
        "complete": not missing and ros_error is None,
        "ros_error": ros_error,
        "physical_stop": "not_evaluated",
        "acceptance_frozen": False,
    }


class TraceWriter:
    def __init__(self, stream: TextIO) -> None:
        self.stream, self.size = stream, 0
        self.counts: Counter[str] = Counter()

    def emit(self, record: dict[str, Any]) -> None:
        text = json.dumps(_safe(record), allow_nan=False, separators=(",", ":")) + "\n"
        self.size += len(text.encode())
        if self.size > MAX_BYTES:
            raise ValueError("128 MiB capture limit reached; recording incomplete")
        self.stream.write(text)
        if record.get("kind") == "sample":
            self.counts[record["stream"]] += 1


def attach_dds_identity(
    record: dict[str, Any], *, publisher_gid: str | None, source_ns: int, received_ns: int
) -> None:
    """Do not replace the algorithm epoch with the transport endpoint identity."""
    record["dds"] = {
        "publisher_gid": publisher_gid,
        "source_timestamp_ns": source_ns,
        "received_timestamp_ns": received_ns,
    }
    record["data"]["producer_epoch"] = record["data"].get("source_epoch")
    record["data"]["publisher_gid"] = publisher_gid


class RosCapture:
    """Lazy ROS imports leave fixture analysis runnable without a ROS installation."""

    def __init__(self, writer: TraceWriter) -> None:
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from rosidl_runtime_py.convert import message_to_ordereddict
        from rosidl_runtime_py.utilities import get_message
        from senior_robot.motion_lifecycle import MotionExecutor, with_dds_info

        self.ros = rclpy
        self.context = rclpy.Context()
        rclpy.init(context=self.context)
        self.node = rclpy.create_node("vehicle_motion_capture", context=self.context)
        self.executor = MotionExecutor(context=self.context)
        self.executor.add_node(self.node)
        try:
            for name, (topic, type_name) in TOPICS.items():

                def make_callback(stream: str) -> Any:
                    def receive(msg: Any, info: Any) -> None:
                        record = ros_sample(stream, message_to_ordereddict(msg), time.monotonic())
                        # //Compatible: Humble drops callback metadata and may omit GID.
                        # MotionExecutor preserves available metadata; missing GID stays unknown.
                        raw_gid = info.get("publisher_gid")
                        gid = bytes(raw_gid).hex() if raw_gid is not None else None
                        attach_dds_identity(
                            record,
                            publisher_gid=gid,
                            source_ns=info["source_timestamp"],
                            received_ns=info["received_timestamp"],
                        )
                        writer.emit(record)

                    return with_dds_info(receive)

                callback = make_callback(name)
                self.node.create_subscription(
                    get_message(type_name), topic, callback, qos_profile_sensor_data
                )
        except BaseException:
            self.close()
            raise

    def spin(self) -> None:
        self.executor.spin_once(timeout_sec=0.0)

    def close(self) -> None:
        self.executor.shutdown()
        self.node.destroy_node()
        self.ros.shutdown(context=self.context)


async def capture(socket: Path, seconds: float, writer: TraceWriter) -> dict[str, Any]:
    ros, ros_error = None, None
    try:
        ros = RosCapture(writer)
    except (ImportError, RuntimeError) as exc:
        ros_error = f"ROS subscriptions unavailable: {type(exc).__name__}"
        writer.emit({"kind": "error", "component": "ros", "detail": ros_error})
    client = IpcClient(socket, timeout=min(0.2, seconds))
    deadline, next_poll = time.monotonic() + seconds, time.monotonic()
    pending: asyncio.Task[dict[str, Any]] | None = None
    try:
        while time.monotonic() < deadline:
            if ros is not None:
                ros.spin()
            now = time.monotonic()
            if pending is None and now >= next_poll:
                pending = asyncio.create_task(client.request("state.get"))
                next_poll = now + 0.25
            if pending is not None and pending.done():
                try:
                    response = pending.result()
                    if response.get("ok") is not True:
                        raise IpcError("state.get refused")
                    writer.emit(
                        {
                            "kind": "sample",
                            "stream": "ipc",
                            "received_steady": time.monotonic(),
                            "stamp_ros": None,
                            "data": _safe(response),
                        }
                    )
                except IpcError:
                    writer.emit(
                        {
                            "kind": "error",
                            "component": "ipc",
                            "received_steady": time.monotonic(),
                            "detail": "state.get failed",
                        }
                    )
                pending = None
            await asyncio.sleep(min(0.005, max(0, deadline - time.monotonic())))
    finally:
        if pending is not None:
            pending.cancel()
            await asyncio.gather(pending, return_exceptions=True)
        await client.close()
        if ros is not None:
            ros.close()
    return capture_summary(dict(writer.counts), ros_error=ros_error)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--socket", required=True, type=Path)
    parser.add_argument("--seconds", required=True, type=float)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 0 < args.seconds <= 3600:
        parser.error("seconds must be finite and in (0, 3600]")
    revision = subprocess.run(
        ["git", "rev-parse", "HEAD"], capture_output=True, text=True, timeout=5, check=False
    ).stdout.strip()
    with args.output.open("x", encoding="utf-8") as stream:
        writer = TraceWriter(stream)
        writer.emit(
            {
                "kind": "manifest",
                "schema": 1,
                "revision": revision,
                "tool_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                "domain": os.environ.get("ROS_DOMAIN_ID"),
                "seconds": args.seconds,
                "socket": str(args.socket),
                "topics": TOPICS,
                "encoding": "nonfinite floats encoded as strings",
                "publisher_identity": (
                    "per-sample DDS GID may be unavailable on Humble; null is not inferred"
                ),
                "qos": "best_effort keep_last=5; receive time is callback time",
                "diagnostic_gaps": [
                    "numeric alignment wait and expiry-specific worker counters "
                    "are not exported by current product; not inferred from text"
                ],
            }
        )
        try:
            result = asyncio.run(capture(args.socket, args.seconds, writer))
        except (ValueError, RuntimeError, KeyboardInterrupt, OSError) as exc:
            result = capture_summary(dict(writer.counts), ros_error=type(exc).__name__)
        # Reserve no additional samples after the hard cap; write a bounded final summary.
        stream.write(json.dumps(result, allow_nan=False) + "\n")
    return 0 if result["complete"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
