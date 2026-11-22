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
from dataclasses import asdict
from pathlib import Path
from typing import Any, TextIO

from vehicle_core.ipc import IpcClient, IpcError
from vehicle_core.sample_time import ClockTracker, SampleTime, map_sample

TOPICS = {
    "clock": ("/clock", "rosgraph_msgs/msg/Clock"),
    "scan": ("/scan", "sensor_msgs/msg/LaserScan"),
    "scan_motion": ("/motion/scan", "senior_interfaces/msg/ScanMotion"),
    "motion": ("/motion/core_reference", "std_msgs/msg/String"),
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


def _stamp(value: object) -> int | None:
    if not isinstance(value, dict):
        return None
    sec, nano = value.get("sec"), value.get("nanosec")
    if type(sec) is int and type(nano) is int and 0 <= nano < 10**9:
        return sec * 1_000_000_000 + nano
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


def ros_sample(
    stream: str,
    message: dict[str, Any],
    received_steady_ns: int,
    *,
    received_ros_ns: int,
    clock_generation: int,
) -> dict[str, Any]:
    stamp = (
        _stamp(message.get("clock"))
        if stream == "clock"
        else _stamp(_path(message, "header", "stamp"))
    )
    sample_time = None
    if stamp is not None and stream not in {"clock", "motion", "scan_motion"}:
        sample_time = asdict(
            map_sample(
                stamp_ns=stamp,
                received_ros_ns=received_ros_ns,
                received_steady_ns=received_steady_ns,
                clock_generation=clock_generation,
            )
        )
    endpoints = {}
    data: dict[str, Any] = {}
    if stream == "clock":
        data = {"clock_ros_ns": stamp}
    elif stream in {"wheel", "truth"}:
        data = {
            "vx": _path(message, "twist", "twist", "linear", "x"),
            "wz": _path(message, "twist", "twist", "angular", "z"),
        }
    elif stream == "command":
        data = {"vx": _path(message, "linear", "x"), "wz": _path(message, "angular", "z")}
    elif stream == "motion":
        try:
            value = json.loads(message["data"])
            reference = value["reference"]
            endpoints = {
                key: asdict(SampleTime(**reference[key])) for key in ("first_time", "last_time")
            }
            sample_time = endpoints["last_time"]
            stamp = sample_time["stamp_ns"]
            average = reference.get("average")
            data = {
                "valid": average is not None,
                "source_epoch": value["boot_id"],
                "sequence": value["sequence"],
                "reason": reference["current_reference"]["reason"],
                "mode": reference["mode"],
                "vx": average.get("vx") if average else None,
                "wz": average.get("wz") if average else None,
            }
        except (ValueError, KeyError, TypeError):
            data = {"valid": False, "reason": "invalid_core_reference"}
    elif stream == "scan_motion":
        start = _stamp(message.get("interval_start"))
        if message.get("valid"):
            endpoints = {
                key: capture_time(message[capture])
                for key, capture in (
                    ("first_time", "first_capture"),
                    ("last_time", "second_capture"),
                )
            }
            sample_time = endpoints["last_time"]
            if start != endpoints["first_time"]["stamp_ns"] or stamp != sample_time["stamp_ns"]:
                raise ValueError("scan motion endpoints differ from original captures")
        data = {
            "valid": message.get("valid"),
            "source_epoch": message.get("source_epoch"),
            "reason": message.get("reason"),
        }
        if stamp is not None and start is not None and stamp > start:
            delta = message.get("body_delta", {})
            x, y, yaw = (delta.get(key) for key in ("x", "y", "theta"))
            if all(type(v) in (int, float) and math.isfinite(v) for v in (x, y, yaw)):
                a = math.sin(yaw) / yaw if abs(yaw) > 1e-8 else 1.0
                b = (1 - math.cos(yaw)) / yaw if abs(yaw) > 1e-8 else yaw / 2
                denominator = a * a + b * b
                if denominator > 1e-12:
                    data.update(
                        vx=(a * x + b * y) / denominator / ((stamp - start) / 1e9),
                        wz=yaw / ((stamp - start) / 1e9),
                    )
    return {
        "kind": "sample",
        "stream": stream,
        "received_steady_ns": received_steady_ns,
        **({"time": sample_time} if sample_time is not None else {}),
        **endpoints,
        "data": _safe(data),
        "raw": _safe(message),
    }


def capture_time(capture: dict[str, Any]) -> dict[str, int]:
    """Decode an existing original capture without remapping at recording time."""
    return asdict(
        SampleTime(
            stamp_ns=_stamp(capture["stamp"]),
            received_ros_ns=_stamp(capture["received_ros"]),
            received_steady_ns=capture["received_steady_ns"],
            sample_steady_ns=capture["sample_steady_ns"],
            clock_generation=capture["clock_generation"],
        )
    )


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
        self.errors = 0

    def emit(self, record: dict[str, Any]) -> None:
        text = json.dumps(_safe(record), allow_nan=False, separators=(",", ":")) + "\n"
        self.size += len(text.encode())
        if self.size > MAX_BYTES:
            raise ValueError("128 MiB capture limit reached; recording incomplete")
        self.stream.write(text)
        if record.get("kind") == "error":
            self.errors += 1
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


class RawCaptureFrontiers:
    """Keep each admitted identity's last original record for this bounded capture.

    Full identity slots are never evicted: an unseen source beyond the recording
    budget is reported as an error rather than allowing an old source to look new.
    """

    MAX_IDENTITIES_PER_STREAM = 64

    def __init__(self) -> None:
        self._streams: dict[str, dict[tuple[bytes, int], tuple[int, dict[str, Any]]]] = {}

    def cached(
        self, stream: str, identity: bytes, generation: int, stamp_ns: int
    ) -> dict[str, Any] | None:
        sources = self._streams.setdefault(stream, {})
        previous = sources.get((identity, generation))
        if previous is None:
            if len(sources) >= self.MAX_IDENTITIES_PER_STREAM:
                raise ValueError("source_identity_capacity")
            return None
        old_stamp, record = previous
        if stamp_ns < old_stamp:
            raise ValueError("out_of_order_stamp")
        return record if stamp_ns == old_stamp else None

    def remember(
        self, stream: str, identity: bytes, generation: int, stamp_ns: int, record: dict[str, Any]
    ) -> None:
        self._streams[stream][identity, generation] = (stamp_ns, record)


def capture_error(
    stream: str, detail: str, *, steady_ns: int, body: Any, info: dict[str, Any]
) -> dict[str, Any]:
    """Retain the rejected callback's time, payload and transport identity."""
    gid = info.get("publisher_gid")
    return {
        "kind": "error",
        "component": stream,
        "detail": detail,
        "received_steady_ns": steady_ns,
        "raw": _safe(body),
        "dds": {
            "publisher_gid": bytes(gid).hex() if gid is not None else None,
            "source_timestamp_ns": info.get("source_timestamp"),
            "received_timestamp_ns": info.get("received_timestamp"),
        },
    }


class RosCapture:
    """Lazy ROS imports leave fixture analysis runnable without a ROS installation."""

    def __init__(self, writer: TraceWriter, *, use_sim_time: bool = False) -> None:
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from rosidl_runtime_py.convert import message_to_ordereddict
        from rosidl_runtime_py.utilities import get_message
        from senior_sensor_ingress.ros_lifecycle import MotionExecutor, with_dds_info

        self.ros = rclpy
        self.context = rclpy.Context()
        rclpy.init(
            args=["--ros-args", "-p", f"use_sim_time:={str(use_sim_time).lower()}"],
            context=self.context,
        )
        self.node = rclpy.create_node("vehicle_motion_capture", context=self.context)
        self.executor = MotionExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.clock = ClockTracker()
        self._raw_frontiers = RawCaptureFrontiers()
        try:
            for name, (topic, type_name) in TOPICS.items():

                def make_callback(stream: str) -> Any:
                    def receive(msg: Any, info: Any) -> None:
                        steady_ns = time.monotonic_ns()
                        body = None
                        try:
                            ros_ns = self.node.get_clock().now().nanoseconds
                            generation = self.clock.observe(
                                ros_ns=ros_ns, steady_ns=steady_ns
                            ).generation
                            body = message_to_ordereddict(msg)
                            stamp_ns = _stamp(_path(body, "header", "stamp"))
                            identity = bytes(info.get("publisher_gid") or b"")
                            raw_stream = stream not in {"clock", "scan_motion", "motion"}
                            cached = None
                            if raw_stream and stamp_ns is not None:
                                cached = self._raw_frontiers.cached(
                                    stream, identity, generation, stamp_ns
                                )
                            if cached is not None:
                                record = {
                                    **cached,
                                    "data": dict(cached["data"]),
                                    "received_steady_ns": steady_ns,
                                }
                            else:
                                record = ros_sample(
                                    stream,
                                    body,
                                    steady_ns,
                                    received_ros_ns=ros_ns,
                                    clock_generation=generation,
                                )
                            # //Compatible: Humble drops callback metadata and may omit GID.
                            # MotionExecutor retains metadata; missing GID stays unknown.
                            raw_gid = info.get("publisher_gid")
                            gid = bytes(raw_gid).hex() if raw_gid is not None else None
                            attach_dds_identity(
                                record,
                                publisher_gid=gid,
                                source_ns=info["source_timestamp"],
                                received_ns=info["received_timestamp"],
                            )
                        except (ValueError, KeyError, TypeError) as exc:
                            writer.emit(
                                capture_error(
                                    stream, str(exc), steady_ns=steady_ns, body=body, info=info
                                )
                            )
                            return
                        if raw_stream and stamp_ns is not None and cached is None:
                            self._raw_frontiers.remember(
                                stream, identity, generation, stamp_ns, record
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


async def capture(
    socket: Path, seconds: float, writer: TraceWriter, *, use_sim_time: bool = False
) -> dict[str, Any]:
    ros, ros_error = None, None
    try:
        ros = RosCapture(writer, use_sim_time=use_sim_time)
    except (ImportError, RuntimeError) as exc:
        ros_error = f"ROS subscriptions unavailable: {type(exc).__name__}"
        writer.emit(
            {
                "kind": "error",
                "component": "ros",
                "detail": ros_error,
                "received_steady_ns": time.monotonic_ns(),
            }
        )
    client = IpcClient(socket, timeout_ms=(min(0.2, seconds)) * 1000)
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
                            "received_steady_ns": time.monotonic_ns(),
                            "data": _safe(response),
                        }
                    )
                except IpcError:
                    writer.emit(
                        {
                            "kind": "error",
                            "component": "ipc",
                            "received_steady_ns": time.monotonic_ns(),
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
    return capture_summary(
        dict(writer.counts),
        ros_error=ros_error or (f"{writer.errors} capture errors" if writer.errors else None),
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--socket", required=True, type=Path)
    parser.add_argument("--seconds", required=True, type=float)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument(
        "--use-sim-time",
        action="store_true",
        help="Pass the standard ROS clock parameter to the capture node",
    )
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
                "schema": 2,
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
            result = asyncio.run(
                capture(args.socket, args.seconds, writer, use_sim_time=args.use_sim_time)
            )
        except (ValueError, RuntimeError, KeyboardInterrupt, OSError) as exc:
            result = capture_summary(dict(writer.counts), ros_error=type(exc).__name__)
        # Reserve no additional samples after the hard cap; write a bounded final summary.
        stream.write(json.dumps(result, allow_nan=False) + "\n")
    return 0 if result["complete"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
