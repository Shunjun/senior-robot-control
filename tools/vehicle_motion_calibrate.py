"""Validate or explicitly execute a reviewed, isolated Gazebo stimulus.

Validation is the default. Execution requires the exact plan SHA, a dedicated
simulation domain/command topic and continuously verified simulation evidence.
No vehicle credentials or production profile are accepted by this tool.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import re
import signal
import sqlite3
import subprocess
import time
from collections.abc import Callable
from concurrent.futures import ThreadPoolExecutor
from contextlib import closing
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Literal, Protocol

from pydantic import BaseModel, ConfigDict, Field, model_validator

PERIOD = 0.05
ALLOWED_NODES = {
    "drive",
    "controller_manager",
    "senior_sim_bridge",
    "robot_state_publisher",
    "joint_state_broadcaster",
}
# Reviewed observation-only roles used by vehicle_motion_fixture; actuator exclusivity is separate.
OBSERVATION_NODES = {"vehicle_motion_capture", "scan_adapter", "scan_motion_node", "scan_receipt"}


class Segment(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True, allow_inf_nan=False)
    seconds: float = Field(gt=0, le=15)
    vx: float
    wz: float


class CalibrationPlan(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True, allow_inf_nan=False)
    schema_version: Literal[1] = Field(alias="schema")
    kind: Literal["isolated_gazebo_calibration"]
    run_id: str = Field(pattern=r"^[a-z][a-z0-9_]{0,23}$")
    robot_id: Literal["senior_4wd_dl"]
    ros_domain_id: int = Field(ge=100, le=199)
    target_topic: str
    world: Literal["vehicle_motion_calibration"]
    field_bounds_m: list[float] = Field(min_length=4, max_length=4)
    max_linear_mps: float = Field(gt=0, le=25 / 18)
    max_angular_rps: float = Field(gt=0, le=1)
    max_accel_mps2: float = Field(gt=0, le=1)
    max_yaw_accel_rps2: float = Field(gt=0, le=2)
    segments: list[Segment] = Field(min_length=3, max_length=20)
    hard_seconds: float = Field(gt=0, le=30, exclude=True)

    @classmethod
    def from_dict(cls, data: dict[str, Any], *, hard_seconds: float) -> CalibrationPlan:
        if "hard_seconds" in data:
            raise ValueError("hard_seconds is a CLI ceiling, not a plan override")
        return cls.model_validate({**data, "hard_seconds": hard_seconds})

    @model_validator(mode="after")
    def bounded_stimulus(self) -> CalibrationPlan:
        if self.target_topic != f"/vehicle_motion_calibration/{self.run_id}/cmd_vel":
            raise ValueError("dedicated command topic required; /cmd_vel is forbidden")
        if sum(s.seconds for s in self.segments) + PERIOD > self.hard_seconds:
            raise ValueError("stimulus plus final zero exceeds hard deadline")
        for segment in self.segments:
            if (
                abs(segment.vx) > self.max_linear_mps
                or abs(segment.wz) > self.max_angular_rps
                or abs(segment.vx) + 0.481154 / 2 * abs(segment.wz) > 0.076 * 20
            ):
                raise ValueError("segment exceeds declared linear/yaw or wheel envelope")
        if any(
            (self.segments[0].vx, self.segments[0].wz, self.segments[-1].vx, self.segments[-1].wz)
        ):
            raise ValueError("first and last segment must be stationary targets")
        steps = command_schedule(self.segments, self.max_accel_mps2, self.max_yaw_accel_rps2)
        zero_hold = 0.0
        previous_zero = True
        for step in steps:
            zero = abs(step.vx) <= 1e-12 and abs(step.wz) <= 1e-12
            zero_hold = zero_hold + step.delay if zero and previous_zero else 0.0
            previous_zero = zero
        if not previous_zero or zero_hold < PERIOD - 1e-9:
            raise ValueError("complete trajectory needs a regular zero command held for one tick")
        xmin, ymin, xmax, ymax = self.field_bounds_m
        travel = sum(abs(s.vx) * s.seconds for s in self.segments) + 3.0
        if not (xmin < -travel and ymin < -travel and xmax > travel and ymax > travel):
            raise ValueError("field must contain origin plus worst path length and 3 m guard")
        return self


def verified_passive_roles(
    plan: CalibrationPlan, endpoints: dict[str, tuple[str, set[str], set[str]]]
) -> set[str]:
    """Prove the two additional roles observed in the reviewed Humble fixture.

    The recorder may subscribe to the command but can never publish one. The
    plugin metadata node is distinct from the drive actuator/controller nodes.
    """
    recorder = f"vehicle_motion_recorder_{plan.run_id}"
    roles = {
        recorder: (
            {"/rosout", "/events/write_split"},
            {
                "/parameter_events",
                "/clock",
                "/ground_truth/odom",
                "/wheel/odom",
                "/joint_states",
                "/imu/data",
                "/scan",
                "/motion/scan",
                "/safety/raw/scan",
                "/perception/scan_candidate",
                "/perception/scan_filtered",
                "/perception/scan_status",
                plan.target_topic,
            },
        ),
        "gz_ros2_control": ({"/rosout", "/parameter_events"}, {"/parameter_events"}),
    }
    return {
        name
        for name, (namespace, publications, subscriptions) in endpoints.items()
        if name in roles
        and namespace == "/"
        and publications <= roles[name][0]
        and subscriptions <= roles[name][1]
    }


@dataclass(frozen=True)
class NativeBagReceipts:
    counts: dict[str, int]
    missing: tuple[str, ...]
    storage_errors: tuple[str, ...]


def native_bag_receipts(bag: Path, required: tuple[str, ...]) -> NativeBagReceipts:
    """Inspect a finalized, stopped native bag; never query a live recorder.

    Metadata is only a finalization marker; the caller must also own and stop
    the recorder. Live reads can lock Humble SQLite writes. One sample does
    not prove complete-window quality.
    """
    counts = dict.fromkeys(required, 0)
    if not (bag / "metadata.yaml").is_file():
        return NativeBagReceipts(counts, required, ("bag_not_finalized",))
    errors = []
    for path in sorted(bag.glob("*.db3")):
        try:
            with closing(
                sqlite3.connect(path.resolve().as_uri() + "?mode=ro", uri=True, timeout=0.05)
            ) as db:
                rows = db.execute(
                    "SELECT topics.name, COUNT(messages.id) FROM topics "
                    "LEFT JOIN messages ON messages.topic_id=topics.id GROUP BY topics.name"
                )
                for topic, count in rows:
                    if topic in counts:
                        counts[topic] += count
        except sqlite3.Error as error:
            errors.append(type(error).__name__)
    return NativeBagReceipts(counts, tuple(t for t in required if counts[t] == 0), tuple(errors))


class GraphValidationError(ValueError):
    """Carry the exact read-only snapshot when discovery or exclusivity fails."""

    def __init__(self, reason: str, graph: dict[str, Any]) -> None:
        self.graph = graph.copy()
        super().__init__(reason + "; graph=" + json.dumps(self.graph, sort_keys=True))


def validate_graph(plan: CalibrationPlan, graph: dict[str, Any]) -> None:
    watchdog = graph.get("watchdog_seconds")
    if (
        not isinstance(watchdog, (float, int))
        or isinstance(watchdog, bool)
        or not math.isfinite(watchdog)
        or not 0 < watchdog <= 0.3
        or graph.get("unstamped_command") is not True
    ):
        raise GraphValidationError(
            "bounded controller watchdog and unstamped command required", graph
        )
    verified_tf = {
        name
        for name in graph.get("verified_tf_listeners", [])
        if re.fullmatch(r"transform_listener_impl_[a-fA-F0-9]{1,32}", name)
    }
    passive = set(graph.get("verified_passive_roles", [])) & {
        f"vehicle_motion_recorder_{plan.run_id}",
        "gz_ros2_control",
    }
    subscribers = graph.get("target_subscribers", [])
    if (
        graph.get("domain") != plan.ros_domain_id
        or graph.get("sim_time") is not True
        or graph.get("clock_source") != "senior_sim_bridge"
        or graph.get("truth_source") != "senior_sim_bridge"
        or graph.get("target_publishers") != 0
        or subscribers.count("drive") != 1
        or len(subscribers) != len(set(subscribers))
        or set(subscribers) - ({"drive"} | (passive & {f"vehicle_motion_recorder_{plan.run_id}"}))
        or set(graph.get("nodes", [])) - (ALLOWED_NODES | OBSERVATION_NODES | verified_tf | passive)
        or not {"drive", "controller_manager", "senior_sim_bridge"} <= set(graph.get("nodes", []))
        or graph.get("gazebo_worlds") != [plan.world]
    ):
        raise GraphValidationError(
            "graph is not an exclusive simulation calibration fixture", graph
        )


@dataclass(frozen=True)
class SimulationSample:
    stamp_ros: float
    received_steady: float
    x: float
    y: float
    vx: float
    vy: float
    wz: float


def validate_motion_evidence(
    plan: CalibrationPlan,
    *,
    clock: tuple[float, float] | None,
    clock_advanced: float,
    truth: SimulationSample | None,
    now: float,
    require_origin: bool = False,
) -> None:
    if (
        clock is None
        or truth is None
        or not 0 <= now - clock[0] <= 0.25
        or not 0 <= now - clock_advanced <= 0.25
        or not 0 <= now - truth.received_steady <= 0.25
        or not 0 <= clock[1] - truth.stamp_ros <= 0.25
    ):
        raise RuntimeError("fresh advancing Gazebo clock and source-timed truth required")
    if not all(math.isfinite(v) for v in (truth.x, truth.y, truth.vx, truth.vy, truth.wz)):
        raise RuntimeError("nonfinite simulation pose or speed")
    xmin, ymin, xmax, ymax = plan.field_bounds_m
    if not (xmin + 3 < truth.x < xmax - 3 and ymin + 3 < truth.y < ymax - 3):
        raise RuntimeError("simulation field guard reached")
    if require_origin and (
        math.hypot(truth.x, truth.y) > 0.25
        or math.hypot(truth.vx, truth.vy) > 0.01
        or abs(truth.wz) > 0.01
    ):
        raise RuntimeError("fixture must start still at origin")


class CalibrationPort(Protocol):
    def monotonic(self) -> float: ...
    def wait(self, seconds: float) -> None: ...
    def check(self) -> None: ...
    def publish(self, vx: float, wz: float) -> None: ...
    def close(self) -> None: ...


def _approach(value: float, target: float, max_step: float) -> float:
    return value + max(-max_step, min(max_step, target - value))


@dataclass(frozen=True)
class CommandStep:
    delay: float
    vx: float
    wz: float
    segment: int


def command_schedule(segments: list[Segment], accel: float, yaw_accel: float) -> list[CommandStep]:
    """Carry both velocities through every segment, including sub-tick target changes."""
    result: list[CommandStep] = []
    vx = wz = 0.0
    for index, segment in enumerate(segments):
        remaining = segment.seconds
        while remaining > 1e-9:
            delay = min(PERIOD, remaining)
            vx = _approach(vx, segment.vx, accel * delay)
            wz = _approach(wz, segment.wz, yaw_accel * delay)
            result.append(CommandStep(delay, vx, wz, index))
            remaining -= delay
    return result


def execute_plan(
    plan: CalibrationPlan, port: CalibrationPort, emit: Callable[[dict[str, Any]], None]
) -> None:
    """Execute the validated schedule without skipping ramp steps to catch up.

    Each requested delay is a minimum hold; scheduling/check latency only slows
    the ramp. A deadline overrun is exceptional, never normal ramp completion.
    """
    started = previous = port.monotonic()
    try:
        for step in command_schedule(plan.segments, plan.max_accel_mps2, plan.max_yaw_accel_rps2):
            port.wait(step.delay)
            now = port.monotonic()
            if now - previous < step.delay - 1e-9 or now - previous > 0.2:
                raise RuntimeError("calibration scheduler gap exceeded")
            port.check()
            checked = port.monotonic()
            if checked - now > 0.2 or checked - started >= plan.hard_seconds or checked < now:
                raise RuntimeError("simulation check consumed the scheduling budget")
            port.publish(step.vx, step.wz)
            emit(
                {
                    "kind": "stimulus",
                    "segment": step.segment,
                    "steady": checked,
                    "vx_mps": step.vx,
                    "wz_rps": step.wz,
                }
            )
            previous = checked
    finally:
        try:
            port.publish(0.0, 0.0)
            port.wait(PERIOD)
            emit(
                {
                    "kind": "cleanup",
                    "steady": port.monotonic(),
                    "zero_requested": True,
                    "physical_stop": "not_evaluated",
                }
            )
        finally:
            port.close()


def gazebo_worlds() -> list[str]:
    # //Compatible: Fortress's ign wrapper adds version discovery latency; invoke
    # the installed transport11 list-only CLI directly, as confirmed by its help.
    binary = Path("/usr/libexec/gz/transport11/ign-transport-topic")
    command = [str(binary), "-l"] if binary.is_file() else ["ign", "topic", "-l"]
    # Transport11 list waits a fixed 2 s for discovery (measured 2.014–2.016 s).
    # Allow bounded process overhead; the encompassing 8 s alarm is unchanged.
    output = subprocess.run(
        command, capture_output=True, text=True, check=True, timeout=2.5
    ).stdout.splitlines()
    return sorted({m.group(1) for line in output if (m := re.match(r"^/world/([^/]+)/", line))})


class RosCalibrationPort:
    """Own one publisher after a fail-closed graph and simulation-time check."""

    def __init__(self, plan: CalibrationPlan) -> None:
        import rclpy
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from rcl_interfaces.srv import GetParameters
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import qos_profile_sensor_data
        from rosgraph_msgs.msg import Clock

        self.plan, self.ros, self.twist = plan, rclpy, Twist
        self.context = rclpy.Context()
        self.ros.init(context=self.context)
        self.node = rclpy.create_node(
            f"vehicle_motion_calibrator_{plan.run_id}", context=self.context
        )
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.publisher = None
        self.clock: tuple[float, float] | None = None
        self.clock_advanced = 0.0
        self.truth: SimulationSample | None = None
        self._last_graph_check = float("-inf")
        self._preflight_parameters: dict[str, Any] = {}

        def clock(msg: Any) -> None:
            stamp = msg.clock.sec + msg.clock.nanosec / 1e9
            now = time.monotonic()
            if self.clock and stamp > self.clock[1]:
                self.clock_advanced = now
            elif self.clock and stamp < self.clock[1]:
                self.clock_advanced = 0.0
            self.clock = (now, stamp)

        def truth(msg: Any) -> None:
            self.truth = SimulationSample(
                msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
                time.monotonic(),
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.angular.z,
            )

        self.node.create_subscription(Clock, "/clock", clock, qos_profile_sensor_data)
        self.node.create_subscription(
            Odometry, "/ground_truth/odom", truth, qos_profile_sensor_data
        )
        try:
            # Independent ROS and Gazebo discovery overlap within the same hard
            # deadline. No motion publisher exists during either readiness probe.
            client = self.node.create_client(GetParameters, "/drive/get_parameters")
            try:
                with ThreadPoolExecutor(max_workers=1) as probes:
                    worlds = probes.submit(gazebo_worlds)
                    deadline = time.monotonic() + 3
                    while time.monotonic() < deadline and (
                        not self.truth or not self.clock_advanced
                    ):
                        self.executor.spin_once(timeout_sec=0.05)
                    self._preflight(client, worlds.result())
            finally:
                self.node.destroy_client(client)
            # All checks complete before creating an actuator publisher.
            self.publisher = self.node.create_publisher(Twist, plan.target_topic, 1)
        except BaseException:
            self.close()
            raise

    def _preflight(self, client: Any, worlds: list[str]) -> None:
        from rcl_interfaces.srv import GetParameters

        if not client.wait_for_service(timeout_sec=0.5):
            raise ValueError("simulation drive parameter service unavailable")
        request = GetParameters.Request()
        request.names = ["use_sim_time", "cmd_vel_timeout", "use_stamped_vel"]
        future = client.call_async(request)
        self.executor.spin_until_future_complete(future, timeout_sec=0.5)
        result = future.result() if future.done() else None
        sim_time = bool(
            result
            and len(result.values) == 3
            and result.values[0].type == 1
            and result.values[0].bool_value
        )
        watchdog = (
            result.values[1].double_value
            if result and len(result.values) == 3 and result.values[1].type == 3
            else None
        )
        unstamped = bool(
            result
            and len(result.values) == 3
            and result.values[2].type == 1
            and not result.values[2].bool_value
        )
        graph = self._graph() | {
            "sim_time": sim_time,
            "gazebo_worlds": worlds,
            "watchdog_seconds": watchdog,
            "unstamped_command": unstamped,
        }
        validate_graph(self.plan, graph)
        self._preflight_parameters = {
            "sim_time": sim_time,
            "gazebo_worlds": worlds,
            "watchdog_seconds": watchdog,
            "unstamped_command": unstamped,
        }
        # Read-only service/transport discovery may queue sensor callbacks. Drain
        # for one bounded tick before checking source age, without weakening it.
        self.wait(PERIOD)
        self._check_motion_evidence(require_origin=True)

    def _graph(self) -> dict[str, Any]:
        def sources(topic: str) -> list[str]:
            return [info.node_name for info in self.node.get_publishers_info_by_topic(topic)]

        clock, truth = sources("/clock"), sources("/ground_truth/odom")
        own_name = self.node.get_name()
        publishers = self.node.get_publishers_info_by_topic(self.plan.target_topic)
        if self.publisher is None:
            foreign_publishers = len(publishers)
        else:
            own_endpoint = (
                len(publishers) == 1
                and publishers[0].node_name == own_name
                and publishers[0].node_namespace == "/"
            )
            foreign_publishers = 0 if own_endpoint else max(1, len(publishers))
        verified_tf = []
        passive_endpoints = {}
        for name, namespace in self.node.get_node_names_and_namespaces():
            is_tf = bool(re.fullmatch(r"transform_listener_impl_[a-fA-F0-9]{1,32}", name))
            if (
                name not in {f"vehicle_motion_recorder_{self.plan.run_id}", "gz_ros2_control"}
                and not is_tf
            ):
                continue
            publications = {
                topic
                for topic, _ in self.node.get_publisher_names_and_types_by_node(name, namespace)
            }
            subscriptions = {
                topic
                for topic, _ in self.node.get_subscriber_names_and_types_by_node(name, namespace)
            }
            passive_endpoints[name] = (namespace, publications, subscriptions)
            if (
                is_tf
                and namespace == "/"
                and publications <= {"/rosout", "/parameter_events"}
                and subscriptions <= {"/tf", "/tf_static", "/clock", "/parameter_events"}
                and {"/tf", "/tf_static"} <= subscriptions
            ):
                verified_tf.append(name)
        return {
            "domain": self.context.get_domain_id(),
            "verified_tf_listeners": verified_tf,
            "verified_passive_roles": sorted(verified_passive_roles(self.plan, passive_endpoints)),
            "clock_source": clock[0] if len(clock) == 1 else None,
            "truth_source": truth[0] if len(truth) == 1 else None,
            "target_publishers": foreign_publishers,
            "target_subscribers": sorted(
                info.node_name
                for info in self.node.get_subscriptions_info_by_topic(self.plan.target_topic)
            ),
            "nodes": [name for name in self.node.get_node_names() if name != own_name],
        }

    def _check_motion_evidence(self, *, require_origin: bool = False) -> None:
        validate_motion_evidence(
            self.plan,
            clock=self.clock,
            clock_advanced=self.clock_advanced,
            truth=self.truth,
            now=time.monotonic(),
            require_origin=require_origin,
        )

    def monotonic(self) -> float:
        return time.monotonic()

    def wait(self, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=min(0.005, deadline - time.monotonic()))

    def check(self) -> None:
        self.executor.spin_once(timeout_sec=0.0)
        self._check_motion_evidence()
        if time.monotonic() - self._last_graph_check >= 0.1:
            graph = self._graph() | self._preflight_parameters
            validate_graph(self.plan, graph)
            self._last_graph_check = time.monotonic()

    def publish(self, vx: float, wz: float) -> None:
        if self.publisher is not None:
            msg = self.twist()
            msg.linear.x, msg.angular.z = vx, wz
            self.publisher.publish(msg)

    def close(self) -> None:
        self.executor.shutdown()
        self.node.destroy_node()
        self.ros.shutdown(context=self.context)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", required=True, type=Path)
    parser.add_argument("--seconds", required=True, type=float)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--reviewed-sha256")
    args = parser.parse_args()
    raw = args.plan.read_bytes()
    digest = hashlib.sha256(raw).hexdigest()
    plan = CalibrationPlan.from_dict(json.loads(raw), hard_seconds=args.seconds)
    if args.execute and (
        args.reviewed_sha256 != digest or os.environ.get("ROS_DOMAIN_ID") != str(plan.ros_domain_id)
    ):
        parser.error("execution requires reviewed plan SHA256 and matching isolated ROS_DOMAIN_ID")
    port: RosCalibrationPort | None = None
    with args.output.open("x", encoding="utf-8") as output:

        def emit(event: dict[str, Any]) -> None:
            record = {
                **event,
                "emitted_steady": time.monotonic(),
                "emitted_wall_ns": time.time_ns(),
            }
            if port is not None and port.clock is not None:
                record["observed_clock_ros"] = port.clock[1]
                record["clock_received_steady"] = port.clock[0]
            output.write(json.dumps(record, allow_nan=False) + "\n")
            output.flush()

        emit(
            {
                "kind": "plan",
                "sha256": digest,
                "execute": args.execute,
                "hard_work_deadline_s": args.seconds,
                "cleanup_delivery_grace_s": PERIOD,
                "tool_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                "plan": plan.model_dump(by_alias=True),
                "physical_stop": "not_evaluated",
            }
        )
        if not args.execute:
            return 0

        def interrupt(signum: int, frame: Any) -> None:
            raise InterruptedError(f"calibration interrupted by signal {signum}")

        previous = {
            sig: signal.signal(sig, interrupt)
            for sig in (signal.SIGTERM, signal.SIGINT, signal.SIGALRM)
        }
        try:
            # Alarm covers preflight as well as stimulus; the per-tick deadline is independent.
            signal.setitimer(signal.ITIMER_REAL, args.seconds)
            port = RosCalibrationPort(plan)
            execute_plan(plan, port, emit)
        finally:
            signal.setitimer(signal.ITIMER_REAL, 0)
            for sig, handler in previous.items():
                signal.signal(sig, handler)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
