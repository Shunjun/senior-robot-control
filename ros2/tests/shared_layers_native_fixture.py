"""Opt-in real Launch composition, immutable capture matching and exit evidence.

Imports ROS only on entry. Never fabricates selected reference, permits, Q or coverage.
"""

import asyncio
import hashlib
import importlib.util
import json
import math
import os
import re
import subprocess
import sys
import time
from collections import OrderedDict, deque
from dataclasses import asdict, replace
from importlib.machinery import SourceFileLoader
from pathlib import Path

from tools.shared_layers_system_trace import (
    KEY_FIELDS,
    ResourceSampler,
    TraceWriter,
    alive,
    identity,
)


class CaptureIndex:
    def __init__(self, *, capacity=512):
        self.capacity, self.evicted = capacity, 0
        self.rows = OrderedDict()

    def add(self, stage: str, row: dict):
        assert row["deadline_ns"] == row["sample_steady_ns"] + 500_000_000, "deadline renewed"
        key = identity(row)
        stages = self.rows.setdefault(key, {})
        for other in stages.values():
            for field in ("stamp_ns", "sample_steady_ns", "deadline_ns"):
                assert row[field] == other[field], f"same capture changed {field}"
        stages[stage] = row
        self.rows.move_to_end(key)
        if len(self.rows) > self.capacity:
            self.rows.popitem(last=False)
            self.evicted += 1

    def pair(self, row: dict, *stages):
        found = self.rows.get(identity(row), {})
        return tuple(found[stage] for stage in stages) if all(s in found for s in stages) else None

    def coverage(self, row: dict):
        found = self.rows.get(identity(row), {})
        q = found.get("q")
        if q is None or not all(stage in found for stage in ("tf", "slot")):
            return None
        if "map_id" in q:
            assert (row["map_id"], row["reference_revision"], row["reference_generation"]) == (
                q["map_id"],
                q["reference_revision"],
                q["map_generation"],
            )
        assert row["sample_steady_ns"] == q["sample_steady_ns"]
        assert row["scan_deadline_steady_ns"] == q["deadline_ns"]
        assert row["cycle_completed_steady_ns"] < row["valid_until_steady_ns"] <= q["deadline_ns"]
        assert row["cleanup_complete"] and row["transform_valid"]
        return q


def capture_record(backend: str, host: str, capture, *, deadline: int) -> dict:
    timed = getattr(capture, "time", capture)
    stamp = getattr(timed, "stamp_ns", None)
    if stamp is None:
        stamp = timed.stamp.sec * 1_000_000_000 + timed.stamp.nanosec
    key = {
        name: getattr(capture, name)
        for name in KEY_FIELDS
        if name not in ("backend_boot", "clock_generation")
    }
    key.update(backend_boot=backend, clock_generation=timed.clock_generation)
    return dict(
        host_boot_id=host,
        key=key,
        stamp_ns=stamp,
        sample_steady_ns=timed.sample_steady_ns,
        deadline_ns=deadline,
    )


def read_coverage(path):
    from senior_mapping.shared_scan import (
        MAX_CONTRACT_BYTES,
        CurrentLayerCoverage,
        _contract_decode,
        _unique_contract_pairs,
    )

    with path.open("rb") as stream:
        raw = stream.read(MAX_CONTRACT_BYTES + 1)
    if len(raw) > MAX_CONTRACT_BYTES:
        raise ValueError("coverage observation exceeds contract bound")
    row = json.loads(raw, object_pairs_hook=_unique_contract_pairs)
    if type(row.get("schema")) is not int or row["schema"] != 1:
        raise ValueError("unsupported coverage contract schema")
    if row["status"] == "ready":
        _contract_decode(row, CurrentLayerCoverage)
    # Native pre-authorization invalid records legitimately have no owner yet.
    # Observe their whole payload, but never turn them into ready evidence.
    return row


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_loader(name, SourceFileLoader(name, str(path)))
    if spec is None or spec.loader is None:
        raise ImportError(path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def admitted_profile():
    # Reuse the leaf fixture only; no aggregate test_agent/rig imports.
    from test_scan_geometry import synthetic_profile

    return replace(synthetic_profile(), profile_id="synthetic-admitted")


class NativeFixture:
    def __init__(self, root: Path):
        self.root = root.resolve()
        self.socket = self.root / "adapter.sock"
        self.host = Path("/proc/sys/kernel/random/boot_id").read_text().strip()
        self.index = CaptureIndex()
        self.maps = deque(maxlen=4)
        self.current = deque(maxlen=512)
        self.pending_tf = OrderedDict()
        self.startup = None
        self.managed = None
        self.launch_task = self.pump_task = None
        self.events = deque(maxlen=512)
        self.coverage_rows = {}
        self.coverage_versions = {}
        self.observed_session = None
        self.latest_slot = None
        self.control_subscriptions = {}
        self.shutdown_done = False
        self.localization_task = None
        self.localization_initialized = False
        self.localization_scan_stamp = -1
        self.localization_pose_count = 0

    async def __aenter__(self):
        try:
            return await self._enter()
        except BaseException:
            await self.__aexit__(*sys.exc_info())
            raise

    async def _enter(self):
        import rclpy
        from ament_index_python.packages import get_package_prefix, get_package_share_directory
        from launch import LaunchDescription, LaunchService
        from launch.actions import OpaqueFunction, RegisterEventHandler, SetLaunchConfiguration
        from launch.event_handlers import OnProcessExit, OnProcessStart
        from launch.substitutions import Command
        from launch_ros.actions import Node
        from launch_ros.parameter_descriptions import ParameterValue
        from nav_msgs.msg import OccupancyGrid
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from senior_interfaces.msg import CurrentObstacles, QualifiedMappingScan
        from shared_layers_sensor_source import SensorSource
        from tf2_ros import Buffer, TransformListener

        assert os.environ.get("ROS_LOCALHOST_ONLY") == "1", "explicit localhost isolation required"
        assert "ROS_DOMAIN_ID" in os.environ, "VM must reserve an unused domain"
        self.root.mkdir(mode=0o700, parents=True, exist_ok=True)
        sim = Path(get_package_share_directory("senior_sim"))
        module = load_module(
            "shared_layers_original_motion_launch", sim / "launch/simulation_motion.launch.py"
        )
        reference_owner = load_module(
            "shared_layers_original_world_reference",
            Path(get_package_prefix("senior_sim")) / "lib/senior_sim/world_reference",
        )
        world, urdf = sim / "worlds/motion_qualification.sdf", sim / "urdf/senior.urdf.xacro"
        assets = self.root / "reference"
        reference = reference_owner.generate_reference(world, assets)
        source_root = Path(__file__).resolve().parents[2]
        version = subprocess.check_output(
            ["git", "rev-parse", "HEAD", "HEAD^{tree}"],
            cwd=source_root,
            text=True,
        ).splitlines()
        digests = {
            str(path): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in (
                world,
                urdf,
                reference,
                assets / "grid.json",
                assets / "map.yaml",
                sim / "config/motion_profile.json",
                sim / "config/controllers.yaml",
                Path(get_package_share_directory("senior_navigation")) / "config/nav2.yaml",
                sim / "launch/simulation_motion.launch.py",
                source_root / "uv.lock",
            )
        }
        self.input_metadata = dict(
            source_commit=version[0],
            source_tree=version[1],
            asset_sha256=digests,
            host_boot_id=self.host,
            world=str(world),
            urdf=str(urdf),
        )
        self.trace = TraceWriter(
            self.root / "system-trace.jsonl",
            dict(
                run_id=self.root.name,
                host_boot_id=self.host,
                source_commit=version[0],
                source_tree=version[1],
                asset_sha256=digests,
                recorder_pid=os.getpid(),
                observation_capacity=512,
                source_root=str(Path(__file__).resolve().parents[2]),
                world=str(world),
                urdf=str(urdf),
                geometry_profile=asdict(admitted_profile()),
                motion_profile=str(sim / "config/motion_profile.json"),
                clock="same-host monotonic; ROS wall stamp",
                queue_depth="N/A",
                lock_wait="N/A",
                input="functional raw A->B->invalid; frozen input is a separate comparison mode",
            ),
        )
        self.trace.__enter__()
        self.resources = ResourceSampler(self.trace)
        self.resources.register(os.getpid())
        self.resources.start()
        rclpy.init(args=[])
        self.node = rclpy.create_node("shared_layers_process_observer")
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self.node)
        self.node.create_subscription(
            CurrentObstacles, "/senior/mapping/current_obstacles", self._q, 32
        )
        self.node.create_subscription(
            QualifiedMappingScan, "/senior/slam/qualified_scan", self._qualified, 32
        )
        self.node.create_subscription(
            OccupancyGrid,
            "/map",
            self.maps.append,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self.source = SensorSource(world, urdf, self.trace)
        for node in self.source.nodes:
            self.executor.add_node(node)
        self.launch = LaunchService(argv=[], noninteractive=True)

        def started(event, context):
            self.resources.register(event.pid)
            self.events.append(("start", event.action, event.pid, time.monotonic_ns()))
            self.trace.emit("launch_start", pid=event.pid, action=str(event.action))
            return []

        def exited(event, context):
            self.events.append(("exit", event.action, event.returncode, time.monotonic_ns()))
            self.trace.emit("launch_exit", returncode=event.returncode, action=str(event.action))
            return []

        def compose(context):
            stack = module.motion_stack_actions(
                context,
                before_navigation=lambda context, common: [],
                after_navigation=lambda context, common: [],
            )
            self.startup = stack.startup
            self.parameters = self.startup.parameters
            self.run_directory = self.startup.directory
            self.static_overlay = self.parameters.read_text()
            self.resources.files = [
                self.run_directory / name
                for name in ("reference.json", "global-coverage.json", "local-coverage.json")
            ]
            return stack.actions

        values = dict(
            agent_socket=str(self.socket),
            robot_id="shared-process-test",
            use_sim_time="false",
            navigation_enabled="true",
            navigation_reference=str(reference),
            map_yaml=str(assets / "map.yaml"),
            hardware_motion_profile=str(sim / "config/motion_profile.json"),
        )
        self.launch.include_launch_description(
            LaunchDescription(
                [
                    *[SetLaunchConfiguration(key, value) for key, value in values.items()],
                    RegisterEventHandler(OnProcessStart(on_start=started)),
                    RegisterEventHandler(OnProcessExit(on_exit=exited)),
                    Node(
                        package="robot_state_publisher",
                        executable="robot_state_publisher",
                        parameters=[
                            dict(
                                use_sim_time=False,
                                robot_description=ParameterValue(
                                    Command(
                                        [
                                            "xacro ",
                                            str(urdf),
                                            " rear_attachment:=false controllers_file:=",
                                            str(sim / "config/controllers.yaml"),
                                        ]
                                    ),
                                    value_type=str,
                                ),
                            )
                        ],
                    ),
                    OpaqueFunction(function=compose),
                ]
            )
        )
        self.launch_task = asyncio.create_task(self.launch.run_async())
        self.pump_task = asyncio.create_task(self._pump())
        await self.wait(lambda: self.socket.is_socket() and self.startup is not None)
        self.trace.emit(
            "startup_parameters", path=str(self.parameters), contents=self.static_overlay
        )
        return self

    async def wait(self, predicate, *, timeout=30):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.pump_task and self.pump_task.done():
                await self.pump_task
            if self.launch_task and self.launch_task.done() and not self.shutdown_done:
                raise AssertionError(f"real Launch exited early: {self.launch_task.result()}")
            if (
                self.localization_task
                and self.localization_task is not asyncio.current_task()
                and self.localization_task.done()
            ):
                self.localization_task.result()  # Service failures remain failures.
            result = predicate()
            if asyncio.iscoroutine(result):
                result = await result
            if result:
                return result
            await asyncio.sleep(0.02)
        raise AssertionError("real production observation timed out; preserve run/trace")

    def _q(self, message):
        self.current.append(message)
        if not message.valid:
            self.trace.emit(
                "q_invalid", reason=message.reason, deadline_ns=message.valid_until_steady_ns
            )
            return
        row = capture_record(
            message.backend_boot,
            message.host_boot_id,
            message.capture,
            deadline=message.valid_until_steady_ns,
        )
        row.update(
            map_id=message.map_id,
            reference_revision=message.reference_revision,
            map_generation=message.map_generation,
        )
        endpoints = self.node.get_publishers_info_by_topic("/senior/mapping/current_obstacles")
        row["observed_endpoint_gids"] = [
            bytes(endpoint.endpoint_gid).hex() for endpoint in endpoints
        ]
        # Humble rclpy does not expose per-message GID: actual admission is proven
        # by the native plugin coverage, not by this graph-only observer field.
        self.index.add("q", row)
        self.trace.emit("q", **row)
        self.pending_tf[identity(row)] = row
        if len(self.pending_tf) > 512:
            self.pending_tf.popitem(last=False)

    def _qualified(self, message):
        if message.status != message.QUALIFIED:
            self.trace.emit("qualified_invalid", reason=message.reason)
            return
        capture = message.scan.capture
        row = capture_record(
            message.backend_boot,
            self.host,
            capture,
            deadline=capture.sample_steady_ns + 500_000_000,
        )
        self.index.add("qualified", row)
        self.trace.emit("qualified", **row)

    async def _pump(self):
        from rclpy.time import Time
        from tf2_ros import TransformException

        next_observation = next_topics = 0
        observation = None
        try:
            while True:
                if observation is not None and observation.done():
                    observation.result()  # The original assertion/error must reach wait().
                    observation = None
                self.executor.spin_once(timeout_sec=0)
                for key, row in tuple(self.pending_tf.items()):
                    try:
                        transform = self.tf.lookup_transform(
                            "map", "odom", Time(nanoseconds=row["stamp_ns"])
                        )
                    except TransformException:
                        continue
                    stamp = (
                        transform.header.stamp.sec * 1_000_000_000 + transform.header.stamp.nanosec
                    )
                    assert stamp == row["stamp_ns"], "TF was not exact original scan stamp"
                    self.index.add("tf", row)
                    self.trace.emit("tf", **row)
                    del self.pending_tf[key]
                now = time.monotonic_ns()
                if now >= next_topics:
                    self._observe_controller_topics()
                    next_topics = now + 1_000_000_000
                if (
                    observation is None
                    and self.observed_session is not None
                    and now >= next_observation
                ):
                    observation = asyncio.create_task(self.observe(self.observed_session))
                    next_observation = now + 100_000_000
                await asyncio.sleep(0.005)
        finally:
            if observation is not None:
                # Join immutable IO reads before closing trace, files, or ROS nodes.
                await observation

    def _observe_controller_topics(self):
        from geometry_msgs.msg import Twist

        for topic, types in self.node.get_topic_names_and_types():
            if (
                re.fullmatch(r"/control/navigation/run_[0-9a-f]{32}/cmd_vel", topic)
                and "geometry_msgs/msg/Twist" in types
                and topic not in self.control_subscriptions
            ):
                assert len(self.control_subscriptions) < 16, "controller observation bound exceeded"
                self.control_subscriptions[topic] = self.node.create_subscription(
                    Twist,
                    topic,
                    lambda msg, topic=topic: self.trace.emit(
                        "controller_output",
                        topic=topic,
                        vx=msg.linear.x,
                        wz=msg.angular.z,
                        association="observed authorization window only; no capture cause",
                    ),
                    10,
                )

    def _slot_record(self, session):
        from senior_mapping.shared_scan import read_latest_scan

        result = read_latest_scan(
            session._scan_path,
            0,
            nonce=session._scan_nonce,
            backend_boot=session.graph.boot,
            host_boot_id=session._host_boot_id,
        )
        envelope = result.envelope
        if envelope and envelope.scan:
            row = capture_record(
                envelope.key.backend_boot,
                envelope.scan.host_boot_id,
                envelope.scan.capture,
                deadline=envelope.valid_until_steady_ns,
            )
            return row
        return None

    def observe_slot(self, session):
        row = self._slot_record(session)
        if row:
            self.index.add("slot", row)
            self.trace.emit("slot", **row)
        return row

    async def observe(self, session):
        """One bounded read pass; only this asyncio owner touches the capture index.

        The IO workers return immutable records, never callbacks into the index.
        The pump keeps observing during save/restore IPC awaits, beyond ready().
        """
        row = await asyncio.to_thread(self._slot_record, session)
        self.latest_slot = row
        if row:
            self.index.add("slot", row)
            self.trace.emit("slot", **row)
        for role in ("global", "local"):
            path = self.run_directory / (role + "-coverage.json")
            try:
                coverage = await asyncio.to_thread(read_coverage, path)
            except FileNotFoundError:
                self.coverage_rows.pop(role, None)
                continue
            assert coverage["role"] == role
            if self.coverage_versions.get(role) != coverage:
                self.trace.emit(role, record=coverage)
                self.coverage_versions[role] = coverage
            if coverage.get("status") != "ready" or not coverage.get("key"):
                self.coverage_rows.pop(role, None)
                continue
            original = self.index.coverage(coverage)
            if original is not None and time.monotonic_ns() < coverage["valid_until_steady_ns"]:
                self.coverage_rows[role] = coverage
            else:
                self.coverage_rows.pop(role, None)

    async def ready(self, client, session):
        self.observed_session = session
        row = self.latest_slot
        if (
            not row
            or row["key"]["backend_boot"] != session.graph.boot
            or not self.index.pair(row, "q", "qualified", "tf", "slot")
        ):
            return None
        for role in ("global", "local"):
            coverage = self.coverage_rows.get(role)
            if (
                not coverage
                or coverage["key"]["backend_boot"] != session.graph.boot
                or time.monotonic_ns() >= coverage["valid_until_steady_ns"]
            ):
                return None
        state = await client.request("state.get")
        self.trace.emit("state", record=state)
        if state.get("ok") and state.get("navigation_ready"):
            assert self.node.count_publishers("/map") == 1
            assert self.node.count_publishers("/senior/mapping/current_obstacles") == 1
            return state
        return None

    async def route_status(self, client, plan):
        reply = await client.request(
            "navigation.task_status",
            execution_id=plan["execution_id"],
            task_epoch=plan["task_epoch"],
        )
        progress = reply.get("progress")
        status = progress.get("status") if progress else "missing"
        self.trace.emit(
            "route_state",
            record=progress,
            status=status,
            reply=reply,
            execution_id=plan["execution_id"],
            task_epoch=plan["task_epoch"],
        )
        assert reply.get("ok") and reply.get("execution_id") == plan["execution_id"], reply
        assert status in {
            "starting",
            "running",
            "cancel_pending",
            "failed",
            "succeeded",
            "canceled",
            "missing",
        }, reply
        if progress:
            assert progress["execution_id"] == plan["execution_id"], reply
            assert progress["task_epoch"] == plan["task_epoch"], reply
        return progress or dict(status="missing")

    async def cancel_route(self, client, plan):
        fields = dict(
            execution_id=plan["execution_id"],
            task_epoch=plan["task_epoch"],
            effect_id="reference-cancel",
            effect_sequence=2,
        )

        async def request(kind):
            reply = await client.request(
                kind, **fields, **({"effect_kind": "revoke"} if kind.endswith("status") else {})
            )
            self.trace.emit("route_cancel", request_kind=kind, request=fields, reply=reply)
            assert reply.get("ok") and reply.get("lifecycle_version") == 2, reply
            assert all(
                reply.get(k) == fields[k] for k in ("execution_id", "task_epoch", "effect_id")
            ), reply
            return reply

        await request("navigation.task_cancel")  # ok only means the request was handled.

        async def confirmed():
            reply = await request("navigation.task_status")
            progress = await self.route_status(client, plan)
            epoch = plan["task_epoch"]
            return (
                reply.get("accepted") is True
                and reply.get("controller_output_exited") is True
                and (reply.get("sources_fenced_through_epoch") or 0) >= epoch
                and progress.get("controller_output_exited") is True
                and (progress.get("sources_fenced_through_epoch") or 0) >= epoch
                and progress["status"] in {"failed", "succeeded", "canceled"}
            )

        await self.wait(confirmed)

    def _observe_localization_scan(self, message):
        stamp = message.header.stamp
        self.localization_scan_stamp = max(
            self.localization_scan_stamp, stamp.sec * 1_000_000_000 + stamp.nanosec
        )

    def _observe_localization_pose(self, message):
        # Read-only evidence: no reference callback, receipt, timestamp or ready mutation.
        self.localization_pose_count += 1
        stamp = message.header.stamp
        self.trace.emit(
            "static_amcl_pose",
            count=self.localization_pose_count,
            stamp_ns=stamp.sec * 1_000_000_000 + stamp.nanosec,
            frame_id=message.header.frame_id,
            covariance=list(message.pose.covariance),
            identity="observer only; native MessageInfo admission remains authoritative",
        )

    def _localization_static(self):
        return not self.shutdown_done and self.startup.phase == "static"

    async def _localization_call(self, service, request, name, **fields):
        future = None
        try:
            await self.wait(lambda: not self._localization_static() or service.service_is_ready())
            if not self._localization_static():
                raise asyncio.CancelledError()
            future = service.call_async(request)
            self.trace.emit("static_amcl_service", service=name, phase="request", **fields)
            await self.wait(lambda: not self._localization_static() or future.done())
            if not self._localization_static():
                raise asyncio.CancelledError()
            response = future.result()
            self.trace.emit("static_amcl_service", service=name, phase="response", **fields)
            return response  # Empty service acknowledgement is never localization evidence.
        except BaseException as exc:
            self.trace.emit(
                "static_amcl_service", service=name, phase="error", error_type=type(exc).__name__
            )
            raise
        finally:
            if future is not None and not future.done():
                future.cancel()
                service.remove_pending_request(future)
                self.trace.emit("static_amcl_service", service=name, phase="cancel_settled")

    async def _initialize_localization(self, service, request_type):
        await self.wait(lambda: not self._localization_static() or service.service_is_ready())
        if not self._localization_static():
            raise asyncio.CancelledError()
        request = request_type.Request()
        request.pose.header.frame_id = "map"
        request.pose.header.stamp = self.node.get_clock().now().to_msg()
        request.pose.pose.pose.position.x = request.pose.pose.pose.position.y = 0.0
        request.pose.pose.pose.orientation.w = 1.0  # Known isolated-world spawn: yaw=0.
        covariance = [0.0] * 36
        covariance[0] = covariance[7] = 0.5**2
        covariance[35] = (math.pi / 12) ** 2
        request.pose.pose.covariance = covariance
        stamp = request.pose.header.stamp
        await self._localization_call(
            service,
            request,
            "/set_initial_pose",
            frame_id="map",
            pose=[0.0, 0.0, 0.0],
            stamp_ns=stamp.sec * 1_000_000_000 + stamp.nanosec,
            covariance=covariance,
            source="navigation2 a097086719c88f781aa59788eca29ac6ca5e56db "
            "nav2_amcl/src/amcl_node.cpp:initOdometry:1571-1578",
            source_sha256="29a6402bf643fc8ea0d2dcea29619d4cbdcaf7014c811332ed4e86e593d3f1af",
            lifecycle="map_server and amcl GetState ACTIVE observed",
        )
        self.localization_initialized = True  # Input delivered, not posterior/qualification.

    async def _static_localization(self):
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from nav2_msgs.srv import SetInitialPose
        from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
        from sensor_msgs.msg import LaserScan
        from std_srvs.srv import Empty

        services, subscriptions = [], []
        try:
            for kind, name in (
                (SetInitialPose, "/set_initial_pose"),
                (Empty, "/request_nomotion_update"),
            ):
                services.append(self.node.create_client(kind, name))
            subscriptions.append(
                self.node.create_subscription(
                    LaserScan, "/scan", self._observe_localization_scan, qos_profile_sensor_data
                )
            )
            subscriptions.append(
                self.node.create_subscription(
                    PoseWithCovarianceStamped,
                    "/amcl_pose",
                    self._observe_localization_pose,
                    QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
                )
            )
            await self._initialize_localization(services[0], SetInitialPose)
            consumed, next_request = self.localization_scan_stamp, 0.0
            while self._localization_static():
                # One request per newly observed scan, no faster than the original 5Hz.
                # Acknowledgement only sets AMCL force_update; a subsequent real scan
                # must still produce the posterior through its original scan/TF path.
                if self.localization_scan_stamp > consumed and time.monotonic() >= next_request:
                    consumed = self.localization_scan_stamp
                    await self._localization_call(
                        services[1],
                        Empty.Request(),
                        "/request_nomotion_update",
                        scan_stamp_ns=consumed,
                        observed_pose_count=self.localization_pose_count,
                    )
                    # Bound actual calls even when discovery or a response was slow.
                    next_request = time.monotonic() + 0.2
                await asyncio.sleep(0.02)
        finally:
            for service in services:
                self.node.destroy_client(service)
            for subscription in subscriptions:
                self.node.destroy_subscription(subscription)
            self.trace.emit("static_amcl_settled", pose_count=self.localization_pose_count)

    async def _stop_static_localization(self):
        task = self.localization_task
        if task is not None:
            if not task.done():
                task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass
            finally:
                self.localization_task = None

    async def static_ready(self, client):
        from lifecycle_msgs.msg import State
        from lifecycle_msgs.srv import GetState

        from vehicle_core.ipc import IpcError

        def observe(stage, *, exception=None, **fields):
            try:
                if exception is not None:
                    fields.update(error_type=type(exception).__name__, error=str(exception)[:256])
                self.trace.emit(stage, **fields)
            except Exception as exc:
                # Preserve the service result/exception, but keep trace settlement
                # failed. Never recursively emit an error through a broken sink.
                if getattr(self.trace, "error", None) is None:
                    self.trace.error = exc

        observing = True

        async def snapshot():
            # Independent read while the original service wait runs. Never gates readiness.
            while observing:
                try:
                    state = await asyncio.wait_for(client.request("state.get"), timeout=1)
                    observe("static_diagnostic_state", record=state)
                except (asyncio.TimeoutError, OSError, IpcError) as exc:
                    observe(
                        "static_diagnostic_state",
                        exception=exc,
                    )
                # wait_for may return a just-completed result during cancellation.
                if observing:
                    await asyncio.sleep(2)

        observation = asyncio.create_task(snapshot())
        try:
            names = ("map_server", "amcl", "planner_server", "controller_server", "bt_navigator")
            for name in names:
                service_name = f"/{name}/get_state"
                phase = "begin"
                service = self.node.create_client(GetState, service_name)
                try:
                    observe("static_service", service=service_name, phase=phase)
                    await self.wait(service.service_is_ready)
                    phase = "ready"
                    observe("static_service", service=service_name, phase=phase)
                    future = service.call_async(GetState.Request())
                    phase = "request"
                    observe("static_service", service=service_name, phase=phase)
                    await self.wait(future.done)
                    state_id = future.result().current_state.id
                    observe(
                        "static_service", service=service_name, phase="response", state_id=state_id
                    )
                    if state_id != State.PRIMARY_STATE_ACTIVE:
                        return None
                    if name == "amcl" and not self.localization_initialized:
                        assert self._localization_static(), "static AMCL initialization required"
                        if self.localization_task is None:
                            self.localization_task = asyncio.create_task(
                                self._static_localization()
                            )
                        await self.wait(lambda: self.localization_initialized)
                except BaseException as exc:
                    observe(
                        "static_service",
                        service=service_name,
                        phase="wait_error",
                        waiting_phase=phase,
                        exception=exc,
                    )
                    raise
                finally:
                    self.node.destroy_client(service)
            state = await client.request("state.get")
            self.trace.emit("static_state", record=state)
            if state.get("navigation_ready"):
                # Settle before returning control to the caller that can prepare live.
                await self._stop_static_localization()
                return state
            return None
        except BaseException:
            await self._stop_static_localization()
            raise
        finally:
            observing = False
            observation.cancel()
            await asyncio.gather(observation, return_exceptions=True)

    async def native_watermark(self):
        from senior_interfaces.srv import ReadGraph

        service = self.node.create_client(ReadGraph, "/senior/slam/read_graph")
        try:
            await self.wait(service.service_is_ready)
            future = service.call_async(ReadGraph.Request())
            await self.wait(future.done)
            graph = future.result()
            assert graph.graph_commit_watermark_valid
            result = (graph.boot_id, graph.graph_commit_sequence, graph.last_committed_keyframe_id)
            self.trace.emit(
                "native_watermark",
                backend_boot=result[0],
                graph_sequence=result[1],
                last_keyframe_id=result[2],
            )
            return result
        finally:
            self.node.destroy_client(service)

    async def consumer_parameters(self):
        from rcl_interfaces.srv import GetParameters

        names = [
            "FollowPath.batch_size",
            "FollowPath.vx_max",
            "FollowPath.vx_min",
            "FollowPath.wz_max",
        ]
        client = self.node.create_client(GetParameters, "/controller_server/get_parameters")
        try:
            await self.wait(client.service_is_ready)
            request = GetParameters.Request()
            request.names = names
            future = client.call_async(request)
            await self.wait(future.done)
            values = future.result().values
            actual = [values[0].integer_value, *(value.double_value for value in values[1:])]
            assert actual == [400, 0.22, -0.15, 0.4], actual
            self.trace.emit(
                "consumer_parameters",
                role="local",
                parameters=dict(zip(names, actual, strict=True)),
            )
        finally:
            self.node.destroy_client(client)

    def new_raw_source(self):
        from shared_layers_sensor_source import SensorSource

        old = self.source
        for node in old.nodes:
            self.executor.remove_node(node)
        old.close()
        self.source = SensorSource(
            Path(self.trace.metadata["world"]), Path(self.trace.metadata["urdf"]), self.trace
        )
        self.source.offset = 0.4
        for node in self.source.nodes:
            self.executor.add_node(node)
        self.trace.emit("source_restart")

    async def shutdown(self):
        if self.shutdown_done:
            return
        self.shutdown_done = True
        self.observed_session = None
        localization_failure = None
        try:
            await self._stop_static_localization()
        except BaseException as exc:
            localization_failure = exc  # Still settle Launch; never turn service failure green.
        await asyncio.to_thread(self.resources._discover)
        obligations = self.resources.snapshot()
        self.trace.emit("shutdown_begin", owned=list(obligations))
        shutdown = self.launch.shutdown()
        if asyncio.iscoroutine(shutdown):
            await shutdown
        deadline = time.monotonic() + 45
        while not self.launch_task.done():
            assert time.monotonic() < deadline, "Launch shutdown unconfirmed; preserve evidence"
            # Relevant actual descendants must settle before run deletion.
            pending = []
            for row in obligations:
                if not alive(row):
                    continue
                try:
                    command = Path(f"/proc/{row['pid']}/cmdline").read_bytes()
                except FileNotFoundError:
                    continue
                if any(
                    name in command
                    for name in (
                        b"controller_server",
                        b"bt_navigator",
                        b"slam_graph_node",
                        b"agent_adapter",
                    )
                ):
                    pending.append(row)
            if pending:
                assert self.run_directory.exists(), (
                    "startup run removed before native/controller exit"
                )
            await asyncio.sleep(0.02)
        result = await self.launch_task
        assert result == 0, f"Launch cleanup failed: {result}"
        await self.wait(
            lambda: (
                not any(
                    alive(row) for row in self.resources.snapshot() if row["pid"] != os.getpid()
                )
            ),
            timeout=15,
        )
        assert not self.startup.groups and not self.startup.consumers
        assert not self.startup.cleanup_failed
        await self.wait(lambda: not self.run_directory.exists(), timeout=15)
        self.trace.emit("shutdown_settled")
        if localization_failure is not None:
            raise localization_failure

    async def __aexit__(self, exc_type, exc, traceback):
        import rclpy

        failures = []
        if getattr(self, "trace", None) and exc is not None:
            try:
                self.trace.emit("test_failure", reason=str(exc))
            except BaseException as error:
                failures.append(error)
        try:
            if self.launch_task is not None:
                await asyncio.wait_for(self.shutdown(), timeout=45)
        except BaseException as error:
            failures.append(error)
        finally:
            if self.pump_task:
                self.pump_task.cancel()
                results = await asyncio.gather(self.pump_task, return_exceptions=True)
                error = results[0]
                if isinstance(error, BaseException) and not isinstance(
                    error, asyncio.CancelledError
                ):
                    # A prior body exception remains primary; retain the observer failure too.
                    if getattr(self, "trace", None):
                        self.trace.emit("observation_failure", reason=repr(error))
                    if exc is None and not any(error is item for item in failures):
                        failures.append(error)
                    elif exc is not None:
                        failures = [item for item in failures if item is not error]
            cleanup = []
            if getattr(self, "resources", None):
                cleanup.append(self.resources.close)
            if getattr(self, "source", None):
                cleanup.append(self.source.close)
            if getattr(self, "node", None):
                cleanup.append(self.node.destroy_node)
            if getattr(self, "executor", None):
                cleanup.append(self.executor.shutdown)
            if rclpy.ok():
                cleanup.append(rclpy.shutdown)
            for close in cleanup:
                try:
                    close()
                except BaseException as error:
                    failures.append(error)
            if getattr(self, "trace", None):
                try:
                    self.trace.emit(
                        "capture_index", evicted=self.index.evicted, pending_tf=len(self.pending_tf)
                    )
                    for error in failures:
                        self.trace.emit("cleanup_unconfirmed", reason=str(error))
                finally:
                    self.trace.__exit__()
        if failures:
            raise RuntimeError(
                "real fixture cleanup unconfirmed; preserve run/trace"
            ) from failures[0]
