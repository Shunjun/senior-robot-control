"""Opt-in, isolated-domain recovery checks against installed ROS processes.

No test here connects to a vehicle or launches the hardware motion composition.
A local skip is not an installed Humble pass.
"""

import copy
import os
import re
import shlex
import signal
import subprocess
import sys
import time
from collections import deque
from contextlib import contextmanager
from pathlib import Path
from types import SimpleNamespace

import pytest


@pytest.fixture
def isolated_ros():
    rclpy = pytest.importorskip("rclpy", reason="ROS 2 is unavailable on this host")
    if os.environ.get("SENIOR_LAUNCH_RECOVERY_LIVE") != "1":
        pytest.skip("explicit isolated ROS recovery run not selected")
    domain = os.environ.get("ROS_DOMAIN_ID", "")
    assert domain.isdecimal() and 0 <= int(domain) <= 101, "choose an isolated ROS domain"
    assert os.environ.get("ROS_LOCALHOST_ONLY") == "1", "localhost-only ROS required"
    if sys.platform != "linux":
        pytest.skip("installed recovery checks require Linux process semantics")
    rclpy.init()
    try:
        probe = rclpy.create_node(f"launch_recovery_domain_check_{os.getpid()}")
        try:
            deadline = time.monotonic() + 2
            while time.monotonic() < deadline:
                rclpy.spin_once(probe, timeout_sec=0.1)
            assert set(probe.get_node_names_and_namespaces()) == {(probe.get_name(), "/")}, (
                "recovery tests require an otherwise empty ROS domain"
            )
        finally:
            probe.destroy_node()
        yield rclpy
    finally:
        if rclpy.ok():
            rclpy.shutdown()


@contextmanager
def prefixed_process(command, logfile):
    from senior_control.runtime.launch_recovery import restart_prefix

    with logfile.open("w") as log:
        process = subprocess.Popen(
            [*shlex.split(restart_prefix()), *command],
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        try:
            yield process
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
            assert process.wait(timeout=8) == 0, logfile.read_text()


def latest_node_pid(logfile):
    matches = re.findall(r"started pid=(\d+) attempt=\d+", logfile.read_text())
    assert matches, logfile.read_text()
    return int(matches[-1])


def test_scan_receipt_crash_recovers_new_identity_and_rejects_old_capture(isolated_ros, tmp_path):
    from ament_index_python.packages import get_package_prefix
    from geometry_msgs.msg import TransformStamped
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from senior_motion_msgs.msg import BoundedRawLaserScan, FencedFilteredScan
    from senior_robot.scan_adapter import ScanAdapter
    from sensor_msgs.msg import LaserScan
    from test_scan_epoch_live import _room_scan
    from tf2_ros import StaticTransformBroadcaster

    # Reuse the installed native_scan_process path and the existing real room
    # scan fixture. Keep the actual Python adapter alive across receipt death.
    executable = (
        Path(get_package_prefix("senior_sensor_ingress"))
        / "lib/senior_sensor_ingress/scan_receipt_node"
    )
    logfile = tmp_path / "receipt-recovery.log"
    probe = isolated_ros.create_node("launch_recovery_scan_probe")
    adapter = ScanAdapter()
    executor = SingleThreadedExecutor()
    executor.add_node(probe)
    executor.add_node(adapter)
    publisher = probe.create_publisher(LaserScan, "/scan", qos_profile_sensor_data)
    raw, filtered = deque(maxlen=128), deque(maxlen=128)
    probe.create_subscription(
        BoundedRawLaserScan, "/safety/raw/scan", raw.append, qos_profile_sensor_data
    )
    probe.create_subscription(
        FencedFilteredScan, "/perception/scan_filtered", filtered.append, qos_profile_sensor_data
    )
    transform = TransformStamped()
    transform.header.frame_id, transform.child_frame_id = "base_link", "laser_link"
    transform.transform.rotation.w = 1.0
    adapter.buffer.set_transform_static(transform, "fixture")
    broadcaster = StaticTransformBroadcaster(probe)
    broadcaster.sendTransform(transform)
    scan = _room_scan()

    def until(predicate, wrapper):
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline and not predicate():
            assert wrapper.poll() is None, logfile.read_text()
            scan.header.stamp = probe.get_clock().now().to_msg()
            publisher.publish(scan)
            for _ in range(5):
                executor.spin_once(timeout_sec=0.01)
        assert predicate(), logfile.read_text()

    try:
        with prefixed_process([str(executable)], logfile) as wrapper:
            until(lambda: any(value.owner.valid for value in filtered), wrapper)
            first = copy.deepcopy(next(value for value in reversed(raw) if value.capture.valid))
            old_pid = latest_node_pid(logfile)
            os.kill(old_pid, signal.SIGKILL)
            filtered.clear()
            until(
                lambda: any(
                    value.owner.valid
                    and value.body.raw_capture.bridge_gid != first.capture.bridge_gid
                    for value in filtered
                ),
                wrapper,
            )
            second = next(value for value in reversed(raw) if value.capture.valid)
            assert latest_node_pid(logfile) != old_pid
            # A lingering orphan zombie is a failure, never proof of recovery.
            assert not Path(f"/proc/{old_pid}").exists(), logfile.read_text()
            assert second.capture.source_epoch != first.capture.source_epoch
            assert second.capture.bridge_boot_id != first.capture.bridge_boot_id
            accepted = adapter._admission.latest
            lineage = adapter._lineage
            generation = adapter._admission.generation
            adapter.on_scan(first)
            assert adapter._admission.latest == accepted
            assert adapter._lineage == lineage
            assert adapter._admission.generation == generation
    finally:
        executor.shutdown()
        adapter.destroy_node()
        probe.destroy_node()


def test_installed_command_guard_core_recovery_keeps_estop_and_retires_old_manual(
    isolated_ros, tmp_path, monkeypatch
):
    """Actual guard/receipt/output processes; isolated domain, no chassis or scan source."""
    import json

    from ament_index_python.packages import get_package_prefix
    from geometry_msgs.msg import Twist
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from senior_motion_msgs.msg import FinalCommand
    from senior_robot.command_node import CommandNode
    from senior_robot.motion_lifecycle import MotionExecutor
    from std_msgs.msg import String

    from senior_control.protocol import SimpleEnvelope
    from senior_control.settings.models import simulation_settings_profile

    monkeypatch.delenv("SENIOR_STARTUP_LEARNING_CLAIM", raising=False)
    # Learning run preservation is checked by the owner/client production-chain
    # test. This live probe starts no learning service and has no physical inputs.
    executable = Path(get_package_prefix("senior_robot")) / "lib/senior_robot/safety_node"
    profile = Path(__file__).parents[1] / "fixtures/generic_motion_profile.json"
    command = CommandNode(parameter_overrides=[Parameter("robot_id", value="launch-recovery")])
    probe = isolated_ros.create_node("launch_recovery_output_probe")
    observer = SingleThreadedExecutor()
    observer.add_node(probe)
    commands = MotionExecutor()
    commands.add_node(command)
    runtimes, velocities, final_frames = (deque(maxlen=256) for _ in range(3))
    probe.create_subscription(
        String,
        "/motion/runtime",
        lambda msg: runtimes.append(json.loads(msg.data)),
        qos_profile_sensor_data,
    )
    output_topic = "/launch_recovery/cmd_vel"
    probe.create_subscription(Twist, output_topic, velocities.append, 10)
    probe.create_subscription(
        FinalCommand, "/safety/final_command", final_frames.append, qos_profile_sensor_data
    )
    logfile = tmp_path / "critical-recovery.log"
    sequence = 0

    def envelope(kind, payload=None):
        nonlocal sequence
        sequence += 1
        value = {
            "v": 1,
            "robot_id": "launch-recovery",
            "session_id": "operator",
            "connection_epoch": 1,
            "request_id": f"live-{sequence}",
            "seq": sequence,
            "expires_at": time.time() + 30,
            "type": kind,
            "payload": payload or {},
        }
        if not kind.startswith("safety."):
            value["controller_generation"] = 1
        return value

    def manual(kind, payload=None):
        now = time.monotonic()
        return command.manual_request(
            {
                "envelope": json.dumps(envelope(kind, payload)),
                "driving": simulation_settings_profile().driving_defaults.model_dump(),
                "received_steady": now,
                "deadline_steady": now + 0.3,
            }
        )

    def until(predicate, wrapper, *, spin_command=True):
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline and not predicate():
            assert wrapper.poll() is None, logfile.read_text()
            observer.spin_once(timeout_sec=0.01)
            if spin_command:
                commands.spin_once(timeout_sec=0.01)
        assert predicate(), logfile.read_text()

    try:
        with prefixed_process(
            [
                str(executable),
                "--ros-args",
                "-p",
                "robot_id:=launch-recovery",
                "-p",
                f"hardware_motion_profile:={profile}",
                "-p",
                "depth_policy:=laser_only",
                "-p",
                f"output_topic:={output_topic}",
            ],
            logfile,
        ) as wrapper:
            until(
                lambda: (
                    runtimes
                    and command._output_incarnation is not None
                    and command._output_handoff_boot is None
                ),
                wrapper,
            )
            authority = manual("control.acquire")["authority_id"]
            old_prepared = copy.deepcopy(command._barrier_frame)
            # Prepared ownership has no movement TTL. An active stream left
            # silent through the two-second backoff would correctly expire at
            # 300 ms and could not generate the fresh old-source input below.
            assert command._manual_ingress.view(time.monotonic()).state.value == "prepared"
            old_boot = runtimes[-1]["boot_id"]
            os.kill(latest_node_pid(logfile), signal.SIGKILL)
            until(
                lambda: runtimes and runtimes[-1]["boot_id"] != old_boot,
                wrapper,
                spin_command=False,
            )
            new_boot = runtimes[-1]["boot_id"]
            # Keep Command's runtime callback queued. This first actual intent
            # uses the pre-crash prepared authority, without reviving an expired
            # stream or changing any ingress/output deadline.
            command._final_publisher.publish(old_prepared)
            accepted = manual(
                "drive.intent",
                {
                    "authority_id": authority,
                    "gesture": 1,
                    "sequence": 1,
                    "linear": 0.1,
                    "turn": 0,
                    "gear": "low",
                },
            )
            assert accepted["ok"], accepted
            until(
                lambda: any(
                    frame.source_id == authority and frame.close_reason == "intent"
                    for frame in final_frames
                ),
                wrapper,
                spin_command=False,
            )
            old_intent = next(
                frame
                for frame in reversed(final_frames)
                if frame.source_id == authority and frame.close_reason == "intent"
            )
            assert old_intent.received_steady <= time.monotonic() < old_intent.steady_deadline
            observed_sequence = runtimes[-1]["sequence"]
            # Replay the real production frame verbatim: identity, receipt and
            # deadline stay frozen. The actual native guard/Core must refuse it.
            command._final_publisher.publish(old_intent)
            until(
                lambda: (
                    len(
                        {
                            value["sequence"]
                            for value in runtimes
                            if value["boot_id"] == new_boot
                            and value["sequence"] > observed_sequence
                        }
                    )
                    >= 4
                ),
                wrapper,
                spin_command=False,
            )
            assert all(
                value["mode"] == "idle" for value in runtimes if value["boot_id"] == new_boot
            )
            until(
                lambda: (
                    command._output_incarnation is not None
                    and command._output_incarnation[1] == new_boot
                    and command._output_handoff_boot is None
                ),
                wrapper,
            )
            assert command._manual_target is None
            for _ in range(3):
                request = envelope("safety.estop")
                expected = command._latched_estop_revision + 1
                assert command.submit(SimpleEnvelope.model_validate(request)).accepted
                until(
                    lambda expected=expected: (
                        runtimes[-1]["estop_revision"] == expected
                        and command._pending_estop_request is None
                    ),
                    wrapper,
                )
            assert runtimes[-1]["estop_revision"] == 3
            original = request["request_id"]
            os.kill(latest_node_pid(logfile), signal.SIGKILL)
            until(
                lambda: (
                    runtimes[-1]["boot_id"] != new_boot
                    and runtimes[-1]["confirmed_estop_status"] == "software_latched"
                    and runtimes[-1]["estop_revision"] == 3
                    and command._output_handoff_boot is None
                ),
                wrapper,
            )
            assert command._latched_estop_request.request_id == original
            assert manual("safety.estop.reset", {"revision": 3})["ok"]
            until(
                lambda: (
                    runtimes[-1]["confirmed_estop_status"] == "clear"
                    and command._latched_estop_request is None
                ),
                wrapper,
            )
            assert len(probe.get_publishers_info_by_topic(output_topic)) == 1
            assert not probe.get_publishers_info_by_topic("/cmd_vel")
            assert not probe.get_subscriptions_info_by_topic("/cmd_vel")
            assert velocities and all(
                value.linear.x == value.angular.z == 0 for value in velocities
            )
    finally:
        commands.shutdown()
        observer.shutdown()
        command.destroy_node()
        probe.destroy_node()


@pytest.fixture
def native_navigation(isolated_ros, tmp_path):
    """Synthetic lifecycle recovery with explicit test-only installed MPPI 1.1.9 parameters."""
    import yaml
    from geometry_msgs.msg import TransformStamped
    from lifecycle_msgs.msg import State
    from lifecycle_msgs.srv import GetState
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from std_srvs.srv import Trigger
    from test_rgbd_diagnostic_launch import _child_with_argument, _group_exists
    from test_scan_epoch_live import _room_scan
    from tf2_ros import StaticTransformBroadcaster

    parameters = yaml.safe_load(
        (Path(__file__).resolve().parents[2] / "ros2/src/senior_robot/config/nav2.yaml").read_text()
    )
    follow = parameters["controller_server"]["ros__parameters"]["FollowPath"]
    # //Compatible: actual wheeltec MPPI 1.1.9 exports ObstaclesCritic, not
    # CostCritic. This temporary recovery fixture does not validate or modify
    # production navigation scoring. No CostCritic weights are mapped across.
    follow["critics"][follow["critics"].index("CostCritic")] = "ObstaclesCritic"
    del follow["CostCritic"]
    inflation = parameters["local_costmap"]["local_costmap"]["ros__parameters"]["inflation_layer"]
    follow["ObstaclesCritic"] = {
        "enabled": True,
        "consider_footprint": True,  # Keep the project's existing footprint collision mode.
        "cost_power": 1,
        # Exact defaults from actual 1.1.9 obstacles_critic.cpp, used only for
        # this synthetic no-chassis scene. The margin is a critic score term,
        # not a new static safety frame or a control stopping threshold.
        "repulsion_weight": 1.5,
        "critical_weight": 20.0,
        "collision_cost": 10000.0,
        "collision_margin_distance": 0.10,
        "near_goal_distance": 0.5,
        "cost_scaling_factor": inflation["cost_scaling_factor"],
        "inflation_radius": inflation["inflation_radius"],
    }
    parameters_path = tmp_path / "test-only-nav2-mppi-1.1.9.yaml"
    parameters_path.write_text(
        "# ISOLATED RECOVERY TEST ONLY: not production navigation parameters.\n"
        + yaml.safe_dump(parameters, sort_keys=False)
    )
    probe = isolated_ros.create_node("launch_recovery_nav_probe")
    executor = SingleThreadedExecutor()
    executor.add_node(probe)
    map_pub = probe.create_publisher(
        OccupancyGrid,
        "/map",
        QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        ),
    )
    odom_pub = probe.create_publisher(Odometry, "/wheel/odom", 10)
    scan_pub = probe.create_publisher(LaserScan, "/scan", qos_profile_sensor_data)
    transforms = []
    for parent, child in (("map", "odom"), ("odom", "base_link"), ("base_link", "laser_link")):
        value = TransformStamped()
        value.header.frame_id, value.child_frame_id = parent, child
        value.transform.rotation.w = 1.0
        transforms.append(value)
    broadcaster = StaticTransformBroadcaster(probe)
    broadcaster.sendTransform(transforms)
    occupancy = OccupancyGrid()
    occupancy.header.frame_id = "map"
    occupancy.info.resolution = 0.05
    occupancy.info.width = occupancy.info.height = 80
    occupancy.info.origin.position.x = occupancy.info.origin.position.y = -2.0
    occupancy.info.origin.orientation.w = 1.0
    occupancy.data = [0] * (80 * 80)
    occupancy.data[60 * 80 + 60] = 100
    scan = _room_scan()
    odom = Odometry()
    odom.header.frame_id, odom.child_frame_id = "odom", "base_link"
    odom.pose.pose.orientation.w = 1.0
    managed = ("controller_server", "planner_server", "bt_navigator")
    clients = {name: probe.create_client(GetState, f"/{name}/get_state") for name in managed}
    clients["manager"] = probe.create_client(Trigger, "/lifecycle_manager_navigation/is_active")
    states, pending = {}, {}
    log_path = tmp_path / "nav-group.log"
    log = log_path.open("w")
    process = subprocess.Popen(
        [
            sys.executable,
            "-c",
            "from ros2cli.cli import main; raise SystemExit(main())",
            "launch",
            "senior_robot",
            "navigation.launch.py",
            "use_sim_time:=false",
            f"parameters:={parameters_path}",
        ],
        stdout=log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )

    def tick():
        assert process.poll() is None, log_path.read_text()
        stamp = probe.get_clock().now().to_msg()
        occupancy.header.stamp = odom.header.stamp = scan.header.stamp = stamp
        map_pub.publish(occupancy)
        odom_pub.publish(odom)
        scan_pub.publish(scan)
        executor.spin_once(timeout_sec=0.05)
        assert not probe.get_publishers_info_by_topic("/cmd_vel"), log_path.read_text()

    def until(predicate):
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline and not predicate():
            tick()
        assert predicate(), log_path.read_text()

    def reset_states():
        for future, _ in pending.values():
            future.cancel()
        pending.clear()
        states.clear()

    def active():
        for name, client in clients.items():
            item = pending.get(name)
            if item is not None:
                future, sent = item
                if future.done():
                    result = future.result() if future.exception() is None else None
                    states[name] = (
                        (
                            result.success
                            if name == "manager"
                            else result.current_state.id == State.PRIMARY_STATE_ACTIVE
                        )
                        if result
                        else False
                    )
                    del pending[name]
                elif time.monotonic() - sent > 1:
                    future.cancel()
                    del pending[name]
                    states[name] = False
            if name not in pending and client.service_is_ready():
                request = Trigger.Request() if name == "manager" else GetState.Request()
                pending[name] = client.call_async(request), time.monotonic()
        return len(states) == 4 and all(states.values())

    tracked = {}

    def birth(pid):
        try:
            # Match the lifetime before any failure-path signal, not a reused PID.
            return Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()[19]
        except FileNotFoundError:
            return None

    def pids():
        values = {
            name: _child_with_argument(process.pid, ("/" + name).encode())
            for name in (*managed, "lifecycle_manager", "navigation_adapter")
        }
        for pid in values.values():
            if pid is not None and (identity := birth(pid)) is not None:
                tracked[pid] = identity
        return values

    try:
        until(active)
        until(lambda: all(pids().values()))
        yield SimpleNamespace(
            probe=probe,
            tick=tick,
            until=until,
            active=active,
            reset=reset_states,
            pids=pids,
            log=log_path,
        )
    finally:
        reset_states()
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
        try:
            assert process.wait(timeout=15) == 0, log_path.read_text()
            assert not _group_exists(process.pid), log_path.read_text()
            assert "replacement refused" not in log_path.read_text()
            assert all(birth(pid) != identity for pid, identity in tracked.items()), (
                log_path.read_text()
            )
        finally:
            for pid, identity in tracked.items():
                if birth(pid) == identity:
                    try:
                        os.killpg(pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
            log.close()
            executor.shutdown()
            probe.destroy_node()


def test_native_nav2_server_crash_rebuilds_old_group_before_active(native_navigation):
    nav = native_navigation
    before = nav.pids()
    os.kill(before["planner_server"], signal.SIGKILL)
    nav.until(lambda: nav.pids()["planner_server"] not in (None, before["planner_server"]))
    assert all(not Path(f"/proc/{pid}").exists() for pid in before.values())
    nav.reset()
    nav.until(nav.active)
    assert all(pid is not None and pid != before[name] for name, pid in nav.pids().items())


def test_native_nav2_manager_crash_cleans_old_group_and_does_not_replay_goal(native_navigation):
    from action_msgs.msg import GoalStatus, GoalStatusArray
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    nav = native_navigation
    statuses = deque(maxlen=128)

    old_subscription = nav.probe.create_subscription(
        GoalStatusArray,
        "/navigate_to_pose/_action/status",
        statuses.append,
        QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        ),
    )
    action = ActionClient(nav.probe, NavigateToPose, "/navigate_to_pose")
    nav.until(action.server_is_ready)
    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = "map"
    goal.pose.pose.position.x, goal.pose.pose.orientation.w = 0.4, 1.0
    sending = action.send_goal_async(goal)
    nav.until(sending.done)
    accepted = sending.result()
    assert accepted.accepted
    goal_id = bytes(accepted.goal_id.uuid)
    nav.until(
        lambda: any(
            bytes(item.goal_info.goal_id.uuid) == goal_id
            for message in statuses
            for item in message.status_list
        )
    )
    before = nav.pids()
    nav.probe.destroy_subscription(old_subscription)
    statuses.clear()
    os.kill(before["lifecycle_manager"], signal.SIGKILL)
    nav.until(lambda: nav.pids()["lifecycle_manager"] not in (None, before["lifecycle_manager"]))
    assert all(not Path(f"/proc/{pid}").exists() for pid in before.values())
    nav.reset()
    nav.until(nav.active)
    nav.until(
        lambda: len(nav.probe.get_publishers_info_by_topic("/navigate_to_pose/_action/status")) == 1
    )
    # Old native members are all gone and the rebuilt graph has one writer.
    # A fresh volatile reader cannot reuse the old subscriber's queued history;
    # no unavailable RMW publisher_gid field is invented or inferred.
    subscription = nav.probe.create_subscription(
        GoalStatusArray,
        "/navigate_to_pose/_action/status",
        statuses.append,
        QoSProfile(
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.RELIABLE,
        ),
    )
    # //Compatible: Humble has no Subscription publisher-count API; the public
    # Node graph query above already confirmed the unique rebuilt writer.
    # Humble publishes action status on changes, not an empty idle heartbeat.
    # Query the original UUID through the public goal handle as a causal real
    # response from the new server; silence on the status topic cannot pass it.
    forgotten = accepted.get_result_async()
    nav.until(forgotten.done)
    assert forgotten.result().status == GoalStatus.STATUS_UNKNOWN
    for _ in range(20):
        nav.tick()
    assert not any(
        bytes(item.goal_info.goal_id.uuid) == goal_id
        for message in statuses
        for item in message.status_list
    )
    nav.probe.destroy_subscription(subscription)
    action.destroy()
