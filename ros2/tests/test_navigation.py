"""Nav2 process wiring. Skipped without rclpy; a skip is not a Linux pass.

Do not treat a local skip as the Humble navigation check.
"""

from pathlib import Path

import pytest
import yaml


def test_nav2_costmap_nodes_receive_footprint_and_layers():
    config_path = (
        Path(__file__).resolve().parents[2] / "ros2/src/senior_navigation/config/nav2.yaml"
    )
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    # The controller and planner create separate, namespaced costmap nodes.
    local = config["local_costmap"]["local_costmap"]["ros__parameters"]
    global_ = config["global_costmap"]["global_costmap"]["ros__parameters"]
    assert local["global_frame"] == "odom"
    assert global_["global_frame"] == "map"
    assert local["footprint"] == global_["footprint"]
    assert len(yaml.safe_load(local["footprint"])) == 4
    # Live global/local consume one complete current Q, after fixed and before inflation.
    for costmap in (local, global_):
        assert costmap["plugins"] == ["static_layer", "current_scan_layer", "inflation_layer"]
        assert costmap["current_scan_layer"]["plugin"] == "senior_navigation::CurrentScanLayer"
        assert costmap["use_maximum"] is False
        assert costmap["static_layer"]["map_subscribe_transient_local"]
    controller = config["controller_server"]["ros__parameters"]
    assert controller["progress_checker_plugin"] == "progress_checker"
    assert controller["FollowPath"]["CostCritic"]["consider_footprint"]


def test_nav2_launch_config_keeps_recovery_output_before_arbitration():
    config_path = (
        Path(__file__).resolve().parents[2] / "ros2/src/senior_navigation/config/nav2.yaml"
    )
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    assert (
        config["navigation_adapter"]["ros__parameters"]["command_topic"] == "/control/recovery_cmd"
    )


def test_nav2_humble_uses_project_through_poses_tree():
    from xml.etree import ElementTree

    package = Path(__file__).resolve().parents[2] / "ros2/src/senior_navigation"
    config = yaml.safe_load((package / "config/nav2.yaml").read_text(encoding="utf-8"))
    navigator = config["bt_navigator"]["ros__parameters"]
    assert "nav2_compute_path_through_poses_action_bt_node" in navigator["plugin_lib_names"]
    path = package / "behavior_trees" / navigator["default_nav_through_poses_bt_xml"]
    sequence = ElementTree.parse(path).find("./BehaviorTree/Sequence")
    assert sequence is not None
    assert [child.tag for child in sequence] == ["ComputePathThroughPoses", "FollowPath"]
    assert sequence[0].attrib == {
        "goals": "{goals}",
        "path": "{path}",
        "planner_id": "GridBased",
    }
    assert sequence[1].attrib == {"path": "{path}", "controller_id": "FollowPath"}


def test_navigation_node_does_not_publish_cmd_vel():
    rclpy = pytest.importorskip("rclpy")
    from senior_navigation.navigation_adapter import SAFETY_COMMAND_TOPIC, create_navigation_node

    assert SAFETY_COMMAND_TOPIC == "/control/recovery_cmd"
    rclpy.init()
    node = None
    try:
        node = create_navigation_node()()
        publishers = getattr(node, "publishers", None)
        if publishers is None or callable(publishers):
            publishers = getattr(node, "_publishers", ())
        topics = {publisher.topic_name for publisher in publishers}
        assert topics - {"/parameter_events", "/rosout"} == {"/control/recovery_cmd"}
        assert "/cmd_vel" not in topics
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_installed_nav2_bringup_with_synthetic_map(tmp_path):
    """Verify actual Humble lifecycle and loaded plugins without a simulator."""
    import os
    import signal
    import subprocess
    import time
    from collections import deque
    from contextlib import suppress

    if os.environ.get("NAV2_TEST_INSTALLED_LAUNCH") != "1":
        pytest.skip("installed Nav2 bringup not selected")
    if os.environ.get("ROS_LOCALHOST_ONLY") != "1" or not os.environ.get("ROS_DOMAIN_ID"):
        pytest.fail("set ROS_LOCALHOST_ONLY=1 and an isolated ROS_DOMAIN_ID for the probe")

    import json
    import math

    import rclpy
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
    from lifecycle_msgs.msg import State
    from lifecycle_msgs.srv import GetState
    from nav2_msgs.action import ComputePathToPose, FollowPath
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rcl_interfaces.srv import GetParameters
    from rclpy.action import ActionClient
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from senior_motion_estimation.kinematics import compose, exponential
    from tf2_ros import TransformBroadcaster
    from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

    from vehicle_core.values import Pose2D

    rclpy.init()
    probe = rclpy.create_node("senior_nav2_bringup_probe")
    qos = QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    )
    map_pub = probe.create_publisher(OccupancyGrid, "/map", qos)
    transform_pub = StaticTransformBroadcaster(probe)
    received = deque(maxlen=4)
    costmap_sub = probe.create_subscription(
        OccupancyGrid, "/global_costmap/costmap", received.append, qos
    )
    assert costmap_sub is not None

    stamp = probe.get_clock().now().to_msg()
    transforms = []
    for parent, child in (("map", "odom"),):
        transform = TransformStamped()
        transform.header.frame_id, transform.child_frame_id = parent, child
        transform.header.stamp = stamp
        transform.transform.rotation.w = 1.0
        transforms.append(transform)
    transform_pub.sendTransform(transforms)
    body_tf = TransformBroadcaster(probe)
    odom_pub = probe.create_publisher(Odometry, "/wheel/odom", 1)
    fixture = {
        "pose": Pose2D(0, 0, 0),
        "command": Twist(),
        "received": 0.0,
        "previous": time.monotonic(),
        "following": False,
    }
    commands = []
    trajectory = []

    def receive_control(command):
        fixture["command"], fixture["received"] = command, time.monotonic()
        commands.append([fixture["received"], command.linear.x, command.angular.z])

    def advance_fixture():
        now = time.monotonic()
        dt, fixture["previous"] = min(now - fixture["previous"], 0.1), now
        command = fixture["command"]
        moving = fixture["following"] and now - fixture["received"] < 0.3
        vx, wz = (command.linear.x, command.angular.z) if moving else (0.0, 0.0)
        fixture["pose"] = compose(fixture["pose"], exponential(vx, 0, wz, dt))
        pose = fixture["pose"]
        transform = TransformStamped()
        transform.header.frame_id, transform.child_frame_id = "odom", "base_link"
        transform.header.stamp = probe.get_clock().now().to_msg()
        transform.transform.translation.x, transform.transform.translation.y = pose.x, pose.y
        transform.transform.rotation.z = math.sin(pose.yaw / 2)
        transform.transform.rotation.w = math.cos(pose.yaw / 2)
        body_tf.sendTransform(transform)
        odom = Odometry()
        odom.header, odom.child_frame_id = transform.header, "base_link"
        odom.pose.pose.position.x, odom.pose.pose.position.y = pose.x, pose.y
        odom.pose.pose.orientation = transform.transform.rotation
        odom.twist.twist.linear.x, odom.twist.twist.angular.z = vx, wz
        odom_pub.publish(odom)
        if fixture["following"]:
            trajectory.append([now, pose.x, pose.y, pose.yaw])

    fixture_timer = probe.create_timer(0.05, advance_fixture)

    occupancy = OccupancyGrid()
    occupancy.header.stamp = stamp
    occupancy.header.frame_id = "map"
    occupancy.info.resolution = 0.05
    occupancy.info.width = occupancy.info.height = 80
    occupancy.info.origin.position.x = occupancy.info.origin.position.y = -2.0
    occupancy.info.origin.orientation.w = 1.0
    occupied_cell = 60 * occupancy.info.width + 60
    occupancy.data = [0] * (occupancy.info.width * occupancy.info.height)
    occupancy.data[occupied_cell] = 100
    map_pub.publish(occupancy)

    managed = ("controller_server", "planner_server", "bt_navigator")
    state_clients = {name: probe.create_client(GetState, f"/{name}/get_state") for name in managed}
    parameters = {
        "controller_server": {"FollowPath.plugin": "nav2_mppi_controller::MPPIController"},
        "planner_server": {
            "GridBased.plugin": "nav2_smac_planner/SmacPlannerLattice",
            "GridBased.lattice_filepath": "lattice_diff.json",
        },
        "bt_navigator": {
            "default_nav_to_pose_bt_xml": "navigate.xml",
            "default_nav_through_poses_bt_xml": "navigate_through_poses.xml",
        },
    }
    parameter_clients = {
        name: probe.create_client(GetParameters, f"/{name}/get_parameters") for name in parameters
    }

    # S13 is a deterministic map/odometry fixture, not a second mapping owner.
    # It deliberately exercises static + inflation so Q authorization is tested
    # separately through S12 and the production reference consumer.
    package = Path(__file__).resolve().parents[2] / "ros2/src/senior_navigation"
    synthetic_params = yaml.safe_load((package / "config/nav2.yaml").read_text())
    for name in ("global_costmap", "local_costmap"):
        params = synthetic_params[name][name]["ros__parameters"]
        params["plugins"] = ["static_layer", "inflation_layer"]
        params["use_maximum"] = False
        params["static_layer"] = {
            "plugin": "nav2_costmap_2d::StaticLayer",
            "map_subscribe_transient_local": True,
        }
    synthetic_params["controller_server"]["ros__parameters"]["odom_topic"] = "/wheel/odom"
    parameter_file = tmp_path / "synthetic-nav2.yaml"
    parameter_file.write_text(yaml.safe_dump(synthetic_params, sort_keys=False))
    log_path = tmp_path / "nav2-launch.log"
    try:
        with log_path.open("w") as log:
            process = subprocess.Popen(
                [
                    "ros2",
                    "launch",
                    "senior_navigation",
                    "navigation.launch.py",
                    "use_sim_time:=false",
                    f"parameters:={parameter_file}",
                ],
                start_new_session=True,
                stdout=log,
                stderr=subprocess.STDOUT,
            )
            try:
                states = {}
                pending = {}
                map_ready = False
                deadline = time.monotonic() + 30.0
                while time.monotonic() < deadline and process.poll() is None:
                    assert not probe.get_publishers_info_by_topic("/cmd_vel"), log_path.read_text()
                    for name, client in state_clients.items():
                        future = pending.get(name)
                        if future is not None and future.done():
                            states[name] = future.result().current_state.id
                            del pending[name]
                        if name not in pending and client.service_is_ready():
                            pending[name] = client.call_async(GetState.Request())
                    map_ready = any(
                        grid.header.frame_id == "map"
                        and grid.info.width == occupancy.info.width
                        and grid.info.height == occupancy.info.height
                        and len(grid.data) > occupied_cell
                        and grid.data[occupied_cell] == 100
                        for grid in received
                    )
                    if map_ready and all(
                        states.get(name) == State.PRIMARY_STATE_ACTIVE for name in managed
                    ):
                        break
                    rclpy.spin_once(probe, timeout_sec=0.05)
                assert process.poll() is None, log_path.read_text()
                assert all(states.get(name) == State.PRIMARY_STATE_ACTIVE for name in managed), (
                    states,
                    log_path.read_text(),
                )
                assert map_ready, log_path.read_text()

                for name, expected in parameters.items():
                    client = parameter_clients[name]
                    assert client.wait_for_service(timeout_sec=2.0), name
                    request = GetParameters.Request(names=list(expected))
                    future = client.call_async(request)
                    rclpy.spin_until_future_complete(probe, future, timeout_sec=2.0)
                    assert future.done(), name
                    values = [value.string_value for value in future.result().values]
                    assert len(values) == len(expected), name
                    for (parameter, wanted), actual in zip(expected.items(), values, strict=True):
                        if parameter.endswith(("filepath", "_xml")):
                            assert Path(actual).name == wanted and Path(actual).is_file(), actual
                        else:
                            assert actual == wanted, (name, parameter, actual)

                # The launch assigns a private standard Twist run topic. Nav2
                # never acquires the final chassis publication boundary.
                import re

                private_topics = [
                    topic
                    for topic, types in probe.get_topic_names_and_types()
                    if re.fullmatch(r"/control/navigation/run_[0-9a-f]{32}/cmd_vel", topic)
                    and types == ["geometry_msgs/msg/Twist"]
                ]
                assert len(private_topics) == 1, private_topics
                expected_owners = {
                    private_topics[0]: {"controller_server"},
                    "/control/recovery_cmd": {"navigation_adapter"},
                }
                publishers = {}
                deadline = time.monotonic() + 5.0
                while time.monotonic() < deadline and process.poll() is None:
                    assert not probe.get_publishers_info_by_topic("/cmd_vel"), log_path.read_text()
                    publishers = {
                        topic: {
                            endpoint.node_name
                            for endpoint in probe.get_publishers_info_by_topic(topic)
                        }
                        for topic in expected_owners
                    }
                    if publishers == expected_owners:
                        break
                    rclpy.spin_once(probe, timeout_sec=0.05)
                assert process.poll() is None, log_path.read_text()
                assert publishers == expected_owners, (publishers, log_path.read_text())
                assert not probe.get_publishers_info_by_topic("/cmd_vel"), log_path.read_text()

                control_sub = probe.create_subscription(
                    Twist, private_topics[0], receive_control, 1
                )
                assert control_sub is not None and fixture_timer is not None
                planner = ActionClient(probe, ComputePathToPose, "/compute_path_to_pose")
                follower = ActionClient(probe, FollowPath, "/follow_path")

                def complete(future, seconds=10.0):
                    until = time.monotonic() + seconds
                    while not future.done() and time.monotonic() < until and process.poll() is None:
                        rclpy.spin_once(probe, timeout_sec=0.02)
                    assert future.done(), log_path.read_text()
                    return future.result()

                def pose_message(x, y):
                    pose = PoseStamped()
                    pose.header.frame_id = "map"
                    pose.header.stamp = probe.get_clock().now().to_msg()
                    pose.pose.position.x, pose.pose.position.y = x, y
                    pose.pose.orientation.w = 1.0
                    return pose

                assert planner.wait_for_server(timeout_sec=5.0), log_path.read_text()
                assert follower.wait_for_server(timeout_sec=5.0), log_path.read_text()
                # Finite channel, unknown beyond the observed start patch, lethal exterior.
                occupancy.data = [100] * (80 * 80)
                for y in range(24, 56):
                    for x in range(8, 72):
                        occupancy.data[y * 80 + x] = 0 if x < 48 else -1
                received.clear()
                occupancy.header.stamp = probe.get_clock().now().to_msg()
                map_pub.publish(occupancy)
                until = time.monotonic() + 8.0
                while time.monotonic() < until:
                    rclpy.spin_once(probe, timeout_sec=0.05)
                    if received and received[-1].data[40 * 80 + 60] == -1:
                        break
                assert received and received[-1].data[40 * 80 + 60] == -1, log_path.read_text()
                goal = ComputePathToPose.Goal()
                goal.start, goal.goal = pose_message(0.0, 0.0), pose_message(1.2, 0.0)
                goal.use_start, goal.planner_id = True, "GridBased"
                planning = complete(planner.send_goal_async(goal))
                assert planning.accepted, log_path.read_text()
                planned = complete(planning.get_result_async())
                assert planned.status == GoalStatus.STATUS_SUCCEEDED, log_path.read_text()
                path = planned.result.path
                assert path.poses and any(p.pose.position.x > 0.5 for p in path.poses)
                assert all(abs(p.pose.position.y) < 0.8 for p in path.poses)
                follow = FollowPath.Goal()
                follow.path, follow.controller_id, follow.goal_checker_id = (
                    path,
                    "FollowPath",
                    "general_goal_checker",
                )
                following = complete(follower.send_goal_async(follow))
                assert following.accepted, log_path.read_text()
                fixture["following"] = True
                followed = complete(following.get_result_async(), seconds=30.0)
                fixture["following"] = False
                final_pose = fixture["pose"]
                evidence = {
                    "fixture": (
                        "original SE2 compose/exponential; "
                        "actual private MPPI Twist; 20 Hz odometry"
                    ),
                    "path": [[p.pose.position.x, p.pose.position.y] for p in path.poses],
                    "commands": commands,
                    "trajectory": trajectory,
                    "final_pose": [final_pose.x, final_pose.y, final_pose.yaw],
                    "planning_status": planned.status,
                    "following_status": followed.status,
                }
                (tmp_path / "unknown-nav2-evidence.json").write_text(json.dumps(evidence, indent=2))
                assert followed.status == GoalStatus.STATUS_SUCCEEDED, log_path.read_text()
                assert commands and any(vx > 0.01 for _, vx, _ in commands)
                assert math.hypot(final_pose.x - 1.2, final_pose.y) <= 0.3
                assert trajectory and all(abs(y) + 0.27 < 0.8 for _, _, y, _ in trajectory)

                # Same finite channel, now partitioned by known lethal cells: request
                # a new actual Smac plan after the updated costmap has been observed.
                fixture["pose"] = Pose2D(0, 0, 0)
                fixture["command"] = Twist()
                for y in range(24, 56):
                    for x in range(52, 56):
                        occupancy.data[y * 80 + x] = 100
                received.clear()
                occupancy.header.stamp = probe.get_clock().now().to_msg()
                map_pub.publish(occupancy)
                until = time.monotonic() + 8.0
                while time.monotonic() < until:
                    rclpy.spin_once(probe, timeout_sec=0.05)
                    if received and received[-1].data[40 * 80 + 53] == 100:
                        break
                assert received and received[-1].data[40 * 80 + 53] == 100
                goal.start.header.stamp = goal.goal.header.stamp = probe.get_clock().now().to_msg()
                replanning = complete(planner.send_goal_async(goal))
                assert replanning.accepted, log_path.read_text()
                blocked = complete(replanning.get_result_async())
                evidence["blocked_planning_status"] = blocked.status
                evidence["blocked_path"] = [
                    [p.pose.position.x, p.pose.position.y] for p in blocked.result.path.poses
                ]
                (tmp_path / "unknown-nav2-evidence.json").write_text(json.dumps(evidence, indent=2))
                assert blocked.status != GoalStatus.STATUS_SUCCEEDED
                assert not blocked.result.path.poses
                planner.destroy()
                follower.destroy()

                os.kill(process.pid, signal.SIGINT)
                assert process.wait(timeout=12.0) == 0, log_path.read_text()
                assert "process has died" not in log_path.read_text(), log_path.read_text()
                with pytest.raises(ProcessLookupError):
                    os.killpg(process.pid, 0)
            finally:
                with suppress(ProcessLookupError):
                    os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5.0)
    finally:
        probe.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize("end", ["succeeded", "failed", "map_changed", "canceled"])
async def test_qualified_route_runs_in_order(end):
    import asyncio

    from senior_navigation.route import QualifiedRoute

    from vehicle_core.values import MapRef, Pose2D

    goals = []
    generation = "map-a"
    arrived = asyncio.Event()
    release = asyncio.Event()

    async def navigate(pose):
        goals.append(pose.x)
        if pose.x == 2:
            arrived.set()
            await release.wait()
        return end != "failed" or pose.x != 2

    route = QualifiedRoute(navigate, lambda ref, nonce: nonce == generation)
    run = asyncio.create_task(
        route.run(
            MapRef("map", 1),
            "map-a",
            (
                Pose2D(1, 0, 0),
                Pose2D(2, 0, 0),
                Pose2D(3, 0, 0),
            ),
        )
    )
    await asyncio.wait_for(arrived.wait(), 1)
    if end == "map_changed":
        generation = "map-b"
    elif end == "canceled":
        run.cancel()
    release.set()
    if end == "canceled":
        with pytest.raises(asyncio.CancelledError):
            await run
    else:
        assert await run is (end == "succeeded")
    assert goals == ([1, 2, 3] if end == "succeeded" else [1, 2])


@pytest.fixture
def navigation_source_path(monkeypatch):
    root = Path(__file__).resolve().parents[2]
    monkeypatch.syspath_prepend(str(root / "ros2/src/senior_navigation"))


async def test_route_service_reports_actual_index_and_blocks_next_on_lost_reference(
    navigation_source_path,
):
    import asyncio

    from senior_navigation.navigation_adapter import NavigationAdapter, RouteService
    from senior_navigation.route import RoutePlan

    from vehicle_core.values import MapRef, OperationResult, OperationStatus

    class GoalExecutor:
        def __init__(self):
            self.started = []
            self.results = {}
            self.canceled = []

        async def start(self, goal):
            self.started.append(goal.pose.x)
            key = str(len(self.started))
            self.results[key] = asyncio.get_running_loop().create_future()
            return key

        async def result(self, key):
            return await self.results[key]

        async def cancel(self, key):
            self.canceled.append(key)
            return OperationResult(key, OperationStatus.SUCCEEDED)

    ref = MapRef("yard", 3)
    generation = "gen-a"
    executor = GoalExecutor()
    service = RouteService(
        NavigationAdapter(executor), qualified=lambda r, token: (r, token) == (ref, generation)
    )
    plan = RoutePlan(
        execution_id="patrol-1",
        map_id="yard",
        reference_revision=3,
        map_generation=generation,
        poses=[[1.0, 0.0, 0.0], [2.0, 0.0, 0.0], [3.0, 0.0, 0.0]],
        task_epoch=1,
        effect_id="plan",
        action_ids=[
            str(i) for i in range(len([[1.0, 0.0, 0.0], [2.0, 0.0, 0.0], [3.0, 0.0, 0.0]]))
        ],
    )
    assert await service.start(plan)
    assert service.status("patrol-1").active_waypoint_index == 0
    executor.results["1"].set_result(True)
    for _ in range(20):
        if len(executor.started) == 2:
            break
        await asyncio.sleep(0.01)
    assert service.status("patrol-1").active_waypoint_index == 1
    generation = "gen-b"
    assert await service.cancel("patrol-1")
    assert service.status("patrol-1").status == "canceled"
    assert executor.started == [1, 2]
    assert executor.canceled == ["2"]
