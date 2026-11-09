"""Nav2 process wiring. Skipped without rclpy; a skip is not a Linux pass.

Do not treat a local skip as the Humble navigation check.
"""

from pathlib import Path

import pytest
import yaml


def test_nav2_costmap_nodes_receive_footprint_and_layers():
    config_path = Path(__file__).resolve().parents[2] / "ros2/src/senior_robot/config/nav2.yaml"
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    # The controller and planner create separate, namespaced costmap nodes.
    local = config["local_costmap"]["local_costmap"]["ros__parameters"]
    global_ = config["global_costmap"]["global_costmap"]["ros__parameters"]
    assert local["global_frame"] == "odom"
    assert global_["global_frame"] == "map"
    assert local["footprint"] == global_["footprint"]
    assert len(yaml.safe_load(local["footprint"])) == 4
    assert "obstacle_layer" in local["plugins"]
    assert "static_layer" in global_["plugins"]
    controller = config["controller_server"]["ros__parameters"]
    assert controller["progress_checker_plugin"] == "progress_checker"
    assert controller["FollowPath"]["CostCritic"]["consider_footprint"]


def test_nav2_launch_config_keeps_recovery_output_before_arbitration():
    config_path = Path(__file__).resolve().parents[2] / "ros2/src/senior_robot/config/nav2.yaml"
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    assert (
        config["navigation_adapter"]["ros__parameters"]["command_topic"] == "/control/recovery_cmd"
    )


def test_nav2_humble_uses_project_through_poses_tree():
    from xml.etree import ElementTree

    package = Path(__file__).resolve().parents[2] / "ros2/src/senior_robot"
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
    from senior_robot.navigation_adapter import SAFETY_COMMAND_TOPIC, create_navigation_node

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

    import rclpy
    from geometry_msgs.msg import TransformStamped
    from lifecycle_msgs.msg import State
    from lifecycle_msgs.srv import GetState
    from nav_msgs.msg import OccupancyGrid
    from rcl_interfaces.srv import GetParameters
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

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
    for parent, child in (("map", "odom"), ("odom", "base_link")):
        transform = TransformStamped()
        transform.header.frame_id, transform.child_frame_id = parent, child
        transform.header.stamp = stamp
        transform.transform.rotation.w = 1.0
        transforms.append(transform)
    transform_pub.sendTransform(transforms)

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

    log_path = tmp_path / "nav2-launch.log"
    try:
        with log_path.open("w") as log:
            process = subprocess.Popen(
                ["ros2", "launch", "senior_robot", "navigation.launch.py", "use_sim_time:=false"],
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

                expected_owners = {
                    "/control/navigation_cmd": {"controller_server"},
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

    from senior_control.models import MapRef, Pose2D
    from senior_control.navigation import QualifiedRoute

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
    monkeypatch.syspath_prepend(str(root / "ros2/src/senior_robot"))


async def test_route_service_reports_actual_index_and_blocks_next_on_lost_reference(
    navigation_source_path,
):
    import asyncio

    from senior_robot.navigation_adapter import NavigationAdapter, RouteService

    from senior_control.models import MapRef, OperationResult, OperationStatus
    from senior_control.navigation import RoutePlan

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


async def test_starting_route_cannot_inherit_new_connection_epoch(navigation_source_path):
    import asyncio
    from contextlib import asynccontextmanager

    from senior_robot.navigation_adapter import NavigationAdapter, RouteService

    from senior_control.navigation import RoutePlan

    epoch = 4
    setup = asyncio.Event()
    ready = asyncio.Event()

    @asynccontextmanager
    async def prepare(execution_id):
        assert execution_id == "old"
        setup.set()
        await ready.wait()
        yield "/control/navigation/run_fixture/cmd_vel"

    class NeverNavigate:
        async def start(self, goal):
            pytest.fail("old route attempted to start after reconnect")

        async def result(self, execution_id):
            pytest.fail("old route attempted to read a result")

    service = RouteService(
        NavigationAdapter(NeverNavigate()),
        qualified=lambda _ref, _generation: True,
        prepare=prepare,
        connection_epoch=lambda: epoch,
    )
    plan = RoutePlan(
        execution_id="old",
        map_id="yard",
        reference_revision=3,
        map_generation="g",
        poses=[[1.0, 0.0, 0.0]],
        connection_epoch=4,
    )
    assert await service.start(plan, wait_for_goal=False)
    await asyncio.wait_for(setup.wait(), 1)
    epoch = 5
    ready.set()
    for _ in range(20):
        if service.current is None:
            break
        await asyncio.sleep(0.01)
    assert service.status("old").status == "failed"
    retry = plan.model_copy(update={"execution_id": "stale-queued"})
    assert not await service.start(retry, wait_for_goal=False)
