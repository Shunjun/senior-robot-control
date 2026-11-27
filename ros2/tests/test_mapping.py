"""Mapping process wiring. Skipped without rclpy; a skip is not a Linux pass.

Do not treat a local skip as the Humble SLAM check. This host does not load slam_toolbox.
"""

import runpy
import sys
from pathlib import Path
from types import ModuleType

import pytest


@pytest.fixture
def mapping_launch(monkeypatch):
    class Action:
        def __init__(self, *args, **kwargs):
            self.args, self.kwargs = args, kwargs

    class LaunchConfiguration(Action):
        def perform(self, context):
            return context[self.args[0]]

    modules = {
        "launch": {"LaunchDescription": Action},
        "launch.actions": {
            "DeclareLaunchArgument": type("DeclareLaunchArgument", (Action,), {}),
            "OpaqueFunction": type("OpaqueFunction", (Action,), {}),
            "RegisterEventHandler": type("RegisterEventHandler", (Action,), {}),
        },
        "launch.event_handlers": {
            "OnProcessStart": type("OnProcessStart", (Action,), {}),
            "OnProcessExit": type("OnProcessExit", (Action,), {}),
        },
        "launch.substitutions": {
            "LaunchConfiguration": LaunchConfiguration,
            "PathJoinSubstitution": Action,
        },
        "launch_ros": {},
        "launch_ros.actions": {"Node": type("Node", (Action,), {})},
        "launch_ros.substitutions": {"FindPackageShare": Action},
    }
    for name, members in modules.items():
        module = ModuleType(name)
        vars(module).update(members)
        monkeypatch.setitem(sys.modules, name, module)
    launch_file = (
        Path(__file__).resolve().parents[2] / "ros2/src/senior_mapping/launch/mapping.launch.py"
    )
    description = runpy.run_path(str(launch_file))["generate_launch_description"]()
    actions = description.args[0]
    defaults = {
        action.args[0]: action.kwargs["default_value"]
        for action in actions
        if isinstance(action, modules["launch.actions"]["DeclareLaunchArgument"])
    }
    assert defaults["capture_directory"] == ""
    assert defaults["use_sim_time"] == "false"
    assert defaults["odom_frame"] == "odom"
    (select,) = [
        action.kwargs["function"]
        for action in actions
        if isinstance(action, modules["launch.actions"]["OpaqueFunction"])
    ]

    def select_nodes(context):
        # Launch diagnostics wrap the same nodes; retain all original command assertions.
        return [
            action
            for action in select(context)
            if isinstance(action, modules["launch_ros.actions"]["Node"])
        ]

    return select_nodes, defaults


def test_default_mapping_keeps_stock_graph_and_adapter(mapping_launch):
    select, defaults = mapping_launch
    actions = select(defaults)
    assert [(node.kwargs["package"], node.kwargs["executable"]) for node in actions] == [
        ("slam_toolbox", "async_slam_toolbox_node"),
        ("senior_mapping", "slam_adapter"),
    ]
    assert actions[0].kwargs["parameters"][-1]["odom_frame"] == "odom"
    assert actions[1].kwargs["parameters"][-1]["odom_frame"] == "odom"


def test_mapping_capture_selects_single_native_graph_owner(mapping_launch, tmp_path):
    select, defaults = mapping_launch
    (node,) = select({**defaults, "capture_directory": str(tmp_path), "use_sim_time": "true"})
    assert node.kwargs["package"] == "senior_mapping"
    assert node.kwargs["executable"] == "slam_graph_node"
    assert node.kwargs["name"] == "slam_toolbox"  # Match the existing slam.yaml section.
    config, overrides = node.kwargs["parameters"]
    assert config.args[0][-1] == "slam.yaml"
    assert overrides["use_sim_time"].perform({"use_sim_time": "true"}) == "true"
    assert overrides["capture_directory"] == str(tmp_path)
    assert overrides["use_map_saver"] is False
    assert overrides["enable_interactive_mode"] is False
    assert overrides["odom_frame"] == "odom"


def test_mapping_capture_requires_absolute_directory(mapping_launch):
    select, defaults = mapping_launch
    with pytest.raises(ValueError, match="absolute"):
        select({**defaults, "capture_directory": "relative/capture"})


@pytest.mark.parametrize("capture", [False, True])
def test_bounded_lidar_uses_standard_tf_without_credential_relay(mapping_launch, tmp_path, capture):
    select, defaults = mapping_launch
    capture_directory = str(tmp_path) if capture else ""
    (graph,) = select(
        {**defaults, "verified_static_lidar": "true", "capture_directory": capture_directory}
    )
    assert graph.kwargs["package"] == "senior_mapping"
    assert graph.kwargs["executable"] == "slam_graph_node"
    _, bridge = graph.kwargs["parameters"]
    assert bridge["capture_directory"] == capture_directory
    assert bridge["odom_frame"] == "odom"
    assert bridge["bounded_scan_topic"] == "/safety/raw/scan"
    assert bridge["lidar_frame"] == "laser_link"
    assert not any("receipt" in key or "verified_static" in key for key in bridge)


@pytest.mark.parametrize(
    ("argument", "value"),
    [
        ("bounded_scan_topic", "/scan"),
        ("lidar_frame", "base_link"),
        ("odom_frame", ""),
        ("odom_frame", "map"),
    ],
)
def test_verified_static_lidar_rejects_unqualified_inputs(mapping_launch, argument, value):
    select, defaults = mapping_launch
    with pytest.raises(ValueError, match=argument):
        select({**defaults, "verified_static_lidar": "true", argument: value})


@pytest.mark.parametrize("mode", ["stock", "capture", "verified"])
def test_mapping_passes_explicit_odom_frame_to_selected_graph_and_adapter(
    mapping_launch, tmp_path, mode
):
    select, defaults = mapping_launch
    context = {**defaults, "odom_frame": "odom_combined"}
    if mode == "capture":
        context["capture_directory"] = str(tmp_path)
    elif mode == "verified":
        context["verified_static_lidar"] = "true"
    nodes = select(context)
    graphs = [node for node in nodes if node.kwargs["name"] == "slam_toolbox"]
    assert len(graphs) == 1
    assert graphs[0].kwargs["parameters"][-1]["odom_frame"] == "odom_combined"
    if mode == "stock":
        adapters = [node for node in nodes if node.kwargs["executable"] == "slam_adapter"]
        assert len(adapters) == 1
        assert adapters[0].kwargs["parameters"][-1]["odom_frame"] == "odom_combined"
    else:
        assert graphs[0].kwargs["package"] == "senior_mapping"
        assert graphs[0].kwargs["executable"] == "slam_graph_node"
        assert all(node.kwargs["executable"] != "slam_adapter" for node in nodes)


def test_mapping_node_does_not_publish_cmd_vel():
    rclpy = pytest.importorskip("rclpy")
    from senior_mapping.slam_adapter import create_mapping_node

    rclpy.init()
    node = None
    try:
        node = create_mapping_node()()
        publishers = getattr(node, "publishers", None)
        if publishers is None or callable(publishers):
            publishers = getattr(node, "_publishers", ())
        topics = {publisher.topic_name for publisher in publishers}
        assert "/cmd_vel" not in topics
        assert all("cmd_vel" not in topic for topic in topics)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
