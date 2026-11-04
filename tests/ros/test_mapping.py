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
        Path(__file__).resolve().parents[2] / "ros2/src/senior_robot/launch/mapping.launch.py"
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
    (select,) = [
        action.kwargs["function"]
        for action in actions
        if isinstance(action, modules["launch.actions"]["OpaqueFunction"])
    ]
    return select, defaults


def test_default_mapping_keeps_stock_graph_and_adapter(mapping_launch):
    select, defaults = mapping_launch
    actions = select(defaults)
    assert [(node.kwargs["package"], node.kwargs["executable"]) for node in actions] == [
        ("slam_toolbox", "async_slam_toolbox_node"),
        ("senior_robot", "slam_adapter"),
    ]


def test_mapping_capture_selects_single_native_graph_owner(mapping_launch, tmp_path):
    select, defaults = mapping_launch
    (node,) = select({**defaults, "capture_directory": str(tmp_path), "use_sim_time": "true"})
    assert node.kwargs["package"] == "senior_slam_bridge"
    assert node.kwargs["executable"] == "slam_graph_node"
    assert node.kwargs["name"] == "slam_toolbox"  # Match the existing slam.yaml section.
    config, overrides = node.kwargs["parameters"]
    assert config.args[0][-1] == "slam.yaml"
    assert overrides["use_sim_time"].perform({"use_sim_time": "true"}) == "true"
    assert overrides["capture_directory"] == str(tmp_path)
    assert overrides["use_map_saver"] is False
    assert overrides["enable_interactive_mode"] is False


def test_mapping_capture_requires_absolute_directory(mapping_launch):
    select, defaults = mapping_launch
    with pytest.raises(ValueError, match="absolute"):
        select({**defaults, "capture_directory": "relative/capture"})


def test_mapping_node_does_not_publish_cmd_vel():
    rclpy = pytest.importorskip("rclpy")
    from senior_robot.slam_adapter import create_mapping_node

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
