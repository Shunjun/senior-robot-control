"""Selection and installation of the scoped RGB-D simulation transport."""

import runpy
import sys
from pathlib import Path
from types import ModuleType
from unittest.mock import Mock, patch
from xml.etree import ElementTree

import pytest

ROOT = Path(__file__).resolve().parents[2]
ROBOT = ROOT / "ros2/src/senior_robot"
sys.path.insert(0, str(ROBOT))


def select(env=None, enabled=True, profile=None, cwd=None, rmw="rmw_fastrtps_cpp"):
    if profile is None:
        profile = str(ROBOT / "config/rgbd_transport.xml")
    from senior_robot.rgbd_transport import transport_environment

    return transport_environment(env or {}, enabled, profile, cwd or ROOT, lambda: rmw)


def test_default_profile_for_humble_fastdds():
    assert select() == {"FASTRTPS_DEFAULT_PROFILES_FILE": str(ROBOT / "config/rgbd_transport.xml")}


def test_explicit_profile(tmp_path):
    profile = tmp_path / "custom.xml"
    profile.write_text("<profiles/>")
    assert select(profile=str(profile)) == {"FASTRTPS_DEFAULT_PROFILES_FILE": str(profile)}


def test_invalid_profile_fails_clearly(tmp_path):
    profile = tmp_path / "bad.xml"
    with pytest.raises(ValueError, match="profile"):
        select(profile=str(profile))
    profile.write_text("<invalid")
    with pytest.raises(ValueError, match="profile"):
        select(profile=str(profile))


def test_detected_non_fastdds_does_not_read_profile():
    assert select(profile="/missing.xml", rmw="rmw_cyclonedds_cpp") == {}


def test_unknown_rmw_query_fails_clearly():
    from senior_robot.rgbd_transport import transport_environment

    def unavailable():
        raise ImportError("no rclpy")

    with pytest.raises(RuntimeError, match="RMW"):
        transport_environment({}, True, "/missing.xml", ROOT, unavailable)


def test_disabled():
    assert select(enabled=False) == {}


@pytest.mark.parametrize(
    "name",
    [
        "FASTRTPS_DEFAULT_PROFILES_FILE",
        "FASTDDS_DEFAULT_PROFILES_FILE",
        "FASTDDS_BUILTIN_TRANSPORTS",
        "RMW_FASTRTPS_USE_QOS_FROM_XML",
        "SKIP_DEFAULT_XML",
    ],
)
def test_existing_configuration_is_not_overridden(name):
    assert select({name: "custom"}) == {}


@pytest.mark.parametrize(
    "name",
    [
        "FASTRTPS_DEFAULT_PROFILES_FILE",
        "FASTDDS_DEFAULT_PROFILES_FILE",
        "FASTDDS_BUILTIN_TRANSPORTS",
        "RMW_FASTRTPS_USE_QOS_FROM_XML",
        "SKIP_DEFAULT_XML",
    ],
)
def test_empty_configuration_does_not_disable_automatic_profile(name):
    assert select({name: ""}) == select()


def test_non_fastdds_rmw_is_untouched():
    assert select({"RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp"}) == {}


@pytest.mark.parametrize("rmw", ["rmw_fastrtps_cpp", "rmw_fastrtps_dynamic_cpp"])
def test_explicit_fastdds_rmw(rmw):
    assert select({"RMW_IMPLEMENTATION": rmw})


@pytest.mark.parametrize(
    "filename", ["DEFAULT_FASTRTPS_PROFILES.xml", "DEFAULT_FASTDDS_PROFILES.xml"]
)
def test_working_directory_profile_is_preserved(tmp_path, filename):
    (tmp_path / filename).touch()
    assert select(cwd=tmp_path) == {}


def test_profile_is_installed_and_contains_tested_transport():
    setuptools = ModuleType("setuptools")
    setuptools.setup = Mock()
    with patch.dict(sys.modules, {"setuptools": setuptools}):
        runpy.run_path(str(ROBOT / "setup.py"))
    setup = setuptools.setup
    files = dict(setup.call_args.kwargs["data_files"])["share/senior_robot/config"]
    assert "config/rgbd_transport.xml" in files
    root = ElementTree.parse(ROBOT / "config/rgbd_transport.xml").getroot()
    ns = {"dds": "http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles"}
    assert root.find(".//dds:segment_size", ns).text == "16777216"
    assert root.find(".//dds:useBuiltinTransports", ns).text == "false"
    assert {
        element.text for element in root.findall(".//dds:transport_descriptor/dds:type", ns)
    } == {"UDPv4", "SHM"}
    assert root.find(".//dds:maxMessageSize", ns) is None


def test_launch_scopes_transport_to_rgbd_children(monkeypatch):
    import importlib.util
    from types import SimpleNamespace

    class Action:
        def __init__(self, *args, **kwargs):
            self.args, self.kwargs = args, kwargs

    class Config(Action):
        def perform(self, context):
            return context.config[self.args[0]]

    launch = ModuleType("launch")
    launch.LaunchDescription = Action
    actions = ModuleType("launch.actions")
    for name in (
        "DeclareLaunchArgument",
        "GroupAction",
        "IncludeLaunchDescription",
        "OpaqueFunction",
        "SetEnvironmentVariable",
    ):
        setattr(actions, name, type(name, (Action,), {}))
    sources = ModuleType("launch.launch_description_sources")
    sources.PythonLaunchDescriptionSource = Action
    substitutions = ModuleType("launch.substitutions")
    substitutions.LaunchConfiguration = Config
    substitutions.PathJoinSubstitution = Action
    ros_actions = ModuleType("launch_ros.actions")
    ros_actions.Node = type("Node", (Action,), {})
    ros_substitutions = ModuleType("launch_ros.substitutions")
    ros_substitutions.FindPackageShare = Action
    for name, module in {
        "launch": launch,
        "launch.actions": actions,
        "launch.launch_description_sources": sources,
        "launch.substitutions": substitutions,
        "launch_ros": ModuleType("launch_ros"),
        "launch_ros.actions": ros_actions,
        "launch_ros.substitutions": ros_substitutions,
    }.items():
        monkeypatch.setitem(sys.modules, name, module)
    spec = importlib.util.spec_from_file_location(
        "rgbd_launch", ROBOT / "launch/simulation_rgbd.launch.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    group = next(item for item in description.args[0] if isinstance(item, actions.GroupAction))
    assert group.kwargs["scoped"] is True
    children = group.kwargs["actions"]
    assert isinstance(children[0], actions.OpaqueFunction)
    assert any(isinstance(item, actions.IncludeLaunchDescription) for item in children)
    assert sum(isinstance(item, ros_actions.Node) for item in children) == 3
    context = SimpleNamespace(
        environment={},
        config={
            "enable_rgbd_transport": "true",
            "rgbd_transport_profile": str(ROBOT / "config/rgbd_transport.xml"),
        },
    )
    rclpy = ModuleType("rclpy")
    utilities = ModuleType("rclpy.utilities")
    utilities.get_rmw_implementation_identifier = lambda: "rmw_fastrtps_cpp"
    monkeypatch.setitem(sys.modules, "rclpy", rclpy)
    monkeypatch.setitem(sys.modules, "rclpy.utilities", utilities)
    additions = children[0].kwargs["function"](context)
    assert {action.args for action in additions} == set(select().items())
    assert context.environment == {}
