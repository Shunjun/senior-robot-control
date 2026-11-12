"""Exercise the actual command callback with controlled clocks and ROS types."""

import importlib.util
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace

import pytest


@pytest.fixture
def navigation(monkeypatch):
    modules = {
        "rclpy": {},
        "geometry_msgs.msg": {"Twist": object},
        "rclpy.clock": {"Clock": object, "ClockType": object},
        "rclpy.node": {"Node": object},
        "rclpy.parameter": {"Parameter": object},
        "rclpy.qos": {"qos_profile_sensor_data": object()},
        "rclpy.subscription": {"Subscription": object},
        "senior_motion_msgs.msg": {"DriveExecutionFeedback": object, "FinalCommand": object},
        "senior_robot.motion_lifecycle": {"with_dds_info": lambda callback: callback},
    }
    for name, attributes in modules.items():
        stub = ModuleType(name)
        stub.__dict__.update(attributes)
        monkeypatch.setitem(sys.modules, name, stub)
    path = Path("ros2/src/senior_robot/senior_robot/command_node.py")
    spec = importlib.util.spec_from_file_location("command_clock_under_test", path)
    module = importlib.util.module_from_spec(spec)
    monkeypatch.setitem(sys.modules, spec.name, module)
    spec.loader.exec_module(module)
    clock = SimpleNamespace(mono=10.0, wall=100.0, qualification=0.03, graph=0.0, jump=0.0)
    monkeypatch.setattr(
        module,
        "time",
        SimpleNamespace(time=lambda: clock.wall, time_ns=lambda: round(clock.wall * 1e9)),
    )
    node = module.CommandNode.__new__(module.CommandNode)
    node._monotonic_clock = SimpleNamespace(monotonic=lambda: clock.mono)
    attempts, accepted, ended, published, logs = [], [], [], [], []

    def qualified():
        clock.mono += clock.qualification
        clock.wall += clock.qualification + clock.jump
        return True

    def endpoints(_topic):
        clock.mono += clock.graph
        clock.wall += clock.graph
        return [SimpleNamespace(node_name="controller_server", endpoint_gid=b"controller")]

    def accept(execution, sequence, velocity, sampled, now):
        attempts.append((sampled, now))
        if not sampled <= now < sampled + 0.3:
            return False
        accepted.append((execution, sequence, velocity))
        return True

    node._arbiter = SimpleNamespace(accept_navigation_sample=accept, end_navigation=ended.append)
    node.get_publishers_info_by_topic = endpoints
    node.publish_command = lambda: published.append(True)
    node.destroy_subscription = lambda _subscription: None
    node.get_logger = lambda: SimpleNamespace(warning=logs.append)
    topic = "/control/navigation/run_" + "a" * 32 + "/cmd_vel"
    node._navigation_source = module._NavigationSource(
        "private-execution", topic, 90.0, object(), 99_000_000_000, qualified
    )
    msg = SimpleNamespace(linear=SimpleNamespace(x=0.08), angular=SimpleNamespace(z=0.0))
    info = {"source_timestamp": 100_000_000_000, "publisher_gid": b"controller"}
    return SimpleNamespace(
        node=node,
        clock=clock,
        attempts=attempts,
        accepted=accepted,
        ended=ended,
        published=published,
        logs=logs,
        topic=topic,
        msg=msg,
        info=info,
    )


def test_qualification_work_is_not_a_wall_clock_jump(navigation):
    rig = navigation
    source = rig.node._navigation_source
    rig.node._on_navigation_sample(rig.topic, rig.msg, rig.info)
    assert rig.node._navigation_source is source
    assert len(rig.accepted) == 1 and not rig.ended and not rig.logs
    assert rig.attempts[0][1] == pytest.approx(10.03)


@pytest.mark.parametrize("jump", [-0.03, 0.03])
def test_real_wall_clock_jump_still_clears_source_once_with_reason(navigation, jump):
    rig = navigation
    rig.clock.jump = jump
    rig.node._on_navigation_sample(rig.topic, rig.msg, rig.info)
    rig.node._on_navigation_sample(rig.topic, rig.msg, rig.info)
    assert rig.node._navigation_source is None and len(rig.ended) == 1
    assert not rig.accepted and len(rig.logs) == 1
    assert "clock_offset_changed" in rig.logs[0]
    assert "private-execution" not in rig.logs[0] and rig.topic not in rig.logs[0]


@pytest.mark.parametrize("delay_stage", ["qualification", "graph"])
def test_freshness_uses_current_time_after_all_qualification_work(navigation, delay_stage):
    rig = navigation
    rig.clock.qualification = 0.0
    setattr(rig.clock, delay_stage, 0.35)
    rig.node._on_navigation_sample(rig.topic, rig.msg, rig.info)
    assert rig.node._navigation_source is not None and not rig.accepted
    assert rig.attempts[0][1] == pytest.approx(10.35)


def test_old_dds_source_stamp_is_never_restamped_by_slow_qualification(navigation):
    rig = navigation
    rig.info["source_timestamp"] = 99_600_000_000
    rig.node._on_navigation_sample(rig.topic, rig.msg, rig.info)
    assert not rig.accepted and rig.node._navigation_source is not None
    assert rig.attempts[0][0] == pytest.approx(9.59)
    assert rig.node._navigation_source.last_stamp == 0


def test_real_qualification_loss_is_cleared_with_one_reason(navigation):
    rig = navigation
    rig.node._navigation_source.qualified = lambda: False
    rig.node._on_navigation_sample(rig.topic, rig.msg, rig.info)
    rig.node.clear_navigation_source("private-execution")
    assert not rig.accepted and len(rig.ended) == 1 and len(rig.logs) == 1
    assert "qualification_lost" in rig.logs[0]
