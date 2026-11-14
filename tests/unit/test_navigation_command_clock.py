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
        "std_msgs.msg": {"String": object},
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
    node._retirements = {}
    node._generation = 1
    node._active = False
    node._publish_barrier = lambda *args: None
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


def test_retirement_waits_for_correlated_gate_receipt(navigation):
    rig = navigation
    node = rig.node
    node._owner_boot = "command-boot"
    node._awaiting_feedback = {}
    node.get_parameter = lambda _: SimpleNamespace(value="robot")
    node.get_publishers_info_by_topic = lambda _: [
        SimpleNamespace(node_name="safety_gate", node_namespace="/", endpoint_gid=b"gate")
    ]
    node.clear_navigation_source("private-execution")
    token, generation, _ = node._retirements["private-execution"]
    assert not node.navigation_source_retired("private-execution")
    msg = SimpleNamespace(
        robot_id="robot",
        owner_boot_id="old-boot",
        request_id=token,
        session_id="private-execution",
        sequence=generation,
        status="retired",
    )
    node._on_drive_feedback(msg, {"publisher_gid": b"gate"})
    assert not node.navigation_source_retired("private-execution")
    msg.owner_boot_id = "command-boot"
    node._on_drive_feedback(msg, {"publisher_gid": b"gate"})
    assert node.navigation_source_retired("private-execution")


@pytest.mark.parametrize("handoff", [True, False])
def test_gate_retires_generation_only_after_brake_owner_handoff(handoff):
    import ast
    import time

    from senior_control.models import Velocity
    from senior_control.safety.arbiter import FinalCommandFence

    # Execute the actual callback without importing/starting rclpy. Its ROS
    # transport objects are controlled below; no gate algorithm is substituted.
    tree = ast.parse(Path("ros2/src/senior_robot/senior_robot/safety_node.py").read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "SafetyGate")
    callback = next(
        n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == "_on_command"
    )
    namespace = {"time": time, "Velocity": Velocity, "DriveFeedbackMessage": SimpleNamespace}
    exec(
        compile(ast.Module(body=[callback], type_ignores=[]), "actual_safety_callback", "exec"),
        namespace,
    )
    fence = FinalCommandFence()
    fence.set_publisher(b"gate-input")
    events = []
    node = SimpleNamespace(
        _final_fence=fence,
        _motion_core=SimpleNamespace(
            command=lambda message: events.append(("handoff", message.generation)) or handoff,
            revoke=lambda reason: events.append(("closed", reason)),
        ),
        _final_identity=None,
        _command_deadline=0,
        _command_generation=0,
        get_parameter=lambda _: SimpleNamespace(value="robot"),
        _replace_drive_feedback=lambda *args: None,
        _publish=lambda velocity, reason: events.append(("output", velocity)),
        _drive_feedback_publisher=SimpleNamespace(
            publish=lambda msg: events.append(("receipt", msg))
        ),
    )
    msg = SimpleNamespace(
        robot_id="robot",
        owner_boot_id="owner",
        generation=4,
        sequence=0,
        steady_deadline=0,
        vx=0.0,
        wz=0.0,
        session_id="task",
        connection_epoch=0,
        request_id="retire-token",
        gesture=0,
        source_kind="navigation",
        source_id="task",
        source_epoch=1,
        received_steady=1.0,
        close_reason="task_revoke",
    )
    namespace["_on_command"](node, msg, b"gate-input")
    assert events[0] == ("handoff", 4)
    if not handoff:
        assert events[1] == ("closed", "target_handoff_failed")
        assert not any(event[0] == "receipt" for event in events)
        return
    assert not any(event[0] == "output" for event in events)
    assert events[1][1].status == "retired" and events[1][1].sequence == 4
    msg.generation = 3
    namespace["_on_command"](node, msg, b"gate-input")
    assert len(events) == 2


def test_shutdown_and_publish_convert_velocity_to_ros_float_fields(navigation, monkeypatch):
    from senior_control.models import Velocity

    class FloatVector:
        def __setattr__(self, name, value):
            assert isinstance(value, float), f"ROS float field {name} rejected {type(value)}"
            super().__setattr__(name, value)

    class StrictTwist:
        def __init__(self):
            self.linear = FloatVector()
            self.angular = FloatVector()

    monkeypatch.setattr(sys.modules["command_clock_under_test"], "Twist", StrictTwist)
    node = navigation.node
    messages, events = [], []
    node._publisher = SimpleNamespace(publish=messages.append)
    node._arbiter.shutdown = lambda: events.append("shutdown")
    node._publish_barrier = lambda: events.append("barrier")
    node.shutdown_motion()  # Actual shutdown uses Velocity(0, 0).
    assert events == ["shutdown", "barrier"]
    assert (messages[0].linear.x, messages[0].angular.z) == (0.0, 0.0)
    assert node._generation == 2 and not node._active
    node._publish(Velocity(1, -1))
    assert (messages[1].linear.x, messages[1].angular.z) == (1.0, -1.0)
