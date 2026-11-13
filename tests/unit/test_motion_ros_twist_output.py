"""ROS float setters must also accept an internal integer-valued zero command."""

import ast
from pathlib import Path
from types import SimpleNamespace as N

import pytest

from senior_control.models import Velocity


class Axis:
    def __setattr__(self, name, value):
        assert isinstance(value, float), "ROS Vector3 requires float"
        super().__setattr__(name, value)


class Twist:
    def __init__(self):
        self.linear, self.angular = Axis(), Axis()


@pytest.mark.parametrize("velocity", [Velocity(0, 0), Velocity(0.2, -0.3)])
def test_actual_core_emitter_serializes_zero_and_tracking_as_ros_floats(velocity):
    p = Path(__file__).resolve().parents[2] / "ros2/src/senior_robot/senior_robot/safety_node.py"
    node = next(
        x
        for x in ast.parse(p.read_text()).body
        if isinstance(x, ast.ClassDef) and x.name == "SafetyGate"
    )
    method = next(
        x for x in node.body if isinstance(x, ast.FunctionDef) and x.name == "_emit_motion_core"
    )
    scope = {"Twist": Twist, "Velocity": Velocity, "time": N(monotonic=lambda: 10.0)}
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(p), "exec"), scope)
    outputs = []
    gate = N(
        _publisher=N(publish=outputs.append),
        _publish_drive_feedback=lambda _: None,
        _feedback=N(published=lambda *_: None),
        _report=lambda _: None,
    )
    scope["_emit_motion_core"](gate, velocity, "test")
    assert len(outputs) == 1
    assert (outputs[0].linear.x, outputs[0].angular.z) == (velocity.vx, velocity.wz)
