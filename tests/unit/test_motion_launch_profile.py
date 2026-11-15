"""Exercise the actual launch profile resolver without starting ROS processes."""

import ast
from pathlib import Path
from types import SimpleNamespace as N

import pytest


@pytest.mark.parametrize("candidate", ["fortress-candidate-unvalidated-v1"])
def test_standard_simulation_uses_shared_core_profile(candidate):
    p = (
        Path(__file__).resolve().parents[2]
        / "ros2/src/senior_robot/launch/simulation_motion.launch.py"
    )
    outer = next(
        x
        for x in ast.parse(p.read_text()).body
        if isinstance(x, ast.FunctionDef) and x.name == "generate_launch_description"
    )
    profile = next(x for x in outer.body if isinstance(x, ast.FunctionDef) and x.name == "profile")

    def lc(name):
        return N(perform=lambda context: context.get(name, ""))

    scope = {
        "LaunchConfiguration": lc,
        "IfCondition": lambda value: N(evaluate=lambda context: context["qualified_reference"]),
        "SetLaunchConfiguration": lambda name, value: (name, value),
        "PathJoinSubstitution": lambda values: "/".join(values),
        "sim": "sim",
        "robot": "robot",
        "qualified": lc("qualified_reference"),
    }
    exec(compile(ast.Module(body=[profile], type_ignores=[]), str(p), "exec"), scope)
    values = dict(
        scope["profile"](
            {
                "qualified_reference": False,
                "motion_core_profile": candidate,
                "motion_core_world": "/world/straight.sdf" if candidate else "",
            }
        )
    )
    assert values["core_matcher_linear"] == "1.52"
    assert values["core_matcher_yaw"] == "2.0"
    if candidate:
        assert values["motion_world"] == "/world/straight.sdf"

    text = p.read_text()
    assert 'default_value="fortress-candidate-unvalidated-v1"' in text
    assert '"rear_attachment": "false"' in text
    assert "safety_overlay" not in text
