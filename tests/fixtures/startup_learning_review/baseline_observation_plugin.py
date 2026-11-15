"""Temporary test-only replay of the reviewed baseline observation method."""

import ast
import subprocess


def pytest_configure(config):
    import senior_robot.motion_core_output as module

    source = subprocess.check_output(
        [
            "git",
            "show",
            "b64bca01dc31936d9b0de407b153bce7e8ec71d2:ros2/src/senior_robot/senior_robot/motion_core_output.py",
        ],
        text=True,
    )
    tree = ast.parse(source)
    cls = next(
        node
        for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == "MotionCoreOutput"
    )
    method = next(
        node
        for node in cls.body
        if isinstance(node, ast.FunctionDef) and node.name == "_observation"
    )
    tree = ast.Module(body=[method], type_ignores=[])
    exec(
        compile(ast.fix_missing_locations(tree), "<reviewed-baseline-observation>", "exec"),
        module.__dict__,
    )
    module.MotionCoreOutput._observation = module.__dict__["_observation"]
