"""Execute the actual timer callback with controlled reception/prefetch clocks."""

import ast
from pathlib import Path
from types import SimpleNamespace as N

import pytest

from senior_control.models import Velocity
from senior_control.motion.cadence import ControlCadence


@pytest.mark.parametrize("work", ["input", "prefetch", "tick"])
def test_gate_observes_work_time_and_never_catches_up_missed_slot(work):
    path = Path("ros2/src/senior_robot/senior_robot/safety_node.py")
    cls = next(
        n
        for n in ast.parse(path.read_text()).body
        if isinstance(n, ast.ClassDef) and n.name == "SafetyGate"
    )
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == "_on_timer")
    clock = N(now=0.0)
    scope = {"time": N(monotonic=lambda: clock.now), "Velocity": Velocity}
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), "exec"), scope)
    ticks, zeros = [], []
    delayed = False

    def consume(kind):
        nonlocal delayed
        if kind == "tick":
            ticks.append(clock.now)
        if kind == work and not delayed:
            clock.now += 0.080
            delayed = True

    core = N(
        sensors=N(),
        prefetch_scan=lambda: consume("prefetch"),
        tick=lambda: consume("tick"),
        revoke=lambda reason: zeros.append(reason),
    )
    gate = N(
        _cadence=ControlCadence(),
        _motion_core=core,
        _input_process=N(update=lambda *_: consume("input")),
        _poll_command_guard=lambda: None,
        _command_topology_ready=lambda: True,
        _emit_motion_core=lambda *_: zeros.append("emit"),
    )
    callback = scope["_on_timer"]
    callback(gate)
    assert ticks == ([0.0] if work == "tick" else [0.080])
    callback(gate)
    assert len(ticks) == 1
    clock.now = 0.099
    callback(gate)
    assert len(ticks) == 1
    clock.now = 0.100
    callback(gate)
    assert len(ticks) == 2 and ticks[-1] == 0.100
    assert not zeros
