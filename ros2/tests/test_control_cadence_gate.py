"""Execute the actual timer callback with controlled reception/prefetch clocks."""

import ast
from pathlib import Path
from types import SimpleNamespace as N

import pytest
from senior_motion_control.cadence import ControlCadence

from vehicle_core.values import Velocity


@pytest.mark.parametrize("work", ["input", "guard", "tick"])
def test_gate_observes_work_time_and_never_catches_up_missed_slot(work):
    path = Path("ros2/src/senior_motion_control/senior_motion_control/safety_node.py")
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
        _poll_command_guard=lambda: consume("guard"),
        _last_control_tick=None,
        _control_timing={},
        _output_transport=N(mark=lambda *_: None, trace_active=lambda: False, _trace_marks=[]),
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


@pytest.fixture
def output_rig(monkeypatch):
    from test_motion_core_exit_deadline import rig

    yield from rig.__wrapped__(monkeypatch)


def test_failed_actual_output_revokes_source_and_emits_zero(output_rig, monkeypatch):
    from test_motion_core_exit_deadline import start

    rig = output_rig
    start(rig, 0.5)
    emitted = []
    original = rig.node._emit_motion_core

    def failed_first(velocity, reason, deadline=0, *, computed_steady):
        emitted.append(velocity)
        if len(emitted) == 1:
            raise ValueError("actuator write failed")
        return original(velocity, reason, deadline, computed_steady=computed_steady)

    monkeypatch.setattr(rig.node, "_emit_motion_core", failed_first)
    path = Path("ros2/src/senior_motion_control/senior_motion_control/safety_node.py")
    cls = next(
        n
        for n in ast.parse(path.read_text()).body
        if isinstance(n, ast.ClassDef) and n.name == "SafetyGate"
    )
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == "_on_timer")
    scope = {"time": N(monotonic=lambda: rig.clock.steady), "Velocity": Velocity}
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), "exec"), scope)
    gate = N(
        _cadence=ControlCadence(),
        _motion_core=rig.core,
        _input_process=N(update=lambda *_: None),
        _poll_command_guard=lambda: None,
        _command_topology_ready=lambda: True,
        _last_control_tick=None,
        _control_timing={},
        _output_transport=N(mark=lambda *_: None, trace_active=lambda: False, _trace_marks=[]),
        _emit_motion_core=failed_first,
    )
    scope["_on_timer"](gate)
    assert len(emitted) == 2 and emitted[0] != Velocity(0, 0) and emitted[1] == Velocity(0, 0)
    assert rig.core._output == Velocity(0, 0)
    assert rig.core.target.source == "idle"
