# ruff: noqa: F811
"""No ROS process: actual output owner and runtime receiver with fake ROS I/O."""

import json
from pathlib import Path
from types import SimpleNamespace as N

import pytest
from control_process_test_support import generic_motion_profile
from pydantic import ValidationError
from senior_robot.motion_core_output import MotionCoreOutput
from test_motion_core_handoff import Node, barrier, core  # noqa: F401
from test_navigation_command_clock import navigation  # noqa: F401
from test_runtime_guard_ready import CASE, ready  # noqa: F401

from senior_control.models import Velocity
from senior_control.motion.contracts import MotionMode
from senior_control.protocol import MotionRuntimeState


@pytest.mark.parametrize("old_state", [None, '{"latched":true}', "broken", "unreadable"])
def test_new_output_owner_ignores_old_files_and_never_restores_motion(
    core, tmp_path, monkeypatch, old_state
):
    path = tmp_path / "estop-state.json"
    if old_state is not None:
        path.write_text(old_state)
    if old_state == "unreadable":
        path.chmod(0)

    # File access itself is forbidden: no parse/fallback can manufacture a latch.
    def forbidden(*args, **kwargs):
        raise AssertionError("software estop startup must not access a state file")

    profile = generic_motion_profile()
    with monkeypatch.context() as patch:
        patch.setattr(Path, "open", forbidden)
        patch.setattr("builtins.open", forbidden)
        fresh = MotionCoreOutput(Node(), profile)
        try:
            assert fresh.machine.state.mode is MotionMode.IDLE
            assert fresh.machine.state.estop_revision == 0
            assert fresh.target.source == "idle" and fresh.target.velocity == Velocity(0, 0)
            assert fresh.node.emitted == []
        finally:
            fresh.close()
    if old_state is not None:
        path.chmod(0o600)
        assert path.read_text() == old_state
    else:
        assert not path.exists()
    assert not path.with_suffix(".json.lock").exists()


def test_software_request_is_only_latch_source_and_duplicate_is_idempotent(core):
    assert not hasattr(core, "_hardware_subscription")
    assert not hasattr(core, "_on_hardware_estop")
    message = barrier("estop", request="explicit")
    assert core.command(message)
    revision = core.machine.state.estop_revision
    assert core.machine.state.mode is MotionMode.ESTOP
    assert core.command(message)
    assert core.machine.state.estop_revision == revision
    assert core.target.source == "idle"


@pytest.mark.parametrize("status", ["hardware_latched", "clear"])
def test_retired_hardware_packets_are_not_software_state_or_permission(ready, status):
    packet = dict(CASE["packets"][1], confirmed_estop_status=status)
    if status == "clear":
        packet["hardware_estop_observation"] = "clear"
    with pytest.raises(ValidationError):
        MotionRuntimeState.model_validate(packet)
    previous = ready.node._motion_runtime.value
    ready.node._on_motion_runtime(N(data=json.dumps(packet)), {})
    assert ready.node._motion_runtime.value is previous
    assert ready.node.task_motion_allowed(continuing=True)
    assert not hasattr(ready.node._arbiter, "confirm_local_estop")


def test_expired_or_old_estop_report_cannot_relock_after_owner_clear(ready):
    ready.clock.mono = CASE["packets"][1]["sampled_steady"] + 0.001
    packet = dict(CASE["packets"][1], estop_revision=2)
    ready.node._on_motion_runtime(N(data=json.dumps(packet)), {})
    prior = ready.node._motion_runtime.value
    old = dict(packet, sequence=198, mode="estop", confirmed_estop_status="software_latched")
    ready.node._on_motion_runtime(N(data=json.dumps(old)), {})
    expired = dict(old, sequence=200, estop_revision=1)
    ready.node._on_motion_runtime(N(data=json.dumps(expired)), {})
    assert ready.node._motion_runtime.value is prior
    assert prior.confirmed_estop_status == "clear"
    assert not any(frame.close_reason == "estop" for frame in ready.barriers)


def test_raw_wheel_imu_reset_without_independent_body_never_restores_old_target(core):
    from test_unified_motion_runtime import point

    from senior_control.motion.observations import Source

    receiver = core.node._reception_test
    receiver.sensors.ingest(point(Source.WHEEL, 10, 10, (0.015, 0.02)), 10, 10, 10)
    receiver.sensors.ingest(point(Source.IMU, 10, 10, (0.015,)), 10, 10, 10)
    receiver.publish(10, 10)
    assert core.sensors.independent is None and core.sensors.stationary(10)
    assert core.command(barrier("estop", generation=2, request="raw-only-estop"))
    reset = barrier("estop_reset", generation=3, request="raw-only-reset")
    reset.gesture = core.machine.state.estop_revision
    assert core.command(reset)
    assert core.machine.state.mode is MotionMode.IDLE
    assert core.controller.trajectory.velocity == Velocity(0, 0)
    assert core.target.source == "idle"
    old = barrier("intent", generation=2, request="retired-intent")
    old.sequence, old.steady_deadline = 1, 10.3
    old.vx, old.manual_linear, old.manual_linear_limit, old.degraded_linear_limit = 0.2, 1, 0.2, 0.3
    assert not core.command(old)
    assert core.target.source == "idle" and core.node.emitted == []
