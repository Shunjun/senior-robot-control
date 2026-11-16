"""Captured packet, real core/controller: one fresh, same-packet STOP at exit."""

import json

import pytest
from test_motion_core_exit_deadline import delayed_decision, start
from test_motion_core_exit_deadline import rig as exit_rig

from senior_control.models import Velocity


@pytest.fixture
def rig(monkeypatch):
    yield from exit_rig.__wrapped__(monkeypatch)


def command_state(controller):
    return (
        vars(controller.trajectory).copy(),
        controller.integral,
        controller._direction,
        controller._brake_deadline,
    )


def stage(rig, monkeypatch):
    rig.clock.ns = 5_110_000_000
    rig.core._reference_clock = None  # Independent setup clock, before any control tick.
    start(rig, 0.5)
    rig.clock.steady += 0.01
    return delayed_decision(rig, monkeypatch, lambda d: d.evidence_deadline)


def test_fallback_rebuilds_current_ros_risk_without_readmitting_or_committing_drive(
    rig, monkeypatch
):
    decisions = stage(rig, monkeypatch)
    before = command_state(rig.core.controller)
    inputs = []
    original = rig.core.controller.prepare_stop
    admitted = []
    admit = rig.core.controller._admit_feedback

    def observe_admit(*args):
        admitted.append(args[0])
        return admit(*args)

    monkeypatch.setattr(rig.core.controller, "_admit_feedback", observe_admit)

    def stop(prepared, ctx, body, wheel, now):
        assert command_state(rig.core.controller) == before
        assert ctx is not prepared.context
        assert ctx.motion.computed_steady > prepared.context.motion.computed_steady
        assert rig.core.controller._same_feedback_evidence(body, prepared.body)
        assert rig.core.controller._same_feedback_evidence(wheel, prepared.wheel)
        result = original(prepared, ctx, body, wheel, now)
        inputs.append(result)
        return result

    monkeypatch.setattr(rig.core.controller, "prepare_stop", stop)
    base = rig.core._stop_fallback

    def fresh(*args):
        rig.clock.ns = 5_140_000_000
        return base(*args)

    monkeypatch.setattr(rig.core, "_stop_fallback", fresh)
    rig.core.tick()
    state = json.loads(rig.node.messages[-1].data)
    assert state["healthy"] and state["fault_revision"] == 0
    assert len(inputs) == 1 and inputs[0].fallback and inputs[0].braking
    assert admitted == ["body", "wheel", "imu"]
    assert rig.core.controller.trajectory.velocity == inputs[0].decision.command
    assert decisions[0].evidence_deadline < inputs[0].decision.evidence_deadline
    assert rig.core.target.source == "navigation"


@pytest.mark.parametrize("invalid", ["source_expired", "steady_source"])
def test_fallback_never_salvages_invalid_or_different_packet(rig, monkeypatch, invalid):
    stage(rig, monkeypatch)
    before = command_state(rig.core.controller)
    original = rig.core._stop_fallback
    attempted = []

    def stop(prepared, now):
        attempted.append(now)
        if invalid == "source_expired":
            now = prepared.body.deadline_steady
            rig.core.node._reception_test.publish(6.150000001, now)
        elif invalid == "packet_changed":
            raise AssertionError("retired local packet replay")
        else:
            now = prepared.body.deadline_steady
        return original(prepared, now)

    monkeypatch.setattr(rig.core, "_stop_fallback", stop)
    rig.core.tick()
    assert len(attempted) == 1
    assert not json.loads(rig.node.messages[-1].data)["healthy"]
    assert rig.node.output[-1][0] == Velocity(0, 0)
    assert command_state(rig.core.controller) == before
    assert rig.core.controller._prepared is None


def test_stop_expiring_during_fallback_is_rejected_once_without_commit(rig, monkeypatch):
    stage(rig, monkeypatch)
    before = command_state(rig.core.controller)
    original = rig.core.controller.prepare_stop
    stops = []

    def stop(*args):
        result = original(*args)
        stops.append(result)
        rig.clock.steady = result.decision.evidence_deadline
        return result

    monkeypatch.setattr(rig.core.controller, "prepare_stop", stop)
    rig.core.tick()
    assert len(stops) == 1 and stops[0].decision.verified
    assert not json.loads(rig.node.messages[-1].data)["healthy"]
    assert rig.node.output[-1][0] == Velocity(0, 0)
    assert command_state(rig.core.controller) == before
    with pytest.raises(ValueError):
        rig.core.controller.commit(stops[0])


def test_failed_actual_output_never_commits_proposed_command(rig, monkeypatch):
    rig.clock.ns = 5_110_000_000
    rig.core._reference_clock = None  # Independent setup clock, before any control tick.
    start(rig, 0.5)
    rig.clock.steady += 0.01
    before = command_state(rig.core.controller)
    emit = rig.node._emit_motion_core
    calls = []

    def failed_output(command, reason, deadline=0, *, computed_steady):
        calls.append(command)
        if len(calls) == 1:
            raise ValueError("captured actuator write failure")
        return emit(command, reason, deadline, computed_steady=computed_steady)

    monkeypatch.setattr(rig.node, "_emit_motion_core", failed_output)
    rig.core.tick()
    assert calls[0] != Velocity(0, 0) and calls[1] == Velocity(0, 0)
    assert command_state(rig.core.controller) == before
    assert rig.core.controller._prepared is None
    state = json.loads(rig.node.messages[-1].data)
    assert not state["healthy"] and state["reason"] == "captured actuator write failure"
