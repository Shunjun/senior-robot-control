"""Sample time is stable; reducer events follow actual post-fallback time."""

import json
from dataclasses import replace

import pytest
from test_motion_core_exit_deadline import delayed_decision, start
from test_motion_core_stop_fallback import rig as stop_rig
from test_motion_core_stop_fallback import stage

from senior_control.motion.contracts import MotionEventKind as K


@pytest.fixture
def rig(monkeypatch):
    yield from stop_rig.__wrapped__(monkeypatch)


def test_physical_stop_fault_then_exit_expiry_applies_feedback_loss_without_clock_fault(
    rig, monkeypatch
):
    stage(rig, monkeypatch)
    sampled = rig.clock.steady
    rig.core.controller._brake_deadline = sampled + 0.000001
    stop = rig.core.controller.prepare_stop
    decisions = []

    def expire(*args):
        prepared = stop(*args)
        decisions.append(prepared.decision)
        assert prepared.decision.reason == "stop_response_failed"
        rig.clock.steady = prepared.decision.evidence_deadline
        return prepared

    monkeypatch.setattr(rig.core.controller, "prepare_stop", expire)
    events = []
    reduce = rig.core.machine.reduce

    def record(event, *, now_steady):
        result = reduce(event, now_steady=now_steady)
        events.append((event.kind, now_steady, result.reason))
        return result

    monkeypatch.setattr(rig.core.machine, "reduce", record)
    rig.core.tick()
    state = json.loads(rig.node.messages[-1].data)
    assert decisions and state["fault_revision"] == 1
    assert rig.core.machine._feedback_lost
    assert not any(reason == "clock_fault" for _, _, reason in events)
    assert events[-1][0] is K.FEEDBACK_LOST
    assert events[-1][1] == rig.clock.steady
    assert state["sampled_steady"] == sampled
    assert "valid_until" not in state


def test_unified_feedback_does_not_repeat_pair_qualification_at_output(rig, monkeypatch):
    rig.clock.ns = 5_110_000_000
    rig.core._reference_clock = None
    start(rig, 0.5)
    rig.clock.steady += 0.01
    # Controlled limited-source fixture: genuine captured wheel/IMU, no body.
    # Synthetic bounded risk isolates event ordering from unknown-body geometry.
    rig.core.scheduler.stop()  # Fence the body-domain setup flight.
    rig.core.scheduler._flight.future.result(timeout=2)
    rig.core.sensors.estimator.invalidate("laser")
    snapshot = rig.core._snapshot

    def bounded(*args, visibility=None):
        ctx = snapshot(*args, visibility=visibility)
        return replace(
            ctx,
            envelope=replace(
                ctx.envelope, linear_interval=(-0.05, 0.05), yaw_interval=(-0.01, 0.01)
            ),
        )

    monkeypatch.setattr(rig.core, "_snapshot", bounded)
    rig.core.tick()
    rig.core.scheduler._flight.future.result(timeout=2)
    rig.clock.steady += 0.01
    proposals = delayed_decision(rig, monkeypatch, lambda d: d.evidence_deadline)
    stop = rig.core.controller.prepare_stop
    fallbacks = []

    def finish(*args):
        result = stop(*args)
        fallbacks.append(result)
        rig.clock.steady += 0.001
        return result

    monkeypatch.setattr(rig.core.controller, "prepare_stop", finish)
    seen = []
    observe = rig.core.machine.observe

    def observation(value, now):
        seen.append(("observe", now))
        return observe(value, now)

    event = rig.core._event

    def record(kind, now, *args):
        if kind is K.FEEDBACK_HEALTHY:
            seen.append(("healthy", now))
        return event(kind, now, *args)

    monkeypatch.setattr(rig.core.machine, "observe", observation)
    monkeypatch.setattr(rig.core, "_event", record)
    sampled = rig.clock.steady
    rig.core.tick()
    state = json.loads(rig.node.messages[-1].data)
    assert state["healthy"] and state["fault_revision"] == 0, (
        state["reason"],
        [p.reason for p in proposals],
    )
    assert proposals[0].reason == "tracking"
    assert len(fallbacks) == 1 and fallbacks[0].braking and fallbacks[0].fallback
    assert [name for name, _ in seen] == ["observe", "healthy"]
    assert all(now == sampled for _, now in seen)
    assert rig.clock.steady > sampled  # Actual output deadline is checked after fallback.
    assert state["sampled_steady"] == sampled


@pytest.mark.parametrize("rollback_at", ["t1", "t2"])
def test_real_sampled_clock_rollback_rejects_without_clamping(rig, monkeypatch, rollback_at):
    rig.clock.ns = 5_110_000_000
    rig.core._reference_clock = None
    start(rig, 0.5)
    rig.clock.steady += 0.01
    sampled = rig.clock.steady
    delayed_decision(
        rig,
        monkeypatch,
        lambda d: sampled - 0.000001 if rollback_at == "t1" else d.evidence_deadline,
    )
    if rollback_at == "t2":
        stop = rig.core.controller.prepare_stop

        def backwards(*args):
            result = stop(*args)
            rig.clock.steady -= 0.000001
            return result

        monkeypatch.setattr(rig.core.controller, "prepare_stop", backwards)
    rig.core.tick()
    state = json.loads(rig.node.messages[-1].data)
    assert not state["healthy"] and state["reason"] == "motion_clock_regressed"
    assert rig.node.output[-1][0].vx == rig.node.output[-1][0].wz == 0
    assert rig.core.controller._prepared is None
    assert state["sampled_steady"] == sampled
    assert "valid_until" not in state
    if rollback_at == "t1":
        # The regressed instant still reaches the unmodified reducer guard.
        assert state["fault_revision"] == 1
        assert rig.core.machine.state.reason == "clock_fault"
