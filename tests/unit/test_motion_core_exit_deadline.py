"""Real controller/T4; delay only the actual output time after its decision."""

import json
from types import SimpleNamespace as N

import pytest
from test_reference_nanosecond_boundary import replay as boundary_fixture

from senior_control.models import Velocity


@pytest.fixture
def rig(monkeypatch):
    fixture = boundary_fixture.__wrapped__(monkeypatch)
    value = next(fixture)
    try:
        yield value
    finally:
        value.core.executor.shutdown(wait=True)
        try:
            next(fixture)
        except StopIteration:
            pass


def start(rig, target_span):
    core = rig.core
    core.tick()  # Actual captured body and full scan establish health.
    rig.clock.steady += 0.01
    target_deadline = rig.clock.steady + target_span
    assert core.command(
        N(
            close_reason="intent",
            source_kind="navigation",
            source_id="route",
            source_epoch=1,
            owner_boot_id="command-owner",
            generation=2,
            sequence=1,
            received_steady=rig.clock.steady,
            steady_deadline=target_deadline,
            vx=0.01,
            wz=0.01,
        )
    )
    core.tick()
    assert core.scheduler._flight is not None
    core.scheduler._flight.future.result(timeout=2)
    rig.clock.steady += 0.01
    core.tick()
    assert rig.node.output[-1][0] != Velocity(0, 0)
    return target_deadline


def delayed_decision(rig, monkeypatch, delay):
    original = rig.core.controller.tick
    captured = []

    def tick(*args, **kwargs):
        decision = original(*args, **kwargs)
        assert decision.verified and decision.command != Velocity(0, 0)
        captured.append(decision)
        rig.clock.steady = delay(decision)
        return decision

    monkeypatch.setattr(rig.core.controller, "tick", tick)
    return captured


@pytest.mark.parametrize("delta", [-0.000001, 0.0, 0.000001])
def test_actual_exit_must_precede_original_aggregate_deadline(rig, monkeypatch, delta):
    start(rig, 0.5)
    rig.clock.steady += 0.01
    sampled = rig.clock.steady
    decisions = delayed_decision(rig, monkeypatch, lambda d: d.evidence_deadline + delta)
    rig.core.tick()
    state = json.loads(rig.node.messages[-1].data)
    laser = next(row for row in rig.core._reference["sources"] if row["source"] == "laser")
    assert state["body_reference"]["computed_steady"] == laser["sample"]["received_steady"]
    assert state["body_reference"]["valid_until_steady"] == laser["valid_until_steady"]
    assert state["sampled_steady"] == sampled
    assert state["valid_until"] == sampled + 0.15
    assert state["fault_revision"] == 0
    if delta < 0:
        assert rig.node.output[-1][0] == decisions[0].command
        assert state["healthy"]
    else:
        assert rig.node.output[-1][0] == Velocity(0, 0)
        assert not state["healthy"]
        assert state["mode"] == "feedback_stop"
        assert not rig.core._last_healthy
        assert rig.core.target.source == "idle"


@pytest.mark.parametrize("limiting", ["target", "permit"])
def test_running_target_and_permit_deadline_are_not_renewed_at_exit(rig, monkeypatch, limiting):
    target_deadline = start(rig, 0.1 if limiting == "target" else 0.5)
    rig.clock.steady += 0.01
    original = rig.core.scheduler.update
    permissions = []

    def update(*args, **kwargs):
        result = original(*args, **kwargs)
        assert result.permit is not None
        permissions.append(result.permit)
        return result

    monkeypatch.setattr(rig.core.scheduler, "update", update)
    decisions = delayed_decision(rig, monkeypatch, lambda d: d.evidence_deadline)
    rig.core.tick()
    assert decisions[0].reason == "tracking"
    if limiting == "target":
        assert decisions[0].evidence_deadline == target_deadline
    else:
        assert decisions[0].evidence_deadline == permissions[0].deadline_steady < target_deadline
    assert rig.node.output[-1][0] == Velocity(0, 0)
    assert not json.loads(rig.node.messages[-1].data)["healthy"]


@pytest.mark.parametrize("expired", [False, True])
def test_release_braking_uses_its_evidence_not_revoked_target_deadline(rig, monkeypatch, expired):
    old_target_deadline = start(rig, 0.1)
    rig.clock.steady += 0.01
    rig.core.revoke("release", now=rig.clock.steady)
    assert rig.core.target.source == "idle"
    proofs = []
    original_proof = rig.core.engine.braking_proof

    def braking_proof(*args, **kwargs):
        proof = original_proof(*args, **kwargs)
        proofs.append(proof)
        return proof

    monkeypatch.setattr(rig.core.engine, "braking_proof", braking_proof)
    decisions = delayed_decision(
        rig,
        monkeypatch,
        lambda d: d.evidence_deadline if expired else old_target_deadline + 0.001,
    )
    rig.core.tick()
    decision = decisions[0]
    assert len(proofs) == 1 and proofs[0].brake.stop_verified
    assert decision.evidence_deadline > old_target_deadline + 0.001
    state = json.loads(rig.node.messages[-1].data)
    if expired:
        assert rig.node.output[-1][0] == Velocity(0, 0)
        assert not state["healthy"]
        assert state["mode"] == "feedback_stop" and state["fault_revision"] == 0
    else:
        assert rig.node.output[-1][0] == decision.command
        assert state["healthy"]
