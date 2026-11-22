"""Real controller/T4; delay only the actual output time after its decision."""

from dataclasses import replace
from types import SimpleNamespace as N

import pytest
from test_reference_nanosecond_boundary import replay as boundary_fixture

from vehicle_core.values import Velocity


@pytest.fixture
def rig(monkeypatch):
    fixture = boundary_fixture.__wrapped__(monkeypatch)
    value = next(fixture)
    # Explicit synthetic open scan isolates output/identity tests from geometry.
    import math

    raw = value.node._scans[-1]
    raw.predictions = ()
    raw.message.ranges = [math.inf, math.inf]
    raw.message.angle_min = 0.0
    raw.message.angle_increment = math.pi
    raw.message.range_min, raw.message.range_max = 0.05, 10.0
    raw.message.sensor_x = raw.message.sensor_y = raw.message.sensor_yaw = 0.0
    try:
        yield value
    finally:
        try:
            next(fixture)
        except StopIteration:
            pass


def start(rig, target_span):
    core = rig.core
    core.tick()  # Captured raw motion and explicitly synthetic open geometry.
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
            manual_linear=0.0,
            manual_linear_limit=0.0,
            degraded_linear_limit=0.0,
            vx=0.01,
            wz=0.01,
        )
    )
    core.tick()
    rig.clock.steady += 0.01
    core.tick()
    assert rig.node.output[-1][0] != Velocity(0, 0)
    return target_deadline


def test_output_deadline_keeps_source_bound_and_local_lease(rig, monkeypatch):
    target_deadline = start(rig, 0.1)
    emitted = []
    original = rig.node._emit_motion_core

    def capture(velocity, reason, deadline, *, computed_steady):
        emitted.append((velocity, deadline, computed_steady))
        return original(velocity, reason, deadline, computed_steady=computed_steady)

    monkeypatch.setattr(rig.node, "_emit_motion_core", capture)
    rig.core.tick()
    velocity, deadline, computed = emitted[-1]
    assert velocity != Velocity(0, 0)
    assert deadline == target_deadline
    assert deadline <= computed + 0.25
    rig.core.target = replace(rig.core.target, deadline_steady=computed + 10)
    rig.core.tick()
    assert emitted[-1][1] == emitted[-1][2] + 0.25


def test_expired_target_emits_zero_without_replaying_source(rig):
    deadline = start(rig, 0.1)
    rig.clock.steady = deadline
    rig.core.tick()
    assert rig.node.output[-1][0] == Velocity(0, 0)
    assert rig.core.target.source == "idle"
    rig.clock.steady += 0.01
    rig.core.tick()
    assert rig.node.output[-1][0] == Velocity(0, 0)
    assert rig.core.machine.state.fault_revision == 0
