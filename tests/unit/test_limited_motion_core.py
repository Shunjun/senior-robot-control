"""Real T5 owner and T4 engine; synthetic point sensors and ROS I/O only."""

import json
from concurrent.futures import Future
from dataclasses import replace

import pytest
import test_motion_core_handoff as handoff
from test_directional_constraints import scene
from test_motion_core_handoff import barrier

from senior_control.models import Pose2D, Velocity
from senior_control.motion.contracts import MotionEventKind as K
from senior_control.motion.contracts import MotionTarget
from senior_control.motion.state_machine import RecoveryObservation
from senior_control.motion.trajectory import Trajectory


@pytest.fixture
def core(monkeypatch):
    yield from handoff.core.__wrapped__(monkeypatch)


def setup_stop(c, monkeypatch):
    clock = [10.0]
    monkeypatch.setattr("senior_robot.motion_core_output.time.monotonic", lambda: clock[0])
    monkeypatch.setattr(
        c,
        "_observation",
        lambda now: (replace(scene(now), frame_id="scan_reference"), Pose2D(0, 0, 0)),
    )
    c.machine.observe(RecoveryObservation("former-body", 1, 10.0, 10.1, True, False), 10.0)
    c._event(K.MANUAL_START, 10.0, "authority")
    c.target = MotionTarget(Velocity(0.277, 0), "manual", "authority:42", 10.2)
    c.controller.trajectory = Trajectory(Velocity(0.277, 0), Velocity(0, 0))

    class InlineWorker:
        def submit(self, task):
            result = Future()
            result.set_result(task())
            return result

    c.scheduler.worker = InlineWorker()
    c.controller._last_now = 9.95
    assert c.command(barrier())

    def tick():
        now = clock[0]
        seq = round(now * 1000)
        wheel = c.node.emitted[-1][0] if c.node.emitted else Velocity(0.277, 0)
        sources = []
        for name, values in [("wheel", [wheel.vx, wheel.wz]), ("imu", [0.0])]:
            sources.append(
                dict(
                    source=name,
                    ready=True,
                    valid_until_steady=now + 0.1,
                    epoch=name,
                    sample=dict(
                        values=values,
                        epoch=name,
                        sequence=seq,
                        received_steady=now,
                        stamp_ros=now - 0.001,
                    ),
                )
            )
        c._reference = dict(
            sampled_steady=now,
            sources=sources,
            reference=dict(
                average=None,
                mode="unavailable",
                start_ros=now - 0.1,
                end_ros=now,
                source_health=sources,
            ),
        )
        c.tick()
        result = json.loads(c.node.publishers["/motion/runtime"].messages[-1].data)
        assert result["body_reference"] is None and not result["stationary"]
        clock[0] += 0.05
        return result

    return clock, tick


def intent(clock, authority, sequence, vx):
    m = barrier("intent", generation=3)
    m.source_id = authority
    m.gesture = 2
    m.request_id = f"unit-{authority}-{sequence}"
    m.sequence = sequence
    m.received_steady = clock[0]
    m.steady_deadline = clock[0] + 0.2
    m.vx = vx
    return m


def test_release_rearm_prepared_neutral_new_gesture_and_second_release(core, monkeypatch):
    clock, tick = setup_stop(core, monkeypatch)
    for _ in range(170):
        r = tick()
    assert r["mode"] == "braking" and r["allowed_actions"] == ["acquire"]
    deadline = core.controller._brake_deadline
    prepared = barrier("prepared", generation=3)
    prepared.source_id = "new-authority"
    assert core.command(prepared)
    tick()
    assert core.controller._brake_deadline == deadline
    assert core.command(intent(clock, "new-authority", 1, 0.0))
    tick()
    assert core.controller._brake_deadline == deadline
    assert core.command(intent(clock, "new-authority", 2, 5 / 9))
    r = tick()
    # A real refinement may need the next poll; keep sending fresh intents.
    for seq in range(3, 7):
        if core.node.emitted[-1][0].vx > 0:
            break
        assert core.command(intent(clock, "new-authority", seq, 5 / 9))
        r = tick()
    assert core.node.emitted[-1][0].vx > 0 and r["mode"] == "manual"
    for seq in range(7, 47):
        assert core.command(intent(clock, "new-authority", seq, 5 / 9))
        tick()
    stop = barrier("release", generation=4)
    stop.source_id = "new-authority"
    assert core.command(stop)
    for _ in range(170):
        r = tick()
    assert r["mode"] == "braking" and r["allowed_actions"] == ["acquire"]
    assert core.node.emitted[-1][0] == Velocity(0, 0)


@pytest.mark.parametrize("change", ["expire", "epoch"])
def test_first_motion_rechecks_evidence_after_neutral(core, monkeypatch, change):
    clock, tick = setup_stop(core, monkeypatch)
    for _ in range(170):
        tick()
    assert core.command(intent(clock, "new-authority", 1, 0.0))
    tick()
    if change == "expire":
        clock[0] += 0.2
    else:
        row = core._reference["sources"][1]
        row["epoch"] = row["sample"]["epoch"] = "replacement"
    assert not core.command(intent(clock, "new-authority", 2, 5 / 9))
    assert core.node.emitted[-1][0] == Velocity(0, 0)
