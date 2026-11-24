"""Real T5 owner and T4 engine; synthetic point sensors and ROS I/O only."""

import json

import pytest
import test_motion_core_handoff as handoff
from senior_motion_control.contracts import MotionEventKind as K
from senior_motion_control.contracts import MotionTarget
from sensor_time_test_support import sample_time
from test_motion_core_handoff import barrier

from vehicle_core.values import Velocity


@pytest.fixture
def core(monkeypatch):
    yield from handoff.core.__wrapped__(monkeypatch)


def setup_stop(c, monkeypatch):
    import math
    from types import SimpleNamespace

    from senior_perception.raw_safety_observation import RawSafetyObservation

    clock = [10.0]
    monkeypatch.setattr("senior_motion_control.motion_core_output.time.monotonic", lambda: clock[0])
    c._event(K.MANUAL_START, 10.0, "authority")
    c.target = MotionTarget(Velocity(0.277, 0), "manual", "authority:42", 10.2)
    c._output = Velocity(0.277, 0)
    assert c.command(barrier())

    def tick():
        now = clock[0]
        scan = SimpleNamespace(
            ranges=[math.inf, math.inf],
            angle_min=0.0,
            angle_increment=math.pi,
            range_min=0.05,
            range_max=10.0,
            sensor_x=0.0,
            sensor_y=0.0,
            sensor_yaw=0.0,
        )
        c.node._scans.clear()
        c.node._scans.append(RawSafetyObservation(scan, now, now, now, now))
        c.tick()
        result = json.loads(c.node.publishers["/motion/runtime"].messages[-1].data)
        assert result["body_reference"] is None
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
    m.manual_linear = vx / (5 / 9)
    m.manual_linear_limit = 5 / 9
    m.degraded_linear_limit = 0.3
    return m


def test_release_rearm_prepared_neutral_new_gesture_and_second_release(core, monkeypatch):
    clock, tick = setup_stop(core, monkeypatch)
    for _ in range(170):
        r = tick()
    assert r["mode"] == "idle" and r["allowed_actions"] == ["acquire"]
    prepared = barrier("prepared", generation=3)
    prepared.source_id = "new-authority"
    assert core.command(prepared)
    tick()
    assert core.command(intent(clock, "new-authority", 1, 0.0))
    tick()
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
    assert r["mode"] == "idle" and r["allowed_actions"] == ["acquire"]
    assert core.node.emitted[-1][0] == Velocity(0, 0)


@pytest.mark.parametrize("change", ["expire", "epoch"])
def test_first_motion_rechecks_evidence_after_neutral(core, monkeypatch, change):
    clock, tick = setup_stop(core, monkeypatch)
    for _ in range(170):
        tick()
    assert core.command(intent(clock, "new-authority", 1, 0.0))
    tick()
    if change == "expire":
        clock[0] += 1.0
    else:
        from senior_motion_estimation.estimator import MotionObservation, Origin

        now = clock[0]
        core.sensors.ingest_observation(
            MotionObservation(
                "imu",
                (None, None, 0.0),
                now,
                (
                    Origin(
                        "imu",
                        "replacement",
                        1,
                        sample_time(
                            stamp_ns=round((now) * 1e9),
                            received_ros_ns=round((now) * 1e9),
                            received_steady_ns=round((now) * 1e9),
                        ),
                    ),
                ),
                measured_steady=now,
                stamp_ns=round((now) * 1e9),
            ),
            now,
            now,
        )
    # Command entry does not repeat report/feedback qualification; the next
    # control calculation owns expired input evidence and must hold zero.
    assert core.command(intent(clock, "new-authority", 2, 5 / 9))
    assert core.node.emitted[-1][0] == Velocity(0, 0)
