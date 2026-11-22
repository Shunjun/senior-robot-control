"""Real current core estop semantics, without retired qualification workers."""

from test_motion_core_handoff import barrier
from test_startup_learning_chain import rig  # noqa: F401

from vehicle_core.motion_state import MotionMode
from vehicle_core.values import Velocity


def test_estop_before_output_never_consumes_or_restarts_learning(rig):  # noqa: F811
    assert rig.core.command(barrier("estop", request="operator-stop"))
    for i in range(8):
        assert rig.step(10.05 + i * 0.05) == Velocity(0, 0)
    assert rig.core.machine.state.mode is MotionMode.ESTOP
    assert not rig.run.owner.record.action.consumed
    assert rig.run.owner.record.phase == "failed"
    rig.core = rig.new_core()
    assert rig.step(11.0) == Velocity(0, 0)
    assert not rig.core.startup.prepare_action()


def test_withdrawn_started_move_keeps_deadline_and_waits_for_post_window_frames(rig):  # noqa: F811
    start = rig.start()
    rig.step(start + 0.5, wheel=0.05)
    rig.core.startup.hold_motion("scan_temporarily_unavailable")
    held = rig.now
    for _ in range(10):
        assert rig.step(rig.now + 0.05) == Velocity(0, 0)
    assert rig.run.owner.record.capture_summary.state == "open"
    assert rig.run.owner.record.model is None
    assert rig.core.startup.deadline == start + 3
    while rig.now + 0.05 < start + 3:
        assert rig.step(rig.now + 0.05) == Velocity(0, 0)
    rig.step(start + 3)
    for _ in range(10):
        rig.step(rig.now + 0.05)
    record = rig.run.owner.record
    assert record.action.stage == "window_finished"
    assert 0 < record.action.reported_nonzero_interval_ns < 3_000_000_000
    assert record.capture_summary.state == "complete"
    assert all(value.vx == 0 for time, value, *_ in rig.core.node.emitted if time > held)
