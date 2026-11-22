"""Navigation target reaches the real core, async constraints and final output seam."""

import pytest
from test_motion_core_exit_deadline import rig as core_fixture
from test_task_terminal_release import intent

from vehicle_core.values import Velocity


@pytest.fixture
def rig(monkeypatch):
    yield from core_fixture.__wrapped__(monkeypatch)


@pytest.mark.parametrize("vx,wz", [(8.0, 0.0), (0.0, 4.0), (8.0, 4.0), (-8.0, -4.0)])
def test_navigation_overlimits_reach_real_core_output_without_downstream_rejection(rig, vx, wz):
    core = rig.core
    # Capture real raw motion and the fixture's explicitly synthetic open scene.
    core.tick()
    rig.clock.steady += 0.01
    msg = intent(rig, "route", epoch=1, generation=2)
    msg.vx, msg.wz = vx, wz
    assert core.command(msg)
    rig.clock.steady += 0.01
    core.tick()
    if core.scheduler._flight is not None:
        core.scheduler._flight.future.result(timeout=2)
    rig.clock.steady += 0.01
    core.tick()
    output, _reason = rig.node.output[-1]
    assert output != Velocity(0, 0), _reason
    assert output.vx * vx >= 0 and output.wz * wz >= 0
    assert abs(output.vx) <= core.profile.cap
    assert abs(output.wz) <= core.profile.yaw_cap
    assert core.target.velocity == Velocity(vx, wz)
    assert core.target.deadline_steady == msg.steady_deadline
