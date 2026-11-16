"""Legal nonzero Nav2 intent must not turn into an idle output barrier."""

from types import SimpleNamespace as N

import pytest
from test_navigation_command_clock import navigation as clock_fixture

from senior_control.models import Velocity
from senior_control.safety.arbiter import Arbiter


@pytest.fixture
def rig(monkeypatch):
    import sys

    r = clock_fixture.__wrapped__(monkeypatch)
    node = r.node
    node._arbiter = Arbiter("robot", 300)
    assert node._arbiter.begin_navigation("private-execution", r.clock.mono, task_epoch=1)
    node._publish_manual_target = lambda: False
    node.get_parameter = lambda _: N(value="robot")
    node._owner_boot = "command-owner"
    node._drive_sample = None
    final, barriers = [], []
    node._final_publisher = N(publish=final.append)
    node._publish_barrier = lambda *args, **kwargs: barriers.append(args)
    node._publish = lambda *args: None
    node.publish_command = type(node).publish_command.__get__(node)
    monkeypatch.setattr(sys.modules["command_clock_under_test"], "FinalCommand", N)
    return N(r=r, final=final, barriers=barriers)


@pytest.mark.parametrize("vx,wz", [(0.0049, 0.0), (-0.0049, 0.0), (0.0, -0.0037), (0.0099, 0.0037)])
def test_subcentimeter_nonzero_intent_keeps_exact_twist_and_original_deadline(rig, vx, wz):
    r = rig.r
    r.msg.vx, r.msg.wz = vx, wz
    r.node._on_navigation_sample(r.topic, r.msg, r.info)
    assert len(rig.final) == 1 and not rig.barriers
    final = rig.final[0]
    assert Velocity(final.vx, final.wz) == Velocity(vx, wz)
    assert final.source_kind == "navigation" and final.sequence == 1
    assert final.steady_deadline == pytest.approx(10.0 + 0.3)


def test_small_nonzero_keeps_existing_supersedence_interval(rig):
    r = rig.r
    r.node._navigation_source.last_forwarded = r.clock.mono
    r.msg.vx = 0.0049
    r.node._on_navigation_sample(r.topic, r.msg, r.info)
    assert r.node._navigation_source.sequence == 0
    assert not rig.final and not rig.barriers


def test_exact_producer_zero_still_stops_immediately(rig):
    r = rig.r
    assert r.node._arbiter.accept_navigation_sample(
        "private-execution", 1, Velocity(0.1, 0), 10.0, 10.0
    )
    r.node._navigation_source.sequence = 1
    r.node._navigation_source.last_forwarded = r.clock.mono
    r.node._active = True
    r.msg.vx = r.msg.wz = 0.0
    r.node._on_navigation_sample(r.topic, r.msg, r.info)
    assert r.node._navigation_source.sequence == 2
    assert not rig.final and len(rig.barriers) == 1
    assert r.node._arbiter.final_command(r.clock.mono) is None
