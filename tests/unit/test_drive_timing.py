import pytest

from senior_control.drive_timing import ClockWindow, DelayBudget


def test_asymmetric_exchange_preserves_bound_and_expires():
    clock = ClockWindow()
    # Receiver clock leads by five seconds. Forward delay 80 ms, reverse 20 ms.
    assert clock.observe(10.0, 15.08, 15.09, 10.11)
    bounds = clock.bounds(10.11)
    assert bounds is not None
    lower, upper = bounds
    assert lower <= 4.98 and upper >= 5.08
    assert clock.bounds(20.12) is None


def test_clock_jump_clears_estimate():
    clock = ClockWindow()
    assert clock.observe(10.0, 15.02, 15.025, 10.045)
    assert not clock.observe(11.0, 21.02, 21.025, 11.045)
    assert clock.bounds(11.045) is None


def test_browser_age_upper_bound_and_window():
    clock = ClockWindow()
    assert clock.observe(10.0, 15.02, 15.025, 10.045)
    assert clock.age_upper(10.2, 15.45) == pytest.approx(0.271, abs=0.001)
    assert clock.age_upper(10.2, 15.5) > 0.3


def test_delay_budget_requires_recent_samples_and_uses_ceiling_p90():
    budget = DelayBudget()
    budget.note(1.0, 0.019)
    budget.note(1.1, 0.0541)
    assert budget.compensation(1.1) is None
    budget.note(1.2, 0.024)
    assert budget.compensation(1.2) == 0.055
    assert budget.compensation(11.01) is None


async def test_measured_asymmetric_transport_bounds_reach_final_admission(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from types import SimpleNamespace

    from senior_control.agent.websocket import WebSocketLink
    from senior_control.cloud import relay
    from senior_control.cloud.limits import CloudLimits
    from senior_control.protocol import (
        EmptyPayload,
        SimpleEnvelope,
        VelocityEnvelope,
        VelocityPayload,
    )
    from senior_control.safety.arbiter import Arbiter

    now = 20.0
    reverse_delay = 0.02
    offset = 5.0
    monkeypatch.setattr(
        relay, "time", SimpleNamespace(monotonic=lambda: now, time=lambda: 100 + now)
    )
    hub = relay.Hub(CloudLimits())
    agent = hub.attach_agent("car-a")

    class FakeSocket:
        async def send(self, data: str) -> None:
            nonlocal now
            now += 0.03
            hub.handle_agent_frame(agent, data.encode())

        async def recv(self) -> bytes:
            nonlocal now
            raw = await agent.outbox.get()
            assert isinstance(raw, bytes)
            now += reverse_delay
            return raw

        async def close(self) -> None:
            pass

    link = WebSocketLink(
        FakeSocket(), "car-a", agent.epoch,
        clock=lambda: now - offset, wall_clock=lambda: 200 + now, drive_timing=True,
    )
    for _ in range(4):
        await link.send_heartbeat()
        await link.recv()
        now += 0.2
    assert hub.is_online("car-a")
    assert link.command_clock_valid()
    assert agent.drive_budget.compensation(now) is not None

    # Calibration remains valid as the reverse link slows, so the larger recent
    # one-way observations enter the cloud's compensation without relaxing the age cap.
    reverse_delay = 0.4
    for _ in range(4):
        await link.send_heartbeat()
        await link.recv()
        now += 0.1
    compensation = agent.drive_budget.compensation(now)
    assert compensation is not None
    assert 0.4 <= compensation <= 0.435

    issued = now
    deadline = issued + 0.3 + compensation
    sample = VelocityEnvelope(
        v=1, type="drive.velocity", robot_id="car-a", session_id="page-a",
        connection_epoch=agent.epoch, request_id="drive-a", seq=2, expires_at=0.0,
        payload=VelocityPayload(
            vx=0.2, wz=0.0, gesture=1, sampled_at=100.0,
            server_issued_at=issued, server_deadline=deadline,
        ),
    )
    arbiter = Arbiter("car-a", 0.2, 2.0, 0.3)
    acquire = SimpleEnvelope(
        **sample.model_dump(exclude={"type", "payload", "seq"}),
        type="control.acquire", seq=1, payload=EmptyPayload(),
    )
    assert arbiter.accept(acquire, now - offset).accepted
    now += reverse_delay
    timing = link.local_drive_timing(issued, deadline)
    assert timing is not None
    earliest_issue, earliest_deadline = timing
    assert arbiter.accept(
        sample, now - offset, received_at=earliest_issue, local_deadline=earliest_deadline,
    ).accepted
    # The IPC queues can make a sample expire even though the Agent received it in time.
    now += 0.11
    later = sample.model_copy(update={"seq": 3})
    assert not arbiter.accept(
        later, now - offset, received_at=earliest_issue, local_deadline=earliest_deadline,
    ).accepted
