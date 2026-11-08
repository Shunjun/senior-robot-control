import json
import time
from types import SimpleNamespace

import pytest

from senior_control.cloud import relay
from senior_control.cloud.limits import CloudLimits
from senior_control.cloud.relay import Hub
from senior_control.errors import ErrorCode
from senior_control.protocol import validate_frame
from senior_control.safety.arbiter import Arbiter


def frame(kind: str, seq: int, payload: dict | None = None, *, epoch: int = 1) -> bytes:
    return json.dumps(
        {
            "v": 1,
            "robot_id": "car-a",
            "session_id": "session-a",
            "connection_epoch": epoch,
            "request_id": f"r{seq}",
            "seq": seq,
            "expires_at": time.time() + 5,
            "type": kind,
            "payload": payload or {},
        }
    ).encode()


def answer(raw: bytes, *, ok: bool = True) -> bytes:
    request = json.loads(raw)
    return json.dumps(
        {**request, "type": "response", "payload": {"request_type": request["type"], "ok": ok}}
    ).encode()


async def test_challenge_requires_ack_and_is_single_use() -> None:
    hub = Hub(CloudLimits())
    agent = hub.attach_agent("car-a")
    control = hub.attach_controller("car-a", "auth")
    assert control is not None
    hub.handle_control_frame(control, frame("drive.challenge", 1))
    denied = json.loads(await control.outbox.get())
    assert not denied["payload"]["ok"]
    hub.handle_control_frame(control, frame("control.acquire", 2))
    await agent.outbox.get()
    hub.handle_agent_frame(
        agent,
        json.dumps(
            {
                **json.loads(frame("control.acquire", 2)),
                "type": "response",
                "payload": {"request_type": "control.acquire", "ok": True},
            }
        ).encode(),
    )
    await control.outbox.get()
    hub.handle_control_frame(control, frame("drive.challenge", 3))
    challenge = json.loads(await control.outbox.get())["payload"]
    assert challenge["ok"] and challenge["challenge_id"]
    velocity = frame(
        "drive.velocity", 4, {"vx": 0.1, "wz": 0.0, "challenge_id": challenge["challenge_id"]}
    )
    hub.handle_control_frame(control, velocity)
    admitted = json.loads(await agent.outbox.get())
    assert admitted["payload"]["server_deadline"] == challenge["server_deadline"]
    hub.handle_control_frame(control, velocity)
    assert not json.loads(await control.outbox.get())["payload"]["ok"]


async def test_challenge_is_revoked_by_agent_replacement() -> None:
    hub = Hub(CloudLimits())
    agent = hub.attach_agent("car-a")
    control = hub.attach_controller("car-a", "auth")
    assert control is not None
    acquire = frame("control.acquire", 1)
    hub.handle_control_frame(control, acquire)
    await agent.outbox.get()
    hub.handle_agent_frame(
        agent,
        json.dumps(
            {
                **json.loads(acquire),
                "type": "response",
                "payload": {"request_type": "control.acquire", "ok": True},
            }
        ).encode(),
    )
    await control.outbox.get()
    hub.handle_control_frame(control, frame("drive.challenge", 2))
    token = json.loads(await control.outbox.get())["payload"]["challenge_id"]
    replacement = hub.attach_agent("car-a")
    hub.handle_control_frame(
        control, frame("drive.velocity", 3, {"vx": 0.1, "wz": 0.0, "challenge_id": token})
    )
    assert not json.loads(await control.outbox.get())["payload"]["ok"]
    assert replacement.outbox.pending == 0


async def test_cloud_wall_jump_does_not_extend_a_forwarded_drive_deadline(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    now = SimpleNamespace(wall=100.0, mono=0.0)
    monkeypatch.setattr(
        relay, "time", SimpleNamespace(time=lambda: now.wall, monotonic=lambda: now.mono)
    )
    hub = Hub(CloudLimits())
    agent = hub.attach_agent("car-a")
    control = hub.attach_controller("car-a", "auth")
    assert control is not None
    heartbeat = json.dumps(
        {
            "v": 1,
            "type": "heartbeat.request",
            "robot_id": "car-a",
            "connection_epoch": 1,
            "nonce": "probe",
            "sent_at": 100.0,
        }
    ).encode()
    hub.handle_agent_frame(agent, heartbeat)
    anchor = json.loads(await agent.outbox.get())["server_time"]
    acquire = frame("control.acquire", 1)
    hub.handle_control_frame(control, acquire)
    await agent.outbox.get()
    hub.handle_agent_frame(
        agent,
        json.dumps(
            {
                **json.loads(acquire),
                "type": "response",
                "payload": {"request_type": "control.acquire", "ok": True},
            }
        ).encode(),
    )
    await control.outbox.get()

    now.wall, now.mono = 100.21, 0.01
    hub.handle_control_frame(control, frame("drive.challenge", 2))
    grant = json.loads(await control.outbox.get())["payload"]
    hub.handle_control_frame(
        control,
        frame("drive.velocity", 3, {"vx": 0.1, "wz": 0.0, "challenge_id": grant["challenge_id"]}),
    )
    forwarded = json.loads(await agent.outbox.get())["payload"]
    assert forwarded["server_deadline"] - anchor == pytest.approx(0.31)

    hub.handle_agent_frame(agent, heartbeat)
    assert json.loads(await agent.outbox.get())["server_time"] - anchor == pytest.approx(0.01)


async def test_emergency_stop_without_snapshot_uses_authenticated_agent_epoch() -> None:
    hub = Hub(CloudLimits())
    agent = hub.attach_agent("car-a")
    control = hub.attach_controller("car-a", "auth")
    assert control is not None
    request = json.loads(frame("safety.estop", 1))
    request["connection_epoch"] = 0
    hub.handle_control_frame(control, json.dumps(request).encode())
    forwarded = json.loads(await agent.outbox.get())
    assert forwarded["connection_epoch"] == agent.epoch
    assert forwarded["type"] == "safety.estop"


async def test_later_drive_stop_cannot_overtake_an_earlier_estop() -> None:
    hub = Hub(CloudLimits())
    agent = hub.attach_agent("car-a")
    control = hub.attach_controller("car-a", "auth")
    assert control is not None
    arbiter = Arbiter("car-a", 0.2, 1.0, 0.3, require_server_heartbeat=True)
    assert arbiter.server_heartbeat(agent.epoch, 0.0, 0.8)
    assert arbiter.accept(validate_frame(frame("control.acquire", 1), "car-a"), 0.1).accepted

    hub.handle_control_frame(control, frame("safety.estop", 2))
    hub.handle_control_frame(control, frame("drive.stop", 3))
    for _ in range(2):
        queued = await agent.outbox.get()
        assert isinstance(queued, bytes)
        arbiter.accept(validate_frame(queued, "car-a"), 0.2)

    assert arbiter.mode(0.2) == "estop"


async def test_browser_cannot_supply_server_deadline_even_null() -> None:
    hub = Hub(CloudLimits())
    agent = hub.attach_agent("car-a")
    control = hub.attach_controller("car-a", "auth")
    assert control is not None
    acquire = frame("control.acquire", 1)
    hub.handle_control_frame(control, acquire)
    await agent.outbox.get()
    hub.handle_agent_frame(
        agent,
        json.dumps(
            {
                **json.loads(acquire),
                "type": "response",
                "payload": {"request_type": "control.acquire", "ok": True},
            }
        ).encode(),
    )
    await control.outbox.get()
    hub.handle_control_frame(control, frame("drive.challenge", 2))
    token = json.loads(await control.outbox.get())["payload"]["challenge_id"]
    hub.handle_control_frame(
        control,
        frame(
            "drive.velocity",
            3,
            {"vx": 0.1, "wz": 0.0, "challenge_id": token, "server_deadline": None},
        ),
    )
    assert not json.loads(await control.outbox.get())["payload"]["ok"]
    assert agent.outbox.pending == 0


async def test_controller_disconnect_revokes_queued_velocity_and_requests_stop() -> None:
    hub = Hub(CloudLimits())
    agent = hub.attach_agent("car-a")
    control = hub.attach_controller("car-a", "auth")
    assert control is not None
    acquire = frame("control.acquire", 1)
    hub.handle_control_frame(control, acquire)
    await agent.outbox.get()
    hub.handle_agent_frame(
        agent,
        json.dumps(
            {
                **json.loads(acquire),
                "type": "response",
                "payload": {"request_type": "control.acquire", "ok": True},
            }
        ).encode(),
    )
    await control.outbox.get()
    hub.handle_control_frame(control, frame("drive.challenge", 2))
    token = json.loads(await control.outbox.get())["payload"]["challenge_id"]
    hub.handle_control_frame(
        control, frame("drive.velocity", 3, {"vx": 0.1, "wz": 0.0, "challenge_id": token})
    )
    hub.detach(control)
    stop = json.loads(await agent.outbox.get())
    assert stop["type"] == "control.release" and stop["seq"] > 3
    assert agent.outbox.pending == 0


async def test_competing_socket_cannot_steal_or_discard_confirmed_lease() -> None:
    hub = Hub(CloudLimits())
    agent = hub.attach_agent("car-a")
    owner = hub.attach_controller("car-a", "owner")
    other = hub.attach_controller("car-a", "other")
    assert owner is not None and other is not None
    acquire = frame("control.acquire", 1)
    hub.handle_control_frame(owner, acquire)
    await agent.outbox.get()
    hub.handle_agent_frame(agent, answer(acquire))
    await owner.outbox.get()
    await other.outbox.get()

    hub.handle_control_frame(other, frame("control.acquire", 2))
    hub.handle_control_frame(other, frame("control.release", 3))
    assert agent.outbox.pending == 0
    assert not json.loads(await other.outbox.get())["payload"]["ok"]
    assert not json.loads(await other.outbox.get())["payload"]["ok"]
    hub.handle_control_frame(owner, frame("drive.challenge", 4))
    assert json.loads(await owner.outbox.get())["payload"]["ok"]

    hub.detach(owner)
    retired = await agent.outbox.get()
    assert isinstance(retired, bytes)
    assert json.loads(retired)["type"] == "control.release"
    arbiter = Arbiter("car-a", 0.2, 1.0, 0.3, require_server_heartbeat=True)
    assert arbiter.server_heartbeat(agent.epoch, 0.0, 0.8)
    assert arbiter.accept(validate_frame(acquire, "car-a"), 0.1).accepted
    assert arbiter.accept(validate_frame(retired, "car-a"), 0.2).accepted
    hub.handle_agent_frame(agent, answer(retired))
    replacement = json.loads(frame("control.acquire", 1))
    replacement["session_id"] = "new-browser"
    hub.handle_control_frame(other, json.dumps(replacement).encode())
    queued = await agent.outbox.get()
    assert isinstance(queued, bytes)
    assert arbiter.accept(validate_frame(queued, "car-a"), 0.3).accepted


async def test_acquire_ack_after_controller_disconnect_releases_exact_lease() -> None:
    hub = Hub(CloudLimits())
    agent = hub.attach_agent("car-a")
    controller = hub.attach_controller("car-a", "auth")
    assert controller is not None
    acquire = frame("control.acquire", 1)
    hub.handle_control_frame(controller, acquire)
    await agent.outbox.get()
    hub.detach(controller)
    hub.handle_agent_frame(agent, answer(acquire))
    assert agent.outbox.pending == 1
    release = json.loads(await agent.outbox.get())
    assert release["type"] == "control.release"
    assert release["session_id"] == "session-a" and release["seq"] > 1


async def test_unanswered_lease_transition_revokes_link_despite_fresh_heartbeat(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    now = SimpleNamespace(mono=0.0)
    monkeypatch.setattr(relay, "time", SimpleNamespace(time=time.time, monotonic=lambda: now.mono))
    hub = Hub(CloudLimits())
    agent = hub.attach_agent("car-a")
    control = hub.attach_controller("car-a", "auth")
    assert control is not None
    hub.handle_control_frame(control, frame("control.acquire", 1))
    await agent.outbox.get()
    now.mono = 1.1
    agent.heartbeat_at = now.mono

    hub.expire_stale_agent(agent)

    assert agent.outbox.closed


async def test_other_socket_cannot_poison_lease_sequence_with_copied_session() -> None:
    hub = Hub(CloudLimits())
    agent = hub.attach_agent("car-a")
    owner = hub.attach_controller("car-a", "owner")
    other = hub.attach_controller("car-a", "other")
    assert owner is not None and other is not None
    arbiter = Arbiter("car-a", 0.2, 1.0, 0.3, require_server_heartbeat=True)
    assert arbiter.server_heartbeat(agent.epoch, 0.0, 0.8)
    hub.handle_control_frame(owner, frame("control.acquire", 1))
    acquire = await agent.outbox.get()
    assert isinstance(acquire, bytes)
    assert arbiter.accept(validate_frame(acquire, "car-a"), 0.1).accepted
    hub.handle_agent_frame(agent, answer(acquire))
    await owner.outbox.get()
    await other.outbox.get()

    for kind in ("drive.stop", "navigation.start"):
        hub.handle_control_frame(other, frame(kind, 100))
        assert agent.outbox.pending == 0
        assert not json.loads(await other.outbox.get())["payload"]["ok"]

    hub.handle_control_frame(owner, frame("safety.estop", 2))
    estop = await agent.outbox.get()
    assert isinstance(estop, bytes)
    assert arbiter.accept(validate_frame(estop, "car-a"), 0.2).accepted
    assert arbiter.mode(0.2) == "estop"

    hub.handle_control_frame(other, frame("safety.estop", 100))
    other_estop = await agent.outbox.get()
    assert isinstance(other_estop, bytes)
    other_command = validate_frame(other_estop, "car-a")
    assert other_command.session_id != "session-a"
    assert arbiter.accept(other_command, 0.3).accepted

    hub.handle_control_frame(owner, frame("control.release", 3))
    release = await agent.outbox.get()
    assert isinstance(release, bytes)
    assert arbiter.accept(validate_frame(release, "car-a"), 0.4).accepted


async def test_reacquire_retires_queued_velocity_and_old_challenges() -> None:
    hub = Hub(CloudLimits())
    agent = hub.attach_agent("car-a")
    owner = hub.attach_controller("car-a", "owner")
    assert owner is not None

    acquire = frame("control.acquire", 1)
    hub.handle_control_frame(owner, acquire)
    await agent.outbox.get()
    hub.handle_agent_frame(agent, answer(acquire))
    await owner.outbox.get()

    async def grant(seq: int) -> str:
        hub.handle_control_frame(owner, frame("drive.challenge", seq))
        return json.loads(await owner.outbox.get())["payload"]["challenge_id"]

    token = await grant(2)
    older_token = await grant(3)
    hub.handle_control_frame(
        owner, frame("drive.velocity", 4, {"vx": 0.1, "wz": 0.0, "challenge_id": token})
    )
    reacquire = frame("control.acquire", 5)
    hub.handle_control_frame(owner, reacquire)
    assert await agent.outbox.get() == reacquire

    hub.handle_control_frame(owner, frame("drive.challenge", 6))
    assert not json.loads(await owner.outbox.get())["payload"]["ok"]
    hub.handle_agent_frame(agent, answer(reacquire))
    await owner.outbox.get()

    hub.handle_control_frame(
        owner, frame("drive.velocity", 7, {"vx": 0.1, "wz": 0.0, "challenge_id": older_token})
    )
    assert not json.loads(await owner.outbox.get())["payload"]["ok"]
    assert await grant(8) != older_token


async def test_reconnect_blocks_motion_until_new_heartbeat_and_lease() -> None:
    hub = Hub(CloudLimits())
    previous = hub.attach_agent("car-a")
    control = hub.attach_controller("car-a", "owner")
    assert control is not None

    def heartbeat(epoch: int) -> bytes:
        return json.dumps(
            {
                "v": 1,
                "type": "heartbeat.request",
                "robot_id": "car-a",
                "connection_epoch": epoch,
                "nonce": "probe",
                "sent_at": time.time(),
            }
        ).encode()

    hub.handle_agent_frame(previous, heartbeat(previous.epoch))
    await previous.outbox.get()
    acquire = frame("control.acquire", 1)
    hub.handle_control_frame(control, acquire)
    await previous.outbox.get()
    hub.handle_agent_frame(previous, answer(acquire))
    await control.outbox.get()

    hub.detach(previous)
    assert hub.is_reconnecting("car-a")
    current = hub.attach_agent("car-a")
    hub.handle_control_frame(control, frame("control.acquire", 2, epoch=current.epoch))
    assert current.outbox.pending == 0
    assert json.loads(await control.outbox.get())["payload"]["error"] == ErrorCode.OFFLINE

    hub.handle_agent_frame(current, heartbeat(current.epoch))
    await current.outbox.get()
    assert hub.is_online("car-a")
    hub.handle_agent_frame(previous, answer(acquire))
    assert control.outbox.pending == 0
    hub.handle_control_frame(control, frame("drive.challenge", 3, epoch=current.epoch))
    assert not json.loads(await control.outbox.get())["payload"]["ok"]

    acquire = frame("control.acquire", 4, epoch=current.epoch)
    hub.handle_control_frame(control, acquire)
    assert await current.outbox.get() == acquire
    hub.handle_agent_frame(current, answer(acquire))
    await control.outbox.get()
    hub.handle_control_frame(control, frame("drive.challenge", 5, epoch=current.epoch))
    assert json.loads(await control.outbox.get())["payload"]["ok"]


async def test_timed_input_uses_browser_age_and_cloud_compensation(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    now = SimpleNamespace(wall=100.0, mono=1.0)
    monkeypatch.setattr(
        relay, "time", SimpleNamespace(time=lambda: now.wall, monotonic=lambda: now.mono)
    )
    hub = Hub(CloudLimits())
    agent = hub.attach_agent("car-a")
    control = hub.attach_controller("car-a", "auth")
    assert control is not None
    agent.drive_timing = control.drive_timing = True
    agent.drive_budget.note(1.0, 0.049)
    agent.drive_budget.note(1.0, 0.051)
    agent.drive_budget.note(1.0, 0.060)
    assert agent.drive_clock.window.observe(0.95, 1.0, 1.0, 1.05)
    assert control.drive_clock.window.observe(0.95, 1.0, 1.0, 1.05)
    control.lease_session = "session-a"
    now.mono = 1.2
    latest = {"vx": 0.1, "wz": 0.0, "gesture": 1, "sampled_at": 1.15}
    hub.handle_control_frame(control, frame("drive.velocity", 2, latest))
    assert agent.outbox.pending == 1
    admitted = json.loads(await agent.outbox.get())["payload"]
    assert admitted["server_issued_at"] == 1.2
    assert admitted["server_deadline"] == pytest.approx(1.56)
    hub.handle_control_frame(control, frame("drive.velocity", 3, {**latest, "sampled_at": 0.92}))
    assert json.loads(await control.outbox.get())["payload"]["error"] == "timeout"
    hub.handle_control_frame(control, frame("drive.velocity", 2, latest))
    assert agent.outbox.pending == 0
    assert json.loads(await control.outbox.get())["payload"]["error"] == "conflict"
