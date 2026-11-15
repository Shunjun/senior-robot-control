import json
import time

from senior_control.cloud.limits import CloudLimits
from senior_control.cloud.relay import Hub
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
    arbiter = Arbiter("car-a", 0.3)

    hub.handle_control_frame(control, frame("safety.estop", 2))
    hub.handle_control_frame(control, frame("drive.stop", 3))
    for _ in range(2):
        queued = await agent.outbox.get()
        assert isinstance(queued, bytes)
        arbiter.accept(validate_frame(queued, "car-a"), 0.2)

    assert arbiter.mode(0.2) == "estop"
