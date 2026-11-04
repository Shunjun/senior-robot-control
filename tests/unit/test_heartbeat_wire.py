import json

from senior_control.cloud.limits import CloudLimits
from senior_control.cloud.relay import Hub


async def test_authenticated_heartbeat_echoes_epoch_and_nonce() -> None:
    hub = Hub(CloudLimits())
    connection = hub.attach_agent("robot-a")
    assert not hub.is_online("robot-a")
    hub.handle_agent_frame(
        connection,
        json.dumps(
            {
                "v": 1,
                "type": "heartbeat.request",
                "robot_id": "robot-a",
                "connection_epoch": connection.epoch,
                "nonce": "fresh-nonce",
                "sent_at": 100.0,
            }
        ).encode(),
    )
    reply = json.loads(await connection.outbox.get())
    assert reply["type"] == "heartbeat.reply"
    assert reply["robot_id"] == "robot-a"
    assert reply["connection_epoch"] == connection.epoch
    assert reply["nonce"] == "fresh-nonce"
    assert reply["sent_at"] == 100.0
    assert hub.is_online("robot-a")


def test_missing_heartbeat_expires_connection_even_if_socket_is_open(monkeypatch) -> None:
    now = [10.0]
    monkeypatch.setattr("senior_control.cloud.relay.time.monotonic", lambda: now[0])
    hub = Hub(CloudLimits())
    connection = hub.attach_agent("robot-a")
    now[0] = 11.1
    hub.expire_stale_agent(connection)
    assert connection.outbox.closed
    assert not hub.is_online("robot-a")
