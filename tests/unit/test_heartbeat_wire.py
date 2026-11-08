import json

import pytest

from senior_control.cloud.limits import CloudLimits
from senior_control.cloud.relay import Connection, Hub


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


def test_missing_heartbeat_expires_connection_even_if_socket_is_open(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    now = [10.0]
    monkeypatch.setattr("senior_control.cloud.relay.time.monotonic", lambda: now[0])
    hub = Hub(CloudLimits())
    connection = hub.attach_agent("robot-a")
    now[0] = 11.1
    hub.expire_stale_agent(connection)
    assert not connection.outbox.closed
    now[0] = 12.1
    hub.expire_stale_agent(connection)
    assert connection.outbox.closed
    assert not hub.is_online("robot-a")


def test_reconnect_grace_ends_without_a_fresh_epoch_heartbeat(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    now = [10.0]
    monkeypatch.setattr("senior_control.cloud.relay.time.monotonic", lambda: now[0])
    hub = Hub(CloudLimits())
    connection = hub.attach_agent("robot-a")

    def heartbeat(agent: Connection) -> None:
        hub.handle_agent_frame(
            agent,
            json.dumps(
                {
                    "v": 1,
                    "type": "heartbeat.request",
                    "robot_id": "robot-a",
                    "connection_epoch": agent.epoch,
                    "nonce": "fresh-nonce",
                    "sent_at": 100.0,
                }
            ).encode(),
        )

    assert not hub.is_reconnecting("robot-a")
    heartbeat(connection)
    assert hub.is_online("robot-a")
    hub.detach(connection)
    assert not hub.is_online("robot-a")
    assert hub.is_reconnecting("robot-a")

    replacement = hub.attach_agent("robot-a")
    heartbeat(connection)
    assert not hub.is_online("robot-a")
    assert hub.is_reconnecting("robot-a")
    heartbeat(replacement)
    assert hub.is_online("robot-a")
    assert not hub.is_reconnecting("robot-a")

    hub.detach(replacement)
    now[0] += 59
    assert hub.is_reconnecting("robot-a")
    now[0] += 2
    assert not hub.is_online("robot-a")
    assert not hub.is_reconnecting("robot-a")


@pytest.mark.parametrize("transition", ["detach", "replace"])
def test_expired_heartbeat_grace_starts_before_cleanup_and_cannot_restart(
    monkeypatch: pytest.MonkeyPatch, transition: str
) -> None:
    now = [10.0]
    monkeypatch.setattr("senior_control.cloud.relay.time.monotonic", lambda: now[0])
    hub = Hub(CloudLimits())
    previous = hub.attach_agent("robot-a")
    previous.heartbeat_at = now[0]
    assert hub.is_online("robot-a")
    assert not hub.is_reconnecting("robot-a")

    now[0] = 12.01
    assert not hub.is_online("robot-a")
    assert hub.is_reconnecting("robot-a")
    assert not previous.outbox.closed

    now[0] = 71.9
    if transition == "replace":
        hub.attach_agent("robot-a")
        hub.detach(previous)  # Stale socket cleanup must preserve the replacement's state.
    else:
        hub.expire_stale_agent(previous)
    assert hub.is_reconnecting("robot-a")

    now[0] = 72.0
    assert not hub.is_online("robot-a")
    assert not hub.is_reconnecting("robot-a")


def test_revocation_clears_reconnect_grace_before_and_after_detach(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    now = [10.0]
    monkeypatch.setattr("senior_control.cloud.relay.time.monotonic", lambda: now[0])
    hub = Hub(CloudLimits())
    agent = hub.attach_agent("robot-a")
    agent.heartbeat_at = now[0]
    now[0] = 12.01
    hub.revoke_robot("robot-a")
    assert not hub.is_online("robot-a")
    assert not hub.is_reconnecting("robot-a")
    hub.detach(agent)
    assert not hub.is_reconnecting("robot-a")
