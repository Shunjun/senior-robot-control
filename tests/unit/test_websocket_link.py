import json

import pytest

from senior_control.agent.connection import HeartbeatFrame
from senior_control.agent.websocket import WebSocketLink


class Socket:
    def __init__(self) -> None:
        self.sent: list[str] = []
        self.inbox: list[str] = []

    async def send(self, data: str) -> None:
        self.sent.append(data)

    async def recv(self) -> str:
        return self.inbox.pop(0)

    async def close(self) -> None:
        pass


async def test_reply_must_match_outstanding_nonce_and_epoch() -> None:
    socket = Socket()
    link = WebSocketLink(socket, "robot-a", 3, wall_clock=lambda: 100.0, clock=lambda: 1.0)
    await link.send_heartbeat()
    request = json.loads(socket.sent[0])
    reply = {**request, "type": "heartbeat.reply", "server_time": 100.0}
    socket.inbox.append(json.dumps({**reply, "nonce": "other"}))
    with pytest.raises(ConnectionError):
        await link.recv()
    socket.inbox.append(json.dumps(reply))
    assert await link.recv() == HeartbeatFrame("robot-a", 1.8)


@pytest.mark.parametrize("server_time", [float("nan"), 101.0])
async def test_reply_rejects_unbounded_server_clock(server_time: float) -> None:
    socket = Socket()
    link = WebSocketLink(socket, "robot-a", 3, wall_clock=lambda: 100.0, clock=lambda: 1.0)
    await link.send_heartbeat()
    request = json.loads(socket.sent[0])
    socket.inbox.append(
        json.dumps({**request, "type": "heartbeat.reply", "server_time": server_time})
    )
    with pytest.raises(ConnectionError):
        await link.recv()


async def test_connector_uses_authenticated_socket_and_epoch() -> None:
    from websockets.asyncio.server import serve

    from senior_control.agent.websocket import WebSocketConnector

    async def handler(socket: object) -> None:
        await socket.wait_closed()  # type: ignore[attr-defined]

    def headers(connection: object, request: object, response: object) -> None:
        response.headers["X-Connection-Epoch"] = "9"  # type: ignore[attr-defined]
        return None

    async with serve(handler, "127.0.0.1", 0, process_response=headers) as server:
        port = server.sockets[0].getsockname()[1]
        connector = WebSocketConnector(f"ws://127.0.0.1:{port}", "token", "robot-a")
        link = await connector()
        assert link.epoch == 9
        await link.close()


async def test_policy_close_is_terminal_auth_rejection() -> None:
    from websockets.exceptions import ConnectionClosedError
    from websockets.frames import Close

    from senior_control.runtime.supervisor import AuthRejected

    class RejectedSocket(Socket):
        async def recv(self) -> str:
            raise ConnectionClosedError(Close(1008, "revoked"), None)

    link = WebSocketLink(RejectedSocket(), "robot-a", 3)
    with pytest.raises(AuthRejected):
        await link.recv()


async def test_heartbeat_rejects_wall_clock_jump_during_round_trip() -> None:
    socket = Socket()
    wall, mono = [100.0], [1.0]
    link = WebSocketLink(socket, "robot-a", 3, wall_clock=lambda: wall[0], clock=lambda: mono[0])
    await link.send_heartbeat()
    request = json.loads(socket.sent[0])
    mono[0] = 1.02
    wall[0] = 102.0
    socket.inbox.append(json.dumps({**request, "type": "heartbeat.reply", "server_time": 100.01}))
    with pytest.raises(ConnectionError):
        await link.recv()


async def test_link_detects_wall_clock_change_after_heartbeat() -> None:
    socket = Socket()
    wall, mono = [100.0], [1.0]
    link = WebSocketLink(socket, "robot-a", 3, wall_clock=lambda: wall[0], clock=lambda: mono[0])
    await link.send_heartbeat()
    request = json.loads(socket.sent[0])
    socket.inbox.append(json.dumps({**request, "type": "heartbeat.reply", "server_time": 100.0}))
    await link.recv()
    assert link.command_clock_valid()
    wall[0] += 1.0
    assert not link.command_clock_valid()
