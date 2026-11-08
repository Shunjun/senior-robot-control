import json

import pytest

from senior_control.agent.connection import HeartbeatFrame
from senior_control.agent.websocket import WebSocketLink
from senior_control.runtime.supervisor import AuthRejected


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
    assert await link.recv() == HeartbeatFrame("robot-a", 3.0)


async def test_reply_rejects_nonfinite_server_clock() -> None:
    socket = Socket()
    link = WebSocketLink(socket, "robot-a", 3, wall_clock=lambda: 100.0, clock=lambda: 1.0)
    await link.send_heartbeat()
    request = json.loads(socket.sent[0])
    socket.inbox.append(
        json.dumps({**request, "type": "heartbeat.reply", "server_time": float("nan")})
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


@pytest.mark.parametrize(
    ("code", "terminal"), [(4403, True), (1008, False), (1007, False), (1009, False)]
)
async def test_only_explicit_revocation_is_terminal(code: int, terminal: bool) -> None:
    from websockets.exceptions import ConnectionClosedError
    from websockets.frames import Close

    class RejectedSocket(Socket):
        async def recv(self) -> str:
            raise ConnectionClosedError(Close(code, "closed"), None)

    link = WebSocketLink(RejectedSocket(), "robot-a", 3)
    with pytest.raises((AuthRejected, ConnectionError)) as failure:
        await link.recv()
    assert isinstance(failure.value, AuthRejected) is terminal


async def test_clock_jump_invalidates_command_clock_without_losing_heartbeat() -> None:
    socket = Socket()
    wall, mono = [100.0], [1.0]
    link = WebSocketLink(socket, "robot-a", 3, wall_clock=lambda: wall[0], clock=lambda: mono[0])
    await link.send_heartbeat()
    request = json.loads(socket.sent[0])
    mono[0] = 1.02
    wall[0] = 102.0
    socket.inbox.append(json.dumps({**request, "type": "heartbeat.reply", "server_time": 100.01}))
    assert await link.recv() == HeartbeatFrame("robot-a", 3.0)
    assert not link.command_clock_valid()


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


async def test_server_deadline_conversion_preserves_round_trip_age_and_expires() -> None:
    socket = Socket()
    wall, mono = [100.0], [1.0]
    link = WebSocketLink(socket, "robot-a", 3, wall_clock=lambda: wall[0], clock=lambda: mono[0])
    assert link.local_command_deadline(100.5) is None
    await link.send_heartbeat()
    request = json.loads(socket.sent[0])
    mono[0], wall[0] = 1.02, 100.02
    socket.inbox.append(json.dumps({**request, "type": "heartbeat.reply", "server_time": 100.01}))
    await link.recv()
    deadline = link.local_command_deadline(100.5)
    assert deadline == pytest.approx(1.44)
    mono[0], wall[0] = 1.45, 100.45
    assert link.local_command_deadline(100.5) is None


async def test_server_deadline_conversion_rejects_wall_clock_jump() -> None:
    socket = Socket()
    wall, mono = [100.0], [1.0]
    link = WebSocketLink(socket, "robot-a", 3, wall_clock=lambda: wall[0], clock=lambda: mono[0])
    await link.send_heartbeat()
    request = json.loads(socket.sent[0])
    socket.inbox.append(json.dumps({**request, "type": "heartbeat.reply", "server_time": 100.0}))
    await link.recv()
    wall[0] += 1.0
    assert link.local_command_deadline(100.5) is None


async def test_server_deadline_conversion_rejects_stale_heartbeat() -> None:
    socket = Socket()
    wall, mono = [100.0], [1.0]
    link = WebSocketLink(socket, "robot-a", 3, wall_clock=lambda: wall[0], clock=lambda: mono[0])
    await link.send_heartbeat()
    request = json.loads(socket.sent[0])
    socket.inbox.append(json.dumps({**request, "type": "heartbeat.reply", "server_time": 100.0}))
    await link.recv()
    wall[0], mono[0] = 102.01, 3.01
    assert link.local_command_deadline(102.0) is None


@pytest.mark.parametrize(("elapsed", "offset"), [(0.5, 0.0), (0.02, 1.0)])
async def test_unqualified_command_clock_keeps_link_alive(elapsed: float, offset: float) -> None:
    socket = Socket()
    wall, mono = [100.0], [1.0]
    link = WebSocketLink(socket, "robot-a", 3, wall_clock=lambda: wall[0], clock=lambda: mono[0])
    await link.send_heartbeat()
    request = json.loads(socket.sent[0])
    mono[0], wall[0] = 1.0 + elapsed, 100.0 + elapsed
    socket.inbox.append(
        json.dumps({**request, "type": "heartbeat.reply", "server_time": 100.0 + offset})
    )
    assert await link.recv() == HeartbeatFrame("robot-a", 3.0)
    assert not link.command_clock_valid()
    assert link.local_command_deadline(101.0) is None


async def test_missing_reply_times_out_even_if_receive_is_idle() -> None:
    import asyncio

    class IdleSocket(Socket):
        async def recv(self) -> str:
            await asyncio.Future()
            raise AssertionError("receive should have been cancelled")

    link = WebSocketLink(IdleSocket(), "robot-a", 3, heartbeat_timeout=0.01)
    reader = asyncio.create_task(link.recv())
    try:
        await asyncio.sleep(0)
        await link.send_heartbeat()
        done, _ = await asyncio.wait({reader}, timeout=0.2)
        assert reader in done
        with pytest.raises(ConnectionError, match="heartbeat reply timed out"):
            await reader
    finally:
        reader.cancel()
        await asyncio.gather(reader, return_exceptions=True)


async def test_link_receives_and_sends_video_signaling() -> None:
    from senior_control.protocol import SdpPayload, VideoSdpEnvelope

    socket = Socket()
    link = WebSocketLink(socket, "robot-a", 3)
    offer = VideoSdpEnvelope(
        v=1,
        type="video.offer",
        robot_id="robot-a",
        session_id="viewer",
        request_id="offer",
        seq=1,
        connection_epoch=3,
        expires_at=100.0,
        payload=SdpPayload(sdp="v=0"),
    )
    socket.inbox.append(offer.model_dump_json())
    assert await link.recv() == offer
    answer = offer.model_copy(update={"type": "video.answer"})
    await link.send_video(answer)
    assert json.loads(socket.sent[0]) == answer.model_dump()
