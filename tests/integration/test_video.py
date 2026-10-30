"""Task 16 contract: authenticated video session, TURN credentials, no touch on the drive path.

Interface assumed by these tests:

* ``VideoSession(robot_id=, viewer=, source=, factory=, ...)`` is the vehicle-side session of
  one authenticated viewer. ``viewer`` carries the identity the cloud already verified on
  ``/ws/control`` (``role`` and the selected ``robot_id``). Nothing here opens a port.
* ``await session.offer(robot_id, SessionDescription)`` returns the answer, or raises
  ``VideoDenied`` / ``VideoUnavailable`` / ``VideoBusy`` / ``NoCommonCodec``.
* ``issue_turn_credentials(session_id, now)`` follows the coturn REST scheme
  (``use-auth-secret``): ``username = "<expiry>:<session_id>"``,
  ``credential = base64(HMAC-SHA1(secret, username))``.
* ``source`` is ``None`` when the robot has no camera. The session then reports
  ``unavailable`` and never produces a frame.
* Peers are built through an injected factory, so no ``aiortc`` import is needed here.
"""

from __future__ import annotations

import asyncio
import base64
import hashlib
import hmac
import time
from dataclasses import dataclass, field

import pytest

from senior_control.runtime.supervisor import Supervisor
from senior_control.vision.stream import (
    NoCommonCodec,
    PeerRequest,
    PeerStats,
    SessionDescription,
    VideoBusy,
    VideoDenied,
    VideoLimits,
    VideoSession,
    VideoUnavailable,
    ViewerBudget,
    negotiate_codec,
)
from senior_control.vision.synchronizer import ImageFrame
from senior_control.vision.turn import (
    IceCredentials,
    TurnConfig,
    TurnCredentialIssuer,
    verify_turn_credentials,
)

SECRET = b"s" * 32
URLS = ("turn:turn.example.test:3478?transport=udp", "turns:turn.example.test:5349")

OFFER_SDP = (
    "v=0\r\no=- 1 2 IN IP4 127.0.0.1\r\ns=-\r\nt=0 0\r\n"
    "m=audio 9 UDP/TLS/RTP/SAVPF 111\r\na=rtpmap:111 opus/48000/2\r\n"
    "m=video 9 UDP/TLS/RTP/SAVPF {payloads}\r\n{maps}"
)


def video_offer(*codecs: str) -> SessionDescription:
    payloads = " ".join(str(96 + i) for i in range(len(codecs)))
    maps = "".join(f"a=rtpmap:{96 + i} {name}/90000\r\n" for i, name in enumerate(codecs))
    return SessionDescription("offer", OFFER_SDP.format(payloads=payloads, maps=maps))


def rgb_frame(stamp: float) -> ImageFrame:
    return ImageFrame(stamp=stamp, width=1, height=1, encoding="rgb8", data=b"\x01\x02\x03")


# ------------------------------------------------------------------------------- fakes


@dataclass
class Viewer:
    role: str
    robot_id: str


class FakeClock:
    def __init__(self, now: float = 1000.0) -> None:
        self.now = now

    def __call__(self) -> float:
        return self.now


class FakeSource:
    def __init__(self) -> None:
        self._frames: asyncio.Queue[ImageFrame] = asyncio.Queue()

    def feed(self, frame: ImageFrame) -> None:
        self._frames.put_nowait(frame)

    async def next_frame(self, timeout: float) -> ImageFrame | None:
        try:
            return await asyncio.wait_for(self._frames.get(), timeout)
        except asyncio.TimeoutError:
            return None


class FakePeer:
    def __init__(self, factory: FakePeerFactory, request: PeerRequest) -> None:
        self.factory = factory
        self.request = request
        self.pushed: list[ImageFrame] = []
        self.candidates: list[str] = []
        self.connection_state = "connected"
        self.bytes_sent = 0
        self.open = True

    async def accept_offer(self, offer: SessionDescription) -> SessionDescription:
        return SessionDescription("answer", f"answer-for-{self.request.codec}")

    async def add_remote_candidate(self, candidate: str) -> None:
        self.candidates.append(candidate)

    async def push(self, frame: ImageFrame) -> None:
        if self.factory.congested:
            await asyncio.Event().wait()
        self.pushed.append(frame)

    async def stats(self) -> PeerStats:
        return PeerStats(self.connection_state, self.bytes_sent)

    async def close(self) -> None:
        if self.open:
            self.open = False
            self.factory.encoders -= 1


@dataclass
class FakePeerFactory:
    encoders: int = 0
    opened: int = 0
    congested: bool = False
    peers: list[FakePeer] = field(default_factory=list)

    async def open(self, request: PeerRequest) -> FakePeer:
        self.encoders += 1
        self.opened += 1
        peer = FakePeer(self, request)
        self.peers.append(peer)
        return peer


def make_session(
    *,
    viewer: Viewer | None = None,
    source: FakeSource | None | str = "default",
    factory: FakePeerFactory | None = None,
    clock: FakeClock | None = None,
    limits: VideoLimits | None = None,
    budget: ViewerBudget | None = None,
    turn: TurnCredentialIssuer | None = None,
    relay_only: bool = False,
) -> tuple[VideoSession, FakePeerFactory, FakeSource | None, FakeClock]:
    factory = factory or FakePeerFactory()
    clock = clock or FakeClock()
    camera = FakeSource() if source == "default" else source
    assert not isinstance(camera, str)
    session = VideoSession(
        robot_id="robot-a",
        viewer=viewer or Viewer("control", "robot-a"),
        source=camera,
        factory=factory,
        limits=limits or VideoLimits(frame_poll_s=0.01, push_timeout_s=0.05),
        budget=budget,
        turn=turn,
        now=clock,
        relay_only=relay_only,
    )
    return session, factory, camera, clock


# -------------------------------------------------------------------------- authorization


async def test_unselected_robot_stream_denied() -> None:
    session, factory, _, _ = make_session()
    with pytest.raises(VideoDenied):
        await session.offer("robot-b", video_offer("VP8"))
    assert factory.opened == 0

    other_selection, factory2, _, _ = make_session(viewer=Viewer("control", "robot-b"))
    with pytest.raises(VideoDenied):
        await other_selection.offer("robot-a", video_offer("VP8"))
    assert factory2.opened == 0

    device_socket, factory3, _, _ = make_session(viewer=Viewer("agent", "robot-a"))
    with pytest.raises(VideoDenied):
        await device_socket.offer("robot-a", video_offer("VP8"))
    assert factory3.opened == 0

    with pytest.raises(VideoDenied):
        await other_selection.add_ice_candidate("candidate:1 1 udp 1 10.0.0.1 9 typ host")
    with pytest.raises(VideoDenied):
        other_selection.turn_credentials(1000)
    await session.close()


async def test_hub_controller_selection_gates_video() -> None:
    pytest.importorskip("fastapi")
    from senior_control.cloud.limits import CloudLimits
    from senior_control.cloud.relay import Hub

    hub = Hub(CloudLimits())
    controller = hub.attach_controller("robot-a", "session-hash")
    assert controller is not None
    allowed, factory, _, _ = make_session(viewer=controller)  # type: ignore[arg-type]
    answer = await allowed.offer("robot-a", video_offer("VP8"))
    assert answer.type == "answer"
    with pytest.raises(VideoDenied):
        await allowed.offer("robot-b", video_offer("VP8"))
    await allowed.close()
    assert factory.encoders == 0

    agent = hub.attach_agent("robot-a")
    device, factory2, _, _ = make_session(viewer=agent)  # type: ignore[arg-type]
    with pytest.raises(VideoDenied):
        await device.offer("robot-a", video_offer("VP8"))
    assert factory2.opened == 0


# ------------------------------------------------------------------------------- TURN


def test_turn_credentials_expire() -> None:
    issuer = TurnCredentialIssuer(TurnConfig(urls=URLS, secret=SECRET, ttl_seconds=600))
    creds = issuer.issue_turn_credentials("session-1", 1000)
    assert isinstance(creds, IceCredentials)
    assert creds.urls == URLS
    assert creds.expires_at == 1600
    assert creds.username == "1600:session-1"
    expected = base64.b64encode(
        hmac.new(SECRET, creds.username.encode(), hashlib.sha1).digest()
    ).decode()
    assert creds.credential == expected

    assert verify_turn_credentials(creds.username, creds.credential, 1000, SECRET)
    assert verify_turn_credentials(creds.username, creds.credential, 1599, SECRET)
    assert not verify_turn_credentials(creds.username, creds.credential, 1600, SECRET)
    assert not verify_turn_credentials(creds.username, creds.credential, 9999, SECRET)
    assert not verify_turn_credentials("1700:session-1", creds.credential, 1000, SECRET)
    assert not verify_turn_credentials(creds.username, creds.credential, 1000, b"t" * 32)
    assert not verify_turn_credentials("session-1", creds.credential, 1000, SECRET)

    later = issuer.issue_turn_credentials("session-1", 5000)
    assert later.expires_at == 5600
    assert later.credential != creds.credential
    assert "secret" not in repr(creds) and creds.credential not in repr(creds)

    with pytest.raises(ValueError):
        issuer.issue_turn_credentials("bad:id", 1000)
    with pytest.raises(ValueError):
        TurnConfig(urls=URLS, secret=b"short")
    with pytest.raises(ValueError):
        TurnConfig(urls=("http://x",), secret=SECRET)
    with pytest.raises(ValueError):
        TurnConfig(urls=URLS, secret=SECRET, ttl_seconds=10**6)


async def test_session_hands_out_credentials_for_an_opaque_session_id() -> None:
    issuer = TurnCredentialIssuer(TurnConfig(urls=URLS, secret=SECRET, ttl_seconds=600))
    session, factory, _, clock = make_session(turn=issuer, relay_only=True)
    creds = session.turn_credentials(int(clock.now))
    assert creds.expires_at == 1600
    assert creds.relay_only is True
    assert "session-hash" not in creds.username
    assert creds.username.split(":", 1)[1] == session.session_id
    await session.offer("robot-a", video_offer("VP8"))
    request = factory.peers[0].request
    assert request.ice is not None and request.ice.urls == URLS
    assert request.relay_only is True
    await session.close()


# --------------------------------------------------------------------------- congestion


async def test_congested_video_keeps_heartbeat_and_stop() -> None:
    session, factory, source, clock = make_session(factory=FakePeerFactory(congested=True))
    assert source is not None
    await session.offer("robot-a", video_offer("VP8"))

    beats = 0
    driven: list[tuple[float, float]] = []

    async def heartbeat() -> None:
        nonlocal beats
        beats += 1

    async def ros(name: str) -> None:
        return None

    async def sink(vx: float, wz: float) -> None:
        driven.append((vx, wz))

    async def cmd_vel(vx: float, wz: float) -> None:
        raise AssertionError("video must never publish drive")

    supervisor = Supervisor(
        send_heartbeat=heartbeat,
        call_ros=ros,
        drive_sink=sink,
        publish_cmd_vel=cmd_vel,
        heartbeat_seconds=0.01,
    )
    submitted = 0
    original = supervisor.submit_drive

    def counted(vx: float, wz: float) -> bool:
        nonlocal submitted
        submitted += 1
        return original(vx, wz)

    supervisor.submit_drive = counted  # type: ignore[method-assign]

    async with supervisor:
        assert supervisor.submit_drive(0.3, 0.0)
        for index in range(300):
            source.feed(rgb_frame(clock.now + index * 0.001))
            if index % 50 == 0:
                await asyncio.sleep(0.01)
        beats_before = beats
        await asyncio.sleep(0.2)

        assert supervisor.pending == 0  # the video flood never occupied the drive slot
        assert submitted == 1  # only the test's own drive command
        assert beats > beats_before  # heartbeat kept running under congestion
        assert supervisor.submit_drive(0.0, 0.0)
        await asyncio.sleep(0.05)
        assert driven[-1] == (0.0, 0.0)  # stop still reaches the sink

    status = await session.status()
    assert status.frames_sent == 0
    assert status.frames_dropped > 0
    assert status.buffered_frames <= 1  # one latest frame, never a growing backlog
    assert factory.peers[0].pushed == []
    await session.close()
    assert factory.encoders == 0


# ------------------------------------------------------------------------------ camera


async def test_no_camera_returns_unavailable() -> None:
    session, factory, _, _ = make_session(source=None)
    assert (await session.status()).state == "unavailable"
    with pytest.raises(VideoUnavailable):
        await session.offer("robot-a", video_offer("VP8"))
    status = await session.status()
    assert status.state == "unavailable"
    assert status.reason == "no_camera"
    assert status.frame_age_s is None
    assert status.frames_sent == 0
    assert factory.opened == 0
    await session.close()


async def test_frame_age_is_measured_not_assumed() -> None:
    session, factory, source, clock = make_session()
    assert source is not None
    await session.offer("robot-a", video_offer("VP8"))
    assert (await session.status()).state == "stalled"  # connected, no frame seen yet
    assert (await session.status()).frame_age_s is None

    source.feed(rgb_frame(clock.now - 0.25))
    await asyncio.sleep(0.1)
    status = await session.status()
    assert status.state == "live"
    assert status.frame_age_s == pytest.approx(0.25)
    assert status.frames_sent == 1

    clock.now += 10.0
    status = await session.status()
    assert status.state == "stalled"
    assert status.frame_age_s == pytest.approx(10.25)

    source.feed(rgb_frame(clock.now - 30.0))  # a stale frame is not forwarded as live
    await asyncio.sleep(0.1)
    assert (await session.status()).frames_sent == 1
    await session.close()


async def test_relay_failure_reports_state_without_video() -> None:
    session, factory, _, _ = make_session(relay_only=True)
    await session.offer("robot-a", video_offer("VP8"))
    factory.peers[0].connection_state = "failed"
    status = await session.status()
    assert status.state == "relay_failed"
    assert status.frames_sent == 0
    await session.close()


# --------------------------------------------------------------------- close and limits


async def test_session_close_releases_encoder() -> None:
    budget = ViewerBudget(1)
    session, factory, source, clock = make_session(budget=budget)
    assert source is not None
    await session.offer("robot-a", video_offer("VP8"))
    source.feed(rgb_frame(clock.now))
    await asyncio.sleep(0.1)
    assert factory.encoders == 1
    assert budget.active == 1

    await session.close()
    assert factory.encoders == 0
    assert budget.active == 0
    await session.close()  # idempotent
    assert factory.encoders == 0
    assert (await session.status()).state == "closed"
    with pytest.raises(VideoDenied):
        await session.offer("robot-a", video_offer("VP8"))
    source.feed(rgb_frame(clock.now))
    await asyncio.sleep(0.05)
    assert len(factory.peers[0].pushed) <= 1  # nothing is pushed after close


async def test_viewer_limit_and_single_peer_per_session() -> None:
    budget = ViewerBudget(1)
    first, factory, _, _ = make_session(budget=budget)
    second, _, _, _ = make_session(budget=budget, factory=factory)
    await first.offer("robot-a", video_offer("VP8"))
    with pytest.raises(VideoBusy):
        await first.offer("robot-a", video_offer("VP8"))
    with pytest.raises(VideoBusy):
        await second.offer("robot-a", video_offer("VP8"))
    assert factory.opened == 1
    await first.close()
    await second.offer("robot-a", video_offer("VP8"))
    assert budget.active == 1
    await second.close()
    assert budget.active == 0 and factory.encoders == 0


async def test_failed_negotiation_releases_everything() -> None:
    budget = ViewerBudget(1)
    session, factory, _, _ = make_session(budget=budget)

    async def broken(offer: SessionDescription) -> SessionDescription:
        raise RuntimeError("dtls failed")

    original_open = factory.open

    async def open_broken(request: PeerRequest) -> FakePeer:
        peer = await original_open(request)
        peer.accept_offer = broken  # type: ignore[method-assign]
        return peer

    factory.open = open_broken  # type: ignore[method-assign]
    with pytest.raises(RuntimeError):
        await session.offer("robot-a", video_offer("VP8"))
    assert factory.encoders == 0
    assert budget.active == 0
    await session.close()


# --------------------------------------------------------------------- codecs and bounds


def test_codec_follows_what_the_client_actually_offers() -> None:
    assert negotiate_codec(video_offer("H264").sdp) == "H264"
    assert negotiate_codec(video_offer("VP9", "VP8").sdp) == "VP8"
    assert negotiate_codec(video_offer("H264", "VP8").sdp) == "VP8"  # server preference
    assert negotiate_codec(video_offer("h264").sdp) == "H264"
    # An audio-only codec list never counts as video.
    with pytest.raises(NoCommonCodec):
        negotiate_codec(video_offer("AV1", "VP9").sdp)
    audio_only = "v=0\r\nm=audio 9 UDP/TLS/RTP/SAVPF 111\r\na=rtpmap:111 VP8/48000\r\n"
    with pytest.raises(NoCommonCodec):
        negotiate_codec(audio_only)


async def test_negotiated_codec_and_bitrate_cap_reach_the_peer() -> None:
    limits = VideoLimits(max_bitrate_bps=800_000, frame_poll_s=0.01, push_timeout_s=0.05)
    session, factory, _, _ = make_session(limits=limits)
    answer = await session.offer("robot-a", video_offer("H264"))
    request = factory.peers[0].request
    assert request.codec == "H264"
    assert request.max_bitrate_bps == 800_000
    assert answer.sdp == "answer-for-H264"
    await session.close()

    unsupported, factory2, _, _ = make_session()
    with pytest.raises(NoCommonCodec):
        await unsupported.offer("robot-a", video_offer("AV1"))
    assert factory2.opened == 0


async def test_offer_and_candidates_are_bounded() -> None:
    limits = VideoLimits(
        max_sdp_bytes=2048, max_candidates=2, frame_poll_s=0.01, push_timeout_s=0.05
    )
    session, factory, _, _ = make_session(limits=limits)
    huge = SessionDescription("offer", video_offer("VP8").sdp + "a=x:" + "y" * 4096 + "\r\n")
    with pytest.raises(ValueError):
        await session.offer("robot-a", huge)
    with pytest.raises(ValueError):
        await session.offer("robot-a", SessionDescription("answer", video_offer("VP8").sdp))
    assert factory.opened == 0

    with pytest.raises(VideoDenied):
        await session.add_ice_candidate("candidate:1 1 udp 1 10.0.0.1 9 typ host")  # no peer yet
    await session.offer("robot-a", video_offer("VP8"))
    await session.add_ice_candidate("candidate:1 1 udp 1 10.0.0.1 9 typ host")
    await session.add_ice_candidate("candidate:2 1 udp 1 10.0.0.2 9 typ relay")
    with pytest.raises(ValueError):
        await session.add_ice_candidate("candidate:3 1 udp 1 10.0.0.3 9 typ host")
    with pytest.raises(ValueError):
        await session.add_ice_candidate("x" * 600)
    assert len(factory.peers[0].candidates) == 2
    await session.close()


async def test_relay_only_ignores_non_relay_remote_candidates() -> None:
    session, factory, _, _ = make_session(relay_only=True)
    await session.offer("robot-a", video_offer("VP8"))
    await session.add_ice_candidate("candidate:1 1 udp 1 10.0.0.1 9 typ host")
    await session.add_ice_candidate("candidate:2 1 udp 1 203.0.113.5 9 typ relay")
    assert factory.peers[0].candidates == ["candidate:2 1 udp 1 203.0.113.5 9 typ relay"]
    await session.close()


# ------------------------------------------------------------------------ optional aiortc


def test_aiortc_is_not_imported_with_the_package() -> None:
    import subprocess
    import sys

    code = (
        "import sys, senior_control, senior_control.vision, senior_control.vision.stream, "
        "senior_control.vision.turn; "
        "assert not any(n == 'aiortc' or n.startswith('aiortc.') for n in sys.modules)"
    )
    result = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr


async def test_missing_aiortc_reports_unavailable(monkeypatch: pytest.MonkeyPatch) -> None:
    import sys

    from senior_control.vision.stream import AiortcPeerFactory

    monkeypatch.setitem(sys.modules, "aiortc", None)  # makes `import aiortc` raise ImportError
    with pytest.raises(VideoUnavailable) as raised:
        await AiortcPeerFactory().open(PeerRequest("VP8", 1_000_000, None, False))
    assert raised.value.reason == "aiortc_not_installed"


async def test_aiortc_loopback_delivers_a_real_frame() -> None:
    """Runs only where the `video` extra is installed; uses local sockets, no server."""
    aiortc = pytest.importorskip("aiortc")
    from senior_control.vision.stream import AiortcPeerFactory

    class Camera:
        async def next_frame(self, timeout: float) -> ImageFrame | None:
            await asyncio.sleep(0.05)
            return ImageFrame(time.time(), 64, 48, "rgb8", bytes([200, 10, 10]) * 64 * 48)

    session = VideoSession(
        robot_id="robot-a",
        viewer=Viewer("control", "robot-a"),
        source=Camera(),
        factory=AiortcPeerFactory(),
    )
    client = aiortc.RTCPeerConnection()
    client.addTransceiver("video", direction="recvonly")
    arrived: asyncio.Future[tuple[int, int]] = asyncio.get_running_loop().create_future()

    @client.on("track")  # type: ignore[untyped-decorator]
    def on_track(track: object) -> None:
        async def read() -> None:
            frame = await track.recv()  # type: ignore[attr-defined]
            arrived.set_result((frame.width, frame.height))

        asyncio.ensure_future(read())

    try:
        await client.setLocalDescription(await client.createOffer())
        offer = SessionDescription("offer", client.localDescription.sdp)
        answer = await session.offer("robot-a", offer)
        assert "H264" not in answer.sdp  # only the negotiated codec is answered
        await client.setRemoteDescription(aiortc.RTCSessionDescription(answer.sdp, "answer"))
        assert await asyncio.wait_for(arrived, 20) == (64, 48)
        assert (await session.status()).state == "live"
    finally:
        await session.close()
        await client.close()
