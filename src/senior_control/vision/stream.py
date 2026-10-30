"""Vehicle-side video session for one authenticated viewer.

Signaling rides the sockets the cloud has already authenticated (`/ws/agent` and
`/ws/control`); this module opens no listener. A session is built with the identity of the
control connection that asked for video, and every call re-checks it against the robot.

Video never touches the drive path. Frames flow camera -> one latest-only slot -> peer. They
do not call `Supervisor.submit_drive` and never wait on it, so a congested peer drops frames
and leaves heartbeat and stop alone.

`aiortc` is an optional extra. It is imported only inside `AiortcPeerFactory`, never at
import time, and tests drive the session through a fake `PeerFactory`.
"""

import asyncio
import importlib
import logging
import secrets
import time
from collections.abc import Callable
from dataclasses import dataclass, replace
from typing import Any, Literal, Protocol

from senior_control.vision.synchronizer import ImageFrame
from senior_control.vision.turn import IceCredentials, TurnCredentialIssuer

logger = logging.getLogger(__name__)

SERVER_CODECS = ("VP8", "H264")
"""Codecs `aiortc` can encode, in the order the vehicle prefers them."""
MAX_CANDIDATE_CHARS = 512


class VideoError(Exception):
    """Base of every refusal the session reports to the signaling peer."""


class VideoDenied(VideoError):
    """The viewer is not the authenticated controller of this robot, or the session ended."""


class VideoUnavailable(VideoError):
    """There is nothing to stream. The reason is machine readable."""

    def __init__(self, reason: str) -> None:
        super().__init__(reason)
        self.reason = reason


class VideoBusy(VideoError):
    """The viewer limit is reached or this session already has a peer."""


class NoCommonCodec(VideoError):
    """The client offers no video codec the vehicle can encode."""


@dataclass(frozen=True)
class SessionDescription:
    type: Literal["offer", "answer"]
    sdp: str


@dataclass(frozen=True)
class PeerStats:
    connection_state: str
    bytes_sent: int


@dataclass(frozen=True)
class PeerRequest:
    codec: str
    max_bitrate_bps: int
    ice: IceCredentials | None
    relay_only: bool


@dataclass(frozen=True)
class VideoLimits:
    max_bitrate_bps: int = 1_500_000
    max_sdp_bytes: int = 16384
    max_candidates: int = 64
    max_frame_age_s: float = 2.0
    frame_poll_s: float = 0.1
    push_timeout_s: float = 0.5

    def __post_init__(self) -> None:
        if self.max_bitrate_bps <= 0 or self.max_sdp_bytes <= 0 or self.max_candidates <= 0:
            raise ValueError("video limits must be positive")
        if self.max_frame_age_s <= 0 or self.frame_poll_s <= 0 or self.push_timeout_s <= 0:
            raise ValueError("video timing limits must be positive")


class ViewerBudget:
    """Counts live viewers across sessions so one robot cannot be streamed without bound."""

    def __init__(self, maximum: int) -> None:
        if maximum < 1:
            raise ValueError("viewer budget must allow at least one viewer")
        self._maximum = maximum
        self.active = 0

    def acquire(self) -> bool:
        if self.active >= self._maximum:
            return False
        self.active += 1
        return True

    def release(self) -> None:
        if self.active > 0:
            self.active -= 1


class Viewer(Protocol):
    """The authenticated identity of a signaling socket (`Connection` in the cloud relay)."""

    @property
    def role(self) -> str: ...

    @property
    def robot_id(self) -> str: ...


class CameraSource(Protocol):
    async def next_frame(self, timeout: float) -> ImageFrame | None:
        """The next RGB frame, or None when none arrived within `timeout` seconds."""
        ...


class PeerConnection(Protocol):
    async def accept_offer(self, offer: SessionDescription) -> SessionDescription: ...

    async def add_remote_candidate(self, candidate: str) -> None: ...

    async def push(self, frame: ImageFrame) -> None: ...

    async def stats(self) -> PeerStats: ...

    async def close(self) -> None:
        """Release the encoder and every transport. Safe to call twice."""
        ...


class PeerFactory(Protocol):
    async def open(self, request: PeerRequest) -> PeerConnection: ...


@dataclass(frozen=True)
class VideoStatus:
    state: Literal[
        "idle",
        "unavailable",
        "connecting",
        "live",
        "stalled",
        "relay_failed",
        "ice_failed",
        "closed",
    ]
    reason: str | None
    frame_age_s: float | None
    frames_sent: int
    frames_dropped: int
    buffered_frames: int
    bitrate_bps: int | None = None
    codec: str | None = None


def negotiate_codec(sdp: str, supported: tuple[str, ...] = SERVER_CODECS) -> str:
    """Pick the vehicle's most preferred codec among those the client offers for video."""
    offered: set[str] = set()
    in_video = False
    for line in sdp.splitlines():
        if line.startswith("m="):
            in_video = line.startswith("m=video")
        elif in_video and line.startswith("a=rtpmap:"):
            _, _, mapping = line.partition(" ")
            offered.add(mapping.split("/", 1)[0].strip().upper())
    for codec in supported:
        if codec in offered:
            return codec
    raise NoCommonCodec("the client offers no video codec the vehicle can encode")


class VideoSession:
    def __init__(
        self,
        *,
        robot_id: str,
        viewer: Viewer,
        source: CameraSource | None,
        factory: PeerFactory,
        limits: VideoLimits | None = None,
        budget: ViewerBudget | None = None,
        turn: TurnCredentialIssuer | None = None,
        now: Callable[[], float] = time.time,
        relay_only: bool = False,
    ) -> None:
        self.session_id = secrets.token_urlsafe(12)
        self._robot_id = robot_id
        self._viewer = viewer
        self._source = source
        self._factory = factory
        self._limits = limits or VideoLimits()
        self._budget = budget
        self._turn = turn
        self._now = now
        self._relay_only = relay_only
        self._peer: PeerConnection | None = None
        self._codec: str | None = None
        self._opening = False
        self._holding = False
        self._closed = False
        self._tasks: list[asyncio.Task[None]] = []
        self._latest: ImageFrame | None = None
        self._wake = asyncio.Event()
        self._last_stamp: float | None = None
        self._last_push_at: float | None = None
        self._sent = 0
        self._dropped = 0
        self._candidates = 0
        self._bitrate_mark: tuple[float, int] | None = None

    def _authorize(self, robot_id: str) -> None:
        if self._closed:
            raise VideoDenied("video session is closed")
        if self._viewer.role != "control":
            raise VideoDenied("video is served to control connections only")
        if robot_id != self._robot_id or self._viewer.robot_id != robot_id:
            raise VideoDenied("robot is not the one selected for this connection")

    def turn_credentials(self, now: int) -> IceCredentials:
        self._authorize(self._robot_id)
        if self._turn is None:
            raise VideoUnavailable("turn_not_configured")
        return self._issue(now)

    def _issue(self, now: int) -> IceCredentials:
        assert self._turn is not None
        issued = self._turn.issue_turn_credentials(self.session_id, now)
        return replace(issued, relay_only=self._relay_only)

    async def offer(self, robot_id: str, offer: SessionDescription) -> SessionDescription:
        self._authorize(robot_id)
        if self._source is None:
            raise VideoUnavailable("no_camera")
        if offer.type != "offer":
            raise ValueError("expected an offer")
        if len(offer.sdp.encode()) > self._limits.max_sdp_bytes:
            raise ValueError("session description too large")
        if self._peer is not None or self._opening:
            raise VideoBusy("this session already has a peer")
        codec = negotiate_codec(offer.sdp)
        if self._budget is not None:
            if not self._budget.acquire():
                raise VideoBusy("viewer limit reached")
            self._holding = True
        self._opening = True
        try:
            ice = self._issue(int(self._now())) if self._turn is not None else None
            request = PeerRequest(codec, self._limits.max_bitrate_bps, ice, self._relay_only)
            peer = await self._factory.open(request)
            self._peer = peer
            answer = await peer.accept_offer(offer)
            if self._closed:
                raise VideoDenied("video session is closed")
        except BaseException:
            await self._release_peer()
            raise
        finally:
            self._opening = False
        self._codec = codec
        self._tasks = [
            asyncio.ensure_future(self._read_loop(self._source)),
            asyncio.ensure_future(self._send_loop(peer)),
        ]
        return answer

    async def add_ice_candidate(self, candidate: str) -> None:
        self._authorize(self._robot_id)
        peer = self._peer
        if peer is None:
            raise VideoDenied("no negotiated peer")
        if len(candidate) > MAX_CANDIDATE_CHARS or not candidate.startswith("candidate:"):
            raise ValueError("invalid ICE candidate")
        if self._candidates >= self._limits.max_candidates:
            raise ValueError("too many ICE candidates")
        self._candidates += 1
        if self._relay_only and " typ relay" not in candidate:
            return
        await peer.add_remote_candidate(candidate)

    def _status(
        self,
        state: Literal[
            "idle",
            "unavailable",
            "connecting",
            "live",
            "stalled",
            "relay_failed",
            "ice_failed",
            "closed",
        ],
        reason: str | None = None,
        age: float | None = None,
        bitrate: int | None = None,
    ) -> VideoStatus:
        return VideoStatus(
            state,
            reason,
            age,
            frames_sent=self._sent,
            frames_dropped=self._dropped,
            buffered_frames=0 if self._latest is None else 1,
            bitrate_bps=bitrate,
            codec=self._codec,
        )

    async def status(self) -> VideoStatus:
        if self._closed:
            return self._status("closed")
        if self._source is None:
            return self._status("unavailable", "no_camera")
        peer = self._peer
        if peer is None:
            return self._status("idle")
        age = None
        if self._last_stamp is not None:
            age = max(0.0, self._now() - self._last_stamp)
        try:
            stats = await peer.stats()
        except Exception:
            logger.warning("peer stats failed", exc_info=True)
            stats = PeerStats("failed", 0)
        bitrate = self._bitrate(stats.bytes_sent)
        if stats.connection_state == "failed":
            if self._relay_only:
                return self._status("relay_failed", "ice_failed", age, bitrate)
            return self._status("ice_failed", "ice_failed", age, bitrate)
        if stats.connection_state != "connected":
            return self._status("connecting", None, age, bitrate)
        if self._flowing(age):
            return self._status("live", None, age, bitrate)
        return self._status("stalled", "no_fresh_frames", age, bitrate)

    def _flowing(self, age: float | None) -> bool:
        limit = self._limits.max_frame_age_s
        if age is None or age > limit or self._last_push_at is None:
            return False
        return self._now() - self._last_push_at <= limit

    def _bitrate(self, total: int) -> int | None:
        now = self._now()
        mark, self._bitrate_mark = self._bitrate_mark, (now, total)
        if mark is None or now <= mark[0] or total < mark[1]:
            return None
        return int((total - mark[1]) * 8 / (now - mark[0]))

    async def close(self) -> None:
        self._closed = True
        await self._release_peer()

    async def _release_peer(self) -> None:
        tasks, self._tasks = self._tasks, []
        for task in tasks:
            task.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)
        self._latest = None
        peer, self._peer = self._peer, None
        holding, self._holding = self._holding, False
        try:
            if peer is not None:
                await peer.close()
        finally:
            if holding and self._budget is not None:
                self._budget.release()

    async def _read_loop(self, source: CameraSource) -> None:
        limits = self._limits
        while True:
            frame = await source.next_frame(limits.frame_poll_s)
            if frame is None:
                continue
            self._last_stamp = frame.stamp
            if self._now() - frame.stamp > limits.max_frame_age_s:
                self._dropped += 1
                continue
            if self._latest is not None:
                self._dropped += 1
            self._latest = frame
            self._wake.set()

    async def _send_loop(self, peer: PeerConnection) -> None:
        while True:
            await self._wake.wait()
            self._wake.clear()
            frame, self._latest = self._latest, None
            if frame is None:
                continue
            try:
                await asyncio.wait_for(peer.push(frame), self._limits.push_timeout_s)
            except asyncio.TimeoutError:
                self._dropped += 1
            except Exception:
                logger.warning("video push failed", exc_info=True)
                self._dropped += 1
            else:
                self._sent += 1
                self._last_push_at = self._now()


# ------------------------------------------------------------------------------- aiortc


def _load_aiortc() -> Any:
    try:
        return importlib.import_module("aiortc")
    except ImportError:
        raise VideoUnavailable("aiortc_not_installed") from None


def _to_av_frame(av: Any, image: ImageFrame) -> Any:
    layout = {"rgb8": ("rgb24", 3), "bgr8": ("bgr24", 3), "mono8": ("gray", 1)}.get(image.encoding)
    if layout is None:
        raise ValueError(f"unsupported image encoding {image.encoding!r}")
    pixel_format, channels = layout
    frame = av.VideoFrame(image.width, image.height, pixel_format)
    plane = frame.planes[0]
    row = image.width * channels
    if plane.line_size == row:
        plane.update(image.data)
    else:
        plane.update(
            b"".join(
                image.data[y * row : (y + 1) * row].ljust(plane.line_size, b"\0")
                for y in range(image.height)
            )
        )
    return frame


def _only_relay_candidates(sdp: str) -> str:
    kept = [
        line
        for line in sdp.split("\r\n")
        if not line.startswith("a=candidate:") or " typ relay" in line
    ]
    return "\r\n".join(kept)


def _lower_bitrate_cap(cap_bps: int) -> None:
    """aiortc reads these module constants per encoder, so the cap is process wide."""
    for name in ("aiortc.codecs.vpx", "aiortc.codecs.h264"):
        module = importlib.import_module(name)
        maximum = min(int(getattr(module, "MAX_BITRATE")), cap_bps)  # noqa: B009
        setattr(module, "MAX_BITRATE", maximum)  # noqa: B010
        default = min(int(getattr(module, "DEFAULT_BITRATE")), maximum)  # noqa: B009
        setattr(module, "DEFAULT_BITRATE", default)  # noqa: B010


def _make_track(aiortc: Any) -> Any:
    av = importlib.import_module("av")
    ended = importlib.import_module("aiortc.mediastreams").MediaStreamError

    class LatestFrameTrack(aiortc.VideoStreamTrack):  # type: ignore[misc]
        """Hands the encoder the newest frame and never queues a backlog."""

        def __init__(self) -> None:
            super().__init__()
            self._image: ImageFrame | None = None
            self._arrived = asyncio.Event()

        def push(self, image: ImageFrame) -> None:
            self._image = image
            self._arrived.set()

        def stop(self) -> None:
            super().stop()
            self._arrived.set()

        async def recv(self) -> Any:
            while True:
                await self._arrived.wait()
                if self.readyState == "ended":
                    raise ended()
                image, self._image = self._image, None
                self._arrived.clear()
                if image is None:
                    continue
                pts, time_base = await self.next_timestamp()
                frame = _to_av_frame(av, image)
                frame.pts = pts
                frame.time_base = time_base
                return frame

    return LatestFrameTrack()


class _AiortcPeer:
    def __init__(self, aiortc: Any, pc: Any, track: Any, request: PeerRequest) -> None:
        self._aiortc = aiortc
        self._pc = pc
        self._track = track
        self._request = request

    async def accept_offer(self, offer: SessionDescription) -> SessionDescription:
        aiortc, pc = self._aiortc, self._pc
        # aiortc fixes the codec list inside setRemoteDescription, so the transceiver and its
        # preferences must exist before the offer is applied.
        wanted = {f"VIDEO/{self._request.codec}", "VIDEO/RTX"}
        codecs = [
            codec
            for codec in aiortc.RTCRtpSender.getCapabilities("video").codecs
            if codec.mimeType.upper() in wanted
        ]
        transceiver = pc.addTransceiver(self._track, direction="sendonly")
        transceiver.setCodecPreferences(codecs)
        await pc.setRemoteDescription(aiortc.RTCSessionDescription(sdp=offer.sdp, type="offer"))
        await pc.setLocalDescription(await pc.createAnswer())
        sdp = pc.localDescription.sdp
        if self._request.relay_only:
            sdp = _only_relay_candidates(sdp)
        return SessionDescription("answer", sdp)

    async def add_remote_candidate(self, candidate: str) -> None:
        # aiortc has no candidate parser on the public surface other than the SDP one.
        from_sdp = importlib.import_module("aiortc.sdp").candidate_from_sdp
        try:
            parsed = from_sdp(candidate.removeprefix("candidate:"))
        except (AssertionError, ValueError, IndexError):
            raise ValueError("invalid ICE candidate") from None
        # Browsers bundle every m-line onto the first one, which owns the transport.
        parsed.sdpMLineIndex = 0
        await self._pc.addIceCandidate(parsed)

    async def push(self, frame: ImageFrame) -> None:
        self._track.push(frame)

    async def stats(self) -> PeerStats:
        report = await self._pc.getStats()
        sent = sum(int(item.bytesSent) for item in report.values() if item.type == "outbound-rtp")
        state = str(self._pc.connectionState)
        if str(self._pc.iceConnectionState) == "failed":
            state = "failed"
        return PeerStats(state, sent)

    async def close(self) -> None:
        self._track.stop()
        await self._pc.close()


class AiortcPeerFactory:
    """Builds real peers. Needs the `video` extra; importing this module does not."""

    async def open(self, request: PeerRequest) -> PeerConnection:
        aiortc = _load_aiortc()
        _lower_bitrate_cap(request.max_bitrate_bps)
        servers = []
        if request.ice is not None:
            servers.append(
                aiortc.RTCIceServer(
                    urls=list(request.ice.urls),
                    username=request.ice.username,
                    credential=request.ice.credential,
                )
            )
        pc = aiortc.RTCPeerConnection(aiortc.RTCConfiguration(iceServers=servers))
        return _AiortcPeer(aiortc, pc, _make_track(aiortc), request)
