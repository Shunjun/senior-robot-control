"""Bounded vehicle-side handling of video frames from the authenticated cloud link."""

import asyncio
import logging
from collections.abc import Awaitable, Callable

from senior_control.protocol import (
    SdpPayload,
    VideoCandidateEnvelope,
    VideoCloseEnvelope,
    VideoEnvelope,
    VideoSdpEnvelope,
)
from senior_control.vision.stream import SessionDescription, VideoError, VideoLimits, VideoSession

logger = logging.getLogger(__name__)


class VideoSignaling:
    """Own one viewer and its candidate queue for the life of an agent WebSocket."""

    def __init__(
        self,
        robot_id: str,
        epoch: int,
        *,
        send: Callable[[VideoEnvelope], Awaitable[None]],
        make_session: Callable[[], VideoSession | None],
    ) -> None:
        self._robot_id = robot_id
        self._epoch = epoch
        self._send = send
        self._make_session = make_session
        self._viewer: str | None = None
        self._task: asyncio.Task[None] | None = None
        self._refusal: asyncio.Task[None] | None = None
        self._candidates: asyncio.Queue[str] = asyncio.Queue(VideoLimits().max_candidates)
        self._closed = False

    def submit(self, frame: VideoEnvelope) -> None:
        """Schedule negotiation separately from the command receive loop."""
        if (
            self._closed
            or frame.robot_id != self._robot_id
            or frame.connection_epoch != self._epoch
        ):
            return
        if isinstance(frame, VideoSdpEnvelope):
            if frame.type != "video.offer":
                return
            if self._task is not None and not self._task.done():
                if frame.session_id != self._viewer and (
                    self._refusal is None or self._refusal.done()
                ):
                    metadata = frame.model_dump(exclude={"type", "payload"})
                    self._refusal = asyncio.create_task(self._notify_closed(metadata))
                return
            self._viewer = frame.session_id
            self._candidates = asyncio.Queue(VideoLimits().max_candidates)
            self._task = asyncio.create_task(self._serve(frame))
        elif frame.session_id == self._viewer and self._task is not None:
            if isinstance(frame, VideoCloseEnvelope):
                self._viewer = None
                self._task.cancel()
            elif isinstance(frame, VideoCandidateEnvelope) and not self._candidates.full():
                self._candidates.put_nowait(frame.payload.candidate)

    def retire(self) -> None:
        """Stop accepting frames and cancel negotiation or media immediately."""
        self._closed = True
        if self._viewer is not None and self._task is not None:
            self._viewer = None
            self._task.cancel()
        if self._refusal is not None:
            self._refusal.cancel()

    async def close(self) -> None:
        self.retire()
        tasks = {task for task in (self._task, self._refusal) if task is not None}
        if tasks:
            # A caller's cleanup deadline must not send a second cancellation into peer.close().
            await asyncio.wait(tasks)

    async def _serve(self, offer: VideoSdpEnvelope) -> None:
        session = None
        metadata = offer.model_dump(exclude={"type", "payload"})
        try:
            session = self._make_session()
            if session is None:
                await self._notify_closed(metadata)
                return
            answer = await session.offer(
                self._robot_id, SessionDescription("offer", offer.payload.sdp)
            )
            await self._send(
                VideoSdpEnvelope(
                    **metadata, type="video.answer", payload=SdpPayload(sdp=answer.sdp)
                )
            )
            while True:
                await session.add_ice_candidate(await self._candidates.get())
        except (VideoError, ValueError) as error:
            logger.info("video session refused: %s", error)
            await self._notify_closed(metadata)
        except Exception:
            logger.warning("video signaling failed", exc_info=True)
            await self._notify_closed(metadata)
        finally:
            self._viewer = None
            if session is not None:
                try:
                    await session.close()
                except Exception:
                    logger.warning("video peer cleanup failed", exc_info=True)

    async def _notify_closed(self, metadata: dict[str, object]) -> None:
        try:
            await self._send(
                VideoCloseEnvelope.model_validate(
                    {**metadata, "type": "video.close", "payload": {}}
                )
            )
        except Exception:
            logger.warning("video close could not be sent", exc_info=True)
