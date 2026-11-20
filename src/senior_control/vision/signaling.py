"""//Compatible: original signaling signature and diagnostic queue owner."""

from collections.abc import Awaitable, Callable

from senior_contracts.wire import VideoEnvelope
from vehicle_manager.media.signaling import VideoSignaling as _VideoSignaling
from vehicle_manager.media.signaling import logger as logger
from vehicle_manager.media.stream import VideoSession

from senior_control.runtime.diagnostics import DiagnosticAsyncQueue


class VideoSignaling(_VideoSignaling):
    """//Compatible: sole inherited management loop, original per-instance queue binding."""

    def __init__(
        self,
        robot_id: str,
        epoch: int,
        *,
        send: Callable[[VideoEnvelope], Awaitable[None]],
        make_session: Callable[[], VideoSession | None],
    ) -> None:
        super().__init__(
            robot_id,
            epoch,
            send=send,
            make_session=make_session,
            queue_factory=DiagnosticAsyncQueue,
        )
