"""Management latest-frame consumption over the original shared RGB1 reader."""

import asyncio
from collections.abc import Callable
from pathlib import Path

from vehicle_manager.media import ImageReading


class LatestFrameFile:
    def __init__(
        self,
        path: Path,
        *,
        read_frame: Callable[[Path, int], tuple[int, ImageReading] | None],
        poll_seconds: float,
    ) -> None:
        self._read_frame = read_frame
        self._poll_seconds = poll_seconds
        self._path = path
        self._last_capture = 0

    async def next_frame(self, timeout: float) -> ImageReading | None:
        loop = asyncio.get_running_loop()
        deadline = loop.time() + timeout
        while True:
            result = await asyncio.to_thread(self._read_frame, self._path, self._last_capture)
            if result is not None:
                self._last_capture, frame = result
                return frame
            remaining = deadline - loop.time()
            if remaining <= 0:
                return None
            await asyncio.sleep(min(self._poll_seconds, remaining))
