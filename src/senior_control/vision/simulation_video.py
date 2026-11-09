"""Private latest-frame slot shared by the isolated simulator and its Agent."""

import asyncio
import os
import stat
import struct
import tempfile
import time
from pathlib import Path

from senior_control.vision.synchronizer import ImageFrame

_HEADER = struct.Struct("!4sBHHQ")
_MAGIC = b"RGB1"
_ENCODINGS = ("rgb8", "bgr8")
_MAX_WIDTH = 640
_MAX_HEIGHT = 480
_MAX_AGE_NS = 1_000_000_000
_POLL_SECONDS = 0.025


def _size(width: int, height: int) -> int:
    return 3 * width * height if 0 < width <= _MAX_WIDTH and 0 < height <= _MAX_HEIGHT else 0


def publish_image(
    path: Path,
    width: int,
    height: int,
    step: int,
    encoding: str,
    data: bytes | memoryview,
    *,
    captured_ns: int | None = None,
) -> bool:
    size = _size(width, height)
    row = 3 * width
    if (
        size == 0
        or encoding not in _ENCODINGS
        or not row <= step <= row + 128
        or len(data) != step * height
    ):
        return False
    captured = time.monotonic_ns() if captured_ns is None else captured_ns
    if not 0 < captured < 2**64:
        return False
    header = _HEADER.pack(_MAGIC, _ENCODINGS.index(encoding), width, height, captured)
    fd, temporary = tempfile.mkstemp(prefix=".frame-", dir=path.parent)
    try:
        with os.fdopen(fd, "wb") as stream:
            stream.write(header)
            if row == step:
                stream.write(data)
            else:
                stream.write(
                    b"".join(data[start : start + row] for start in range(0, len(data), step))
                )
        os.replace(temporary, path)
    finally:
        Path(temporary).unlink(missing_ok=True)
    return True


def _read_frame(path: Path, last_capture: int) -> tuple[int, ImageFrame] | None:
    try:
        # A malformed or replaced slot must not block the Agent, even on a FIFO.
        fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
        with os.fdopen(fd, "rb") as stream:
            info = os.fstat(stream.fileno())
            if not stat.S_ISREG(info.st_mode) or info.st_uid != os.getuid() or info.st_mode & 0o077:
                return None
            raw = stream.read(_HEADER.size)
            if len(raw) != _HEADER.size:
                return None
            magic, encoding, width, height, captured = _HEADER.unpack(raw)
            size = _size(width, height)
            if (
                magic != _MAGIC
                or encoding >= len(_ENCODINGS)
                or size == 0
                or info.st_size != _HEADER.size + size
                or captured <= last_capture
                or not 0 <= time.monotonic_ns() - captured <= _MAX_AGE_NS
            ):
                return None
            data = stream.read(size)
    except OSError:
        return None
    age = time.monotonic_ns() - captured
    if len(data) != size or not 0 <= age <= _MAX_AGE_NS:
        return None
    stamp = time.time() - age / 1e9
    return captured, ImageFrame(stamp, width, height, _ENCODINGS[encoding], data)


class LatestFrameFile:
    def __init__(self, path: Path) -> None:
        self._path = path
        self._last_capture = 0

    async def next_frame(self, timeout: float) -> ImageFrame | None:
        loop = asyncio.get_running_loop()
        deadline = loop.time() + timeout
        while True:
            result = await asyncio.to_thread(_read_frame, self._path, self._last_capture)
            if result is not None:
                self._last_capture, frame = result
                return frame
            remaining = deadline - loop.time()
            if remaining <= 0:
                return None
            await asyncio.sleep(min(_POLL_SECONDS, remaining))
