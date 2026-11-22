"""Concrete latest-frame reader bound to the sole shared RGB1 codec."""

from pathlib import Path

from vehicle_core.rgb1 import _POLL_SECONDS, _read_frame
from vehicle_manager.media.latest_frame import LatestFrameFile as _LatestFrameFile


class LatestFrameFile(_LatestFrameFile):
    """//Compatible: bind the sole original codec and polling interval per reader."""

    def __init__(self, path: Path) -> None:
        super().__init__(path, read_frame=_read_frame, poll_seconds=_POLL_SECONDS)
