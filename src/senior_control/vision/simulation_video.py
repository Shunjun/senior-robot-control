"""//Compatible: original RGB1 symbols and latest-reader signature, no duplicate codec."""

from pathlib import Path

from vehicle_manager.media.latest_frame import LatestFrameFile as _LatestFrameFile

from senior_control.vision.rgb1 import _ENCODINGS as _ENCODINGS
from senior_control.vision.rgb1 import _HEADER as _HEADER
from senior_control.vision.rgb1 import _MAGIC as _MAGIC
from senior_control.vision.rgb1 import _MAX_AGE_NS as _MAX_AGE_NS
from senior_control.vision.rgb1 import _MAX_HEIGHT as _MAX_HEIGHT
from senior_control.vision.rgb1 import _MAX_WIDTH as _MAX_WIDTH
from senior_control.vision.rgb1 import _POLL_SECONDS as _POLL_SECONDS
from senior_control.vision.rgb1 import _read_frame as _read_frame
from senior_control.vision.rgb1 import _size as _size
from senior_control.vision.rgb1 import publish_image as publish_image
from vehicle_core.image_frame import ImageFrame as ImageFrame


class LatestFrameFile(_LatestFrameFile):
    """//Compatible: bind the sole original codec and polling interval per reader."""

    def __init__(self, path: Path) -> None:
        super().__init__(path, read_frame=_read_frame, poll_seconds=_POLL_SECONDS)
