"""//Compatible: original RGB1 symbols and latest-reader signature, no duplicate codec."""

from pathlib import Path as Path

from vehicle_manager.media.frame_bindings import LatestFrameFile as LatestFrameFile

from vehicle_core.image_frame import ImageFrame as ImageFrame
from vehicle_core.rgb1 import _ENCODINGS as _ENCODINGS
from vehicle_core.rgb1 import _HEADER as _HEADER
from vehicle_core.rgb1 import _MAGIC as _MAGIC
from vehicle_core.rgb1 import _MAX_AGE_NS as _MAX_AGE_NS
from vehicle_core.rgb1 import _MAX_HEIGHT as _MAX_HEIGHT
from vehicle_core.rgb1 import _MAX_WIDTH as _MAX_WIDTH
from vehicle_core.rgb1 import _POLL_SECONDS as _POLL_SECONDS
from vehicle_core.rgb1 import _read_frame as _read_frame
from vehicle_core.rgb1 import _size as _size
from vehicle_core.rgb1 import publish_image as publish_image
