"""Bounded ROS image layout decoding without ROS or optional image libraries."""

import array
import sys

from senior_control.vision.synchronizer import DepthFrame, ImageFrame

MAX_IMAGE_PIXELS = 2_073_600
MAX_IMAGE_BYTES = 16_777_216
MAX_IMAGE_DIMENSION = 4096


def _validate_layout(width: int, height: int, step: int, size: int, length: int) -> None:
    if (
        width <= 0
        or height <= 0
        or max(width, height) > MAX_IMAGE_DIMENSION
        or width * height > MAX_IMAGE_PIXELS
    ):
        raise ValueError("image size exceeds limit")
    if step < width * size:
        raise ValueError("image row is shorter than declared width")
    if step * height > MAX_IMAGE_BYTES:
        raise ValueError("image bytes exceed limit")
    if length != step * height:
        raise ValueError("image data is shorter than or differs from its declared size")


def decode_rgb(
    stamp: float, width: int, height: int, step: int, encoding: str, data: bytes
) -> ImageFrame:
    """Validate and pack padded RGB/BGR/mono rows before any model sees them."""
    size = {"rgb8": 3, "bgr8": 3, "mono8": 1}.get(encoding)
    if size is None:
        raise ValueError(f"unsupported color encoding: {encoding}")
    _validate_layout(width, height, step, size, len(data))
    packed = b"".join(data[row * step : row * step + width * size] for row in range(height))
    return ImageFrame(stamp, width, height, encoding, packed)



def decode_depth(
    stamp: float,
    width: int,
    height: int,
    step: int,
    encoding: str,
    data: bytes,
    big_endian: bool,
) -> DepthFrame:
    """Decode 16UC1 or 32FC1 rows, honoring row padding. Other encodings are rejected."""
    typecode, size = {"16UC1": ("H", 2), "32FC1": ("f", 4)}.get(encoding, ("", 0))
    if not typecode:
        raise ValueError(f"unsupported depth encoding: {encoding}")
    _validate_layout(width, height, step, size, len(data))
    values: list[float] = []
    for row in range(height):
        samples = array.array(typecode)
        samples.frombytes(data[row * step : row * step + width * size])
        if big_endian != (sys.byteorder == "big"):
            samples.byteswap()
        values.extend(float(value) for value in samples)
    return DepthFrame(stamp, width, height, tuple(values))

